"""v5 2·3단계 학습 루프 (26_09_15_23) — 집합 기반 상위 정책.

상위만 학습한다 (하위는 직진 제어, v4에서 학습 하위가 해롭다고 확인). 상위 전이는
옵션 단위(semi-MDP)로 저장하고, 홀드아웃은 확률 샘플링으로 잰다. 드론 수·목적지 수·
제어 센터 위치를 에피소드마다 바꿀 수 있어(--random-config), 학습에서 본 적 없는
구성으로 평가하는 것이 목표다.
"""

import argparse
import csv
import os
import sys
import time

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.tensorboard import SummaryWriter

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv, N_MAX
from model.hier_net_v5_26_09_15_23 import SetManagerActor, SetManagerTwinQ
from pipeline.common_v5_26_09_15_22 import action_mask, chain_manager, straight_worker
from util.instance_generator_v5_26_09_15_22 import sample_config, sample_instance

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BATCH, WARMUP, TAU, CKPT_EVERY = 256, 400, 0.005, 10

P = argparse.ArgumentParser()
P.add_argument("--tag", default="v5_26_09_15_23")
P.add_argument("--comm-range", type=float, default=300.0)
P.add_argument("--hl-every", type=int, default=20)
P.add_argument("--max-episodes", type=int, default=1500)
P.add_argument("--max-steps", type=int, default=1000)
P.add_argument("--no-progress-limit", type=int, default=500)
P.add_argument("--eval-every", type=int, default=50)
P.add_argument("--eval-n", type=int, default=12)
P.add_argument("--eval-seed0", type=int, default=201)
P.add_argument("--patience", type=int, default=15)
P.add_argument("--seed", type=int, default=1)
P.add_argument("--gamma", type=float, default=0.997)
P.add_argument("--update-every", type=int, default=4, help="몇 env 스텝마다 상위를 갱신할지")
P.add_argument("--num-drones", type=int, default=4)
P.add_argument("--num-dests", type=int, default=50)
P.add_argument("--random-config", action="store_true",
               help="드론 3-8대, 목적지 20-80곳, 제어 센터 위치를 에피소드마다 무작위화 (3단계)")
P.add_argument("--drones-range", type=int, nargs=2, default=[3, 8])
P.add_argument("--drone-weights", type=float, nargs="*", default=None,
               help="드론 수별 표본 비중 (범위 길이와 같게). 예: --drones-range 3 4 --drone-weights 3 1 이면 3대를 75%로 본다")
P.add_argument("--dests-range", type=int, nargs=2, default=[20, 80])
P.add_argument("--fixed-instance", action="store_true", help="고정 지도 한 장으로 학습 (절제용)")
P.add_argument("--n-far", type=int, default=0, help="후보 중 먼 곳 수. 2로 두면 v4 상위가 -2.1 (기본 0)")
P.add_argument("--commit-max", type=int, default=150)
P.add_argument("--commit", action="store_true",
               help="목표 유지 마스크. 켜면 v4 상위가 -6.5이고 단일행동 행이 늘어 alpha가 폭주한다 (기본 off)")
P.add_argument("--no-relay", action="store_true")
P.add_argument("--relay-hold", type=float, default=0.02)
P.add_argument("--resume", action="store_true")
P.add_argument("--complete-bonus", type=float, default=0.0,
               help="전량 완주 시 (1 - 스텝/상한)에 곱해 주는 팀 보상. makespan 신호 강화용")
P.add_argument("--init-from", default="", help="다른 실행의 latest.pth에서 actor·critic·alpha를 이어받아 시작 (에피소드·최고 기록은 초기화)")
P.add_argument("--residual-penalty", type=float, default=0.0,
               help="매 스텝 팀 보상 -= 값 x 드론수 x 미배송비율 (makespan 대리 목적)")
P.add_argument("--rule-obs", action="store_true", help="기하 규칙의 제안 행동(원핫)을 상위 관측에 넣는다")
P.add_argument("--blocked-penalty", type=float, default=0.0, help="투영이 잘라낸 변위 비율 합에 곱해 팀 보상에서 뺀다")
P.add_argument("--coverage-shaping", type=float, default=0.0, help="도달권(연결 드론 반경 안 미배송지 비율) 잠재 shaping 계수")
P.add_argument("--autoregressive", action="store_true", help="상위 결정을 먼 드론부터 순차로 (앞 드론의 선택을 보고 결정)")
P.add_argument("--stall-switch-bonus", type=float, default=0.0,
               help="정체 중(이동 실현율 EMA<0.3, 목표 20스텝 이상) 드론이 목표를 바꾸면 팀 보상 c. 무작위성 없이 탈출을 배우게 한다")
P.add_argument("--ent-frac-final", type=float, default=-1.0,
               help="0 이상이면 ent_frac에서 이 값까지 선형 감쇠 (탐색 -> 활용)")
P.add_argument("--ent-anneal-episodes", type=int, default=0,
               help="감쇠를 몇 에피소드에 걸쳐 끝낼지. 0이면 max_episodes 전체. 사이클 8에서 전체에 걸쳐 "
                    "감쇠했더니 최고 체크포인트가 목표 0.5 구간(ep300-400)에서 나오고 좋은 구간(0.25-0.3)에 "
                    "닿기 전에 조기 종료됐다 (26-09-22)")
P.add_argument("--ent-frac", type=float, default=0.6,
               help="엔트로피 목표 = ent_frac·log(유효 행동 수). 0.6이면 alpha가 2~3에 머물러 정책이 흐트러진다 (26-09-19 온도 프로브)")
P.add_argument("--rule-reg", type=float, default=0.0,
               help="actor 손실에 lambda x (규칙 행동의 음의 로그확률)을 더한다. 정책이 규칙 근처에 머물되 Q가 강하게 반대할 때만 벗어난다")
P.add_argument("--holdout-steps", type=int, default=1000, help="홀드아웃 상한. makespan 사이클은 3000")
P.add_argument("--speed-weight", type=float, default=10.0,
               help="makespan 선택 점수의 속도 항 가중치. 기본 10이면 완주 1%p(1점)가 속도 300스텝과 "
                    "맞먹어 빠른 체크포인트가 완주 1~2곳 때문에 버려진다 (사이클 10에서 750 에피소드 낭비). "
                    "40으로 두면 완주 1%p가 75스텝에 대응한다")
P.add_argument("--select", choices=["delivered", "makespan"], default="delivered",
               help="최고 체크포인트 판정 기준. makespan이면 배송 + 5*(1 - 평균스텝/상한)")
A = P.parse_args()
GAMMA = A.gamma
K = 7
MASK_KW = dict(no_relay=A.no_relay, commit=A.commit)


def make_env(n, m):
	"""주어진 구성의 환경을 만든다."""
	return DisasterRelayDroneEnv(map_path="", comm_range=A.comm_range, cluster_penalty=False,
	                             max_steps=A.max_steps, deadlock_limit=10 ** 9,
	                             no_progress_limit=A.no_progress_limit, arrive_once=False,
	                             relay_hold=A.relay_hold, num_drones=n, num_dests=m,
	                             n_far=A.n_far, commit_max=A.commit_max,
	                             complete_bonus=A.complete_bonus, residual_penalty=A.residual_penalty,
	                             rule_obs=A.rule_obs, blocked_penalty=A.blocked_penalty,
	                             coverage_shaping=A.coverage_shaping, stall_switch_bonus=A.stall_switch_bonus)


def pad_obs(o, n):
	"""집합 관측을 N_MAX 드론으로 패딩하고 drone_mask를 붙인다."""
	C = o["cand"].shape[1]
	out = dict(self=np.zeros((N_MAX, 10), np.float32), cand=np.zeros((N_MAX, C, 6), np.float32),
	           cand_mask=np.zeros((N_MAX, C), bool), peer=np.zeros((N_MAX, N_MAX, 9), np.float32),
	           peer_mask=np.zeros((N_MAX, N_MAX), bool), relay=np.zeros((N_MAX, 4), np.float32),
	           drone_mask=np.zeros(N_MAX, bool))
	if "advice" in o:
		out["advice"] = np.zeros((N_MAX, K), np.float32)
		out["advice"][:n] = o["advice"]
	out["self"][:n], out["cand"][:n], out["cand_mask"][:n] = o["self"], o["cand"], o["cand_mask"]
	out["peer"][:n, :n], out["peer_mask"][:n, :n], out["relay"][:n] = o["peer"], o["peer_mask"], o["relay"]
	out["drone_mask"][:n] = True
	return out


def to_t(o, dev, batch=True):
	"""관측 dict를 텐서로. batch=True면 앞에 배치 축을 붙인다."""
	t = {k: torch.as_tensor(v, device=dev) for k, v in o.items()}
	return {k: v.unsqueeze(0) for k, v in t.items()} if batch else t


class OptionBuffer:
	"""옵션 단위 상위 전이 버퍼. 드론 수는 N_MAX로 패딩해 담는다."""

	def __init__(self, cap=60000, C=5):
		self.cap, self.ptr, self.size = cap, 0, 0
		z = lambda *s: np.zeros(s, np.float32)
		b = lambda *s: np.zeros(s, bool)
		self.o = dict(self=z(cap, N_MAX, 10), cand=z(cap, N_MAX, C, 6), cand_mask=b(cap, N_MAX, C),
		              peer=z(cap, N_MAX, N_MAX, 9), peer_mask=b(cap, N_MAX, N_MAX),
		              relay=z(cap, N_MAX, 4), drone_mask=b(cap, N_MAX))
		if A.rule_obs:
			self.o["advice"] = z(cap, N_MAX, K)
		self.o2 = {k: np.zeros_like(v) for k, v in self.o.items()}
		self.ma = np.zeros((cap, N_MAX), np.int64)
		self.ra = np.zeros((cap, N_MAX), np.int64)     # 옵션 시작 시 규칙이 제안한 행동
		self.mask, self.mask2 = b(cap, N_MAX, K), b(cap, N_MAX, K)
		self.r, self.disc, self.d = z(cap, 1), z(cap, 1), z(cap, 1)

	def push(self, o, ma, mask, r, o2, mask2, disc, d, ra=None):
		i = self.ptr
		for k in self.o:
			self.o[k][i], self.o2[k][i] = o[k], o2[k]
		self.ma[i, :len(ma)] = ma
		if ra is not None:
			self.ra[i, :len(ra)] = ra
		self.mask[i], self.mask2[i] = mask, mask2
		self.r[i], self.disc[i], self.d[i] = r, disc, d
		self.ptr = (self.ptr + 1) % self.cap
		self.size = min(self.size + 1, self.cap)

	def sample(self, bs, dev):
		idx = np.random.randint(0, self.size, bs)
		t = lambda x: torch.as_tensor(x[idx], device=dev)
		return dict(o={k: t(v) for k, v in self.o.items()}, o2={k: t(v) for k, v in self.o2.items()},
		            ma=t(self.ma), ra=t(self.ra), mask=t(self.mask), mask2=t(self.mask2), r=t(self.r), disc=t(self.disc), d=t(self.d))

	def __len__(self):
		return self.size


def act(env, actor, dev):
	"""상위 정책으로 드론별 행동을 샘플링한다. (행동, 패딩 관측, 마스크) 반환."""
	o = pad_obs(env.manager_set_obs(), env.num_drones)
	m = action_mask(env, dev, **MASK_KW)
	mp = torch.zeros(N_MAX, K, dtype=torch.bool, device=dev)
	mp[:env.num_drones] = m
	mp[env.num_drones:, K - 2] = True     # 패딩 드론: 손실에서 제외되지만 분포는 정의돼야 한다
	with torch.no_grad():
		a, _p, _ = actor(to_t(o, dev), mp.unsqueeze(0))
	return a[0, :env.num_drones].cpu().numpy(), o, mp.cpu().numpy()


def act_rule(env):
	"""규칙 행동 (정규화용). 마스크에 걸리면 정규화 표적에서 제외되도록 -1."""
	ra = chain_manager(env).astype(np.int64)
	return ra


def rollout(env, actor, dev, seed, cc_mode=None):
	"""홀드아웃 인스턴스 하나를 굴린다 (확률 샘플링)."""
	env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=seed, num_drones=env.num_drones,
	                                            comm_range=env.comm_range, cc_pos=cc_mode)
	env.reset()
	a, _o, _m = act(env, actor, dev)
	env.set_goals(a)
	done, t = False, 0
	while not done and t < env.max_steps:
		_o, _wr, _tr, done = env.step(straight_worker(env))
		t += 1
		if t % A.hl_every == 0 or env.goal_invalid().any():
			a, _o, _m = act(env, actor, dev)
			env.set_goals(a)
	return env.episode_stats()


def holdout(actor, dev):
	"""표준 구성(드론 4·목적지 50·CC 고정)에서 샘플링 평가. 상한은 --holdout-steps."""
	env = make_env(4, 50)
	env.max_steps = A.holdout_steps
	st = [rollout(env, actor, dev, A.eval_seed0 + i) for i in range(A.eval_n)]
	g = lambda k: float(np.mean([s[k] for s in st]))
	steps = g("steps")
	out = {"delivered": g("delivered"), "delivered_sd": float(np.std([s["delivered"] for s in st])),
	       "reloads": g("reloads"), "hop2plus": g("hop2plus"), "max_reach": g("max_reach"),
	       "full": float(np.mean([1.0 if s["makespan"] else 0.0 for s in st])), "steps": steps}
	# 선택 점수: 배송을 우선하되 같은 배송이면 빨리 끝낸 쪽을 고른다
	# makespan 선택: 완주율을 최우선으로, 같으면 평균 소요 스텝이 짧은 쪽 (상한 3000이면 완주 시각과 같다)
	out["score"] = (100.0 * out["full"] + (A.holdout_steps - steps) / A.holdout_steps * A.speed_weight
	                if A.select == "makespan" else out["delivered"])
	return out


def main():
	"""집합 상위 정책을 학습한다."""
	torch.manual_seed(A.seed)
	np.random.seed(A.seed)
	dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
	inst_rng = np.random.default_rng(A.seed + 1000)
	actor = SetManagerActor(advice=A.rule_obs, autoregressive=A.autoregressive).to(dev)
	mq, mq_t = SetManagerTwinQ(K, advice=A.rule_obs).to(dev), SetManagerTwinQ(K, advice=A.rule_obs).to(dev)
	mq_t.load_state_dict(mq.state_dict())
	a_opt, q_opt = optim.Adam(actor.parameters(), lr=3e-4), optim.Adam(mq.parameters(), lr=3e-4)
	log_alpha = torch.tensor(np.log(0.1), requires_grad=True, device=dev)
	al_opt = optim.Adam([log_alpha], lr=3e-4)
	buf = OptionBuffer()

	LOG_DIR, W_DIR = os.path.join(ROOT, "runs", A.tag), os.path.join(ROOT, "weights", A.tag)
	os.makedirs(LOG_DIR, exist_ok=True), os.makedirs(W_DIR, exist_ok=True)
	writer = SummaryWriter(log_dir=LOG_DIR)
	CSV, EVL = os.path.join(LOG_DIR, f"metrics_{A.tag}.csv"), os.path.join(LOG_DIR, f"evals_{A.tag}.csv")
	cols = ["episode", "n_drones", "n_dests", "steps", "team_reward", "delivered", "makespan", "reloads",
	        "hop2plus", "max_reach", "stall_ratio", "max_stall_run", "term_reason", "relay_frac", "wall_sec"]
	ecols = ["episode", "delivered", "delivered_sd", "reloads", "hop2plus", "max_reach", "full", "steps", "score", "is_best", "wall_sec"]

	start_ep, best, best_ep, stale = 1, -1.0, 0, 0
	latest = os.path.join(W_DIR, "latest.pth")
	if A.resume and os.path.exists(latest):
		ck = torch.load(latest, map_location=dev, weights_only=False)
		actor.load_state_dict(ck["actor"]); mq.load_state_dict(ck["critic"]); mq_t.load_state_dict(ck["critic_t"])
		log_alpha.data.fill_(ck["log_alpha"])
		start_ep, best, best_ep, stale = ck["episode"] + 1, ck["best"], ck["best_ep"], ck["stale"]
		print(f"재개: ep{start_ep}, 최고 {best:.1f}@ep{best_ep} 정체 {stale}", flush=True)
	if A.init_from and not A.resume:
		ck = torch.load(os.path.join(ROOT, A.init_from), map_location=dev, weights_only=False)
		actor.load_state_dict(ck["actor"]); mq.load_state_dict(ck["critic"]); mq_t.load_state_dict(ck["critic_t"])
		log_alpha.data.fill_(ck["log_alpha"])
		print(f"이어받기: {A.init_from} (ep{ck['episode']}) — 에피소드·최고 기록은 초기화", flush=True)
	mode = "a" if start_ep > 1 else "w"
	cf, ef = open(CSV, mode, newline="", encoding="utf-8"), open(EVL, mode, newline="", encoding="utf-8")
	cw, ew = csv.DictWriter(cf, fieldnames=cols), csv.DictWriter(ef, fieldnames=ecols)
	if mode == "w":
		cw.writeheader(); ew.writeheader()

	print(f"집합 상위 학습 | 반경 {A.comm_range:.0f} 상한 {A.max_steps} 무배송 {A.no_progress_limit} "
	      f"gamma {GAMMA} 구성={'무작위 드론' + str(A.drones_range) + str(A.drone_weights or '') + ' 목적지' + str(A.dests_range) + ' CC무작위' if A.random_config else f'고정 드론 {A.num_drones} 목적지 {A.num_dests}'} "
	      f"인스턴스={'고정' if A.fixed_instance else '무작위'} 목표유지={A.commit_max if A.commit else 'off'} "
	      f"완주보상={A.complete_bonus} 잔여페널티={A.residual_penalty} 규칙관측={A.rule_obs} 규칙정규화={A.rule_reg} 막힘페널티={A.blocked_penalty} 도달권shaping={A.coverage_shaping} 정체전환보상={A.stall_switch_bonus} 엔트로피비={A.ent_frac}->{A.ent_frac_final}@{A.ent_anneal_episodes} 자기회귀={A.autoregressive} 홀드아웃상한={A.holdout_steps} 선택={A.select}/속도가중치={A.speed_weight}", flush=True)
	t0 = time.time()
	env = make_env(A.num_drones, A.num_dests)

	for ep in range(start_ep, A.max_episodes + 1):
		# 엔트로피 목표를 선형 감쇠시킨다. 초반에는 넓게 탐색하고 후반에는 정책을 날카롭게 한다
		span = A.ent_anneal_episodes if A.ent_anneal_episodes > 0 else A.max_episodes
		ent_frac = (A.ent_frac if A.ent_frac_final < 0 else
		            A.ent_frac + (A.ent_frac_final - A.ent_frac) * min(1.0, (ep - 1) / max(1, span - 1)))
		if A.random_config:
			n, m, cc, d = sample_config(inst_rng, A.drones_range, A.dests_range, random_cc=True,
			                            drone_weights=A.drone_weights)
			if (n, m) != (env.num_drones, env.num_dests):
				env = make_env(n, m)
			env.cc_pos, env.dests_pos = cc, d
		elif not A.fixed_instance:
			env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=int(inst_rng.integers(0, 2 ** 31 - 1)),
			                                            num_drones=env.num_drones, comm_range=env.comm_range)
		env.reset()
		a, o0, m0 = act(env, actor, dev)
		r0 = act_rule(env)
		env.set_goals(a)
		done, t, ep_tr, opt_r, opt_k, relay_n, dec_n = False, 0, 0.0, 0.0, 0, 0, 0
		while not done and t < A.max_steps:
			_obs, _wr, tr, done = env.step(straight_worker(env))
			t += 1
			ep_tr += tr
			opt_r += (GAMMA ** opt_k) * tr
			opt_k += 1
			hl_now = (t % A.hl_every == 0) or bool(env.goal_invalid().any())
			if hl_now or done:
				a2, o2, m2 = act(env, actor, dev)
				r2 = act_rule(env)
				buf.push(o0, a, m0, opt_r, o2, m2, GAMMA ** opt_k, float(done), ra=r0)
				relay_n += int((a == 6).sum()); dec_n += len(a)
				if not done:
					env.set_goals(a2)
					a, o0, m0, r0, opt_r, opt_k = a2, o2, m2, r2, 0.0, 0

			if len(buf) > WARMUP and t % A.update_every == 0:
				b = buf.sample(BATCH, dev)
				dm = b["o"]["drone_mask"].float()
				alpha = log_alpha.exp().detach()
				with torch.no_grad():
					_, p2, lp2 = actor(b["o2"], b["mask2"])
					nq1, nq2 = mq_t(b["o2"], b["o2"]["drone_mask"])
					v2 = (p2 * (torch.min(nq1, nq2) - alpha * lp2)).sum(-1)          # (B,N)
					v2 = (v2 * dm).sum(-1, keepdim=True) / dm.sum(-1, keepdim=True)
					y = b["r"] + b["disc"] * (1 - b["d"]) * v2
				q1, q2 = mq(b["o"], b["o"]["drone_mask"])
				qa1 = (q1.gather(-1, b["ma"].unsqueeze(-1)).squeeze(-1) * dm).sum(-1, keepdim=True) / dm.sum(-1, keepdim=True)
				qa2 = (q2.gather(-1, b["ma"].unsqueeze(-1)).squeeze(-1) * dm).sum(-1, keepdim=True) / dm.sum(-1, keepdim=True)
				lq = nn.MSELoss()(qa1, y) + nn.MSELoss()(qa2, y)
				q_opt.zero_grad(); lq.backward(); nn.utils.clip_grad_norm_(mq.parameters(), 1.0); q_opt.step()

				_, p, lp = actor(b["o"], b["mask"], given=b["ma"])
				with torch.no_grad():
					q1d, q2d = mq(b["o"], b["o"]["drone_mask"])
					qmin = torch.min(q1d, q2d)
				la = ((p * (alpha * lp - qmin)).sum(-1) * dm).sum() / dm.sum()
				if A.rule_reg > 0.0:
					# 규칙 행동이 마스크 안에 있는 드론만 정규화한다
					ok = dm * b["mask"].gather(-1, b["ra"].unsqueeze(-1)).squeeze(-1).float()
					nll = -lp.gather(-1, b["ra"].unsqueeze(-1)).squeeze(-1)
					la = la + A.rule_reg * (nll * ok).sum() / ok.sum().clamp_min(1.0)
				a_opt.zero_grad(); la.backward(); nn.utils.clip_grad_norm_(actor.parameters(), 1.0); a_opt.step()
				ent = (-(p * lp).sum(-1) * dm).sum() / dm.sum()
				# 목표 엔트로피는 유효 행동 수에 비례한다. 고정값을 쓰면 마스크로 선택지가 줄 때
				# 달성 불가능해져 alpha가 폭주한다 (실측: 0.1 -> 6e7, ep281).
				n_valid = b["mask"].float().sum(-1).clamp_min(1.0)
				target_ent = ((ent_frac * torch.log(n_valid)) * dm).sum() / dm.sum()
				al = -(log_alpha * (target_ent - ent).detach())
				al_opt.zero_grad(); al.backward(); al_opt.step()
				log_alpha.data.clamp_(np.log(1e-3), np.log(10.0))   # 안전장치
				for pt, pp in zip(mq_t.parameters(), mq.parameters()):
					pt.data.copy_((1 - TAU) * pt.data + TAU * pp.data)

		st = env.episode_stats()
		row = {"episode": ep, "n_drones": env.num_drones, "n_dests": env.num_dests, "steps": t,
		       "team_reward": round(ep_tr, 3), "delivered": st["delivered"], "makespan": st["makespan"] or "",
		       "reloads": st["reloads"], "hop2plus": round(st["hop2plus"], 4), "max_reach": round(st["max_reach"], 1),
		       "stall_ratio": round(st["stall_ratio"], 4), "max_stall_run": st["max_stall_run"],
		       "term_reason": st["term_reason"], "relay_frac": round(relay_n / max(1, dec_n), 4),
		       "wall_sec": round(time.time() - t0, 1)}
		cw.writerow(row); cf.flush()
		writer.add_scalar("train/delivered", st["delivered"], ep)
		writer.add_scalar("train/relay_frac", row["relay_frac"], ep)
		writer.add_scalar("train/alpha", float(log_alpha.exp()), ep)
		row_alpha = float(log_alpha.exp())
		print(f"Ep {ep} | 드론{env.num_drones} 목적지{env.num_dests:2d} | {t:4d}스텝 | 배송 {st['delivered']:2d}/{env.num_dests} "
		      f"| 재적재 {st['reloads']:2d} | 중계율 {row['relay_frac']:.2f} | alpha {row_alpha:.3f} | {st['term_reason']}", flush=True)

		if ep % CKPT_EVERY == 0:
			torch.save(actor.state_dict(), os.path.join(W_DIR, f"manager_ep{ep}.pth"))
			tmp = latest + ".tmp"
			torch.save({"episode": ep, "actor": actor.state_dict(), "critic": mq.state_dict(),
			            "critic_t": mq_t.state_dict(), "log_alpha": float(log_alpha),
			            "best": best, "best_ep": best_ep, "stale": stale}, tmp)
			os.replace(tmp, latest)

		if ep % A.eval_every == 0 and len(buf) > WARMUP:
			ev = holdout(actor, dev)
			is_best = ev["score"] > best
			if is_best:
				best, best_ep, stale = ev["score"], ep, 0
				torch.save(actor.state_dict(), os.path.join(W_DIR, "best_manager.pth"))
			else:
				stale += 1
			ew.writerow({"episode": ep, **{k: round(v, 4) for k, v in ev.items()}, "is_best": int(is_best),
			             "wall_sec": round(time.time() - t0, 1)}); ef.flush()
			for k, v in ev.items():
				writer.add_scalar(f"holdout/{k}", v, ep)
			print(f"  [홀드아웃] 배송 {ev['delivered']:.1f}±{ev['delivered_sd']:.1f} 완주 {ev['full']:.2f} 스텝 {ev['steps']:.0f} 점수 {ev['score']:.1f} "
			      f"재적재 {ev['reloads']:.1f} 2홉+ {ev['hop2plus']:.2f} | 최고 {best:.1f}@ep{best_ep} 정체 {stale}/{A.patience}"
			      f"{'  ← 갱신' if is_best else ''}", flush=True)
			if stale >= A.patience:
				print(f"\n조기 종료. 최고 {best:.1f}@ep{best_ep}", flush=True)
				break
	print(f"Training finished. best={best:.1f} @ ep{best_ep}", flush=True)


if __name__ == "__main__":
	main()
