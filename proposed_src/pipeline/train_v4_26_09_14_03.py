"""계층 MARL 학습 루프 (v3, 26_09_07_17).

--stage worker : 상위를 규칙으로 고정하고 하위(추력)만 학습 — 계층 분해 타당성 검증
--stage manager: 하위를 고정하고 상위(이산 할당)만 학습 — 학습된 할당이 규칙을 이기는지
--stage joint  : 두 층 동시 학습
"""

import argparse
import csv
import os
import sys
import time

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.tensorboard import SummaryWriter

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v4_26_09_14_03 import DisasterRelayDroneEnv
from pipeline.common_v4_26_09_14_03 import (action_mask, chain_manager,
	plain_manager, straight_worker)
from model.hier_net_v3_26_08_31_19 import ManagerActor, ManagerTwinQ, TwinQ, WorkerActor
from util.instance_generator_v2_26_08_25_14 import sample_instance

# GAMMA 0.99 → 0.997: 에피소드 상한을 10배로 늘렸으므로 유효 지평도 100 → 333스텝으로
# 넓힌다. 재적재 왕복은 수백 스텝짜리 사이클이라 0.99에서는 사실상 보이지 않았다.
BATCH, WARMUP, TAU = 512, 5000, 0.001
POLICY_DELAY, CKPT_EVERY = 2, 10

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

P = argparse.ArgumentParser()
P.add_argument("--tag", default="v3_26_08_31_19")
P.add_argument("--stage", choices=["worker", "manager", "joint"], default="worker")
P.add_argument("--comm-range", type=float, default=300.0)
P.add_argument("--hl-every", type=int, default=20, help="상위 결정 주기 (스텝)")
P.add_argument("--worker-ckpt", default="", help="manager/joint 단계에서 불러올 하위 가중치")
P.add_argument("--manager-ckpt", default="", help="joint 단계에서 이어받을 상위 가중치")
P.add_argument("--max-episodes", type=int, default=100000)
P.add_argument("--eval-every", type=int, default=50)
P.add_argument("--eval-n", type=int, default=6)
P.add_argument("--eval-seed0", type=int, default=201)
P.add_argument("--patience", type=int, default=15)
P.add_argument("--no-cluster-penalty", action="store_true")
P.add_argument("--resume", action="store_true")
P.add_argument("--seed", type=int, default=0, help="학습 시드 (초기화·탐색 재현용)")
P.add_argument("--no-stuck-obs", action="store_true", help="교착 신호 관측 제거 (절제 실험)")
P.add_argument("--max-steps", type=int, default=1000, help="에피소드 스텝 상한")
P.add_argument("--deadlock-limit", type=int, default=10**9)
P.add_argument("--no-progress-limit", type=int, default=10**9)
P.add_argument("--warmup-episodes", type=int, default=300,
               help="이 에피소드까지는 무배송 한도를 완화해 재적재 사이클을 경험시킨다")
P.add_argument("--warmup-no-progress", type=int, default=10**9)
P.add_argument("--gamma", type=float, default=0.997, help="할인율 (지평 = 1/(1-gamma))")
P.add_argument("--no-relay", action="store_true",
               help="중계 행동을 마스크로 완전히 차단 — 진짜 통제군")
P.add_argument("--relay-hold", type=float, default=0.02,
               help="중계 정박 보상 (컷 정점일 때만 지급). 배송 보상과의 균형을 정한다")
P.add_argument("--straight-worker", action="store_true",
               help="하위를 학습 대신 직진 제어로 고정 — 상위 배분 능력만 격리해서 본다")
P.add_argument("--rule-manager", choices=["plain", "chain"], default="chain",
               help="규칙 상위 종류 — chain은 중계를 쓴다 (하위 학습 시 중계 목표를 겪게 함)")
P.add_argument("--arrive-once", action="store_true",
               help="도달 보너스를 목표당 1회로 (회귀 절제용 — 기본은 매 스텝)")
P.add_argument("--incomplete-penalty", type=float, default=0.0,
               help="종료 시 남은 목적지당 감점 (0이면 비활성)")
A = P.parse_args()
rule_manager = chain_manager if A.rule_manager == "chain" else plain_manager
GAMMA = A.gamma

LOG_DIR = os.path.join(ROOT, "runs", A.tag)
W_DIR = os.path.join(ROOT, "weights", A.tag)
CSV_PATH = os.path.join(LOG_DIR, f"metrics_{A.tag}.csv")
EVAL_PATH = os.path.join(LOG_DIR, f"evals_{A.tag}.csv")
CSV_COLS = ["episode", "steps", "team_reward", "worker_reward", "delivered", "makespan",
            "t_last_deliv", "idle_drones", "reloads", "arrivals", "blocked", "hop2plus",
            "max_reach", "stall_ratio", "max_stall_run", "term_reason", "comm_loss", "env_steps", "wall_sec"]
EVAL_COLS = ["episode", "delivered", "delivered_sd", "reloads", "idle_drones",
             "hop2plus", "max_reach", "comm_loss", "is_best", "wall_sec"]



class Buffer:
	"""하위·상위 전이를 함께 담는 리플레이 버퍼."""

	def __init__(self, n, wo, mo, sdim, k, cap=100000):
		self.cap, self.n, self.ptr, self.size = cap, n, 0, 0
		z = lambda *s: np.zeros(s, dtype=np.float32)
		self.wo, self.wo2 = z(cap, n, wo), z(cap, n, wo)
		self.wa, self.wr = z(cap, n, 2), z(cap, n)
		self.mo, self.mo2 = z(cap, n, mo), z(cap, n, mo)
		self.ma = np.zeros((cap, n), dtype=np.int64)
		self.mask2 = np.ones((cap, n, k), dtype=bool)
		self.s, self.s2 = z(cap, sdim), z(cap, sdim)
		self.tr, self.d, self.is_hl = z(cap, 1), z(cap, 1), z(cap, 1)
		# 상위 전이는 옵션(hl_every 스텝) 단위다. 구간 누적 보상과 gamma^k 를 따로 담는다.
		self.mtr, self.mdisc = z(cap, 1), z(cap, 1)

	def push(self, **kw):
		i = self.ptr
		for key, val in kw.items():
			getattr(self, key)[i] = val
		self.ptr = (self.ptr + 1) % self.cap
		self.size = min(self.size + 1, self.cap)
		return i

	def sample(self, bs, dev, hl_only=False):
		"""hl_only면 상위 결정이 있었던 전이만 뽑는다."""
		pool = np.flatnonzero(self.is_hl[:self.size, 0] > 0) if hl_only else np.arange(self.size)
		if len(pool) < bs:
			return None
		idx = pool[np.random.randint(0, len(pool), bs)]
		t = lambda x: torch.as_tensor(x[idx], device=dev)
		return dict(wo=t(self.wo), wo2=t(self.wo2), wa=t(self.wa), wr=t(self.wr),
		            mo=t(self.mo), mo2=t(self.mo2), ma=t(self.ma), mask2=t(self.mask2),
		            s=t(self.s), s2=t(self.s2), tr=t(self.tr), d=t(self.d),
		            mtr=t(self.mtr), mdisc=t(self.mdisc))

	def __len__(self):
		return self.size


def rollout(env, worker, manager, device, deterministic, hl_every, seed=None):
	"""한 에피소드를 굴리고 지표를 반환한다. manager가 None이면 규칙 상위를 쓴다."""
	if seed is not None:
		env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=seed)
	obs = env.reset()
	env.set_goals(rule_manager(env) if manager is None else _mgr_act(env, manager, device, True))
	done, t = False, 0
	while not done and t < A.max_steps:
		if A.straight_worker:
			a = straight_worker(env)
		else:
			with torch.no_grad():
				wa, _ = worker(torch.as_tensor(obs, dtype=torch.float32, device=device),
				               deterministic=deterministic)
			a = wa.cpu().numpy()
		obs, _wr, _tr, done = env.step(a)
		t += 1
		if t % hl_every == 0 or env.goal_invalid().any():
			env.set_goals(rule_manager(env) if manager is None
			              else _mgr_act(env, manager, device, deterministic))
	return env.episode_stats()


def _mgr_act(env, manager, device, deterministic):
	"""상위 정책으로 드론별 행동을 고른다."""
	mo = torch.as_tensor(env.manager_obs(), dtype=torch.float32, device=device)
	with torch.no_grad():
		act, probs, _ = manager(mo, action_mask(env, device, A.no_relay))
	return (probs.argmax(-1) if deterministic else act).cpu().numpy()


def holdout(env, worker, manager, device, n, seed0, hl_every):
	"""홀드아웃 인스턴스에서 확률 샘플링으로 평가한다."""
	# argmax로 재면 안 된다. 같은 가중치·같은 인스턴스에서 argmax 2.67 대 샘플링 30.58로
	# 11배 차이가 났다. argmax는 교착에서 같은 관측에 같은 행동을 내놓아 빠져나오지 못한다.
	# 이 지표로 조기 종료와 최고 체크포인트를 정하면 엉뚱한 가중치가 선택된다.
	st = [rollout(env, worker, manager, device, False, hl_every, seed0 + i) for i in range(n)]
	g = lambda k: float(np.mean([s[k] for s in st]))
	return {"delivered": g("delivered"),
	        "delivered_sd": float(np.std([s["delivered"] for s in st])),
	        "reloads": g("reloads"), "idle_drones": g("idle_drones"),
	        "hop2plus": g("hop2plus"), "max_reach": g("max_reach"),
	        "comm_loss": int(sum(s["comm_loss"] for s in st))}


def main():
	"""단계에 맞춰 학습 루프를 돌린다."""
	# 학습 시드. 같은 설정을 여러 시드로 돌려 결과가 재현되는지 확인하기 위함이다.
	torch.manual_seed(A.seed)
	np.random.seed(A.seed)
	dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
	kw = dict(comm_range=A.comm_range, cluster_penalty=not A.no_cluster_penalty,
	          stuck_obs=not A.no_stuck_obs, max_steps=A.max_steps,
	          deadlock_limit=A.deadlock_limit, no_progress_limit=A.no_progress_limit,
	          incomplete_penalty=A.incomplete_penalty, arrive_once=A.arrive_once,
	          relay_hold=A.relay_hold)
	env, eval_env = DisasterRelayDroneEnv(**kw), DisasterRelayDroneEnv(**kw)
	env.reset()
	wo_dim = env.worker_obs().shape[-1]
	mo_dim = env.manager_obs().shape[-1]
	s_dim = env.global_state().shape[0]
	K = env.n_cand + 2
	N = env.num_drones

	print(f"단계={A.stage} 장치={dev} 반경={A.comm_range} 상위주기={A.hl_every} 시드={A.seed} 교착관측={not A.no_stuck_obs}", flush=True)
	print(f"상한={A.max_steps} 교착한도={A.deadlock_limit} 무배송한도={A.no_progress_limit} "
	      f"(초반 {A.warmup_episodes}ep까지 {A.warmup_no_progress}) gamma={GAMMA} "
	      f"미완료페널티={A.incomplete_penalty} "
	      f"도달보너스={'목표당1회' if A.arrive_once else '매스텝'} "
	      f"중계정박={A.relay_hold}", flush=True)
	print(f"worker_obs={wo_dim} manager_obs={mo_dim} state={s_dim} 상위행동={K}", flush=True)

	worker = WorkerActor(wo_dim).to(dev)
	wq, wq_t = TwinQ(s_dim, 2 * N).to(dev), TwinQ(s_dim, 2 * N).to(dev)
	wq_t.load_state_dict(wq.state_dict())
	w_opt = optim.Adam(worker.parameters(), lr=3e-4)
	wq_opt = optim.Adam(wq.parameters(), lr=3e-4)
	w_alpha = torch.tensor(np.log(0.1), requires_grad=True, device=dev)
	w_alpha_opt = optim.Adam([w_alpha], lr=3e-4)
	w_target_ent = -2.0 * N * 0.6

	manager = ManagerActor(mo_dim, K).to(dev)
	mq, mq_t = ManagerTwinQ(s_dim, N, K).to(dev), ManagerTwinQ(s_dim, N, K).to(dev)
	mq_t.load_state_dict(mq.state_dict())
	m_opt = optim.Adam(manager.parameters(), lr=3e-4)
	mq_opt = optim.Adam(mq.parameters(), lr=3e-4)
	m_alpha = torch.tensor(np.log(0.1), requires_grad=True, device=dev)
	m_alpha_opt = optim.Adam([m_alpha], lr=3e-4)
	m_target_ent = 0.6 * np.log(K)          # 이산 SAC 관례: 최대 엔트로피의 일정 비율

	if A.worker_ckpt:
		worker.load_state_dict(torch.load(os.path.join(ROOT, A.worker_ckpt), map_location=dev))
		print(f"하위 가중치 로드: {A.worker_ckpt}", flush=True)
	if A.manager_ckpt:
		manager.load_state_dict(torch.load(os.path.join(ROOT, A.manager_ckpt), map_location=dev))
		print(f"상위 가중치 로드: {A.manager_ckpt}", flush=True)

	train_w = A.stage in ("worker", "joint")
	train_m = A.stage in ("manager", "joint")
	use_learned_mgr = A.stage in ("manager", "joint")

	buf = Buffer(N, wo_dim, mo_dim, s_dim, K)
	os.makedirs(LOG_DIR, exist_ok=True)
	os.makedirs(W_DIR, exist_ok=True)
	writer = SummaryWriter(log_dir=LOG_DIR)

	start_ep, resumed = 1, None
	latest = os.path.join(W_DIR, "latest.pth")
	if A.resume and os.path.exists(latest):
		resumed = torch.load(latest, map_location=dev, weights_only=False)
		worker.load_state_dict(resumed["worker"])
		manager.load_state_dict(resumed["manager"])
		start_ep = int(resumed["episode"]) + 1
		print(f"재개: ep{start_ep}부터", flush=True)

	mode = "a" if resumed else "w"
	cf = open(CSV_PATH, mode, newline="", encoding="utf-8")
	cw = csv.DictWriter(cf, fieldnames=CSV_COLS)
	ef = open(EVAL_PATH, mode, newline="", encoding="utf-8")
	ew = csv.DictWriter(ef, fieldnames=EVAL_COLS)
	if not resumed:
		cw.writeheader()
		ew.writeheader()

	best, best_ep, stale, env_steps = -1.0, 0, 0, 0
	# 재개 시 최고 기록을 이어받지 않으면 첫 홀드아웃이 무조건 최고로 기록되어
	# 재개 전에 저장해둔 best_*.pth 를 더 나쁜 가중치로 덮어쓴다.
	if resumed:
		best = float(resumed.get("best", -1.0))
		best_ep = int(resumed.get("best_ep", 0))
		stale = int(resumed.get("stale", 0))
		if best < 0 and os.path.exists(EVAL_PATH):
			with open(EVAL_PATH, newline="", encoding="utf-8") as f:
				prev = [r for r in csv.DictReader(f) if r.get("delivered")]
			hit = [(float(r["delivered"]), int(r["episode"])) for r in prev
			       if r["episode"].isdigit()]
			if hit:
				best, best_ep = max(hit)
				stale = sum(1 for d, e in hit if e > best_ep)
		print(f"최고 기록 이어받음: {best:.1f}@ep{best_ep} 정체 {stale}", flush=True)
	t0 = time.time()

	for ep in range(start_ep, A.max_episodes + 1):
		# 학습 초반에는 무배송 한도를 늘려, 아직 배송을 못 하는 정책도
		# 재적재 왕복 사이클을 한 번은 경험하게 한다.
		env.no_progress_limit = (A.warmup_no_progress if ep <= A.warmup_episodes
		                         else A.no_progress_limit)
		obs = env.reset()
		mo = env.manager_obs()
		ma = _mgr_act(env, manager, dev, False) if use_learned_mgr else rule_manager(env)
		env.set_goals(ma)
		s = env.global_state()
		done, t, ep_tr, ep_wr = False, 0, 0.0, 0.0
		# 진행 중인 상위 옵션의 시작 인덱스·누적 보상·길이. opt_new면 다음 전이가 옵션의 첫 스텝이다.
		opt_i, opt_r, opt_k, opt_new = None, 0.0, 0, True

		while not done and t < A.max_steps:
			if A.straight_worker:
				wa = straight_worker(env)
			else:
				with torch.no_grad():
					wa, _ = worker(torch.as_tensor(obs, dtype=torch.float32, device=dev))
				wa = wa.cpu().numpy()
			nobs, wr, tr, done = env.step(wa)
			t += 1
			env_steps += 1
			ep_tr += tr
			ep_wr += float(wr.mean())

			hl_now = (t % A.hl_every == 0) or bool(env.goal_invalid().any())
			nmo, ns = env.manager_obs(), env.global_state()
			nmask = action_mask(env, dev, A.no_relay).cpu().numpy()
			idx = buf.push(wo=obs, wo2=nobs, wa=wa, wr=wr, mo=mo, mo2=nmo, ma=ma,
			               mask2=nmask, s=s, s2=ns,
			               tr=tr, d=float(done), is_hl=0.0)
			# 상위 행동은 다음 결정까지 hl_every 스텝 유지된다. 1스텝 전이로 저장하면
			# 중계처럼 나중에 값이 나오는 행동이 원리적으로 평가되지 않는다.
			# 구간 누적 보상과 구간 끝 상태를 이전 결정 지점에 소급해서 적는다.
			if opt_new:                        # 이 전이가 새 옵션의 첫 스텝이다
				opt_i, opt_r, opt_k, opt_new = idx, 0.0, 0, False
			opt_r += (GAMMA ** opt_k) * tr
			opt_k += 1
			obs, s = nobs, ns
			if hl_now or done:
				if opt_i is not None:
					buf.mtr[opt_i] = opt_r
					buf.mdisc[opt_i] = GAMMA ** opt_k
					buf.mo2[opt_i] = nmo
					buf.s2[opt_i] = ns
					buf.mask2[opt_i] = nmask
					buf.d[opt_i] = float(done)
					buf.is_hl[opt_i] = 1.0     # 옵션 시작점만 상위 학습 표본이 된다
				opt_i, opt_new = None, True
			if hl_now:
				ma = _mgr_act(env, manager, dev, False) if use_learned_mgr else rule_manager(env)
				env.set_goals(ma)
				mo = env.manager_obs()

			if len(buf) > WARMUP:
				b = buf.sample(BATCH, dev)
				if b is not None and train_w:
					alpha = w_alpha.exp().detach()
					with torch.no_grad():
						na, nlp = worker(b["wo2"])
						q1, q2 = wq_t(b["s2"], na.flatten(1))
						y = (b["wr"].mean(1, keepdim=True)
						     + GAMMA * (1 - b["d"]) * (torch.min(q1, q2) - alpha * nlp.sum(1)))
					q1, q2 = wq(b["s"], b["wa"].flatten(1))
					lq = nn.MSELoss()(q1, y) + nn.MSELoss()(q2, y)
					wq_opt.zero_grad(); lq.backward()
					nn.utils.clip_grad_norm_(wq.parameters(), 1.0); wq_opt.step()
					if env_steps % POLICY_DELAY == 0:
						ca, clp = worker(b["wo"])
						la = (alpha * clp.sum(1) - wq(b["s"], ca.flatten(1))[0]).mean()
						w_opt.zero_grad(); la.backward()
						nn.utils.clip_grad_norm_(worker.parameters(), 1.0); w_opt.step()
						al = -(w_alpha * (clp.sum(1) + w_target_ent).detach()).mean()
						w_alpha_opt.zero_grad(); al.backward(); w_alpha_opt.step()
					for p_t, p in zip(wq_t.parameters(), wq.parameters()):
						p_t.data.copy_((1 - TAU) * p_t.data + TAU * p.data)

				bh = buf.sample(BATCH, dev, hl_only=True) if train_m else None
				if bh is not None:
					alpha = m_alpha.exp().detach()
					with torch.no_grad():
						_, p2, lp2 = manager(bh["mo2"], bh["mask2"])
						nq1, nq2 = mq_t(bh["s2"])
						v2 = (p2 * (torch.min(nq1, nq2) - alpha * lp2)).sum(-1).mean(-1, keepdim=True)
						y = bh["mtr"] + bh["mdisc"] * (1 - bh["d"]) * v2
					q1, q2 = mq(bh["s"])
					a_idx = bh["ma"].unsqueeze(-1)
					qa1 = q1.gather(-1, a_idx).squeeze(-1).mean(-1, keepdim=True)
					qa2 = q2.gather(-1, a_idx).squeeze(-1).mean(-1, keepdim=True)
					lq = nn.MSELoss()(qa1, y) + nn.MSELoss()(qa2, y)
					mq_opt.zero_grad(); lq.backward()
					nn.utils.clip_grad_norm_(mq.parameters(), 1.0); mq_opt.step()

					_, p, lp = manager(bh["mo"], bh["mask2"])
					with torch.no_grad():
						q1d, q2d = mq(bh["s"])
						qmin = torch.min(q1d, q2d)
					la = (p * (alpha * lp - qmin)).sum(-1).mean()
					m_opt.zero_grad(); la.backward()
					nn.utils.clip_grad_norm_(manager.parameters(), 1.0); m_opt.step()
					ent = -(p * lp).sum(-1).mean()
					al = -(m_alpha * (m_target_ent - ent).detach()).mean()
					m_alpha_opt.zero_grad(); al.backward(); m_alpha_opt.step()
					for p_t, p_ in zip(mq_t.parameters(), mq.parameters()):
						p_t.data.copy_((1 - TAU) * p_t.data + TAU * p_.data)

		st = env.episode_stats()
		for k, v in [("Reward/Team", ep_tr), ("Reward/Worker", ep_wr),
		             ("Delivery/Completed", st["delivered"]), ("Delivery/Reloads", st["reloads"]),
		             ("Delivery/IdleDrones", st["idle_drones"]), ("Worker/Arrivals", st["arrivals"]),
		             ("Relay/Hop2PlusRatio", st["hop2plus"]), ("Relay/MaxReach", st["max_reach"]),
		             ("Comm/BlockedMoves", st["blocked"]), ("Comm/LossTerminated", st["comm_loss"]),
		             ("Deadlock/StallRatio", st["stall_ratio"]),
		             ("Deadlock/MaxStallRun", st["max_stall_run"])]:
			writer.add_scalar(k, v, ep)
		cw.writerow({"episode": ep, "steps": t, "team_reward": round(ep_tr, 3),
		             "worker_reward": round(ep_wr, 3), "delivered": st["delivered"],
		             "makespan": st["makespan"] if st["makespan"] else "",
		             "t_last_deliv": st["t_last_deliv"], "idle_drones": st["idle_drones"],
		             "reloads": st["reloads"], "arrivals": st["arrivals"], "blocked": st["blocked"],
		             "hop2plus": round(st["hop2plus"], 4), "max_reach": round(st["max_reach"], 1),
		             "stall_ratio": round(st["stall_ratio"], 4),
		             "max_stall_run": st["max_stall_run"], "term_reason": st["term_reason"],
		             "comm_loss": st["comm_loss"], "env_steps": env_steps,
		             "wall_sec": round(time.time() - t0, 1)})
		cf.flush()
		print(f"Ep {ep} | {t:5d}스텝 | 배송 {st['delivered']:2d}/50 | 재적재 {st['reloads']:2d} | "
		      f"정지 {st['stall_ratio']:.2f} 교착최장 {st['max_stall_run']:3d} | {st['term_reason']}", flush=True)

		if ep % CKPT_EVERY == 0:
			torch.save(worker.state_dict(), os.path.join(W_DIR, f"worker_ep{ep}.pth"))
			torch.save(manager.state_dict(), os.path.join(W_DIR, f"manager_ep{ep}.pth"))
			tmp = os.path.join(W_DIR, "latest.tmp")
			torch.save({"episode": ep, "worker": worker.state_dict(),
			            "manager": manager.state_dict(),
			            "best": best, "best_ep": best_ep, "stale": stale}, tmp)
			os.replace(tmp, latest)

		if ep % A.eval_every == 0 and len(buf) > WARMUP:
			worker.eval()
			ev = holdout(eval_env, worker, manager if use_learned_mgr else None,
			             dev, A.eval_n, A.eval_seed0, A.hl_every)
			worker.train()
			is_best = ev["delivered"] > best
			if is_best:
				best, best_ep, stale = ev["delivered"], ep, 0
				torch.save(worker.state_dict(), os.path.join(W_DIR, "best_worker.pth"))
				torch.save(manager.state_dict(), os.path.join(W_DIR, "best_manager.pth"))
			else:
				stale += 1
			for k, v in [("Holdout/Delivered", ev["delivered"]), ("Holdout/Reloads", ev["reloads"]),
			             ("Holdout/Hop2PlusRatio", ev["hop2plus"])]:
				writer.add_scalar(k, v, ep)
			ew.writerow({"episode": ep, **{k: round(v, 4) for k, v in ev.items()},
			             "is_best": int(is_best), "wall_sec": round(time.time() - t0, 1)})
			ef.flush()
			print(f"  [홀드아웃] 배송 {ev['delivered']:.1f}±{ev['delivered_sd']:.1f} "
			      f"재적재 {ev['reloads']:.1f} 2홉+ {ev['hop2plus']:.2f} | "
			      f"최고 {best:.1f}@ep{best_ep} 정체 {stale}/{A.patience}"
			      f"{'  ← 갱신' if is_best else ''}", flush=True)
			if stale >= A.patience:
				print(f"\n조기 종료. 최고 {best:.1f}@ep{best_ep}", flush=True)
				break

	writer.close(); cf.close(); ef.close()
	print(f"Training finished. best={best:.1f} @ ep{best_ep}", flush=True)


if __name__ == "__main__":
	main()
