"""계층 MARL 학습 루프 (v3, 26_08_31_19).

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

from env.disaster_relay_env_v3_26_08_31_19 import DisasterRelayDroneEnv
from model.hier_net_v3_26_08_31_19 import ManagerActor, ManagerTwinQ, TwinQ, WorkerActor
from util.instance_generator_v2_26_08_25_14 import sample_instance

BATCH, WARMUP, TAU, GAMMA = 512, 5000, 0.001, 0.99
POLICY_DELAY, MAX_STEPS, CKPT_EVERY = 2, 1000, 10

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
A = P.parse_args()

LOG_DIR = os.path.join(ROOT, "runs", A.tag)
W_DIR = os.path.join(ROOT, "weights", A.tag)
CSV_PATH = os.path.join(LOG_DIR, f"metrics_{A.tag}.csv")
EVAL_PATH = os.path.join(LOG_DIR, f"evals_{A.tag}.csv")
CSV_COLS = ["episode", "steps", "team_reward", "worker_reward", "delivered", "makespan",
            "t_last_deliv", "idle_drones", "reloads", "arrivals", "blocked", "hop2plus",
            "max_reach", "comm_loss", "env_steps", "wall_sec"]
EVAL_COLS = ["episode", "delivered", "delivered_sd", "reloads", "idle_drones",
             "hop2plus", "max_reach", "comm_loss", "is_best", "wall_sec"]


def rule_manager(env):
	"""규칙 기반 상위: 적재량이 있으면 최근접 미배송지, 없으면 복귀.

	후보 목록의 0번이 최근접이므로 배송은 항상 행동 0을 고른다.
	"""
	acts = np.zeros(env.num_drones, dtype=int)
	for i in range(env.num_drones):
		if env.drones_capacity[i] == 0:
			acts[i] = env.n_cand                       # 복귀
		elif env.candidates(i)[0] < 0:
			acts[i] = env.n_cand + 1                   # 남은 목적지 없음 → 유지
		else:
			acts[i] = 0                                # 최근접 미배송지
	return acts


def action_mask(env, device):
	"""선택 불가능한 상위 행동을 가린다 — 없는 후보, 적재량 0일 때의 배송."""
	m = np.ones((env.num_drones, env.n_cand + 2), dtype=bool)
	for i in range(env.num_drones):
		cand = env.candidates(i)
		for k in range(env.n_cand):
			m[i, k] = cand[k] >= 0 and env.drones_capacity[i] > 0
		m[i, env.n_cand] = env.drones_capacity[i] < env.max_capacity   # 복귀
		if not m[i].any():
			m[i, env.n_cand + 1] = True
	return torch.as_tensor(m, device=device)


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

	def push(self, **kw):
		i = self.ptr
		for key, val in kw.items():
			getattr(self, key)[i] = val
		self.ptr = (self.ptr + 1) % self.cap
		self.size = min(self.size + 1, self.cap)

	def sample(self, bs, dev, hl_only=False):
		"""hl_only면 상위 결정이 있었던 전이만 뽑는다."""
		pool = np.flatnonzero(self.is_hl[:self.size, 0] > 0) if hl_only else np.arange(self.size)
		if len(pool) < bs:
			return None
		idx = pool[np.random.randint(0, len(pool), bs)]
		t = lambda x: torch.as_tensor(x[idx], device=dev)
		return dict(wo=t(self.wo), wo2=t(self.wo2), wa=t(self.wa), wr=t(self.wr),
		            mo=t(self.mo), mo2=t(self.mo2), ma=t(self.ma), mask2=t(self.mask2),
		            s=t(self.s), s2=t(self.s2), tr=t(self.tr), d=t(self.d))

	def __len__(self):
		return self.size


def rollout(env, worker, manager, device, deterministic, hl_every, seed=None):
	"""한 에피소드를 굴리고 지표를 반환한다. manager가 None이면 규칙 상위를 쓴다."""
	if seed is not None:
		env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=seed)
	obs = env.reset()
	env.set_goals(rule_manager(env) if manager is None else _mgr_act(env, manager, device, True))
	done, t = False, 0
	while not done and t < MAX_STEPS:
		with torch.no_grad():
			a, _ = worker(torch.as_tensor(obs, dtype=torch.float32, device=device),
			              deterministic=deterministic)
		obs, _wr, _tr, done = env.step(a.cpu().numpy())
		t += 1
		if t % hl_every == 0 or env.goal_invalid().any():
			env.set_goals(rule_manager(env) if manager is None
			              else _mgr_act(env, manager, device, deterministic))
	return env.episode_stats()


def _mgr_act(env, manager, device, deterministic):
	"""상위 정책으로 드론별 행동을 고른다."""
	mo = torch.as_tensor(env.manager_obs(), dtype=torch.float32, device=device)
	with torch.no_grad():
		act, probs, _ = manager(mo, action_mask(env, device))
	return (probs.argmax(-1) if deterministic else act).cpu().numpy()


def holdout(env, worker, manager, device, n, seed0, hl_every):
	"""홀드아웃 인스턴스에서 결정론적으로 평가한다."""
	st = [rollout(env, worker, manager, device, True, hl_every, seed0 + i) for i in range(n)]
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
	kw = dict(comm_range=A.comm_range, cluster_penalty=not A.no_cluster_penalty)
	env, eval_env = DisasterRelayDroneEnv(**kw), DisasterRelayDroneEnv(**kw)
	env.reset()
	wo_dim = env.worker_obs().shape[-1]
	mo_dim = env.manager_obs().shape[-1]
	s_dim = env.global_state().shape[0]
	K = env.n_cand + 2
	N = env.num_drones

	print(f"단계={A.stage} 장치={dev} 반경={A.comm_range} 상위주기={A.hl_every} 시드={A.seed}", flush=True)
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
	t0 = time.time()

	for ep in range(start_ep, A.max_episodes + 1):
		obs = env.reset()
		mo = env.manager_obs()
		ma = _mgr_act(env, manager, dev, False) if use_learned_mgr else rule_manager(env)
		env.set_goals(ma)
		s = env.global_state()
		done, t, ep_tr, ep_wr = False, 0, 0.0, 0.0

		while not done and t < MAX_STEPS:
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
			buf.push(wo=obs, wo2=nobs, wa=wa, wr=wr, mo=mo, mo2=nmo, ma=ma,
			         mask2=action_mask(env, dev).cpu().numpy(), s=s, s2=ns,
			         tr=tr, d=float(done), is_hl=float(hl_now))
			obs, s = nobs, ns
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
						y = bh["tr"] + GAMMA * (1 - bh["d"]) * v2
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
		             ("Comm/BlockedMoves", st["blocked"]), ("Comm/LossTerminated", st["comm_loss"])]:
			writer.add_scalar(k, v, ep)
		cw.writerow({"episode": ep, "steps": t, "team_reward": round(ep_tr, 3),
		             "worker_reward": round(ep_wr, 3), "delivered": st["delivered"],
		             "makespan": st["makespan"] if st["makespan"] else "",
		             "t_last_deliv": st["t_last_deliv"], "idle_drones": st["idle_drones"],
		             "reloads": st["reloads"], "arrivals": st["arrivals"], "blocked": st["blocked"],
		             "hop2plus": round(st["hop2plus"], 4), "max_reach": round(st["max_reach"], 1),
		             "comm_loss": st["comm_loss"], "env_steps": env_steps,
		             "wall_sec": round(time.time() - t0, 1)})
		cf.flush()
		print(f"Ep {ep} | 배송 {st['delivered']:2d}/50 | 재적재 {st['reloads']} | "
		      f"도착 {st['arrivals']:3d} | 2홉+ {st['hop2plus']:.2f} | 도달 {st['max_reach']:4.0f} | "
		      f"차단 {st['blocked']:4d}", flush=True)

		if ep % CKPT_EVERY == 0:
			torch.save(worker.state_dict(), os.path.join(W_DIR, f"worker_ep{ep}.pth"))
			torch.save(manager.state_dict(), os.path.join(W_DIR, f"manager_ep{ep}.pth"))
			tmp = os.path.join(W_DIR, "latest.tmp")
			torch.save({"episode": ep, "worker": worker.state_dict(),
			            "manager": manager.state_dict()}, tmp)
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
