"""v3 계층 정책과 규칙 베이스라인을 동일 환경·동일 인스턴스에서 비교한다.

상위는 양쪽 모두 같은 규칙(최근접 미배송지 / 비면 복귀)을 쓰고 하위만 바꿔서,
학습된 저수준 제어가 직진 제어보다 나은지를 격리해서 잰다.
"""

import argparse
import csv
import os
import sys

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v3_26_09_07_17 import DisasterRelayDroneEnv
from model.hier_net_v3_26_08_31_19 import ManagerActor, WorkerActor
from util.instance_generator_v2_26_08_25_14 import sample_instance

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MAX_STEPS = 10000


def rule_manager(env):
	"""규칙 상위: 적재량이 있으면 최근접 미배송지, 없으면 복귀. 후보 0번이 최근접이다."""
	acts = np.zeros(env.num_drones, dtype=int)
	for i in range(env.num_drones):
		if env.drones_capacity[i] == 0:
			acts[i] = env.n_cand
		elif env.candidates(i)[0] < 0:
			acts[i] = env.n_cand + 1
		else:
			acts[i] = 0
	return acts


def action_mask(env, device):
	"""선택 불가능한 상위 행동을 가린다 — 없는 후보, 적재량 0일 때의 배송."""
	m = np.ones((env.num_drones, env.n_cand + 2), dtype=bool)
	for i in range(env.num_drones):
		cand = env.candidates(i)
		for k in range(env.n_cand):
			m[i, k] = cand[k] >= 0 and env.drones_capacity[i] > 0
		# 복귀는 항상 허용 — 교착에서 빠져나오는 유일한 경로다
		m[i, env.n_cand] = True
		if not m[i].any():
			m[i, env.n_cand + 1] = True
	return torch.as_tensor(m, device=device)


def straight_worker(env):
	"""하위 목표를 향해 최대 추력으로 직진한다 (규칙 베이스라인의 저수준 제어)."""
	g = env.goal_pos - env.drones_pos
	n = np.linalg.norm(g, axis=1, keepdims=True)
	return np.where(n > 1e-6, g / np.maximum(n, 1e-6), 0.0)


def hold_relay_manager(env, n_relay):
	"""앞의 n_relay대는 유지(중계), 나머지는 규칙대로 배송/복귀."""
	acts = rule_manager(env)
	for i in range(min(n_relay, env.num_drones)):
		acts[i] = env.n_cand + 1
	return acts


def rollout(env, worker_fn, manager_fn, hl_every, seed):
	"""한 인스턴스를 굴리고 지표를 반환한다."""
	env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=seed)
	env.reset()
	env.set_goals(manager_fn(env))
	done, t = False, 0
	while not done and t < MAX_STEPS:
		_o, _wr, _tr, done = env.step(worker_fn(env))
		t += 1
		if t % hl_every == 0 or env.goal_invalid().any():
			env.set_goals(manager_fn(env))
	return env.episode_stats()


def learned_worker(ckpt, obs_dim, dev):
	"""학습된 하위 정책을 결정론적 제어 함수로 감싼다."""
	w = WorkerActor(obs_dim).to(dev)
	w.load_state_dict(torch.load(ckpt, map_location=dev))
	w.eval()

	def fn(env):
		with torch.no_grad():
			a, _ = w(torch.as_tensor(env.worker_obs(), dtype=torch.float32, device=dev),
			         deterministic=True)
		return a.cpu().numpy()
	return fn


def learned_manager(ckpt, obs_dim, n_act, dev):
	"""학습된 상위 정책을 결정론적 할당 함수로 감싼다."""
	m = ManagerActor(obs_dim, n_act).to(dev)
	m.load_state_dict(torch.load(ckpt, map_location=dev))
	m.eval()

	def fn(env):
		with torch.no_grad():
			_a, probs, _ = m(torch.as_tensor(env.manager_obs(), dtype=torch.float32, device=dev),
			                 action_mask(env, dev))
		return probs.argmax(-1).cpu().numpy()
	return fn


def main():
	"""규칙 베이스라인과 학습 정책을 비교하고 표·CSV로 남긴다."""
	p = argparse.ArgumentParser()
	p.add_argument("--n", type=int, default=30)
	p.add_argument("--seed0", type=int, default=101)
	p.add_argument("--comm-range", type=float, default=300.0)
	p.add_argument("--hl-every", type=int, default=20)
	p.add_argument("--worker", nargs="*", default=[], help="학습된 하위 가중치 경로")
	p.add_argument("--manager", nargs="*", default=[], help="학습된 상위 가중치 경로 (하위와 짝)")
	p.add_argument("--no-cluster-penalty", action="store_true")
	p.add_argument("--no-stuck-obs", action="store_true")
	p.add_argument("--reps", type=int, default=1, help="시드당 반복 횟수 (환경 무작위성 평균)")
	p.add_argument("--out", default="eval_v3_26_08_31_19")
	args = p.parse_args()

	dev = torch.device("cpu")
	env = DisasterRelayDroneEnv(comm_range=args.comm_range,
	                            cluster_penalty=not args.no_cluster_penalty,
	                            stuck_obs=not args.no_stuck_obs)
	env.reset()
	wo, mo, K = env.worker_obs().shape[-1], env.manager_obs().shape[-1], env.n_cand + 2

	methods = [
		("규칙상위 + 직진하위 (= greedy)", straight_worker, rule_manager),
		("규칙상위 + 직진 (중계1대 고정)", straight_worker, lambda e: hold_relay_manager(e, 1)),
		("규칙상위 + 직진 (중계2대 고정)", straight_worker, lambda e: hold_relay_manager(e, 2)),
	]
	for i, wck in enumerate(args.worker):
		name = os.path.basename(os.path.dirname(wck)).replace("v3_26_08_31_19_", "")
		wf = learned_worker(os.path.join(ROOT, wck), wo, dev)
		if i < len(args.manager) and args.manager[i]:
			mf = learned_manager(os.path.join(ROOT, args.manager[i]), mo, K, dev)
			methods.append((f"학습상위 + 학습하위 ({name})", wf, mf))
		else:
			methods.append((f"규칙상위 + 학습하위 ({name})", wf, rule_manager))

	rows = []
	for name, wf, mf in methods:
		# 연결성 투영이 동점을 무작위로 깨므로 같은 시드도 실행마다 결과가 다르다.
		# 시드당 reps회 반복해 그 분산을 평균으로 걷어낸다.
		st = [rollout(env, wf, mf, args.hl_every, s)
		      for _ in range(args.reps)
		      for s in range(args.seed0, args.seed0 + args.n)]
		g = lambda k: float(np.mean([x[k] for x in st]))
		done = [x["makespan"] for x in st if x["makespan"]]
		rows.append({
			"method": name,
			"delivered": round(g("delivered"), 2),
			"sd": round(float(np.std([x["delivered"] for x in st])), 2),
			"se": round(float(np.std([x["delivered"] for x in st]) / np.sqrt(len(st))), 2),
			"reloads": round(g("reloads"), 2),
			"idle": round(g("idle_drones"), 2),
			"hop2plus": round(g("hop2plus"), 3),
			"reach": round(g("max_reach"), 0),
			"blocked": round(g("blocked"), 0),
			"stall": round(g("stall_ratio"), 3),
			"maxstall": round(g("max_stall_run"), 0),
			"steps": round(g("steps"), 0),
			"comm_loss": int(sum(x["comm_loss"] for x in st)),
			"full": len(done),
			"makespan": round(float(np.mean(done)), 0) if done else "-",
		})
		print(f"  {name}: {rows[-1]['delivered']}/50", flush=True)

	cols = list(rows[0].keys())
	w = {c: max(len(c), max(len(str(r[c])) for r in rows)) for c in cols}
	print(f"\n홀드아웃 {args.n}개 × {args.reps}회 = 롤아웃 {args.n * args.reps}개 "
	      f"(시드 {args.seed0}~{args.seed0 + args.n - 1}), 반경 {args.comm_range:.0f}\n")
	print(" | ".join(c.ljust(w[c]) for c in cols))
	print("-" * (sum(w.values()) + 3 * len(cols)))
	for r in rows:
		print(" | ".join(str(r[c]).ljust(w[c]) for c in cols))

	os.makedirs(os.path.join(ROOT, "figures"), exist_ok=True)
	out = os.path.join(ROOT, "figures", f"{args.out}.csv")
	with open(out, "w", newline="", encoding="utf-8") as f:
		wr = csv.DictWriter(f, fieldnames=cols)
		wr.writeheader()
		wr.writerows(rows)
	print(f"\n저장: {out}")


if __name__ == "__main__":
	main()
