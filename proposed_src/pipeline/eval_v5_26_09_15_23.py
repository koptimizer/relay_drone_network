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

from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
from model.hier_net_v3_26_08_31_19 import ManagerActor, WorkerActor
from model.hier_net_v5_26_09_15_23 import SetManagerActor
from pipeline.common_v5_26_09_15_22 import (action_mask, chain_manager,
	hold_relay_manager, rule_manager, straight_worker)
from util.instance_generator_v5_26_09_15_22 import sample_instance

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MASK_KW = {}
CC_MODE = None





def rollout(env, worker_fn, manager_fn, hl_every, seed):
	"""한 인스턴스를 굴리고 지표를 반환한다."""
	env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=seed, num_drones=env.num_drones,
	                                            comm_range=env.comm_range, cc_pos=CC_MODE)
	env.reset()
	env.set_goals(manager_fn(env))
	done, t = False, 0
	while not done and t < env.max_steps:
		_o, _wr, _tr, done = env.step(worker_fn(env))
		t += 1
		if t % hl_every == 0 or env.goal_invalid().any():
			env.set_goals(manager_fn(env))
	return env.episode_stats()


def learned_worker(ckpt, obs_dim, dev, stoch=False):
	"""학습된 하위 정책을 결정론적 제어 함수로 감싼다."""
	w = WorkerActor(obs_dim).to(dev)
	w.load_state_dict(torch.load(ckpt, map_location=dev))
	w.eval()

	def fn(env):
		with torch.no_grad():
			a, _ = w(torch.as_tensor(env.worker_obs(), dtype=torch.float32, device=dev),
			         deterministic=not stoch)
		return a.cpu().numpy()
	return fn


def set_manager(ckpt, dev, stoch=False):
	"""집합 기반 상위 정책(드론 수 무관)을 할당 함수로 감싼다."""
	m = SetManagerActor().to(dev)
	m.load_state_dict(torch.load(ckpt, map_location=dev))
	m.eval()

	def fn(env):
		o = {k: torch.as_tensor(v, device=dev).unsqueeze(0) for k, v in env.manager_set_obs().items()}
		with torch.no_grad():
			act, probs, _ = m(o, action_mask(env, dev, **MASK_KW).unsqueeze(0))
		return (act if stoch else probs.argmax(-1))[0].cpu().numpy()
	return fn


def learned_manager(ckpt, obs_dim, n_act, dev, stoch=False):
	"""학습된 상위 정책을 결정론적 할당 함수로 감싼다."""
	m = ManagerActor(obs_dim, n_act).to(dev)
	m.load_state_dict(torch.load(ckpt, map_location=dev))
	m.eval()

	def fn(env):
		with torch.no_grad():
			act, probs, _ = m(torch.as_tensor(env.manager_obs(), dtype=torch.float32, device=dev),
			                 action_mask(env, dev, **MASK_KW))
		return (act if stoch else probs.argmax(-1)).cpu().numpy()
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
	p.add_argument("--out", default="eval_v5_26_09_15_23")
	p.add_argument("--max-steps", type=int, default=10000, help="롤아웃 스텝 상한")
	p.add_argument("--no-progress-limit", type=int, default=10**9,
	               help="배송 증가 없이 견딜 스텝 — makespan 측정 시 크게 줄 것")
	p.add_argument("--deadlock-limit", type=int, default=10**9)
	p.add_argument("--arch", choices=["mlp", "set"], default="set", help="상위 정책 구조")
	p.add_argument("--num-drones", type=int, default=4)
	p.add_argument("--num-dests", type=int, default=50)
	p.add_argument("--random-cc", action="store_true", help="제어 센터 위치를 시드마다 무작위로")
	p.add_argument("--commit", action="store_true", help="목표 유지 마스크 (학습에 켠 경우에만)")
	p.add_argument("--no-relay", action="store_true", help="중계 열 차단 (통제군 평가용)")
	p.add_argument("--straight-worker", action="store_true",
	               help="학습 하위 대신 직진 제어를 쓴다 — 직진으로 학습한 상위를 맞춰 평가할 때")
	p.add_argument("--stochastic", action="store_true",
	               help="추론을 argmax 대신 확률 샘플링으로 — 교착 탈출 경로를 남긴다")
	args = p.parse_args()

	dev = torch.device("cpu")
	env = DisasterRelayDroneEnv(comm_range=args.comm_range,
	                            cluster_penalty=not args.no_cluster_penalty,
	                            stuck_obs=not args.no_stuck_obs,
	                            max_steps=args.max_steps,
	                            deadlock_limit=args.deadlock_limit,
	                            no_progress_limit=args.no_progress_limit,
	                            num_drones=args.num_drones, num_dests=args.num_dests)
	# 학습 조건과 평가 조건을 일치시킨다 — v4에서 이 불일치로 결론이 세 번 뒤집혔다.
	global MASK_KW, CC_MODE
	CC_MODE = "random" if args.random_cc else None
	MASK_KW = dict(no_relay=args.no_relay, commit=args.commit)
	env.reset()
	wo, mo, K = env.worker_obs().shape[-1], env.manager_obs().shape[-1], env.n_cand + 2

	# v3의 '중계 n대 고정'은 목표가 현 위치 유지라 사실상 편대 축소였다. v4에서는 같은
	# 행동이 기하 슬롯으로 해석되므로 진짜 중계 베이스라인이 되고, 여기에 필요한 만큼만
	# 중계를 쓰는 기하 규칙을 더해 베이스라인을 과소평가하지 않도록 한다.
	methods = [
		("규칙상위 + 직진하위 (= greedy)", straight_worker, rule_manager),
		("기하 중계 규칙 (무학습)", straight_worker, chain_manager),
		("규칙상위 + 직진 (중계1대 고정)", straight_worker, lambda e: hold_relay_manager(e, 1)),
		("규칙상위 + 직진 (중계2대 고정)", straight_worker, lambda e: hold_relay_manager(e, 2)),
	]
	wlist = args.worker if args.worker else [""] * len(args.manager)
	for i, wck in enumerate(wlist):
		name = os.path.basename(os.path.dirname(wck if wck else args.manager[i])).replace("v3_26_08_31_19_", "")
		wf = (straight_worker if args.straight_worker
		      else learned_worker(os.path.join(ROOT, wck), wo, dev, args.stochastic))
		if i < len(args.manager) and args.manager[i]:
			mf = (set_manager(os.path.join(ROOT, args.manager[i]), dev, args.stochastic) if args.arch == "set"
			      else learned_manager(os.path.join(ROOT, args.manager[i]), mo, K, dev, args.stochastic))
			low = "직진하위" if args.straight_worker else "학습하위"
			methods.append((f"학습상위 + {low} ({name})", wf, mf))
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
			"full_pct": round(100.0 * len(done) / len(st), 1),
			"makespan": round(float(np.mean(done)), 0) if done else "-",
			"mk_sd": round(float(np.std(done)), 0) if done else "-",
		})
		print(f"  {name}: {rows[-1]['delivered']}/50", flush=True)

	cols = list(rows[0].keys())
	w = {c: max(len(c), max(len(str(r[c])) for r in rows)) for c in cols}
	print(f"\n홀드아웃 {args.n}개 × {args.reps}회 = 롤아웃 {args.n * args.reps}개 "
	      f"(시드 {args.seed0}~{args.seed0 + args.n - 1}), 반경 {args.comm_range:.0f}, "
	      f"드론 {args.num_drones} 목적지 {args.num_dests} CC={'무작위' if args.random_cc else '고정'}\n")
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
