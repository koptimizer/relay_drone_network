"""문제 난이도 축을 훑어 규칙 기반 정책이 무너지는 지점을 찾는다.

현재 설정에서는 탐욕이 50/50을 완주하므로 연구가 주장할 영역이 없다.
통신 반경과 드론 수를 줄여가며 (1) 탐욕이 언제 실패하는지, (2) 고정 중계 대수로는
못 맞추고 인스턴스마다 다른 대수가 필요해지는 구간이 있는지를 본다. 후자가 있다면
학습된 적응적 역할 배분이 고정 규칙을 이길 수 있는 영역이다.
"""

import argparse
import csv
import itertools
import os
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MAX_STEPS = 1000


def job(cfg):
	"""한 (통신반경, 드론수, 중계대수) 조합을 여러 인스턴스에서 평가한다."""
	from env.disaster_relay_env_v2_26_08_25_14 import DisasterRelayDroneEnv
	from pipeline.baseline_v2_26_08_25_14 import greedy
	from util.instance_generator_v2_26_08_25_14 import sample_instance
	R, K, nr, seeds = cfg

	env = DisasterRelayDroneEnv()
	env.comm_range = float(R)
	env.num_drones = K
	per = []
	for s in seeds:
		env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=s)
		env.reset()
		done, step = False, 0
		while not done and step < MAX_STEPS:
			_o, _s, _r, done = env.step(greedy(env, nr))
			step += 1
		st = env.episode_stats()
		per.append((st["delivered"], st["makespan"] if st["makespan"] else MAX_STEPS))
	d = [p[0] for p in per]
	return {"comm_range": R, "n_drones": K, "n_relay": nr,
	        "delivered": float(np.mean(d)), "delivered_sd": float(np.std(d)),
	        "full": sum(1 for x in d if x == env.num_dests), "n": len(seeds),
	        "makespan": float(np.mean([p[1] for p in per])),
	        "per_instance": d}


def main():
	"""축을 훑고, 고정 규칙과 인스턴스별 최적 선택의 격차를 계산한다."""
	p = argparse.ArgumentParser()
	p.add_argument("--ranges", nargs="*", type=int, default=[500, 400, 300, 250, 200, 150])
	p.add_argument("--drones", nargs="*", type=int, default=[4])
	p.add_argument("--n", type=int, default=12)
	p.add_argument("--workers", type=int, default=10)
	p.add_argument("--out", default="difficulty_v2_26_08_25_14")
	args = p.parse_args()

	seeds = list(range(101, 101 + args.n))
	tasks = []
	for R, K in itertools.product(args.ranges, args.drones):
		for nr in range(0, K):          # 중계 전용 0 ~ K-1대 (최소 1대는 배송해야 함)
			tasks.append((R, K, nr, seeds))

	print(f"조합 {len(tasks)}개 × 인스턴스 {args.n}개", flush=True)
	with Pool(args.workers) as pool:
		rows = pool.map(job, tasks)

	print(f"\n{'반경':>5} {'드론':>4} | " + "  ".join(f"중계{n}대" for n in range(max(args.drones)))
	      + " |  최선고정  인스턴스별최적  격차")
	print("-" * 96)
	summary = []
	for R, K in itertools.product(args.ranges, args.drones):
		grp = sorted([r for r in rows if r["comm_range"] == R and r["n_drones"] == K],
		             key=lambda r: r["n_relay"])
		cells = "  ".join(f"{g['delivered']:6.1f}" for g in grp)
		best_fixed = max(g["delivered"] for g in grp)
		# 인스턴스마다 가장 좋은 중계 대수를 골랐을 때 (적응적 배분의 상한)
		oracle = float(np.mean([max(g["per_instance"][i] for g in grp)
		                        for i in range(args.n)]))
		gap = oracle - best_fixed
		print(f"{R:5d} {K:4d} | {cells} | {best_fixed:8.1f} {oracle:13.1f} {gap:+7.1f}")
		summary.append({"comm_range": R, "n_drones": K, "best_fixed": round(best_fixed, 2),
		                "oracle_adaptive": round(oracle, 2), "gap": round(gap, 2),
		                "best_n_relay": max(grp, key=lambda g: g["delivered"])["n_relay"]})

	os.makedirs(os.path.join(ROOT, "figures"), exist_ok=True)
	out = os.path.join(ROOT, "figures", f"{args.out}.csv")
	with open(out, "w", newline="", encoding="utf-8") as f:
		w = csv.DictWriter(f, fieldnames=[k for k in rows[0] if k != "per_instance"])
		w.writeheader()
		w.writerows([{k: v for k, v in r.items() if k != "per_instance"} for r in rows])
	print(f"\n저장: {out}")
	print("\n격차(gap)가 큰 구간 = 고정 중계 대수로는 못 맞추고 인스턴스마다 다른 배분이 필요한 구간")
	print("→ 학습된 적응적 역할 배분이 규칙을 이길 수 있는 후보 영역")


if __name__ == "__main__":
	main()
