"""휴리스틱(기하 사슬 + 교착 탈출)의 파라미터 절제 (RQ4, 사이클 26).

탈출 주기 k_stall·유지 hold·탈출 방식 mode·탈출 없음을 격자로 재고, 완주율 우선·완주 중앙 makespan
차선으로 고른다. 튜닝 시드(1001-1040)는 평가 시드(101-, 501-, 701-, 901-)와 겹치지 않는다 —
평가 집합에 맞춰 고르면 시험 집합 튜닝이 된다.
"""

import argparse
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
from pipeline.common_v5_26_09_15_22 import chain_escape_manager, chain_manager, straight_worker
from util.instance_generator_v5_26_09_15_22 import sample_instance

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def run(env, mf, seed, cc, hl_every=20):
	"""한 인스턴스를 굴려 makespan(미완주면 None)을 돌려준다 (평가 규약과 같은 루프)."""
	env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=seed, num_drones=env.num_drones,
	                                            comm_range=env.comm_range, cc_pos=cc)
	env.reset()
	env.set_goals(mf(env))
	done, t = False, 0
	while not done and t < env.max_steps:
		_o, _w, _r, done = env.step(straight_worker(env))
		t += 1
		if t % hl_every == 0 or env.goal_invalid().any():
			env.set_goals(mf(env))
	s = env.episode_stats()
	return s["makespan"] if s["makespan"] else None


def main():
	"""격자의 모든 설정을 한 구성에서 재고 CSV 행으로 남긴다."""
	p = argparse.ArgumentParser()
	p.add_argument("--num-drones", type=int, default=3)
	p.add_argument("--num-dests", type=int, default=30)
	p.add_argument("--random-cc", action="store_true")
	p.add_argument("--seed0", type=int, default=1001)
	p.add_argument("--n", type=int, default=40)
	p.add_argument("--comm-range", type=float, default=300.0)
	p.add_argument("--out", default="heur_ablate_v5_26_10_17_15")
	A = p.parse_args()

	env = DisasterRelayDroneEnv(map_path="", comm_range=A.comm_range, cluster_penalty=False, max_steps=10000,
	                            deadlock_limit=10 ** 9, no_progress_limit=10 ** 9,
	                            num_drones=A.num_drones, num_dests=A.num_dests)
	env.reset()
	cc = "random" if A.random_cc else None
	grid = [("none", 0, 0)]
	for mode in ("shuffle", "retreat"):
		for k in (30, 60, 90, 120):
			for h in (20, 40, 80):
				grid.append((mode, k, h))
	tag = f"d{A.num_drones}m{A.num_dests}{'r' if A.random_cc else 'f'}"
	out = os.path.join(ROOT, "figures", f"{A.out}_{tag}.csv")
	raw = open(out.replace(".csv", "_seeds.csv"), "w", newline="", encoding="utf-8")   # per-seed makespan (유의성 검정용)
	rw = csv.writer(raw)
	rw.writerow(["config", "mode", "k_stall", "hold", "seed", "makespan"])
	with open(out, "w", newline="", encoding="utf-8") as f:
		w = csv.writer(f)
		w.writerow(["config", "mode", "k_stall", "hold", "n", "complete", "median", "mean", "p90", "max"])
		for mode, k, h in grid:
			mf = (lambda e: chain_manager(e)) if mode == "none" else chain_escape_manager(k, h, mode)
			ms = [run(env, mf, s, cc) for s in range(A.seed0, A.seed0 + A.n)]
			for sd, m in zip(range(A.seed0, A.seed0 + A.n), ms):
				rw.writerow([tag, mode, k, h, sd, m if m else ""])
			raw.flush()
			done = [m for m in ms if m]
			row = [tag, mode, k, h, A.n, len(done),
			       int(np.median(done)) if done else "", round(float(np.mean(done)), 1) if done else "",
			       int(np.percentile(done, 90)) if done else "", int(max(done)) if done else ""]
			w.writerow(row)
			f.flush()
			print(f"{tag} {mode:8}{k:4d}{h:4d} | 완주 {len(done):2d}/{A.n} 중앙 {row[6]} 평균 {row[7]} p90 {row[8]} 최대 {row[9]}",
			      flush=True)
	print(f"저장: {out}", flush=True)


if __name__ == "__main__":
	main()
