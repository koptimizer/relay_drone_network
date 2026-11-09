"""정제한 증류 교사(이탈 여유폭·후보당 공통 난수 평균)를 정책으로 굴려 3/30 쌍별 시드에서 잰다 (사이클 32, DA2 의 상한).

매 결정 시점마다 distill 의 expert_label(margin, reps) 로 배정을 고른다. 결과는 SWa·규칙·정제 전 교사(469)와 비교한다.
"""

import argparse
import csv
import multiprocessing as mp
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def job(args):
	seed, N, M, margin, reps = args
	import time
	import torch
	torch.set_num_threads(1)
	from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
	from pipeline.common_v5_26_09_15_22 import chain_escape_manager, straight_worker
	from pipeline.distill_v5_26_11_09_17 import expert_label
	from util.instance_generator_v5_26_09_15_22 import sample_instance
	env = DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=3000,
	                            deadlock_limit=10 ** 9, no_progress_limit=10 ** 9, num_drones=N, num_dests=M)
	env.cc_pos, env.dests_pos = sample_instance(M, seed=seed, num_drones=N, comm_range=300.0, cc_pos="random")
	env.reset()
	env.rng = np.random.default_rng(seed)
	np.random.seed(seed)
	base = chain_escape_manager()
	t0 = time.time()
	env.set_goals(expert_label(env, base, 3, 1000, margin, reps)[0])
	done = False
	while not done and env.current_step < 3000:
		_o, _w, _r, done = env.step(straight_worker(env))
		if not done and (env.current_step % 20 == 0 or env.goal_invalid().any()):
			env.set_goals(expert_label(env, base, 3, 1000, margin, reps)[0])
	return dict(seed=seed, makespan=env.makespan, sec=round(time.time() - t0, 1))


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--num-drones", type=int, default=3)
	p.add_argument("--num-dests", type=int, default=30)
	p.add_argument("--seed0", type=int, default=501)
	p.add_argument("--n", type=int, default=40)
	p.add_argument("--margin", type=float, default=10.0)
	p.add_argument("--reps", type=int, default=2)
	p.add_argument("--workers", type=int, default=2)
	p.add_argument("--out", default="teacher_c32_v5_26_11_09_22")
	A = p.parse_args()
	os.chdir(ROOT)
	with mp.get_context("spawn").Pool(A.workers) as pool:
		rows = pool.map(job, [(s, A.num_drones, A.num_dests, A.margin, A.reps) for s in range(A.seed0, A.seed0 + A.n)])
	out = os.path.join(ROOT, "figures", f"{A.out}.csv")
	with open(out, "w", newline="") as fh:
		w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
	ms = [r["makespan"] for r in rows if r["makespan"]]
	print(f"{A.num_drones}/{A.num_dests} 정제 교사 (여유폭 {A.margin}, 롤아웃 {A.reps}회): 평균 {np.mean(ms):.0f} 완주 {len(ms)}/{len(rows)} | {np.mean([r['sec'] for r in rows]):.0f}초/건 | 저장 {out}")


if __name__ == "__main__":
	main()
