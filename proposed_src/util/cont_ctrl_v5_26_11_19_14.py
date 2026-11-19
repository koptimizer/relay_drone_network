"""연속 모형 비교 인스턴스(드론 3, 용량 3, M=9–12, 거점 고정)에서 제어기들을 연속 공간으로 잰다 (사이클 34).

규칙(+복구), 선견 규칙(100스텝마다, 후보 3, 토글 없음), 학습 정책 SWa 와 증류 DA2 (각 30회 표본 중앙값).
결과는 연속 모형의 목적값(하한)과 같은 단위(스텝)로 비교한다.
"""

import argparse
import csv
import multiprocessing as mp
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def run(env, setter, hl=20, max_steps=3000):
	from pipeline.common_v5_26_09_15_22 import straight_worker
	env.reset()
	env.relay_lookahead = False
	setter(env)
	done = False
	while not done and env.current_step < max_steps:
		_o, _w, _r, done = env.step(straight_worker(env))
		if env.current_step % hl == 0 or env.goal_invalid().any():
			setter(env)
	return env.makespan


def job(args):
	M, seed, Q, reps = args
	import torch
	torch.set_num_threads(1)
	from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
	from pipeline.common_v5_26_09_15_22 import chain_escape_manager
	from util.instance_generator_v5_26_09_15_22 import sample_instance
	from util.paired_v5_26_09_27_08 import learned
	from util.rule2_cont_v5_26_10_26_20 import lookahead_cont
	from util.rule2_v5_26_10_26_20 import rule_setter
	cc, dests = sample_instance(M, seed=seed, num_drones=3, comm_range=300.0, cc_pos=None)
	def fresh():
		e = DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=3000,
		                          deadlock_limit=10 ** 9, no_progress_limit=10 ** 9, num_drones=3, num_dests=M)
		e.max_capacity = Q
		e.cc_pos, e.dests_pos = cc.copy(), dests.copy()
		return e
	out = dict(dests=M, seed=seed)
	np.random.seed(seed); torch.manual_seed(seed)
	out["rule"] = run(fresh(), rule_setter(chain_escape_manager(), False))
	out["look"] = run(fresh(), lookahead_cont(20, 3000, k_cand=3, la_every=100, toggles=False))
	for name, w in (("swa", "weights/v5_26_09_23_18_SWa/best_manager.pth"), ("da2", "weights/v5_26_11_09_17_DA2/best_manager.pth")):
		lf = learned(w, torch.device("cpu"), True)
		vals = [run(fresh(), rule_setter(lf, False)) for _ in range(reps)]
		ok = [x for x in vals if x]
		out[name] = int(np.median(ok)) if ok else None
		out[name + "_min"] = min(ok) if ok else None
		out[name + "_complete"] = f"{len(ok)}/{reps}"
	return out


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--spec", default="9:1201,1202,1203,1204,1205;10:1201,1202,1203,1204,1205;11:1201,1202,1203,1204,1205;12:1201,1202,1203,1204,1205")
	p.add_argument("--capacity", type=int, default=3)
	p.add_argument("--reps", type=int, default=30)
	p.add_argument("--workers", type=int, default=10)
	p.add_argument("--out", default="cont_ctrl_v5_26_11_19_14")
	A = p.parse_args()
	jobs = []
	for part in A.spec.split(";"):
		M, seeds = part.split(":")
		jobs += [(int(M), int(s), A.capacity, A.reps) for s in seeds.split(",")]
	with mp.get_context("spawn").Pool(A.workers) as pool:
		rows = pool.map(job, jobs)
	out = os.path.join(ROOT, "figures", f"{A.out}.csv")
	with open(out, "w", newline="") as fh:
		w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
	for r in rows:
		print(r)
	print("저장:", out)


if __name__ == "__main__":
	main()
