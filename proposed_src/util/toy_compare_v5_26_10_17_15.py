"""격자 MILP 참조 인스턴스에서 휴리스틱과 학습 정책을 같은 인스턴스로 잰다 (RQ2).

MILP JSON(figures/milp_n3m{M}s{seed}_*.json)을 읽어 같은 시드·크기 인스턴스를 재생하고,
휴리스틱 1회(결정론)·학습 정책 reps회(표본 추출)의 makespan을 기록한다. MILP 값은 격자
제약이 있는 참조 계획이지 연속 최적값이 아니므로, 학습 중앙값이 그보다 빠를 수 있다.
"""

import argparse
import csv
import glob
import json
import os
import sys

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
from pipeline.common_v5_26_09_15_22 import chain_escape_manager, escape_wrap, straight_worker
from pipeline.milp_v5_26_09_21_03 import build_nodes
from util.instance_generator_v5_26_09_15_22 import sample_instance
from util.paired_v5_26_09_27_08 import learned

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def run(env, mf, seed):
	"""고정 CC 인스턴스(MILP와 같은 시드)를 굴려 makespan을 돌려준다."""
	env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=seed, num_drones=env.num_drones,
	                                            comm_range=env.comm_range, cc_pos=None)
	env.reset()
	env.set_goals(mf(env))
	done, t = False, 0
	while not done and t < env.max_steps:
		_o, _w, _r, done = env.step(straight_worker(env))
		t += 1
		if t % 20 == 0 or env.goal_invalid().any():
			env.set_goals(mf(env))
	s = env.episode_stats()
	return s["makespan"] if s["makespan"] else None


def main():
	"""MILP JSON마다 세 방법의 값을 한 행으로 남긴다."""
	p = argparse.ArgumentParser()
	p.add_argument("--pattern", default="milp_n3m*_26_10_17_15.json")
	p.add_argument("--manager", default="weights/v5_26_09_23_18_SWa/best_manager.pth")
	p.add_argument("--reps", type=int, default=30)
	p.add_argument("--out", default="toy_compare_v5_26_10_17_15")
	p.add_argument("--escape", action="store_true", help="학습 정책에 교착 탈출 덮어쓰기를 씌운 열도 함께 잰다")
	p.add_argument("--key", default="milp_steps", help="참조값 키 (MILP: milp_steps, BFS 도 같음)")
	p.add_argument("--lattice-snap", action="store_true",
	               help="중계가 설 수 있는 자리를 참조 모형과 같은 격자 노드로 제한해 이산화 차이를 없앤다")
	A = p.parse_args()
	torch.set_num_threads(2)
	dev = torch.device("cpu")
	lf = learned(A.manager, dev, True)
	rows = []
	for f in sorted(glob.glob(os.path.join(ROOT, "figures", A.pattern))):
		d = json.load(open(f))
		N, M, seed = int(d["drones"]), int(d["dests"]), int(d["seed"])
		env = DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=10000,
		                            deadlock_limit=10 ** 9, no_progress_limit=10 ** 9, num_drones=N, num_dests=M)
		env.reset()
		if A.lattice_snap:
			cc, dests = sample_instance(M, seed=seed, num_drones=N, comm_range=300.0, cc_pos=None)
			ell = env.max_speed * int(d.get("kappa", 10))
			sp = float(d.get("spacing", ell / np.sqrt(2.0)))
			nodes, _nd = build_nodes(cc, dests, ell, sp, 300.0, N, hull_margin=sp / np.sqrt(2.0))
			env.relay_nodes = nodes
		h = run(env, chain_escape_manager(), seed)
		ls = [run(env, lf, seed) for _ in range(A.reps)]
		lv = [x for x in ls if x]
		le = []
		if A.escape:
			lfe = escape_wrap(lf)
			le = [x for x in (run(env, lfe, seed) for _ in range(A.reps)) if x]
		rows.append(dict(drones=N, dests=M, seed=seed,
		                 milp=d.get(A.key), milp_gap=d.get("gap"), milp_bound=d.get("bound_steps"), milp_sec=d.get("solve_sec"),
		                 replay=(d["replay"] or {}).get("makespan") if d.get("replay") else "",
		                 heuristic=h, learned_median=int(np.median(lv)) if lv else "",
		                 learned_q1=int(np.percentile(lv, 25)) if lv else "", learned_q3=int(np.percentile(lv, 75)) if lv else "",
		                 learned_mean=round(float(np.mean(lv)), 1) if lv else "",
		                 learned_min=min(lv) if lv else "", learned_max=max(lv) if lv else "",
		                 learned_complete=f"{len(lv)}/{A.reps}",
		                 learned_std=round(float(np.std(lv)), 1) if lv else "",
		                 esc_median=int(np.median(le)) if le else "", esc_q1=int(np.percentile(le, 25)) if le else "",
		                 esc_q3=int(np.percentile(le, 75)) if le else "", esc_mean=round(float(np.mean(le)), 1) if le else "",
		                 esc_std=round(float(np.std(le)), 1) if le else "", esc_min=min(le) if le else "", esc_max=max(le) if le else "",
		                 esc_complete=f"{len(le)}/{A.reps}" if A.escape else "",
		                 spacing=d.get("spacing", ""), nodes=d.get("nodes", ""), method=d.get("method", "milp")))
		r = rows[-1]
		print(f"M={M:2d} seed {seed}: MILP {r['milp']} (재생 {r['replay']}, gap {r['milp_gap']}) | 휴리스틱 {h} | "
		      f"학습 중앙 {r['learned_median']} ({r['learned_min']}-{r['learned_max']}) 완주 {r['learned_complete']}", flush=True)
	out = os.path.join(ROOT, "figures", f"{A.out}.csv")
	with open(out, "w", newline="", encoding="utf-8") as fh:
		w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
		w.writeheader()
		w.writerows(rows)
	print(f"저장: {out}", flush=True)


if __name__ == "__main__":
	main()
