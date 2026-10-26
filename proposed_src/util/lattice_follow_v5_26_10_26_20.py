"""격자 모드에서 학습 정책·규칙에 '따라가기 중계'(중계 목표 = 선두의 목표)를 씌워 잰다.

따라가기 중계는 실행 계층의 변경이라 어떤 상위 결정 함수에도 씌울 수 있다. 학습 정책은 30회 표본의 중앙값.
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
from pipeline.common_v5_26_09_15_22 import chain_escape_manager, escape_wrap
from pipeline.milp_v5_26_09_21_03 import build_nodes
from util.instance_generator_v5_26_09_15_22 import sample_instance
from util.lattice_eval_v5_26_10_20_17 import hop_table, lattice_rollout
from util.paired_v5_26_09_27_08 import learned
from util.rule2_v5_26_10_26_20 import rule_setter

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--pattern", default="bfs2_n3m*_h98_26_10_21_03.json")
	p.add_argument("--manager", default="weights/v5_26_09_23_18_SWa/best_manager.pth")
	p.add_argument("--reps", type=int, default=30)
	p.add_argument("--out", default="lattice_follow_v5_26_10_26_20")
	A_ = p.parse_args()
	torch.set_num_threads(2)
	lf = learned(A_.manager, torch.device("cpu"), True)
	rows = []
	for f in sorted(glob.glob(os.path.join(ROOT, "figures", A_.pattern))):
		d = json.load(open(f))
		N, M, seed, kappa = int(d["drones"]), int(d["dests"]), int(d["seed"]), int(d["kappa"])
		env = DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=10000,
		                            deadlock_limit=10 ** 9, no_progress_limit=10 ** 9, num_drones=N, num_dests=M)
		if "capacity" in d:
			env.max_capacity = int(d["capacity"])
		env.reset()
		cc, dests = sample_instance(M, seed=seed, num_drones=N, comm_range=300.0, cc_pos=None)
		ell, sp = env.max_speed * kappa, float(d["spacing"])
		pos, _nd = build_nodes(cc, dests, ell, sp, 300.0, N, hull_margin=sp / np.sqrt(2.0))
		Dm = np.linalg.norm(pos[:, None, :] - pos[None, :, :], axis=2)
		A = [np.flatnonzero(Dm[i] <= ell + 1e-6) for i in range(len(pos))]
		hop = hop_table(A, len(pos))
		rf = lattice_rollout(env, rule_setter(chain_escape_manager(), True), seed, pos, A, kappa, hop)
		lv = [x for x in (lattice_rollout(env, rule_setter(lf, True), seed, pos, A, kappa, hop) for _ in range(A_.reps)) if x]
		le = [x for x in (lattice_rollout(env, rule_setter(escape_wrap(lf), True), seed, pos, A, kappa, hop) for _ in range(A_.reps)) if x]
		rows.append(dict(drones=N, dests=M, seed=seed, opt=d["milp_steps"], rule_follow=rf,
		                 learned_follow_median=int(np.median(lv)) if lv else "", learned_follow_min=min(lv) if lv else "",
		                 learned_follow_max=max(lv) if lv else "", learned_follow_complete=f"{len(lv)}/{A_.reps}",
		                 esc_follow_median=int(np.median(le)) if le else "", esc_follow_complete=f"{len(le)}/{A_.reps}"))
		r = rows[-1]
		print(f"M={M} s{seed}: 최적 {d['milp_steps']:.0f} | 규칙+따라가기 {rf} | 학습+따라가기 {r['learned_follow_median']} ({r['learned_follow_complete']}) | 복구+따라가기 {r['esc_follow_median']}", flush=True)
	out = os.path.join(ROOT, "figures", f"{A_.out}.csv")
	with open(out, "w", newline="", encoding="utf-8") as fh:
		w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
	print("저장:", out, flush=True)


if __name__ == "__main__":
	main()
