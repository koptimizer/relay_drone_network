"""연속 문제의 하한: 연결성 제약만 완화하고 나머지는 그대로 둔 채 모든 배정·순서를 전수 열거한다.

제약을 하나 빼면 최적값은 커질 수 없으므로 이 값은 연속 최적값의 유효한 하한이다. M<=6, 드론 3대에서
(배정, 순서) 조합은 M! x (조합 수) = 최대 20,160가지뿐이라 정확히 센다. 용량 Q를 넘기면 거점 복귀를
끼워 넣는다. makespan 은 마지막 배송이 시작되는 시각(환경 규약과 동일)이다.
"""

import argparse
import csv
import glob
import json
import os
import sys
from itertools import permutations

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
from util.instance_generator_v5_26_09_15_22 import sample_instance

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def route_time(seq, cc, dests, v, S, Q, ir):
	"""한 드론이 seq 순서로 배송할 때 마지막 배송이 시작되는 시각. 용량 초과 시 거점 복귀를 끼운다."""
	t, p, load = 0.0, cc, Q
	for k, j in enumerate(seq):
		if load == 0:
			t += max(0.0, (np.linalg.norm(cc - p) - ir)) / v + S      # 복귀 + 재적재
			p, load = cc, Q
		t += max(0.0, (np.linalg.norm(dests[j] - p) - ir)) / v
		p, load = dests[j], load - 1
		if k < len(seq) - 1:
			t += S                                                    # 하역 시간 (마지막은 시작 시각만 센다)
	return t


def best(cc, dests, N, v, S, Q, ir):
	"""모든 (배정, 순서)에서 makespan 최솟값. M<=6 에서 전수 열거."""
	M = len(dests)
	bestv = np.inf
	for assign in np.ndindex(*([N] * M)):
		grp = [[j for j in range(M) if assign[j] == i] for i in range(N)]
		tops = []
		for g in grp:
			if not g:
				tops.append([0.0]); continue
			tops.append([route_time(o, cc, dests, v, S, Q, ir) for o in permutations(g)])
		# 드론별로 독립이므로 각자 최소를 쓰면 makespan 최소
		mk = max(min(x) for x in tops)
		bestv = min(bestv, mk)
	return bestv


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--pattern", default="bfs_n3m*_h98_26_10_19_22.json")
	p.add_argument("--out", default="lb_cont_v5_26_10_21_03")
	A = p.parse_args()
	env0 = DisasterRelayDroneEnv(num_drones=3, num_dests=6)
	v, S, Q, ir = env0.max_speed, env0.unload_time, env0.max_capacity, env0.interaction_radius
	rows = []
	for f in sorted(glob.glob(os.path.join(ROOT, "figures", A.pattern))):
		d = json.load(open(f))
		N, M, seed = int(d["drones"]), int(d["dests"]), int(d["seed"])
		cc, dests = sample_instance(M, seed=seed, num_drones=N, comm_range=300.0, cc_pos=None)
		lb = best(cc, dests, N, v, S, Q, ir)
		rows.append(dict(drones=N, dests=M, seed=seed, lb=round(lb, 1), opt=d["milp_steps"],
		                 ratio=round(d["milp_steps"] / lb, 3)))
		print(f"M={M} s{seed}: 연속 하한 {lb:6.1f} | 격자 최적 {d['milp_steps']:5.0f} | 비 {d['milp_steps']/lb:.2f}", flush=True)
	out = os.path.join(ROOT, "figures", f"{A.out}.csv")
	with open(out, "w", newline="", encoding="utf-8") as fh:
		w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
	r = np.array([x["ratio"] for x in rows])
	print(f"저장: {out} | 격자최적/연속하한 평균 {r.mean():.2f} (범위 {r.min():.2f}-{r.max():.2f})", flush=True)


if __name__ == "__main__":
	main()
