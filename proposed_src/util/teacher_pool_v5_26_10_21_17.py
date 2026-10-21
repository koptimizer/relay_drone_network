"""교사 풀 생성: 학습 분포의 인스턴스마다 탐색 휴리스틱(ILS)으로 드론별 방문 순서를 구해 저장한다.

학습에서는 이 풀의 인스턴스를 뽑고, 정규화 목표를 규칙 대신 이 순서가 그 상태에서 내릴 옵션으로 둔다.
샤드 번호 w 의 작업자는 i % W == w 인 인스턴스만 맡는다.
"""

import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
from util.metaheur_v5_26_10_20_17 import rollout, search, seed_order

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def spec(i, seed0):
	"""인스턴스 i 의 (N, M, Q, seed). 풀 생성과 학습이 같은 규칙으로 재현한다."""
	r = np.random.default_rng(seed0 + i)
	N = int(r.choice([3, 4]))
	if r.random() < 0.4:                       # 재적재가 강제되는 작은 인스턴스 (RQ2 영역)
		return N, int(r.integers(5, 13)), 2, seed0 + i
	return N, int(r.integers(13, 61)), 5, seed0 + i      # 학습 분포와 같은 영역


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--n-inst", type=int, default=300)
	p.add_argument("--seed0", type=int, default=50001)
	p.add_argument("--iters", type=int, default=200)
	p.add_argument("--shard", type=int, default=0)
	p.add_argument("--shards", type=int, default=1)
	p.add_argument("--out", default="teacher_pool_v5_26_10_21_17")
	A = p.parse_args()
	rows = []
	for i in range(A.n_inst):
		if i % A.shards != A.shard:
			continue
		N, M, Q, seed = spec(i, A.seed0)
		env = DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=6000,
		                            deadlock_limit=10 ** 9, no_progress_limit=10 ** 9, num_drones=N, num_dests=M)
		env.max_capacity = Q
		env.reset()
		t0 = time.time()
		o0 = seed_order(env, seed, None, 10, cc_mode="random")
		r = rollout(env, o0, seed, None, 10, cc_mode="random")
		env.max_steps = 6000
		best, bv, n = search(env, seed, None, 10, A.iters, np.random.default_rng(seed), algo="ils", cc_mode="random")
		rows.append(dict(i=i, N=N, M=M, Q=Q, seed=seed, cc_mode="random", order=[[int(x) for x in o] for o in best],
		                 rule_ms=r, teacher_ms=(None if bv >= 10 ** 6 else int(bv)), evals=n, sec=round(time.time() - t0, 1)))
		print(f"[{i:3d}] N={N} M={M:2d} Q={Q}: 규칙 {r} → 교사 {bv} ({time.time()-t0:.0f}초)", flush=True)
	out = os.path.join(ROOT, "figures", f"{A.out}_shard{A.shard}.json")
	json.dump(rows, open(out, "w"))
	print("저장:", out, flush=True)


if __name__ == "__main__":
	main()
