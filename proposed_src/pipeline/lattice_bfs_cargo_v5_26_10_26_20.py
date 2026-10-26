"""적재량을 상태에 넣은 정확 탐색: 재적재가 강제되는 크기(N*Q < M)에서 시공간 모형을 최적으로 푼다.

상태 = (정렬된 (노드, 적재) 삼중, 배송 완료 비트). 한 기간에 각 드론은 A(i) 안의 노드로 가며,
제자리에 머물면 미배송 수요지에서는 배송(적재 -1), 거점에서는 재적재(적재 Q)가 일어난다. 연결성은
기간 양 끝에서 모두 반경 안인 간선만 인정해 연속 시간 연결성을 보장한다.

26-10-26 수정: 수요지 노드에 적재를 지닌 채 '배송하지 않고 머무는' 전이를 추가했다. 이전 판은
제자리 = 배송만 허용해, 이미 배송된 수요지에 중계로 서 있으려면 적재를 버려야 했다. 모형(s_jkt 는
결정 변수)보다 좁은 공간이므로 그 값은 최적값의 상한이었다. 지금 판은 모형과 같은 공간이다.
"""

import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
from pipeline.lattice_bfs_v5_26_10_19_22 import conn_table, persist_ok
from pipeline.milp_v5_26_09_21_03 import build_nodes, heuristic_makespan
from util.instance_generator_v5_26_09_15_22 import sample_instance

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def succ_table(A, n, M, Q):
	"""드론 상태 id=(노드*(Q+1)+적재) 마다 (다음 id, 배송 비트) 목록을 만든다."""
	K = n * (Q + 1)
	deg = max(len(A[i]) for i in range(n)) + 1          # +1: 수요지에서 배송 없이 머무는 전이
	nid = -np.ones((K, deg), dtype=np.int64)
	nbit = np.zeros((K, deg), dtype=np.int64)
	for node in range(n):
		for cg in range(Q + 1):
			i = node * (Q + 1) + cg
			k = 0
			for v in A[node]:
				v = int(v)
				if v != node:
					nid[i, k], nbit[i, k] = v * (Q + 1) + cg, 0
				elif 1 <= node <= M and cg > 0:            # 제자리 = 배송, 또는 배송 없이 대기
					nid[i, k], nbit[i, k] = node * (Q + 1) + cg - 1, 1 << (node - 1)
					k += 1
					nid[i, k], nbit[i, k] = i, 0
				elif node == 0 and cg < Q:                  # 제자리 = 재적재
					nid[i, k], nbit[i, k] = Q, 0
				else:                                        # 제자리 = 대기
					nid[i, k], nbit[i, k] = i, 0
				k += 1
	return nid, nbit, deg, K


def solve(pos, M, R, ell, Q, max_periods, verbose=True):
	"""최소 기간 수와 계획을 돌려준다 (드론 3대 전용)."""
	n = len(pos)
	d = np.linalg.norm(pos[:, None, :] - pos[None, :, :], axis=2)
	A = [np.flatnonzero(d[i] <= ell + 1e-6) for i in range(n)]
	L = d <= R + 1e-6
	CT = conn_table()
	nid, nbit, deg, K = succ_table(A, n, M, Q)
	full = (1 << M) - 1
	NS = K * K * K
	size = NS * (1 << M)
	if size > 3.0e9:
		raise MemoryError(f"상태 공간 {size/1e9:.1f}G — 노드 {n}, K {K}, M {M}")
	visited = np.zeros(size, dtype=bool)
	enc = lambda a, b, c, m: ((a * K + b) * K + c) * (1 << M) + m
	start = Q                                           # 거점(노드 0) + 적재 Q
	s0 = enc(start, start, start, 0)
	frontier = np.array([s0], dtype=np.int64)
	visited[frontier] = True
	hist = [dict(states=frontier, parent=np.array([-1]), u=np.array([0]))]
	t0 = time.time()
	for T in range(1, max_periods + 1):
		st_all = frontier
		chunk = max(1, int(2.5e7 // (deg ** 3)))
		parts = []
		for q0 in range(0, len(st_all), chunk):
			st = st_all[q0:q0 + chunk]
			m = st % (1 << M); p = st // (1 << M)
			c = p % K; b = (p // K) % K; a = p // (K * K)
			ia = nid[a][:, :, None, None]; ib = nid[b][:, None, :, None]; ic = nid[c][:, None, None, :]
			ba = nbit[a][:, :, None, None]; bb = nbit[b][:, None, :, None]; bc = nbit[c][:, None, None, :]
			ia, ib, ic = np.broadcast_arrays(ia, ib, ic)
			ba, bb, bc = np.broadcast_arrays(ba, bb, bc)
			src = np.broadcast_to(np.arange(len(st))[:, None, None, None] + q0, ia.shape)
			ok = (ia >= 0) & (ib >= 0) & (ic >= 0)
			ia, ib, ic, ba, bb, bc, src = ia[ok], ib[ok], ic[ok], ba[ok], bb[ok], bc[ok], src[ok]
			na, nb_, nc = ia // (Q + 1), ib // (Q + 1), ic // (Q + 1)
			cur = (a[src - q0] // (Q + 1), b[src - q0] // (Q + 1), c[src - q0] // (Q + 1))
			ok = persist_ok(L, cur, (na, nb_, nc), CT)
			ia, ib, ic, ba, bb, bc, src = ia[ok], ib[ok], ic[ok], ba[ok], bb[ok], bc[ok], src[ok]
			m2 = m[src - q0] | ba | bb | bc
			u = (ia * K + ib) * K + ic
			trip = np.sort(np.stack([ia, ib, ic], 1), axis=1)
			code = enc(trip[:, 0], trip[:, 1], trip[:, 2], m2)
			new = ~visited[code]
			code, src, u = code[new], src[new], u[new]
			code, idx = np.unique(code, return_index=True)
			visited[code] = True
			parts.append((code, src[idx], u[idx]))
		if parts:
			code = np.concatenate([x[0] for x in parts]); src = np.concatenate([x[1] for x in parts]); u = np.concatenate([x[2] for x in parts])
			o = np.argsort(code); code, src, u = code[o], src[o], u[o]
		else:
			code = src = u = np.array([], dtype=np.int64)
		frontier = code
		hist.append(dict(states=code, parent=src, u=u))
		done = code[code % (1 << M) == full]
		if verbose:
			print(f"  기간 {T}: 새 상태 {len(code)} ({time.time()-t0:.0f}초)", flush=True)
		if len(done) > 0:
			return T - 1, hist, int(done[0]), K
		if len(code) == 0:
			return None, hist, None, K
	return None, hist, None, K


def path_from(hist, goal, K, Q, N=3):
	"""목표 상태에서 거꾸로 올라가 기간별 드론 노드 열을 만든다."""
	T = len(hist) - 1
	idx = int(np.searchsorted(hist[T]["states"], goal))
	us = []
	for t in range(T, 0, -1):
		us.append(int(hist[t]["u"][idx]))
		idx = int(hist[t]["parent"][idx])
	us = us[::-1]
	plan = np.zeros((T + 1, N), dtype=int)
	perm = np.arange(N)
	for t, uc in enumerate(us, start=1):
		U = np.array([uc // (K * K), (uc // K) % K, uc % K])
		plan[t] = (U[perm]) // (Q + 1)
		order = np.argsort(U, kind="stable")
		inv = np.empty(N, dtype=int); inv[order] = np.arange(N)
		perm = inv[perm]
	return plan


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--num-dests", type=int, default=7)
	p.add_argument("--seed", type=int, default=901)
	p.add_argument("--capacity", type=int, default=2)
	p.add_argument("--comm-range", type=float, default=300.0)
	p.add_argument("--kappa", type=int, default=10)
	p.add_argument("--max-periods", type=int, default=40)
	p.add_argument("--out", default="")
	A_ = p.parse_args()
	N, M, Q = 3, A_.num_dests, A_.capacity
	cc, dests = sample_instance(M, seed=A_.seed, num_drones=N, comm_range=A_.comm_range, cc_pos=None)
	env0 = DisasterRelayDroneEnv(num_drones=N, num_dests=M)
	ell = env0.max_speed * A_.kappa
	sp = ell / np.sqrt(2.0)
	pos, _nd = build_nodes(cc, dests, ell, sp, A_.comm_range, N, hull_margin=sp / np.sqrt(2.0))
	print(f"[격자] 노드 {len(pos)} 간격 {sp:.1f} | 용량 {Q}x{N}={Q*N} < 수요지 {M} → 재적재 필요", flush=True)
	t0 = time.time()
	C, hist, goal, K = solve(pos, M, A_.comm_range, ell, Q, A_.max_periods)
	sec = time.time() - t0
	if C is None:
		print("[결과] 해 없음", flush=True); return
	plan = path_from(hist, goal, K, Q, N)
	print(f"[결과] 최적 {C}기간 = {C*A_.kappa}스텝 ({sec:.0f}초)", flush=True)
	if A_.out:
		res = dict(drones=N, dests=M, seed=A_.seed, kappa=A_.kappa, spacing=round(sp, 1), capacity=Q,
		           nodes=len(pos), method="bfs-cargo-wait", status="optimal", gap=0.0,
		           milp_periods=C, milp_steps=C * A_.kappa, bound_steps=C * A_.kappa, solve_sec=round(sec, 1))
		with open(os.path.join(ROOT, "figures", f"{A_.out}.json"), "w") as f:
			json.dump(res, f, ensure_ascii=False, indent=1)
		np.save(os.path.join(ROOT, "figures", f"{A_.out}_plan.npy"),
		        dict(plan=plan, pos=pos, cc=cc, dests=dests), allow_pickle=True)


if __name__ == "__main__":
	main()
