"""시공간 격자 모형의 정확해를 너비 우선 탐색으로 구한다 (재적재가 필요 없는 M <= Q 인스턴스).

드론이 동일하므로 상태는 (정렬된 위치 다중집합, 배송 완료 비트)이고, 기간마다 도달 가능한 상태
집합을 벡터화해 확장한다. 모든 전이는 MILP와 같은 이동 집합 A(i)·통신 집합 E 를 따르므로 결과는
같은 노드 집합 위 MILP의 최적값과 일치한다 (검증: 시드 902, 10기간).
"""

import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
from pipeline.milp_v5_26_09_21_03 import build_nodes, heuristic_makespan, replay
from util.instance_generator_v5_26_09_15_22 import sample_instance

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def conn_table():
	"""4노드(거점+드론3) 그래프의 6간선 비트 패턴별 연결 여부 표 (64개)."""
	E = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]
	out = np.zeros(64, dtype=bool)
	for m in range(64):
		adj = {i: set() for i in range(4)}
		for k, (u, v) in enumerate(E):
			if m >> k & 1:
				adj[u].add(v); adj[v].add(u)
		seen, st = {0}, [0]
		while st:
			u = st.pop()
			for v in adj[u]:
				if v not in seen:
					seen.add(v); st.append(v)
		out[m] = len(seen) == 4
	return out


def persist_ok(L, cur, nxt, table):
	"""전이가 허용되는가: 기간의 양 끝에서 모두 반경 안인 간선만으로 연결되어야 한다.

	두 드론이 직선으로 움직이면 거리는 시간에 대해 볼록하므로 양 끝에서 R 이하면 기간 내내 R 이하다.
	따라서 이 조건은 연속 시간 연결성을 보장한다 (경계에서만 보는 조건은 보장하지 못한다).
	"""
	a, b, c = cur
	a2, b2, c2 = nxt
	m = (L[0, a] & L[0, a2]).astype(np.int64)
	m |= (L[0, b] & L[0, b2]).astype(np.int64) << 1
	m |= (L[0, c] & L[0, c2]).astype(np.int64) << 2
	m |= (L[a, b] & L[a2, b2]).astype(np.int64) << 3
	m |= (L[a, c] & L[a2, c2]).astype(np.int64) << 4
	m |= (L[b, c] & L[b2, c2]).astype(np.int64) << 5
	return table[m]


def solve(pos, n_dest, R, step_len, max_periods, verbose=True):
	"""BFS 로 모든 목적지가 배송되는 최소 기간 수와 드론별 경로를 돌려준다."""
	n = len(pos)
	d = np.linalg.norm(pos[:, None, :] - pos[None, :, :], axis=2)
	mv = d <= step_len + 1e-6
	deg = int(mv.sum(1).max())               # 격자 8이웃 + 자기 + 정확 노드(센터·목적지)
	nb = -np.ones((n, deg), dtype=np.int64)
	for i in range(n):
		js = np.flatnonzero(mv[i])
		nb[i, :len(js)] = js
	L = d <= R + 1e-6
	CT = conn_table()
	M = n_dest
	full = (1 << M) - 1
	destbit = np.zeros(n, dtype=np.int64)
	destbit[1:M + 1] = 1 << np.arange(M)
	NS = n * n * n
	visited = np.zeros(NS * (1 << M), dtype=bool)
	enc = lambda a, b, c, m: ((a * n + b) * n + c) * (1 << M) + m
	frontier = np.array([enc(0, 0, 0, 0)], dtype=np.int64)
	visited[frontier] = True
	hist = [dict(states=frontier, parent=np.array([-1]), u=np.array([0]))]
	t0 = time.time()
	for T in range(1, max_periods + 1):
		st_all = frontier
		chunk = max(1, int(4e7 // (deg ** 3)))      # 조합 수가 4e7 을 넘지 않게 프런티어를 나눈다
		parts = []
		for s0 in range(0, len(st_all), chunk):
			st = st_all[s0:s0 + chunk]
			m = st % (1 << M); p = st // (1 << M)
			c = p % n; b = (p // n) % n; a = p // (n * n)
			na = nb[a][:, :, None, None]; nbb = nb[b][:, None, :, None]; nc = nb[c][:, None, None, :]
			na, nbb, nc = np.broadcast_arrays(na, nbb, nc)
			src = np.broadcast_to(np.arange(len(st))[:, None, None, None] + s0, na.shape)
			ok = (na >= 0) & (nbb >= 0) & (nc >= 0)
			na, nbb, nc, src = na[ok], nbb[ok], nc[ok], src[ok]
			ok = persist_ok(L, (a[src - s0], b[src - s0], c[src - s0]), (na, nbb, nc), CT)
			na, nbb, nc, src = na[ok], nbb[ok], nc[ok], src[ok]
			m2 = m[src - s0].copy()
			for x, nx in ((a, na), (b, nbb), (c, nc)):
				stay = (x[src - s0] == nx)
				m2 |= np.where(stay, destbit[nx], 0)
			u = (na * n + nbb) * n + nc
			trip = np.sort(np.stack([na, nbb, nc], 1), axis=1)
			code = enc(trip[:, 0], trip[:, 1], trip[:, 2], m2)
			new = ~visited[code]
			code, src, u = code[new], src[new], u[new]
			code, idx = np.unique(code, return_index=True)
			visited[code] = True
			parts.append((code, src[idx], u[idx]))
		if parts:
			code = np.concatenate([q[0] for q in parts]); src = np.concatenate([q[1] for q in parts]); u = np.concatenate([q[2] for q in parts])
			order = np.argsort(code); code, src, u = code[order], src[order], u[order]
		else:
			code = src = u = np.array([], dtype=np.int64)
		frontier = code
		hist.append(dict(states=code, parent=src, u=u))
		done = code[code % (1 << M) == full]
		if verbose:
			print(f"  기간 {T}: 새 상태 {len(code)} ({time.time()-t0:.0f}초)", flush=True)
		if len(done) > 0:
			return T - 1, hist, int(done[0])
		if len(code) == 0:
			return None, hist, None
	return None, hist, None


def path_from(hist, goal, n, M, N=3):
	"""목표 상태에서 거꾸로 올라가 드론별 노드 열(기간 x 드론)을 만든다."""
	T = len(hist) - 1
	idx = int(np.searchsorted(hist[T]["states"], goal))
	us = []
	for t in range(T, 0, -1):
		us.append(int(hist[t]["u"][idx]))
		idx = int(hist[t]["parent"][idx])
	us = us[::-1]
	plan = np.zeros((T + 1, N), dtype=int)
	perm = np.arange(N)                      # 고정 라벨 -> 직전 정렬 슬롯
	for t, uc in enumerate(us, start=1):
		U = np.array([uc // (n * n), (uc // n) % n, uc % n])
		D = U[perm]
		plan[t] = D
		order = np.argsort(U, kind="stable")
		inv = np.empty(N, dtype=int); inv[order] = np.arange(N)
		perm = inv[perm]
	return plan


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--num-drones", type=int, default=3)
	p.add_argument("--num-dests", type=int, default=4)
	p.add_argument("--seed", type=int, default=902)
	p.add_argument("--comm-range", type=float, default=300.0)
	p.add_argument("--kappa", type=int, default=10)
	p.add_argument("--spacing", type=float, default=0.0, help="격자 간격. 0이면 기간 이동거리/sqrt(2)")
	p.add_argument("--hull-margin", type=float, default=-2.0, help="볼록 껍질 여유 (m). -2: 간격/sqrt(2) 자동, -1: 끔")
	p.add_argument("--max-periods", type=int, default=0, help="0이면 휴리스틱 makespan 의 1.5배")
	p.add_argument("--out", default="")
	A = p.parse_args()
	assert A.num_drones == 3, "현재 구현은 드론 3대 전용"
	cc, dests = sample_instance(A.num_dests, seed=A.seed, num_drones=A.num_drones, comm_range=A.comm_range, cc_pos=None)
	env0 = DisasterRelayDroneEnv(num_drones=A.num_drones, num_dests=A.num_dests)
	assert A.num_dests <= 2 * env0.max_capacity, "용량 비추적 완화의 타당성을 확인할 수 있는 크기까지만"
	step_len = env0.max_speed * A.kappa
	spacing = A.spacing if A.spacing > 0 else step_len / np.sqrt(2.0)
	hm = spacing / np.sqrt(2.0) if A.hull_margin == -2.0 else A.hull_margin
	pos, n_dest = build_nodes(cc, dests, step_len, spacing, A.comm_range, A.num_drones, hull_margin=hm)
	hs = heuristic_makespan(cc, dests, A.num_drones, A.comm_range)
	maxp = A.max_periods if A.max_periods > 0 else int(np.ceil(1.5 * (hs["makespan"] or 1200) / A.kappa)) + 2
	print(f"[격자] 간격 {spacing:.1f}m 껍질여유 {hm:.1f} 노드 {len(pos)} | 휴리스틱 {hs['makespan']} | 최대 {maxp}기간", flush=True)
	t0 = time.time()
	C, hist, goal = solve(pos, n_dest, A.comm_range, step_len, maxp)
	sec = time.time() - t0
	if C is None:
		print("[결과] 해 없음", flush=True)
		res = dict(drones=A.num_drones, dests=A.num_dests, seed=A.seed, kappa=A.kappa, spacing=round(spacing, 1),
		           hull_margin=hm, nodes=len(pos), method="bfs", status="infeasible", gap=None, milp_periods=None,
		           milp_steps=None, bound_steps=None, solve_sec=round(sec, 1), rule_makespan=hs["makespan"], replay=None)
	else:
		plan = path_from(hist, goal, len(pos), n_dest, A.num_drones)
		rep = replay(plan, pos, cc, dests, A.num_drones, A.kappa, A.comm_range)
		res = dict(drones=A.num_drones, dests=A.num_dests, seed=A.seed, kappa=A.kappa, spacing=round(spacing, 1),
		           hull_margin=hm, nodes=len(pos), method="bfs", status="optimal", gap=0.0, milp_periods=C,
		           milp_steps=C * A.kappa, bound_steps=C * A.kappa, solve_sec=round(sec, 1),
		           rule_makespan=hs["makespan"], rule_delivered=hs["delivered"], replay=rep)
		print(f"[결과] 최적 {C}기간 = {C*A.kappa}스텝 ({sec:.0f}초) | 재생 배송 {rep['delivered']}/{A.num_dests} makespan {rep['makespan']} "
		      f"단절 {rep['comm_loss']} | 휴리스틱 {hs['makespan']}", flush=True)
	if A.out:
		os.makedirs(os.path.join(ROOT, "figures"), exist_ok=True)
		with open(os.path.join(ROOT, "figures", f"{A.out}.json"), "w") as f:
			json.dump(res, f, ensure_ascii=False, indent=1)
		if C is not None:
			np.save(os.path.join(ROOT, "figures", f"{A.out}_plan.npy"), dict(plan=plan, pos=pos, cc=cc, dests=dests), allow_pickle=True)


if __name__ == "__main__":
	main()
