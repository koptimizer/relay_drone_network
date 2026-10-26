"""빔 탐색 휴리스틱: 시공간 격자 모형의 상태 공간을 기간 단위로 펼치되, 기간마다 점수 상위 B개만 남긴다.

정확 탐색(너비 우선)과 같은 전이 집합(이동 집합 A(i), 기간 양 끝 연결성, 제자리 = 배송/재적재/대기)을
쓰므로 결과는 모형의 실행 가능한 계획이고 최적값보다 작을 수 없다. 상태 점수는 남은 기간 수의 추정치
(임계 경로 + 남은 일의 양/N) 이고, 폭 B 가 커질수록 정확 탐색에 가까워진다. 드론 수 N 은 일반이다.
"""

import argparse
import csv
import glob
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
from pipeline.milp_v5_26_09_21_03 import build_nodes
from util.instance_generator_v5_26_09_15_22 import sample_instance
from util.lattice_eval_v5_26_10_20_17 import hop_table

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def succ_table(A, n, M, Q):
	"""드론 상태 id=(노드*(Q+1)+적재) 마다 (다음 id, 배송 비트) 목록. 수요지에서는 배송/대기 둘 다 허용."""
	K = n * (Q + 1)
	deg = max(len(A[i]) for i in range(n)) + 1
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
				elif 1 <= node <= M and cg > 0:
					nid[i, k], nbit[i, k] = node * (Q + 1) + cg - 1, 1 << (node - 1)
					k += 1
					nid[i, k], nbit[i, k] = i, 0
				elif node == 0 and cg < Q:
					nid[i, k], nbit[i, k] = Q, 0
				else:
					nid[i, k], nbit[i, k] = i, 0
				k += 1
	return nid, nbit, deg, K


def connected_batch(E):
	"""(S, V, V) 인접 행렬 묶음에서 노드 0 이 전체와 이어진 상태를 가린다."""
	V = E.shape[1]
	R = E.copy()
	for _ in range(V - 1):
		R = R | (np.matmul(R.astype(np.uint8), E.astype(np.uint8)) > 0)
	return R[:, 0, :].all(1)


def estimate(ids, mask, H, M, Q, N):
	"""남은 기간 수 추정: 수요지별 최소 도달 기간의 최댓값 + (합 + 미배송 수) / N."""
	node, cargo = ids // (Q + 1), ids % (Q + 1)
	S = len(ids)
	c = H[node[:, :, None], np.arange(1, M + 1)[None, None, :]].astype(np.float32)        # (S, N, M)
	via = (H[node, 0] + 1)[:, :, None] + H[0, 1:M + 1][None, None, :]
	c = np.where((cargo > 0)[:, :, None], c, via)
	dmin = c.min(1)                                                                      # (S, M)
	un = ((mask[:, None] >> np.arange(M)[None, :]) & 1) == 0
	dmin = np.where(un, dmin, 0.0)
	U = un.sum(1)
	est = dmin.max(1) + (dmin.sum(1) + U) / N
	if EST == 2:
		# 재적재 항: 미배송 수가 총 적재량을 넘으면 그만큼 거점 왕복이 더 필요하다
		need = np.maximum(0, U - cargo.sum(1))
		trips = np.ceil(need / Q)
		back = H[0, 1:M + 1][None, :].astype(np.float32)
		mean_back = np.where(U > 0, (np.where(un, back, 0.0)).sum(1) / np.maximum(U, 1), 0.0)
		est = est + trips * (2.0 * mean_back + 1.0) / N
	return est


EST = 1


def search(pos, M, R, ell, Q, N, B, kappa, max_periods, rng, verbose=False):
	"""빔 탐색. (makespan 스텝, 계획 (T+1, N) 노드, 확장 상태 수) 를 돌려준다. 실패하면 None."""
	n = len(pos)
	d = np.linalg.norm(pos[:, None, :] - pos[None, :, :], axis=2)
	A = [np.flatnonzero(d[i] <= ell + 1e-6) for i in range(n)]
	L = d <= R + 1e-6
	H = hop_table(A, n)
	nid, nbit, deg, K = succ_table(A, n, M, Q)
	full = (1 << M) - 1
	total = deg ** N
	digit = np.stack([(np.arange(total) // deg ** (N - 1 - i)) % deg for i in range(N)])   # (N, total)
	ids = np.full((1, N), Q, dtype=np.int64)
	mask = np.zeros(1, dtype=np.int64)
	seen = np.array([_enc(ids, mask, K, M)[0]], dtype=np.int64)
	hist = [dict(ids=ids, parent=np.array([-1]), u=ids.copy())]
	expanded = 0
	chunk = max(1, int(3e6 // total))
	for T in range(1, max_periods + 1):
		parts = []
		for s0 in range(0, len(ids), chunk):
			ci, cm = ids[s0:s0 + chunk], mask[s0:s0 + chunk]
			c = len(ci)
			nx = np.stack([nid[ci[:, i]][:, digit[i]] for i in range(N)], 2)             # (c, total, N)
			bt = np.zeros((c, total), dtype=np.int64)
			for i in range(N):
				bt |= nbit[ci[:, i]][:, digit[i]]
			ok = (nx >= 0).all(2)
			src = np.broadcast_to(np.arange(c)[:, None], (c, total))[ok] + s0
			nx, bt = nx[ok], bt[ok]
			cur = ci[src - s0] // (Q + 1); nn = nx // (Q + 1)
			V = N + 1
			E = np.zeros((len(nx), V, V), dtype=bool)
			E[:, np.arange(V), np.arange(V)] = True
			for i in range(N):
				e = L[0, cur[:, i]] & L[0, nn[:, i]]
				E[:, 0, i + 1] = e; E[:, i + 1, 0] = e
				for j in range(i + 1, N):
					e = L[cur[:, i], cur[:, j]] & L[nn[:, i], nn[:, j]]
					E[:, i + 1, j + 1] = e; E[:, j + 1, i + 1] = e
			ok = connected_batch(E)
			nx, bt, src = nx[ok], bt[ok], src[ok]
			m2 = cm[src - s0] | bt
			srt = np.sort(nx, axis=1)
			code = _enc(srt, m2, K, M)
			code, idx = np.unique(code, return_index=True)
			keep = ~np.isin(code, seen, assume_unique=False)
			idx = idx[keep]
			parts.append((srt[idx], m2[idx], src[idx], nx[idx], code[keep]))
		if not parts or sum(len(q[0]) for q in parts) == 0:
			return None
		sids = np.concatenate([q[0] for q in parts]); sm = np.concatenate([q[1] for q in parts])
		ssrc = np.concatenate([q[2] for q in parts]); su = np.concatenate([q[3] for q in parts])
		scode = np.concatenate([q[4] for q in parts])
		scode, idx = np.unique(scode, return_index=True)
		sids, sm, ssrc, su = sids[idx], sm[idx], ssrc[idx], su[idx]
		expanded += len(sids)
		done = np.flatnonzero(sm == full)
		if len(done):
			g = int(done[0])
			hist.append(dict(ids=sids[g:g + 1], parent=ssrc[g:g + 1], u=su[g:g + 1]))
			return (T - 1) * kappa, _plan(hist, N), expanded
		if len(sids) > B:
			est = np.concatenate([estimate(sids[i:i + 500000], sm[i:i + 500000], H, M, Q, N)
			                      for i in range(0, len(sids), 500000)])
			est = est + rng.random(len(est)).astype(np.float32) * 0.5
			sel = np.argpartition(est, B)[:B]
			sids, sm, ssrc, su, scode = sids[sel], sm[sel], ssrc[sel], su[sel], scode[sel]
		seen = np.union1d(seen, scode)
		hist.append(dict(ids=sids, parent=ssrc, u=su))
		ids, mask = sids, sm
		if verbose:
			print(f"  기간 {T}: 빔 {len(ids)} (누적 확장 {expanded})", flush=True)
	return None


def _enc(ids, mask, K, M):
	"""정렬된 id 행과 비트를 하나의 정수로 부호화한다."""
	code = np.zeros(len(ids), dtype=np.int64)
	for i in range(ids.shape[1]):
		code = code * K + ids[:, i]
	return code * (1 << M) + mask


def _plan(hist, N):
	"""마지막 상태에서 거슬러 올라가 드론별 노드 열 (기간 x 드론) 을 만든다 (정렬 라벨 추적)."""
	T = len(hist) - 1
	idx = 0
	us = []
	for t in range(T, 0, -1):
		us.append(hist[t]["u"][idx].copy())
		idx = int(hist[t]["parent"][idx])
	us = us[::-1]
	Q1 = None
	plan = np.zeros((T + 1, N), dtype=int)
	perm = np.arange(N)
	for t, U in enumerate(us, start=1):
		plan[t] = U[perm]
		order = np.argsort(U, kind="stable")
		inv = np.empty(N, dtype=int); inv[order] = np.arange(N)
		perm = inv[perm]
	return plan


def verify(plan_ids, pos, cc, dests, N, Q, kappa, R):
	"""계획을 연속 환경에서 재생해 배송 수·makespan·단절 수를 확인한다 (용량 Q 반영)."""
	env = DisasterRelayDroneEnv(num_drones=N, num_dests=len(dests), max_steps=10000, cluster_penalty=False,
	                            deadlock_limit=10 ** 9, no_progress_limit=10 ** 9, comm_range=R)
	env.max_capacity = Q
	env.cc_pos, env.dests_pos = np.asarray(cc, dtype=float), np.asarray(dests, dtype=float)
	env.reset()
	done, t = False, 0
	plan = plan_ids // (Q + 1)
	for p in range(1, plan.shape[0]):
		tgt = pos[plan[p]]
		for _ in range(kappa):
			if done:
				break
			vec = tgt - env.drones_pos
			nrm = np.linalg.norm(vec, axis=1, keepdims=True)
			_o, _w, _r, done = env.step(np.where(nrm > 1e-6, vec / np.maximum(nrm, 1e-6), 0.0))
			t += 1
		if done:
			break
	s = env.episode_stats()
	return dict(delivered=s["delivered"], makespan=s["makespan"], comm_loss=s["comm_loss"])


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--pattern", default="bfs2_n3m*_h98_26_10_21_03.json")
	p.add_argument("--widths", default="100,1000,10000")
	p.add_argument("--out", default="beam_v5_26_10_26_20")
	p.add_argument("--seed", type=int, default=0)
	p.add_argument("--est", type=int, default=1, help="상태 점수: 1 임계경로+일의 양, 2 거기에 재적재 왕복 항")
	A_ = p.parse_args()
	global EST
	EST = A_.est
	widths = [int(x) for x in A_.widths.split(",")]
	rows = []
	for fjs in sorted(glob.glob(os.path.join(ROOT, "figures", A_.pattern))):
		d = json.load(open(fjs))
		N, M, seed, kappa = int(d["drones"]), int(d["dests"]), int(d["seed"]), int(d["kappa"])
		env0 = DisasterRelayDroneEnv(num_drones=N, num_dests=M)
		Q = int(d.get("capacity", env0.max_capacity))
		cc, dests = sample_instance(M, seed=seed, num_drones=N, comm_range=300.0, cc_pos=None)
		ell, sp = env0.max_speed * kappa, float(d["spacing"])
		pos, _nd = build_nodes(cc, dests, ell, sp, 300.0, N, hull_margin=sp / np.sqrt(2.0))
		row = dict(drones=N, dests=M, seed=seed, capacity=Q, nodes=len(pos), opt=d["milp_steps"])
		for B in widths:
			t0 = time.time()
			r = search(pos, M, 300.0, ell, Q, N, B, kappa, 120, np.random.default_rng(A_.seed))
			sec = time.time() - t0
			if r is None:
				row[f"beam{B}"], row[f"sec{B}"], row[f"exp{B}"], row[f"chk{B}"] = "", round(sec, 1), "", ""
			else:
				ms, plan, ex = r
				v = verify(plan, pos, cc, dests, N, Q, kappa, 300.0)
				row[f"beam{B}"], row[f"sec{B}"], row[f"exp{B}"] = ms, round(sec, 1), ex
				row[f"chk{B}"] = f"{v['delivered']}/{M} ms{v['makespan']} loss{v['comm_loss']}"
		rows.append(row)
		print(f"M={M} Q={Q} s{seed}: 최적 {d['milp_steps']:.0f} | " +
		      " | ".join(f"B{B} {row[f'beam{B}']} ({row[f'sec{B}']}s, {row[f'chk{B}']})" for B in widths), flush=True)
	out = os.path.join(ROOT, "figures", f"{A_.out}.csv")
	with open(out, "w", newline="", encoding="utf-8") as fh:
		w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
	print("저장:", out, flush=True)


if __name__ == "__main__":
	main()
