"""드론 수 일반 빔 탐색: 드론별 이동 후보를 제한해 (이동 집합)^N 폭발을 피하고, 상태를 행 해시로 중복 제거한다.

전이 의미는 정확해 솔버·빔 탐색(beam_v5)과 같다 (기간 이동, 양 끝 연결성, 제자리 = 배송/재적재/대기).
드론마다 한 기간의 후보 = 제자리 전이(배송·재적재·대기) + 자기 목표(적재 있으면 가장 가까운 미배송, 없으면 거점)
쪽으로 가장 유리한 k 개 + 거점 쪽 1 개. 큰 구성(3/30, 5/50)에서 '격자 상 거의 최적' 기준선을 만들고,
같은 격자 결정 공간에서 규칙과 학습 정책을 함께 잰다.
"""

import argparse
import csv
import os
import sys
import time

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
from pipeline.common_v5_26_09_15_22 import chain_escape_manager, escape_wrap
from pipeline.milp_v5_26_09_21_03 import build_nodes
from util.beam_v5_26_10_26_20 import connected_batch, succ_table, verify
from util.instance_generator_v5_26_09_15_22 import sample_instance
from util.lattice_eval_v5_26_10_20_17 import hop_table, run as lat_run
from util.paired_v5_26_09_27_08 import learned

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def estimate(ids, mask, H, M, Q, N):
	"""남은 기간 추정 (beam_v5 의 est2 와 같다)."""
	node, cargo = ids // (Q + 1), ids % (Q + 1)
	c = H[node[:, :, None], np.arange(1, M + 1)[None, None, :]].astype(np.float32)
	via = (H[node, 0] + 1)[:, :, None] + H[0, 1:M + 1][None, None, :]
	c = np.where((cargo > 0)[:, :, None], c, via)
	dmin = c.min(1)
	un = ((mask[:, None] >> np.arange(M)[None, :]) & 1) == 0
	dmin = np.where(un, dmin, 0.0)
	U = un.sum(1)
	est = dmin.max(1) + (dmin.sum(1) + U) / N
	need = np.maximum(0, U - cargo.sum(1))
	trips = np.ceil(need / Q)
	back = H[0, 1:M + 1][None, :].astype(np.float32)
	mean_back = np.where(U > 0, (np.where(un, back, 0.0)).sum(1) / np.maximum(U, 1), 0.0)
	return est + trips * (2.0 * mean_back + 1.0) / N


def move_choices(ids, mask, nid, nbit, H, M, Q, N, k):
	"""상태·드론별 후보 이동 id 표 (S, N, k+3). 빈 자리는 -1."""
	S = len(ids)
	node, cargo = ids // (Q + 1), ids % (Q + 1)
	un = ((mask[:, None] >> np.arange(M)[None, :]) & 1) == 0                       # (S, M)
	out = -np.ones((S, N, k + 3), dtype=np.int64)
	for i in range(N):
		cand = nid[ids[:, i]]                                                      # (S, deg)
		cn = np.where(cand >= 0, cand // (Q + 1), 0)
		# 목표 거리: 적재 있으면 가장 가까운 미배송 수요지, 없으면 거점
		dj = H[cn[:, :, None], np.arange(1, M + 1)[None, None, :]].astype(np.float32)   # (S, deg, M)
		dj = np.where(un[:, None, :], dj, 1e6).min(2)
		d0 = H[cn, 0].astype(np.float32)
		tgt = np.where((cargo[:, i] > 0)[:, None], dj, d0)
		tgt = np.where(cand >= 0, tgt, 1e9)
		stay = (cn == node[:, i][:, None]) & (cand >= 0)                              # 제자리 전이 (배송/재적재/대기)
		order = np.argsort(tgt + np.where(stay, 1e8, 0.0), axis=1)[:, :k]               # 목표 쪽 k 개
		out[:, i, :k] = np.take_along_axis(np.where(cand >= 0, cand, -1), order, 1)
		st_idx = np.argsort(np.where(stay, 0.0, 1.0) + np.arange(cand.shape[1])[None, :] * 1e-3, axis=1)[:, :2]
		st_val = np.take_along_axis(cand, st_idx, 1)
		st_ok = np.take_along_axis(stay, st_idx, 1)
		out[:, i, k:k + 2] = np.where(st_ok, st_val, -1)
		home = np.argmin(np.where(cand >= 0, d0, 1e9), axis=1)
		out[:, i, k + 2] = cand[np.arange(S), home]
	# 같은 드론 안의 중복 제거 (뒤에 나온 중복은 -1)
	for i in range(N):
		row = out[:, i, :]
		for a in range(row.shape[1]):
			for b in range(a):
				row[:, a] = np.where((row[:, a] == row[:, b]) & (row[:, a] >= 0), -1, row[:, a])
	return out


def search(pos, M, R, ell, Q, N, B, k, kappa, max_periods, rng, verbose=False):
	"""빔 탐색. (makespan 스텝, 계획 (T+1, N) id, 확장 수) 또는 None."""
	n = len(pos)
	d = np.linalg.norm(pos[:, None, :] - pos[None, :, :], axis=2)
	A = [np.flatnonzero(d[i] <= ell + 1e-6) for i in range(n)]
	L = d <= R + 1e-6
	H = hop_table(A, n)
	nid, nbit, deg, K = succ_table(A, n, M, Q)
	full = (1 << M) - 1
	W = k + 3
	total = W ** N
	digit = np.stack([(np.arange(total) // W ** (N - 1 - i)) % W for i in range(N)])
	ids = np.full((1, N), Q, dtype=np.int64); mask = np.zeros(1, dtype=np.int64)
	seen = {_key(ids[0], int(mask[0]))}
	hist = [dict(ids=ids, parent=np.array([-1]), u=ids.copy())]
	expanded = 0
	chunk = max(1, int(2e6 // total))
	for T in range(1, max_periods + 1):
		parts = []
		for s0 in range(0, len(ids), chunk):
			ci, cm = ids[s0:s0 + chunk], mask[s0:s0 + chunk]
			c = len(ci)
			ch = move_choices(ci, cm, nid, nbit, H, M, Q, N, k)                      # (c, N, W)
			nx = np.stack([ch[np.arange(c)[:, None], i, digit[i][None, :]] for i in range(N)], 2)   # (c, total, N)
			ok = (nx >= 0).all(2)
			src = np.broadcast_to(np.arange(c)[:, None], (c, total))[ok] + s0
			nx = nx[ok]
			cur = ci[src - s0]
			# 배송 비트: 제자리 전이 중 배송인 것 (succ 표에서 찾는다)
			bt = np.zeros(len(nx), dtype=np.int64)
			for i in range(N):
				cand = nid[cur[:, i]]; cb = nbit[cur[:, i]]
				hit = (cand == nx[:, i][:, None])
				bt |= np.where(hit, cb, 0).max(1)
			curn, nn = cur // (Q + 1), nx // (Q + 1)
			V = N + 1
			E = np.zeros((len(nx), V, V), dtype=bool)
			E[:, np.arange(V), np.arange(V)] = True
			for i in range(N):
				e = L[0, curn[:, i]] & L[0, nn[:, i]]
				E[:, 0, i + 1] = e; E[:, i + 1, 0] = e
				for j in range(i + 1, N):
					e = L[curn[:, i], curn[:, j]] & L[nn[:, i], nn[:, j]]
					E[:, i + 1, j + 1] = e; E[:, j + 1, i + 1] = e
			ok = connected_batch(E)
			nx, bt, src = nx[ok], bt[ok], src[ok]
			m2 = cm[src - s0] | bt
			srt = np.sort(nx, axis=1)
			parts.append((srt, m2, src, nx))
		if not parts or sum(len(q[0]) for q in parts) == 0:
			return None
		sids = np.concatenate([q[0] for q in parts]); sm = np.concatenate([q[1] for q in parts])
		ssrc = np.concatenate([q[2] for q in parts]); su = np.concatenate([q[3] for q in parts])
		rows_ = np.concatenate([sids, sm[:, None]], 1)
		_u, idx = np.unique(rows_, axis=0, return_index=True)
		keep = np.array([i for i in idx if _key(sids[i], int(sm[i])) not in seen], dtype=np.int64)
		if len(keep) == 0:
			return None
		sids, sm, ssrc, su = sids[keep], sm[keep], ssrc[keep], su[keep]
		expanded += len(sids)
		done = np.flatnonzero(sm == full)
		if len(done):
			g = int(done[0])
			hist.append(dict(ids=sids[g:g + 1], parent=ssrc[g:g + 1], u=su[g:g + 1]))
			return (T - 1) * kappa, _plan(hist, N), expanded
		if len(sids) > B:
			est = np.concatenate([estimate(sids[i:i + 200000], sm[i:i + 200000], H, M, Q, N)
			                      for i in range(0, len(sids), 200000)])
			est = est + rng.random(len(est)).astype(np.float32) * 0.5
			sel = np.argpartition(est, B)[:B]
			sids, sm, ssrc, su = sids[sel], sm[sel], ssrc[sel], su[sel]
		for i in range(len(sids)):
			seen.add(_key(sids[i], int(sm[i])))
		hist.append(dict(ids=sids, parent=ssrc, u=su))
		ids, mask = sids, sm
		if verbose:
			print(f"  기간 {T}: 빔 {len(ids)} (누적 확장 {expanded}) 배송 {int(bin(int(sm.max())).count('1'))}", flush=True)
	return None


def _key(row, m):
	return (tuple(int(x) for x in row), m)


def _plan(hist, N):
	T = len(hist) - 1
	idx = 0; us = []
	for t in range(T, 0, -1):
		us.append(hist[t]["u"][idx].copy()); idx = int(hist[t]["parent"][idx])
	us = us[::-1]
	plan = np.zeros((T + 1, N), dtype=int); perm = np.arange(N)
	for t, U in enumerate(us, start=1):
		plan[t] = U[perm]
		order = np.argsort(U, kind="stable"); inv = np.empty(N, dtype=int); inv[order] = np.arange(N); perm = inv[perm]
	return plan


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--num-drones", type=int, default=3)
	p.add_argument("--num-dests", type=int, default=30)
	p.add_argument("--seed0", type=int, default=501)
	p.add_argument("--n", type=int, default=5)
	p.add_argument("--random-cc", action="store_true")
	p.add_argument("--width", type=int, default=2000)
	p.add_argument("--k", type=int, default=3)
	p.add_argument("--kappa", type=int, default=10)
	p.add_argument("--max-periods", type=int, default=300)
	p.add_argument("--manager", default="weights/v5_26_09_23_18_SWa/best_manager.pth")
	p.add_argument("--reps", type=int, default=5)
	p.add_argument("--no-learned", action="store_true")
	p.add_argument("--out", default="beam_n_v5_26_10_27_00")
	p.add_argument("--verbose", action="store_true")
	A_ = p.parse_args()
	torch.set_num_threads(2)
	lf = None if A_.no_learned else escape_wrap(learned(A_.manager, torch.device("cpu"), True))
	rows = []
	N, M = A_.num_drones, A_.num_dests
	for seed in range(A_.seed0, A_.seed0 + A_.n):
		env = DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=10000,
		                            deadlock_limit=10 ** 9, no_progress_limit=10 ** 9, num_drones=N, num_dests=M)
		env.reset()
		Q = env.max_capacity
		cc, dests = sample_instance(M, seed=seed, num_drones=N, comm_range=300.0, cc_pos="random" if A_.random_cc else None)
		ell = env.max_speed * A_.kappa; sp = ell / np.sqrt(2.0)
		pos, _nd = build_nodes(cc, dests, ell, sp, 300.0, N, hull_margin=sp / np.sqrt(2.0))
		Dm = np.linalg.norm(pos[:, None, :] - pos[None, :, :], axis=2)
		A = [np.flatnonzero(Dm[i] <= ell + 1e-6) for i in range(len(pos))]
		hop = hop_table(A, len(pos))
		t0 = time.time()
		r = search(pos, M, 300.0, ell, Q, N, A_.width, A_.k, A_.kappa, A_.max_periods, np.random.default_rng(0), A_.verbose)
		bsec = time.time() - t0
		if r is None:
			bm, chk = None, ""
		else:
			bm, plan, ex = r
			v = verify(plan, pos, cc, dests, N, Q, A_.kappa, 300.0)
			chk = f"{v['delivered']}/{M} ms{v['makespan']} loss{v['comm_loss']}"
		# 같은 격자 결정 공간에서 규칙(+복구)과 학습(+복구)
		setter = lambda e, s=seed: None
		env.cc_pos, env.dests_pos = cc, dests
		def run_fixed(mf):
			env.cc_pos, env.dests_pos = cc, dests
			env.reset(); env.no_project = env.no_service = True; env.relay_nodes = pos
			env.map_size = (float(max(pos.max(), env.map_size[0])) + 10.0,) * 2
			from util.lattice_eval_v5_26_10_20_17 import lattice_rollout
			return lattice_rollout(env, lambda e: e.set_goals(mf(e)), seed, pos, A, A_.kappa, hop) if not A_.random_cc else _roll_fixed(env, mf, pos, A, hop, A_.kappa, cc, dests)
		rule = run_fixed(chain_escape_manager())
		lv = [x for x in (run_fixed(lf) for _ in range(A_.reps)) if x] if lf else []
		rows.append(dict(drones=N, dests=M, seed=seed, nodes=len(pos), beam=bm, beam_sec=round(bsec, 1), beam_chk=chk,
		                 rule=rule, learned_median=int(np.median(lv)) if lv else "", learned_complete=f"{len(lv)}/{A_.reps}"))
		print(f"{N}/{M} s{seed}: 노드 {len(pos)} | 빔 {bm} ({bsec:.0f}초, {chk}) | 규칙 {rule} | 학습+복구 중앙 {rows[-1]['learned_median']} ({rows[-1]['learned_complete']})", flush=True)
	out = os.path.join(ROOT, "figures", f"{A_.out}.csv")
	with open(out, "w", newline="", encoding="utf-8") as fh:
		w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
	print("저장:", out, flush=True)


def _roll_fixed(env, mf, pos, A, hop, kappa, cc, dests, hl_every=20):
	"""거점이 무작위인 인스턴스를 격자 모드로 굴린다 (lattice_rollout 과 같되 좌표를 직접 준다)."""
	from util.lattice_eval_v5_26_10_20_17 import lattice_jam, lattice_step, serve_at_boundary
	env.cc_pos, env.dests_pos = cc, dests
	env.reset()
	env.no_project = env.no_service = True; env.relay_nodes = pos
	env.map_size = (float(max(pos.max(), env.map_size[0])) + 10.0,) * 2
	node = np.zeros(env.num_drones, dtype=int); env.drones_pos[:] = pos[0]
	env.set_goals(mf(env)); t = 0; jam_run = 0
	while env.makespan is None and t < env.max_steps:
		serve_at_boundary(env, node, node, kappa)
		if env.makespan is not None:
			break
		tgt = lattice_step(env, node, pos, A, hop, env.cc_pos, env.comm_range)
		jam = lattice_jam(env, node, tgt, pos)
		stp = (pos[tgt] - pos[node]) / (kappa * env.max_speed)
		for _ in range(kappa):
			env.step(stp); t += 1
			jam_run = jam_run + 1 if jam else 0; env.deadlock_run = jam_run
			if t % hl_every == 0 or env.goal_invalid().any():
				env.set_goals(mf(env))
		node = tgt; env.drones_pos[:] = pos[node]
	return env.makespan


if __name__ == "__main__":
	main()
