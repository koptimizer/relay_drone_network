"""격자 모드 평가: 제어기를 수리모형과 같은 결정 공간(노드 위치·기간 이동·기간 서비스)에서 돌린다.

드론은 기간 경계마다 A(i) 안의 노드로만 이동하고, 연결성은 기간 경계에서만 판정하며, 서비스는 한
기간을 쓴다. 따라서 모형의 최적값이 이 공간에서 달성 가능한 최선이고 제어기는 그보다 빠를 수 없다.
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
from util.paired_v5_26_09_27_08 import learned

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def connected(pts, cc, R):
	"""제어 거점과 모든 드론이 반경 R 그래프에서 이어져 있는지."""
	P = np.vstack([cc[None, :], pts])
	D = np.linalg.norm(P[:, None, :] - P[None, :, :], axis=2) <= R + 1e-6
	seen = {0}; stack = [0]
	while stack:
		u = stack.pop()
		for v in np.flatnonzero(D[u]):
			if v not in seen:
				seen.add(int(v)); stack.append(int(v))
	return len(seen) == len(P)


def hop_table(A, n):
	"""이동 그래프에서 노드 쌍 사이 최소 기간 수 (너비 우선)."""
	INF = 10 ** 6
	D = np.full((n, n), INF, dtype=np.int32)
	for s0 in range(n):
		D[s0, s0] = 0
		q = [s0]
		while q:
			u = q.pop(0)
			for v in A[u]:
				if D[s0, v] > D[s0, u] + 1:
					D[s0, v] = D[s0, u] + 1
					q.append(int(v))
	return D


def goal_node(env, i, pos):
	"""드론 i의 목표를 가장 가까운 노드 색인으로 바꾼다."""
	return int(np.argmin(np.linalg.norm(pos - env.goal_pos[i], axis=1)))


def lattice_step(env, node, pos, A, hop, cc, R):
	"""한 기간의 공동 이동: 연결을 지키면서 목표까지 남은 기간 수의 합을 최소화하는 조합.

	각 드론이 자기 목표로 가는 최단 격자 경로를 따르되, 연결 제약으로 충돌하면 조합 전체에서
	가장 적게 손해 보는 쪽을 고른다. 이동 계층이 병목이 되지 않게 하는 것이 목적이다.
	"""
	N = env.num_drones
	gn = [goal_node(env, i, pos) for i in range(N)]
	opts = []
	for i in range(N):
		if env.drones_timer[i] > 0:
			opts.append([int(node[i])])
		else:
			c = [int(v) for v in A[node[i]]]
			c.sort(key=lambda v: (int(hop[v, gn[i]]), float(np.linalg.norm(pos[v] - env.goal_pos[i]))))
			opts.append(c[:6])        # 목표에 가까운 6개면 충분하다 (조합 216개)
	best, bs = None, None
	for a0 in opts[0]:
		for b0 in (opts[1] if N > 1 else [0]):
			for c0 in (opts[2] if N > 2 else [0]):
				t = np.array([a0, b0, c0][:N], dtype=int)
				if not connected(pos[t], cc, R):
					continue
				sc = sum(int(hop[t[i], gn[i]]) for i in range(N))
				if bs is None or sc < bs:
					best, bs = t, sc
	return best if best is not None else np.asarray(node, dtype=int)


def resolve(tgt, cur, pos, cc, R):
	"""목표 노드 배치가 끊기면 드론을 하나씩 제자리로 되돌려 연결을 회복한다."""
	t = tgt.copy()
	movers = [i for i in range(len(t)) if t[i] != cur[i]]
	while movers and not connected(pos[t], cc, R):
		best, bt = None, None
		for i in movers:
			q = t.copy(); q[i] = cur[i]
			if connected(pos[q], cc, R):
				best, bt = i, q
				break
		if best is None:
			best = movers[0]; bt = t.copy(); bt[best] = cur[best]
		t = bt; movers.remove(best)
	return t


def serve_at_boundary(env, node, tgt, kappa, force=True):
	"""기간 경계에서만 서비스를 처리한다 (모형과 같이 한 기간을 통째로 쓴다).

	force면 미배송 수요지 노드에 적재를 싣고 서 있는 드론은 반드시 배송하고 그 기간을 머문다.
	이는 모형의 s_jkt=1 과 정확히 같은 행동이므로 하한 성질이 유지된다.
	"""
	nd = env.num_dests
	for i in range(env.num_drones):
		if env.drones_timer[i] > 0 or (not force and tgt[i] != node[i]):
			continue
		j = int(node[i]) - 1
		if 0 <= j < nd and env.dests_active[j] and env.drones_capacity[i] > 0:
			env.dests_active[j] = False
			env.drones_capacity[i] -= 1
			env.drones_timer[i] = kappa
			env.deliveries[i] += 1
			env.t_last_deliv = env.current_step
			if not env.dests_active.any() and env.makespan is None:
				env.makespan = env.current_step
				env.term_reason = "complete"
		elif node[i] == 0 and env.drones_capacity[i] == 0:
			env.drones_capacity[i] = env.max_capacity
			env.drones_timer[i] = kappa
			env.reloads[i] += 1


def lattice_rollout(env, setter, seed, pos, A, kappa, hop=None, hl_every=20):
	"""격자 결정 공간에서 한 인스턴스를 굴린다. setter(env)가 목표를 세운다."""
	env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=seed, num_drones=env.num_drones,
	                                            comm_range=env.comm_range, cc_pos=None)
	env.reset()
	env.no_project = env.no_service = True
	env.relay_nodes = pos
	if hop is None:
		hop = hop_table(A, len(pos))
	node = np.zeros(env.num_drones, dtype=int)
	env.drones_pos[:] = pos[0]
	setter(env)
	t = 0
	while env.makespan is None and t < env.max_steps:
		tgt = lattice_step(env, node, pos, A, hop, env.cc_pos, env.comm_range)
		serve_at_boundary(env, node, tgt, kappa)
		if env.makespan is not None:
			break
		tgt = np.where(env.drones_timer > 0, node, tgt)
		stp = (pos[tgt] - pos[node]) / (kappa * env.max_speed)
		for _ in range(kappa):
			env.step(stp)
			t += 1
			if t % hl_every == 0 or env.goal_invalid().any():
				setter(env)
		node = tgt
		env.drones_pos[:] = pos[node]
	return env.makespan


def run(env, mf, seed, pos, A, kappa, hop=None, hl_every=20):
	"""상위 결정 함수 하나를 격자 모드로 굴려 makespan(미완주면 None)을 돌려준다."""
	return lattice_rollout(env, lambda e: e.set_goals(mf(e)), seed, pos, A, kappa, hop, hl_every)


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--pattern", default="bfs_n3m*_h98_26_10_19_22.json")
	p.add_argument("--manager", default="weights/v5_26_09_23_18_SWa/best_manager.pth")
	p.add_argument("--reps", type=int, default=30)
	p.add_argument("--out", default="lattice_eval_v5_26_10_20_17")
	A_ = p.parse_args()
	torch.set_num_threads(2)
	lf = learned(A_.manager, torch.device("cpu"), True)
	lfe = escape_wrap(lf)
	rows = []
	for f in sorted(glob.glob(os.path.join(ROOT, "figures", A_.pattern))):
		d = json.load(open(f))
		N, M, seed, kappa = int(d["drones"]), int(d["dests"]), int(d["seed"]), int(d["kappa"])
		env = DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=10000,
		                            deadlock_limit=10 ** 9, no_progress_limit=10 ** 9, num_drones=N, num_dests=M)
		env.reset()
		cc, dests = sample_instance(M, seed=seed, num_drones=N, comm_range=300.0, cc_pos=None)
		ell = env.max_speed * kappa
		sp = float(d["spacing"])
		pos, _nd = build_nodes(cc, dests, ell, sp, 300.0, N, hull_margin=sp / np.sqrt(2.0))
		D = np.linalg.norm(pos[:, None, :] - pos[None, :, :], axis=2)
		A = [np.flatnonzero(D[i] <= ell + 1e-6) for i in range(len(pos))]
		hop = hop_table(A, len(pos))
		h = run(env, chain_escape_manager(), seed, pos, A, kappa, hop)
		lv = [x for x in (run(env, lf, seed, pos, A, kappa, hop) for _ in range(A_.reps)) if x]
		le = [x for x in (run(env, lfe, seed, pos, A, kappa, hop) for _ in range(A_.reps)) if x]
		rows.append(dict(drones=N, dests=M, seed=seed, nodes=len(pos), opt=d["milp_steps"], rule=h,
		                 learned_median=int(np.median(lv)) if lv else "",
		                 learned_q1=int(np.percentile(lv, 25)) if lv else "",
		                 learned_q3=int(np.percentile(lv, 75)) if lv else "",
		                 learned_min=min(lv) if lv else "", learned_max=max(lv) if lv else "",
		                 learned_complete=f"{len(lv)}/{A_.reps}",
		                 esc_median=int(np.median(le)) if le else "", esc_complete=f"{len(le)}/{A_.reps}"))
		r = rows[-1]
		print(f"M={M:2d} s{seed}: 최적 {d['milp_steps']:.0f} | 규칙 {h} | 학습 {r['learned_median']} ({r['learned_complete']}) | 복구 {r['esc_median']}", flush=True)
	out = os.path.join(ROOT, "figures", f"{A_.out}.csv")
	with open(out, "w", newline="", encoding="utf-8") as fh:
		w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
	print("저장:", out, flush=True)


if __name__ == "__main__":
	main()
