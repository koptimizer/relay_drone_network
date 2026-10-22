"""작업 열 탐색 휴리스틱: 드론마다 (배송 j / 중계 노드 p, 트리거 j / 재적재) 작업 열을 탐색한다.

방문 순서만 탐색하던 휴리스틱과 달리 중계의 자리와 시점까지 탐색 변수로 둔다. 중계 작업은 노드 p에
서서 수요지 j 가 (누구에 의해서든) 배송될 때까지 머무는 것이다. 실행은 모형과 같은 격자 결정 공간
(기간 이동·경계 연결성·기간 서비스)에서 하므로 모형 최적값이 유효한 하한이다. 초기해는 규칙의
롤아웃에서 읽어 낸 작업 열이고, ILS 로 재배치·교환·중계 삽입/삭제/이동을 탐색한다.
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

from env.disaster_relay_env_v5_26_09_15_22 import (DisasterRelayDroneEnv, GOAL_DELIVER, GOAL_HOLD,
                                                   GOAL_RELAY, GOAL_RETURN)
from pipeline.common_v5_26_09_15_22 import chain_escape_manager
from pipeline.milp_v5_26_09_21_03 import build_nodes
from util.instance_generator_v5_26_09_15_22 import sample_instance
from util.lattice_eval_v5_26_10_20_17 import hop_table, lattice_step, serve_at_boundary

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
D, RL, RE = "D", "R", "L"          # 작업 종류: 배송 / 중계 / 재적재


def setup(env, seed, pos):
	"""인스턴스를 깔고 격자 모드로 초기화한다."""
	env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=seed, num_drones=env.num_drones,
	                                            comm_range=env.comm_range, cc_pos=None)
	env.reset()
	env.no_project = env.no_service = True
	env.relay_nodes = pos
	env.map_size = (float(max(pos.max(), env.map_size[0])) + 10.0,) * 2


def set_goal(env, i, task, pos):
	"""드론 i 의 현재 작업을 하위 목표로 바꾼다."""
	if task is None:
		env.goal_kind[i], env.goal_pos[i], env.goal_dest[i] = GOAL_HOLD, env.drones_pos[i].copy(), -1
	elif task[0] == D:
		env.goal_kind[i], env.goal_pos[i], env.goal_dest[i] = GOAL_DELIVER, env.dests_pos[task[1]].copy(), task[1]
	elif task[0] == RL:
		env.goal_kind[i], env.goal_pos[i], env.goal_dest[i] = GOAL_RELAY, pos[task[1]].copy(), -1
	else:
		env.goal_kind[i], env.goal_pos[i], env.goal_dest[i] = GOAL_RETURN, env.cc_pos.copy(), -1


def advance(env, seqs, ptr, i, pos):
	"""드론 i 의 작업 포인터를 끝난 작업 너머로 옮기고 유효한 현재 작업을 돌려준다."""
	while ptr[i] < len(seqs[i]):
		t = seqs[i][ptr[i]]
		if t[0] == D:
			if not env.dests_active[t[1]]:
				ptr[i] += 1; continue
			if env.drones_capacity[i] == 0:
				return (RE,)                                   # 적재가 없으면 먼저 재적재
			return t
		if t[0] == RL:
			if not env.dests_active[t[2]]:
				ptr[i] += 1; continue
			return t
		if env.drones_capacity[i] == env.max_capacity:
			ptr[i] += 1; continue
		return t
	return None


def rollout(env, seqs, seed, pos, A, hop, kappa, max_periods=400, stall_limit=6):
	"""작업 열을 격자 모드로 실행해 makespan(미완주면 None)을 돌려준다."""
	setup(env, seed, pos)
	N = env.num_drones
	node = np.zeros(N, dtype=int)
	env.drones_pos[:] = pos[node]
	ptr = [0] * N
	stall, p = 0, 0
	while env.makespan is None and p < max_periods:
		for i in range(N):
			set_goal(env, i, advance(env, seqs, ptr, i, pos), pos)
		served = env.dests_active.sum()
		serve_at_boundary(env, node, node, kappa)
		if env.makespan is not None:
			break
		for i in range(N):                      # 서비스로 끝난 작업은 바로 넘긴다
			set_goal(env, i, advance(env, seqs, ptr, i, pos), pos)
		tgt = lattice_step(env, node, pos, A, hop, env.cc_pos, env.comm_range)
		moved = bool(np.any(tgt != node)) or served != env.dests_active.sum() or bool(np.any(env.drones_timer > 0))
		stall = 0 if moved else stall + 1
		if stall >= stall_limit:                 # 교착: 막힌 드론의 현재 작업을 뒤로 돌린다
			for i in range(N):
				if ptr[i] < len(seqs[i]) and env.drones_timer[i] == 0:
					seqs[i].append(seqs[i].pop(ptr[i]))
			stall = 0
		stp = (pos[tgt] - pos[node]) / (kappa * env.max_speed)
		for _ in range(kappa):
			env.step(stp)
		node = tgt
		env.drones_pos[:] = pos[node]
		p += 1
	return env.makespan


def seed_seqs(env, seed, pos, A, hop, kappa, hl_every=20):
	"""규칙+복구를 격자 모드로 굴리며 드론별 작업 열(배송·중계·재적재)을 읽어 낸다."""
	setup(env, seed, pos)
	N, M = env.num_drones, env.num_dests
	mf = chain_escape_manager()
	node = np.zeros(N, dtype=int); env.drones_pos[:] = pos[node]
	env.set_goals(mf(env))
	seqs = [[] for _ in range(N)]
	relay_node = [None] * N
	cap = env.drones_capacity.copy()
	t, p = 0, 0
	while env.makespan is None and t < env.max_steps:
		act0 = env.dests_active.copy()
		serve_at_boundary(env, node, node, kappa)
		for j in np.flatnonzero(act0 & ~env.dests_active):
			i = int(np.argmin(np.linalg.norm(env.drones_pos - env.dests_pos[j], axis=1)))
			seqs[i].append((D, int(j)))
			for k in range(N):                  # 그때 중계 중이던 드론은 j 를 트리거로 하는 중계 작업
				if k != i and relay_node[k] is not None:
					seqs[k].append((RL, int(relay_node[k]), int(j))); relay_node[k] = None
		for i in range(N):
			if env.drones_capacity[i] > cap[i]:
				seqs[i].append((RE,))
		cap = env.drones_capacity.copy()
		if env.makespan is not None:
			break
		tgt = lattice_step(env, node, pos, A, hop, env.cc_pos, env.comm_range)
		for i in range(N):
			if env.goal_kind[i] == GOAL_RELAY:
				relay_node[i] = int(tgt[i])
			elif relay_node[i] is not None and env.goal_kind[i] != GOAL_RELAY:
				relay_node[i] = None
		stp = (pos[tgt] - pos[node]) / (kappa * env.max_speed)
		for _ in range(kappa):
			env.step(stp); t += 1
			if t % hl_every == 0 or env.goal_invalid().any():
				env.set_goals(mf(env))
		node = tgt; env.drones_pos[:] = pos[node]; p += 1
	seen = {j for s in seqs for x in s if x[0] == D for j in [x[1]]}
	for j in range(M):
		if j not in seen:
			seqs[int(np.argmin([len(s) for s in seqs]))].append((D, j))
	return seqs


def neighbour(cur, rng, M, pos, A, cc, R, kind=None):
	"""이웃 하나: 0 배송 재배치, 1 배송 교환, 2 중계 삽입, 3 중계 삭제, 4 중계 노드 이동, 5 트리거 변경."""
	s = [list(x) for x in cur]
	N = len(s)
	k = kind if kind is not None else int(rng.integers(6))
	dl = [(i, q) for i in range(N) for q in range(len(s[i])) if s[i][q][0] == D]
	rl = [(i, q) for i in range(N) for q in range(len(s[i])) if s[i][q][0] == RL]
	if k == 0 and dl:
		i, q = dl[int(rng.integers(len(dl)))]; v = s[i].pop(q); j = int(rng.integers(N))
		s[j].insert(int(rng.integers(len(s[j]) + 1)), v)
	elif k == 1 and len(dl) >= 2:
		(i, q), (i2, q2) = (dl[x] for x in rng.choice(len(dl), 2, replace=False))
		s[i][q], s[i2][q2] = s[i2][q2], s[i][q]
	elif k == 2 and dl:
		i, q = dl[int(rng.integers(len(dl)))]; j = s[i][q][1]
		d = np.linalg.norm(pos[1 + j] - cc)
		n = int(np.ceil(d / (0.9 * R))) - 1
		if n >= 1:
			f = (1 + int(rng.integers(n))) / (n + 1)
			pt = cc + (pos[1 + j] - cc) * f
			p = int(np.argmin(np.linalg.norm(pos - pt, axis=1)))
			k2 = int(rng.integers(N))
			if k2 != i:
				s[k2].insert(int(rng.integers(len(s[k2]) + 1)), (RL, p, int(j)))
	elif k == 3 and rl:
		i, q = rl[int(rng.integers(len(rl)))]; s[i].pop(q)
	elif k == 4 and rl:
		i, q = rl[int(rng.integers(len(rl)))]; _r, p, j = s[i][q]
		s[i][q] = (RL, int(A[p][int(rng.integers(len(A[p])))]), j)
	elif k == 5 and rl:
		i, q = rl[int(rng.integers(len(rl)))]; _r, p, j = s[i][q]
		s[i][q] = (RL, p, int(rng.integers(M)))
	else:
		return neighbour(cur, rng, M, pos, A, cc, R, 0)
	return s


def local(cur, fc, f, rng, budget, ctx):
	"""첫 개선 국소 탐색."""
	n, improved = 0, True
	while improved and n < budget:
		improved = False
		for k in range(6):
			for _ in range(8):
				if n >= budget:
					break
				c = neighbour(cur, rng, *ctx, kind=k); v = f(c); n += 1
				if v < fc:
					cur, fc, improved = c, v, True
					break
	return cur, fc, n


def search(env, seed, pos, A, hop, kappa, iters, rng):
	"""ILS: 국소 탐색 + 3회 교란. (최적 작업 열, 값, 평가 횟수)."""
	M, cc, R = env.num_dests, env.cc_pos.copy(), env.comm_range
	ctx = (M, pos, A, cc, R)
	init = seed_seqs(env, seed, pos, A, hop, kappa)
	cap = {"mp": 400}
	f = lambda s: rollout(env, [list(x) for x in s], seed, pos, A, hop, kappa, max_periods=cap["mp"]) or 10 ** 6
	fc = f(init); cur = [list(x) for x in init]
	if fc < 10 ** 6:
		cap["mp"] = int(np.ceil(2.5 * fc / kappa))        # 나쁜 해를 끝까지 굴리지 않는다
	cur, fc, n = local(cur, fc, f, rng, iters // 4, ctx)
	best, fb = [list(x) for x in cur], fc
	while n < iters:
		sh = [list(x) for x in best]
		for _ in range(3):
			sh = neighbour(sh, rng, *ctx)
		fs = f(sh); n += 1
		sh, fs, m = local(sh, fs, f, rng, max(10, (iters - n) // 4), ctx); n += m
		if fs <= fb:
			best, fb = [list(x) for x in sh], fs
	return best, fb, n, (f(init))


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--pattern", default="bfs2_n3m*_h98_26_10_21_03.json")
	p.add_argument("--iters", type=int, default=1200)
	p.add_argument("--restarts", type=int, default=2)
	p.add_argument("--out", default="taskseq_v5_26_10_22_14")
	A_ = p.parse_args()
	rows = []
	for fjs in sorted(glob.glob(os.path.join(ROOT, "figures", A_.pattern))):
		d = json.load(open(fjs))
		N, M, seed, kappa = int(d["drones"]), int(d["dests"]), int(d["seed"]), int(d["kappa"])
		env = DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=6000,
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
		t0 = time.time(); bv, ev, iv = None, 0, None
		for r in range(A_.restarts):
			_s, v, n, v0 = search(env, seed, pos, A, hop, kappa, A_.iters, np.random.default_rng(2000 + r))
			ev += n; iv = v0
			bv = v if bv is None else min(bv, v)
		sec = time.time() - t0
		rows.append(dict(drones=N, dests=M, seed=seed, opt=d["milp_steps"], seed_val=(iv if iv < 10 ** 6 else ""),
		                 taskseq=(bv if bv < 10 ** 6 else ""), evals=ev, sec=round(sec, 1)))
		print(f"M={M:2d} s{seed}: 최적 {d['milp_steps']:.0f} | 초기(규칙 열) {iv} → 작업열 탐색 {bv} ({ev}회, {sec:.0f}초)", flush=True)
	out = os.path.join(ROOT, "figures", f"{A_.out}.csv")
	with open(out, "w", newline="", encoding="utf-8") as fh:
		w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
	print("저장:", out, flush=True)


if __name__ == "__main__":
	main()
