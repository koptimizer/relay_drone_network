"""탐색 기반 휴리스틱: 드론별 수요지 방문 순서를 모의 담금질로 최적화한다 (규칙·학습과 비교할 제3의 방법).

해는 드론마다의 수요지 우선순위 목록이고, 중계 배정과 교착 복구는 기하 규칙과 같다. 평가는
시뮬레이터 한 번이며, 격자 모드에서는 결정론이라 같은 해가 항상 같은 값을 낸다.
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

from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv, GOAL_RELAY
from pipeline.common_v5_26_09_15_22 import ESC_HOLD, ESC_K, chain_escape_manager, straight_worker
from pipeline.milp_v5_26_09_21_03 import build_nodes
from util.instance_generator_v5_26_09_15_22 import sample_instance
from util.lattice_eval_v5_26_10_20_17 import hop_table, lattice_rollout

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def seq_actions(env, order, st, k_stall=None, hold=None):
	"""우선순위 목록에서 드론별 (옵션, 수요지)를 만든다. 중계·복구는 기하 규칙과 같다."""
	k_stall = ESC_K if k_stall is None else k_stall
	hold = ESC_HOLD if hold is None else hold
	N = env.num_drones
	acts = np.zeros(N, dtype=np.int64)
	dst = -np.ones(N, dtype=np.int64)
	if env.current_step <= 1:
		st["esc"], st["n"] = 0, 0
	if env.deadlock_run >= k_stall and st["esc"] == 0:
		st["esc"], st["n"] = hold, st["n"] + 1
	for i in range(N):
		tgt = next((j for j in order[i] if env.dests_active[j]), -1)
		if env.drones_capacity[i] <= 0 or tgt < 0:
			acts[i] = env.n_cand                      # 복귀
		else:
			acts[i], dst[i] = 0, tgt
	# 중계: 선두가 닿을 거리에 맞춰 필요한 만큼
	if env.dests_active.any():
		reach = np.linalg.norm(env.drones_pos - env.cc_pos, axis=1)
		live = [i for i in range(N) if dst[i] >= 0]
		if live:
			tip = max(live, key=lambda i: reach[i])
			d = max(float(reach[tip]), float(np.linalg.norm(env.dests_pos[dst[tip]] - env.cc_pos)))
			need = min(max(1, int(np.ceil(d / (env.relay_beta * env.comm_range)))) - 1, N - 1)
			for i in [j for j in np.argsort(-reach) if j != tip][:need]:
				acts[i], dst[i] = env.n_cand + 1, -1
			if st["esc"] > 0:
				st["esc"] -= 1
				order_r = list(np.argsort(-reach))
				new_tip = order_r[min(st["n"], len(order_r) - 1)]
				acts[tip], dst[tip] = env.n_cand, -1
				t2 = next((j for j in order[new_tip] if env.dests_active[j]), -1)
				if env.drones_capacity[new_tip] > 0 and t2 >= 0:
					acts[new_tip], dst[new_tip] = 0, t2
	return acts, dst


def rollout(env, order, seed, lat=None, kappa=10, hl_every=20, cc_mode=None):
	"""해 하나를 시뮬레이션해 makespan(미완주면 None)을 돌려준다. lat=(pos, A, hop)면 격자 모드."""
	st = {"esc": 0, "n": 0}
	setter = lambda e: e.set_goals(*seq_actions(e, order, st))
	if lat is not None:
		pos, A, hop = lat
		st["esc"], st["n"] = 0, 0
		return lattice_rollout(env, setter, seed, pos, A, kappa, hop, hl_every)
	env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=seed, num_drones=env.num_drones,
	                                            comm_range=env.comm_range, cc_pos=cc_mode)
	env.reset()
	setter(env)
	done, t = False, 0
	while not done and t < env.max_steps:
		_o, _w, _r, done = env.step(straight_worker(env))
		t += 1
		if not done and (t % hl_every == 0 or env.goal_invalid().any()):
			setter(env)
	s = env.episode_stats()
	return s["makespan"] if s["makespan"] else None


def seed_order(env, seed, lat=None, kappa=10, hl_every=20, cc_mode=None):
	"""규칙+복구를 한 번 굴리며 드론별 배송 순서를 뽑아 초기해로 쓴다."""
	order = [[] for _ in range(env.num_drones)]
	mf = chain_escape_manager()
	prev = {"a": None}

	def setter(e):
		if prev["a"] is not None:
			for j in np.flatnonzero(prev["a"] & ~e.dests_active):
				i = int(np.argmin(np.linalg.norm(e.drones_pos - e.dests_pos[j], axis=1)))
				order[i].append(int(j))
		prev["a"] = e.dests_active.copy()
		e.set_goals(mf(e))

	if lat is not None:
		pos, A, hop = lat
		lattice_rollout(env, setter, seed, pos, A, kappa, hop, hl_every)
	else:
		env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=seed, num_drones=env.num_drones,
		                                            comm_range=env.comm_range, cc_pos=cc_mode)
		env.reset()
		setter(env)
		done, t = False, 0
		while not done and t < env.max_steps:
			_o, _w, _r, done = env.step(straight_worker(env))
			t += 1
			if not done and (t % hl_every == 0 or env.goal_invalid().any()):
				setter(env)
	seen = {j for o in order for j in o}
	for j in range(env.num_dests):
		if j not in seen:
			order[int(np.argmin([len(o) for o in order]))].append(j)
	return order


def _moves(cur, rng, kind=None):
	"""이웃 하나를 만든다: 재배치 / 교환 / 목록 내 구간 뒤집기."""
	cand = [list(x) for x in cur]
	full = [(i, p) for i in range(len(cand)) for p in range(len(cand[i]))]
	if not full:
		return cand
	k = kind if kind is not None else int(rng.integers(3))
	i, p = full[int(rng.integers(len(full)))]
	if k == 0 or len(cand) == 1:                               # 재배치
		j = int(rng.integers(len(cand)))
		v = cand[i].pop(p)
		cand[j].insert(int(rng.integers(len(cand[j]) + 1)), v)
	elif k == 1:                                               # 교환
		j = int(rng.integers(len(cand)))
		if not cand[j]:
			return _moves(cur, rng, 0)
		q = int(rng.integers(len(cand[j])))
		cand[i][p], cand[j][q] = cand[j][q], cand[i][p]
	else:                                                      # 구간 뒤집기 (2-opt)
		if len(cand[i]) < 2:
			return _moves(cur, rng, 0)
		q = int(rng.integers(len(cand[i])))
		lo, hi = min(p, q), max(p, q)
		cand[i][lo:hi + 1] = cand[i][lo:hi + 1][::-1]
	return cand


def _local(cur, fc, f, rng, budget):
	"""첫 개선 국소 탐색: 세 이웃을 번갈아 시도하고 개선이 없으면 멈춘다."""
	n = 0
	improved = True
	while improved and n < budget:
		improved = False
		for k in (0, 1, 2):
			for _ in range(12):
				if n >= budget:
					break
				cand = _moves(cur, rng, k)
				fv = f(cand); n += 1
				if fv < fc:
					cur, fc, improved = cand, fv, True
					break
	return cur, fc, n


def search(env, seed, lat, kappa, iters, rng, algo="sa", t0=1.0, t1=0.02, cc_mode=None):
	"""탐색 기반 휴리스틱. algo: sa(모의 담금질) / vns / ils / grasp. (최적해, 값, 평가 횟수)."""
	init = seed_order(env, seed, lat, kappa, cc_mode=cc_mode)
	f = lambda o: rollout(env, o, seed, lat, kappa, cc_mode=cc_mode) or 10 ** 6
	fc = f(init)
	if fc < 10 ** 6:
		env.max_steps = int(2.5 * fc)        # 나쁜 해를 끝까지 굴리지 않는다 (탐색 속도)
	cur = [list(x) for x in init]
	best, fb, n = [list(x) for x in cur], fc, 1
	if algo == "sa":
		for k in range(iters):
			T = t0 * (t1 / t0) ** (k / max(1, iters - 1))
			cand = _moves(cur, rng)
			fv = f(cand); n += 1
			if fv <= fc or rng.random() < np.exp(-(fv - fc) / max(1e-9, T * max(1.0, fc) * 0.05)):
				cur, fc = cand, fv
				if fv < fb:
					best, fb = [list(x) for x in cand], fv
	elif algo == "vns":
		kmax = 6
		cur, fc, m = _local(cur, fc, f, rng, iters // 4); n += m
		best, fb = [list(x) for x in cur], fc
		kk = 1
		while n < iters:
			sh = [list(x) for x in cur]
			for _ in range(kk):
				sh = _moves(sh, rng)
			fs = f(sh); n += 1
			sh, fs, m = _local(sh, fs, f, rng, max(10, (iters - n) // 4)); n += m
			if fs < fc:
				cur, fc, kk = sh, fs, 1
				if fs < fb:
					best, fb = [list(x) for x in sh], fs
			else:
				kk = kk + 1 if kk < kmax else 1
	elif algo == "ils":
		cur, fc, m = _local(cur, fc, f, rng, iters // 4); n += m
		best, fb = [list(x) for x in cur], fc
		while n < iters:
			sh = [list(x) for x in best]
			for _ in range(3):
				sh = _moves(sh, rng)
			fs = f(sh); n += 1
			sh, fs, m = _local(sh, fs, f, rng, max(10, (iters - n) // 4)); n += m
			if fs <= fb:
				best, fb = [list(x) for x in sh], fs
	elif algo == "grasp":
		N, M = env.num_drones, env.num_dests
		cc, dests = env.cc_pos, env.dests_pos
		while n < iters:
			left = list(range(M)); o = [[] for _ in range(N)]
			last = [cc.copy() for _ in range(N)]
			while left:
				i = int(np.argmin([len(x) for x in o]))
				dd = np.array([np.linalg.norm(dests[j] - last[i]) for j in left])
				rcl = np.argsort(dd)[:max(1, int(np.ceil(0.3 * len(left))))]
				j = left[int(rcl[int(rng.integers(len(rcl)))])]
				o[i].append(j); last[i] = dests[j]; left.remove(j)
			fv = f(o); n += 1
			o, fv, m = _local(o, fv, f, rng, max(10, (iters - n) // 6)); n += m
			if fv < fb:
				best, fb = [list(x) for x in o], fv
	else:
		raise ValueError(algo)
	return best, fb, n


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--pattern", default="bfs2_n3m*_h98_26_10_21_03.json")
	p.add_argument("--iters", type=int, default=400)
	p.add_argument("--restarts", type=int, default=3)
	p.add_argument("--lattice", action="store_true", help="수리모형과 같은 격자 결정 공간에서 평가")
	p.add_argument("--algo", default="sa", choices=["sa", "vns", "ils", "grasp"])
	p.add_argument("--out", default="metaheur_v5_26_10_20_17")
	A_ = p.parse_args()
	rows = []
	for f in sorted(glob.glob(os.path.join(ROOT, "figures", A_.pattern))):
		d = json.load(open(f))
		N, M, seed, kappa = int(d["drones"]), int(d["dests"]), int(d["seed"]), int(d["kappa"])
		env = DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=10000,
		                            deadlock_limit=10 ** 9, no_progress_limit=10 ** 9, num_drones=N, num_dests=M)
		if "capacity" in d:
			env.max_capacity = int(d["capacity"])
		env.reset()
		lat = None
		if A_.lattice:
			cc, dests = sample_instance(M, seed=seed, num_drones=N, comm_range=300.0, cc_pos=None)
			ell, sp = env.max_speed * kappa, float(d["spacing"])
			pos, _nd = build_nodes(cc, dests, ell, sp, 300.0, N, hull_margin=sp / np.sqrt(2.0))
			D = np.linalg.norm(pos[:, None, :] - pos[None, :, :], axis=2)
			Ax = [np.flatnonzero(D[i] <= ell + 1e-6) for i in range(len(pos))]
			lat = (pos, Ax, hop_table(Ax, len(pos)))
		t0 = time.time(); bv, ev = None, 0
		for r in range(A_.restarts):
			_o, v, n = search(env, seed, lat, kappa, A_.iters, np.random.default_rng(1000 + r), algo=A_.algo)
			ev += n
			bv = v if bv is None else min(bv, v)
		sec = time.time() - t0
		rows.append(dict(drones=N, dests=M, seed=seed, opt=d["milp_steps"], algo=A_.algo,
		                 meta=bv if bv < 10 ** 6 else "", evals=ev, sec=round(sec, 1)))
		print(f"M={M:2d} s{seed}: 최적 {d['milp_steps']:.0f} | 탐색 {bv} ({ev}회 평가, {sec:.0f}초)", flush=True)
	out = os.path.join(ROOT, "figures", f"{A_.out}.csv")
	with open(out, "w", newline="", encoding="utf-8") as fh:
		w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
	print("저장:", out, flush=True)


if __name__ == "__main__":
	main()
