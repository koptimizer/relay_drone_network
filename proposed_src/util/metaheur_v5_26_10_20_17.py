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


def rollout(env, order, seed, lat=None, kappa=10, hl_every=20):
	"""해 하나를 시뮬레이션해 makespan(미완주면 None)을 돌려준다. lat=(pos, A, hop)면 격자 모드."""
	st = {"esc": 0, "n": 0}
	setter = lambda e: e.set_goals(*seq_actions(e, order, st))
	if lat is not None:
		pos, A, hop = lat
		st["esc"], st["n"] = 0, 0
		return lattice_rollout(env, setter, seed, pos, A, kappa, hop, hl_every)
	env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=seed, num_drones=env.num_drones,
	                                            comm_range=env.comm_range, cc_pos=None)
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


def seed_order(env, seed, lat=None, kappa=10, hl_every=20):
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
		                                            comm_range=env.comm_range, cc_pos=None)
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


def search(env, seed, lat, kappa, iters, rng, t0=1.0, t1=0.02):
	"""모의 담금질: 재배치·교환 이웃, 지수 냉각. (최적해, 값, 평가 횟수)."""
	cur = seed_order(env, seed, lat, kappa)
	f = lambda o: rollout(env, o, seed, lat, kappa) or 10 ** 6
	fc = f(cur)
	if fc < 10 ** 6:
		env.max_steps = int(2.5 * fc)        # 나쁜 해를 끝까지 굴리지 않는다 (탐색 속도)
	best, fb, n = [list(x) for x in cur], fc, 1
	for k in range(iters):
		T = t0 * (t1 / t0) ** (k / max(1, iters - 1))
		cand = [list(x) for x in cur]
		full = [(i, p) for i in range(len(cand)) for p in range(len(cand[i]))]
		if not full:
			break
		i, p = full[rng.integers(len(full))]
		j = int(rng.integers(len(cand)))
		if rng.random() < 0.5 or len(cand[j]) == 0:          # 재배치
			v = cand[i].pop(p)
			cand[j].insert(int(rng.integers(len(cand[j]) + 1)), v)
		else:                                                 # 교환
			q = int(rng.integers(len(cand[j])))
			cand[i][p], cand[j][q] = cand[j][q], cand[i][p]
		fv = f(cand); n += 1
		if fv <= fc or rng.random() < np.exp(-(fv - fc) / max(1e-9, T * max(1.0, fc) * 0.05)):
			cur, fc = cand, fv
			if fv < fb:
				best, fb = [list(x) for x in cand], fv
	return best, fb, n


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--pattern", default="bfs_n3m*_h98_26_10_19_22.json")
	p.add_argument("--iters", type=int, default=400)
	p.add_argument("--restarts", type=int, default=3)
	p.add_argument("--lattice", action="store_true", help="수리모형과 같은 격자 결정 공간에서 평가")
	p.add_argument("--out", default="metaheur_v5_26_10_20_17")
	A_ = p.parse_args()
	rows = []
	for f in sorted(glob.glob(os.path.join(ROOT, "figures", A_.pattern))):
		d = json.load(open(f))
		N, M, seed, kappa = int(d["drones"]), int(d["dests"]), int(d["seed"]), int(d["kappa"])
		env = DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=10000,
		                            deadlock_limit=10 ** 9, no_progress_limit=10 ** 9, num_drones=N, num_dests=M)
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
			_o, v, n = search(env, seed, lat, kappa, A_.iters, np.random.default_rng(1000 + r))
			ev += n
			bv = v if bv is None else min(bv, v)
		sec = time.time() - t0
		rows.append(dict(drones=N, dests=M, seed=seed, opt=d["milp_steps"], meta=bv if bv < 10 ** 6 else "",
		                 evals=ev, sec=round(sec, 1)))
		print(f"M={M:2d} s{seed}: 최적 {d['milp_steps']:.0f} | 탐색 {bv} ({ev}회 평가, {sec:.0f}초)", flush=True)
	out = os.path.join(ROOT, "figures", f"{A_.out}.csv")
	with open(out, "w", newline="", encoding="utf-8") as fh:
		w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
	print("저장:", out, flush=True)


if __name__ == "__main__":
	main()
