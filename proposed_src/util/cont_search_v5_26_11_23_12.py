"""연속 공간 탐색 휴리스틱 두 가지 (방문 순서 탐색, 작업열 탐색)를 연속 모형 비교 인스턴스에서 잰다 (사이클 35).

해는 드론별 작업열이다. 배송 작업 (D, j)만 있으면 방문 순서 탐색이고, 실행은 규칙과 같다 (목표는 열의 첫 미배송 수요지,
중계 수와 위치·교착 복구는 규칙 기하). 작업열 탐색은 여기에 강제 중계 (R, j) "j 가 배송될 때까지 중계"를 더해 누가 언제까지
중계하는지를 탐색 변수로 둔다. 강제 중계가 없으면 두 실행이 같으므로, 방문 순서 탐색의 최적해에서 출발하는 작업열 탐색은 그보다 나쁠 수 없다.
"""

import argparse
import csv
import multiprocessing as mp
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
D, RL = "D", "R"          # 작업 종류: 배송 / 강제 중계 (둘 다 t[1] 이 수요지 번호)
VO_KINDS = (0, 1, 2)
TS_KINDS = (0, 1, 2, 3, 4, 5, 6)


def mkenv(M, seed, Q, max_steps=3000):
	"""cont_ctrl 과 같은 설정으로 인스턴스를 깐 환경을 만든다."""
	from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
	from util.instance_generator_v5_26_09_15_22 import sample_instance
	e = DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=max_steps,
	                          deadlock_limit=10 ** 9, no_progress_limit=10 ** 9, num_drones=3, num_dests=M)
	e.max_capacity = Q
	e.reset()
	e.cc_pos, e.dests_pos = sample_instance(M, seed=seed, num_drones=3, comm_range=300.0, cc_pos=None)
	e.relay_lookahead = False
	return e


def cur(env, s):
	"""작업열 s 에서 아직 유효한 첫 작업의 위치를 돌려준다 (없으면 -1)."""
	return next((q for q, t in enumerate(s) if env.dests_active[t[1]]), -1)


def act(env, seqs, st):
	"""작업열에서 드론별 상위 행동을 만들어 건다. 강제 중계 외의 중계 수·위치와 교착 복구는 규칙과 같다."""
	from pipeline.common_v5_26_09_15_22 import ESC_HOLD, ESC_K
	N = env.num_drones
	acts = np.zeros(N, dtype=np.int64)
	dst = -np.ones(N, dtype=np.int64)
	if env.deadlock_run >= ESC_K and st["esc"] == 0:
		st["esc"], st["n"] = ESC_HOLD, st["n"] + 1
		for s in seqs:                                   # 복구 시작: 서 있던 강제 중계는 버린다
			q = cur(env, s)
			if q >= 0 and s[q][0] == RL:
				s.pop(q)
	forced = []
	for i in range(N):
		q = cur(env, seqs[i])
		if q >= 0 and seqs[i][q][0] == RL:
			acts[i] = env.n_cand + 1
			forced.append(i)
		elif env.drones_capacity[i] <= 0 or q < 0:
			acts[i] = env.n_cand
		else:
			acts[i], dst[i] = 0, seqs[i][q][1]
	if env.dests_active.any():
		reach = np.linalg.norm(env.drones_pos - env.cc_pos, axis=1)
		live = [i for i in range(N) if dst[i] >= 0]
		if live:
			tip = max(live, key=lambda i: reach[i])
			d = max(float(reach[tip]), float(np.linalg.norm(env.dests_pos[dst[tip]] - env.cc_pos)))
			need = min(max(1, int(np.ceil(d / (env.relay_beta * env.comm_range)))) - 1, N - 1)
			extra = [j for j in np.argsort(-reach) if j != tip and j not in forced][:max(0, need - len(forced))]
			for i in extra:
				acts[i], dst[i] = env.n_cand + 1, -1
			if st["esc"] > 0:
				st["esc"] -= 1
				order_r = list(np.argsort(-reach))
				new_tip = order_r[min(st["n"], len(order_r) - 1)]
				acts[tip], dst[tip] = env.n_cand, -1
				q = next((q for q, t in enumerate(seqs[new_tip]) if t[0] == D and env.dests_active[t[1]]), -1)
				if env.drones_capacity[new_tip] > 0 and q >= 0:
					acts[new_tip], dst[new_tip] = 0, seqs[new_tip][q][1]
	env.set_goals(acts, dst)


def rollout(env, seqs, seed, hl=20):
	"""작업열 하나를 연속 공간에서 실행해 makespan(미완주면 None)을 돌려준다. seed 는 투영의 동률 처리 난수."""
	from pipeline.common_v5_26_09_15_22 import straight_worker
	seqs = [list(s) for s in seqs]
	has_rl = any(t[0] == RL for s in seqs for t in s)
	env.rng = np.random.default_rng(seed)
	env.reset()
	st = {"esc": 0, "n": 0}
	act(env, seqs, st)
	done = False
	while not done and env.current_step < env.max_steps:
		n0 = int(env.dests_active.sum())
		_o, _w, _r, done = env.step(straight_worker(env))
		if not done and (env.current_step % hl == 0 or env.goal_invalid().any()
		                 or (has_rl and int(env.dests_active.sum()) != n0)):
			act(env, seqs, st)
	return env.makespan


def neighbour(cur_s, rng, env, kinds):
	"""이웃 하나: 0 배송 재배치, 1 배송 교환, 2 구간 뒤집기, 3 중계 삽입, 4 중계 삭제, 5 중계 열 내 이동, 6 트리거 변경."""
	s = [list(x) for x in cur_s]
	N, M = len(s), env.num_dests
	k = int(kinds[int(rng.integers(len(kinds)))])
	dl = [(i, q) for i in range(N) for q in range(len(s[i])) if s[i][q][0] == D]
	rl = [(i, q) for i in range(N) for q in range(len(s[i])) if s[i][q][0] == RL]
	if k == 0 and dl:
		i, q = dl[int(rng.integers(len(dl)))]
		v = s[i].pop(q)
		j = int(rng.integers(N))
		s[j].insert(int(rng.integers(len(s[j]) + 1)), v)
	elif k == 1 and len(dl) >= 2:
		(i, q), (i2, q2) = (dl[x] for x in rng.choice(len(dl), 2, replace=False))
		s[i][q], s[i2][q2] = s[i2][q2], s[i][q]
	elif k == 2 and dl:
		i = dl[int(rng.integers(len(dl)))][0]
		if len(s[i]) >= 2:
			a, b = sorted(rng.choice(len(s[i]), 2, replace=False))
			s[i][a:b + 1] = s[i][a:b + 1][::-1]
	elif k == 3:
		i = int(rng.integers(N))
		s[i].insert(int(rng.integers(len(s[i]) + 1)), (RL, int(rng.integers(M))))
	elif k == 4 and rl:
		i, q = rl[int(rng.integers(len(rl)))]
		s[i].pop(q)
	elif k == 5 and rl:
		i, q = rl[int(rng.integers(len(rl)))]
		v = s[i].pop(q)
		s[i].insert(int(rng.integers(len(s[i]) + 1)), v)
	elif k == 6 and rl:
		i, q = rl[int(rng.integers(len(rl)))]
		s[i][q] = (RL, int(rng.integers(M)))
	else:
		return neighbour(cur_s, rng, env, (0,))
	return s


def local(c, fc, f, rng, budget, env, kinds):
	"""첫 개선 국소 탐색: 이웃 종류를 번갈아 시도하고 개선이 없으면 멈춘다."""
	n, improved = 0, True
	while improved and n < budget:
		improved = False
		for k in kinds:
			for _ in range(8):
				if n >= budget:
					break
				x = neighbour(c, rng, env, (k,))
				v = f(x)
				n += 1
				if v < fc:
					c, fc, improved = x, v, True
					break
	return c, fc, n


def ils(env, init, f, rng, iters, kinds):
	"""반복 국소 탐색: 국소 탐색 + 최적해 3회 교란. (최적해, 값, 평가 횟수)."""
	c, fc, n = local([list(x) for x in init], f(init), f, rng, iters // 4, env, kinds)
	best, fb = [list(x) for x in c], fc
	while n < iters:
		sh = [list(x) for x in best]
		for _ in range(3):
			sh = neighbour(sh, rng, env, kinds)
		fs = f(sh)
		n += 1
		sh, fs, m = local(sh, fs, f, rng, max(10, (iters - n) // 4), env, kinds)
		n += m
		if fs <= fb:
			best, fb = [list(x) for x in sh], fs
	return best, fb, n


def robust(env, sol, seeds):
	"""해 하나를 여러 동률 난수로 다시 굴려 중앙값을 돌려준다 (탐색 난수에 맞춘 이득을 걷어 낸다)."""
	env.max_steps = 3000
	v = [rollout(env, sol, s) or 10 ** 6 for s in seeds]
	return float(np.median(v))


def job(args):
	"""인스턴스 하나에서 방문 순서 탐색, 이어서 그 최적해에서 작업열 탐색을 돈다."""
	M, seed, Q, vo_iters, ts_iters, rs = args
	from util.metaheur_v5_26_10_20_17 import seed_order
	env = mkenv(M, seed, Q)
	init = [[(D, int(j)) for j in o] for o in seed_order(env, seed)]
	env = mkenv(M, seed, Q)
	f = lambda s: rollout(env, s, seed) or 10 ** 6
	v0 = f(init)
	env.max_steps = int(2.5 * v0) if v0 < 10 ** 6 else 3000      # 나쁜 해를 끝까지 굴리지 않는다
	out = dict(dests=M, seed=seed, vo_init=v0)
	t0 = time.time()
	bv, fv, ev = None, None, 0
	for r in range(rs):
		sol, v, n = ils(env, init, f, np.random.default_rng(1000 + r), vo_iters, VO_KINDS)
		ev += n
		if fv is None or v < fv:
			bv, fv = sol, v
	out.update(vorder=fv, vorder_evals=ev, vorder_sec=round(time.time() - t0, 1))
	t0 = time.time()
	bt, ft, ev = bv, fv, 0
	for r in range(rs):
		sol, v, n = ils(env, bv, f, np.random.default_rng(2000 + r), ts_iters, TS_KINDS)
		ev += n
		if v < ft:
			bt, ft = sol, v
	out.update(taskseq=ft, taskseq_evals=ev, taskseq_sec=round(time.time() - t0, 1),
	           taskseq_relays=sum(1 for s in bt for t in s if t[0] == RL))
	seeds = [seed * 100 + k for k in range(1, 11)]
	out["vorder_med10"] = robust(env, bv, seeds)
	out["taskseq_med10"] = robust(env, bt, seeds)
	print(out, flush=True)
	return out


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--spec", default="9:1201,1202,1203,1204,1205;10:1201,1202,1203,1204,1205;11:1201,1202,1203,1204,1205;12:1201,1202,1203,1204,1205")
	p.add_argument("--capacity", type=int, default=3)
	p.add_argument("--vo-iters", type=int, default=400)
	p.add_argument("--ts-iters", type=int, default=1000)
	p.add_argument("--restarts", type=int, default=3)
	p.add_argument("--workers", type=int, default=6)
	p.add_argument("--out", default="cont_search_v5_26_11_23_12")
	A = p.parse_args()
	jobs = []
	for part in A.spec.split(";"):
		M, seeds = part.split(":")
		jobs += [(int(M), int(s), A.capacity, A.vo_iters, A.ts_iters, A.restarts) for s in seeds.split(",")]
	with mp.get_context("spawn").Pool(A.workers) as pool:
		rows = pool.map(job, jobs, chunksize=1)
	out = os.path.join(ROOT, "figures", f"{A.out}.csv")
	with open(out, "w", newline="") as fh:
		w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
		w.writeheader()
		w.writerows(rows)
	print("저장:", out, flush=True)


if __name__ == "__main__":
	main()
