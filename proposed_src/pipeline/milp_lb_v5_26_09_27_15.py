"""연속 문제의 makespan 하한을 증명하려던 완화 MILP (v5, 26_09_27_15). **실패 기록이다.**

격자를 점이 아니라 '칸'으로 보고 칸 사이 최소거리로 이동·통신을 판정하므로 모든 연속해가 이 모형의
실행 가능해로 사상되고, 따라서 Gurobi의 하한에 기간 길이를 곱한 값은 연속 최적의 유효 하한이다.
그러나 실측 결과 하한이 쓸모없이 느슨하다 (드론 3·목적지 4에서 40스텝, 같은 인스턴스의 자명한
해석적 하한 60스텝보다도 낮다). 원인은 칸 최소거리가 사슬 도달 거리를 한 홉마다 칸 대각선만큼
부풀리는 데 있다 — 칸 98.2m에서 3홉 도달이 900m 대신 1,317m(46% 과대)가 되어 먼 목적지를
사슬 없이 닿는 것처럼 보이게 만든다. 오차를 10% 아래로 내리려면 칸을 21m로 줄여야 하고
그러면 반경 900m 안에 칸이 5,800개, 흐름 변수가 6,800만 개가 되어 풀 수 없다.
**연결성 제약이 이 문제의 난이도 전부이며, 그것을 조금이라도 느슨하게 하는 이산화는 하한을 버린다.**
"""

import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pipeline.common_v5_26_09_15_22 import chain_escape_manager, straight_worker
from util.instance_generator_v5_26_09_15_22 import sample_instance

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def cell_grid(cc, dests, spacing, comm_range, num_drones, horizon, step_len):
	"""제어 센터를 원점으로 하는 정사각 칸 격자를 만들고 도달 불가능한 칸을 버린다."""
	reach = num_drones * comm_range                      # 사슬로 닿을 수 있는 최대 거리 (여유 계수 없음)
	span = min(reach, horizon * step_len)                # 지평선 안에 갈 수 있는 거리
	rad = int(np.ceil((span + spacing) / spacing))
	pts = []
	for a in range(-rad, rad + 1):
		for b in range(-rad, rad + 1):
			p = np.array([cc[0] + a * spacing, cc[1] + b * spacing])
			if max(0.0, np.linalg.norm(p - cc) - spacing / np.sqrt(2.0)) > span + 1e-9:
				continue                             # 칸의 어느 점도 닿을 수 없으면 버린다
			pts.append(p)
	pos = np.array(pts)
	i0 = int(np.argmin(np.linalg.norm(pos - np.asarray(cc), axis=1)))
	cell = [int(np.argmin(np.linalg.norm(pos - np.asarray(q), axis=1))) for q in dests]
	return pos, i0, cell


def cell_dist(pos, spacing):
	"""축 정렬 정사각 칸 사이의 최소 거리 행렬 (칸 중심 좌표와 한 변 길이로 계산)."""
	dx = np.abs(pos[:, None, 0] - pos[None, :, 0])
	dy = np.abs(pos[:, None, 1] - pos[None, :, 1])
	ax = np.maximum(0.0, dx - spacing)
	ay = np.maximum(0.0, dy - spacing)
	return np.sqrt(ax ** 2 + ay ** 2)


def build(pos, i0, cell, num_drones, cap, horizon, step_len, comm_range, spacing,
          threads=6, time_limit=600.0, verbose=True, symmetry=True, gseed=0, eps=1e-5):
	"""완화 MILP를 만든다. 서비스는 한 기간 경계 점유만 요구한다 (연속해 사상을 보존)."""
	import gurobipy as gp
	from gurobipy import GRB

	n = len(pos)
	K = range(num_drones)
	T = range(horizon + 1)
	D = range(len(cell))
	d = cell_dist(pos, spacing)
	move = [np.flatnonzero(d[i] <= step_len + 1e-6) for i in range(n)]
	link = [(i, j) for i in range(n) for j in range(n) if i != j and d[i, j] <= comm_range + 1e-6]

	m = gp.Model("relay_milp_lb")
	m.Params.OutputFlag = 1 if verbose else 0
	m.Params.Threads = threads
	m.Params.TimeLimit = time_limit
	m.Params.Symmetry = 2
	m.Params.Seed = gseed
	y = m.addVars(n, num_drones, horizon + 1, vtype=GRB.BINARY, name="y")
	s = m.addVars(D, num_drones, horizon + 1, vtype=GRB.BINARY, name="s")
	r = m.addVars(num_drones, horizon + 1, vtype=GRB.BINARY, name="r")
	q = m.addVars(num_drones, horizon + 1, lb=0, ub=cap, vtype=GRB.INTEGER, name="q")
	rho = m.addVars(num_drones, horizon + 1, lb=0, ub=cap, name="rho")
	g = m.addVars(link, horizon + 1, lb=0, ub=num_drones, name="g")
	C = m.addVar(lb=0, ub=horizon, name="C")
	m.setObjective(C + eps * gp.quicksum(y[i, k, t] for i in range(n) if i != i0
	                                     for k in K for t in T), GRB.MINIMIZE)
	for k in K:
		m.addConstr(y[i0, k, 0] == 1, name=f"start_{k}")
		m.addConstr(q[k, 0] == cap, name=f"load0_{k}")
		for t in T:
			m.addConstr(gp.quicksum(y[i, k, t] for i in range(n)) == 1, name=f"one_{k}_{t}")
			# 하역·재적재는 각각 한 기간 길이라, 한 경계에 하나만 담긴다
			m.addConstr(gp.quicksum(s[j, k, t] for j in D) + r[k, t] <= 1, name=f"busy_{k}_{t}")
			for j in D:
				m.addConstr(s[j, k, t] <= y[cell[j], k, t], name=f"sat_{j}_{k}_{t}")
			m.addConstr(r[k, t] <= y[i0, k, t], name=f"rat_{k}_{t}")
			m.addConstr(rho[k, t] <= cap * r[k, t], name=f"rlim_{k}_{t}")
			m.addConstr(gp.quicksum(s[j, k, t] for j in D) <= q[k, t], name=f"cap_{k}_{t}")
		for t in range(horizon):
			for i in range(n):
				m.addConstr(y[i, k, t + 1] <= gp.quicksum(y[j, k, t] for j in move[i]),
				            name=f"mv_{i}_{k}_{t}")
			m.addConstr(q[k, t + 1] == q[k, t] - gp.quicksum(s[j, k, t] for j in D) + rho[k, t],
			            name=f"bal_{k}_{t}")
			m.addConstr(q[k, t + 1] >= cap * r[k, t], name=f"full_{k}_{t}")
	# 도달 시각 하한: 제어 센터 칸에서 칸 i까지 최소거리를 한 기간 이동거리로 나눈 만큼은 걸린다.
	# 완화에서도 유효한 부등식이라 변수를 직접 0으로 고정한다 (초기 기간의 먼 칸이 대거 사라진다).
	early = np.ceil(d[i0] / step_len - 1e-9).astype(int)
	for i in range(n):
		for t in T:
			if t < early[i]:
				for k in K:
					y[i, k, t].ub = 0.0
	for j in D:
		m.addConstr(gp.quicksum(s[j, k, t] for k in K for t in T) == 1, name=f"serve_{j}")
		for k in K:
			for t in T:
				if t < early[cell[j]]:
					s[j, k, t].ub = 0.0
				m.addConstr(C >= t * s[j, k, t], name=f"mk_{j}_{k}_{t}")
	# 같은 드론이 두 목적지를 연달아 맡으려면 그 사이를 이동할 시간이 있어야 한다
	for a_ in D:
		for b_ in D:
			if b_ <= a_:
				continue
			gap = int(np.ceil(d[cell[a_], cell[b_]] / step_len - 1e-9))
			if gap < 1:
				continue
			for k in K:
				for t in T:
					for tp in range(t, min(t + gap, horizon + 1)):
						m.addConstr(s[a_, k, t] + s[b_, k, tp] <= 1, name=f"sep_{a_}_{b_}_{k}_{t}_{tp}")
						m.addConstr(s[b_, k, t] + s[a_, k, tp] <= 1, name=f"sepr_{a_}_{b_}_{k}_{t}_{tp}")
	# makespan 누적 선형화: 하한만 필요한 모형이므로 LP 완화가 강한 쪽을 쓴다
	nd = len(D)
	av = m.addVars(range(1, horizon + 1), vtype=GRB.BINARY, name="a")
	for t in range(1, horizon + 1):
		m.addConstr(nd * av[t] <= gp.quicksum(s[j, k, tp] for j in D for k in K for tp in range(t)),
		            name=f"acum_{t}")
		if t < horizon:
			m.addConstr(av[t] <= av[t + 1], name=f"amono_{t}")
	m.addConstr(C >= horizon - gp.quicksum(av[t] for t in range(1, horizon + 1)), name="mkcum")
	if symmetry:
		for j in D:
			for k in K:
				if j < k:
					for t in T:
						m.addConstr(s[j, k, t] == 0, name=f"sym_{j}_{k}_{t}")
	# 연결성: 점유된 칸마다 1단위를 제어 센터 칸으로 흘린다 (단일 상품 흐름). 칸 사이 최소거리로
	# 간선을 깔았으므로 실제로 통신 가능한 모든 배치가 허용된다 (완화 방향이 맞다).
	adj = {i: [] for i in range(n)}
	for (i, j) in link:
		adj[i].append(j)
	for t in T:
		u = {i: gp.quicksum(y[i, k, t] for k in K) for i in range(n)}
		for i in range(n):
			if i == i0:
				continue
			m.addConstr(gp.quicksum(g[i, j, t] for j in adj[i])
			            - gp.quicksum(g[j, i, t] for j in adj[i]) == u[i], name=f"flow_{i}_{t}")
		for (i, j) in link:
			m.addConstr(g[i, j, t] <= num_drones * u[i], name=f"ci_{i}_{j}_{t}")
	# 사슬 하한 절단면: 칸 i가 점유되면 제어 센터 칸을 벗어난 드론이 ceil(dmin_0i/R)대 이상이다
	for i in range(n):
		if i == i0:
			continue
		h1 = int(np.ceil(d[i0, i] / comm_range - 1e-9))
		if h1 < 2:
			continue
		for t in T:
			away = num_drones - gp.quicksum(y[i0, k, t] for k in K)
			for k in K:
				m.addConstr(away >= h1 * y[i, k, t], name=f"ring_{i}_{k}_{t}")
	m.update()
	return m, dict(y=y, s=s, C=C, n=n, link=link, move=move, d=d)


def heuristic_makespan(cc, dests, num_drones, comm_range):
	"""같은 인스턴스에서 규칙+탈출 베이스라인의 makespan을 잰다 (비교용)."""
	from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
	env = DisasterRelayDroneEnv(map_path="", comm_range=comm_range, cluster_penalty=False,
	                            max_steps=20000, deadlock_limit=10 ** 9, no_progress_limit=10 ** 9,
	                            num_drones=num_drones, num_dests=len(dests))
	env.reset()
	env.cc_pos, env.dests_pos = np.asarray(cc, dtype=float), np.asarray(dests, dtype=float)
	env.reset()
	mf = chain_escape_manager(60, 40, "shuffle")
	env.set_goals(mf(env))
	done, t = False, 0
	while not done and t < env.max_steps:
		_o, _w, _r, done = env.step(straight_worker(env))
		t += 1
		if t % 20 == 0 or env.goal_invalid().any():
			env.set_goals(mf(env))
	st = env.episode_stats()
	return st["makespan"], int(env.num_dests - env.dests_active.sum())


def main():
	"""완화 모형을 풀어 연속 최적 makespan의 증명된 하한을 출력한다."""
	p = argparse.ArgumentParser()
	p.add_argument("--num-drones", type=int, default=3)
	p.add_argument("--num-dests", type=int, default=4)
	p.add_argument("--seed", type=int, default=901)
	p.add_argument("--comm-range", type=float, default=300.0)
	p.add_argument("--kappa", type=int, default=10, help="한 기간의 환경 스텝 수 (하역 시간과 같게)")
	p.add_argument("--spacing", type=float, default=0.0, help="칸 한 변. 0이면 기간 이동거리/sqrt(2)")
	p.add_argument("--horizon", type=int, default=0, help="기간 수. 0이면 규칙 makespan에서 자동")
	p.add_argument("--slack", type=float, default=1.0, help="자동 지평선 여유 배수")
	p.add_argument("--cap", type=int, default=5)
	p.add_argument("--time-limit", type=float, default=900.0)
	p.add_argument("--threads", type=int, default=6)
	p.add_argument("--gurobi-seed", type=int, default=0)
	p.add_argument("--no-symmetry", action="store_true")
	p.add_argument("--quiet", action="store_true")
	p.add_argument("--out", default="milp_lb_v5_26_09_27_15")
	a = p.parse_args()

	speed = 50.0 * 1000.0 / 3600.0
	step_len = speed * a.kappa
	spacing = a.spacing if a.spacing > 0 else step_len / np.sqrt(2.0)
	cc, dests = sample_instance(a.num_dests, seed=a.seed, num_drones=a.num_drones,
	                            comm_range=a.comm_range)
	mk, got = heuristic_makespan(cc, dests, a.num_drones, a.comm_range)
	print(f"[규칙+탈출] 배송 {got}/{a.num_dests} makespan {mk}")
	horizon = a.horizon if a.horizon > 0 else int(np.ceil(mk * a.slack / a.kappa))
	pos, i0, cell = cell_grid(cc, dests, spacing, a.comm_range, a.num_drones, horizon, step_len)
	print(f"[격자] 칸 {spacing:.1f}m 한 기간 이동 {step_len:.1f}m 칸수 {len(pos)} 지평선 {horizon}기간")
	t0 = time.time()
	m, v = build(pos, i0, cell, a.num_drones, a.cap, horizon, step_len, a.comm_range, spacing,
	             threads=a.threads, time_limit=a.time_limit, verbose=not a.quiet,
	             symmetry=not a.no_symmetry, gseed=a.gurobi_seed)
	print(f"[모형] 변수 {m.NumVars} (이진 {m.NumBinVars}) 제약 {m.NumConstrs} 생성 {time.time()-t0:.0f}초")
	m.optimize()
	import gurobipy as gp
	bound = m.ObjBound if m.SolCount > 0 or m.Status in (gp.GRB.TIME_LIMIT, gp.GRB.OPTIMAL) else None
	res = {"seed": a.seed, "num_drones": a.num_drones, "num_dests": a.num_dests,
	       "spacing": spacing, "kappa": a.kappa, "horizon": horizon, "cells": len(pos),
	       "rule_makespan": mk, "status": int(m.Status), "runtime": m.Runtime}
	if bound is None:
		print("[결과] 하한을 얻지 못했다")
	else:
		lb_period = int(np.floor(bound + 1e-6))
		lb_steps = lb_period * a.kappa
		res.update(obj_bound=float(bound), lb_period=lb_period, lb_steps=lb_steps)
		if m.SolCount > 0:
			res["relaxed_obj"] = float(m.ObjVal)
		print("\n=== 결과 ===")
		print(f"완화 최적값의 하한 {bound:.3f}기간 -> 연속 최적 makespan >= {lb_steps} 스텝")
		if m.SolCount > 0:
			print(f"완화 모형의 잠정해 {m.ObjVal:.3f}기간 (완화이므로 실행 가능한 계획은 아니다)")
		print(f"규칙+탈출 {mk} 스텝 -> 최적 대비 최대 {100.0 * (mk - lb_steps) / max(1, lb_steps):.0f}% 초과")
	out = os.path.join(ROOT, "figures", f"{a.out}.json")
	with open(out, "w", encoding="utf-8") as f:
		json.dump(res, f, ensure_ascii=False, indent=1)
	print(f"저장: {out}")


if __name__ == "__main__":
	main()
