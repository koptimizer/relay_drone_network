"""연속 위치·이산 시간 수리모형 (사이클 34, 26-11-19): 드론 위치를 연속 변수로 두고 기간(κ 스텝)마다 변위를 정한다.

표기.  드론 i ∈ {1..N}, 수요지 j ∈ {1..M} (좌표 d_j), 거점 c (노드 0). 기간 t = 0..T-1, 매듭(knot) t = 0..T.
      v = 스텝당 속도, κ = 기간 길이(스텝), L = vκ = 기간당 최대 변위, R = 통신 반경, ρ = 서비스 반경(25 m),
      σ = ceil(10/κ) = 서비스(하역·재적재)가 차지하는 기간 수, Q = 적재 용량.
변수.  p_i(t) ∈ R^2 위치 (연속).  y_ab(t) ∈ {0,1}: 기간 t 에 쌍 (a,b) 링크 사용 (a,b ∈ {0..N}, 0 = 거점).
      f_ab(t) ≥ 0 거점에서 나가는 연결성 유량 (유향).  s_ijt ∈ {0,1}: 드론 i 가 기간 t 에 수요지 j 서비스를 시작.
      r_it ∈ {0,1}: 재적재 시작.  g_it ≥ 0 재적재량.  q_i(t) ∈ [0,Q] 적재량.  C ≥ 0 makespan (스텝).
목적.  min C.
제약.  (1) p_i(0) = c.  (2) 이동: ||p_i(t+1) − p_i(t)|| ≤ L·(1 − w_i(t)), w_i(t) = 그 기간에 서비스/재적재 창이 열려 있는지.
      (3) 통신: y_ab(t) = 1 ⇒ ||p_a(t) − p_b(t)|| ≤ R 이고 ||p_a(t+1) − p_b(t+1)|| ≤ R (직선 이동 중 거리는 볼록 →
          양 끝에서 R 이하면 기간 내내 R 이하).  (4) 연결성: 기간마다 거점이 N 단위를 내보내고 드론마다 1 단위를 받는
          유량, 호 용량 N·y_ab(t).  (5) 서비스: s_ijt = 1 ⇒ ||p_i(t) − d_j|| ≤ ρ, 그 뒤 σ 기간 정지; r_it 도 거점에 대해 같다.
      (6) Σ_{i,t} s_ijt = 1.  (7) 적재: q_i(0) = Q, q_i(t+1) = q_i(t) − Σ_j s_ijt + g_it, g_it ≤ Q·r_it, q ≤ Q, Σ_j s_ijt ≤ q_i(t).
      (8) 창은 겹치지 않는다: Σ_j Σ_{τ=t−σ+1..t} s_ijτ + Σ_τ r_iτ ≤ 1.  (9) C ≥ κ·t·s_ijt (makespan = 마지막 배송 시작 시각,
          환경의 정의와 같다).  (10) 대칭 제거: 매듭 1 에서 드론의 x 좌표 순서.
노름.  --norm soc : 보조 거리 변수 e 에 dx²+dy² ≤ e² (2차 원뿔) 와 e ≤ R + M(1−y) (MISOCP, 정확).
      --norm poly: 정 n 각형 바깥 근사 a_k·Δ ≤ R + M(1−y) (순수 MILP; 원판을 포함하므로 하한 성질 유지, 오차 ≤ 1/cos(π/n) − 1).
결과는 연속 환경에 재생해 배송 수·단절 수를 확인한다.
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
from pipeline.common_v5_26_09_15_22 import chain_escape_manager, straight_worker
from util.instance_generator_v5_26_09_15_22 import sample_instance

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RHO, SERVICE, MAP = 25.0, 10, 1000.0


def rule_makespan(cc, dests, N, Q, R, max_steps=3000):
	"""규칙(+복구)의 연속 공간 makespan (지평 상한용)."""
	env = DisasterRelayDroneEnv(map_path="", comm_range=R, cluster_penalty=False, max_steps=max_steps,
	                            deadlock_limit=10 ** 9, no_progress_limit=10 ** 9, num_drones=N, num_dests=len(dests))
	env.max_capacity = Q
	env.cc_pos, env.dests_pos = cc.copy(), dests.copy()
	env.reset()
	mf = chain_escape_manager()
	env.set_goals(mf(env))
	done = False
	while not done and env.current_step < max_steps:
		_o, _w, _r, done = env.step(straight_worker(env))
		if env.current_step % 20 == 0 or env.goal_invalid().any():
			env.set_goals(mf(env))
	return env.makespan, env.max_speed


def build(cc, dests, N, Q, R, kappa, T, v, norm="soc", ngon=32, big_m=None, comm_inner=True):
	"""모형을 만든다. (model, vars) 를 돌려준다.

	comm_inner=True (기본): 통신 반경만 내접 다각형(R·cos(π/n))으로 두어 해의 모든 링크가 환경의 반경 R 안에 들어가게 한다.
	환경의 연결성 투영은 반경을 조금이라도 넘으면 이동을 통째로 막으므로(26-11-19: 64각형 바깥 근사에서 한 기간 이동이
	막혀 추종 오차 139.6 m, 배송 실패), 통신만은 보수적으로 둔다. 이동·서비스 반경은 바깥 근사(완화)를 유지한다 — 그 초과분은
	기간당 0.17 m, 서비스 0.03 m 로 재생 허용 오차 안이다. 따라서 모형값은 통신 반경 0.12% 축소를 제외하면 κ 해상도 문제의 하한이다.
	"""
	import gurobipy as gp
	from gurobipy import GRB
	M = len(dests)
	L = v * kappa
	sigma = int(math.ceil(SERVICE / kappa))
	Mbig = big_m if big_m else 1.5 * MAP
	nodes = list(range(N + 1))                      # 0 = 거점
	pairs = [(a, b) for a in nodes for b in nodes if a < b]
	m = gp.Model("cont_relay")
	m.Params.OutputFlag = 0
	# 위치 (거점은 상수)
	px = m.addVars(range(1, N + 1), range(T + 1), lb=0.0, ub=MAP, name="px")
	py = m.addVars(range(1, N + 1), range(T + 1), lb=0.0, ub=MAP, name="py")
	def X(a, t):
		return (cc[0], cc[1]) if a == 0 else (px[a, t], py[a, t])
	y = m.addVars(pairs, range(T), vtype=GRB.BINARY, name="y")
	arcs = [(a, b) for (a, b) in pairs] + [(b, a) for (a, b) in pairs]
	f = m.addVars(arcs, range(T), lb=0.0, ub=N, name="f")
	TS = range(T - sigma + 1)                        # 서비스 시작 가능 기간 (창이 지평 안에 들어가야 한다)
	s = m.addVars(range(1, N + 1), range(M), TS, vtype=GRB.BINARY, name="s")
	r = m.addVars(range(1, N + 1), TS, vtype=GRB.BINARY, name="r")
	g = m.addVars(range(1, N + 1), TS, lb=0.0, ub=Q, name="g")
	q = m.addVars(range(1, N + 1), range(T + 1), lb=0.0, ub=Q, name="q")
	C = m.addVar(lb=0.0, ub=kappa * T, name="C")
	m.setObjective(C, GRB.MINIMIZE)
	# (1) 출발
	for i in range(1, N + 1):
		m.addConstr(px[i, 0] == cc[0]); m.addConstr(py[i, 0] == cc[1])
		m.addConstr(q[i, 0] == Q)
	# 창 지시자 w_i(t): 기간 t 에 서비스/재적재 창이 열려 있음
	def w(i, t):
		terms = [s[i, j, tau] for j in range(M) for tau in range(max(0, t - sigma + 1), t + 1) if tau in TS]
		terms += [r[i, tau] for tau in range(max(0, t - sigma + 1), t + 1) if tau in TS]
		return gp.quicksum(terms)
	# 노름 제약 생성기: ||(dx, dy)|| ≤ rhs
	angles = [2 * math.pi * k / ngon for k in range(ngon)]
	def norm_le(dx, dy, rhs, tag):
		if norm == "poly":
			for k, th in enumerate(angles):
				m.addConstr(math.cos(th) * dx + math.sin(th) * dy <= rhs, name=f"{tag}_{k}")
			return None
		e = m.addVar(lb=0.0, ub=2 * MAP, name=f"e_{tag}")
		m.addQConstr(dx * dx + dy * dy <= e * e, name=f"q_{tag}")
		m.addConstr(e <= rhs, name=f"l_{tag}")
		return e
	# (2) 이동
	for i in range(1, N + 1):
		for t in range(T):
			norm_le(px[i, t + 1] - px[i, t], py[i, t + 1] - py[i, t], L * (1 - w(i, t)), f"mv{i}_{t}")
	# (3) 통신: 기간 양 끝 (내접 다각형이면 반경을 R·cos(π/n) 으로)
	Rc = R * math.cos(math.pi / ngon) if (comm_inner and norm == "poly") else R
	for (a, b) in pairs:
		for t in range(T):
			for tt in (t, t + 1):
				ax, ay = X(a, tt); bx, by = X(b, tt)
				norm_le(ax - bx, ay - by, Rc + Mbig * (1 - y[a, b, t]), f"cm{a}_{b}_{t}_{tt}")
	# (4) 연결성 유량
	for t in range(T):
		for (a, b) in pairs:
			m.addConstr(f[a, b, t] <= N * y[a, b, t]); m.addConstr(f[b, a, t] <= N * y[a, b, t])
		m.addConstr(gp.quicksum(f[0, b, t] for b in range(1, N + 1)) - gp.quicksum(f[b, 0, t] for b in range(1, N + 1)) == N)
		for i in range(1, N + 1):
			m.addConstr(gp.quicksum(f[a, i, t] for a in nodes if a != i) - gp.quicksum(f[i, a, t] for a in nodes if a != i) == 1)
	# (5) 서비스 위치, (8) 창 비겹침
	for i in range(1, N + 1):
		for t in TS:
			for j in range(M):
				norm_le(px[i, t] - dests[j][0], py[i, t] - dests[j][1], RHO + Mbig * (1 - s[i, j, t]), f"sv{i}_{j}_{t}")
			norm_le(px[i, t] - cc[0], py[i, t] - cc[1], RHO + Mbig * (1 - r[i, t]), f"rl{i}_{t}")
		for t in range(T):
			m.addConstr(w(i, t) <= 1, name=f"win{i}_{t}")
	# (6) 수요지마다 한 번
	for j in range(M):
		m.addConstr(gp.quicksum(s[i, j, t] for i in range(1, N + 1) for t in TS) == 1, name=f"once{j}")
	# (7) 적재
	for i in range(1, N + 1):
		for t in range(T):
			sv = gp.quicksum(s[i, j, t] for j in range(M)) if t in TS else 0
			gt = g[i, t] if t in TS else 0
			m.addConstr(q[i, t + 1] == q[i, t] - sv + gt)
			if t in TS:
				m.addConstr(g[i, t] <= Q * r[i, t])
				m.addConstr(sv <= q[i, t])
	# (9) makespan
	for i in range(1, N + 1):
		for j in range(M):
			for t in TS:
				m.addConstr(C >= kappa * t * s[i, j, t])
	# (10) 대칭 제거: 매듭 1 의 x 좌표 순서 (드론이 동일하므로 유효)
	for i in range(1, N):
		m.addConstr(px[i, 1] <= px[i + 1, 1])
	return m, dict(px=px, py=py, y=y, s=s, r=r, q=q, C=C, TS=list(TS), sigma=sigma)


def build_scip(cc, dests, N, Q, R, kappa, T, v, norm="poly", ngon=16, big_m=None, threads=4, time_limit=3600.0, gap=1e-3):
	"""같은 모형을 SCIP(pyscipopt)으로 만든다. Gurobi 라이선스가 없을 때의 대체 경로 (26-11-19: WLS 만료)."""
	from pyscipopt import Model, quicksum
	M = len(dests)
	L = v * kappa
	sigma = int(math.ceil(SERVICE / kappa))
	Mbig = big_m if big_m else 1.5 * MAP
	nodes = list(range(N + 1))
	pairs = [(a, b) for a in nodes for b in nodes if a < b]
	m = Model("cont_relay")
	m.hideOutput()
	m.setParam("limits/time", time_limit)
	m.setParam("limits/gap", gap)
	m.setParam("parallel/maxnthreads", threads)
	px = {(i, t): m.addVar(lb=0.0, ub=MAP, name=f"px{i}_{t}") for i in range(1, N + 1) for t in range(T + 1)}
	py = {(i, t): m.addVar(lb=0.0, ub=MAP, name=f"py{i}_{t}") for i in range(1, N + 1) for t in range(T + 1)}
	def X(a, t):
		return (cc[0], cc[1]) if a == 0 else (px[a, t], py[a, t])
	y = {(a, b, t): m.addVar(vtype="B", name=f"y{a}_{b}_{t}") for (a, b) in pairs for t in range(T)}
	arcs = [(a, b) for (a, b) in pairs] + [(b, a) for (a, b) in pairs]
	f = {(a, b, t): m.addVar(lb=0.0, ub=N, name=f"f{a}_{b}_{t}") for (a, b) in arcs for t in range(T)}
	TS = list(range(T - sigma + 1))
	s_ = {(i, j, t): m.addVar(vtype="B", name=f"s{i}_{j}_{t}") for i in range(1, N + 1) for j in range(M) for t in TS}
	r = {(i, t): m.addVar(vtype="B", name=f"r{i}_{t}") for i in range(1, N + 1) for t in TS}
	g = {(i, t): m.addVar(lb=0.0, ub=Q, name=f"g{i}_{t}") for i in range(1, N + 1) for t in TS}
	q = {(i, t): m.addVar(lb=0.0, ub=Q, name=f"q{i}_{t}") for i in range(1, N + 1) for t in range(T + 1)}
	C = m.addVar(lb=0.0, ub=kappa * T, name="C")
	m.setObjective(C, "minimize")
	for i in range(1, N + 1):
		m.addCons(px[i, 0] == cc[0]); m.addCons(py[i, 0] == cc[1]); m.addCons(q[i, 0] == Q)
	def w(i, t):
		terms = [s_[i, j, tau] for j in range(M) for tau in range(max(0, t - sigma + 1), t + 1) if tau in TS]
		terms += [r[i, tau] for tau in range(max(0, t - sigma + 1), t + 1) if tau in TS]
		return quicksum(terms)
	angles = [2 * math.pi * k / ngon for k in range(ngon)]
	def norm_le(dx, dy, rhs, tag):
		if norm == "poly":
			for k, th in enumerate(angles):
				m.addCons(math.cos(th) * dx + math.sin(th) * dy <= rhs, name=f"{tag}_{k}")
			return
		e = m.addVar(lb=0.0, ub=2 * MAP, name=f"e_{tag}")
		m.addCons(dx * dx + dy * dy <= e * e, name=f"q_{tag}")
		m.addCons(e <= rhs, name=f"l_{tag}")
	for i in range(1, N + 1):
		for t in range(T):
			norm_le(px[i, t + 1] - px[i, t], py[i, t + 1] - py[i, t], L * (1 - w(i, t)), f"mv{i}_{t}")
	for (a, b) in pairs:
		for t in range(T):
			for tt in (t, t + 1):
				ax, ay = X(a, tt); bx, by = X(b, tt)
				norm_le(ax - bx, ay - by, R + Mbig * (1 - y[a, b, t]), f"cm{a}_{b}_{t}_{tt}")
	for t in range(T):
		for (a, b) in pairs:
			m.addCons(f[a, b, t] <= N * y[a, b, t]); m.addCons(f[b, a, t] <= N * y[a, b, t])
		m.addCons(quicksum(f[0, b, t] for b in range(1, N + 1)) - quicksum(f[b, 0, t] for b in range(1, N + 1)) == N)
		for i in range(1, N + 1):
			m.addCons(quicksum(f[a, i, t] for a in nodes if a != i) - quicksum(f[i, a, t] for a in nodes if a != i) == 1)
	for i in range(1, N + 1):
		for t in TS:
			for j in range(M):
				norm_le(px[i, t] - dests[j][0], py[i, t] - dests[j][1], RHO + Mbig * (1 - s_[i, j, t]), f"sv{i}_{j}_{t}")
			norm_le(px[i, t] - cc[0], py[i, t] - cc[1], RHO + Mbig * (1 - r[i, t]), f"rl{i}_{t}")
		for t in range(T):
			m.addCons(w(i, t) <= 1, name=f"win{i}_{t}")
	for j in range(M):
		m.addCons(quicksum(s_[i, j, t] for i in range(1, N + 1) for t in TS) == 1, name=f"once{j}")
	for i in range(1, N + 1):
		for t in range(T):
			sv = quicksum(s_[i, j, t] for j in range(M)) if t in TS else 0
			gt = g[i, t] if t in TS else 0
			m.addCons(q[i, t + 1] == q[i, t] - sv + gt)
			if t in TS:
				m.addCons(g[i, t] <= Q * r[i, t])
				m.addCons(sv <= q[i, t])
	for i in range(1, N + 1):
		for j in range(M):
			for t in TS:
				m.addCons(C >= kappa * t * s_[i, j, t])
	for i in range(1, N):
		m.addCons(px[i, 1] <= px[i + 1, 1])
	return m, dict(px=px, py=py, y=y, s=s_, r=r, q=q, C=C, TS=TS, sigma=sigma)


def extract_scip(m, V, N, M, T):
	sol = m.getBestSol()
	P = np.zeros((T + 1, N, 2))
	for i in range(1, N + 1):
		for t in range(T + 1):
			P[t, i - 1] = (m.getSolVal(sol, V["px"][i, t]), m.getSolVal(sol, V["py"][i, t]))
	serv = [(i - 1, j, t) for i in range(1, N + 1) for j in range(M) for t in V["TS"] if m.getSolVal(sol, V["s"][i, j, t]) > 0.5]
	rel = [(i - 1, t) for i in range(1, N + 1) for t in V["TS"] if m.getSolVal(sol, V["r"][i, t]) > 0.5]
	return P, sorted(serv, key=lambda x: x[2]), sorted(rel, key=lambda x: x[1])


def extract(V, N, M, T):
	"""해에서 매듭 위치와 서비스 열을 읽는다."""
	P = np.zeros((T + 1, N, 2))
	for i in range(1, N + 1):
		for t in range(T + 1):
			P[t, i - 1] = (V["px"][i, t].X, V["py"][i, t].X)
	serv = [(i - 1, j, t) for i in range(1, N + 1) for j in range(M) for t in V["TS"] if V["s"][i, j, t].X > 0.5]
	rel = [(i - 1, t) for i in range(1, N + 1) for t in V["TS"] if V["r"][i, t].X > 0.5]
	return P, sorted(serv, key=lambda x: x[2]), sorted(rel, key=lambda x: x[1])


def replay(P, cc, dests, N, Q, R, kappa, v, services=(), reloads=(), tol=None):
	"""매듭 궤적을 연속 환경에서 모형의 의미로 재생해 운동·연결성 실현 가능성을 확인한다.

	환경의 자동 서비스(지나가면 배송, 빈 드론만 재적재)는 끄고(no_service), 모형이 계획한 매듭에서만 배송·재적재를 적용한다.
	확인 항목: 매듭 위치 추종 오차(투영이 이동을 줄였으면 커진다), 서비스 매듭에서 수요지까지 거리(≤ ρ + tol), 단절 수.
	tol 은 다각형 바깥 근사의 허용 오차 (기본 ρ 의 40%).
	"""
	tol = 0.4 * RHO if tol is None else tol
	env = DisasterRelayDroneEnv(map_path="", comm_range=R, cluster_penalty=False, max_steps=20000,
	                            deadlock_limit=10 ** 9, no_progress_limit=10 ** 9, num_drones=N, num_dests=len(dests))
	env.max_capacity = Q
	env.cc_pos, env.dests_pos = cc.copy(), dests.copy()
	env.reset()
	env.no_service = True
	sv_at = {}
	for i, j, t in services:
		sv_at.setdefault(t, []).append((i, j))
	rl_at = {}
	for i, t in reloads:
		rl_at.setdefault(t, []).append(i)
	max_dev, bad_sv, delivered, makespan = 0.0, 0, 0, None
	for t in range(0, P.shape[0]):
		if t > 0:
			tgt = P[t]
			for _ in range(kappa):
				vec = tgt - env.drones_pos
				nrm = np.linalg.norm(vec, axis=1, keepdims=True)
				thr = np.where(nrm > 1e-9, vec / np.maximum(nrm, 1e-9), 0.0) * np.minimum(1.0, nrm / v)
				env.step(thr)
			max_dev = max(max_dev, float(np.max(np.linalg.norm(env.drones_pos - P[t], axis=1))))
		for i, j in sv_at.get(t, []):
			d = float(np.linalg.norm(env.drones_pos[i] - dests[j]))
			if d <= RHO + tol and env.drones_capacity[i] > 0 and env.dests_active[j]:
				env.dests_active[j] = False
				env.drones_capacity[i] -= 1
				delivered += 1
				makespan = t * kappa if makespan is None else max(makespan, t * kappa)
			else:
				bad_sv += 1
		for i in rl_at.get(t, []):
			if np.linalg.norm(env.drones_pos[i] - cc) <= RHO + tol:
				env.drones_capacity[i] = Q
	ok = delivered == len(dests)
	return dict(delivered=delivered, makespan=makespan if ok else None, comm_loss=int(env.comm_loss), max_dev=round(max_dev, 1),
	            bad_services=bad_sv, steps=env.current_step)


def far_counts(cc, dests, R):
	d = np.linalg.norm(dests - cc, axis=1)
	return int(np.sum(d > R)), int(np.sum(d > 2 * 0.9 * R))


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--num-drones", type=int, default=3)
	p.add_argument("--num-dests", type=int, default=9)
	p.add_argument("--capacity", type=int, default=3)
	p.add_argument("--seed", type=int, default=1201)
	p.add_argument("--kappa", type=int, default=10)
	p.add_argument("--comm-range", type=float, default=300.0)
	p.add_argument("--norm", choices=["soc", "poly"], default="soc")
	p.add_argument("--ngon", type=int, default=32, help="다각형 바깥 근사 변 수 (32: 반지름 오차 0.48%%)")
	p.add_argument("--horizon-factor", type=float, default=1.25, help="지평 = 규칙 makespan x 이 값 (기간 단위로 올림) + σ")
	p.add_argument("--horizon", type=int, default=0, help="양수면 기간 수를 직접 지정")
	p.add_argument("--time-limit", type=float, default=3600.0)
	p.add_argument("--threads", type=int, default=4)
	p.add_argument("--mip-gap", type=float, default=1e-3)
	p.add_argument("--out", default="")
	p.add_argument("--select", action="store_true", help="모형을 풀지 않고 인스턴스의 원거리 수요지 수만 출력한다")
	p.add_argument("--solver", choices=["gurobi", "scip"], default="gurobi")
	p.add_argument("--comm-outer", action="store_true", help="통신 반경도 바깥 근사로 둔다 (순수 완화·하한, 해가 환경에서 막힐 수 있다)")
	A = p.parse_args()
	N, M, Q, R, kappa = A.num_drones, A.num_dests, A.capacity, A.comm_range, A.kappa
	cc, dests = sample_instance(M, seed=A.seed, num_drones=N, comm_range=R, cc_pos=None)
	fr, fr2 = far_counts(cc, dests, R)
	if A.select:
		print(json.dumps(dict(seed=A.seed, dests=M, far_R=fr, far_2R=fr2, max_dist=round(float(np.max(np.linalg.norm(dests - cc, axis=1))), 1))))
		return
	ms_rule, v = rule_makespan(cc, dests, N, Q, R)
	sigma = int(math.ceil(SERVICE / kappa))
	T = A.horizon if A.horizon > 0 else int(math.ceil(A.horizon_factor * (ms_rule if ms_rule else 1500) / kappa)) + sigma
	print(f"[연속 모형] N={N} M={M} Q={Q} κ={kappa} σ={sigma} | 규칙 makespan {ms_rule} → 지평 T={T} 기간 ({T * kappa}스텝) | "
	      f"원거리 수요지 >R {fr}, >2·0.9R {fr2} | 노름 {A.norm}{'' if A.norm == 'soc' else A.ngon}", flush=True)
	t0 = time.time()
	if A.solver == "scip":
		m, V = build_scip(cc, dests, N, Q, R, kappa, T, v, norm=A.norm, ngon=A.ngon, threads=A.threads, time_limit=A.time_limit, gap=A.mip_gap)
		build_sec = time.time() - t0
		print(f"  SCIP: 변수 {m.getNVars()} (이진 {m.getNBinVars()}) 제약 {m.getNConss()} | 생성 {build_sec:.0f}초", flush=True)
		m.optimize()
		sec = time.time() - t0
		st = m.getStatus()
		status = {"optimal": "optimal", "timelimit": "time_limit", "infeasible": "infeasible", "gaplimit": "optimal"}.get(st, st)
		res = dict(drones=N, dests=M, capacity=Q, seed=A.seed, kappa=kappa, sigma=sigma, horizon=T, norm=A.norm, ngon=A.ngon, solver="scip",
		           status=status, solve_sec=round(sec, 1), rule_makespan=ms_rule, far_R=fr, far_2R=fr2, num_vars=m.getNVars(), num_bin=m.getNBinVars(), num_constrs=m.getNConss())
		if m.getNSols() > 0:
			P, serv, rel = extract_scip(m, V, N, M, T)
			rep = replay(P, cc, dests, N, Q, R, kappa, v, serv, rel)
			res.update(obj=float(m.getObjVal()), bound=float(m.getDualbound()), gap=float(m.getGap()), replay=rep,
			           services=[(int(i), int(j), int(t)) for i, j, t in serv], reloads=[(int(i), int(t)) for i, t in rel])
			print(f"[결과] {status} | 목적 {m.getObjVal():.0f} 하한 {m.getDualbound():.0f} 격차 {100 * m.getGap():.2f}% ({sec:.0f}초) | 재생: 배송 {rep['delivered']}/{M} "
			      f"makespan {rep['makespan']} 단절 {rep['comm_loss']} 추종오차 {rep['max_dev']}m 불량서비스 {rep['bad_services']} | 규칙 {ms_rule}", flush=True)
		else:
			res.update(obj=None, bound=float(m.getDualbound()) if st != "infeasible" else None, gap=None, replay=None)
			print(f"[결과] {status} | 해 없음 ({sec:.0f}초) | 하한 {res['bound']}", flush=True)
		if A.out:
			with open(os.path.join(ROOT, "figures", f"{A.out}.json"), "w") as fh:
				json.dump(res, fh, ensure_ascii=False, indent=1)
			if m.getNSols() > 0:
				np.save(os.path.join(ROOT, "figures", f"{A.out}_traj.npy"), dict(P=P, cc=cc, dests=dests), allow_pickle=True)
		return
	m, V = build(cc, dests, N, Q, R, kappa, T, v, norm=A.norm, ngon=A.ngon, comm_inner=not A.comm_outer)
	m.Params.TimeLimit = A.time_limit
	m.Params.Threads = A.threads
	m.Params.MIPGap = A.mip_gap
	m.Params.Symmetry = 2
	build_sec = time.time() - t0
	m.update()
	print(f"  변수 {m.NumVars} (이진 {m.NumBinVars}) 제약 {m.NumConstrs} 2차제약 {m.NumQConstrs} | 생성 {build_sec:.0f}초", flush=True)
	m.optimize()
	sec = time.time() - t0
	from gurobipy import GRB
	status = {GRB.OPTIMAL: "optimal", GRB.TIME_LIMIT: "time_limit", GRB.INFEASIBLE: "infeasible"}.get(m.Status, str(m.Status))
	res = dict(drones=N, dests=M, capacity=Q, seed=A.seed, kappa=kappa, sigma=sigma, horizon=T, norm=A.norm, ngon=A.ngon,
	           comm_inner=not A.comm_outer, solver="gurobi",
	           status=status, solve_sec=round(sec, 1), rule_makespan=ms_rule, far_R=fr, far_2R=fr2,
	           num_vars=m.NumVars, num_bin=m.NumBinVars, num_constrs=m.NumConstrs)
	if m.SolCount > 0:
		P, serv, rel = extract(V, N, M, T)
		rep = replay(P, cc, dests, N, Q, R, kappa, v, serv, rel)
		res.update(obj=float(m.ObjVal), bound=float(m.ObjBound), gap=float(m.MIPGap), replay=rep,
		           services=[(int(i), int(j), int(t)) for i, j, t in serv], reloads=[(int(i), int(t)) for i, t in rel])
		print(f"[결과] {status} | 목적 {m.ObjVal:.0f} 하한 {m.ObjBound:.0f} 격차 {100 * m.MIPGap:.2f}% ({sec:.0f}초) | 재생: 배송 {rep['delivered']}/{M} "
		      f"makespan {rep['makespan']} 단절 {rep['comm_loss']} 추종오차 {rep['max_dev']}m 불량서비스 {rep['bad_services']} | 규칙 {ms_rule}", flush=True)
	else:
		res.update(obj=None, bound=float(m.ObjBound) if m.Status != GRB.INFEASIBLE else None, gap=None, replay=None)
		print(f"[결과] {status} | 해 없음 ({sec:.0f}초) | 하한 {res['bound']}", flush=True)
	if A.out:
		with open(os.path.join(ROOT, "figures", f"{A.out}.json"), "w") as fh:
			json.dump(res, fh, ensure_ascii=False, indent=1)
		if m.SolCount > 0:
			np.save(os.path.join(ROOT, "figures", f"{A.out}_traj.npy"), dict(P=P, cc=cc, dests=dests), allow_pickle=True)


if __name__ == "__main__":
	main()
