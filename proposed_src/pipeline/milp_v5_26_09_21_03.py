"""시공간 격자 MILP 베이스라인 (v5, 26_09_21_03).

연속 평면 위의 중계 드론 배송 문제를 격자 노드 + 단위 시간 이동의 시간 확장 네트워크로
이산화해 Gurobi로 푼다. legacy/milp와 달리 이동 중 위치가 매 기간 정의되므로 연결성 하드
제약을 모든 기간에 걸어 검증할 수 있고, 중계 후보지를 격자 전체로 둬 사전 지정하지 않는다.
"""

import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
from pipeline.common_v5_26_09_15_22 import chain_escape_manager, chain_manager, straight_worker
from util.instance_generator_v5_26_09_15_22 import reach_limit, sample_instance

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def build_nodes(cc, dests, step_len, spacing, comm_range, num_drones, beta=0.9):
	"""제어 센터 0번, 목적지, 격자 순으로 노드 좌표와 색인을 만든다."""
	pts = [np.asarray(cc, dtype=float)]
	pts += [np.asarray(p, dtype=float) for p in dests]
	lim = reach_limit(num_drones, comm_range, beta)
	lo = np.minimum(np.min(dests, axis=0), cc) - spacing
	hi = np.maximum(np.max(dests, axis=0), cc) + spacing
	# 격자 간격은 중계 사슬의 여유(N*R - 최원거리)보다 촘촘해야 한다. 성기면 먼 목적지가
	# 사슬로 닿지 않아 모형이 실행 불가가 된다 (간격 196m에서 791m 목적지 실패).
	nx = int(np.floor((hi[0] - lo[0]) / spacing)) + 1
	ny = int(np.floor((hi[1] - lo[1]) / spacing)) + 1
	grid = []
	for a in range(nx):
		for b in range(ny):
			p = np.array([lo[0] + a * spacing, lo[1] + b * spacing])
			if np.linalg.norm(p - cc) > lim:
				continue                      # 3대로 닿을 수 없는 칸은 중계로도 쓸모가 없다
			if min(np.linalg.norm(p - q) for q in pts) < 0.35 * spacing:
				continue                      # 목적지·센터와 겹치는 칸은 버린다
			grid.append(p)
	pos = np.array(pts + grid)
	return pos, len(dests)


def build_model(pos, n_dest, num_drones, cap, horizon, step_len, comm_range, eps=1e-5,
                symmetry=True, threads=6, time_limit=600.0, verbose=True, mip_gap=0.0,
                ring_cuts=True, mip_focus=1, norel=0.0, cum=False, gseed=0):
	"""시간 확장 격자 위의 makespan 최소화 MILP를 만든다."""
	import gurobipy as gp
	from gurobipy import GRB

	n = len(pos)
	K = range(num_drones)
	T = range(horizon + 1)
	D = range(1, n_dest + 1)
	d = np.linalg.norm(pos[:, None, :] - pos[None, :, :], axis=2)
	move = [np.flatnonzero(d[i] <= step_len + 1e-6) for i in range(n)]      # 자기 자신 포함
	link = [(i, j) for i in range(n) for j in range(n) if i != j and d[i, j] <= comm_range + 1e-6]

	m = gp.Model("relay_milp")
	m.Params.OutputFlag = 1 if verbose else 0
	m.Params.Threads = threads
	m.Params.TimeLimit = time_limit
	m.Params.MIPGap = mip_gap
	m.Params.MIPFocus = mip_focus
	m.Params.Symmetry = 2
	m.Params.Seed = gseed
	if norel > 0.0:
		m.Params.NoRelHeurTime = norel

	y = m.addVars(n, num_drones, horizon + 1, vtype=GRB.BINARY, name="y")
	s = m.addVars(D, num_drones, horizon, vtype=GRB.BINARY, name="s")
	r = m.addVars(num_drones, horizon, vtype=GRB.BINARY, name="r")
	q = m.addVars(num_drones, horizon + 1, lb=0, ub=cap, vtype=GRB.INTEGER, name="q")
	rho = m.addVars(num_drones, horizon, lb=0, ub=cap, name="rho")
	g = m.addVars(link, horizon + 1, lb=0, ub=num_drones, name="g")
	C = m.addVar(lb=0, ub=horizon, name="C")

	m.setObjective(C + eps * gp.quicksum(y[i, k, t] for i in range(1, n) for k in K for t in T),
	               GRB.MINIMIZE)

	for k in K:
		m.addConstr(y[0, k, 0] == 1, name=f"start_{k}")
		m.addConstr(q[k, 0] == cap, name=f"load0_{k}")
		for t in T:
			m.addConstr(gp.quicksum(y[i, k, t] for i in range(n)) == 1, name=f"one_{k}_{t}")
		for t in range(horizon):
			# 이동: 다음 기간 위치는 현재 위치에서 한 기간에 닿는 칸이어야 한다
			for i in range(n):
				m.addConstr(y[i, k, t + 1] <= gp.quicksum(y[j, k, t] for j in move[i]),
				            name=f"mv_{i}_{k}_{t}")
			# 하역·재적재는 한 기간을 점유하고 그 사이 이동할 수 없다
			for j in D:
				m.addConstr(s[j, k, t] <= y[j, k, t], name=f"sat_{j}_{k}_{t}")
				m.addConstr(s[j, k, t] <= y[j, k, t + 1], name=f"shold_{j}_{k}_{t}")
			m.addConstr(r[k, t] <= y[0, k, t], name=f"rat_{k}_{t}")
			m.addConstr(r[k, t] <= y[0, k, t + 1], name=f"rhold_{k}_{t}")
			m.addConstr(gp.quicksum(s[j, k, t] for j in D) + r[k, t] <= 1, name=f"busy_{k}_{t}")
			m.addConstr(gp.quicksum(s[j, k, t] for j in D) <= q[k, t], name=f"cap_{k}_{t}")
			m.addConstr(rho[k, t] <= cap * r[k, t], name=f"rlim_{k}_{t}")
			m.addConstr(q[k, t + 1] == q[k, t] - gp.quicksum(s[j, k, t] for j in D) + rho[k, t],
			            name=f"bal_{k}_{t}")
			m.addConstr(q[k, t + 1] >= cap * r[k, t], name=f"full_{k}_{t}")

	for j in D:
		m.addConstr(gp.quicksum(s[j, k, t] for k in K for t in range(horizon)) == 1, name=f"serve_{j}")
		for k in K:
			for t in range(horizon):
				m.addConstr(C >= t * s[j, k, t], name=f"mk_{j}_{k}_{t}")
	# makespan 누적 선형화 (기본 off): a_t = "기간 t 이전에 전량 배송 완료". LP 하한은 조금
	# 오르지만(목적지 4곳 81 -> 90) 이진 변수가 늘어 잠정해가 크게 나빠졌다 (140 -> 170).
	if cum:
		n_d = len(D)
		a = m.addVars(range(1, horizon + 1), vtype=GRB.BINARY, name="a")
		for t in range(1, horizon + 1):
			m.addConstr(n_d * a[t] <= gp.quicksum(s[j, k, tp] for j in D for k in K for tp in range(t)),
			            name=f"acum_{t}")
			if t < horizon:
				m.addConstr(a[t] <= a[t + 1], name=f"amono_{t}")
		m.addConstr(C == horizon - gp.quicksum(a[t] for t in range(1, horizon + 1)), name="mkdef")
	if symmetry:
		# 동일 드론 대칭 제거: 드론 k가 맡는 최소 목적지 번호는 k 이상으로 둘 수 있다
		for j in D:
			for k in K:
				if j < k + 1:
					for t in range(horizon):
						m.addConstr(s[j, k, t] == 0, name=f"sym_{j}_{k}_{t}")

	# 연결성: 드론이 점유한 노드마다 1단위를 제어 센터로 흘린다 (단일 상품 흐름)
	out = {i: [] for i in range(n)}
	inn = {i: [] for i in range(n)}
	for (i, j) in link:
		out[i].append(j)
		inn[i].append(j)
	for t in T:
		u = {i: gp.quicksum(y[i, k, t] for k in K) for i in range(n)}
		for i in range(1, n):
			m.addConstr(gp.quicksum(g[i, j, t] for j in out[i])
			            - gp.quicksum(g[j, i, t] for j in inn[i]) == u[i], name=f"flow_{i}_{t}")
		for (i, j) in link:
			# 꼬리 노드가 비면 흐를 수 없다. 머리 노드 조건은 보존식에서 유도되므로 생략한다
			m.addConstr(g[i, j, t] <= num_drones * u[i], name=f"ci_{i}_{j}_{t}")
	if ring_cuts:
		# 사슬 하한 절단면: 노드 i에 드론이 있으면 제어 센터를 벗어난 드론이 ceil(d_0i/R)대 이상,
		# 반경 R 밖에도 ceil((d_0i-R)/R)대 이상 있어야 한다 (사슬의 각 고리가 서로 다른 드론이다).
		near = [j for j in range(n) if d[0, j] <= comm_range + 1e-6]
		for i in range(1, n):
			h1 = int(np.ceil(d[0, i] / comm_range - 1e-9))
			if h1 < 2:
				continue
			h2 = int(np.ceil((d[0, i] - comm_range) / comm_range - 1e-9))
			for t in T:
				away = num_drones - gp.quicksum(y[0, k, t] for k in K)
				far = num_drones - gp.quicksum(y[j, k, t] for j in near for k in K)
				for k in K:
					m.addConstr(away >= h1 * y[i, k, t], name=f"ring1_{i}_{k}_{t}")
					if h2 >= 2:
						m.addConstr(far >= h2 * y[i, k, t], name=f"ring2_{i}_{k}_{t}")
	m.update()
	return m, dict(y=y, s=s, r=r, q=q, g=g, C=C, d=d, move=move, link=link)


def extract_plan(v, pos, num_drones, horizon):
	"""해에서 드론별 기간 위치 색인을 뽑는다."""
	y = v["y"]
	plan = np.zeros((horizon + 1, num_drones), dtype=int)
	for t in range(horizon + 1):
		for k in range(num_drones):
			for i in range(len(pos)):
				if y[i, k, t].X > 0.5:
					plan[t, k] = i
					break
	return plan


def replay(plan, pos, cc, dests, num_drones, kappa, comm_range):
	"""MILP 경로를 실제 환경에 태워 makespan과 제약 위반을 확인한다."""
	env = DisasterRelayDroneEnv(num_drones=num_drones, num_dests=len(dests), max_steps=10000,
	                            cluster_penalty=False, deadlock_limit=10 ** 9,
	                            no_progress_limit=10 ** 9, comm_range=comm_range)
	env.cc_pos, env.dests_pos = np.asarray(cc, dtype=float), np.asarray(dests, dtype=float)
	env.reset()
	blocked, done, t = 0, False, 0
	for p in range(1, plan.shape[0]):
		tgt = pos[plan[p]]
		for _ in range(kappa):
			if done:
				break
			thrust = np.zeros((num_drones, 2))
			for k in range(num_drones):
				vec = tgt[k] - env.drones_pos[k]
				nrm = np.linalg.norm(vec)
				thrust[k] = vec / nrm if nrm > 1e-6 else 0.0
			before = env.drones_pos.copy()
			_o, _w, _tr, done = env.step(thrust)
			t += 1
			moved = np.linalg.norm(env.drones_pos - before, axis=1)
			want = np.minimum(np.linalg.norm(tgt - before, axis=1), env.max_speed)
			blocked += int(np.sum(want - moved > 1.0))
		if done:
			break
	st = env.episode_stats()
	return dict(steps=t, delivered=st["delivered"], makespan=st["makespan"], blocked=blocked,
	            comm_loss=st["comm_loss"])


def heuristic_makespan(cc, dests, num_drones, comm_range, escape=True):
	"""규칙 베이스라인의 makespan (지평선 상한용)."""
	env = DisasterRelayDroneEnv(num_drones=num_drones, num_dests=len(dests), max_steps=10000,
	                            cluster_penalty=False, deadlock_limit=10 ** 9,
	                            no_progress_limit=10 ** 9, comm_range=comm_range)
	env.cc_pos, env.dests_pos = np.asarray(cc, dtype=float), np.asarray(dests, dtype=float)
	env.reset()
	mf = chain_escape_manager(60, 40, "shuffle") if escape else chain_manager
	env.set_goals(mf(env))
	done, t = False, 0
	while not done and t < env.max_steps:
		_o, _w, _tr, done = env.step(straight_worker(env))
		t += 1
		if t % 20 == 0 or env.goal_invalid().any():
			env.set_goals(mf(env))
	return env.episode_stats()


def main():
	"""인스턴스를 만들고 MILP를 풀어 규칙 베이스라인과 비교한다."""
	p = argparse.ArgumentParser()
	p.add_argument("--num-drones", type=int, default=3)
	p.add_argument("--num-dests", type=int, default=10)
	p.add_argument("--seed", type=int, default=901)
	p.add_argument("--comm-range", type=float, default=300.0)
	p.add_argument("--kappa", type=int, default=10, help="한 기간의 환경 스텝 수 (하역 시간 10과 맞춤)")
	p.add_argument("--spacing", type=float, default=0.0, help="격자 간격. 0이면 기간 이동거리/sqrt(2)")
	p.add_argument("--horizon", type=int, default=0, help="기간 수. 0이면 규칙 makespan에서 자동")
	p.add_argument("--slack", type=float, default=1.15, help="자동 지평선 여유 배수")
	p.add_argument("--time-limit", type=float, default=900.0)
	p.add_argument("--threads", type=int, default=6)
	p.add_argument("--mip-gap", type=float, default=0.0)
	p.add_argument("--no-symmetry", action="store_true")
	p.add_argument("--no-ring-cuts", action="store_true")
	p.add_argument("--mip-focus", type=int, default=1)
	p.add_argument("--gurobi-seed", type=int, default=0, help="같은 모형도 탐색 편차가 커서 시드를 바꿔 반복한다")
	p.add_argument("--cumulative", action="store_true", help="makespan 누적 선형화 (실측 역효과라 기본 off)")
	p.add_argument("--norel", type=float, default=0.0, help="완화 없이 해를 찾는 휴리스틱에 줄 초")
	p.add_argument("--map-size", type=float, default=1000.0)
	p.add_argument("--cc", type=float, nargs=2, default=None)
	p.add_argument("--quiet", action="store_true")
	p.add_argument("--out", default="milp_v5_26_09_21_03")
	A = p.parse_args()

	cc_arg = list(A.cc) if A.cc else None
	cc, dests = sample_instance(A.num_dests, seed=A.seed, num_drones=A.num_drones,
	                            comm_range=A.comm_range, cc_pos=cc_arg, map_size=A.map_size)
	env0 = DisasterRelayDroneEnv(num_drones=A.num_drones, num_dests=A.num_dests)
	speed, cap = env0.max_speed, env0.max_capacity
	step_len = speed * A.kappa
	spacing = A.spacing if A.spacing > 0 else step_len / np.sqrt(2.0)

	hs = heuristic_makespan(cc, dests, A.num_drones, A.comm_range)
	print(f"[규칙+탈출] 배송 {hs['delivered']}/{A.num_dests} makespan {hs['makespan']} 스텝", flush=True)
	base = hs["makespan"] if hs["makespan"] else 1200
	horizon = A.horizon if A.horizon > 0 else int(np.ceil(A.slack * base / A.kappa)) + 1

	pos, n_dest = build_nodes(cc, dests, step_len, spacing, A.comm_range, A.num_drones)
	print(f"[격자] 기간 {A.kappa}스텝({A.kappa}초) 이동거리 {step_len:.1f}m 간격 {spacing:.1f}m "
	      f"노드 {len(pos)}개 (목적지 {n_dest}, 격자 {len(pos)-n_dest-1}) 지평선 {horizon}기간", flush=True)

	t0 = time.time()
	m, v = build_model(pos, n_dest, A.num_drones, cap, horizon, step_len, A.comm_range,
	                   symmetry=not A.no_symmetry, threads=A.threads, time_limit=A.time_limit,
	                   verbose=not A.quiet, mip_gap=A.mip_gap, ring_cuts=not A.no_ring_cuts,
	                   mip_focus=A.mip_focus, norel=A.norel, cum=A.cumulative, gseed=A.gurobi_seed)
	print(f"[모형] 변수 {m.NumVars} (이진 {m.NumBinVars}) 제약 {m.NumConstrs} 생성 {time.time()-t0:.0f}초", flush=True)
	m.optimize()
	if m.SolCount == 0:
		print("[결과] 실행 가능해를 찾지 못했다 (지평선을 늘려야 한다)", flush=True)
		return
	plan = extract_plan(v, pos, A.num_drones, horizon)
	C = float(v["C"].X)
	rep = replay(plan, pos, cc, dests, A.num_drones, A.kappa, A.comm_range)
	res = dict(drones=A.num_drones, dests=A.num_dests, seed=A.seed, kappa=A.kappa,
	           spacing=round(spacing, 1), nodes=len(pos), horizon=horizon,
	           binaries=int(m.NumBinVars), variables=int(m.NumVars), rows=int(m.NumConstrs),
	           status=int(m.Status), gap=round(float(m.MIPGap), 4),
	           milp_periods=round(C, 2), milp_steps=round(C * A.kappa, 1),
	           bound_steps=round(float(m.ObjBound) * A.kappa, 1),
	           solve_sec=round(float(m.Runtime), 1),
	           rule_makespan=hs["makespan"], rule_delivered=hs["delivered"],
	           replay=rep)
	print("\n=== 결과 ===", flush=True)
	print(f"MILP makespan {res['milp_steps']:.0f} 스텝 (하한 {res['bound_steps']:.0f}, 갭 {100*res['gap']:.1f}%) "
	      f"풀이 {res['solve_sec']:.0f}초", flush=True)
	print(f"환경 재생: 배송 {rep['delivered']}/{A.num_dests} makespan {rep['makespan']} "
	      f"막힘 {rep['blocked']} 통신단절 {rep['comm_loss']}", flush=True)
	print(f"규칙+탈출: makespan {hs['makespan']}", flush=True)
	os.makedirs(os.path.join(ROOT, "figures"), exist_ok=True)
	jp = os.path.join(ROOT, "figures", f"{A.out}.json")
	np.save(os.path.join(ROOT, "figures", f"{A.out}_plan.npy"),
	        dict(plan=plan, pos=pos, cc=cc, dests=dests), allow_pickle=True)
	with open(jp, "w") as f:
		json.dump(res, f, ensure_ascii=False, indent=1)
	print(f"저장: {jp}", flush=True)


if __name__ == "__main__":
	main()
