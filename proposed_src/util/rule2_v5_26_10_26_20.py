"""선견 규칙(rollout rule): 규칙의 목표 선택을 한 수 앞 롤아웃으로 고른다.

결정 시점마다 후보 배정(적재가 있는 드론 i 에 후보 수요지 j 를 맡기고 나머지는 규칙대로, 그리고 중계
선행 배치 켜기/끄기)을 열거하고, 각 배정을 규칙(사슬 + 교착 복구)으로 끝까지 굴려 본 makespan 이 가장
짧은 배정을 택한다. 규칙 자체는 그대로이므로 '규칙 + 한 수 선견' 이며, 사전 정보나 학습이 필요 없다.
격자 결정 공간에서 돌려 모형 최적값과 같은 기준으로 잰다.
"""

import argparse
import copy
import csv
import glob
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v5_26_09_15_22 import GOAL_DELIVER, GOAL_RELAY, DisasterRelayDroneEnv
from pipeline.common_v5_26_09_15_22 import chain_escape_manager
from pipeline.milp_v5_26_09_21_03 import build_nodes
from util.instance_generator_v5_26_09_15_22 import sample_instance
from util.lattice_eval_v5_26_10_20_17 import hop_table, lattice_jam, lattice_rollout, lattice_step, serve_at_boundary

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def continue_lattice(env, node, setter, pos, A, hop, kappa, hl_every=20, horizon=None):
	"""현재 상태에서 격자 모드로 끝까지 (horizon 이 있으면 그 스텝까지) 굴려 makespan(미완주면 None)을 돌려준다."""
	lim = env.max_steps if horizon is None else min(env.max_steps, env.current_step + horizon)
	jam_run = 0
	while env.makespan is None and env.current_step < lim:
		serve_at_boundary(env, node, node, kappa)
		if env.makespan is not None:
			break
		tgt = lattice_step(env, node, pos, A, hop, env.cc_pos, env.comm_range)
		jam = lattice_jam(env, node, tgt, pos)
		stp = (pos[tgt] - pos[node]) / (kappa * env.max_speed)
		for _ in range(kappa):
			env.step(stp)
			jam_run = jam_run + 1 if jam else 0; env.deadlock_run = jam_run
			if env.current_step % hl_every == 0 or env.goal_invalid().any():
				setter(env)
		node = tgt
		env.drones_pos[:] = pos[node]
	return env.makespan


def resolved(env, acts):
	"""행동 열을 (수요지 색인 또는 -1) 로 푼다."""
	d = -np.ones(env.num_drones, dtype=int)
	for i, a in enumerate(acts):
		if 0 <= a < env.n_cand:
			d[i] = env.candidates(i)[a]
	return d


def follow_relays(env):
	"""중계 드론의 목표를 슬롯 대신 선두의 목표점으로 바꾼다 (따라가기 중계).

	격자 이동기(또는 연속 공간의 투영)가 연결을 지키므로, 중계는 사슬이 허용하는 만큼만 앞으로 가고
	선두의 목표가 바뀌면 즉시 따라 움직인다. 슬롯에 서서 기다리는 구조를 없애는 것이 목적이다.
	"""
	live = [i for i in range(env.num_drones) if env.goal_kind[i] == GOAL_DELIVER]
	if not live:
		return
	tip = max(live, key=lambda i: float(np.linalg.norm(env.goal_pos[i] - env.cc_pos)))
	for i in range(env.num_drones):
		if env.goal_kind[i] == GOAL_RELAY:
			env.goal_pos[i] = env.goal_pos[tip].copy()


def rule_setter(mf, follow=False, pin=None):
	"""규칙 결정 함수를 설정자로 감싼다. pin=(i, j) 면 드론 i 가 j 를 배송할 때까지 그 배정을 유지한다."""
	def setter(env):
		acts = np.asarray(mf(env)).copy()
		dests = None
		if pin is not None:
			i, j = pin
			if env.dests_active[j] and env.drones_capacity[i] > 0:
				dests = -np.ones(env.num_drones, dtype=int)
				acts[i], dests[i] = 0, j
		env.set_goals(acts, dests)
		if follow:
			follow_relays(env)
	return setter


HORIZON = 1500          # 롤아웃 상한 (스텝). 미완주면 상한 + 50 x 남은 수요지 수로 값을 매긴다
RET_OPT = False         # 참이면 '지금 재적재하러 복귀' 도 후보에 넣는다
UNION = False           # 참이면 공동 열거와 단일 덮어쓰기 열거(+복귀)를 합친다
JOINT = 0               # 양수면 드론별 가까운 JOINT 곳·중계·복귀의 곱집합을 공동 열거한다 (단일 덮어쓰기 열거 대신)


def lookahead_setter(pos, A, hop, kappa, k_cand=5, relay_opts=(False, True), log=None, follow=False, commit=False,
                     base_factory=None):
	"""한 수 선견 설정자: 후보 배정을 기본 정책 롤아웃으로 평가해 가장 좋은 것을 env 에 건다.

	base_factory() 가 기본 정책(결정 함수)을 만든다. 기본값은 규칙(사슬 + 교착 복구)이고, 학습 정책을 주면
	'학습 + 탐색' 혼합이 된다 (후보 제안과 롤아웃 평가를 모두 학습 정책이 한다).
	"""
	base_factory = chain_escape_manager if base_factory is None else base_factory
	base = base_factory()

	# follow: False/True 면 고정, "opt" 면 중계 방식(슬롯/따라가기)도 결정마다 후보로 열거한다
	follow_opts = (False, True) if follow == "opt" else (bool(follow),)
	st = {"pin": None, "follow": follow_opts[0]}

	def setter(env):
		N = env.num_drones
		node = np.array([int(np.argmin(np.linalg.norm(pos - env.drones_pos[i], axis=1))) for i in range(N)])
		acts0 = np.asarray(base(env)).copy()
		d0 = resolved(env, acts0)
		opts = [(acts0, None, bool(env.relay_lookahead), None, st["follow"])]
		if JOINT > 0:
			# 공동 열거: 적재가 있는 드론마다 {가까운 JOINT 곳, 중계, (적재가 덜 찼으면) 복귀} 를 고르고 곱집합을 본다.
			# 규칙은 드론마다 독립으로 최근접을 고르므로 같은 수요지로 여럿이 몰린다. 이를 선견이 풀게 한다.
			import itertools
			ch = []
			for i in range(N):
				if env.drones_capacity[i] == 0:
					ch.append([(env.n_cand, -1)])
					continue
				c = [(0, int(x)) for x in env.candidates(i)[:JOINT] if x >= 0] + [(env.n_cand + 1, -1)]
				if env.drones_capacity[i] < env.max_capacity:
					c.append((env.n_cand, -1))
				ch.append(c)
			for combo in itertools.product(*ch):
				ds = [j for a, j in combo if a == 0]
				if len(ds) != len(set(ds)) or not ds:
					continue
				acts = np.array([a for a, _j in combo], dtype=int); dests = np.array([j for _a, j in combo], dtype=int)
				for fo in follow_opts:
					opts.append((acts, dests, bool(env.relay_lookahead), None, fo))
		for fo in ((() if (JOINT > 0 and not UNION) else follow_opts)):
			for i in range(N):
				if env.drones_capacity[i] == 0:
					continue
				for j in [int(x) for x in env.candidates(i)[:k_cand] if x >= 0]:
					if j == d0[i] or j in d0:
						continue
					acts = acts0.copy(); dests = -np.ones(N, dtype=int)
					acts[i], dests[i] = 0, j
					for la in relay_opts:
						opts.append((acts, dests, la, (i, j) if commit else None, fo))
			for la in relay_opts:
				if la != bool(env.relay_lookahead) or fo != st["follow"]:
					opts.append((acts0, None, la, None, fo))
			if RET_OPT:                      # 적재가 남았어도 지금 재적재하러 돌아가는 선택지 (재적재 시점 결정)
				for i in range(N):
					if 0 < env.drones_capacity[i] < env.max_capacity and acts0[i] != env.n_cand:
						acts = acts0.copy(); acts[i] = env.n_cand
						opts.append((acts, None, bool(env.relay_lookahead), None, fo))
		best, bv = None, None
		for k, (acts, dests, la, pin, fo) in enumerate(opts):
			e = copy.deepcopy(env)
			e.relay_lookahead = la
			e.set_goals(acts, dests)
			if fo:
				follow_relays(e)
			ms = continue_lattice(e, node.copy(), rule_setter(base_factory(), fo, pin), pos, A, hop, kappa,
			                      horizon=HORIZON)
			v = ms if ms is not None else e.current_step + 50.0 * float(e.dests_active.sum())
			if bv is None or v < bv:
				best, bv = k, v
		acts, dests, la, pin, fo = opts[best]
		if log is not None:
			log.append((env.current_step, len(opts), best, bv))
		env.relay_lookahead = la
		env.set_goals(acts, dests)
		if fo:
			follow_relays(env)
		st["pin"], st["follow"] = pin, fo

	def outer(env):
		"""유지 중인 배정이 아직 유효하면 다시 평가하지 않고 이어 간다."""
		if env.current_step <= 1:
			st["pin"], st["follow"] = None, follow_opts[0]
		pin = st["pin"]
		if commit and pin is not None and env.dests_active[pin[1]] and env.drones_capacity[pin[0]] > 0 \
		   and env.goal_kind[pin[0]] == GOAL_DELIVER and env.goal_dest[pin[0]] == pin[1]:
			rule_setter(base, st["follow"], pin)(env)
			return
		setter(env)
	return outer


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--pattern", default="bfs2_n3m*_h98_26_10_21_03.json")
	p.add_argument("--out", default="rule2_v5_26_10_26_20")
	p.add_argument("--no-relay-opts", action="store_true", help="중계 선행 배치 토글을 후보에서 뺀다")
	p.add_argument("--follow", action="store_true", help="중계가 슬롯 대신 선두의 목표를 따라간다")
	p.add_argument("--follow-opt", action="store_true", help="중계 방식(슬롯/따라가기)을 결정마다 선견으로 고른다")
	p.add_argument("--ret-opt", action="store_true", help="적재가 남은 드론의 조기 복귀(재적재)도 후보에 넣는다")
	p.add_argument("--joint", type=int, default=0, help="양수면 드론별 가까운 k 곳·중계·복귀의 곱집합을 공동 열거한다")
	p.add_argument("--union", action="store_true", help="공동 열거에 단일 덮어쓰기 열거(+복귀)를 합친다")
	p.add_argument("--manager", default="", help="주면 기본 정책을 규칙 대신 이 학습 가중치로 한다 (학습 + 탐색)")
	p.add_argument("--reps", type=int, default=1, help="학습 기본 정책일 때 전체 롤아웃 반복 수 (중앙값 보고)")
	p.add_argument("--commit", action="store_true", help="선견으로 고른 배정을 배송까지 유지한다 (롤아웃 안에서도)")
	A_ = p.parse_args()
	global RET_OPT, JOINT, UNION
	RET_OPT = A_.ret_opt
	JOINT = A_.joint
	UNION = A_.union
	bf = None
	if A_.manager:
		import torch
		from util.paired_v5_26_09_27_08 import learned
		torch.set_num_threads(2)
		lf = learned(A_.manager, torch.device("cpu"), True)
		bf = lambda: lf
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
		t0 = time.time()
		rule = lattice_rollout(env, rule_setter((bf or chain_escape_manager)(), A_.follow), seed, pos, A, kappa, hop)
		t1 = time.time()
		log = []
		ro = (False,) if A_.no_relay_opts else (False, True)
		env.relay_lookahead = False
		fo = "opt" if A_.follow_opt else A_.follow
		vals = []
		for _ in range(A_.reps):
			vals.append(lattice_rollout(env, lookahead_setter(pos, A, hop, kappa, relay_opts=ro, log=log, follow=fo,
			                                                  commit=A_.commit, base_factory=bf), seed, pos, A, kappa, hop))
		ok = [v for v in vals if v is not None]
		la = int(np.median(ok)) if ok else None
		env.relay_lookahead = False
		sec = time.time() - t1
		rows.append(dict(drones=N, dests=M, seed=seed, capacity=env.max_capacity, opt=d["milp_steps"], rule=rule,
		                 lookahead=la, decisions=len(log), rollouts=sum(x[1] for x in log), sec=round(sec, 1)))
		print(f"M={M} s{seed}: 최적 {d['milp_steps']:.0f} | 규칙 {rule} | 선견 규칙 {la} "
		      f"({len(log)}결정, {sum(x[1] for x in log)}롤아웃, {sec:.0f}초)", flush=True)
	out = os.path.join(ROOT, "figures", f"{A_.out}.csv")
	with open(out, "w", newline="", encoding="utf-8") as fh:
		w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
	print("저장:", out, flush=True)


if __name__ == "__main__":
	main()
