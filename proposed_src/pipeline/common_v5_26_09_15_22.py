"""v5 학습·평가·시각화가 공유하는 상위 행동 규칙 모음.

v4와 다른 점은 목표 유지(commitment) 마스크다. 아직 유효한 목표로 가는 중인 드론은
그 목표에 해당하는 행동만 남기고 나머지를 가린다. 학습 정책은 지정 목표에 닿기 전에
88.7%가 목표를 바꿔 이동을 낭비했다(기하 규칙은 37%). 마스크로 구현하면 취한 행동이
행동 공간 안에 있어 학습 전이가 일관되고, 선택지가 하나뿐이라 정책 기울기도 0이다.
"""

import numpy as np
import torch

RELAY = 3


def _base_mask(env, i, no_relay):
	"""드론 i가 지금 고를 수 있는 행동 열."""
	k = env.n_cand + 2
	m = np.zeros(k, dtype=bool)
	cand = env.candidates(i)
	full = env.drones_capacity[i] == env.max_capacity
	for c in range(env.n_cand):
		m[c] = cand[c] >= 0 and env.drones_capacity[i] > 0
	# 적재량이 가득하면 복귀해도 재적재할 것이 없다. v4에서 이 무의미 행동이
	# 항상 열려 있어 정책이 거기로 수렴하며 붕괴했다.
	m[env.n_cand] = not full
	slot, _pt, _n = env.relay_slot_for(i)
	free = sum(env.goal_kind[j] != RELAY for j in range(env.num_drones) if j != i)
	m[env.n_cand + 1] = (not no_relay) and slot >= 1 and free >= 1
	return m, cand


def _commit_mask(env, i, base, cand):
	"""유효한 목표로 가는 중이면 그 목표에 해당하는 행동만 남긴다. 못 찾으면 None."""
	k = env.goal_kind[i]
	keep = np.zeros_like(base)
	if k == 0:                                     # 배송: 현재 목적지가 후보에 있어야 한다
		hit = np.flatnonzero(cand == env.goal_dest[i])
		if len(hit) == 0 or not base[hit[0]]:
			return None
		keep[hit[0]] = True
	elif k == 1 and base[env.n_cand]:              # 복귀
		keep[env.n_cand] = True
	elif k == RELAY and base[env.n_cand + 1]:      # 중계
		keep[env.n_cand + 1] = True
	else:
		return None
	return keep


def action_mask(env, device, no_relay=False, commit=False):
	"""선택 불가능한 상위 행동을 가린다 — 모든 열을 명시적으로 켠다. commit 기본값은 off (켜면 -6.5)."""
	n = env.num_drones
	m = np.zeros((n, env.n_cand + 2), dtype=bool)
	for i in range(n):
		base, cand = _base_mask(env, i, no_relay)
		row = None
		if commit and env.committed(i):
			row = _commit_mask(env, i, base, cand)
		m[i] = base if row is None else row
		if not m[i].any():                         # 남는 수가 없으면 복귀로 되돌린다
			m[i, env.n_cand] = True
	return torch.as_tensor(m, device=device)


def plain_manager(env):
	"""규칙 상위: 적재량이 있으면 최근접 미배송지, 없으면 복귀. 중계는 쓰지 않는다."""
	acts = np.zeros(env.num_drones, dtype=int)
	for i in range(env.num_drones):
		if env.drones_capacity[i] == 0:
			acts[i] = env.n_cand
		elif env.candidates(i)[0] < 0:
			acts[i] = env.n_cand + 1
		else:
			acts[i] = 0
	return acts


def straight_worker(env):
	"""하위 목표를 향해 최대 추력으로 직진한다."""
	g = env.goal_pos - env.drones_pos
	n = np.linalg.norm(g, axis=1, keepdims=True)
	return np.where(n > 1e-6, g / np.maximum(n, 1e-6), 0.0)


def chain_manager(env):
	"""기하 중계 규칙 상위: 선두가 닿을 거리에 맞춰 필요한 만큼만 중계로 돌린다."""
	acts = plain_manager(env)
	if not env.dests_active.any():
		return acts
	reach = np.linalg.norm(env.drones_pos - env.cc_pos, axis=1)
	tip = int(np.argmax(reach - 1e6 * (env.drones_capacity == 0)))
	goal = env.candidates(tip)[0]
	d = max(float(reach[tip]),
	        float(np.linalg.norm(env.dests_pos[goal] - env.cc_pos)) if goal >= 0 else 0.0)
	need = max(1, int(np.ceil(d / (env.relay_beta * env.comm_range)))) - 1
	need = min(need, env.num_drones - 1)
	order = [i for i in np.argsort(-reach) if i != tip]
	for i in order[:need]:
		acts[i] = env.n_cand + 1
	return acts


def hold_relay_manager(env, n_relay):
	"""앞의 n_relay대를 중계 슬롯에 세우고 나머지는 규칙대로 배송/복귀한다."""
	acts = plain_manager(env)
	for i in range(min(n_relay, env.num_drones)):
		acts[i] = env.n_cand + 1
	return acts


def hybrid_manager(learned_fn, k_stall=60, hold=100, max_escapes=0, np_limit=0):
	"""혼합 상위: 평소엔 기하 규칙, 부분 교착이 k_stall스텝 이어지면 hold스텝 동안 학습 정책에 맡긴다.

	규칙은 빠르지만 결정론적이라 교착에서 같은 배정을 반복하고, 학습 정책은 완주하지만 느리다.
	탈출이 max_escapes회를 넘거나 무배송이 np_limit스텝을 넘으면 학습 정책에 영구 이관한다 —
	120 롤아웃에서 단순 혼합은 97.5%에 그쳤고(3건 실패), 완주 보장은 학습 정책 쪽에 있다.
	"""
	st = {"esc": 0, "n": 0, "perm": False}

	def fn(env):
		if env.current_step <= 1:
			st["esc"], st["n"], st["perm"] = 0, 0, False
		if st["perm"] or (np_limit and env.no_progress >= np_limit):
			st["perm"] = True
			return learned_fn(env)
		if env.deadlock_run >= k_stall and st["esc"] == 0:
			st["n"] += 1
			if max_escapes and st["n"] > max_escapes:
				st["perm"] = True
				return learned_fn(env)
			st["esc"] = hold
		if st["esc"] > 0:
			st["esc"] -= 1
			return learned_fn(env)
		return chain_manager(env)
	return fn


def chain_escape_manager(k_stall=60, hold=40, mode="retreat"):
	"""기하 규칙 + 교착 탈출: 부분 교착이 k_stall스텝 이어지면 hold스텝 동안 탈출 기동을 한다.

	retreat: 선두(적재 있고 CC에서 가장 먼 드론)를 제외한 전원이 제어 센터 쪽으로 물러나 사슬을 느슨하게 한다.
	shuffle: 선두를 바꿔 다른 드론이 먼 목적지를 맡고 나머지가 중계로 재배치된다.
	교착의 원인이 결정론적 반복이므로, 배정을 한 번 흔드는 것만으로 대부분 풀린다.
	"""
	st = {"esc": 0, "n": 0}

	def fn(env):
		if env.current_step <= 1:
			st["esc"], st["n"] = 0, 0
		if env.deadlock_run >= k_stall and st["esc"] == 0:
			st["esc"], st["n"] = hold, st["n"] + 1
		acts = chain_manager(env)
		if st["esc"] > 0:
			st["esc"] -= 1
			reach = np.linalg.norm(env.drones_pos - env.cc_pos, axis=1)
			tip = int(np.argmax(reach - 1e6 * (env.drones_capacity == 0)))
			if mode == "retreat":
				for i in range(env.num_drones):
					if i != tip:
						acts[i] = env.n_cand
			else:
				# 선두를 n번째로 먼 드론으로 바꾸고 원래 선두는 물러난다
				order = list(np.argsort(-reach))
				new_tip = order[min(st["n"], len(order) - 1)]
				acts[tip] = env.n_cand
				if env.drones_capacity[new_tip] > 0 and env.candidates(new_tip)[0] >= 0:
					acts[new_tip] = 0
		return acts
	return fn


rule_manager = plain_manager   # v3 이름 호환
