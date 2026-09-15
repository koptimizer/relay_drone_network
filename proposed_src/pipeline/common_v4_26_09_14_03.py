"""v4 학습·평가·시각화가 공유하는 상위 행동 규칙 모음.

v3에서는 action_mask와 rule_manager가 학습·평가 파일에 각각 복사되어 있어
세대가 갈리며 규칙이 달라지는 일이 있었다. v4에서는 이 파일 하나만 둔다.
중계 기하는 전부 env.relay_slot_for()에 위임해 관측·마스크·실제 목표가 어긋나지 않게 한다.
"""

import numpy as np
import torch


def action_mask(env, device, no_relay=False):
	"""선택 불가능한 상위 행동을 가린다 — 모든 열을 명시적으로 켠다."""
	# v3는 np.ones로 시작해 유지 열을 한 번도 대입하지 않았고, 그 결과 조건과 무관하게
	# 항상 열려 있었다. 중계 열에서 같은 실수를 반복하지 않도록 zeros에서 출발한다.
	n, k = env.num_drones, env.n_cand + 2
	m = np.zeros((n, k), dtype=bool)
	for i in range(n):
		cand = env.candidates(i)
		full = env.drones_capacity[i] == env.max_capacity
		for c in range(env.n_cand):
			m[i, c] = cand[c] >= 0 and env.drones_capacity[i] > 0
		# 적재량이 가득하면 복귀해도 재적재할 것이 없다. v3는 교착 탈출 경로를 남기려고
		# 복귀를 무조건 열어뒀는데, 그 탓에 정책이 '가득한 채 복귀'라는 무의미 행동으로
		# 수렴했다(ep350에서 상위 결정의 51%, 배송 7.25 -> 차단 시 21.75).
		# goal_invalid가 이 조합을 매 스텝 무효로 처리해 리플레이 버퍼까지 오염시킨다.
		m[i, env.n_cand] = not full
		slot, _pt, _n = env.relay_slot_for(i)
		free = sum(env.goal_kind[j] != 3 for j in range(n) if j != i)
		m[i, env.n_cand + 1] = (not no_relay) and slot >= 1 and free >= 1
		if not m[i].any():               # 남는 수가 없으면 복귀로 되돌린다
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
	"""하위 목표를 향해 최대 추력으로 직진한다 (규칙 베이스라인의 저수준 제어)."""
	g = env.goal_pos - env.drones_pos
	n = np.linalg.norm(g, axis=1, keepdims=True)
	return np.where(n > 1e-6, g / np.maximum(n, 1e-6), 0.0)


def chain_manager(env):
	"""기하 중계 규칙 상위: 선두가 닿을 거리에 맞춰 필요한 만큼만 중계로 돌린다."""
	acts = plain_manager(env)
	if not env.dests_active.any():
		return acts
	# 선두 = 적재량이 있는 드론 중 제어 센터에서 가장 먼 기체
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
	# v3의 같은 이름 함수는 목표가 '현 위치 유지'여서 제어 센터에 얹어두는 것과 같았다.
	# v4에서는 같은 행동이 기하 슬롯으로 해석되므로 실제 중계 베이스라인이 된다.
	acts = plain_manager(env)
	for i in range(min(n_relay, env.num_drones)):
		acts[i] = env.n_cand + 1
	return acts


rule_manager = plain_manager   # v3 이름 호환
