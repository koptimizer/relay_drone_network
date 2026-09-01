"""재난 릴레이 드론 환경 (v2, 26_08_25_14).

v1 최신본과 기본 동작이 동일하며, 붕괴 원인 격리 실험용 플래그 두 개가 추가되었다.
n_peers는 관측할 동료 수, relay_credit은 팀 보상을 중계 기여도로 재분배할지 여부다.
"""

import os
import warnings

import numpy as np
import pygame
from gymnasium import spaces

warnings.filterwarnings("ignore", category=UserWarning, module="pygame")

# 기본 지도: proposed_src/util/disaster_map.npy (CWD와 무관하게 해석)
DEFAULT_MAP = os.path.join(
	os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "util", "disaster_map.npy"
)

# --- 2. Environment ---
class DisasterRelayDroneEnv:
	# relay_w > 0이면 중계 사슬 형성에 potential 기반 쉐이핑을 건다.
	# 기본값 0.0은 이 항이 없던 26_08_24_22 실행과 정확히 동일한 동작이다.
	def __init__(self, map_path=DEFAULT_MAP, render_mode=None, relay_w=0.0,
	             n_peers=2, relay_credit=False, comm_range=300.0, action_mode="angle",
	             projection="scale"):
		self.relay_w = relay_w
		# angle : (각도[0,2pi], 속도비[0,1]) — v1 방식. 각도가 SO(2)의 불연속 표현이고,
		#         속도 1.0에 tanh 포화가 필요해 엔트로피 항이 최대 속도를 억제한다(실측 0.511).
		# vector: (vx, vy) in [-1,1]^2를 그대로 속도 벡터로 쓴다. 이음매가 없고,
		#         최대 속도가 포화 없이 도달 가능하다(예: (0.707, 0.707)).
		self.action_mode = action_mode
		# cancel: 연결을 끊는 이동을 통째로 취소 (v2 초기 구현). 13.89 중 10만 안전해도 0을 간다.
		#         실측 결과 드론-스텝의 74%가 이 상태로 묶여 정책이 느리게 가는 법을 학습했다.
		# scale : 연결이 유지되는 최대 변위까지 축소 (최소 섭동 투영). 문헌 권고안.
		self.projection = projection
		self.n_peers = n_peers            # 관측할 동료 수 (기본 2 = v1과 동일)
		self.relay_credit = relay_credit  # 팀 보상을 중계 필수 드론에 가중 재분배
		self.rng = np.random.default_rng()
		self.num_drones = 4
		self.num_dests = 50
		self.map_size = (1000, 1000)
		self.max_capacity = 5
		# 통신 반경. 500에서는 탐욕 휴리스틱이 50/50을 완주해 연구가 주장할 영역이 없다.
		# 난이도 스윕 결과 300이 (a) 탐욕이 22.7/50으로 실패하고 (b) 인스턴스마다 최적
		# 중계 대수가 달라 고정 규칙이 원리적으로 못 맞추는 구간이다.
		self.comm_range = float(comm_range)
		# 아래 두 임계값은 통신 반경에 대한 비율로 정의한다. 비율은 반경 500 시절 값
		# (150/500, 300/500, 480/500)을 그대로 유지하므로 comm_range=500이면 동작이 동일하다.
		self.cluster_frac = 0.30        # 이보다 가까우면 중복 배치로 보고 페널티
		self.relay_band = (0.60, 0.96)  # 중계에 유용한 간격 구간
		self.max_speed = 50.0 * (1000 / 3600)
		self.unload_time = 10
		self.interaction_radius = 25.0
		self.shape_reward_scale = 0.010

		if os.path.exists(map_path):
			map_data = np.load(map_path, allow_pickle=True).item()
			self.cc_pos = map_data['cc_pos']
			self.dests_pos = map_data['dests_pos']
			print(f"지도를 성공적으로 로드했습니다: {map_path}")
		else:
			self.cc_pos = np.array([950.0, 500.0])
			self.dests_pos = np.random.uniform(low=[50, 50], high=[850, 950], size=(self.num_dests, 2))
			print("지도를 찾을 수 없어 임시 지도를 생성했습니다.")

		self.action_space = spaces.Box(
			low=np.array([0.0, 0.0], dtype=np.float32),
			high=np.array([2 * np.pi, 1.0], dtype=np.float32),
			dtype=np.float32
		)

		self.render_mode = render_mode
		self.screen = None
		self.clock = None
		self.render_fps = 120
		self.radius_surface = None
		self.font = None

	def reset(self):
		self.dests_active = np.ones(self.num_dests, dtype=bool)
		self.drones_pos = np.tile(self.cc_pos, (self.num_drones, 1))
		self.drones_capacity = np.full(self.num_drones, self.max_capacity)
		self.drones_timer = np.zeros(self.num_drones)
		self.comm_status = np.ones(self.num_drones, dtype=bool)
		self.prev_target_dists = self._get_target_dists()
		self.current_step = 0

		# 에피소드 지표: 드론별 기여도와 연결 제약 작동 여부를 추적한다
		self.deliveries = np.zeros(self.num_drones, dtype=int)
		self.reloads = np.zeros(self.num_drones, dtype=int)
		self.blocked = np.zeros(self.num_drones, dtype=int)
		self.makespan = None
		self.t_last_deliv = 0
		self.comm_loss = False
		self.hop_sum = 0.0
		self.hop2plus = 0
		self.max_reach = 0.0
		self.prev_phi = self._relay_potential()
		return self._get_local_obs()

	def _hop_depth(self):
		"""각 드론의 제어 센터까지 홉 수를 반환한다 (1=직접 연결)."""
		h = np.full(self.num_drones, -1)
		frontier = np.flatnonzero(np.linalg.norm(self.drones_pos - self.cc_pos, axis=1) <= self.comm_range)
		h[frontier] = 1
		d = np.linalg.norm(self.drones_pos[:, None, :] - self.drones_pos[None, :, :], axis=2)
		k = 1
		while len(frontier):
			nxt = np.flatnonzero((h == -1) & (d[:, frontier] <= self.comm_range).any(axis=1))
			h[nxt] = k + 1
			frontier = nxt
			k += 1
		return h

	def _necessary_relays(self):
		"""자신이 빠지면 다른 드론이 고립되는(중계에 필수인) 드론 마스크를 반환한다."""
		nec = np.zeros(self.num_drones, dtype=bool)
		for i in range(self.num_drones):
			pos = self.drones_pos.copy()
			pos[i] = 1e7                     # i를 통신 범위 밖으로 빼서 중계 능력을 제거
			vis = self._bfs_connected(pos)
			others = np.arange(self.num_drones) != i
			nec[i] = not vis[others].all()
		return nec

	def _relay_potential(self):
		"""중계에 유용한 간격(300~480)을 유지하는 드론 쌍 수에 비례하는 potential."""
		if self.relay_w == 0.0:
			return 0.0
		d = np.linalg.norm(self.drones_pos[:, None, :] - self.drones_pos[None, :, :], axis=2)
		iu = np.triu_indices(self.num_drones, k=1)
		lo, hi = self.relay_band[0] * self.comm_range, self.relay_band[1] * self.comm_range
		return self.relay_w * float(np.sum((d[iu] >= lo) & (d[iu] <= hi)))

	def _get_target_dists(self):
		if np.any(self.dests_active):
			active_dests = self.dests_pos[self.dests_active]
			dist_to_dests = np.linalg.norm(
				active_dests[None, :, :] - self.drones_pos[:, None, :], axis=2
			)
			nearest_dists = np.min(dist_to_dests, axis=1)
		else:
			nearest_dists = np.zeros(self.num_drones)
		return np.where(
			self.drones_capacity > 0,
			nearest_dists,
			np.linalg.norm(self.drones_pos - self.cc_pos, axis=1)
		)

	# Fix 3: BFS 로직을 헬퍼로 분리해 코드 중복 제거
	def _bfs_connected(self, positions):
		dist_to_cc = np.linalg.norm(positions - self.cc_pos, axis=1)
		visited = dist_to_cc <= self.comm_range

		drone_dists = np.linalg.norm(
			positions[:, None, :] - positions[None, :, :], axis=2
		)
		adjacency = drone_dists <= self.comm_range
		np.fill_diagonal(adjacency, False)

		while True:
			new_visited = visited | (adjacency.dot(visited.astype(int)) > 0)
			if np.array_equal(new_visited, visited):
				break
			visited = new_visited
		return visited

	def _check_connectivity(self):
		self.comm_status = self._bfs_connected(self.drones_pos)

	# 연결성을 보상이 아닌 하드 제약으로 강제한다.
	# 구버전은 고립된 '피해' 드론의 이동만 취소해서, 하역 중이라 이동 의사가 없던
	# 드론이 동료의 이탈로 버려지는 경우를 전혀 막지 못했다 (회피 불가능한 -100).
	def _max_safe_scale(self, at, scale, m, iters=5):
		"""나머지 배율을 고정한 채 드론 m이 갈 수 있는 최대 배율을 이분 탐색한다."""
		lo, hi, trial = 0.0, scale[m], scale.copy()
		for _ in range(iters):
			mid = 0.5 * (lo + hi)
			trial[m] = mid
			if self._bfs_connected(at(trial)).all():
				lo = mid
			else:
				hi = mid
		return lo

	def _enforce_connectivity(self, proposed_pos, intended_to_move):
		"""연결이 유지되도록 이동을 축소하고 드론별 변위 배율을 반환한다.

		축소 대상은 '줄였을 때 팀 전체의 실현 변위가 가장 큰' 드론으로 고른다.
		누구를 제한할지 고르는 것이 얼마나 잘게 제한하는지보다 성능에 더 크게 작용한다.
		"""
		delta = proposed_pos - self.drones_pos
		norms = np.linalg.norm(delta, axis=1)
		scale = intended_to_move.astype(float)
		at = lambda sc: self.drones_pos + delta * sc[:, None]

		for _ in range(self.num_drones):
			if self._bfs_connected(at(scale)).all():
				return scale
			movers = np.flatnonzero(scale > 0.0)
			if len(movers) == 0:
				break

			# 각 후보를 제한했을 때 연결되는 드론 수로 고르고, 동점은 무작위로 깬다.
			# 동점을 결정론적으로 깨면(예: 변위 손실 최소) 늘 같은 드론이 제한되어
			# 사슬을 붙잡는 앵커가 굶는다 — 실측에서 배송이 29.4→22.2로 떨어졌다.
			cand_scale, cnts = {}, []
			for m in movers:
				sm = 0.0 if self.projection == "cancel" else self._max_safe_scale(at, scale, m)
				cand = scale.copy()
				cand[m] = sm
				cand_scale[m] = sm
				cnts.append(self._bfs_connected(at(cand)).sum())
			cnts = np.array(cnts)
			m = int(self.rng.choice(movers[cnts == cnts.max()]))
			scale[m] = cand_scale[m]

		if not self._bfs_connected(at(scale)).all():
			scale[:] = 0.0   # 안전망: 직전 위치는 항상 연결 상태다
		return scale

	def step(self, actions):
		self.current_step += 1
		rewards = np.zeros(self.num_drones)
		team_reward = 0.0

		# 이동 제안
		proposed_pos = np.copy(self.drones_pos)
		intended_to_move = np.zeros(self.num_drones, dtype=bool)
		for i in range(self.num_drones):
			if self.drones_timer[i] > 0:
				self.drones_timer[i] -= 1
			else:
				intended_to_move[i] = True
				if self.action_mode == "vector":
					v = np.asarray(actions[i], dtype=float)
					n = np.linalg.norm(v)
					if n > 1.0:
						v = v / n
					delta = v * self.max_speed
				else:
					angle, speed_ratio = actions[i][0], actions[i][1]
					speed = speed_ratio * self.max_speed
					delta = np.array([np.cos(angle) * speed, np.sin(angle) * speed])
				proposed_pos[i] += delta
				proposed_pos[i] = np.clip(proposed_pos[i], 0, self.map_size[0])

		scale = self._enforce_connectivity(proposed_pos, intended_to_move)
		delta = proposed_pos - self.drones_pos

		for i in range(self.num_drones):
			if not intended_to_move[i]:
				continue
			# 페널티를 '잘려나간 변위'에 비례시킨다. 제자리 명령(중계 대기)은 보정량이
			# 0이므로 처벌받지 않고, 일부만 잘린 경우 그만큼만 처벌한다.
			corr = (1.0 - scale[i]) * np.linalg.norm(delta[i]) / self.max_speed
			self.drones_pos[i] = self.drones_pos[i] + delta[i] * scale[i]
			if corr > 1e-3:
				self.blocked[i] += 1
				rewards[i] -= 0.1 * corr

		self._check_connectivity()

		# 안전망: 하드 제약이 정상 작동하면 아래는 발동하지 않는다 (Comm/LossRate로 검증)
		episode_terminated_by_comm_loss = False
		for i in range(self.num_drones):
			if not self.comm_status[i]:
				rewards[i] -= 100.0
				episode_terminated_by_comm_loss = True

		if episode_terminated_by_comm_loss:
			self.comm_loss = True
			if self.render_mode == "human":
				self.render()
			return self._get_local_obs(), self._get_global_state(), rewards, True

		# 이동 보상 (포텐셜 기반 쉐이핑)
		curr_target_dists = self._get_target_dists()
		for i in range(self.num_drones):
			if self.drones_timer[i] == 0:
				dist_diff = np.clip(
					self.prev_target_dists[i] - curr_target_dists[i],
					-self.max_speed, self.max_speed
				)
				if self.drones_capacity[i] == 0:
					rewards[i] += dist_diff * (self.shape_reward_scale * 5.0)
				else:
					rewards[i] += dist_diff * self.shape_reward_scale
			# 지연 페널티 -0.001 → -0.01: makespan 최소화 목적이 실제로 압력을 갖게 한다
			rewards[i] -= 0.01

			# 배달
			if self.drones_capacity[i] > 0 and self.drones_timer[i] == 0:
				for j in range(self.num_dests):
					if (self.dests_active[j] and
							np.linalg.norm(self.drones_pos[i] - self.dests_pos[j]) <= self.interaction_radius):
						self.dests_active[j] = False
						self.drones_capacity[i] -= 1
						self.drones_timer[i] = self.unload_time
						self.deliveries[i] += 1
						self.t_last_deliv = self.current_step
						team_reward += 10.0
						break

			# Fix 8: 보급 보상 +50 → +30 (조기 귀환 유인 완화)
			elif self.drones_capacity[i] == 0 and self.drones_timer[i] == 0:
				if np.linalg.norm(self.drones_pos[i] - self.cc_pos) <= self.interaction_radius:
					self.drones_capacity[i] = self.max_capacity
					self.drones_timer[i] = self.unload_time
					self.reloads[i] += 1
					team_reward += 30.0

		# 스텝당 누적되던 산개 보상(+0.05/쌍)은 제거하고 potential 형태로 대체했다(_relay_potential).
		# 주의: 연결 하드 제약은 사슬이 '끊어지는 것'만 막을 뿐 사슬을 '만들도록' 유도하지 않는다.
		# 제어 센터 옆에 뭉쳐 있어도 제약은 만족되므로, 별도 유인이 없으면 1홉으로 붕괴한다(실측).
		for i in range(self.num_drones):
			for j in range(i + 1, self.num_drones):
				if np.linalg.norm(self.drones_pos[i] - self.drones_pos[j]) < self.cluster_frac * self.comm_range:
					rewards[i] -= 0.05
					rewards[j] -= 0.05

		# Potential 기반 중계 쉐이핑: F = Φ(s') - Φ(s).
		# 에피소드 총합이 Φ(마지막) - Φ(처음)으로 telescoping되므로 길이에 무관하게 유계이고,
		# 대형을 유지하기만 해서는 보상이 누적되지 않아 farming이 불가능하다.
		phi = self._relay_potential()
		rewards += phi - self.prev_phi
		self.prev_phi = phi

		# 중계 도달 지표: 몇 홉짜리 사슬이 실제로 형성되는가 (붕괴를 조기 감지하기 위함)
		h = self._hop_depth()
		self.hop2plus += int(np.sum(h >= 2))
		self.max_reach = max(self.max_reach,
		                     float(np.max(np.linalg.norm(self.drones_pos - self.cc_pos, axis=1))))

		# 팀 보상 분배. 기본은 전원 동일이고, relay_credit이면 중계 필수 드론에 3배 가중한다.
		# 총량은 보존한다(가중치 평균 = 1) — 새 보상을 더하는 것이 아니라 배분만 바꾼다.
		if team_reward != 0.0 and self.relay_credit:
			w = np.where(self._necessary_relays(), 3.0, 1.0)
			rewards += team_reward * w * self.num_drones / w.sum()
		else:
			rewards += team_reward
		done = not np.any(self.dests_active)
		if done and self.makespan is None:
			self.makespan = self.current_step
		self.prev_target_dists = self._get_target_dists()
		if self.render_mode == "human":
			self.render()
		return self._get_local_obs(), self._get_global_state(), rewards, done

	# Fix 6: 관측치에 두 번째 이웃 드론 추가 (15 → 19차원)
	# 새 구조: [cc_dist, my_comm, my_cap, is_delivering,       # 4
	#           target_dist, target_dir(2),                     # 3
	#           peer1(dist, dir_x, dir_y, comm),                # 4
	#           peer2(dist, dir_x, dir_y, comm),                # 4
	#           drone_id one-hot]                               # 4  → 합계 19
	def _get_local_obs(self):
		obs = []
		for i in range(self.num_drones):
			cc_dist = np.linalg.norm(self.drones_pos[i] - self.cc_pos) / self.map_size[0]
			my_comm = 1.0 if self.comm_status[i] else 0.0
			my_cap = self.drones_capacity[i] / self.max_capacity
			is_delivering = 1.0 if self.drones_capacity[i] > 0 else 0.0
			target_dist = self.prev_target_dists[i] / self.map_size[0]

			if self.drones_capacity[i] > 0:
				active_dests = self.dests_pos[self.dests_active]
				if len(active_dests) > 0:
					raw_vec = (
						active_dests[np.argmin(np.linalg.norm(active_dests - self.drones_pos[i], axis=1))]
						- self.drones_pos[i]
					)
				else:
					raw_vec = np.array([0.0, 0.0])
			else:
				raw_vec = self.cc_pos - self.drones_pos[i]

			dist_raw = np.linalg.norm(raw_vec)
			target_dir = (raw_vec / dist_raw) if dist_raw > 0 else np.array([0.0, 0.0])

			# Fix 6: 거리 순 정렬 후 가장 가까운 2대 관측
			other_dists = sorted(
				[(np.linalg.norm(self.drones_pos[i] - self.drones_pos[j]), j)
				 for j in range(self.num_drones) if j != i]
			)
			peer_features = []
			for k in range(self.n_peers):
				nj = other_dists[k][1]
				peer_raw_vec = self.drones_pos[nj] - self.drones_pos[i]
				peer_dist_raw = other_dists[k][0]
				peer_dist = peer_dist_raw / self.map_size[0]
				peer_dir = (peer_raw_vec / peer_dist_raw) if peer_dist_raw > 0 else np.array([0.0, 0.0])
				peer_comm = 1.0 if self.comm_status[nj] else 0.0
				peer_features.extend([peer_dist, peer_dir[0], peer_dir[1], peer_comm])

			drone_id = np.zeros(self.num_drones)
			drone_id[i] = 1.0

			agent_obs = np.array([
				cc_dist, my_comm, my_cap, is_delivering,
				target_dist, target_dir[0], target_dir[1],
				*peer_features,
				*drone_id
			])
			obs.append(agent_obs)
		return np.array(obs)

	def episode_stats(self):
		"""에피소드 종료 시점의 배송 진척, 드론 간 균형, 연결 제약 지표를 반환한다."""
		delivered = int(self.num_dests - self.dests_active.sum())
		return {
			"delivered": delivered,
			"completion_rate": delivered / self.num_dests,
			"makespan": self.makespan,
			"idle_drones": int((self.deliveries == 0).sum()),
			"deliv_min": int(self.deliveries.min()),
			"deliv_max": int(self.deliveries.max()),
			"deliv_std": float(self.deliveries.std()),
			"reloads": int(self.reloads.sum()),
			"blocked": int(self.blocked.sum()),
			"comm_loss": int(self.comm_loss),
			"steps": self.current_step,
			"t_last_deliv": self.t_last_deliv,
			"hop2plus": self.hop2plus / max(1, self.current_step * self.num_drones),
			"max_reach": self.max_reach,
		}

	def _get_global_state(self):
		return np.concatenate((
			self.drones_pos.flatten() / self.map_size[0],
			self.comm_status,
			self.drones_capacity / self.max_capacity,
			self.dests_active
		))

	def render(self):
		if self.render_mode != "human":
			return
		if self.screen is None:
			pygame.init()
			pygame.font.init()
			self.screen = pygame.display.set_mode(self.map_size)
			self.clock = pygame.time.Clock()
			self.radius_surface = pygame.Surface(self.map_size, pygame.SRCALPHA)
			self.font = pygame.font.SysFont('Arial', 14, bold=True)

		self.screen.fill((255, 255, 255))
		self.radius_surface.fill((0, 0, 0, 0))
		pygame.draw.circle(self.radius_surface, (0, 0, 255, 30), self.cc_pos.astype(int), int(self.comm_range))
		self.screen.blit(self.radius_surface, (0, 0))

		for i in range(self.num_drones):
			pygame.draw.circle(self.screen, (0, 255, 0), self.drones_pos[i].astype(int), int(self.comm_range), 2)
		for i in range(self.num_drones):
			if np.linalg.norm(self.drones_pos[i] - self.cc_pos) <= self.comm_range:
				pygame.draw.line(self.screen, (150, 150, 150), self.cc_pos, self.drones_pos[i], 2)
			for j in range(i + 1, self.num_drones):
				if np.linalg.norm(self.drones_pos[i] - self.drones_pos[j]) <= self.comm_range:
					pygame.draw.line(self.screen, (150, 150, 150), self.drones_pos[i], self.drones_pos[j], 2)

		for j in range(self.num_dests):
			if self.dests_active[j]:
				pygame.draw.rect(self.screen, (255, 0, 0), (*self.dests_pos[j] - 5, 10, 10))

		pygame.draw.circle(self.screen, (0, 0, 255), self.cc_pos.astype(int), 15)
		for i in range(self.num_drones):
			color = (139, 69, 19) if self.drones_capacity[i] == 0 else (0, 200, 0)
			if not self.comm_status[i]:
				color = (0, 0, 0)
			pygame.draw.circle(self.screen, color, self.drones_pos[i].astype(int), 12)
			if self.drones_timer[i] > 0:
				pygame.draw.circle(self.screen, (255, 215, 0), self.drones_pos[i].astype(int), 16, 3)
			cap_text = self.font.render(str(int(self.drones_capacity[i])), True, (255, 255, 255))
			self.screen.blit(cap_text, cap_text.get_rect(center=self.drones_pos[i].astype(int)))

		pygame.display.flip()
		self.clock.tick(self.render_fps)
