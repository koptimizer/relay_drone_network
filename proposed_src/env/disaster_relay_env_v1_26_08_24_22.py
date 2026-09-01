"""재난 릴레이 드론 환경 (v1, 26_08_24_22).

통신 연결을 하드 제약으로 강제하고, 에피소드 단위 배송/균형 지표를 집계한다.
분할 이전 원본은 legacy/pre_v1/CTDERL.py에 보존되어 있다.
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
	def __init__(self, map_path=DEFAULT_MAP, render_mode=None, relay_w=0.0):
		self.relay_w = relay_w
		self.num_drones = 4
		self.num_dests = 50
		self.map_size = (1000, 1000)
		self.max_capacity = 5
		self.comm_range = 500.0
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
		self.comm_loss = False
		self.prev_phi = self._relay_potential()
		return self._get_local_obs()

	def _relay_potential(self):
		"""중계에 유용한 간격(300~480)을 유지하는 드론 쌍 수에 비례하는 potential."""
		if self.relay_w == 0.0:
			return 0.0
		d = np.linalg.norm(self.drones_pos[:, None, :] - self.drones_pos[None, :, :], axis=2)
		iu = np.triu_indices(self.num_drones, k=1)
		return self.relay_w * float(np.sum((d[iu] >= 300.0) & (d[iu] <= 480.0)))

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
	def _enforce_connectivity(self, proposed_pos, intended_to_move):
		"""전원이 연결될 때까지 원인이 되는 이동을 하나씩 취소하고 허용 마스크를 반환한다."""
		valid = intended_to_move.copy()
		while True:
			pos = np.where(valid[:, None], proposed_pos, self.drones_pos)
			if self._bfs_connected(pos).all():
				return valid

			movers = np.flatnonzero(valid)
			if len(movers) == 0:
				return valid   # 모든 이동을 되돌린 상태 = 직전 위치이므로 여기 도달하지 않는다

			# 되돌렸을 때 연결 드론 수가 가장 많아지는 이동을 취소한다
			best, best_cnt = movers[0], -1
			for m in movers:
				trial = valid.copy()
				trial[m] = False
				cnt = self._bfs_connected(
					np.where(trial[:, None], proposed_pos, self.drones_pos)
				).sum()
				if cnt > best_cnt:
					best, best_cnt = m, cnt
			valid[best] = False

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
				angle, speed_ratio = actions[i][0], actions[i][1]
				speed = speed_ratio * self.max_speed
				proposed_pos[i] += np.array([np.cos(angle) * speed, np.sin(angle) * speed])
				proposed_pos[i] = np.clip(proposed_pos[i], 0, self.map_size[0])

		valid_movement = self._enforce_connectivity(proposed_pos, intended_to_move)

		for i in range(self.num_drones):
			if intended_to_move[i]:
				if valid_movement[i]:
					self.drones_pos[i] = proposed_pos[i]
				else:
					self.blocked[i] += 1
					rewards[i] -= 0.1

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
						team_reward += 10.0
						break

			# Fix 8: 보급 보상 +50 → +30 (조기 귀환 유인 완화)
			elif self.drones_capacity[i] == 0 and self.drones_timer[i] == 0:
				if np.linalg.norm(self.drones_pos[i] - self.cc_pos) <= self.interaction_radius:
					self.drones_capacity[i] = self.max_capacity
					self.drones_timer[i] = self.unload_time
					self.reloads[i] += 1
					team_reward += 30.0

		# 산개 보상(+0.05/쌍/스텝)은 제거한다. 시간 상한이 없어 대형만 유지하며 버티는 것이
		# 배달보다 이득이 되는 보상 해킹 경로였다 (이상 대형 67스텝 = 배달 1건).
		# 뭉침 페널티만 남겨도, 연결이 하드 제약이므로 릴레이 사슬은 구조적으로 유지된다.
		for i in range(self.num_drones):
			for j in range(i + 1, self.num_drones):
				if np.linalg.norm(self.drones_pos[i] - self.drones_pos[j]) < 150.0:
					rewards[i] -= 0.05
					rewards[j] -= 0.05

		# Potential 기반 중계 쉐이핑: F = Φ(s') - Φ(s).
		# 에피소드 총합이 Φ(마지막) - Φ(처음)으로 telescoping되므로 길이에 무관하게 유계이고,
		# 대형을 유지하기만 해서는 보상이 누적되지 않아 farming이 불가능하다.
		phi = self._relay_potential()
		rewards += phi - self.prev_phi
		self.prev_phi = phi

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
			for k in range(2):
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
