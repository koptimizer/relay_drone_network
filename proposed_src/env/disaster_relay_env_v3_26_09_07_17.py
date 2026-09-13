"""재난 릴레이 드론 환경 (v3, 26_09_07_17) — makespan 달성을 위한 장기 에피소드.

26_09_04_15 대비 세 가지를 고쳤다.
1) 에피소드 상한을 10배로 늘려 50곳 전량 배송(makespan)이 가능하게 했다.
2) 무의미하게 길어지는 에피소드는 조기 종료한다 — 교착 100스텝 또는 무배송 200스텝.
3) 정지 판정을 팀 총변위가 아니라 드론별로 한다. 이전 지표는 한 대만 움직여도
   '이동 중'으로 집계되어, 나머지가 갇힌 부분 교착을 놓쳤다.
또한 도달 보너스가 주석과 달리 매 스텝 지급되던 것을 목표당 1회로 바로잡았다.
"""

import os
import warnings

import numpy as np
import pygame

warnings.filterwarnings("ignore", category=UserWarning, module="pygame")

DEFAULT_MAP = os.path.join(
	os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "util", "disaster_map.npy"
)

# 하위 목표 종류
GOAL_DELIVER, GOAL_RETURN, GOAL_HOLD = 0, 1, 2


class DisasterRelayDroneEnv:
	"""계층 제어용 재난 배송 환경.

	연결성은 보상이 아니라 하드 제약으로, 이동을 연결 유지 최대치까지 축소해 강제한다.
	행동은 2D 추력이고, 하위 목표는 set_goals()로 상위가 지정한다.
	"""

	def __init__(self, map_path=DEFAULT_MAP, render_mode=None, comm_range=300.0,
	             n_cand=5, cluster_penalty=True, stuck_obs=True,
	             max_steps=10000, deadlock_limit=100, no_progress_limit=200,
	             incomplete_penalty=0.0, arrive_once=True):
		self.num_drones = 4
		self.num_dests = 50
		self.map_size = (1000, 1000)
		self.max_capacity = 5
		self.comm_range = float(comm_range)
		self.n_cand = n_cand              # 상위가 고를 수 있는 후보 목적지 수
		self.cluster_penalty = cluster_penalty
		self.stuck_obs = stuck_obs        # 상위 관측에 교착 신호를 포함할지
		self.stuck_decay = 0.95           # 이동 실현율 EMA 감쇠
		self.max_steps = max_steps            # 에피소드 상한
		self.deadlock_limit = deadlock_limit  # 이 스텝 이상 교착이면 조기 종료
		self.no_progress_limit = no_progress_limit  # 이 스텝 이상 무배송이면 조기 종료
		# 종료 시 남은 목적지당 감점. 시간 페널티가 커진 탓에 '일부러 멈추는 것'이
		# 이득이 되는 구간이 생기는데, 이를 상쇄한다. 0이면 비활성.
		self.incomplete_penalty = incomplete_penalty
		# False면 26_09_04_15 이전처럼 목표 반경 안에 있는 매 스텝 도달 보너스를 준다.
		# 회귀 원인 절제용 — 기본값은 목표당 1회(현행)다.
		self.arrive_once = arrive_once
		self.cluster_frac = 0.30          # 이보다 가까우면 뭉침으로 보고 페널티
		self.max_speed = 50.0 * (1000 / 3600)
		self.unload_time = 10
		self.interaction_radius = 25.0
		self.shape_scale = 0.010          # 하위 목표까지 거리 감소분에 곱하는 계수
		self.arrive_bonus = 1.0           # 하위 목표 도달 시 1회성 보너스
		self.time_penalty = 0.01
		self.rng = np.random.default_rng()

		if os.path.exists(map_path):
			d = np.load(map_path, allow_pickle=True).item()
			self.cc_pos, self.dests_pos = d['cc_pos'], d['dests_pos']
		else:
			self.cc_pos = np.array([950.0, 500.0])
			self.dests_pos = np.random.uniform(low=[50, 50], high=[850, 950],
			                                   size=(self.num_dests, 2))

		self.render_mode = render_mode
		self.screen = self.clock = self.font = None

	# ---------- 리셋 및 상태 ----------

	def reset(self):
		"""에피소드를 초기화하고 하위 관측을 반환한다."""
		self.dests_active = np.ones(self.num_dests, dtype=bool)
		self.drones_pos = np.tile(self.cc_pos, (self.num_drones, 1)).astype(float)
		self.drones_capacity = np.full(self.num_drones, self.max_capacity)
		self.drones_timer = np.zeros(self.num_drones)
		self.comm_status = np.ones(self.num_drones, dtype=bool)
		self.current_step = 0

		# 하위 목표: 종류와 좌표. 상위가 지정하기 전에는 전원 제자리 유지.
		self.goal_kind = np.full(self.num_drones, GOAL_HOLD)
		self.goal_pos = self.drones_pos.copy()
		self.goal_dest = np.full(self.num_drones, -1)   # 배송 목표의 목적지 인덱스
		self.goal_reached = np.zeros(self.num_drones, dtype=bool)
		self.prev_goal_dist = self._goal_dists()

		self.deliveries = np.zeros(self.num_drones, dtype=int)
		self.reloads = np.zeros(self.num_drones, dtype=int)
		self.blocked = np.zeros(self.num_drones, dtype=int)
		# 교착 추적: 명령 대비 실제로 실현된 변위 비율의 EMA. 1이면 자유, 0이면 완전 정지.
		self.move_ema = np.ones(self.num_drones)
		self.team_move_ema = 1.0
		self.deadlock_run = 0        # 부분 교착이 연속으로 이어진 스텝 수
		self.max_deadlock_run = 0
		self.stalled_dsteps = 0      # 드론별 정지로 집계한 드론-스텝
		self.mover_dsteps = 0
		self.no_progress = 0         # 마지막 배송 이후 경과 스텝
		self.term_reason = "step_limit"
		self.arrivals = np.zeros(self.num_drones, dtype=int)
		self.makespan = None
		self.t_last_deliv = 0
		self.comm_loss = False
		self.hop2plus = 0
		self.max_reach = 0.0
		return self.worker_obs()

	def _goal_dists(self):
		"""각 드론에서 자기 하위 목표까지의 거리."""
		return np.linalg.norm(self.drones_pos - self.goal_pos, axis=1)

	def candidates(self, i):
		"""드론 i 기준 최근접 미배송지 인덱스 n_cand개 (부족하면 -1로 채움)."""
		act = np.flatnonzero(self.dests_active)
		out = np.full(self.n_cand, -1)
		if len(act) == 0:
			return out
		d = np.linalg.norm(self.dests_pos[act] - self.drones_pos[i], axis=1)
		near = act[np.argsort(d)[:self.n_cand]]
		out[:len(near)] = near
		return out

	def set_goals(self, actions):
		"""상위 행동(드론별 정수)을 하위 목표로 변환한다.

		0..n_cand-1 = 해당 후보 목적지로 배송, n_cand = 제어 센터 복귀, n_cand+1 = 현 위치 유지.
		"""
		for i, a in enumerate(actions):
			a = int(a)
			if a == self.n_cand:
				self.goal_kind[i], self.goal_pos[i], self.goal_dest[i] = \
					GOAL_RETURN, self.cc_pos.copy(), -1
			elif a >= self.n_cand + 1:
				self.goal_kind[i], self.goal_pos[i], self.goal_dest[i] = \
					GOAL_HOLD, self.drones_pos[i].copy(), -1
			else:
				cand = self.candidates(i)[a]
				if cand < 0:                       # 후보가 없으면 유지로 대체
					self.goal_kind[i], self.goal_pos[i], self.goal_dest[i] = \
						GOAL_HOLD, self.drones_pos[i].copy(), -1
				else:
					self.goal_kind[i], self.goal_pos[i], self.goal_dest[i] = \
						GOAL_DELIVER, self.dests_pos[cand].copy(), cand
		self.goal_reached[:] = False
		self.prev_goal_dist = self._goal_dists()

	def goal_invalid(self):
		"""상위 재결정이 필요한 드론 마스크를 반환한다."""
		bad = np.zeros(self.num_drones, dtype=bool)
		for i in range(self.num_drones):
			k = self.goal_kind[i]
			if k == GOAL_DELIVER:
				# 목표가 이미 배송됐거나, 적재량이 없어 배송할 수 없음
				bad[i] = (self.goal_dest[i] < 0 or not self.dests_active[self.goal_dest[i]]
				          or self.drones_capacity[i] == 0)
			elif k == GOAL_RETURN:
				bad[i] = self.drones_capacity[i] == self.max_capacity
			else:
				bad[i] = True   # HOLD는 매 결정 주기마다 다시 판단한다
		return bad

	# ---------- 연결성 ----------

	def _bfs_connected(self, positions):
		"""제어 센터에서 다중 홉으로 도달 가능한 드론 마스크."""
		visited = np.linalg.norm(positions - self.cc_pos, axis=1) <= self.comm_range
		adj = np.linalg.norm(positions[:, None, :] - positions[None, :, :], axis=2) <= self.comm_range
		np.fill_diagonal(adj, False)
		while True:
			nxt = visited | (adj.dot(visited.astype(int)) > 0)
			if np.array_equal(nxt, visited):
				return visited
			visited = nxt

	def _hop_depth(self):
		"""각 드론의 제어 센터까지 홉 수 (1=직접 연결, -1=단절)."""
		h = np.full(self.num_drones, -1)
		front = np.flatnonzero(np.linalg.norm(self.drones_pos - self.cc_pos, axis=1) <= self.comm_range)
		h[front] = 1
		d = np.linalg.norm(self.drones_pos[:, None, :] - self.drones_pos[None, :, :], axis=2)
		k = 1
		while len(front):
			nxt = np.flatnonzero((h == -1) & (d[:, front] <= self.comm_range).any(axis=1))
			h[nxt] = k + 1
			front, k = nxt, k + 1
		return h

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

	def _project(self, delta, moving):
		"""연결이 유지되도록 이동을 축소하고 드론별 변위 배율을 반환한다."""
		scale = moving.astype(float)
		at = lambda sc: self.drones_pos + delta * sc[:, None]
		for _ in range(self.num_drones):
			if self._bfs_connected(at(scale)).all():
				return scale
			movers = np.flatnonzero(scale > 0.0)
			if len(movers) == 0:
				break
			# 제한했을 때 연결 드론 수가 가장 많아지는 드론을 고르고, 동점은 무작위로 깬다.
			# 결정론적으로 깨면 사슬을 붙잡는 앵커 드론이 반복 선택되어 굶는다.
			cand, cnts = {}, []
			for m in movers:
				sm = self._max_safe_scale(at, scale, m)
				trial = scale.copy()
				trial[m] = sm
				cand[m] = sm
				cnts.append(self._bfs_connected(at(trial)).sum())
			cnts = np.array(cnts)
			m = int(self.rng.choice(movers[cnts == cnts.max()]))
			scale[m] = cand[m]
		if not self._bfs_connected(at(scale)).all():
			scale[:] = 0.0     # 안전망: 직전 위치는 항상 연결 상태다
		return scale

	# ---------- 전이 ----------

	def step(self, thrust):
		"""2D 추력을 받아 한 스텝 진행하고 (하위 관측, 하위 보상, 팀 보상, 종료)를 반환한다."""
		self.current_step += 1
		w_rew = np.zeros(self.num_drones)   # 하위: 목표 진척
		team = 0.0                          # 상위: 팀 목적함수

		# 이동 제안 — 하역/재적재 중이면 정지
		delta = np.zeros((self.num_drones, 2))
		moving = np.zeros(self.num_drones, dtype=bool)
		for i in range(self.num_drones):
			if self.drones_timer[i] > 0:
				self.drones_timer[i] -= 1
				continue
			moving[i] = True
			v = np.asarray(thrust[i], dtype=float)
			n = np.linalg.norm(v)
			if n > 1.0:
				v = v / n
			delta[i] = v * self.max_speed
		np.clip(self.drones_pos + delta, 0, self.map_size[0], out=(tmp := np.empty_like(delta)))
		delta = tmp - self.drones_pos

		scale = self._project(delta, moving)
		for i in range(self.num_drones):
			if not moving[i]:
				continue
			corr = (1.0 - scale[i]) * np.linalg.norm(delta[i]) / self.max_speed
			self.drones_pos[i] += delta[i] * scale[i]
			if corr > 1e-3:
				self.blocked[i] += 1
				w_rew[i] -= 0.1 * corr      # 잘려나간 변위에 비례

		# 명령 대비 실현 변위 비율을 EMA로 추적한다. 상위가 교착을 인지하는 신호가 된다.
		cmd = np.linalg.norm(delta, axis=1)
		realized = np.where(cmd > 1e-6, scale, 1.0)
		self.move_ema = self.stuck_decay * self.move_ema + (1 - self.stuck_decay) * realized
		team_ratio = float(np.sum(cmd * scale) / max(1e-6, np.sum(cmd))) if cmd.sum() > 1e-6 else 1.0
		self.team_move_ema = self.stuck_decay * self.team_move_ema + (1 - self.stuck_decay) * team_ratio

		# 정지는 드론별로 판정한다. 팀 총변위로 보면 한 대가 제어 센터 근처에서 진동하는 것만으로
		# '이동 중'이 되어, 나머지가 갇힌 부분 교착이 통계에서 사라진다.
		movers = np.flatnonzero(cmd > 1e-6)
		blocked_now = np.array([scale[i] < 0.1 for i in movers], dtype=bool)
		self.mover_dsteps += len(movers)
		self.stalled_dsteps += int(blocked_now.sum())
		# 이동을 시도한 드론 중 한 대를 뺀 나머지가 전부 막혔으면 부분 교착으로 본다
		jammed = len(movers) > 0 and blocked_now.sum() >= max(1, len(movers) - 1)
		self.deadlock_run = self.deadlock_run + 1 if jammed else 0
		self.max_deadlock_run = max(self.max_deadlock_run, self.deadlock_run)

		self.comm_status = self._bfs_connected(self.drones_pos)
		if not self.comm_status.all():      # 하드 제약이 정상이면 발생하지 않는다
			self.comm_loss = True

		# 하위 보상: 목표까지 거리 감소분 + 도달 보너스
		curr = self._goal_dists()
		for i in range(self.num_drones):
			if self.drones_timer[i] > 0:
				continue
			if self.goal_kind[i] != GOAL_HOLD:
				w_rew[i] += np.clip(self.prev_goal_dist[i] - curr[i],
				                    -self.max_speed, self.max_speed) * self.shape_scale
				# 목표당 1회만 지급한다. 이전 버전은 반경 안에 머무는 매 스텝 지급되어
				# 제자리 진동이 이득이 되는 경로가 있었다(시간 페널티의 100배).
				if curr[i] <= self.interaction_radius and not (self.arrive_once
				                                              and self.goal_reached[i]):
					w_rew[i] += self.arrive_bonus
					self.goal_reached[i] = True
					self.arrivals[i] += 1
			w_rew[i] -= self.time_penalty

		# 배송 / 재적재
		for i in range(self.num_drones):
			if self.drones_timer[i] > 0:
				continue
			if self.drones_capacity[i] > 0:
				for j in np.flatnonzero(self.dests_active):
					if np.linalg.norm(self.drones_pos[i] - self.dests_pos[j]) <= self.interaction_radius:
						self.dests_active[j] = False
						self.drones_capacity[i] -= 1
						self.drones_timer[i] = self.unload_time
						self.deliveries[i] += 1
						self.t_last_deliv = self.current_step
						self.no_progress = 0
						team += 10.0
						break
			elif np.linalg.norm(self.drones_pos[i] - self.cc_pos) <= self.interaction_radius:
				self.drones_capacity[i] = self.max_capacity
				self.drones_timer[i] = self.unload_time
				self.reloads[i] += 1
				team += 30.0

		if self.cluster_penalty:
			thr = self.cluster_frac * self.comm_range
			for i in range(self.num_drones):
				for j in range(i + 1, self.num_drones):
					if np.linalg.norm(self.drones_pos[i] - self.drones_pos[j]) < thr:
						w_rew[i] -= 0.05
						w_rew[j] -= 0.05

		team -= self.time_penalty * self.num_drones

		h = self._hop_depth()
		self.hop2plus += int(np.sum(h >= 2))
		self.max_reach = max(self.max_reach,
		                     float(np.max(np.linalg.norm(self.drones_pos - self.cc_pos, axis=1))))
		self.prev_goal_dist = curr

		self.no_progress += 1
		done = not np.any(self.dests_active)
		if done and self.makespan is None:
			self.makespan = self.current_step
			self.term_reason = "complete"
		# 조기 종료: 학습이 무의미한 구간에 시간을 쓰지 않도록 한다
		elif self.deadlock_run >= self.deadlock_limit:
			done, self.term_reason = True, "deadlock"
		elif self.no_progress >= self.no_progress_limit:
			done, self.term_reason = True, "no_progress"
		elif self.current_step >= self.max_steps:
			done, self.term_reason = True, "step_limit"
		if done and self.incomplete_penalty > 0.0:
			team -= self.incomplete_penalty * float(self.dests_active.sum())
		if self.render_mode == "human":
			self.render()
		return self.worker_obs(), w_rew, team, done

	# ---------- 관측 ----------

	def worker_obs(self):
		"""하위 관측: 목표 상대 벡터, 자기 상태, 동료 2대, 홉 깊이. (드론, 16)"""
		h = self._hop_depth()
		out = []
		for i in range(self.num_drones):
			g = self.goal_pos[i] - self.drones_pos[i]
			gd = np.linalg.norm(g)
			gdir = g / gd if gd > 1e-6 else np.zeros(2)
			kind = np.zeros(3)
			kind[self.goal_kind[i]] = 1.0

			others = sorted((np.linalg.norm(self.drones_pos[i] - self.drones_pos[j]), j)
			                for j in range(self.num_drones) if j != i)
			peer = []
			for d, j in others[:2]:
				v = self.drones_pos[j] - self.drones_pos[i]
				peer.extend([d / self.map_size[0], *(v / d if d > 1e-6 else np.zeros(2))])

			out.append(np.array([
				gd / self.map_size[0], gdir[0], gdir[1], *kind,
				self.drones_capacity[i] / self.max_capacity,
				min(self.drones_timer[i], self.unload_time) / self.unload_time,
				np.linalg.norm(self.drones_pos[i] - self.cc_pos) / self.map_size[0],
				h[i] / 4.0,
				*peer,
			]))
		return np.array(out)

	def can_advance(self):
		"""각 드론이 혼자 목표 방향으로 최대 속도 이동해도 연결이 유지되는지."""
		out = np.zeros(self.num_drones)
		for i in range(self.num_drones):
			g = self.goal_pos[i] - self.drones_pos[i]
			n = np.linalg.norm(g)
			if n < 1e-6:
				out[i] = 1.0
				continue
			trial = self.drones_pos.copy()
			trial[i] = np.clip(trial[i] + g / n * self.max_speed, 0, self.map_size[0])
			out[i] = float(self._bfs_connected(trial).all())
		return out

	def manager_obs(self):
		"""상위 관측: 자기 상태, 교착 신호, 후보 목적지 상대 정보, 동료 상태."""
		h = self._hop_depth()
		adv = self.can_advance() if self.stuck_obs else np.ones(self.num_drones)
		out = []
		for i in range(self.num_drones):
			cand = self.candidates(i)
			cf = []
			for c in cand:
				if c < 0:
					cf.extend([0.0, 0.0, 0.0, 0.0])
					continue
				v = self.dests_pos[c] - self.drones_pos[i]
				d = np.linalg.norm(v)
				taken = float(np.any((self.goal_dest == c) & (np.arange(self.num_drones) != i)))
				cf.extend([d / self.map_size[0], *(v / d if d > 1e-6 else np.zeros(2)), taken])

			pf = []
			for j in range(self.num_drones):
				if j == i:
					continue
				v = self.drones_pos[j] - self.drones_pos[i]
				pf.extend([np.linalg.norm(v) / self.map_size[0], *(v / self.map_size[0]),
				           self.drones_capacity[j] / self.max_capacity, h[j] / 4.0])

			# 교착 신호 3종: 자기 이동 실현율, 지금 전진 가능한가, 편대 전체 정지 정도.
			# 이전 버전에서는 상위가 '연결이 끊겼는가'만 알 뿐 '움직일 수 있는가'를 몰라
			# 팽팽하게 당겨진 사슬에서 빠져나오지 못했다.
			stuck = ([self.move_ema[i], adv[i], self.team_move_ema]
			         if self.stuck_obs else [1.0, 1.0, 1.0])
			out.append(np.array([
				self.drones_capacity[i] / self.max_capacity,
				np.linalg.norm(self.drones_pos[i] - self.cc_pos) / self.map_size[0],
				h[i] / 4.0,
				self.dests_active.sum() / self.num_dests,
				self.current_step / 1000.0,
				*stuck,
				*cf, *pf,
			]))
		return np.array(out)

	def global_state(self):
		"""중앙 critic 입력: 전 드론 위치·적재량·연결, 목적지 활성 상태."""
		return np.concatenate((
			self.drones_pos.flatten() / self.map_size[0],
			self.comm_status.astype(float),
			self.drones_capacity / self.max_capacity,
			self.dests_active.astype(float),
		))

	def episode_stats(self):
		"""에피소드 결과 지표."""
		delivered = int(self.num_dests - self.dests_active.sum())
		return {
			"delivered": delivered,
			"completion_rate": delivered / self.num_dests,
			"makespan": self.makespan,
			"t_last_deliv": self.t_last_deliv,
			"idle_drones": int((self.deliveries == 0).sum()),
			"deliv_std": float(self.deliveries.std()),
			"reloads": int(self.reloads.sum()),
			"arrivals": int(self.arrivals.sum()),
			"blocked": int(self.blocked.sum()),
			"comm_loss": int(self.comm_loss),
			"hop2plus": self.hop2plus / max(1, self.current_step * self.num_drones),
			"max_reach": self.max_reach,
			"steps": self.current_step,
			"stall_ratio": self.stalled_dsteps / max(1, self.mover_dsteps),
			"max_stall_run": int(self.max_deadlock_run),
			"term_reason": self.term_reason,
		}

	def render(self):
		"""pygame 창에 현재 상태를 그린다."""
		if self.screen is None:
			pygame.init()
			pygame.font.init()
			self.screen = pygame.display.set_mode(self.map_size)
			self.clock = pygame.time.Clock()
			self.font = pygame.font.SysFont('DejaVu Sans', 14, bold=True)
		self.screen.fill((250, 250, 252))
		pygame.draw.circle(self.screen, (60, 110, 220), self.cc_pos.astype(int),
		                   int(self.comm_range), 2)
		for i in range(self.num_drones):
			for j in range(i + 1, self.num_drones):
				if np.linalg.norm(self.drones_pos[i] - self.drones_pos[j]) <= self.comm_range:
					pygame.draw.line(self.screen, (200, 205, 212),
					                 self.drones_pos[i], self.drones_pos[j], 2)
		for j in np.flatnonzero(self.dests_active):
			pygame.draw.rect(self.screen, (220, 60, 60), (*self.dests_pos[j] - 5, 10, 10))
		pygame.draw.circle(self.screen, (30, 80, 200), self.cc_pos.astype(int), 15)
		for i in range(self.num_drones):
			col = (40, 170, 70) if self.drones_capacity[i] > 0 else (150, 90, 30)
			pygame.draw.circle(self.screen, col, self.drones_pos[i].astype(int), 12)
		pygame.display.flip()
		self.clock.tick(120)
