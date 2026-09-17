"""재난 릴레이 드론 환경 (v5, 26_09_15_22) — 구성 가변화와 목표 유지.

v4 대비 네 가지를 바꿨다.
1) 드론 수·목적지 수를 인자로 받는다. 학습 중 에피소드마다 구성을 바꿀 수 있다.
2) 후보 목적지에 '제어 센터에서 가장 먼 곳'을 섞는다. 최근접만 고르면 먼 목적지가
   편대가 지친 마지막에야 후보로 떠올라 미배송으로 남았다(미배송 평균거리 778 대 493).
3) 목표가 바뀐 드론만 도달 상태를 초기화한다. 같은 목표를 다시 받은 드론은 이어서 간다.
4) 도달 보너스 매 스텝·조기 종료 없음·상한 1000을 기본값으로 둔다(v3에서 확정된 설정).
"""

import os
import warnings

import numpy as np
import pygame

warnings.filterwarnings("ignore", category=UserWarning, module="pygame")

DEFAULT_MAP = os.path.join(
	os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "util", "disaster_map.npy"
)

from util.instance_generator_v5_26_09_15_22 import sample_instance

# 하위 목표 종류
GOAL_DELIVER, GOAL_RETURN, GOAL_HOLD, GOAL_RELAY = 0, 1, 2, 3
N_MAX = 8   # 집합 관측에서 홉·슬롯 수를 정규화하는 상수 (드론 수와 무관해야 한다)


class DisasterRelayDroneEnv:
	"""계층 제어용 재난 배송 환경.

	연결성은 보상이 아니라 하드 제약으로, 이동을 연결 유지 최대치까지 축소해 강제한다.
	행동은 2D 추력이고, 하위 목표는 set_goals()로 상위가 지정한다.
	"""

	def __init__(self, map_path=DEFAULT_MAP, render_mode=None, comm_range=300.0,
	             n_cand=5, cluster_penalty=True, stuck_obs=True,
	             max_steps=1000, deadlock_limit=10 ** 9, no_progress_limit=10 ** 9,
	             incomplete_penalty=0.0, arrive_once=False, relay_beta=0.9,
	             relay_hold=0.02, num_drones=4, num_dests=50, n_far=0, commit_max=150,
	             complete_bonus=0.0):
		self.num_drones = num_drones
		self.num_dests = num_dests
		# 후보 n_cand개 중 n_far개는 제어 센터에서 가장 먼 미배송지로 채운다
		self.n_far = min(n_far, n_cand - 1)
		# 목표를 이 스텝 이상 붙들었으면 재결정을 허용한다 (무한 고착 방지)
		self.commit_max = commit_max
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
		# 전량 완주 시 남은 시간 비율에 비례해 주는 팀 보상. 배송 보상(건당 10, 50곳이면 500)에
		# 비해 시간 페널티(스텝당 0.04)가 약해 makespan 신호가 정책에 거의 닿지 않았다.
		# 200이면 상한 절반에 끝냈을 때 +100으로, 전체 수익의 약 20%가 시간에 걸린다.
		self.complete_bonus = complete_bonus
		# False면 26_09_04_15 이전처럼 목표 반경 안에 있는 매 스텝 도달 보너스를 준다.
		# 회귀 원인 절제용 — 기본값은 목표당 1회(현행)다.
		self.arrive_once = arrive_once
		# 중계 슬롯 간격 = relay_beta * comm_range. 1.0 미만으로 두어 드론이 정확히
		# 슬롯에 서지 못해도 링크가 끊기지 않을 여유를 남긴다.
		self.relay_beta = relay_beta
		# 중계 정박 보상. 시간 페널티(0.01)의 2배로 두어 자리를 지킬 이유는 주되
		# 배송 보상(1.0)을 압도하지 않게 한다. 동료를 실제로 지탱할 때만 지급된다.
		self.relay_hold = relay_hold
		self.cluster_frac = 0.30          # 이보다 가까우면 뭉침으로 보고 페널티
		self.max_speed = 50.0 * (1000 / 3600)
		self.unload_time = 10
		self.interaction_radius = 25.0
		self.shape_scale = 0.010          # 하위 목표까지 거리 감소분에 곱하는 계수
		self.arrive_bonus = 1.0           # 하위 목표 도달 시 1회성 보너스
		self.time_penalty = 0.01
		self.rng = np.random.default_rng()

		loaded = False
		if map_path and os.path.exists(map_path):
			d = np.load(map_path, allow_pickle=True).item()
			if len(d['dests_pos']) == self.num_dests:
				self.cc_pos, self.dests_pos = d['cc_pos'], d['dests_pos']
				loaded = True
		if not loaded:
			self.cc_pos, self.dests_pos = sample_instance(
				self.num_dests, seed=0, num_drones=self.num_drones, comm_range=self.comm_range)

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
		self.relay_slot = np.full(self.num_drones, -1)  # 점유 중인 중계 슬롯 번호 (-1=중계 아님)
		self.goal_age = np.zeros(self.num_drones, dtype=int)   # 현재 목표를 받은 뒤 지난 스텝
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
		"""드론 i의 후보 목적지: 최근접 (n_cand-n_far)곳 + 제어 센터에서 가장 먼 n_far곳."""
		act = np.flatnonzero(self.dests_active)
		out = np.full(self.n_cand, -1)
		if len(act) == 0:
			return out
		d = np.linalg.norm(self.dests_pos[act] - self.drones_pos[i], axis=1)
		near = list(act[np.argsort(d)[:self.n_cand - self.n_far]])
		# 먼 곳을 후보에 넣어야 편대가 온전할 때 처리할 수 있다. 최근접만 고르면
		# 가까운 목적지가 다 사라진 뒤에야 후보로 떠올라 마지막에 남는다.
		far_d = np.linalg.norm(self.dests_pos[act] - self.cc_pos, axis=1)
		far = [j for j in act[np.argsort(-far_d)] if j not in near][:self.n_far]
		pick = near + far
		out[:len(pick)] = pick
		return out

	def _frontier(self, i):
		"""드론 i가 지지해야 할 최원점과 필요한 중계 슬롯 수를 구한다."""
		# 다른 드론의 배송 목표와 현재 위치를 함께 본다. 목표점만 쓰면 선두가
		# 가까운 목적지로 목표를 낮추는 순간 체인이 뒤로 밀려 정체한다.
		pts = [self.goal_pos[j] for j in range(self.num_drones)
		       if j != i and self.goal_kind[j] == GOAL_DELIVER]
		pts += [self.drones_pos[j] for j in range(self.num_drones)
		        if j != i and self.goal_kind[j] != GOAL_RELAY]
		if not pts:
			act = np.flatnonzero(self.dests_active)
			if len(act) == 0:
				return None, 0
			far = act[np.argmax(np.linalg.norm(self.dests_pos[act] - self.cc_pos, axis=1))]
			pts = [self.dests_pos[far]]
		f = max(pts, key=lambda x: float(np.linalg.norm(x - self.cc_pos)))
		d = float(np.linalg.norm(f - self.cc_pos))
		hops = max(1, int(np.ceil(d / (self.relay_beta * self.comm_range))))
		return f, min(hops - 1, self.num_drones - 1)

	def relay_slot_for(self, i):
		"""드론 i가 맡을 중계 슬롯 번호와 좌표를 구한다 (없으면 -1, None)."""
		f, n_slot = self._frontier(i)
		if f is None or n_slot < 1:
			return -1, None, n_slot
		taken = {self.relay_slot[j] for j in range(self.num_drones)
		         if j != i and self.goal_kind[j] == GOAL_RELAY and self.relay_slot[j] > 0}
		free = [k for k in range(1, n_slot + 1) if k not in taken]
		if not free:
			return -1, None, n_slot
		pt = {k: self.cc_pos + (f - self.cc_pos) * (k / (n_slot + 1)) for k in free}
		k = min(free, key=lambda k: float(np.linalg.norm(self.drones_pos[i] - pt[k])))
		return k, pt[k], n_slot

	def set_goals(self, actions):
		"""상위 행동(드론별 정수)을 하위 목표로 변환한다.

		0..n_cand-1 = 해당 후보 목적지로 배송, n_cand = 제어 센터 복귀,
		n_cand+1 = 중계 슬롯으로 이동. 중계는 다른 드론의 배정이 모두 정해진 뒤
		계산해야 프런티어가 확정되므로 두 번에 나눠 처리한다.
		"""
		relay = []
		prev = (self.goal_kind.copy(), self.goal_dest.copy(), self.relay_slot.copy())
		for i, a in enumerate(actions):
			a = int(a)
			if a == self.n_cand:
				self.goal_kind[i], self.goal_pos[i], self.goal_dest[i] = \
					GOAL_RETURN, self.cc_pos.copy(), -1
				self.relay_slot[i] = -1
			elif a == self.n_cand + 1:
				relay.append(i)
			elif 0 <= a < self.n_cand:
				cand = self.candidates(i)[a]
				if cand < 0:                       # 후보가 없으면 유지로 대체
					self.goal_kind[i], self.goal_pos[i], self.goal_dest[i] = \
						GOAL_HOLD, self.drones_pos[i].copy(), -1
				else:
					self.goal_kind[i], self.goal_pos[i], self.goal_dest[i] = \
						GOAL_DELIVER, self.dests_pos[cand].copy(), cand
				self.relay_slot[i] = -1
			else:
				raise ValueError(f"상위 행동 {a}는 0..{self.n_cand + 1} 범위를 벗어난다")
		for i in relay:
			k, pt, _n = self.relay_slot_for(i)
			if k < 0:                              # 설 자리가 없으면 제자리 유지
				self.goal_kind[i], self.goal_pos[i], self.goal_dest[i] = \
					GOAL_HOLD, self.drones_pos[i].copy(), -1
				self.relay_slot[i] = -1
			else:
				self.goal_kind[i], self.goal_pos[i], self.goal_dest[i] = \
					GOAL_RELAY, pt.copy(), -1
				self.relay_slot[i] = k
		# 같은 목표를 다시 받은 드론은 이어서 간다. 전부 초기화하면 20스텝마다 도달
		# 상태가 지워져 목표 유지(commitment)를 판단할 수 없다.
		changed = ((self.goal_kind != prev[0]) | (self.goal_dest != prev[1])
		           | (self.relay_slot != prev[2]))
		self.goal_reached[changed] = False
		self.goal_age[changed] = 0
		self.prev_goal_dist = self._goal_dists()

	def committed(self, i):
		"""드론 i가 아직 유효한 목표로 가는 중이라 재결정을 미뤄야 하는가."""
		if self.goal_age[i] >= self.commit_max or self.goal_reached[i]:
			return False
		k = self.goal_kind[i]
		if k == GOAL_DELIVER:
			return (self.goal_dest[i] >= 0 and self.dests_active[self.goal_dest[i]]
			        and self.drones_capacity[i] > 0)
		if k == GOAL_RETURN:
			return self.drones_capacity[i] < self.max_capacity
		if k == GOAL_RELAY:
			_k, _pt, n_slot = self.relay_slot_for(i)
			return 1 <= self.relay_slot[i] <= n_slot
		return False

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
			elif k == GOAL_RELAY:
				# 체인이 짧아져 맡은 슬롯이 사라졌거나 배송이 끝났을 때만 다시 고른다.
				# 매 스텝 재결정하면 자리를 잡기 전에 목표가 바뀌어 정박하지 못한다.
				_k, _pt, n_slot = self.relay_slot_for(i)
				bad[i] = (self.relay_slot[i] < 1 or self.relay_slot[i] > n_slot
				          or not self.dests_active.any())
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

	def _is_cut(self, i):
		"""드론 i를 빼면 제어 센터와 끊기는 동료가 생기는가 (중계 기여 판정)."""
		keep = [j for j in range(self.num_drones) if j != i]
		if not keep:
			return False
		full = self._bfs_connected(self.drones_pos)
		sub = self._bfs_connected(self.drones_pos[keep])
		return bool(any(full[j] and not sub[n] for n, j in enumerate(keep)))

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
		self.goal_age += 1
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
				if curr[i] <= self.interaction_radius:
					if self.goal_kind[i] == GOAL_RELAY:
						# 중계는 도달 보너스를 받지 않는다. 매 스텝 1.0을 주면 슬롯에
						# 주차하는 것이 시간 페널티의 100배 이득이 되어 배송이 사라진다
						# (실측: 하위 학습이 ep100 이후 배송 17.4 -> 0.0으로 붕괴).
						# 대신 실제로 동료의 제어 센터 경로를 지탱할 때만 소액을 준다.
						if self._is_cut(i):
							w_rew[i] += self.relay_hold
						self.goal_reached[i] = True
					elif not (self.arrive_once and self.goal_reached[i]):
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
		if done and self.term_reason == "complete" and self.complete_bonus > 0.0:
			team += self.complete_bonus * max(0.0, 1.0 - self.current_step / self.max_steps)
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
			# 중계(3)는 배송(0) 슬롯에 싣는다. 하위 입장에서 중계는 '지정된 점으로 가서
			# 머문다'라 배송과 같은 일이고, 유지 슬롯에 실으면 성능이 절반으로 떨어진다
			# (같은 하위·같은 상위 규칙에서 배송 36.8 대 18.5). 차원은 16으로 불변이다.
			k = int(self.goal_kind[i])
			kind[0 if k == GOAL_RELAY else k] = 1.0

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
			# 중계 6종: 맡을 슬롯까지의 거리·방향과 필요 슬롯 수, 하역 잔여, 컷 정점 여부.
			# 상위가 '중계를 고르면 어디에 서게 되는지'를 보지 못하면 그 행동의 가치를
			# 추정할 수 없다. 하역 잔여는 선두가 하역 중이라 체인이 멈춘 상황을 알려준다.
			k, pt, n_slot = self.relay_slot_for(i)
			if k < 0:
				rel = [0.0, 0.0, 0.0, 0.0]
			else:
				v = pt - self.drones_pos[i]
				d = float(np.linalg.norm(v))
				rel = [d / self.map_size[0], *(v / d if d > 1e-6 else np.zeros(2)),
				       n_slot / self.num_drones]
			out.append(np.array([
				self.drones_capacity[i] / self.max_capacity,
				np.linalg.norm(self.drones_pos[i] - self.cc_pos) / self.map_size[0],
				h[i] / 4.0,
				self.dests_active.sum() / self.num_dests,
				self.current_step / 1000.0,
				*stuck,
				*cf, *pf,
				*rel,
				min(self.drones_timer[i], self.unload_time) / self.unload_time,
				float(self._is_cut(i)),
			]))
		return np.array(out)

	def manager_set_obs(self):
		"""상위 집합 관측: 드론 수·목적지 수와 무관한 형태. 거리는 통신 반경으로 정규화한다.

		self  (N, 10)        자기 상태
		cand  (N, n_cand, 6) 후보 목적지 엔티티, cand_mask (N, n_cand)
		peer  (N, N, 9)      동료 엔티티 (자기 자신은 peer_mask로 제외)
		relay (N, 4)         맡을 중계 슬롯
		"""
		R, N = self.comm_range, self.num_drones
		h = self._hop_depth()
		adv = self.can_advance() if self.stuck_obs else np.ones(N)
		d_cc = np.linalg.norm(self.drones_pos - self.cc_pos, axis=1)
		selfv = np.zeros((N, 10), dtype=np.float32)
		cand = np.zeros((N, self.n_cand, 6), dtype=np.float32)
		cmask = np.zeros((N, self.n_cand), dtype=bool)
		peer = np.zeros((N, N, 9), dtype=np.float32)
		pmask = ~np.eye(N, dtype=bool)
		relay = np.zeros((N, 4), dtype=np.float32)
		remaining = self.dests_active.sum() / self.num_dests
		for i in range(N):
			selfv[i] = [self.drones_capacity[i] / self.max_capacity, d_cc[i] / R, h[i] / N_MAX,
			            remaining, self.current_step / self.max_steps,
			            self.move_ema[i] if self.stuck_obs else 1.0, adv[i],
			            self.team_move_ema if self.stuck_obs else 1.0,
			            min(self.drones_timer[i], self.unload_time) / self.unload_time,
			            float(self._is_cut(i))]
			for k, c in enumerate(self.candidates(i)):
				if c < 0:
					continue
				v = self.dests_pos[c] - self.drones_pos[i]
				d = float(np.linalg.norm(v))
				dc = float(np.linalg.norm(self.dests_pos[c] - self.cc_pos))
				taken = float(np.any((self.goal_dest == c) & (np.arange(N) != i)))
				hops = np.ceil(dc / (self.relay_beta * R))
				cand[i, k] = [d / R, *(v / d if d > 1e-6 else np.zeros(2)), taken, dc / R, hops / N_MAX]
				cmask[i, k] = True
			for j in range(N):
				if j == i:
					continue
				v = self.drones_pos[j] - self.drones_pos[i]
				d = float(np.linalg.norm(v))
				kind = np.zeros(4)
				kind[int(self.goal_kind[j])] = 1.0
				peer[i, j] = [d / R, v[0] / R, v[1] / R, self.drones_capacity[j] / self.max_capacity,
				              h[j] / N_MAX, *kind]
			k, pt, n_slot = self.relay_slot_for(i)
			if k >= 0:
				v = pt - self.drones_pos[i]
				d = float(np.linalg.norm(v))
				relay[i] = [d / R, *(v / d if d > 1e-6 else np.zeros(2)), n_slot / N_MAX]
		return dict(self=selfv, cand=cand, cand_mask=cmask, peer=peer, peer_mask=pmask, relay=relay)

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
