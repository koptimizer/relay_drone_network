"""학습용 리플레이 버퍼와 보상 정규화기 (v1, 26-08-24-22).

CTDERL.py에서 RewardNormalizer, EfficientReplayBuffer를 분리한 것으로 로직 변경은 없다.
"""

import numpy as np
import torch

# --- 1. Reward Normalizer ---
# Critic 스파이크 원인: 보상 범위가 -100 ~ +40으로 극단적.
# 샘플링 후 Bellman 타깃 계산 시 정규화하여 Q-값 스케일을 안정화한다.
# 수집 시점이 아닌 학습 시점에 정규화하므로 환경 탐색 행동에 영향을 주지 않는다.
class RewardNormalizer:
	def __init__(self):
		self.mean  = 0.0
		self.var   = 1.0
		self.count = 1e-4   # 0 나눗셈 방지를 위한 초기값

	def update(self, x: torch.Tensor):
		"""배치 통계로 running mean/var 갱신 (Welford 방식)."""
		x_np = x.detach().cpu().numpy().ravel()
		n = len(x_np)
		delta = x_np.mean() - self.mean
		total = self.count + n
		self.mean += delta * n / total
		self.var   = (self.var * self.count + x_np.var() * n
					  + delta ** 2 * self.count * n / total) / total
		self.count = total

	def normalize(self, x: torch.Tensor) -> torch.Tensor:
		"""스케일만 정규화한다 — 평균을 빼면 드리프트가 Q 타깃을 비정상 이동시킨다."""
		std = (self.var ** 0.5 + 1e-8)
		return torch.clamp(x / std, -10.0, 10.0)


# --- 2. Efficient Replay Buffer ---
# Fix 2: rewards를 (capacity, num_drones) shape으로 저장해 개별 기여도 보존
class EfficientReplayBuffer:
	def __init__(self, num_drones, obs_dim, state_dim, action_dim, capacity=100000):
		self.capacity = capacity
		self.num_drones = num_drones
		self.ptr = 0
		self.size = 0

		self.obs = np.zeros((capacity, num_drones, obs_dim), dtype=np.float32)
		self.state = np.zeros((capacity, state_dim), dtype=np.float32)
		self.actions = np.zeros((capacity, num_drones * action_dim), dtype=np.float32)
		self.rewards = np.zeros((capacity, num_drones), dtype=np.float32)  # Fix 2
		self.next_obs = np.zeros((capacity, num_drones, obs_dim), dtype=np.float32)
		self.next_state = np.zeros((capacity, state_dim), dtype=np.float32)
		self.dones = np.zeros((capacity, 1), dtype=np.float32)

	def push(self, obs, state, action, reward, next_obs, next_state, done):
		self.obs[self.ptr] = obs
		self.state[self.ptr] = state
		self.actions[self.ptr] = action
		self.rewards[self.ptr] = reward   # Fix 2: (num_drones,) 배열 그대로 저장
		self.next_obs[self.ptr] = next_obs
		self.next_state[self.ptr] = next_state
		self.dones[self.ptr, 0] = done
		self.ptr = (self.ptr + 1) % self.capacity
		self.size = min(self.size + 1, self.capacity)

	def sample(self, batch_size, device):
		ind = np.random.randint(0, self.size, size=batch_size)
		return (
			torch.FloatTensor(self.obs[ind]).to(device),
			torch.FloatTensor(self.state[ind]).to(device),
			torch.FloatTensor(self.actions[ind]).to(device),
			torch.FloatTensor(self.rewards[ind]).to(device),   # (batch, num_drones)
			torch.FloatTensor(self.next_obs[ind]).to(device),
			torch.FloatTensor(self.next_state[ind]).to(device),
			torch.FloatTensor(self.dones[ind]).to(device)
		)

	def __len__(self):
		return self.size
