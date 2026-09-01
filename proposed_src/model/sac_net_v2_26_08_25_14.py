"""SAC Actor / Twin-Q Critic 네트워크 (v2, 26_08_25_14).

Actor는 각 드론의 local obs만, Critic은 global state + 전체 action을 받는다.
평가용 결정론적 행동 경로가 추가된 것 외에 구조 변경은 없다.
"""

import torch
import torch.nn as nn

# --- 3. SAC Networks ---
# Fix 1: Self-Attention Actor를 진정한 CTDE MLP로 교체.
# 각 드론이 자신의 local obs만으로 독립적으로 행동을 결정한다.
# 이웃 정보는 환경 관측치(peer1/peer2 특징)에 이미 인코딩되어 있으므로
# MLP만으로도 릴레이 협력 행동을 학습할 수 있다.
class SACActor(nn.Module):
	def __init__(self, obs_dim, action_dim):
		super().__init__()
		self.net = nn.Sequential(
			nn.Linear(obs_dim, 256), nn.LayerNorm(256), nn.ReLU(),
			nn.Linear(256, 256), nn.LayerNorm(256), nn.ReLU(),
		)
		self.mu = nn.Linear(256, action_dim)
		self.log_std = nn.Linear(256, action_dim)

	def forward(self, obs, deterministic=False):
		# obs: (..., obs_dim) — 배치/드론 차원에 무관하게 마지막 차원에만 적용
		# deterministic=True는 평가 전용 경로로, 샘플링 대신 분포의 평균을 쓴다
		x = self.net(obs)
		mu = self.mu(x)
		log_std = torch.clamp(self.log_std(x), -20, 2)
		std = torch.exp(log_std)
		dist = torch.distributions.Normal(mu, std)
		z = mu if deterministic else dist.rsample()
		a = torch.tanh(z)
		lp = (dist.log_prob(z) - torch.log(1 - a.pow(2) + 1e-6)).sum(-1, keepdim=True)
		return a, lp


class SACCritic(nn.Module):
	def __init__(self, state_dim, act_dim):
		super().__init__()
		self.q1 = nn.Sequential(
			nn.Linear(state_dim + act_dim, 256), nn.ReLU(),
			nn.Linear(256, 256), nn.ReLU(),
			nn.Linear(256, 1)
		)
		self.q2 = nn.Sequential(
			nn.Linear(state_dim + act_dim, 256), nn.ReLU(),
			nn.Linear(256, 256), nn.ReLU(),
			nn.Linear(256, 1)
		)

	def forward(self, s, a):
		sa = torch.cat([s, a], dim=1)
		return self.q1(sa), self.q2(sa)
