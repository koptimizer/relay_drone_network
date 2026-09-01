"""계층 정책 네트워크 (v3, 26_08_31_19).

상위는 하위 목표를 고르는 이산 정책, 하위는 2D 추력을 내는 연속 정책이다.
두 층 모두 CTDE — actor는 local 관측만, critic은 학습 시 global state를 본다.
"""

import torch
import torch.nn as nn
import torch.nn.functional as F


def mlp(inp, out, hidden=256):
	"""LayerNorm을 낀 2층 MLP 몸통을 만든다."""
	return nn.Sequential(
		nn.Linear(inp, hidden), nn.LayerNorm(hidden), nn.ReLU(),
		nn.Linear(hidden, hidden), nn.LayerNorm(hidden), nn.ReLU(),
		nn.Linear(hidden, out),
	)


class WorkerActor(nn.Module):
	"""하위 정책. 하위 목표까지 가는 2D 추력을 낸다.

	행동은 (v_x, v_y) ∈ [-1,1]²로, 각도 스칼라 표현의 SO(2) 불연속과
	최대 속도에 tanh 포화가 필요한 문제를 함께 피한다.
	"""

	def __init__(self, obs_dim, act_dim=2, hidden=256):
		super().__init__()
		self.net = nn.Sequential(
			nn.Linear(obs_dim, hidden), nn.LayerNorm(hidden), nn.ReLU(),
			nn.Linear(hidden, hidden), nn.LayerNorm(hidden), nn.ReLU(),
		)
		self.mu = nn.Linear(hidden, act_dim)
		self.log_std = nn.Linear(hidden, act_dim)

	def forward(self, obs, deterministic=False):
		"""추력과 로그확률을 반환한다. deterministic이면 분포 평균을 쓴다."""
		x = self.net(obs)
		mu = self.mu(x)
		log_std = torch.clamp(self.log_std(x), -20, 2)
		std = torch.exp(log_std)
		dist = torch.distributions.Normal(mu, std)
		z = mu if deterministic else dist.rsample()
		a = torch.tanh(z)
		lp = (dist.log_prob(z) - torch.log(1 - a.pow(2) + 1e-6)).sum(-1, keepdim=True)
		return a, lp


class ManagerActor(nn.Module):
	"""상위 정책. 후보 목적지 K개 + 복귀 + 유지 중 하나를 고른다.

	이산 SAC용으로 전체 행동 확률을 함께 반환해 기대 Q와 엔트로피를 정확히 계산한다.
	"""

	def __init__(self, obs_dim, n_actions, hidden=256):
		super().__init__()
		self.net = mlp(obs_dim, n_actions, hidden)

	def forward(self, obs, mask=None):
		"""(행동, 확률, 로그확률)을 반환한다. mask가 False인 행동은 선택되지 않는다."""
		logits = self.net(obs)
		if mask is not None:
			logits = logits.masked_fill(~mask, -1e9)
		probs = F.softmax(logits, dim=-1)
		logp = torch.log(probs + 1e-8)
		action = torch.distributions.Categorical(probs=probs).sample()
		return action, probs, logp


class TwinQ(nn.Module):
	"""중앙 critic. global state와 전체 행동을 받아 두 개의 Q를 낸다."""

	def __init__(self, state_dim, act_dim, hidden=256):
		super().__init__()
		self.q1 = mlp(state_dim + act_dim, 1, hidden)
		self.q2 = mlp(state_dim + act_dim, 1, hidden)

	def forward(self, s, a):
		sa = torch.cat([s, a], dim=-1)
		return self.q1(sa), self.q2(sa)


class ManagerTwinQ(nn.Module):
	"""상위용 중앙 critic. global state에서 드론별 전체 행동 가치를 낸다.

	이산 행동이므로 행동을 입력으로 받지 않고 모든 행동의 Q를 한 번에 출력한다.
	"""

	def __init__(self, state_dim, num_drones, n_actions, hidden=256):
		super().__init__()
		self.n, self.k = num_drones, n_actions
		self.q1 = mlp(state_dim, num_drones * n_actions, hidden)
		self.q2 = mlp(state_dim, num_drones * n_actions, hidden)

	def forward(self, s):
		"""(batch, 드론, 행동) 형태의 Q 두 벌을 반환한다."""
		shape = s.shape[:-1] + (self.n, self.k)
		return self.q1(s).view(shape), self.q2(s).view(shape)
