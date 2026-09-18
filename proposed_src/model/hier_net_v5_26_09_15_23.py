"""집합 기반 상위 정책 네트워크 (v5, 26_09_15_23).

드론 수·목적지 수에 무관하게 동작한다. 관측을 엔티티 집합(자기·후보·동료·중계)으로 받아
어텐션으로 풀링하고, 후보 목적지는 포인터 방식으로 점수를 매긴다. 행동 인터페이스는
v3/v4와 같다 — (행동, 확률, 로그확률), 행동 수 K = n_cand + 2.
critic은 드론 임베딩 사이의 어텐션으로 팀 상태를 보고 드론별 Q를 낸다.
"""

import torch
import torch.nn as nn
import torch.nn.functional as F

F_SELF, F_CAND, F_PEER, F_RELAY = 10, 6, 9, 4


def mlp(inp, out, hidden=128):
	"""LayerNorm을 낀 2층 MLP."""
	return nn.Sequential(
		nn.Linear(inp, hidden), nn.LayerNorm(hidden), nn.ReLU(),
		nn.Linear(hidden, out),
	)


class EntityEncoder(nn.Module):
	"""엔티티 집합을 드론별 문맥 벡터 h 와 후보 임베딩으로 바꾼다.

	자기 임베딩을 질의로 동료 집합·후보 집합에 각각 어텐션을 걸어 풀링하므로
	동료·후보의 수가 얼마든 출력 차원이 같다.
	"""

	def __init__(self, d=128, heads=4, advice=False):
		super().__init__()
		self.d, self.advice = d, advice
		# 규칙 조언(원핫 7)을 엔티티로 받으면 정책은 '규칙을 따를지 벗어날지'만 배우면 된다
		self.enc_adv = mlp(7, d, d) if advice else None
		self.enc_self = mlp(F_SELF, d, d)
		self.enc_cand = mlp(F_CAND, d, d)
		self.enc_peer = mlp(F_PEER, d, d)
		self.enc_relay = mlp(F_RELAY, d, d)
		self.att_peer = nn.MultiheadAttention(d, heads, batch_first=True)
		self.att_cand = nn.MultiheadAttention(d, heads, batch_first=True)
		self.fuse = mlp((5 if advice else 4) * d, d, d)

	def forward(self, o):
		"""o: dict of (B, N, ...) 텐서. 반환 h (B, N, d), cand_emb (B, N, C, d)."""
		B, N = o["self"].shape[:2]
		e_self = self.enc_self(o["self"])                       # (B,N,d)
		e_cand = self.enc_cand(o["cand"])                       # (B,N,C,d)
		e_peer = self.enc_peer(o["peer"])                       # (B,N,N,d)
		e_rel = self.enc_relay(o["relay"])                      # (B,N,d)
		q = e_self.reshape(B * N, 1, self.d)
		# 유효 엔티티가 하나도 없는 행은 어텐션이 NaN을 내므로 자기 자신을 더미로 넣는다
		pm = ~o["peer_mask"].reshape(B * N, N)
		pm[pm.all(1), 0] = False
		cm = ~o["cand_mask"].reshape(B * N, -1)
		cm[cm.all(1), 0] = False
		c_peer, _ = self.att_peer(q, e_peer.reshape(B * N, N, self.d), e_peer.reshape(B * N, N, self.d),
		                          key_padding_mask=pm)
		c_cand, _ = self.att_cand(q, e_cand.reshape(B * N, -1, self.d), e_cand.reshape(B * N, -1, self.d),
		                          key_padding_mask=cm)
		parts = [e_self, c_peer.reshape(B, N, self.d), c_cand.reshape(B, N, self.d), e_rel]
		if self.advice:
			parts.append(self.enc_adv(o["advice"]))
		h = self.fuse(torch.cat(parts, dim=-1))
		return h, e_cand


class SetManagerActor(nn.Module):
	"""집합 관측에서 드론별 이산 행동 분포를 낸다. 후보는 포인터, 복귀·중계는 고정 헤드."""

	def __init__(self, d=128, advice=False, autoregressive=False):
		super().__init__()
		self.autoregressive = autoregressive
		self.enc = EntityEncoder(d, advice=advice)
		self.point = mlp(2 * d, 1, d)
		self.fixed = mlp(d, 2, d)

	def _logits(self, o):
		"""집합 관측에서 (B,N,K) 로짓."""
		h, e_cand = self.enc(o)                                 # (B,N,d), (B,N,C,d)
		C = e_cand.shape[2]
		hc = h.unsqueeze(2).expand(-1, -1, C, -1)
		l_cand = self.point(torch.cat([hc, e_cand], dim=-1)).squeeze(-1)   # (B,N,C)
		l_fix = self.fixed(h)                                              # (B,N,2)
		return torch.cat([l_cand, l_fix], dim=-1)                          # (B,N,K)

	def forward(self, o, mask=None, given=None):
		"""(행동, 확률, 로그확률)을 (B, N, ·) 형태로 반환한다.

		autoregressive=True면 제어 센터에서 먼 드론부터 한 대씩 결정하고, 결정된 드론의 새 역할을
		동료 관측(peer의 kind 원핫)에 써넣은 뒤 다음 드론을 결정한다. 편대 대형(선두 1 + 중계 k)은
		드론별 독립 표본으로는 우연히만 나오지만, 순차 결정에서는 앞 드론의 선택을 보고 맞출 수 있다.
		given이 있으면 그 행동으로 조건부를 만든다(학습 시 teacher forcing).
		"""
		if not self.autoregressive:
			logits = self._logits(o)
			if mask is not None:
				logits = logits.masked_fill(~mask, -1e9)
			probs = F.softmax(logits, dim=-1)
			logp = torch.log(probs + 1e-8)
			action = torch.distributions.Categorical(probs=probs).sample()
			return action, probs, logp
		B, N = o["self"].shape[:2]
		K = o["cand"].shape[2] + 2
		peer = o["peer"].clone()                                # (B,N,N,9) — kind 원핫은 5:9
		order = torch.argsort(o["self"][:, :, 1], dim=1, descending=True)   # d_cc 큰 순
		action = torch.zeros(B, N, dtype=torch.long, device=peer.device)
		probs = torch.zeros(B, N, K, device=peer.device)
		logp = torch.zeros(B, N, K, device=peer.device)
		bidx = torch.arange(B, device=peer.device)
		for step in range(N):
			j = order[:, step]                                  # (B,) 이번에 결정할 드론
			oo = dict(o); oo["peer"] = peer
			lg = self._logits(oo)[bidx, j]                      # (B,K)
			if mask is not None:
				lg = lg.masked_fill(~mask[bidx, j], -1e9)
			pj = F.softmax(lg, dim=-1)
			aj = given[bidx, j] if given is not None else torch.distributions.Categorical(probs=pj).sample()
			action[bidx, j] = aj
			probs[bidx, j] = pj
			logp[bidx, j] = torch.log(pj + 1e-8)
			# 결정된 드론 j의 새 역할을 다른 드론들이 보는 peer 특징에 써넣는다
			kind = torch.where(aj >= K - 1, 3, torch.where(aj == K - 2, 1, 0))   # 중계=3, 복귀=1, 배송=0
			onehot = F.one_hot(kind, 4).float()                                 # (B,4)
			peer = peer.clone()
			peer[bidx, :, j, 5:9] = onehot.unsqueeze(1).expand(-1, N, -1)
		return action, probs, logp


class SetManagerTwinQ(nn.Module):
	"""집합 관측용 중앙 critic. 드론 임베딩 사이 어텐션으로 팀 상태를 보고 드론별 Q를 낸다."""

	def __init__(self, n_actions, d=128, heads=4, advice=False):
		super().__init__()
		self.k = n_actions
		self.enc1, self.enc2 = EntityEncoder(d, advice=advice), EntityEncoder(d, advice=advice)
		self.team1 = nn.MultiheadAttention(d, heads, batch_first=True)
		self.team2 = nn.MultiheadAttention(d, heads, batch_first=True)
		self.head1, self.head2 = mlp(2 * d, n_actions, d), mlp(2 * d, n_actions, d)

	def _q(self, enc, team, head, o, dmask):
		h, _ = enc(o)                                           # (B,N,d)
		t, _ = team(h, h, h, key_padding_mask=~dmask)
		return head(torch.cat([h, t], dim=-1))                  # (B,N,K)

	def forward(self, o, drone_mask=None):
		"""(B, N, K) 형태의 Q 두 벌."""
		B, N = o["self"].shape[:2]
		dm = torch.ones(B, N, dtype=torch.bool, device=o["self"].device) if drone_mask is None else drone_mask
		return self._q(self.enc1, self.team1, self.head1, o, dm), self._q(self.enc2, self.team2, self.head2, o, dm)
