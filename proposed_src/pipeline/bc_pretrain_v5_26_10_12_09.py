"""상위 정책을 규칙 행동으로 지도학습해 초기 가중치를 만든다 (사이클 23).

사이클 13의 중계 모방 가중치는 실패했지만 그것은 학습 중 정규화였고, 이것은 초기화다.
탐색이 무작위가 아니라 규칙 근방에서 시작하면 중계 배정을 유지한 채 속도를 깎을 수 있다는
가설을 시험한다. 관측에는 규칙의 중계 배정을 85% 맞출 정보가 있다(26-09-27 탐침).
"""

import argparse
import os
import sys

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv, N_MAX
from model.hier_net_v5_26_09_15_23 import SetManagerActor
from pipeline.common_v5_26_09_15_22 import action_mask, chain_escape_manager, chain_manager, straight_worker
from util.instance_generator_v5_26_09_15_22 import sample_config

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def pad(o, n, C=5):
	"""집합 관측을 N_MAX 드론으로 패딩하고 drone_mask를 붙인다 (학습 스크립트와 같은 규약)."""
	out = dict(self=np.zeros((N_MAX, 10), np.float32), cand=np.zeros((N_MAX, C, 6), np.float32),
	           cand_mask=np.zeros((N_MAX, C), bool), peer=np.zeros((N_MAX, N_MAX, 9), np.float32),
	           peer_mask=np.zeros((N_MAX, N_MAX), bool), relay=np.zeros((N_MAX, 4), np.float32),
	           drone_mask=np.zeros(N_MAX, bool))
	out["self"][:n] = o["self"]
	out["cand"][:n] = o["cand"]
	out["cand_mask"][:n] = o["cand_mask"]
	out["peer"][:n, :n] = o["peer"]
	out["peer_mask"][:n, :n] = o["peer_mask"]
	out["relay"][:n] = o["relay"]
	out["drone_mask"][:n] = True
	return out


def collect(A, dev):
	"""규칙+탈출로 굴리며 (관측, 규칙 행동, 마스크)를 모은다. 규칙이 방문하는 상태 분포다."""
	rng = np.random.default_rng(A.seed)
	K = 7
	O, AC, MK = [], [], []
	env = None
	eps = 0
	while len(O) < A.samples:
		n, m, cc, d = sample_config(rng, tuple(A.drones_range), tuple(A.dests_range), random_cc=True)
		if env is None or (n, m) != (env.num_drones, env.num_dests):
			env = DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False,
			                            max_steps=A.max_steps, deadlock_limit=10 ** 9,
			                            no_progress_limit=10 ** 9, num_drones=n, num_dests=m)
		env.cc_pos, env.dests_pos = cc, d
		env.reset()
		mf = chain_escape_manager()
		# 탐색 상태도 섞는다. 규칙 궤적만 보면 학습 초기에 벗어난 상태에서 무너진다
		eps_mix = A.explore
		a = mf(env)
		env.set_goals(a)
		done, t = False, 0
		while not done and t < A.max_steps and len(O) < A.samples:
			_o, _w, _r, done = env.step(straight_worker(env))
			t += 1
			if t % 20 == 0 or env.goal_invalid().any():
				o = pad(env.manager_set_obs(), env.num_drones)
				msk = action_mask(env, dev).cpu().numpy()
				ra = chain_manager(env).astype(np.int64)
				mp = np.zeros((N_MAX, K), bool)
				mp[:env.num_drones] = msk
				mp[env.num_drones:, K - 2] = True
				ac = np.zeros(N_MAX, np.int64)
				ac[:env.num_drones] = ra
				O.append(o); AC.append(ac); MK.append(mp)
				a = mf(env)
				if eps_mix > 0.0:
					# 일부 드론의 목표를 무작위 유효 행동으로 바꿔 규칙 궤적 밖 상태도 만든다
					for i in range(env.num_drones):
						if rng.random() < eps_mix:
							v = np.flatnonzero(msk[i])
							if len(v):
								a[i] = int(rng.choice(v))
				env.set_goals(a)
		eps += 1
	print(f"수집 완료: 결정 {len(O)}개, 에피소드 {eps}개", flush=True)
	st = {k: np.stack([o[k] for o in O]) for k in O[0]}
	return st, np.stack(AC), np.stack(MK)


def main():
	"""규칙 행동으로 actor를 지도학습해 가중치를 저장한다."""
	p = argparse.ArgumentParser()
	p.add_argument("--samples", type=int, default=40000)
	p.add_argument("--epochs", type=int, default=12)
	p.add_argument("--batch", type=int, default=256)
	p.add_argument("--lr", type=float, default=3e-4)
	p.add_argument("--explore", type=float, default=0.15, help="규칙 궤적 밖 상태를 섞는 비율")
	p.add_argument("--drones-range", type=int, nargs=2, default=[3, 4])
	p.add_argument("--dests-range", type=int, nargs=2, default=[20, 80])
	p.add_argument("--max-steps", type=int, default=1000)
	p.add_argument("--seed", type=int, default=1)
	p.add_argument("--autoregressive", action="store_true")
	p.add_argument("--out", default="weights/bc_v5_26_10_12_09/actor.pth")
	A = p.parse_args()

	torch.manual_seed(A.seed)
	np.random.seed(A.seed)
	dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
	st, ac, mk = collect(A, torch.device("cpu"))
	n = len(ac)
	idx = np.random.permutation(n)
	ntr = int(n * 0.9)
	tr, va = idx[:ntr], idx[ntr:]

	actor = SetManagerActor(autoregressive=A.autoregressive).to(dev)
	opt = optim.Adam(actor.parameters(), lr=A.lr)
	T = lambda x, i: torch.as_tensor(x[i], device=dev)
	K = mk.shape[-1]

	def evaluate(sel):
		"""전체 정확도와 중계 결정만의 정확도를 돌려준다."""
		actor.eval()
		hit = tot = rhit = rtot = 0
		with torch.no_grad():
			for s in range(0, len(sel), 1024):
				b = sel[s:s + 1024]
				o = {k: T(v, b) for k, v in st.items()}
				msk = T(mk, b)
				lg = actor.logits(o).masked_fill(~msk, -1e9)
				pred = lg.argmax(-1)
				y = T(ac, b)
				# 학습 표적과 같은 집합에서만 잰다 (규칙 행동이 마스크에 걸린 드론 제외)
				dm = T(st["drone_mask"], b) & msk.gather(-1, y.unsqueeze(-1)).squeeze(-1)
				ok = (pred == y) & dm
				hit += int(ok.sum()); tot += int(dm.sum())
				isr = dm & (y == K - 1)
				rhit += int((ok & isr).sum()); rtot += int(isr.sum())
		actor.train()
		return hit / max(1, tot), rhit / max(1, rtot), rtot / max(1, tot)

	print(f"학습 {len(tr)} / 검증 {len(va)} 결정", flush=True)
	for e in range(1, A.epochs + 1):
		np.random.shuffle(tr)
		tl = 0.0
		for s in range(0, len(tr), A.batch):
			b = tr[s:s + A.batch]
			o = {k: T(v, b) for k, v in st.items()}
			msk = T(mk, b)
			lg = actor.logits(o).masked_fill(~msk, -1e9)
			y, dm = T(ac, b), T(st["drone_mask"], b).float()
			# 규칙 행동이 마스크에 걸린 드론은 표적에서 뺀다. 넣으면 -1e9 로짓이 손실을 지배한다
			dm = dm * msk.gather(-1, y.unsqueeze(-1)).squeeze(-1).float()
			lp = F.log_softmax(lg, dim=-1)
			nll = -lp.gather(-1, y.unsqueeze(-1)).squeeze(-1)
			loss = (nll * dm).sum() / dm.sum().clamp_min(1.0)
			opt.zero_grad(); loss.backward()
			nn.utils.clip_grad_norm_(actor.parameters(), 1.0)
			opt.step()
			tl += float(loss.detach()) * len(b)
		a_all, a_rel, frac = evaluate(va)
		print(f"epoch {e:2d} | 손실 {tl / len(tr):.4f} | 검증 정확도 전체 {a_all:.3f} 중계 {a_rel:.3f} "
		      f"(중계 비중 {frac:.3f})", flush=True)

	out = os.path.join(ROOT, A.out)
	os.makedirs(os.path.dirname(out), exist_ok=True)
	torch.save(actor.state_dict(), out)
	print(f"저장: {out}", flush=True)


if __name__ == "__main__":
	main()
