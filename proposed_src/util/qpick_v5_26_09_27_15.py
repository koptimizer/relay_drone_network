"""학습된 critic의 탐욕 정책이 actor보다 나은지 본다 (추가 학습 없는 비교).

상위 critic은 드론별 Q를 내고 팀 값은 그 평균이므로, 결합 행동의 최선은 드론별 argmax와 같다.
actor 표본·actor 최빈·critic 탐욕 세 가지를 같은 롤아웃에서 비교한다.
"""

import argparse
import os
import sys

import numpy as np
import torch
from scipy import stats

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
from model.hier_net_v5_26_09_15_23 import SetManagerActor, SetManagerTwinQ
from pipeline.common_v5_26_09_15_22 import action_mask, chain_escape_manager, straight_worker
from util.instance_generator_v5_26_09_15_22 import sample_instance

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def actor_fn(ck, dev, ar, argmax=False):
	"""actor 가중치를 표본 또는 최빈 결정 함수로 감싼다 (latest.pth면 actor 키를 꺼낸다)."""
	m = SetManagerActor(autoregressive=ar).to(dev)
	sd = torch.load(os.path.join(ROOT, ck), map_location=dev)
	m.load_state_dict(sd["actor"] if "actor" in sd else sd)
	m.eval()

	def fn(e):
		o = {k: torch.as_tensor(v, device=dev).unsqueeze(0) for k, v in e.manager_set_obs().items()}
		with torch.no_grad():
			act, probs, _ = m(o, action_mask(e, dev).unsqueeze(0))
		return (probs.argmax(-1) if argmax else act)[0].cpu().numpy()
	return fn


def critic_fn(ck, dev, n_act, mix=False):
	"""latest.pth의 critic을 드론별 Q 탐욕 정책으로 감싼다 (단조 혼합이면 argmax는 그대로다)."""
	c = SetManagerTwinQ(n_act, mix=mix).to(dev)
	c.load_state_dict(torch.load(os.path.join(ROOT, ck), map_location=dev)["critic"])
	c.eval()

	def fn(e):
		o = {k: torch.as_tensor(v, device=dev).unsqueeze(0) for k, v in e.manager_set_obs().items()}
		msk = action_mask(e, dev).unsqueeze(0)
		with torch.no_grad():
			q1, q2 = c(o, o["drone_mask"].unsqueeze(0) if "drone_mask" in o else None)
			q = torch.min(q1, q2).masked_fill(~msk, -1e9)
		return q.argmax(-1)[0].cpu().numpy()
	return fn


def critic_seq_fn(ck, dev, n_act):
	"""순차 크레딧으로 학습한 critic의 탐욕 정책. 학습과 같은 순서로 한 대씩 argmax를 밟는다.

	동시 argmax로 재면 학습 때 본 적 없는 관측(앞 드론의 역할이 비어 있는 상태)에서 평가하게 되어
	그 critic의 정책이 아니다. 지표를 바꾸면 그 지표를 쓰는 경로를 함께 바꿔야 한다.
	"""
	c = SetManagerTwinQ(n_act).to(dev)
	c.load_state_dict(torch.load(os.path.join(ROOT, ck), map_location=dev)["critic"])
	c.eval()

	def fn(e):
		o = {k: torch.as_tensor(v, device=dev).unsqueeze(0) for k, v in e.manager_set_obs().items()}
		msk = action_mask(e, dev).unsqueeze(0)
		N = e.num_drones
		peer = o["peer"].clone()
		order = torch.argsort(o["self"][:, :, 1], dim=1, descending=True, stable=True)
		act = np.zeros(N, dtype=np.int64)
		with torch.no_grad():
			for s in range(N):
				j = int(order[0, s])
				oo = dict(o)
				oo["peer"] = peer
				q1, q2 = c(oo)
				q = torch.min(q1, q2)[0, j].masked_fill(~msk[0, j], -1e9)
				aj = int(q.argmax())
				act[j] = aj
				kind = 3 if aj >= n_act - 1 else (1 if aj == n_act - 2 else 0)
				peer = peer.clone()
				peer[0, :, j, 5:9] = 0.0
				peer[0, :, j, 5 + kind] = 1.0
		return act
	return fn


def run(env, mf, seed, cc):
	"""한 인스턴스를 굴려 makespan(미완주면 None)을 돌려준다."""
	env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=seed, num_drones=env.num_drones,
	                                            comm_range=env.comm_range, cc_pos=cc)
	env.reset()
	env.set_goals(mf(env))
	done, t = False, 0
	while not done and t < env.max_steps:
		_o, _w, _r, done = env.step(straight_worker(env))
		t += 1
		if t % 20 == 0 or env.goal_invalid().any():
			env.set_goals(mf(env))
	s = env.episode_stats()
	return s["makespan"] if s["makespan"] else None


def main():
	"""세 정책을 같은 롤아웃에서 비교한다."""
	p = argparse.ArgumentParser()
	p.add_argument("--dir", default="weights/v5_26_09_23_18_SWa")
	p.add_argument("--autoregressive", action="store_true")
	p.add_argument("--mix-critic", action="store_true", help="단조 혼합망으로 학습한 critic을 읽는다")
	p.add_argument("--seq-decision", action="store_true", help="순차 크레딧 critic — 탐욕 정책도 순차로 밟는다")
	p.add_argument("--n", type=int, default=40)
	p.add_argument("--reps", type=int, default=3)
	p.add_argument("--seed0", type=int, default=501)
	p.add_argument("--num-drones", type=int, default=3)
	p.add_argument("--num-dests", type=int, default=30)
	p.add_argument("--fixed-cc", action="store_true")
	p.add_argument("--actor-latest", action="store_true", help="actor도 latest.pth에서 꺼낸다 (critic과 같은 시점)")
	args = p.parse_args()

	torch.set_num_threads(2)
	dev = torch.device("cpu")
	env = DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=10000,
	                            deadlock_limit=10 ** 9, no_progress_limit=10 ** 9,
	                            num_drones=args.num_drones, num_dests=args.num_dests)
	env.reset()
	cc = None if args.fixed_cc else "random"
	n_act = env.n_cand + 2
	src = f"{args.dir}/latest.pth" if args.actor_latest else f"{args.dir}/best_manager.pth"
	pol = (("actor 표본", actor_fn(src, dev, args.autoregressive)),
	       ("actor 최빈", actor_fn(src, dev, args.autoregressive, True)),
	       ("critic 탐욕", critic_seq_fn(f"{args.dir}/latest.pth", dev, n_act) if args.seq_decision
	        else critic_fn(f"{args.dir}/latest.pth", dev, n_act, args.mix_critic)),
	       ("규칙+탈출", chain_escape_manager(60, 40, "shuffle")))
	seeds = [(r, s) for r in range(args.reps) for s in range(args.seed0, args.seed0 + args.n)]
	res = {}
	for nm, mf in pol:
		v = np.array([x if x else np.nan for x in (run(env, mf, s, cc) for _r, s in seeds)], dtype=float)
		res[nm] = v
		ok = ~np.isnan(v)
		print(f"{nm:<12} 완주 {int(ok.sum())}/{len(v)}  중앙 {np.median(v[ok]):>5.0f}  "
		      f"절단평균 {stats.trim_mean(v[ok], .1):>5.0f}  최대 {v[ok].max():>5.0f}", flush=True)
	base = res["actor 표본"]
	for nm in list(res)[1:]:
		m = ~np.isnan(base) & ~np.isnan(res[nm])
		d = res[nm][m] - base[m]
		print(f"  {nm} 대 actor 표본: 쌍별 중앙 {np.median(d):+.0f} 승 {int((d < 0).sum())} "
		      f"패 {int((d > 0).sum())} p={stats.wilcoxon(d).pvalue:.4f}")


if __name__ == "__main__":
	main()
