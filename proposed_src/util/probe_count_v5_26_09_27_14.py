"""중계 '몇 대'가 중요한지 '누가'가 중요한지 가른다 (진단 탐침).

규칙이 지시한 중계 대수만 맞추되 어느 드론이 중계할지는 정책의 중계 확률 순위로 고른다.
대수만 맞춰도 회복되면 원인은 개수, 아니면 배정 대상이다.
"""
import os
import sys

import numpy as np
import torch
from scipy import stats

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
from model.hier_net_v5_26_09_15_23 import SetManagerActor
from pipeline.common_v5_26_09_15_22 import action_mask, chain_escape_manager, chain_manager, straight_worker
from util.instance_generator_v5_26_09_15_22 import sample_instance

R = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def load(ck, dev, ar):
	"""가중치를 올려 (행동, 확률)을 함께 주는 함수를 만든다."""
	m = SetManagerActor(autoregressive=ar).to(dev)
	m.load_state_dict(torch.load(os.path.join(R, ck), map_location=dev))
	m.eval()

	def fn(e):
		o = {k: torch.as_tensor(v, device=dev).unsqueeze(0) for k, v in e.manager_set_obs().items()}
		with torch.no_grad():
			act, probs, _ = m(o, action_mask(e, dev).unsqueeze(0))
		return act[0].cpu().numpy(), probs[0].cpu().numpy()
	return fn


def policy(fn, mode):
	"""mode: plain(그대로) / count(규칙의 중계 대수만 맞춤) / who(규칙의 중계 드론 그대로)."""
	def g(e):
		a, p = fn(e)
		if mode == 'plain':
			return a
		r = chain_manager(e)
		K = e.n_cand + 1
		m = action_mask(e, torch.device('cpu')).numpy()
		if mode == 'who':
			for i in range(e.num_drones):
				if r[i] == K and m[i, K]:
					a[i] = K
			return a
		need = int(np.sum(r == K))
		cur = np.where(a == K)[0]
		if len(cur) >= need:
			return a
		cand = [i for i in range(e.num_drones) if a[i] != K and m[i, K]]
		cand.sort(key=lambda i: -p[i, K])
		for i in cand[:need - len(cur)]:
			a[i] = K
		return a
	return g


def run(env, mf, seed):
	"""한 인스턴스를 굴려 makespan과 막힘을 돌려준다."""
	env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=seed, num_drones=env.num_drones,
	                                            comm_range=env.comm_range, cc_pos="random")
	env.reset()
	env.set_goals(mf(env))
	done, t = False, 0
	while not done and t < env.max_steps:
		_o, _w, _r, done = env.step(straight_worker(env))
		t += 1
		if t % 20 == 0 or env.goal_invalid().any():
			env.set_goals(mf(env))
	s = env.episode_stats()
	return (s['makespan'] if s['makespan'] else None), s['blocked']


dev = torch.device('cpu')
torch.set_num_threads(2)
env = DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=10000,
                            deadlock_limit=10 ** 9, no_progress_limit=10 ** 9, num_drones=3, num_dests=30)
env.reset()
fn = load('weights/v5_26_09_23_18_SWa/best_manager.pth', dev, True)
pols = (('학습 그대로', policy(fn, 'plain')), ('대수만 맞춤', policy(fn, 'count')),
        ('중계 드론도 규칙', policy(fn, 'who')), ('규칙+탈출', chain_escape_manager(60, 40, 'shuffle')))
seeds = [(rep, s) for rep in range(3) for s in range(501, 541)]
res = {}
for nm, mf in pols:
	out = [run(env, mf, s) for _r, s in seeds]
	res[nm] = np.array([x[0] if x[0] else np.nan for x in out], dtype=float)
	v = res[nm][~np.isnan(res[nm])]
	print(f"{nm:<14} 완주 {len(v)}/{len(seeds)}  중앙 {np.median(v):>5.0f}  절단평균 {stats.trim_mean(v,.1):>5.0f}  "
	      f"최대 {v.max():>5.0f}  막힘 {np.mean([x[1] for x in out]):>6.0f}", flush=True)
base = res['학습 그대로']
for nm in list(res)[1:]:
	m = ~np.isnan(base) & ~np.isnan(res[nm])
	d = res[nm][m] - base[m]
	print(f"  {nm} 대 학습 그대로: 쌍별 중앙 {np.median(d):+.0f} 승 {int((d<0).sum())} 패 {int((d>0).sum())} p={stats.wilcoxon(d).pvalue:.4f}")
