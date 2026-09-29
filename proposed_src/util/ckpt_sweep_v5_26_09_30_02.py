"""저장된 체크포인트들을 3/30에서 재어, 선택이 놓친 더 좋은 지점이 있었는지 본다.

시드는 601-640으로 학습 홀드아웃(801-840)과도 평가(501-540)와도 겹치지 않는다.
선택이 문제인지 정책 자체가 문제인지 가르기 위한 진단이다.
"""

import argparse
import os
import sys

import numpy as np
import torch
from scipy import stats

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
from model.hier_net_v5_26_09_15_23 import SetManagerActor
from pipeline.common_v5_26_09_15_22 import action_mask, chain_escape_manager, straight_worker
from util.instance_generator_v5_26_09_15_22 import sample_instance

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def load(ck, dev, ar=True):
	"""체크포인트를 표본 추출 결정 함수로 감싼다."""
	m = SetManagerActor(autoregressive=ar).to(dev)
	sd = torch.load(os.path.join(ROOT, ck), map_location=dev)
	m.load_state_dict(sd["actor"] if isinstance(sd, dict) and "actor" in sd else sd)
	m.eval()

	def fn(e):
		o = {k: torch.as_tensor(v, device=dev).unsqueeze(0) for k, v in e.manager_set_obs().items()}
		with torch.no_grad():
			act, _p, _ = m(o, action_mask(e, dev).unsqueeze(0))
		return act[0].cpu().numpy()
	return fn


def run(env, mf, seed):
	"""한 인스턴스를 굴려 makespan(미완주면 None)을 돌려준다."""
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
	return s["makespan"] if s["makespan"] else None


def main():
	"""실행별로 여러 체크포인트를 같은 인스턴스에서 재고 표로 출력한다."""
	p = argparse.ArgumentParser()
	p.add_argument("--tags", nargs="+", required=True)
	p.add_argument("--eps", type=int, nargs="+", default=[400, 600, 800, 1000, 1200])
	p.add_argument("--n", type=int, default=40)
	p.add_argument("--reps", type=int, default=2)
	p.add_argument("--seed0", type=int, default=601)
	args = p.parse_args()

	torch.set_num_threads(2)
	dev = torch.device("cpu")
	env = DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=10000,
	                            deadlock_limit=10 ** 9, no_progress_limit=10 ** 9, num_drones=3, num_dests=30)
	env.reset()
	seeds = [s for _r in range(args.reps) for s in range(args.seed0, args.seed0 + args.n)]
	rl = [run(env, chain_escape_manager(60, 40, "shuffle"), s) for s in seeds]
	v = np.array([x for x in rl if x], dtype=float)
	print(f"규칙+탈출: 완주 {len(v)}/{len(seeds)} 중앙 {np.median(v):.0f} 절단평균 {stats.trim_mean(v,.1):.0f}", flush=True)
	for t in args.tags:
		print(f"\n[{t}]", flush=True)
		for e in args.eps + ["best"]:
			ck = (f"weights/{t}/best_manager.pth" if e == "best" else f"weights/{t}/manager_ep{e}.pth")
			if not os.path.exists(os.path.join(ROOT, ck)):
				continue
			out = [run(env, load(ck, dev), s) for s in seeds]
			w = np.array([x for x in out if x], dtype=float)
			print(f"  {str(e):>6}: 완주 {len(w)}/{len(seeds)} 중앙 {np.median(w):>5.0f} "
			      f"절단평균 {stats.trim_mean(w,.1):>5.0f} 최대 {w.max():>5.0f}", flush=True)


if __name__ == "__main__":
	main()
