"""상위 정책의 결정 1회 지연시간을 드론 수별로 측정한다 (자기회귀 대 간소 대 규칙).

자기회귀 상위는 드론마다 신경망을 한 번씩 순차 호출하므로 비용이 드론 수에 비례한다.
실제 에피소드를 굴리며 상위 호출 구간만 시간을 재어, 실시간 추론 주장의 근거를 만든다.
"""

import argparse
import os
import sys
import time

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
from model.hier_net_v5_26_09_15_23 import SetManagerActor
from pipeline.common_v5_26_09_15_22 import action_mask, chain_escape_manager, straight_worker
from util.instance_generator_v5_26_09_15_22 import sample_instance

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def learned(ckpt, dev, ar):
	"""집합 상위 가중치를 결정 함수로 감싼다 (표본 추출, 평가 규약과 동일)."""
	m = SetManagerActor(autoregressive=ar).to(dev)
	m.load_state_dict(torch.load(os.path.join(ROOT, ckpt), map_location=dev))
	m.eval()

	def fn(e):
		o = {k: torch.as_tensor(v, device=dev).unsqueeze(0) for k, v in e.manager_set_obs().items()}
		with torch.no_grad():
			act, _p, _ = m(o, action_mask(e, dev).unsqueeze(0))
		return act[0].cpu().numpy()
	return fn


def measure(fn, n, dests, seed, hl_every, calls, dev):
	"""에피소드를 굴리며 상위 결정 구간만 누적 시간을 재고 밀리초 평균을 돌려준다."""
	env = DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=10 ** 6,
	                            deadlock_limit=10 ** 9, no_progress_limit=10 ** 9,
	                            num_drones=n, num_dests=dests)
	env.reset()
	env.cc_pos, env.dests_pos = sample_instance(dests, seed=seed, num_drones=n, comm_range=env.comm_range)
	env.reset()
	env.set_goals(fn(env))
	t, k, step = 0.0, 0, 0
	while k < calls:
		_o, _wr, _tr, done = env.step(straight_worker(env))
		step += 1
		if step % hl_every == 0 or env.goal_invalid().any():
			t0 = time.perf_counter()
			a = fn(env)
			if dev.type == "cuda":
				torch.cuda.synchronize()
			t += time.perf_counter() - t0
			env.set_goals(a)
			k += 1
		if done:
			env.reset()
			env.set_goals(fn(env))
			step = 0
	return t / k * 1000.0


def main():
	"""드론 수를 늘려가며 세 상위 정책의 결정 지연시간을 표로 출력한다."""
	p = argparse.ArgumentParser()
	p.add_argument("--ar", default="weights/v5_26_09_23_18_SWa/best_manager.pth")
	p.add_argument("--plain", default="weights/v5_26_09_26_17_L2a/best_manager.pth")
	p.add_argument("--drones", type=int, nargs="*", default=[3, 4, 5, 6, 8])
	p.add_argument("--num-dests", type=int, default=50)
	p.add_argument("--calls", type=int, default=120)
	p.add_argument("--hl-every", type=int, default=20)
	p.add_argument("--seed", type=int, default=901)
	p.add_argument("--device", default="cpu")
	args = p.parse_args()

	torch.set_num_threads(2)
	dev = torch.device(args.device)
	pol = (("기하 규칙 + 탈출", lambda: chain_escape_manager(60, 40, "shuffle"), None),
	       ("간소 (자기회귀 없음)", lambda: learned(args.plain, dev, False), args.plain),
	       ("자기회귀", lambda: learned(args.ar, dev, True), args.ar))
	print(f"장치 {args.device} · 상위 호출 {args.calls}회 평균 · 목적지 {args.num_dests} · hl_every {args.hl_every}")
	print(f"{'드론':<6}" + "".join(f"{nm:>22}" for nm, _f, _c in pol))
	rows = []
	for n in args.drones:
		ms = [measure(f(), n, args.num_dests, args.seed, args.hl_every, args.calls, dev) for _nm, f, _c in pol]
		rows.append((n, ms))
		print(f"{n:<6}" + "".join(f"{v:>21.2f}ms" for v in ms))
	print("\n드론당 증가분 (선형 회귀 기울기, ms/대)")
	x = np.array([r[0] for r in rows], dtype=float)
	for j, (nm, _f, _c) in enumerate(pol):
		y = np.array([r[1][j] for r in rows])
		s = np.polyfit(x, y, 1)[0]
		print(f"  {nm:<22}{s:+.2f}")
	print("\n한 에피소드(900스텝, 상위 45회) 누적 상위 시간")
	for j, (nm, _f, _c) in enumerate(pol):
		y = [r[1][j] for r in rows if r[0] == 4]
		if y:
			print(f"  {nm:<22}{y[0] * 45 / 1000:.3f}s (드론 4)")


if __name__ == "__main__":
	main()
