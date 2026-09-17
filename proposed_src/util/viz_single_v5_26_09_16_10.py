"""한 정책을 여러 에피소드 연속으로 굴려 GIF 하나로 저장한다 (v5, 집합 상위 지원).

기하 중계 규칙과 학습 정책을 각각 따로 담아 눈으로 비교하기 위한 것이다.
중계 슬롯은 보라 십자, 중계 중인 드론은 보라 원으로 표시한다.
"""

import argparse
import os
import sys

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import numpy as np
import pygame
import torch
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
from model.hier_net_v3_26_08_31_19 import ManagerActor, WorkerActor
from model.hier_net_v5_26_09_15_23 import SetManagerActor
from pipeline.common_v5_26_09_15_22 import action_mask, chain_manager, hybrid_manager, straight_worker
from util.instance_generator_v5_26_09_15_22 import sample_instance
from util.viz_v4_26_09_14_03 import HUD, S, panel

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def learned(args, env, dev):
	"""학습된 상위·하위 정책을 함수 한 쌍으로 만든다."""
	if args.arch == "set":
		m = SetManagerActor().to(dev)
	else:
		m = ManagerActor(env.manager_obs().shape[-1], env.n_cand + 2).to(dev)
	m.load_state_dict(torch.load(os.path.join(ROOT, args.manager), map_location=dev))
	m.eval()
	w = None
	if not args.straight:
		w = WorkerActor(env.worker_obs().shape[-1]).to(dev)
		w.load_state_dict(torch.load(os.path.join(ROOT, args.worker), map_location=dev))
		w.eval()

	def wf(e):
		if w is None:
			return straight_worker(e)
		with torch.no_grad():
			a, _ = w(torch.as_tensor(e.worker_obs(), dtype=torch.float32, device=dev),
			         deterministic=False)
		return a.cpu().numpy()

	def mf(e):
		with torch.no_grad():
			if args.arch == "set":
				o = {k: torch.as_tensor(v, device=dev).unsqueeze(0) for k, v in e.manager_set_obs().items()}
				act, _p, _ = m(o, action_mask(e, dev).unsqueeze(0))
				return act[0].cpu().numpy()
			act, _p, _ = m(torch.as_tensor(e.manager_obs(), dtype=torch.float32, device=dev),
			               action_mask(e, dev))
		return act.cpu().numpy()

	return wf, mf


def main():
	"""지정한 정책으로 에피소드를 이어 붙여 GIF를 만든다."""
	p = argparse.ArgumentParser()
	p.add_argument("--policy", choices=["rule", "learned", "hybrid"], default="rule")
	p.add_argument("--arch", choices=["mlp", "set"], default="set")
	p.add_argument("--num-drones", type=int, default=4)
	p.add_argument("--num-dests", type=int, default=50)
	p.add_argument("--random-cc", action="store_true")
	p.add_argument("--worker", default="weights/v4_26_09_14_03_w/best_worker.pth")
	p.add_argument("--manager", default="weights/v5_26_09_15_23_s2b/best_manager.pth")
	p.add_argument("--straight", action="store_true", help="하위를 직진 제어로 고정")
	p.add_argument("--episodes", type=int, default=10)
	p.add_argument("--seed0", type=int, default=101)
	p.add_argument("--comm-range", type=float, default=300.0)
	p.add_argument("--hl-every", type=int, default=20)
	p.add_argument("--max-steps", type=int, default=6000)
	p.add_argument("--every", type=int, default=25, help="몇 스텝마다 프레임을 담을지")
	p.add_argument("--title", default="")
	p.add_argument("--out", default="v4_rule_10ep.gif")
	args = p.parse_args()

	pygame.init()
	pygame.display.set_mode((1, 1))
	fonts = (pygame.font.SysFont("DejaVu Sans", 13),
	         pygame.font.SysFont("DejaVu Sans", 10),
	         pygame.font.SysFont("DejaVu Sans", 15, bold=True))

	env = DisasterRelayDroneEnv(map_path="", comm_range=args.comm_range, cluster_penalty=False,
	                            max_steps=args.max_steps, deadlock_limit=10 ** 9,
	                            no_progress_limit=10 ** 9, num_drones=args.num_drones,
	                            num_dests=args.num_dests)
	env.reset()
	dev = torch.device("cpu")
	if args.policy == "rule":
		wf, mf = straight_worker, chain_manager
	elif args.policy == "hybrid":
		_wf, lf = learned(args, env, dev)
		wf, mf = straight_worker, hybrid_manager(lf, 60, 100, 0, 300)
	else:
		wf, mf = learned(args, env, dev)
	title = args.title or {"rule": "Geometric relay rule (no learning)", "learned": "Learned hierarchical policy",
	                       "hybrid": "Hybrid: rule + learned deadlock escape"}[args.policy]

	surf = pygame.Surface((S, S + HUD))
	frames, cum, comp = [], 0, 0
	for ep in range(1, args.episodes + 1):
		env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=args.seed0 + ep - 1, num_drones=env.num_drones,
		                                            comm_range=env.comm_range, cc_pos="random" if args.random_cc else None)
		env.reset()
		env.set_goals(mf(env))
		done, step = False, 0
		while not done and step < args.max_steps:
			_o, _wr, _tr, done = env.step(wf(env))
			step += 1
			if step % args.hl_every == 0 or env.goal_invalid().any():
				env.set_goals(mf(env))
			if step % args.every == 0 or step == 1:
				panel(surf, env, fonts, title, ep, args.episodes, step, cum)
				frames.append(Image.frombytes("RGB", (S, S + HUD),
				                              pygame.image.tostring(surf, "RGB")))
		got = int(env.num_dests - env.dests_active.sum())
		cum += got
		if got == env.num_dests:
			comp += 1
		print(f"  ep {ep}: {got}/{env.num_dests} in {step} steps{' (완주)' if got == 50 else ''}", flush=True)

	out = os.path.join(ROOT, "figures", args.out)
	os.makedirs(os.path.dirname(out), exist_ok=True)
	frames[0].save(out, save_all=True, append_images=frames[1:], duration=60,
	               loop=0, optimize=True)
	print(f"\n프레임 {len(frames)}개 · {os.path.getsize(out) / 1e6:.1f} MB -> {out}")
	print(f"{args.episodes}개 누적: {cum}건 배송, 완주 {comp}/{args.episodes}")
	pygame.quit()


if __name__ == "__main__":
	main()
