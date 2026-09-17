"""같은 인스턴스에서 기하 규칙(좌)과 제안 혼합 상위(우)를 나란히 굴려 여러 에피소드를 GIF 하나에 담는다.

혼합 상위 = 기하 규칙 + 학습 정책(집합 상위, s3b) 교착 탈출·영구 이관. 드론 수·목적지 수·CC 무작위를 인자로 받는다.
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

from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv, GOAL_RELAY
from model.hier_net_v5_26_09_15_23 import SetManagerActor
from pipeline.common_v5_26_09_15_22 import action_mask, chain_manager, hybrid_manager, straight_worker
from util.instance_generator_v5_26_09_15_22 import sample_instance
from util.viz_v4_26_09_14_03 import C, HUD, S, hop_depth

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def panel(surf, env, fonts, title, ep, n_ep, step, cum, comp):
	"""한 정책의 상태를 패널 하나에 그린다 (목적지 수 가변)."""
	f, fs, fb = fonts
	sc = S / env.map_size[0]
	P = lambda p: (int(p[0] * sc), int(p[1] * sc))
	surf.fill(C["bg"])
	R = env.comm_range
	h = hop_depth(env.drones_pos, env.cc_pos, R)
	veil = pygame.Surface((S, S), pygame.SRCALPHA)
	pygame.draw.circle(veil, (30, 80, 200, 30), P(env.cc_pos), int(R * sc))
	for i in range(env.num_drones):
		pygame.draw.circle(veil, (40, 170, 70, 18), P(env.drones_pos[i]), int(R * sc))
	surf.blit(veil, (0, 0))
	pygame.draw.circle(surf, (60, 110, 220), P(env.cc_pos), int(R * sc), 1)
	for i in range(env.num_drones):
		if np.linalg.norm(env.drones_pos[i] - env.cc_pos) <= R:
			pygame.draw.line(surf, C["link_cc"], P(env.cc_pos), P(env.drones_pos[i]), 2)
		for j in range(i + 1, env.num_drones):
			if np.linalg.norm(env.drones_pos[i] - env.drones_pos[j]) <= R:
				relay = h[i] >= 2 or h[j] >= 2
				pygame.draw.line(surf, C["link_relay"] if relay else (205, 210, 216),
				                 P(env.drones_pos[i]), P(env.drones_pos[j]), 3 if relay else 1)
	for k in np.flatnonzero(env.dests_active):
		x, y = P(env.dests_pos[k])
		pygame.draw.rect(surf, C["dest"], (x - 2, y - 2, 5, 5))
	pygame.draw.circle(surf, C["cc"], P(env.cc_pos), 9)
	for i in range(env.num_drones):
		if env.goal_kind[i] == GOAL_RELAY:
			gx, gy = P(env.goal_pos[i])
			pygame.draw.line(surf, C["relay"], (gx - 6, gy), (gx + 6, gy), 2)
			pygame.draw.line(surf, C["relay"], (gx, gy - 6), (gx, gy + 6), 2)
			pygame.draw.line(surf, C["relay"], P(env.drones_pos[i]), (gx, gy), 1)
	for i in range(env.num_drones):
		p = P(env.drones_pos[i])
		col = (C["relay"] if env.goal_kind[i] == GOAL_RELAY
		       else C["deliver"] if env.drones_capacity[i] > 0 else C["return"])
		if env.drones_timer[i] > 0:
			pygame.draw.circle(surf, C["unload"], p, 12, 3)
		pygame.draw.circle(surf, col, p, 7)
		pygame.draw.circle(surf, (255, 255, 255), p, 7, 1)
		surf.blit(fs.render(f"h{h[i]}", True, (40, 40, 50)), (p[0] + 8, p[1] - 6))
	if env.deadlock_run >= 5:
		pygame.draw.rect(surf, (220, 60, 60), (0, 0, S, S), min(6, 2 + env.deadlock_run // 15))
	pygame.draw.rect(surf, C["hud"], (0, S, S, HUD))
	done = int(env.num_dests - env.dests_active.sum())
	reach = float(np.max(np.linalg.norm(env.drones_pos - env.cc_pos, axis=1)))
	surf.blit(fb.render(title, True, C["fg"]), (10, S + 6))
	surf.blit(f.render(f"episode {ep}/{n_ep}   step {step:4d}   delivered {done:3d}/{env.num_dests}",
	                   True, C["fg"]), (10, S + 30))
	stall_txt = f"STALLED {env.deadlock_run:3d}" if env.deadlock_run >= 5 else "moving"
	surf.blit(f.render(f"reloads {int(env.reloads.sum())}  hop>=2 {np.mean(h >= 2):.2f}  reach {reach:4.0f}",
	                   True, C["muted"]), (10, S + 52))
	surf.blit(f.render(stall_txt, True, (255, 120, 120) if env.deadlock_run >= 5 else (140, 200, 150)), (S - 120, S + 52))
	surf.blit(fb.render(f"completed {comp}/{ep - 1 if not done == env.num_dests else ep}   cumulative {cum + done}",
	                    True, (255, 214, 102)), (10, S + 76))


def main():
	"""좌: 기하 규칙, 우: 혼합 상위. 여러 에피소드를 이어 붙인 GIF."""
	p = argparse.ArgumentParser()
	p.add_argument("--manager", default="weights/v5_26_09_15_23_s3b/best_manager.pth")
	p.add_argument("--num-drones", type=int, default=4)
	p.add_argument("--num-dests", type=int, default=50)
	p.add_argument("--random-cc", action="store_true")
	p.add_argument("--episodes", type=int, default=10)
	p.add_argument("--seed0", type=int, default=101)
	p.add_argument("--comm-range", type=float, default=300.0)
	p.add_argument("--hl-every", type=int, default=20)
	p.add_argument("--max-steps", type=int, default=6000)
	p.add_argument("--every", type=int, default=30)
	p.add_argument("--hybrid", type=int, nargs=4, default=[60, 100, 1, 200])
	p.add_argument("--out", default="v5_pair.gif")
	a = p.parse_args()

	pygame.init()
	pygame.display.set_mode((1, 1))
	fonts = (pygame.font.SysFont("DejaVu Sans", 13), pygame.font.SysFont("DejaVu Sans", 10),
	         pygame.font.SysFont("DejaVu Sans", 15, bold=True))
	dev = torch.device("cpu")
	net = SetManagerActor().to(dev)
	net.load_state_dict(torch.load(os.path.join(ROOT, a.manager), map_location=dev))
	net.eval()

	def learned(e):
		o = {k: torch.as_tensor(v, device=dev).unsqueeze(0) for k, v in e.manager_set_obs().items()}
		with torch.no_grad():
			act, _p, _ = net(o, action_mask(e, dev).unsqueeze(0))
		return act[0].cpu().numpy()

	mk = lambda: DisasterRelayDroneEnv(map_path="", comm_range=a.comm_range, cluster_penalty=False,
	                                   max_steps=a.max_steps, deadlock_limit=10 ** 9, no_progress_limit=10 ** 9,
	                                   num_drones=a.num_drones, num_dests=a.num_dests)
	envs = [mk(), mk()]
	titles = ["Geometric relay rule (no learning)", "Proposed: hybrid (rule + learned escape)"]
	surfs = [pygame.Surface((S, S + HUD)) for _ in envs]
	W, H = S * 2 + 8, S + HUD
	frames, cum, comp = [], [0, 0], [0, 0]
	for ep in range(1, a.episodes + 1):
		cc, d = sample_instance(a.num_dests, seed=a.seed0 + ep - 1, num_drones=a.num_drones,
		                        comm_range=a.comm_range, cc_pos="random" if a.random_cc else None)
		pols = [chain_manager, hybrid_manager(learned, *a.hybrid)]
		for e, mf in zip(envs, pols):
			e.cc_pos, e.dests_pos = cc.copy(), d.copy()
			e.reset()
			e.set_goals(mf(e))
		dones, step = [False, False], 0
		while step < a.max_steps and not all(dones):
			for i, (e, mf) in enumerate(zip(envs, pols)):
				if dones[i]:
					continue
				_o, _wr, _tr, dones[i] = e.step(straight_worker(e))
				if (step + 1) % a.hl_every == 0 or e.goal_invalid().any():
					e.set_goals(mf(e))
			step += 1
			if step % a.every == 0 or step == 1:
				for i, e in enumerate(envs):
					panel(surfs[i], e, fonts, titles[i], ep, a.episodes, step, cum[i], comp[i])
				canvas = pygame.Surface((W, H))
				canvas.fill((16, 17, 20))
				canvas.blit(surfs[0], (0, 0))
				canvas.blit(surfs[1], (S + 8, 0))
				frames.append(Image.frombytes("RGB", (W, H), pygame.image.tostring(canvas, "RGB")))
		res = []
		for i, e in enumerate(envs):
			got = int(e.num_dests - e.dests_active.sum())
			cum[i] += got
			if got == e.num_dests:
				comp[i] += 1
			res.append(f"{got}/{e.num_dests}@{e.current_step}")
		print(f"  ep {ep}: 규칙 {res[0]}  혼합 {res[1]}", flush=True)
	out = os.path.join(ROOT, "figures", a.out)
	frames[0].save(out, save_all=True, append_images=frames[1:], duration=60, loop=0, optimize=True)
	print(f"\n프레임 {len(frames)}개 · {os.path.getsize(out) / 1e6:.1f} MB -> {out}")
	print(f"완주 규칙 {comp[0]}/{a.episodes}  혼합 {comp[1]}/{a.episodes} | 누적 배송 규칙 {cum[0]} 혼합 {cum[1]}")
	pygame.quit()


if __name__ == "__main__":
	main()
