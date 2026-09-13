"""v3(교착 수정본) 계층 정책과 탐욕을 여러 에피소드 연속으로 이어붙여 GIF로 저장한다.

두 정책을 같은 인스턴스에서 동시에 굴려 좌우로 붙이고, 에피소드가 끝나면
다음 인스턴스로 넘어가며 하나의 GIF에 이어 담는다.
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

from env.disaster_relay_env_v3_26_09_04_15 import DisasterRelayDroneEnv
from model.hier_net_v3_26_08_31_19 import ManagerActor, WorkerActor
from pipeline.eval_v3_26_09_04_15 import (action_mask, rule_manager, straight_worker)
from util.instance_generator_v2_26_08_25_14 import sample_instance

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
S, HUD = 480, 104
C = {"bg": (250, 250, 252), "cc": (30, 80, 200), "dest": (220, 60, 60),
     "deliver": (40, 170, 70), "return": (150, 90, 30), "unload": (245, 190, 40),
     "link_cc": (120, 170, 250), "link_relay": (230, 130, 40),
     "hud": (28, 30, 36), "fg": (232, 234, 238), "muted": (150, 155, 165)}


def hop_depth(pos, cc, R):
	"""각 드론의 제어 센터까지 홉 수 (1=직접 연결)."""
	n = len(pos)
	h = np.full(n, -1)
	front = [i for i in range(n) if np.linalg.norm(pos[i] - cc) <= R]
	h[front] = 1
	d = np.linalg.norm(pos[:, None, :] - pos[None, :, :], axis=2)
	k = 1
	while front:
		nxt = [i for i in range(n) if h[i] == -1 and any(d[i, j] <= R for j in front)]
		for i in nxt:
			h[i] = k + 1
		front, k = nxt, k + 1
	return h


def panel(surf, env, fonts, title, ep, n_ep, step, cum):
	"""한 정책의 상태를 패널 하나에 그린다."""
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
		pygame.draw.rect(surf, C["dest"], (x - 3, y - 3, 6, 6))
	pygame.draw.circle(surf, C["cc"], P(env.cc_pos), 9)

	for i in range(env.num_drones):
		p = P(env.drones_pos[i])
		col = C["deliver"] if env.drones_capacity[i] > 0 else C["return"]
		if env.drones_timer[i] > 0:
			pygame.draw.circle(surf, C["unload"], p, 12, 3)
		pygame.draw.circle(surf, col, p, 7)
		pygame.draw.circle(surf, (255, 255, 255), p, 7, 1)
		surf.blit(fs.render(f"h{h[i]}", True, (40, 40, 50)), (p[0] + 8, p[1] - 6))

	# 교착 중이면 화면 테두리를 붉게 칠해 눈에 띄게 한다
	stall = getattr(env, "stall_steps", 0)
	if stall >= 5:
		w = min(6, 2 + stall // 15)
		pygame.draw.rect(surf, (220, 60, 60), (0, 0, S, S), w)

	pygame.draw.rect(surf, C["hud"], (0, S, S, HUD))
	done = int(env.num_dests - env.dests_active.sum())
	reach = float(np.max(np.linalg.norm(env.drones_pos - env.cc_pos, axis=1)))
	surf.blit(fb.render(title, True, C["fg"]), (10, S + 6))
	surf.blit(f.render(f"episode {ep}/{n_ep}   step {step:4d}   delivered {done:2d}/50",
	                   True, C["fg"]), (10, S + 30))
	stall_txt = f"STALLED {stall:3d} steps" if stall >= 5 else f"moving"
	stall_col = (255, 120, 120) if stall >= 5 else (140, 200, 150)
	surf.blit(f.render(f"reloads {int(env.reloads.sum())}   hop>=2 {np.mean(h >= 2):.2f}   "
	                   f"reach {reach:4.0f}", True, C["muted"]), (10, S + 52))
	surf.blit(f.render(stall_txt, True, stall_col), (S - 150, S + 52))
	surf.blit(fb.render(f"cumulative over {ep} ep: {cum + done:3d} delivered",
	                    True, (255, 214, 102)), (10, S + 76))


def make_policies(args, env, dev):
	"""좌(탐욕)·우(제안) 정책 함수와 제목을 만든다."""
	left = (lambda e: straight_worker(e), rule_manager, "Greedy (rule + straight)")

	w = WorkerActor(env.worker_obs().shape[-1]).to(dev)
	w.load_state_dict(torch.load(os.path.join(ROOT, args.worker), map_location=dev))
	w.eval()
	m = ManagerActor(env.manager_obs().shape[-1], env.n_cand + 2).to(dev)
	m.load_state_dict(torch.load(os.path.join(ROOT, args.manager), map_location=dev))
	m.eval()

	def wf(e):
		with torch.no_grad():
			a, _ = w(torch.as_tensor(e.worker_obs(), dtype=torch.float32, device=dev),
			         deterministic=True)
		return a.cpu().numpy()

	def mf(e):
		with torch.no_grad():
			_a, probs, _ = m(torch.as_tensor(e.manager_obs(), dtype=torch.float32, device=dev),
			                 action_mask(e, dev))
		return probs.argmax(-1).cpu().numpy()

	return left, (wf, mf, "Proposed (v3 + deadlock fix)")


def main():
	"""여러 에피소드를 이어붙여 좌우 비교 GIF를 저장한다."""
	p = argparse.ArgumentParser()
	p.add_argument("--worker", default="weights/v3_26_09_04_15_m_s1/best_worker.pth")
	p.add_argument("--manager", default="weights/v3_26_09_04_15_m_s1/best_manager.pth")
	p.add_argument("--episodes", type=int, default=10)
	p.add_argument("--seed0", type=int, default=101)
	p.add_argument("--comm-range", type=float, default=300.0)
	p.add_argument("--hl-every", type=int, default=20)
	p.add_argument("--every", type=int, default=14, help="몇 스텝마다 프레임을 담을지")
	p.add_argument("--out", default="v3_stuckfix_10ep_26_09_07_17.gif")
	args = p.parse_args()

	pygame.init()
	pygame.display.set_mode((1, 1))
	fonts = (pygame.font.SysFont("DejaVu Sans", 13),
	         pygame.font.SysFont("DejaVu Sans", 10),
	         pygame.font.SysFont("DejaVu Sans", 15, bold=True))

	kw = dict(comm_range=args.comm_range, cluster_penalty=False)
	envs = [DisasterRelayDroneEnv(**kw), DisasterRelayDroneEnv(**kw)]
	envs[1].reset()
	left, right = make_policies(args, envs[1], torch.device("cpu"))
	pols = [left, right]

	surfs = [pygame.Surface((S, S + HUD)) for _ in envs]
	W, H = S * 2 + 8, S + HUD
	frames, cum = [], [0, 0]

	for ep in range(1, args.episodes + 1):
		for e in envs:
			e.cc_pos, e.dests_pos = sample_instance(e.num_dests, seed=args.seed0 + ep - 1)
			e.reset()
			e.set_goals(rule_manager(e))
		dones, step = [False, False], 0
		while step < 1000 and not all(dones):
			for i, (e, (wf, mf, _)) in enumerate(zip(envs, pols)):
				if dones[i]:
					continue
				_o, _wr, _tr, dones[i] = e.step(wf(e))
				if (step + 1) % args.hl_every == 0 or e.goal_invalid().any():
					e.set_goals(mf(e))
			step += 1
			if step % args.every == 0 or step == 1:
				for i, e in enumerate(envs):
					panel(surfs[i], e, fonts, pols[i][2], ep, args.episodes, step, cum[i])
				canvas = pygame.Surface((W, H))
				canvas.fill((16, 17, 20))
				canvas.blit(surfs[0], (0, 0))
				canvas.blit(surfs[1], (S + 8, 0))
				frames.append(Image.frombytes("RGB", (W, H),
				                              pygame.image.tostring(canvas, "RGB")))
		for i, e in enumerate(envs):
			cum[i] += int(e.num_dests - e.dests_active.sum())
		print(f"  ep {ep}: 탐욕 {cum[0]:3d} 누적 / 제안 {cum[1]:3d} 누적", flush=True)

	out = os.path.join(ROOT, "figures", args.out)
	os.makedirs(os.path.dirname(out), exist_ok=True)
	frames[0].save(out, save_all=True, append_images=frames[1:], duration=55,
	               loop=0, optimize=True)
	print(f"\n프레임 {len(frames)}개 · {os.path.getsize(out)/1e6:.1f} MB → {out}")
	print(f"{args.episodes}개 에피소드 누적: 탐욕 {cum[0]} / 제안 {cum[1]} "
	      f"({cum[1]-cum[0]:+d}건, {100*(cum[1]-cum[0])/max(1,cum[0]):+.1f}%)")
	pygame.quit()


if __name__ == "__main__":
	main()
