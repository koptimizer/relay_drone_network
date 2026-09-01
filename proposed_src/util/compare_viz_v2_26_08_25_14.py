"""탐욕 베이스라인과 제안 정책을 같은 인스턴스에서 나란히 굴려 GIF로 저장한다.

두 환경을 동일 시드로 동시에 진행시키며 매 스텝을 좌우로 붙여 렌더링한다.
디스플레이 없이도 동작하도록 pygame을 dummy 드라이버로 띄운다.
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

from env.disaster_relay_env_v2_26_08_25_14 import DisasterRelayDroneEnv
from model.sac_net_v2_26_08_25_14 import SACActor
from pipeline.baseline_v2_26_08_25_14 import greedy
from util.instance_generator_v2_26_08_25_14 import sample_instance

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
S = 520           # 한 패널의 화면 크기 (지도 1000을 축소)
HUD = 96
C = {
	"bg": (250, 250, 252), "cc": (30, 80, 200), "dest": (220, 60, 60),
	"deliver": (40, 170, 70), "return": (150, 90, 30), "unload": (245, 190, 40),
	"link_cc": (120, 170, 250), "link_relay": (230, 130, 40),
	"hud_bg": (28, 30, 36), "hud_fg": (232, 234, 238), "muted": (150, 155, 165),
}


def hop_depth(pos, cc, R):
	"""각 드론의 제어 센터까지 홉 수 (1=직접 연결)."""
	n = len(pos)
	h = np.full(n, -1)
	frontier = [i for i in range(n) if np.linalg.norm(pos[i] - cc) <= R]
	for i in frontier:
		h[i] = 1
	d = np.linalg.norm(pos[:, None, :] - pos[None, :, :], axis=2)
	k = 1
	while frontier:
		nxt = [i for i in range(n) if h[i] == -1 and any(d[i, j] <= R for j in frontier)]
		for i in nxt:
			h[i] = k + 1
		frontier = nxt
		k += 1
	return h


def panel(surf, env, fonts, title, step):
	"""한 정책의 상태를 패널 하나에 그린다."""
	f, fs, fb = fonts
	sc = S / env.map_size[0]
	P = lambda p: (int(p[0] * sc), int(p[1] * sc))
	surf.fill(C["bg"])
	R = env.comm_range
	hops = hop_depth(env.drones_pos, env.cc_pos, R)

	veil = pygame.Surface((S, S), pygame.SRCALPHA)
	pygame.draw.circle(veil, (30, 80, 200, 28), P(env.cc_pos), int(R * sc))
	for i in range(env.num_drones):
		pygame.draw.circle(veil, (40, 170, 70, 18), P(env.drones_pos[i]), int(R * sc))
	surf.blit(veil, (0, 0))
	pygame.draw.circle(surf, (60, 110, 220), P(env.cc_pos), int(R * sc), 1)

	for i in range(env.num_drones):
		if np.linalg.norm(env.drones_pos[i] - env.cc_pos) <= R:
			pygame.draw.line(surf, C["link_cc"], P(env.cc_pos), P(env.drones_pos[i]), 2)
		for j in range(i + 1, env.num_drones):
			if np.linalg.norm(env.drones_pos[i] - env.drones_pos[j]) <= R:
				relay = hops[i] >= 2 or hops[j] >= 2
				pygame.draw.line(surf, C["link_relay"] if relay else (205, 210, 216),
				                 P(env.drones_pos[i]), P(env.drones_pos[j]), 3 if relay else 1)

	for k in range(env.num_dests):
		if env.dests_active[k]:
			x, y = P(env.dests_pos[k])
			pygame.draw.rect(surf, C["dest"], (x - 3, y - 3, 6, 6))

	pygame.draw.circle(surf, C["cc"], P(env.cc_pos), 9)
	for i in range(env.num_drones):
		p = P(env.drones_pos[i])
		col = C["deliver"] if env.drones_capacity[i] > 0 else C["return"]
		if env.drones_timer[i] > 0:
			pygame.draw.circle(surf, C["unload"], p, 13, 3)
		pygame.draw.circle(surf, col, p, 8)
		pygame.draw.circle(surf, (255, 255, 255), p, 8, 1)
		surf.blit(fs.render(f"h{hops[i]}", True, (40, 40, 50)), (p[0] + 9, p[1] - 6))

	pygame.draw.rect(surf, C["hud_bg"], (0, S, S, HUD))
	done = int(env.num_dests - env.dests_active.sum())
	reach = float(np.max(np.linalg.norm(env.drones_pos - env.cc_pos, axis=1)))
	surf.blit(fb.render(title, True, C["hud_fg"]), (12, S + 8))
	surf.blit(f.render(f"delivered {done:2d}/50    reloads {int(env.reloads.sum())}    "
	                   f"blocked {int(env.blocked.sum()):4d}", True, C["hud_fg"]), (12, S + 34))
	surf.blit(f.render(f"per-drone {env.deliveries.tolist()}    hop>=2 {np.mean(hops >= 2):.2f}    "
	                   f"reach {reach:4.0f}", True, C["muted"]), (12, S + 58))
	surf.blit(fs.render(f"step {step}", True, C["muted"]), (12, S + 78))


def main():
	"""두 정책을 동일 인스턴스에서 굴리고 좌우 비교 GIF를 저장한다."""
	p = argparse.ArgumentParser()
	p.add_argument("--ckpt", default="weights/v2_26_08_25_14_c300_relay/best_actor.pth")
	p.add_argument("--mode", default="angle", choices=["angle", "vector"])
	p.add_argument("--label", default="Proposed (MARL)")
	p.add_argument("--n-relay", type=int, default=1, help="탐욕 쪽 중계 전용 드론 수")
	p.add_argument("--comm-range", type=float, default=300.0)
	p.add_argument("--seed", type=int, default=101)
	p.add_argument("--every", type=int, default=8, help="몇 스텝마다 프레임을 담을지")
	p.add_argument("--out", default="compare_greedy_vs_proposed_v2_26_08_25_14.gif")
	args = p.parse_args()

	pygame.init()
	pygame.display.set_mode((1, 1))
	fonts = (pygame.font.SysFont("DejaVu Sans", 14),
	         pygame.font.SysFont("DejaVu Sans", 11),
	         pygame.font.SysFont("DejaVu Sans", 16, bold=True))

	envs, policies, titles = [], [], []
	# 왼쪽: 탐욕
	e1 = DisasterRelayDroneEnv(comm_range=args.comm_range)
	envs.append(e1)
	policies.append(lambda e: greedy(e, args.n_relay))
	titles.append(f"Greedy + {args.n_relay} fixed relay")
	# 오른쪽: 제안 정책
	e2 = DisasterRelayDroneEnv(comm_range=args.comm_range, action_mode=args.mode)
	actor = SACActor(e2.reset().shape[-1], 2)
	actor.load_state_dict(torch.load(os.path.join(ROOT, args.ckpt), map_location="cpu"))
	actor.eval()

	def rl(env):
		with torch.no_grad():
			a, _ = actor(torch.FloatTensor(env._get_local_obs()), deterministic=True)
		a = a.numpy()
		if args.mode == "vector":
			return a
		return np.stack([(a[:, 0] + 1) * np.pi, (a[:, 1] + 1) * 0.5], axis=1)

	envs.append(e2)
	policies.append(rl)
	titles.append(args.label)

	for e in envs:
		e.cc_pos, e.dests_pos = sample_instance(e.num_dests, seed=args.seed)
		e.reset()

	surfs = [pygame.Surface((S, S + HUD)) for _ in envs]
	frames, dones, step = [], [False, False], 0
	while step < 1000 and not all(dones):
		for i, (e, pol) in enumerate(zip(envs, policies)):
			if not dones[i]:
				_o, _s, _r, dones[i] = e.step(pol(e))
		step += 1
		if step % args.every == 0 or step == 1:
			for i, e in enumerate(envs):
				panel(surfs[i], e, fonts, titles[i], step)
			w, h = S * 2 + 8, S + HUD
			canvas = pygame.Surface((w, h))
			canvas.fill((16, 17, 20))
			canvas.blit(surfs[0], (0, 0))
			canvas.blit(surfs[1], (S + 8, 0))
			frames.append(Image.frombytes(
				"RGB", (w, h), pygame.image.tostring(canvas, "RGB")))

	out = os.path.join(ROOT, "figures", args.out)
	os.makedirs(os.path.dirname(out), exist_ok=True)
	frames[0].save(out, save_all=True, append_images=frames[1:], duration=60, loop=0, optimize=True)
	size_mb = os.path.getsize(out) / 1e6
	print(f"프레임 {len(frames)}개 · {size_mb:.1f} MB → {out}", flush=True)

	for i, e in enumerate(envs):
		st = e.episode_stats()
		print(f"  {titles[i]:26s} 배송 {st['delivered']:2d}/50  재적재 {st['reloads']}  "
		      f"미활동 {st['idle_drones']}  드론별 {e.deliveries.tolist()}", flush=True)

	still = os.path.join(ROOT, "figures", args.out.replace(".gif", "_final.png"))
	pygame.image.save(canvas, still)
	print(f"최종 프레임: {still}")
	pygame.quit()


if __name__ == "__main__":
	main()
