"""학습된 정책의 배송 기동을 pygame으로 연속 재생한다.

환경의 기본 render 대신 자체 렌더러를 써서 홉 깊이, 중계 링크, 드론별 배송량을 함께 표시한다.
CLAUDE.md 6절의 시각 검증(통신권 이탈 방지, 릴레이 형성, 병목)에 쓰기 위한 도구다.
"""

import argparse
import os
import sys

import numpy as np
import pygame
import torch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v1_26_08_24_22 import DisasterRelayDroneEnv
from model.sac_net_v1_26_08_24_22 import SACActor
from util.instance_generator_v1_26_08_24_22 import sample_instance

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
W, H, HUD = 1000, 1000, 132
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


def draw(scr, fonts, env, hops, ep, n_eps, step, label):
	"""한 프레임을 그린다 — 지도, 통신 링크, 드론 상태, 하단 HUD."""
	f, fs, fb = fonts
	scr.fill(C["bg"])
	R = env.comm_range

	# 통신 반경: 제어 센터와 각 드론
	veil = pygame.Surface((W, H), pygame.SRCALPHA)
	pygame.draw.circle(veil, (30, 80, 200, 26), env.cc_pos.astype(int), int(R))
	for i in range(env.num_drones):
		pygame.draw.circle(veil, (40, 170, 70, 16), env.drones_pos[i].astype(int), int(R))
	scr.blit(veil, (0, 0))
	pygame.draw.circle(scr, (60, 110, 220), env.cc_pos.astype(int), int(R), 2)

	# 링크: 제어 센터 직결은 파랑, 드론 간 중계는 주황(굵게)
	for i in range(env.num_drones):
		if np.linalg.norm(env.drones_pos[i] - env.cc_pos) <= R:
			pygame.draw.line(scr, C["link_cc"], env.cc_pos, env.drones_pos[i], 2)
		for j in range(i + 1, env.num_drones):
			if np.linalg.norm(env.drones_pos[i] - env.drones_pos[j]) <= R:
				relay = hops[i] >= 2 or hops[j] >= 2
				pygame.draw.line(scr, C["link_relay"] if relay else (200, 205, 212),
				                 env.drones_pos[i], env.drones_pos[j], 4 if relay else 2)

	# 목적지 (미배송만)
	for k in range(env.num_dests):
		if env.dests_active[k]:
			x, y = env.dests_pos[k]
			pygame.draw.rect(scr, C["dest"], (x - 5, y - 5, 10, 10))

	# 제어 센터
	pygame.draw.circle(scr, C["cc"], env.cc_pos.astype(int), 17)
	scr.blit(fs.render("CC", True, (255, 255, 255)),
	         fs.render("CC", True, (255, 255, 255)).get_rect(center=env.cc_pos.astype(int)))

	# 드론
	for i in range(env.num_drones):
		p = env.drones_pos[i].astype(int)
		col = C["deliver"] if env.drones_capacity[i] > 0 else C["return"]
		if env.drones_timer[i] > 0:
			pygame.draw.circle(scr, C["unload"], p, 20, 4)
		pygame.draw.circle(scr, col, p, 13)
		pygame.draw.circle(scr, (255, 255, 255), p, 13, 2)
		t = f.render(str(int(env.drones_capacity[i])), True, (255, 255, 255))
		scr.blit(t, t.get_rect(center=p))
		hp = fs.render(f"D{i} h{hops[i]}", True, (40, 40, 50))
		scr.blit(hp, (p[0] - 18, p[1] + 16))

	# HUD
	pygame.draw.rect(scr, C["hud_bg"], (0, H, W, HUD))
	done = int(env.num_dests - env.dests_active.sum())
	reach = float(np.max(np.linalg.norm(env.drones_pos - env.cc_pos, axis=1)))
	h2 = float(np.mean(hops >= 2))
	scr.blit(fb.render(f"{label}   |   Episode {ep}/{n_eps}   step {step:4d}", True, C["hud_fg"]),
	         (18, H + 12))
	scr.blit(f.render(
		f"delivered {done:2d}/50     reloads {int(env.reloads.sum())}     "
		f"blocked {int(env.blocked.sum()):4d}     comm-loss {int(env.comm_loss)}",
		True, C["hud_fg"]), (18, H + 46))
	scr.blit(f.render(
		f"per-drone deliveries {env.deliveries.tolist()}     "
		f"hop>=2 now {h2:.2f}     max reach {reach:4.0f} (direct range {int(R)})",
		True, C["muted"]), (18, H + 74))
	scr.blit(fs.render(
		"green = carrying   brown = returning   gold ring = loading/unloading   "
		"orange link = relay hop   blue link = direct to CC", True, C["muted"]), (18, H + 102))
	pygame.display.flip()


def main():
	"""체크포인트를 불러와 여러 에피소드를 연속 재생한다."""
	p = argparse.ArgumentParser()
	p.add_argument("--run", default="v1_26_08_24_22")
	p.add_argument("--ckpt", default="sac_actor_ep600.pth")
	p.add_argument("--episodes", type=int, default=5)
	p.add_argument("--seed0", type=int, default=101, help="홀드아웃 시드 시작값")
	p.add_argument("--fps", type=int, default=120)
	p.add_argument("--shots", default="", help="주면 해당 접미로 에피소드별 스냅샷 저장")
	args = p.parse_args()

	ckpt = os.path.join(ROOT, "weights", args.run, args.ckpt)
	env = DisasterRelayDroneEnv()
	actor = SACActor(env.reset().shape[-1], 2)
	actor.load_state_dict(torch.load(ckpt, map_location="cpu"))
	actor.eval()

	pygame.init()
	pygame.display.set_caption(f"Disaster Relay Drone — {args.run} / {args.ckpt}")
	scr = pygame.display.set_mode((W, H + HUD))
	clock = pygame.time.Clock()
	fonts = (pygame.font.SysFont("DejaVu Sans", 17, bold=True),
	         pygame.font.SysFont("DejaVu Sans", 13),
	         pygame.font.SysFont("DejaVu Sans", 19, bold=True))
	label = f"{args.run} · {args.ckpt[:-4]}"

	summary = []
	for ep in range(1, args.episodes + 1):
		env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=args.seed0 + ep - 1)
		obs = env.reset()
		done, step = False, 0
		# 홉 통계는 여기서 집계한다 — Run A 학습 환경에는 이 지표가 없다
		h2_sum, reach = 0, 0.0
		while not done and step < 1000:
			for e in pygame.event.get():
				if e.type == pygame.QUIT or (e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE):
					pygame.quit()
					return
			with torch.no_grad():
				a, _ = actor(torch.FloatTensor(obs), deterministic=True)
			a = a.numpy()
			obs, _s, _r, done = env.step(
				np.stack([(a[:, 0] + 1) * np.pi, (a[:, 1] + 1) * 0.5], axis=1))
			step += 1
			hops = hop_depth(env.drones_pos, env.cc_pos, env.comm_range)
			h2_sum += int(np.sum(hops >= 2))
			reach = max(reach, float(np.max(np.linalg.norm(env.drones_pos - env.cc_pos, axis=1))))
			draw(scr, fonts, env, hops, ep, args.episodes, step, label)
			clock.tick(args.fps)

		st = env.episode_stats()
		st["hop2plus"] = h2_sum / max(1, step * env.num_drones)
		st["max_reach"] = reach
		summary.append(st)
		print(f"Ep {ep}: 배송 {st['delivered']:2d}/50 | 미활동드론 {st['idle_drones']} | "
		      f"드론별 {env.deliveries.tolist()} | 2홉+ {st['hop2plus']:.2f} | "
		      f"도달 {st['max_reach']:.0f} | 단절 {st['comm_loss']}", flush=True)
		if args.shots:
			out = os.path.join(ROOT, "figures", f"sim_ep{ep}_{args.shots}.png")
			pygame.image.save(scr, out)
			print(f"  스냅샷 저장: {out}", flush=True)
		pygame.time.wait(700)

	d = [s["delivered"] for s in summary]
	print(f"\n{args.episodes}개 에피소드 평균 배송 {np.mean(d):.1f}/50 "
	      f"(개별 {d}) | 통신 단절 {sum(s['comm_loss'] for s in summary)}건")
	pygame.time.wait(1500)
	pygame.quit()


if __name__ == "__main__":
	main()
