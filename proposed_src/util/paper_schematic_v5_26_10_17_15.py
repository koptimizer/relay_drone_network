"""논문 개요 그림: (a) 문제 기하 — 제어 센터, 중계 사슬, 선두, 목적지; (b) 두 제어기의 결정 구조.

실제 인스턴스(시드 501, 드론 3, 목적지 30)의 좌표를 쓰고, 규칙의 배정을 한 번 적용해 그린다.
"""

import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyBboxPatch, FancyArrowPatch
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
from pipeline.common_v5_26_09_15_22 import chain_escape_manager, straight_worker
from util.instance_generator_v5_26_09_15_22 import sample_instance

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FIG = os.path.join(ROOT, "figures")
plt.rcParams.update({"font.size": 8, "pdf.fonttype": 42, "font.family": "DejaVu Sans"})
C_H, C_L, C_D, C_CC = "#b04236", "#1d6d9c", "#777777", "#222222"


def panel_geometry(ax):
	"""규칙+탈출을 220스텝 굴린 뒤의 상태: 선두 한 대, 중계 한 대, 복귀 한 대."""
	# 투영의 동점 처리가 무작위라 장면이 재현되도록 난수 시드를 고정하고, 조건을 만족하는 첫 시드를 쓴다
	for rs in range(40):
		np.random.seed(rs)
		env = DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=10000,
		                            deadlock_limit=10 ** 9, no_progress_limit=10 ** 9, num_drones=3, num_dests=30)
		env.cc_pos, env.dests_pos = sample_instance(30, seed=503, num_drones=3, comm_range=300.0, cc_pos=None)
		env.reset()
		mf = chain_escape_manager()
		env.set_goals(mf(env))
		hit = False
		# 배송·복귀·중계 세 역할이 동시에 보이는 첫 시점(150스텝 이후)에서 멈춘다
		for t in range(1, 1200):
			env.step(straight_worker(env))
			if t % 20 == 0 or env.goal_invalid().any():
				env.set_goals(mf(env))
			kinds = set(int(k) for k in env.goal_kind)
			if t >= 150 and {0, 1, 3} <= kinds:
				reach = np.linalg.norm(env.drones_pos - env.cc_pos, axis=1)
				pd = min(np.linalg.norm(env.drones_pos[a] - env.drones_pos[b]) for a in range(3) for b in range(a + 1, 3))
				# 선두가 1.5R 너머에 있고 드론끼리 120 m 이상 떨어진, 사슬이 보이는 장면만 고른다
				if reach.max() > 1.5 * env.comm_range and pd > 120.0:
					hit = True
					break
		if hit:
			break
	print("개요 그림 시점:", t, "난수시드", rs, "역할", sorted(set(int(k) for k in env.goal_kind)))
	R = env.comm_range
	cc = env.cc_pos
	ax.set_aspect("equal")
	ax.set_xlim(0, 1000); ax.set_ylim(0, 1000)
	ax.set_xticks([0, 500, 1000]); ax.set_yticks([0, 500, 1000])
	ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
	# 목적지
	act = env.dests_active
	ax.scatter(env.dests_pos[act, 0], env.dests_pos[act, 1], s=10, marker="x", color=C_D, label="unserved demand point")
	ax.scatter(env.dests_pos[~act, 0], env.dests_pos[~act, 1], s=10, marker="x", color="#cccccc", label="served")
	# 도달 한계 (0.9 N R)
	ax.add_patch(Circle(cc, 0.9 * 3 * R, fill=False, ls=":", lw=0.8, color="#999"))
	# 통신 원판
	nodes = [cc] + [env.drones_pos[i] for i in range(3)]
	for p in nodes:
		ax.add_patch(Circle(p, R, fill=True, alpha=0.06, color=C_L, lw=0))
		ax.add_patch(Circle(p, R, fill=False, lw=0.5, color=C_L, alpha=0.5))
	# 연결 간선
	for i, a in enumerate(nodes):
		for b in nodes[i + 1:]:
			if np.linalg.norm(np.asarray(a) - np.asarray(b)) <= R:
				ax.plot([a[0], b[0]], [a[1], b[1]], color=C_L, lw=1.2)
	# 제어 센터·드론
	ax.scatter([cc[0]], [cc[1]], s=90, marker="s", color=C_CC, zorder=5, label="depot")
	kinds = {0: ("deliver", "o"), 1: ("return to reload", "v"), 2: ("hold", "d"), 3: ("relay", "^")}
	seen = set()
	for i in range(3):
		k = int(env.goal_kind[i]); nm, mk = kinds.get(k, ("drone", "o"))
		lab = f"drone: {nm}" if nm not in seen else None; seen.add(nm)
		col = C_H if nm == "relay" else ("#2d7a5c" if nm.startswith("return") else C_L)
		ax.scatter([env.drones_pos[i, 0]], [env.drones_pos[i, 1]], s=60, marker=mk,
		           color=col, edgecolor="k", lw=0.5, zorder=6, label=lab)
		g = env.goal_pos[i]
		ax.add_patch(FancyArrowPatch(env.drones_pos[i], g, arrowstyle="-|>", mutation_scale=8, lw=0.8,
		                             color="#333", alpha=0.7, zorder=4))
	ax.legend(frameon=True, framealpha=0.85, edgecolor="none", fontsize=6, loc="upper left", markerscale=0.8,
	          borderpad=0.3, labelspacing=0.3, handletextpad=0.4)
	ax.set_title(f"(a) a multi-hop relay chain (step {t})", fontsize=8.5, loc="left")


def box(ax, x, y, w, h, text, fc="#f4f6f8", ec="#555", fs=7):
	ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.02", fc=fc, ec=ec, lw=0.8))
	ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs)


def arrow(ax, p, q, text=None):
	ax.add_patch(FancyArrowPatch(p, q, arrowstyle="-|>", mutation_scale=9, lw=0.9, color="#333"))
	if text:
		ax.text((p[0] + q[0]) / 2, (p[1] + q[1]) / 2 + 0.03, text, ha="center", fontsize=6.5, color="#333")


def panel_controllers(ax):
	"""두 제어기와 공유 하단(옵션 -> 직진 하위 -> 투영 -> 환경)."""
	ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis("off")
	ax.set_title("(b) two controllers, one shared projection", fontsize=8.5, loc="left")
	# 상단: 관측
	box(ax, 0.25, 0.86, 0.50, 0.10, "fleet state every 20 steps\n(positions, cargo, hops, unserved demand points)", fs=6.5)
	# 왼쪽: 휴리스틱
	box(ax, 0.01, 0.50, 0.47, 0.28,
	    "Geometric chain-relay heuristic\n"
	    "lead: farthest drone with cargo\n"
	    "relays: ⌈d/0.9R⌉−1 next-farthest\n"
	    "others: nearest demand point or return\n"
	    "escape: swap lead after $k_{stall}$ stalled steps",
	    fc="#fbeeec", ec=C_H, fs=5.8)
	# 오른쪽: 학습
	box(ax, 0.52, 0.50, 0.47, 0.28,
	    "Learned set-based manager\n"
	    "entities: self, candidates, peers, slot\n"
	    "attention pooling, pointer + fixed heads\n"
	    "autoregressive, farthest drone first\n"
	    "trained once (SAC) on 3–4 drones",
	    fc="#eaf2f8", ec=C_L, fs=5.8)
	arrow(ax, (0.40, 0.86), (0.25, 0.78)); arrow(ax, (0.60, 0.86), (0.75, 0.78))
	# 공유 하단
	box(ax, 0.12, 0.32, 0.76, 0.10, "per-drone option: one of 5 candidate demand points, return, or relay slot", fs=6.3)
	arrow(ax, (0.24, 0.50), (0.40, 0.42)); arrow(ax, (0.76, 0.50), (0.60, 0.42))
	box(ax, 0.12, 0.17, 0.76, 0.09, "straight-line flight to the goal (low-level controller)", fs=6.3)
	arrow(ax, (0.50, 0.32), (0.50, 0.26))
	box(ax, 0.12, 0.02, 0.76, 0.09, "connectivity projection: scale each move so the graph stays connected\n(constraint holds by construction)",
	    fc="#eeeeee", fs=6.0)
	arrow(ax, (0.50, 0.17), (0.50, 0.11))


def main():
	fig, axes = plt.subplots(1, 2, figsize=(7.6, 3.6), gridspec_kw={"width_ratios": [1.0, 1.2]})
	panel_geometry(axes[0])
	panel_controllers(axes[1])
	fig.tight_layout()
	out = os.path.join(FIG, "fig_overview_v5_26_10_17_15.pdf")
	fig.savefig(out); fig.savefig(out.replace(".pdf", ".png"), dpi=160); plt.close(fig)
	print("저장:", out)


if __name__ == "__main__":
	main()
