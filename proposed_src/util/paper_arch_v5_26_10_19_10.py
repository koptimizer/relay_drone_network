"""계층형 학습 제어기 구조 그림: 상위 집합 기반 관리자(20스텝마다, 자기회귀) → 옵션 → 하위 직진 제어 → 투영.

상위 안쪽은 엔티티 인코더 → 어텐션 풀링 → 융합 → 포인터/고정 헤드, 그리고 결정된 역할을 동료 특징에
써넣는 자기회귀 고리를 보인다.
"""

import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FIG = os.path.join(ROOT, "figures")
plt.rcParams.update({"font.size": 7, "pdf.fonttype": 42, "font.family": "DejaVu Sans"})
C_L, C_W, C_P = "#1d6d9c", "#2d7a5c", "#555555"


def box(ax, x, y, w, h, text, fc="#f4f6f8", ec="#555", fs=6.5, lw=0.8, bold=False):
	ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.008,rounding_size=0.012", fc=fc, ec=ec, lw=lw))
	ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs, fontweight="bold" if bold else "normal")


def arrow(ax, p, q, color="#333", lw=0.9, style="-|>", ls="-"):
	ax.add_patch(FancyArrowPatch(p, q, arrowstyle=style, mutation_scale=8, lw=lw, color=color, linestyle=ls))


def main():
	fig, ax = plt.subplots(figsize=(7.4, 4.3))
	ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis("off")

	# ---- 상위: 관리자 (큰 틀)
	ax.add_patch(FancyBboxPatch((0.01, 0.40), 0.98, 0.58, boxstyle="round,pad=0.004,rounding_size=0.015",
	                            fc="#eaf2f8", ec=C_L, lw=1.2))
	ax.text(0.02, 0.965, "High level: set-based manager, one decision every 20 steps (or when a goal becomes invalid)",
	        fontsize=7.5, fontweight="bold", color=C_L, va="center")

	# 엔티티 입력
	ents = [("self\n10 features", 0.03), ("5 candidate\ndestinations\n5 × 6", 0.16), ("peers\n(N−1) × 9\nincl. role one-hot", 0.29), ("relay slot\n4 features", 0.42)]
	for t, x in ents:
		box(ax, x, 0.80, 0.11, 0.12, t, fs=6)
	ax.text(0.03, 0.935, "entities of drone i (distances / R)", fontsize=6, color="#555")
	# 인코더
	for t, x in ents:
		box(ax, x, 0.665, 0.11, 0.07, "MLP 128", fs=6)
		arrow(ax, (x + 0.055, 0.80), (x + 0.055, 0.735))
	# 어텐션 풀링 (동료·후보)
	box(ax, 0.16, 0.545, 0.11, 0.07, "attention pool\n(query: self)", fs=5.8)
	box(ax, 0.29, 0.545, 0.11, 0.07, "attention pool\n(query: self)", fs=5.8)
	arrow(ax, (0.215, 0.665), (0.215, 0.615)); arrow(ax, (0.345, 0.665), (0.345, 0.615))
	arrow(ax, (0.085, 0.665), (0.19, 0.615), color="#888", lw=0.6, ls="--"); arrow(ax, (0.085, 0.665), (0.32, 0.615), color="#888", lw=0.6, ls="--")
	# 융합
	box(ax, 0.03, 0.43, 0.50, 0.075, "concatenate [self · peers · candidates · slot] → fused embedding h_i (128)", fs=6)
	for x in (0.085, 0.215, 0.345, 0.475):
		arrow(ax, (x, 0.545 if x in (0.215, 0.345) else 0.665), (x, 0.505))
	# 헤드
	box(ax, 0.58, 0.72, 0.17, 0.10, "pointer head\nscore each of the\n5 candidates", fc="#fff8e6", ec="#93641a", fs=6)
	box(ax, 0.58, 0.58, 0.17, 0.10, "fixed head\nreturn · relay", fc="#fff8e6", ec="#93641a", fs=6)
	arrow(ax, (0.53, 0.47), (0.58, 0.77), lw=0.8); arrow(ax, (0.53, 0.47), (0.58, 0.63), lw=0.8)
	box(ax, 0.79, 0.62, 0.19, 0.16, "masked softmax over\n7 options → sample\noption of drone i", fc="#fff8e6", ec="#93641a", fs=6)
	arrow(ax, (0.75, 0.77), (0.79, 0.72)); arrow(ax, (0.75, 0.63), (0.79, 0.68))
	# 자기회귀 고리
	ax.add_patch(FancyArrowPatch((0.88, 0.62), (0.36, 0.92), connectionstyle="arc3,rad=-0.35", arrowstyle="-|>",
	                             mutation_scale=9, lw=1.0, color=C_L, ls="--"))
	ax.text(0.70, 0.90, "autoregressive: write drone i's new role into the peer\nfeatures, then decide the next drone (farthest from centre first)",
	        fontsize=6, color=C_L, ha="center")
	# 출력
	box(ax, 0.60, 0.44, 0.37, 0.09, "options for all N drones: destination / return / relay slot", fc="#ffffff", ec=C_L, fs=6.3, bold=True)
	arrow(ax, (0.885, 0.62), (0.80, 0.53))

	# ---- 하위
	ax.add_patch(FancyBboxPatch((0.01, 0.02), 0.98, 0.33, boxstyle="round,pad=0.004,rounding_size=0.015",
	                            fc="#ebf5f0", ec=C_W, lw=1.2))
	ax.text(0.02, 0.325, "Low level: every step", fontsize=7.5, fontweight="bold", color=C_W, va="center")
	box(ax, 0.05, 0.17, 0.26, 0.10, "straight-line flight to the\nassigned goal at full speed", fc="#ffffff", ec=C_W, fs=6.3)
	box(ax, 0.37, 0.17, 0.26, 0.10, "connectivity projection:\nscale moves so the graph\nstays connected", fc="#ffffff", ec=C_W, fs=6.3)
	box(ax, 0.69, 0.17, 0.26, 0.10, "environment step:\ndeliver / reload / relay,\nservice delays", fc="#ffffff", ec=C_W, fs=6.3)
	arrow(ax, (0.31, 0.22), (0.37, 0.22)); arrow(ax, (0.63, 0.22), (0.69, 0.22))
	arrow(ax, (0.785, 0.44), (0.18, 0.27), lw=1.0)
	ax.text(0.47, 0.37, "goals held for 20 steps (semi-MDP option)", fontsize=6, color="#333", ha="center")
	# 보상·학습 신호
	ax.text(0.05, 0.085, "training (discrete SAC): team reward +10 per delivery, +30 per reload, −0.01N per step,\n"
	                     "residual penalty; option return = discounted reward over the 20-step segment; twin-Q critic over the drone set;\n"
	                     "entropy target annealed 0.6→0.25·log n; cross-entropy regularisation towards the heuristic's assignment (weight 2.0)",
	        fontsize=5.8, color="#333", va="center")
	arrow(ax, (0.82, 0.17), (0.82, 0.13), color="#888", lw=0.6, ls="--")

	fig.tight_layout()
	out = os.path.join(FIG, "fig_arch_v5_26_10_19_10.pdf")
	fig.savefig(out); fig.savefig(out.replace(".pdf", ".png"), dpi=160); plt.close(fig)
	print("저장:", out)


if __name__ == "__main__":
	main()
