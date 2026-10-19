"""격자 지도 그림: 시공간 네트워크 모형에서 드론이 위치할 수 있는 노드 집합을 보여준다.

MILP 스크립트의 build_nodes 로 같은 인스턴스(드론 3, 목적지 4, 시드 902)의 노드를 만들고,
한 노드의 이동 집합 A(i)(한 기간에 갈 수 있는 여덟 이웃)와 통신 집합 E(반경 R 안의 노드)를 표시한다.
"""

import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
from pipeline.milp_v5_26_09_21_03 import build_nodes
from util.instance_generator_v5_26_09_15_22 import reach_limit, sample_instance

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FIG = os.path.join(ROOT, "figures")
plt.rcParams.update({"font.size": 8, "pdf.fonttype": 42, "font.family": "DejaVu Sans"})


def main():
	N, M, R, seed = 3, 4, 300.0, 902
	cc, dests = sample_instance(M, seed=seed, num_drones=N, comm_range=R, cc_pos=None)
	env0 = DisasterRelayDroneEnv(num_drones=N, num_dests=M)
	ell = env0.max_speed * 10                   # 한 기간(10스텝)의 비행 거리
	h = ell / np.sqrt(2.0)
	pos, nd = build_nodes(cc, dests, ell, h, R, N)
	lim = reach_limit(N, R, 0.9)
	grid = pos[1 + nd:]
	fig, ax = plt.subplots(figsize=(4.6, 4.9))
	ax.set_aspect("equal")
	# 도달 한계와 격자
	ax.add_patch(Circle(cc, lim, fill=False, ls=":", lw=0.9, color="#888", label="reach limit $0.9NR$ (nodes beyond are deleted)"))
	ax.scatter(grid[:, 0], grid[:, 1], s=6, color="#9aa6b0", label=f"lattice node (spacing $h$ = {h:.1f} m)")
	# 예시 노드 i: 격자 중 센터에서 약 400 m 떨어진 노드
	d = np.linalg.norm(grid - cc, axis=1)
	i = int(np.argmin(np.abs(d - 400.0)))
	pi = grid[i]
	# 이동 집합 A(i): ell 안의 노드 (여덟 이웃 + 자기)
	dA = np.linalg.norm(pos - pi, axis=1)
	A = pos[(dA <= ell + 1e-6)]
	ax.scatter(A[:, 0], A[:, 1], s=26, facecolor="none", edgecolor="#1d6d9c", lw=1.0,
	           label="move set $A(i)$: reachable in one period ($\\ell$ = %.0f m)" % ell)
	ax.add_patch(Circle(pi, ell, fill=False, ls="-", lw=0.8, color="#1d6d9c", alpha=0.6))
	# 통신 집합 E: R 안의 노드
	ax.add_patch(Circle(pi, R, fill=True, alpha=0.07, color="#b04236", lw=0))
	ax.add_patch(Circle(pi, R, fill=False, ls="--", lw=0.8, color="#b04236"))
	ax.scatter([pi[0]], [pi[1]], s=40, color="#1d6d9c", zorder=5, label="example node $i$")
	ax.text(pi[0] + R * 0.72, pi[1] + R * 0.72, "communication\narcs $E$: within $R$", fontsize=7, color="#b04236")
	# 센터·목적지
	ax.scatter([cc[0]], [cc[1]], s=90, marker="s", color="#222", zorder=6, label="control centre (node 0)")
	ax.scatter(dests[:, 0], dests[:, 1], s=50, marker="x", color="#222", lw=1.5, zorder=6, label="destination nodes $D$")
	ax.set_xlim(pos[:, 0].min() - 120, pos[:, 0].max() + 120); ax.set_ylim(pos[:, 1].min() - 80, pos[:, 1].max() + 230)
	ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
	ax.legend(frameon=False, fontsize=6.3, loc="upper center", bbox_to_anchor=(0.5, -0.14), ncol=2)
	ax.set_title(f"space–time network nodes: $N_v$ = {len(pos)} (3 drones, 4 destinations)", fontsize=8.5)
	fig.tight_layout()
	out = os.path.join(FIG, "fig_lattice_v5_26_10_19_10.pdf")
	fig.savefig(out); fig.savefig(out.replace(".pdf", ".png"), dpi=160); plt.close(fig)
	print("저장:", out, "| 노드", len(pos), "격자", len(grid), "h", round(h, 1), "ell", round(ell, 1))


if __name__ == "__main__":
	main()
