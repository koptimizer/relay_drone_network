"""쌍별 비교 자료를 논문용 그림 한 장으로 만든다 (구성 3개 x 누적분포 + 쌍별 차이).

완주 평균 하나로는 보이지 않는 꼬리와 승패 구조를 한 눈에 보여주는 것이 목적이다.
축과 범례는 영문으로 둔다 (docs/tex 관례).
"""

import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CFG = (("4 drones / 50 dests (seen)", ("paired_v5_26_09_27_08", "학습"),
        ("stall_temp_v5_26_09_27_09", "개입 없음")),
       ("3 drones / 30 dests (unseen)", ("paired_d3_rep5_v5_26_09_27_11", "학습"), None),
       ("5 drones / 50 dests (unseen)", ("paired_d5_rep5_v5_26_09_27_11", "학습"), None))


def load(name, key):
	"""쌍별 CSV에서 (규칙, 학습) 배열을 뽑는다. 미완주는 NaN."""
	f = os.path.join(ROOT, "figures", f"{name}.csv")
	r = list(csv.DictReader(open(f, encoding="utf-8")))
	a = np.array([float(x["규칙+탈출"]) if x["규칙+탈출"] else np.nan for x in r])
	b = np.array([float(x[key]) if x[key] else np.nan for x in r])
	return a, b


def main():
	"""세 구성의 누적분포와 쌍별 차이를 2행 3열로 그린다."""
	fig, ax = plt.subplots(2, 3, figsize=(12.0, 6.4))
	for j, (title, p1, p2) in enumerate(CFG):
		a, b = load(*p1)
		if p2:
			a2, b2 = load(*p2)
			a, b = np.concatenate([a, a2]), np.concatenate([b, b2])
		m = ~np.isnan(a) & ~np.isnan(b)
		A, B, d = a[m], b[m], b[m] - a[m]
		top = ax[0, j]
		for v, lb, c in ((A, "geometric rule + escape", "#c0392b"), (B, "learned policy (ours)", "#2471a3")):
			x = np.sort(v)
			top.step(x, np.arange(1, len(x) + 1) / len(x), where="post", color=c, lw=1.8, label=lb)
			top.axvline(np.median(v), color=c, ls=":", lw=1.0)
		top.set_title(f"{title}\n{int(m.sum())} paired rollouts", fontsize=10)
		top.set_xlabel("makespan (steps)"); top.set_ylabel("cumulative fraction")
		top.set_xlim(min(A.min(), B.min()) * 0.95, np.percentile(np.concatenate([A, B]), 99))
		top.grid(alpha=0.25); top.legend(fontsize=8, loc="lower right")
		bot = ax[1, j]
		lim = np.percentile(np.abs(d), 97)
		bot.hist(np.clip(d, -lim, lim), bins=31, color="#7f8c8d", edgecolor="white")
		bot.axvline(0, color="k", lw=1.0)
		bot.axvline(np.median(d), color="#2471a3", lw=1.8)
		p = stats.wilcoxon(d).pvalue
		ptxt = "p<0.0001" if p < 1e-4 else f"p={p:.3f}"
		bot.set_title(f"paired difference (learned − rule)\nmedian {np.median(d):+.0f}, "
		              f"{int((d < 0).sum())} wins / {int((d > 0).sum())} losses, {ptxt}", fontsize=9)
		if np.abs(d).max() > lim:
			bot.text(0.5, 0.95, f"tails clipped at ±{lim:.0f}", transform=bot.transAxes,
			         fontsize=7, ha="center", va="top", color="#555555")
		bot.set_xlabel("difference (steps)"); bot.set_ylabel("rollouts"); bot.grid(alpha=0.25)
		nf_a, nf_b = int(np.isnan(a).sum()), int(np.isnan(b).sum())
		top.text(0.03, 0.95, f"unfinished: rule {nf_a}, ours {nf_b}", transform=top.transAxes,
		         fontsize=8, va="top")
	fig.suptitle("Paired comparison on identical instances (budget 10,000 steps, sampling inference)", fontsize=11)
	fig.tight_layout(rect=(0, 0, 1, 0.96))
	out = os.path.join(ROOT, "figures", "paired_compare_v5_26_09_27_13.pdf")
	fig.savefig(out); fig.savefig(out.replace(".pdf", ".png"), dpi=160)
	print(f"저장: {out}")


if __name__ == "__main__":
	main()
