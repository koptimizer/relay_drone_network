"""RQ2 자료: h=98 격자 시공간 모형의 정확한 최적값 대 두 제어기 (M=3..6, 각 10 인스턴스).

평가에서 중계 드론이 설 수 있는 자리를 모형과 같은 격자 노드로 제한해(toy_compare --lattice-snap)
중계 배치의 이산화 차이를 없앴다. 남은 차이는 배송 드론의 격자 이동과 10스텝 기간뿐이다.
"""

import argparse
import csv
import glob
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FIG = os.path.join(ROOT, "figures"); TEX = os.path.join(ROOT, "docs", "tex")
plt.rcParams.update({"font.size": 8, "pdf.fonttype": 42, "font.family": "DejaVu Sans"})
C_H, C_L, C_E, C_O = "#b04236", "#1d6d9c", "#2d7a5c", "#333333"
MS = (3, 4, 5, 6)


def pstr(p):
	return "$<10^{-4}$" if p < 1e-4 else f"{p:.3f}" if p >= 0.001 else f"{p:.1e}"


def num(x, k):
	return float(x[k]) if x.get(k) else np.nan


def load():
	"""(M, seed) -> (최적값, 제어기 행). 최적값은 BFS 가 증명한 h=98 격자 최적."""
	toy = {(int(r["dests"]), int(r["seed"])): r for r in csv.DictReader(open(os.path.join(FIG, "toy_snap_v5_26_10_20_16.csv"), encoding="utf-8"))}
	out = {}
	for f in sorted(glob.glob(os.path.join(FIG, "bfs_n3m*_h98_26_10_19_22.json"))):
		d = json.load(open(f)); k = (int(d["dests"]), int(d["seed"]))
		if d["milp_steps"] and k in toy:
			out[k] = (float(d["milp_steps"]), toy[k], d)
	return out


def tab(data, lang):
	"""M별 한 행: 최적값 평균, 완주 수, 제어기별 평균 최적 격차, 휴리 대 학습 검정."""
	head = ("$M$ & inst. & optimum & \\multicolumn{2}{c}{completed} & \\multicolumn{3}{c}{gap to the optimum} & $p$ \\\\"
	        if lang == "en" else
	        "$M$ & 건수 & 최적값 & \\multicolumn{2}{c}{완주} & \\multicolumn{3}{c}{최적값 대비 격차} & $p$ \\\\")
	sub = ("& & (mean) & heur. & learned & heuristic & learned & learned+rec. & \\\\" if lang == "en"
	       else "& & (평균) & 휴리 & 학습 & 휴리스틱 & 학습 & 학습+복구 & \\\\")
	lines, G = [], {"h": [], "l": [], "e": []}
	for M in MS:
		ks = sorted(k for k in data if k[0] == M)
		o = np.array([data[k][0] for k in ks])
		h = np.array([num(data[k][1], "heuristic") for k in ks])
		l = np.array([num(data[k][1], "learned_median") for k in ks])
		e = np.array([num(data[k][1], "esc_median") for k in ks])
		m = ~np.isnan(h) & ~np.isnan(l)
		p = stats.wilcoxon(h[m] / o[m] - l[m] / o[m]).pvalue if m.sum() > 5 else np.nan
		for key, v in (("h", h), ("l", l), ("e", e)):
			G[key] += list(v[m] / o[m])
		nh = int((~np.isnan(h)).sum()); nl = int((~np.isnan(l)).sum())
		lines.append(f"{M} & {len(ks)} & {o.mean():.0f} & {nh}/{len(ks)} & {nl}/{len(ks)} & "
		             f"{100*np.nanmean(h/o-1):+.1f}\\% & {100*np.nanmean(l/o-1):+.1f}\\% & {100*np.nanmean(e/o-1):+.1f}\\% & "
		             + (pstr(p) if not np.isnan(p) else "---") + " \\\\")
	H, L, E = (np.array(G[k]) for k in ("h", "l", "e"))
	tot = ("all" if lang == "en" else "전체")
	lines.append("\\midrule\n" + f"{tot} & {len(H)} & --- & --- & --- & {100*(H.mean()-1):+.1f}\\% & "
	             f"{100*(L.mean()-1):+.1f}\\% & {100*(E.mean()-1):+.1f}\\% & " + pstr(stats.wilcoxon(H - L).pvalue) + " \\\\")
	with open(os.path.join(TEX, f"tab_exact_v5_{lang}_26_10_19_22.tex"), "w") as f:
		f.write("\\begin{tabular}{rrrrrrrrr}\n\\toprule\n" + head + "\n\\cmidrule(lr){4-5}\\cmidrule(lr){6-8}\n"
		        + sub + "\n\\midrule\n" + "\n".join(lines) + "\n\\bottomrule\n\\end{tabular}%\n")
	print(f"tab_exact({lang}): {len(lines)}행 | 전체 휴리 {100*(H.mean()-1):+.1f}% 학습 {100*(L.mean()-1):+.1f}% 복구 {100*(E.mean()-1):+.1f}%")


def fig(data):
	"""M별 패널: 인스턴스별 최적값과 두 제어기의 makespan."""
	f, axes = plt.subplots(1, len(MS), figsize=(7.2, 2.4))
	for ax, M in zip(np.atleast_1d(axes), MS):
		ks = sorted(k for k in data if k[0] == M); x = np.arange(1, len(ks) + 1)
		ax.plot(x, [data[k][0] for k in ks], "s-", ms=4, lw=0.9, color=C_O, label="lattice optimum ($h$ = 98 m)")
		ax.plot(x, [num(data[k][1], "heuristic") for k in ks], "o", ms=4, color=C_H, label="heuristic")
		lm = np.array([num(data[k][1], "learned_median") for k in ks])
		q1 = np.array([num(data[k][1], "learned_q1") for k in ks]); q3 = np.array([num(data[k][1], "learned_q3") for k in ks])
		ax.errorbar(x + 0.15, lm, yerr=[lm - q1, q3 - lm], fmt="^", ms=4, color=C_L, lw=0.8, capsize=1.5,
		            label="learned, median (IQR of 30)")
		ax.plot(x - 0.15, [num(data[k][1], "esc_median") for k in ks], "v", ms=4, color=C_E, label="learned + recovery")
		ax.set_title(f"$M$ = {M}", fontsize=8); ax.set_xticks(x); ax.tick_params(labelsize=7)
		ax.set_xlabel("instance", fontsize=7.5)
	np.atleast_1d(axes)[0].set_ylabel("makespan (steps)", fontsize=7.5)
	h, l = np.atleast_1d(axes)[0].get_legend_handles_labels()
	f.legend(h, l, loc="upper center", ncol=4, frameon=False, fontsize=6.5, bbox_to_anchor=(0.5, 1.03))
	f.tight_layout(rect=(0, 0, 1, 0.9))
	out = os.path.join(FIG, "fig_exact_v5_26_10_19_22.pdf"); f.savefig(out); f.savefig(out.replace(".pdf", ".png"), dpi=160); plt.close(f)
	print("저장:", out)


def main():
	argparse.ArgumentParser().parse_args()
	d = load(); print("인스턴스", len(d))
	for lang in ("en", "ko"):
		tab(d, lang)
	fig(d)


if __name__ == "__main__":
	main()
