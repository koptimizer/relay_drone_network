"""RQ2 자료: h=98 격자 시공간 모형의 정확한 최적값 대 세 방법 (규칙·탐색 휴리스틱·학습, M=3..6).

제어기는 모형과 같은 결정 공간(노드 위치·기간 이동·기간 서비스)에서 돈다. 따라서 모형의 최적값이
이 공간에서 달성 가능한 하한이고 어떤 방법도 그보다 빠를 수 없다 (최적 계획 재생으로 검증).
연속 공간에서 잰 값도 함께 보고해 두 측정이 무엇을 재는지 구분한다.
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
C_R, C_S, C_L, C_O = "#b04236", "#8a5fb0", "#1d6d9c", "#333333"
MS = (3, 4, 5, 6, 7, 8)


def pstr(p):
	return "$<10^{-4}$" if p < 1e-4 else f"{p:.3f}" if p >= 0.001 else f"{p:.1e}"


def num(x, k):
	return float(x[k]) if x and x.get(k) not in (None, "") else np.nan


def load():
	"""(M, seed) -> dict. M<=6: bfs2 최적(연속 연결성 조건) / M=7,8 Q=2: 적재 포함 정확해."""
	g = lambda rows, k: {(int(x["dests"]), int(x["seed"])): x for f in rows for x in csv.DictReader(open(f, encoding="utf-8"))}
	lat = g([os.path.join(FIG, "lattice_eval_v5_26_10_21_03.csv")] + glob.glob(os.path.join(FIG, "lattice_m[78]q2_SWa_v5_26_10_21_17.csv")), "")
	order = g(glob.glob(os.path.join(FIG, "meta2_ils_m*_v5_26_10_21_03.csv")) + glob.glob(os.path.join(FIG, "meta_m[78]q2_v5_26_10_21_17.csv")), "")
	task = g(glob.glob(os.path.join(FIG, "taskseq_m[3-6]_v5_26_10_22_14.csv")) + glob.glob(os.path.join(FIG, "taskseq_m[78]q2_big_v5_26_10_22_16.csv")), "")
	cont = g([os.path.join(FIG, "toy_snap_v5_26_10_20_16.csv")], "")
	out = {}
	for k, x in lat.items():
		out[k] = dict(opt=num(x, "opt"), rule=num(x, "rule"), learned=num(x, "learned_median"),
		              q1=num(x, "learned_q1"), q3=num(x, "learned_q3"), esc=num(x, "esc_median"),
		              meta=num(order.get(k), "meta"), task=num(task.get(k), "taskseq"),
		              tsec=num(task.get(k), "sec"), osec=num(order.get(k), "sec"),
		              c_opt=num(cont.get(k), "milp"), c_rule=num(cont.get(k), "heuristic"),
		              c_learned=num(cont.get(k), "learned_median"))
	return out


def tab(d, lang):
	"""M별 한 행: 최적값, 네 방법의 평균 최적 격차, 최적 도달 수, 완주."""
	head = ("$M$ & $Q$ & inst. & optimum & \\multicolumn{4}{c}{mean gap to the optimum} & \\multicolumn{2}{c}{optima reached} & \\multicolumn{2}{c}{completed} \\\\"
	        if lang == "en" else
	        "$M$ & $Q$ & 건수 & 최적값 & \\multicolumn{4}{c}{최적값 대비 평균 격차} & \\multicolumn{2}{c}{최적 도달} & \\multicolumn{2}{c}{완주} \\\\")
	sub = ("& & & (mean) & rule & order search & task search & learned & order & task & rule & learned \\\\" if lang == "en"
	       else "& & & (평균) & 규칙 & 순서 탐색 & 작업열 탐색 & 학습 & 순서 & 작업열 & 규칙 & 학습 \\\\")
	lines, G = [], {k: [] for k in ("rule", "meta", "task", "learned")}
	for M in MS:
		ks = sorted(k for k in d if k[0] == M)
		if not ks:
			continue
		o = np.array([d[k]["opt"] for k in ks]); Q = 2 if M >= 7 else 5
		cols = []
		for key in ("rule", "meta", "task", "learned"):
			v = np.array([d[k][key] for k in ks]); G[key] += list(v / o)
			cols.append(f"{100*np.nanmean(v/o-1):+.1f}\\%")
		mo = int(np.nansum(np.array([d[k]["meta"] for k in ks]) <= o + 1e-6)); to = int(np.nansum(np.array([d[k]["task"] for k in ks]) <= o + 1e-6))
		cr = int(np.sum(~np.isnan([d[k]["rule"] for k in ks]))); cl = int(np.sum(~np.isnan([d[k]["learned"] for k in ks])))
		lines.append(f"{M} & {Q} & {len(ks)} & {o.mean():.0f} & " + " & ".join(cols) + f" & {mo}/{len(ks)} & {to}/{len(ks)} & {cr}/{len(ks)} & {cl}/{len(ks)} \\\\")
	A = {k: np.array(v) for k, v in G.items()}
	m = ~np.isnan(A["rule"]) & ~np.isnan(A["meta"]) & ~np.isnan(A["task"]) & ~np.isnan(A["learned"])
	tot = "all" if lang == "en" else "전체"
	lines.append("\\midrule\n" + f"{tot} & & {int(m.sum())} & --- & " + " & ".join(f"{100*(np.nanmean(A[k][m])-1):+.1f}\\%" for k in ("rule", "meta", "task", "learned")) + " & & & & \\\\")
	with open(os.path.join(TEX, f"tab_exact_v5_{lang}_26_10_19_22.tex"), "w") as f:
		f.write("\\begin{tabular}{rrrrrrrrrrrr}\n\\toprule\n" + head + "\n\\cmidrule(lr){5-8}\\cmidrule(lr){9-10}\\cmidrule(lr){11-12}\n"
		        + sub + "\n\\midrule\n" + "\n".join(lines) + "\n\\bottomrule\n\\end{tabular}%\n")
	ps = dict(rt=stats.wilcoxon(A["rule"][m] - A["task"][m]).pvalue, mt=stats.wilcoxon(A["meta"][m] - A["task"][m]).pvalue,
	          rl=stats.wilcoxon(A["rule"][m] - A["learned"][m]).pvalue)
	print(f"tab_exact({lang}): {len(lines)}행 | 규칙-작업열 {pstr(ps['rt'])} 순서-작업열 {pstr(ps['mt'])} 규칙-학습 {pstr(ps['rl'])} | n={int(m.sum())}")
	return ps


def fig(d):
	"""M별 패널: 인스턴스별 최적값과 세 방법의 makespan."""
	f, axes = plt.subplots(1, len(MS), figsize=(7.2, 2.4))
	for ax, M in zip(np.atleast_1d(axes), MS):
		ks = sorted(k for k in d if k[0] == M); x = np.arange(1, len(ks) + 1)
		ax.plot(x, [d[k]["opt"] for k in ks], "s-", ms=4, lw=0.9, color=C_O, label="optimum of the model ($h$ = 98 m)")
		ax.plot(x, [d[k]["rule"] for k in ks], "o", ms=4, color=C_R, label="rule")
		ax.plot(x - 0.18, [d[k]["meta"] for k in ks], "D", ms=3.6, color=C_S, alpha=0.5, label="order search")
		ax.plot(x - 0.18, [d[k]["task"] for k in ks], "D", ms=3.6, color=C_S, label="task-sequence search")
		lm = np.array([d[k]["learned"] for k in ks])
		q1 = np.array([d[k]["q1"] for k in ks]); q3 = np.array([d[k]["q3"] for k in ks])
		ax.errorbar(x + 0.18, lm, yerr=[np.maximum(0, lm - q1), np.maximum(0, q3 - lm)], fmt="^", ms=4,
		            color=C_L, lw=0.8, capsize=1.5, label="learned, median (IQR of 30)")
		# 축은 중앙값 기준으로 맞추고, 사분위 수염이 밖으로 나가면 잘라 표시한다
		vals = np.concatenate([[d[k]["opt"] for k in ks], [d[k]["rule"] for k in ks],
		                       [d[k]["meta"] for k in ks], lm])
		lo, hi = np.nanmin(vals) * 0.88, np.nanmax(vals) * 1.12
		ax.set_ylim(lo, hi)
		for xi, q in zip(x + 0.18, q3):
			if q > hi:
				ax.plot([xi], [hi], marker="^", ms=3, color=C_L, clip_on=False, alpha=0.5)
		ax.set_title(f"$M$ = {M}" + (" ($Q$ = 2)" if M >= 7 else ""), fontsize=8); ax.set_xticks(x); ax.tick_params(labelsize=7)
		ax.set_xlabel("instance", fontsize=7.5)
	np.atleast_1d(axes)[0].set_ylabel("makespan (steps)", fontsize=7.5)
	h, l = np.atleast_1d(axes)[0].get_legend_handles_labels()
	f.legend(h, l, loc="upper center", ncol=5, frameon=False, fontsize=6.3, bbox_to_anchor=(0.5, 1.03))
	f.tight_layout(rect=(0, 0, 1, 0.9))
	out = os.path.join(FIG, "fig_exact_v5_26_10_19_22.pdf"); f.savefig(out); f.savefig(out.replace(".pdf", ".png"), dpi=160); plt.close(f)
	print("저장:", out)


def main():
	argparse.ArgumentParser().parse_args()
	d = load(); print("인스턴스", len(d))
	for lang in ("en", "ko"):
		tab(d, lang)
	fig(d)
	for key, nm in (("osec", "순서 탐색"), ("tsec", "작업열 탐색")):
		sec = np.array([d[k][key] for k in d if k[0] <= 6]); sec2 = np.array([d[k][key] for k in d if k[0] >= 7])
		print(f"{nm} 시간: M<=6 평균 {np.nanmean(sec):.0f}초 (최대 {np.nanmax(sec):.0f}) | 재적재 평균 {np.nanmean(sec2)/60:.0f}분")


if __name__ == "__main__":
	main()
