"""RQ2 자료 (사이클 30): 모형 최적값 대 여섯 방법 (규칙·선견 규칙·순서 탐색·작업열 탐색·빔 탐색·학습·미세조정 학습).

모든 방법은 모형과 같은 결정 공간에서 돈다. 격차 셀은 "평균 격차 (최적 도달 수)" 로 적는다.
학습 열은 30회 표본의 중앙값, 미세조정 열은 선택된 설정의 두 시드를 합친 60회의 중앙값이다.
"""

import argparse
import csv
import glob
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FIG = os.path.join(ROOT, "figures"); TEX = os.path.join(ROOT, "docs", "tex")
plt.rcParams.update({"font.size": 8, "pdf.fonttype": 42, "font.family": "DejaVu Sans"})
MS = (3, 4, 5, 6, 7, 8)
KEYS = ("rule", "look", "meta", "task", "beam", "learned", "ft", "hyb")
LAB = {"en": dict(rule="rule", look="lookahead", meta="order", task="task seq.", beam="beam",
                  learned="learned", ft="learned, FT", hyb="learned+LA"),
       "ko": dict(rule="규칙", look="선견", meta="순서", task="작업열", beam="빔", learned="학습", ft="학습 미세조정", hyb="학습+선견")}


def pstr(p):
	return "$<10^{-4}$" if p < 1e-4 else f"{p:.3f}" if p >= 0.001 else f"{p:.1e}"


def num(x, k):
	return float(x[k]) if x and x.get(k) not in (None, "") else np.nan


def rows(pats):
	out = {}
	for p in pats:
		for f in glob.glob(os.path.join(FIG, p)):
			for x in csv.DictReader(open(f, encoding="utf-8")):
				out[(int(x["dests"]), int(x["seed"]))] = x
	return out


def load(ft_runs):
	lat = rows(["lattice_eval_v5_26_10_21_03.csv", "lattice_m[78]q2_SWa_v5_26_10_21_17.csv"])
	look = rows(["rule2_union_m[3-6]_v5_26_10_26_20.csv", "rule2_union_m[78]q2_v5_26_10_26_20.csv"])
	hyb = rows(["rule2_lhSWa_m[3-6]_v5_26_10_26_20.csv", "rule2_lhSWa_m[78]q2_v5_26_10_26_20.csv"])
	order = rows(["meta2_ils_m*_v5_26_10_21_03.csv", "meta_m[78]q2_v5_26_10_21_17.csv"])
	task = rows(["taskseq_m[3-6]_v5_26_10_22_14.csv", "taskseq_m[78]q2_big_v5_26_10_22_16.csv"])
	beam = rows(["beam_m36_e2_v5_26_10_26_20.csv", "beam_m78q2_e2_v5_26_10_26_20.csv"])
	fts = [rows([f"lattice_c30_{r}_m36_v5_26_10_26_20.csv", f"lattice_c30_{r}_m78_v5_26_10_26_20.csv"]) for r in ft_runs]
	out = {}
	for k, x in lat.items():
		if k not in task:
			continue
		# 미세조정: 시드별 중앙값의 평균이 아니라 두 시드 표본을 합친 중앙값에 가깝게, 시드별 중앙값의 중앙값을 쓴다
		fv = [num(f.get(k), "learned_median") for f in fts]
		fc = sum(int(f.get(k, {}).get("learned_complete", "0/30").split("/")[0]) for f in fts if f.get(k))
		out[k] = dict(opt=num(x, "opt"), rule=num(x, "rule"), learned=num(x, "learned_median"),
		              q1=num(x, "learned_q1"), q3=num(x, "learned_q3"),
		              look=num(look.get(k), "lookahead"), lsec=num(look.get(k), "sec"),
		              meta=num(order.get(k), "meta"), task=num(task.get(k), "taskseq"),
		              beam=num(beam.get(k), "beam30000"), bsec=num(beam.get(k), "sec30000"),
		              hyb=num(hyb.get(k), "lookahead"), hsec=num(hyb.get(k), "sec"),
		              ft=np.nanmedian(fv) if fv and not all(np.isnan(fv)) else np.nan, ftc=fc, ftn=30 * len(fts))
	return out


def tab(d, lang):
	head = ("$M$ & $Q$ & inst. & opt. & \\multicolumn{8}{c}{mean gap to the optimum (instances on which the method returns the optimum)} \\\\"
	        if lang == "en" else
	        "$M$ & $Q$ & 건수 & 최적 & \\multicolumn{8}{c}{최적값 대비 평균 격차 (최적에 도달한 건수)} \\\\")
	sub = "& & & " + ("(mean)" if lang == "en" else "(평균)") + " & " + " & ".join(LAB[lang][k] for k in KEYS) + " \\\\"
	lines, G = [], {k: [] for k in KEYS}
	comp = {k: [0, 0] for k in KEYS}
	for M in MS:
		ks = sorted(k for k in d if k[0] == M)
		if not ks:
			continue
		o = np.array([d[k]["opt"] for k in ks]); Q = 2 if M >= 7 else 5
		cols = []
		for key in KEYS:
			v = np.array([d[k][key] for k in ks]); G[key] += list(v / o)
			hit = int(np.nansum(v <= o + 1e-6)); done = int(np.sum(~np.isnan(v)))
			comp[key][0] += done; comp[key][1] += len(ks)
			cols.append(f"{100*np.nanmean(v/o-1):+.1f}\\% ({hit})" + ("" if done == len(ks) else f"$^{{{done}/{len(ks)}}}$"))
		lines.append(f"{M} & {Q} & {len(ks)} & {o.mean():.0f} & " + " & ".join(cols) + " \\\\")
	A = {k: np.array(v) for k, v in G.items()}
	m = np.ones(len(A["rule"]), dtype=bool)
	for k in KEYS:
		m &= ~np.isnan(A[k])
	tot = "all, every method completes" if lang == "en" else "전체, 모든 방법 완주"
	lines.append("\\midrule\n" + f"\\multicolumn{{3}}{{l}}{{{tot}}} & {int(m.sum())} & "
	             + " & ".join(f"{100*(np.nanmean(A[k][m])-1):+.1f}\\%" for k in KEYS) + " \\\\")
	with open(os.path.join(TEX, f"tab_exact_v5_{lang}_26_10_26_20.tex"), "w") as f:
		f.write("\\begin{tabular}{rrrr" + "r" * len(KEYS) + "}\n\\toprule\n" + head + "\n\\cmidrule(lr){5-12}\n"
		        + sub + "\n\\midrule\n" + "\n".join(lines) + "\n\\bottomrule\n\\end{tabular}%\n")
	ps = {}
	for a, b in (("rule", "look"), ("look", "task"), ("task", "beam"), ("meta", "look"), ("learned", "ft"), ("rule", "learned"), ("look", "ft"), ("ft", "task"), ("learned", "hyb"), ("look", "hyb"), ("hyb", "task")):
		ps[a + "-" + b] = stats.wilcoxon(A[a][m] - A[b][m]).pvalue if np.any(A[a][m] != A[b][m]) else 1.0
	print(f"tab_exact2({lang}) n={int(m.sum())} | " + " ".join(f"{k} {pstr(v)}" for k, v in ps.items()))
	print("완주:", {k: f"{c[0]}/{c[1]}" for k, c in comp.items()})
	print("전체 평균 격차:", {k: f"{100*(np.nanmean(A[k][m])-1):+.1f}%" for k in KEYS})
	return ps


def fig(d):
	f, axes = plt.subplots(1, len(MS), figsize=(7.2, 2.7))
	C = dict(opt="#333333", rule="#b04236", look="#d98a3a", task="#8a5fb0", beam="#2d7a5c", learned="#1d6d9c", ft="#5aa0d0", hyb="#0b3d66")
	for ax, M in zip(np.atleast_1d(axes), MS):
		ks = sorted(k for k in d if k[0] == M); x = np.arange(1, len(ks) + 1)
		ax.plot(x, [d[k]["opt"] for k in ks], "s-", ms=4, lw=0.9, color=C["opt"], label="optimum of the model ($h$ = 98 m)")
		ax.plot(x - 0.25, [d[k]["rule"] for k in ks], "o", ms=3.6, color=C["rule"], label="rule")
		ax.plot(x - 0.25, [d[k]["look"] for k in ks], "o", ms=3.6, color=C["look"], label="lookahead rule")
		ax.plot(x - 0.08, [d[k]["task"] for k in ks], "D", ms=3.2, color=C["task"], label="task-sequence search")
		ax.plot(x - 0.08, [d[k]["beam"] for k in ks], "D", ms=3.2, color=C["beam"], label="beam search")
		lm = np.array([d[k]["learned"] for k in ks]); ft = np.array([d[k]["ft"] for k in ks]); hy = np.array([d[k]["hyb"] for k in ks])
		ax.plot(x + 0.1, lm, "^", ms=3.8, color=C["learned"], label="learned, median of 30")
		ax.plot(x + 0.22, ft, "^", ms=3.8, color=C["ft"], label="learned, fine-tuned, median")
		ax.plot(x + 0.34, hy, "v", ms=3.8, color=C["hyb"], label="learned + lookahead")
		vals = np.concatenate([[d[k][q] for k in ks] for q in ("opt", "rule", "look", "task", "beam")])
		lo, hi = np.nanmin(vals) * 0.88, np.nanmax(vals) * 1.15
		ax.set_ylim(lo, hi)
		for xi, med, c in list(zip(x + 0.1, lm, [C["learned"]] * len(x))) + list(zip(x + 0.22, ft, [C["ft"]] * len(x))):
			if med > hi:
				ax.plot([xi], [hi], marker="^", ms=5, color=c, clip_on=False)
				ax.annotate(f"{med:.0f}", (xi, hi), xytext=(0, 3), textcoords="offset points", ha="center", fontsize=5.3, color=c)
		ax.set_title(f"$M$ = {M}" + (" ($Q$ = 2)" if M >= 7 else ""), fontsize=8); ax.set_xticks(x); ax.tick_params(labelsize=7)
		ax.set_xlabel("instance", fontsize=7.5)
	np.atleast_1d(axes)[0].set_ylabel("makespan (steps)", fontsize=7.5)
	h, l = np.atleast_1d(axes)[0].get_legend_handles_labels()
	f.legend(h, l, loc="upper center", ncol=4, frameon=False, fontsize=6.2, bbox_to_anchor=(0.5, 1.0))
	f.tight_layout(rect=(0, 0, 1, 0.84))
	out = os.path.join(FIG, "fig_exact_v5_26_10_26_20.pdf"); f.savefig(out); f.savefig(out.replace(".pdf", ".png"), dpi=160); plt.close(f)
	print("저장:", out)


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--ft-runs", default="FT1,FT2", help="표의 미세조정 열에 쓰는 실행 (관문 지표로 고른 설정의 시드들)")
	A = p.parse_args()
	d = load(A.ft_runs.split(",")); print("인스턴스", len(d))
	for lang in ("en", "ko"):
		tab(d, lang)
	fig(d)
	for key, nm in (("lsec", "선견 규칙"), ("bsec", "빔 탐색"), ("hsec", "학습+선견")):
		s1 = np.array([d[k][key] for k in d if k[0] <= 6]); s2 = np.array([d[k][key] for k in d if k[0] >= 7])
		print(f"{nm} 시간: M<=6 평균 {np.nanmean(s1):.0f}초 (최대 {np.nanmax(s1):.0f}) | 재적재 평균 {np.nanmean(s2):.0f}초 (최대 {np.nanmax(s2):.0f})")


if __name__ == "__main__":
	main()
