"""논문 표 조각 생성 (사이클 26). CSV·JSON에서 LaTeX 행을 만들어 docs/tex/tab_*.tex 에 쓴다.

- tab_heur: 휴리스틱 절제 (RQ4)       <- figures/heur_ablate_v5_26_10_17_15_*.csv
- tab_milp: 격자 MILP 참조 비교 (RQ2) <- figures/toy_compare_v5_26_10_17_15.csv
- tab_range: 통신 반경 전이 (RQ3)     <- figures/paired_R{250,350}_*_SWa_v5_26_10_17_15.csv
- tab_paired: 크기별 쌍별 비교 (RQ3)  <- --suffix 로 지정한 재측정본 또는 기존 파일
"""

import argparse
import csv
import glob
import os

import numpy as np
from scipy import stats

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FIG = os.path.join(ROOT, "figures")
TEX = os.path.join(ROOT, "docs", "tex")


def paired(path):
	r = list(csv.DictReader(open(path, encoding="utf-8")))
	a = np.array([float(x["규칙+탈출"]) if x["규칙+탈출"] else np.nan for x in r])
	b = np.array([float(x["학습"]) if x["학습"] else np.nan for x in r])
	return a, b


def pstr(p):
	return "$<10^{-4}$" if p < 1e-4 else f"{p:.3f}" if p >= 0.001 else f"{p:.1e}"


def tab_heur(tag):
	rows = []
	for f in sorted(glob.glob(os.path.join(FIG, "heur_ablate_v5_26_10_17_15_*.csv"))):
		rows += list(csv.DictReader(open(f, encoding="utf-8")))
	if not rows:
		return
	cfgs = ["d3m30r", "d4m50f", "d5m50r"]
	keys = []
	for r in rows:
		k = (r["mode"], int(r["k_stall"]), int(r["hold"]))
		if k not in keys:
			keys.append(k)
	lines = []
	for k in keys:
		cells, cs = [], 0
		for c in cfgs:
			sel = [r for r in rows if r["config"] == c and (r["mode"], int(r["k_stall"]), int(r["hold"])) == k]
			if not sel:
				cells.append("---"); continue
			r = sel[0]; cs += int(r["complete"])
			cells.append(f"{r['complete']}/{r['n']} \\,/\\, {r['median'] or '---'}")
		mode = {"none": "none", "shuffle": "lead swap", "retreat": "retreat"}[k[0]]
		par = "---" if k[0] == "none" else f"{k[1]} / {k[2]}"
		lines.append(f"{mode} & {par} & " + " & ".join(cells) + f" & {cs}/120 \\\\")
	with open(os.path.join(TEX, f"tab_heur_v5_{tag}.tex"), "w") as f:
		f.write("\\begin{tabular}{llrrrr}\n\\toprule\nmode & $k_{\\mathrm{stall}}$ / $h_{\\mathrm{hold}}$ & $3/30$ & $4/50$ & $5/50$ & completions \\\\\n\\midrule\n"
		        + "\n".join(lines) + "\n\\bottomrule\n\\end{tabular}%\n")
	print("tab_heur:", len(lines), "행")


def tab_milp(tag):
	p = os.path.join(FIG, "toy_compare_v5_26_10_17_15.csv")
	if not os.path.exists(p):
		return
	rows = list(csv.DictReader(open(p, encoding="utf-8")))
	lines = []
	for M in sorted(set(int(r["dests"]) for r in rows)):
		sel = [r for r in rows if int(r["dests"]) == M]
		solved = [r for r in sel if r["replay"]]
		gaps = [float(r["milp_gap"]) for r in solved if r["milp_gap"] not in ("", "None")]
		secs = [float(r["milp_sec"]) for r in sel if r["milp_sec"] not in ("", "None")]
		mp = [float(r["replay"]) for r in solved]
		he = [float(r["heuristic"]) for r in sel if r["heuristic"]]
		lm = [float(r["learned_median"]) for r in sel if r["learned_median"]]
		lmin = [float(r["learned_min"]) for r in sel if r["learned_min"]]
		lmax = [float(r["learned_max"]) for r in sel if r["learned_max"]]
		comp = [r["learned_complete"] for r in sel]
		# 완주 분수 합
		num = sum(int(c.split("/")[0]) for c in comp); den = sum(int(c.split("/")[1]) for c in comp)
		milp_cell = (f"{np.mean(mp):.0f} ({len(solved)}/{len(sel)} solved; gap {100*np.mean(gaps):.0f}\\%; "
		             f"{np.mean(secs)/60:.0f}\\,min)" if solved else f"none ({len(sel)} tried; {np.mean(secs)/60:.0f}\\,min)")
		he_cell = f"{np.mean(he):.0f}" if he else "---"
		lm_cell = f"{np.mean(lm):.0f} ({np.min(lmin):.0f}--{np.max(lmax):.0f})" if lm else "---"
		# 휴리스틱·학습 대비 MILP 비율 (풀린 인스턴스만)
		if solved:
			rh = np.mean([float(r["heuristic"]) / float(r["replay"]) for r in solved if r["heuristic"]])
			rl = np.mean([float(r["learned_median"]) / float(r["replay"]) for r in solved if r["learned_median"]])
			ratio = f"{rh:.2f} / {rl:.2f}"
		else:
			ratio = "---"
		lines.append(f"{M} & {len(sel)} & {milp_cell} & {he_cell} & {lm_cell} & {num}/{den} & {ratio} \\\\")
	with open(os.path.join(TEX, f"tab_milp_v5_{tag}.tex"), "w") as f:
		f.write("\\begin{tabular}{rrlrrrr}\n\\toprule\n$M$ & inst. & MILP plan & heuristic & learned median (range) & learned complete & ratio h / l \\\\\n\\midrule\n"
		        + "\n".join(lines) + "\n\\bottomrule\n\\end{tabular}%\n")
	print("tab_milp:", len(lines), "행")


def tab_range(tag, base):
	"""base: R=300 쌍별 파일 dict {label: path}."""
	lines = []
	for lab, key, n in (("3/30", "330", 200), ("4/50", "450", 120), ("5/50", "550", 200)):
		cells = []
		for R in (250, 300, 350):
			p = base.get(lab) if R == 300 else os.path.join(FIG, f"paired_R{R}_{key}_SWa_v5_26_10_17_15.csv")
			if not p or not os.path.exists(p):
				cells.append("---"); continue
			a, b = paired(p); both = ~np.isnan(a) & ~np.isnan(b); d = b[both] - a[both]
			cells.append(f"{int((~np.isnan(a)).sum())}/{int((~np.isnan(b)).sum())} \\,/\\, ${np.median(d):+.0f}$ "
			             f"({(d<0).sum()}--{(d>0).sum()}, {pstr(stats.wilcoxon(d).pvalue)})")
		lines.append(f"{lab} & " + " & ".join(cells) + " \\\\")
	with open(os.path.join(TEX, f"tab_range_v5_{tag}.tex"), "w") as f:
		f.write("\\begin{tabular}{lrrr}\n\\toprule\n$N/M$ & $R=250$\\,m & $R=300$\\,m & $R=350$\\,m \\\\\n\\midrule\n"
		        + "\n".join(lines) + "\n\\bottomrule\n\\end{tabular}%\n")
	print("tab_range:", len(lines), "행")


def tab_paired(tag, files):
	lines = []
	for lab, p in files:
		if not os.path.exists(p):
			continue
		a, b = paired(p); both = ~np.isnan(a) & ~np.isnan(b); d = b[both] - a[both]
		lines.append(f"{lab} & {len(a)} & {int((~np.isnan(a)).sum())}/{int((~np.isnan(b)).sum())} & "
		             f"{np.nanmedian(a):,.0f} / {np.nanmedian(b):,.0f} & ${np.median(d):+.0f}$ & "
		             f"{(d<0).sum()}--{(d>0).sum()} & {pstr(stats.wilcoxon(d).pvalue)} \\\\")
	with open(os.path.join(TEX, f"tab_paired_v5_{tag}.tex"), "w") as f:
		f.write("\\begin{tabular}{lrrrrrr}\n\\toprule\n$N/M$ & rollouts & complete h/l & median heuristic / learned & paired median & win--loss & $p$ \\\\\n\\midrule\n"
		        + "\n".join(lines) + "\n\\bottomrule\n\\end{tabular}%\n")
	print("tab_paired:", len(lines), "행")


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--tag", default="26_10_17_15")
	p.add_argument("--suffix", default="", help="재측정 쌍별 CSV 접미 (예: v5_26_10_17_15)")
	A = p.parse_args()
	F = lambda n: os.path.join(FIG, n)
	if A.suffix:
		sz = [("3/20", F(f"paired_d3m20_SWa_{A.suffix}.csv")), ("3/30", F(f"paired_d3_SWa_{A.suffix}.csv")),
		      ("4/50 (standard)", F(f"paired_std_SWa_{A.suffix}.csv")), ("4/80", F(f"paired_d4m80_SWa_{A.suffix}.csv")),
		      ("5/50", F(f"paired_d5_SWa_{A.suffix}.csv")), ("6/80", F(f"paired_d6m80_SWa_{A.suffix}.csv")),
		      ("8/150", F(f"paired_d8m150_SWa_{A.suffix}.csv")), ("10/300", F(f"paired_d10m300_SWa_{A.suffix}.csv"))]
	else:
		sz = [("3/20", F("paired_size_d3_m20_v5_26_10_02_11.csv")), ("3/30", F("paired_d3_rep5_v5_26_09_27_11.csv")),
		      ("4/50 (standard)", F("paired_std480_SWa_v5_26_10_02_12.csv")), ("4/80", F("paired_size_d4_m80_v5_26_10_02_11.csv")),
		      ("5/50", F("paired_d5_rep5_v5_26_09_27_11.csv")), ("6/80", F("paired_size_d6_m80_v5_26_10_02_11.csv")),
		      ("8/150", F("paired_size_d8_m150_v5_26_10_02_11.csv")), ("10/300", F("paired_size_d10_m300_v5_26_10_02_11.csv"))]
	tab_heur(A.tag)
	tab_milp(A.tag)
	tab_range(A.tag, {"3/30": dict(sz)["3/30"], "4/50": dict(sz)["4/50 (standard)"], "5/50": dict(sz)["5/50"]})
	tab_paired(A.tag, sz)


if __name__ == "__main__":
	main()
