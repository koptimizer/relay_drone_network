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
LANG = "en"
H = {
	"en": dict(heur="mode & $k_{\\mathrm{stall}}$ / $h_{\\mathrm{hold}}$ & $3/30$ & $4/50$ & $5/50$ & completions",
	           milp="$M$ & inst. & plan / replayed / optimal & gap & min & MILP replay & heuristic & learned median (range) & ratio h / l",
	           range="$N/M$ & $R=250$\\,m & $R=300$\\,m & $R=350$\\,m",
	           paired="$N/M$ & rollouts & complete h/l & median heuristic / learned & paired median & win--loss & $p$",
	           modes={"none": "none", "shuffle": "lead swap", "retreat": "retreat"},
	           solved="solved", none="none", tried="tried",
	           gen="$N/M$ & trained on $N\\in\\{3,4\\}$, $M\\in[20,80]$ & trained on $4/50$ only, seed 1 & seed 2"),
	"ko": dict(heur="방식 & $k_{\\mathrm{stall}}$ / $h_{\\mathrm{hold}}$ & $3/30$ & $4/50$ & $5/50$ & 완주 합",
	           milp="$M$ & 인스턴스 & 계획 / 재생 / 최적 & 갭 & 분 & MILP 재생 & 휴리스틱 & 학습 중앙값 (범위) & 비율 h / l",
	           range="$N/M$ & $R=250$\\,m & $R=300$\\,m & $R=350$\\,m",
	           paired="$N/M$ & 롤아웃 & 완주 h/l & 중앙값 휴리스틱 / 학습 & 쌍별 중앙 & 승--패 & $p$",
	           modes={"none": "없음", "shuffle": "선두 교대", "retreat": "후퇴"},
	           solved="해 있음", none="해 없음", tried="시도",
	           gen="$N/M$ & $N\\in\\{3,4\\}$, $M\\in[20,80]$ 학습 & $4/50$만 학습, 시드 1 & 시드 2"),
}


def suffix(tag):
	return f"{tag}" if LANG == "en" else f"ko_{tag}"


def paired(path):
	r = list(csv.DictReader(open(path, encoding="utf-8")))
	a = np.array([float(x["규칙+탈출"]) if x["규칙+탈출"] else np.nan for x in r])
	b = np.array([float(x["학습"]) if x["학습"] else np.nan for x in r])
	return a, b


def pstr(p):
	return "$<10^{-4}$" if p < 1e-4 else f"{p:.3f}" if p >= 0.001 else f"{p:.1e}"


def tab_heur(tag):
	"""휴리스틱 파라미터 격자: 완주/중앙값과, 확정 설정(선두 교대 120/20) 대비 같은 시드의 쌍별 윌콕슨 p."""
	rows, seeds = [], []
	for f in sorted(glob.glob(os.path.join(FIG, "heur_ablate_v5_26_10_19_22_*[!s].csv"))):
		rows += list(csv.DictReader(open(f, encoding="utf-8")))
	for f in sorted(glob.glob(os.path.join(FIG, "heur_ablate_v5_26_10_19_22_*_seeds.csv"))):
		seeds += list(csv.DictReader(open(f, encoding="utf-8")))
	if not rows:
		return
	cfgs = ["d3m30r", "d4m50f", "d5m50r"]
	SEL = ("shuffle", 120, 20)
	def ms(c, k):
		return {int(r["seed"]): (float(r["makespan"]) if r["makespan"] else np.nan) for r in seeds
		        if r["config"] == c and (r["mode"], int(r["k_stall"]), int(r["hold"])) == k}
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
			ptxt = ""
			if k != SEL and seeds:
				x, y = ms(c, k), ms(c, SEL)
				d = np.array([x[sd] - y[sd] for sd in x if sd in y and not np.isnan(x[sd]) and not np.isnan(y[sd])])
				if len(d) > 5 and np.any(d != 0):
					ptxt = f" ({pstr(stats.wilcoxon(d).pvalue)})"
			cells.append(f"{r['complete']}/{r['n']} \\,/\\, {r['median'] or '---'}{ptxt}")
		mode = H[LANG]["modes"][k[0]]
		par = "---" if k[0] == "none" else f"{k[1]} / {k[2]}"
		line = f"{mode} & {par} & " + " & ".join(cells) + f" & {cs}/120 \\\\"
		if k == SEL:
			line = "\\textbf{" + mode + "} & \\textbf{" + par + "} & " + " & ".join(cells) + f" & \\textbf{{{cs}/120}} \\\\"
		lines.append(line)
	with open(os.path.join(TEX, f"tab_heur_v5_{suffix(tag)}.tex"), "w") as f:
		f.write("\\begin{tabular}{llrrrr}\n\\toprule\n" + H[LANG]["heur"] + " \\\\\n\\midrule\n"
		        + "\n".join(lines) + "\n\\bottomrule\n\\end{tabular}%\n")
	print("tab_heur:", len(lines), "행")


def tab_milp(tag):
	"""M별 요약: 계획 수/재생 성공 수, 재생 MILP 평균, 휴리스틱·학습 평균과 MILP 대비 비율, 풀이 시간."""
	p = os.path.join(FIG, "toy_compare_v5_26_10_17_15.csv")
	if not os.path.exists(p):
		return
	rows = list(csv.DictReader(open(p, encoding="utf-8")))
	h = H[LANG]
	lines = []
	for M in sorted(set(int(r["dests"]) for r in rows)):
		sel = [r for r in rows if int(r["dests"]) == M]
		plan = [r for r in sel if r["milp"] not in ("", "None")]
		rep = [r for r in plan if r["replay"]]
		opt = [r for r in plan if r["milp_gap"] not in ("", "None") and float(r["milp_gap"]) == 0.0]
		secs = [float(r["milp_sec"]) for r in sel if r["milp_sec"] not in ("", "None")]
		he = [float(r["heuristic"]) for r in sel]
		lm = [float(r["learned_median"]) for r in sel]
		lmin = min(float(r["learned_min"]) for r in sel); lmax = max(float(r["learned_max"]) for r in sel)
		if rep:
			mp = np.mean([float(r["replay"]) for r in rep])
			rh = np.mean([float(r["heuristic"]) / float(r["replay"]) for r in rep])
			rl = np.mean([float(r["learned_median"]) / float(r["replay"]) for r in rep])
			milp_cell = f"{mp:.0f}"
			ratio = f"{rh:.2f} / {rl:.2f}"
		else:
			milp_cell, ratio = "---", "---"
		gaps = [float(r["milp_gap"]) for r in plan if r["milp_gap"] not in ("", "None")]
		gap_cell = f"{100*np.mean(gaps):.0f}\\%" if gaps else "---"
		lines.append(f"{M} & {len(sel)} & {len(plan)} / {len(rep)} / {len(opt)} & {gap_cell} & {np.mean(secs)/60:.0f} & "
		             f"{milp_cell} & {np.mean(he):.0f} & {np.mean(lm):.0f} ({lmin:.0f}--{lmax:.0f}) & {ratio} \\\\")
	with open(os.path.join(TEX, f"tab_milp_v5_{suffix(tag)}.tex"), "w") as f:
		f.write("\\begin{tabular}{rrcrrrrrr}\n\\toprule\n" + h["milp"] + " \\\\\n\\midrule\n"
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
	with open(os.path.join(TEX, f"tab_range_v5_{suffix(tag)}.tex"), "w") as f:
		f.write("\\begin{tabular}{lrrr}\n\\toprule\n" + H[LANG]["range"] + " \\\\\n\\midrule\n"
		        + "\n".join(lines) + "\n\\bottomrule\n\\end{tabular}%\n")
	print("tab_range:", len(lines), "행")


def tab_gen(tag):
	"""일반화: 3-4대로 학습한 SWa 대 4/50 고정으로만 학습한 A4 (두 시드)의 큰 편대 영 샷."""
	h = H[LANG]
	cfgs = [("5/50", "d5", os.path.join(FIG, "paired_d5_SWa_v5_26_10_17_15.csv")),
	        ("6/80", "d6m80", os.path.join(FIG, "paired_d6m80_SWa_v5_26_10_17_15.csv")),
	        ("8/150", "d8m150", os.path.join(FIG, "paired_d8m150_SWa_v5_26_10_17_15.csv")),
	        ("10/300", "d10m300", os.path.join(FIG, "paired_d10m300_SWa_v5_26_10_17_15.csv"))]
	lines = []
	for lab, key, swa in cfgs:
		if not os.path.exists(swa):
			continue
		a, b = paired(swa); both = ~np.isnan(a) & ~np.isnan(b); d = b[both] - a[both]
		cells = [f"{int((~np.isnan(b)).sum())}/{len(b)} \\,/\\, ${np.median(d):+.0f}$ ({int((d<0).sum())}--{int((d>0).sum())}, {pstr(stats.wilcoxon(d).pvalue)})"]
		for sd in (1, 2):
			f = os.path.join(FIG, f"paired_gen_{key}_A4s{sd}_v5_26_10_18_06.csv")
			if not os.path.exists(f):
				cells.append("---"); continue
			a2, b2 = paired(f); both2 = ~np.isnan(a2) & ~np.isnan(b2); d2 = b2[both2] - a2[both2]
			cells.append(f"{int((~np.isnan(b2)).sum())}/{len(b2)} \\,/\\, ${np.median(d2):+.0f}$ ({int((d2<0).sum())}--{int((d2>0).sum())}, {pstr(stats.wilcoxon(d2).pvalue)})" if both2.sum() > 5 else f"{int((~np.isnan(b2)).sum())}/{len(b2)} \\,/\\, ---")
		lines.append(f"{lab} & " + " & ".join(cells) + " \\\\")
	with open(os.path.join(TEX, f"tab_gen_v5_{suffix(tag)}.tex"), "w") as f:
		f.write("\\begin{tabular}{lrrr}\n\\toprule\n" + h["gen"] + " \\\\\n\\midrule\n"
		        + "\n".join(lines) + "\n\\bottomrule\n\\end{tabular}%\n")
	print("tab_gen:", len(lines), "행")


def tab_paired(tag, files):
	lines = []
	for lab, p in files:
		if not os.path.exists(p):
			continue
		a, b = paired(p); both = ~np.isnan(a) & ~np.isnan(b); d = b[both] - a[both]
		lines.append(f"{lab} & {len(a)} & {int((~np.isnan(a)).sum())}/{int((~np.isnan(b)).sum())} & "
		             f"{np.nanmedian(a):,.0f} / {np.nanmedian(b):,.0f} & ${np.median(d):+.0f}$ & "
		             f"{(d<0).sum()}--{(d>0).sum()} & {pstr(stats.wilcoxon(d).pvalue)} \\\\")
	with open(os.path.join(TEX, f"tab_paired_v5_{suffix(tag)}.tex"), "w") as f:
		f.write("\\begin{tabular}{lrrrrrr}\n\\toprule\n" + H[LANG]["paired"] + " \\\\\n\\midrule\n"
		        + "\n".join(lines) + "\n\\bottomrule\n\\end{tabular}%\n")
	print("tab_paired:", len(lines), "행")


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--tag", default="26_10_17_15")
	p.add_argument("--suffix", default="", help="재측정 쌍별 CSV 접미 (예: v5_26_10_17_15)")
	p.add_argument("--lang", default="en", choices=["en", "ko"], help="표 머리글 언어 (ko면 tab_*_ko_*.tex)")
	A = p.parse_args()
	global LANG
	LANG = A.lang
	F = lambda n: os.path.join(FIG, n)
	if A.suffix:
		sz = [("3/20", F(f"paired_d3m20_SWa_{A.suffix}.csv")), ("3/30", F(f"paired_d3_SWa_{A.suffix}.csv")),
		      ("4/50 (standard)" if A.lang == "en" else "4/50 (표준)", F(f"paired_std_SWa_{A.suffix}.csv")), ("4/80", F(f"paired_d4m80_SWa_{A.suffix}.csv")),
		      ("5/50", F(f"paired_d5_SWa_{A.suffix}.csv")), ("6/80", F(f"paired_d6m80_SWa_{A.suffix}.csv")),
		      ("8/150", F(f"paired_d8m150_SWa_{A.suffix}.csv")), ("10/300", F(f"paired_d10m300_SWa_{A.suffix}.csv")),
		      ("12/400", F("paired_d12m400_SWa_v5_26_10_18_06.csv"))]
	else:
		sz = [("3/20", F("paired_size_d3_m20_v5_26_10_02_11.csv")), ("3/30", F("paired_d3_rep5_v5_26_09_27_11.csv")),
		      ("4/50 (standard)", F("paired_std480_SWa_v5_26_10_02_12.csv")), ("4/80", F("paired_size_d4_m80_v5_26_10_02_11.csv")),
		      ("5/50", F("paired_d5_rep5_v5_26_09_27_11.csv")), ("6/80", F("paired_size_d6_m80_v5_26_10_02_11.csv")),
		      ("8/150", F("paired_size_d8_m150_v5_26_10_02_11.csv")), ("10/300", F("paired_size_d10_m300_v5_26_10_02_11.csv"))]
	tab_heur(A.tag)
	tab_milp(A.tag)
	d = dict(sz)
	tab_range(A.tag, {"3/30": d["3/30"], "4/50": d.get("4/50 (standard)", d.get("4/50 (표준)")), "5/50": d["5/50"]})
	tab_paired(A.tag, sz)
	tab_gen(A.tag)


if __name__ == "__main__":
	main()
