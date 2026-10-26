"""연속 공간 표: 규칙 / 선견 규칙 / 학습 정책을 쌍별 평가 인스턴스(3/30, 4/50, 5/50)에서 비교한다 (사이클 30).

선견 규칙은 결정론, 학습 정책은 인스턴스별 표본 중앙값(3/30·5/50 은 5회, 4/50 은 8회). 셀은 평균 makespan,
괄호는 완주 수. 승패와 Wilcoxon p 는 세 방법이 모두 완주한 인스턴스 쌍에서 센다.
"""

import csv
import glob
import os

import numpy as np
from scipy import stats

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FIG = os.path.join(ROOT, "figures"); TEX = os.path.join(ROOT, "docs", "tex")
CFG = [("3/30", "paired_d3_SWa_v5_26_10_17_15.csv", "rule2_cont_d3_s*_v5_26_10_26_20.csv"),
       ("4/50", "paired_std_SWa_v5_26_10_17_15.csv", "rule2_cont_std_s*_v5_26_10_26_20.csv"),
       ("5/50", "paired_d5_SWa_v5_26_10_17_15.csv", "rule2_cont_d5_s*_v5_26_10_26_20.csv")]


def pstr(p):
	return "$<10^{-4}$" if p < 1e-4 else f"{p:.3f}" if p >= 0.001 else f"{p:.1e}"


def load(cfg):
	"""인스턴스(시드)별 규칙·선견·학습 중앙값과 완주 수."""
	P = list(csv.DictReader(open(os.path.join(FIG, cfg[1]), encoding="utf-8")))
	by = {}
	for r in P:
		s = int(r["seed"]); b = by.setdefault(s, dict(rule=[], learned=[], n=0)); b["n"] += 1
		if r["규칙+탈출"]:
			b["rule"].append(float(r["규칙+탈출"]))
		if r["학습"]:
			b["learned"].append(float(r["학습"]))
	L = [x for f in sorted(glob.glob(os.path.join(FIG, cfg[2]))) for x in csv.DictReader(open(f, encoding="utf-8"))]
	out = []
	for r in L:
		s = int(r["seed"]); b = by[s]
		out.append(dict(seed=s, rule=float(r["rule"]) if r["rule"] else np.nan, look=float(r["la"]) if r["la"] else np.nan,
		                lsec=float(r["la_sec"]), learned=np.median(b["learned"]) if b["learned"] else np.nan,
		                lcomp=(len(b["learned"]), b["n"])))
	return out


def main():
	res = {}
	for cfg in CFG:
		d = load(cfg)
		r = np.array([x["rule"] for x in d]); l = np.array([x["look"] for x in d]); g = np.array([x["learned"] for x in d])
		m = ~np.isnan(r) & ~np.isnan(l) & ~np.isnan(g)
		res[cfg[0]] = dict(n=len(d), rule=np.nanmean(r), look=np.nanmean(l), learned=np.nanmean(g),
		                   rc=int(np.sum(~np.isnan(r))), lc=int(np.sum(~np.isnan(l))),
		                   gc=f"{sum(x['lcomp'][0] for x in d)}/{sum(x['lcomp'][1] for x in d)}",
		                   wl_r=(int(np.sum(l[m] < r[m])), int(np.sum(l[m] > r[m]))), p_r=stats.wilcoxon(l[m], r[m]).pvalue,
		                   wl_g=(int(np.sum(l[m] < g[m])), int(np.sum(l[m] > g[m]))), p_g=stats.wilcoxon(l[m], g[m]).pvalue,
		                   sec=np.mean([x["lsec"] for x in d]), npair=int(m.sum()))
		x = res[cfg[0]]
		print(f"{cfg[0]}: n{x['n']} 규칙 {x['rule']:.0f} ({x['rc']}) 선견 {x['look']:.0f} ({x['lc']}) 학습 {x['learned']:.0f} ({x['gc']}) | "
		      f"선견-규칙 {x['wl_r']} p={pstr(x['p_r'])} 선견-학습 {x['wl_g']} p={pstr(x['p_g'])} | {x['sec']:.0f}초/건")
	for lang in ("en", "ko"):
		if lang == "en":
			head = ("config. & inst. & rule & lookahead & learned & \\multicolumn{2}{c}{vs rule} & \\multicolumn{2}{c}{vs learned} & time \\\\\n"
			        "\\cmidrule(lr){6-7}\\cmidrule(lr){8-9}\n & & & & & W--L & $p$ & W--L & $p$ & s/inst. \\\\")
		else:
			head = ("구성 & 건수 & 규칙 & 선견 & 학습 & \\multicolumn{2}{c}{대 규칙} & \\multicolumn{2}{c}{대 학습} & 시간 \\\\\n"
			        "\\cmidrule(lr){6-7}\\cmidrule(lr){8-9}\n & & & & & 승--패 & $p$ & 승--패 & $p$ & 초/건 \\\\")
		lines = []
		for k, x in res.items():
			lines.append(f"{k} & {x['n']} & {x['rule']:.0f} ({x['rc']}/{x['n']}) & {x['look']:.0f} ({x['lc']}/{x['n']}) & {x['learned']:.0f} ({x['gc']}) & "
			             f"{x['wl_r'][0]}--{x['wl_r'][1]} & {pstr(x['p_r'])} & {x['wl_g'][0]}--{x['wl_g'][1]} & {pstr(x['p_g'])} & {x['sec']:.0f} \\\\")
		with open(os.path.join(TEX, f"tab_look_v5_{lang}_26_10_26_20.tex"), "w") as f:
			f.write("\\begin{tabular}{lrrrrrrrrr}\n\\toprule\n" + head + "\n\\midrule\n" + "\n".join(lines) + "\n\\bottomrule\n\\end{tabular}%\n")
	print("저장: tab_look_v5_{en,ko}_26_10_26_20.tex")


if __name__ == "__main__":
	main()
