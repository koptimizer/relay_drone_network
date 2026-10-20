"""편차 안정화 표(tab:stab): 제안 정책에 교착 복구 덮어쓰기를 추론 시점에 씌운 효과.

재학습 없이 규칙의 복구 단계를 학습 정책의 행동에 덮어씌운 것과 그렇지 않은 것을 같은
구성에서 비교한다. 쌍별 중앙 차이는 규칙 대비이고, 분포 통계(표준편차·90분위·최대)는
학습 정책의 makespan 자체에 대한 것이다.
"""

import csv
import os

import numpy as np
from scipy import stats

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FIG = os.path.join(ROOT, "figures"); TEX = os.path.join(ROOT, "docs", "tex")
P = [("$3/30$", "paired_d3_SWa_v5_26_10_17_15.csv", "paired_esc_d3_SWa_v5_26_10_19_21.csv"),
     ("$4/50$", "paired_std_SWa_v5_26_10_17_15.csv", "paired_esc_std_SWa_v5_26_10_20_15.csv"),
     ("$5/50$", "paired_d5_SWa_v5_26_10_17_15.csv", "paired_esc_d5_SWa_v5_26_10_20_15.csv")]
H = {"en": ("config. & inference & runs & paired median & wins--losses & $p$ & s.d. & 90th pct. & max & $>$1{,}500 \\\\",
            "proposed", "+ recovery"),
     "ko": ("구성 & 추론 & 실행 & 쌍별 중앙 & 승--패 & $p$ & 표준편차 & 90분위 & 최대 & $>$1{,}500 \\\\",
            "제안", "+ 복구")}


def load(p):
	r = list(csv.DictReader(open(p, encoding="utf-8")))
	a = np.array([float(x["규칙+탈출"]) if x["규칙+탈출"] else np.nan for x in r])
	b = np.array([float(x["학습"]) if x["학습"] else np.nan for x in r])
	return a, b


def pstr(p):
	return "$<10^{-4}$" if p < 1e-4 else f"{p:.3f}" if p >= 0.001 else f"{p:.1e}"


def main():
	for lang in ("en", "ko"):
		head, n0, n1 = H[lang]
		lines = []
		for lab, f0, f1 in P:
			for nm, f in ((n0, f0), (n1, f1)):
				a, b = load(os.path.join(FIG, f)); d = b - a; d = d[~np.isnan(d)]
				p = stats.wilcoxon(d).pvalue if np.any(d != 0) else 1.0
				lines.append(f"{lab if nm == n0 else ''} & {nm} & {len(b)} & ${np.median(d):+.0f}$ & "
				             f"{int((d<0).sum())}--{int((d>0).sum())} & {pstr(p)} & {np.nanstd(b):.0f} & "
				             f"{np.nanpercentile(b,90):.0f} & {np.nanmax(b):.0f} & {int(np.nansum(b>1500))} \\\\")
			lines.append("\\addlinespace")
		with open(os.path.join(TEX, f"tab_stab_v5_{lang}_26_10_20_15.tex"), "w") as f:
			f.write("\\begin{tabular}{llrrrrrrrr}\n\\toprule\n" + head + "\n\\midrule\n"
			        + "\n".join(lines[:-1]) + "\n\\bottomrule\n\\end{tabular}%\n")
		print(lang, len(lines) - 1, "행")


if __name__ == "__main__":
	main()
