"""학습 정책 절제 표(tab:ablation): 제거 정책 대 제안 정책의 같은 인스턴스 쌍별 중앙 차이와 윌콕슨 p (시드 2개).

제안 정책 값은 paired_*_SWa_v5_26_10_17_15.csv 의 앞 reps (std 120, d3 80, d5 80), 절제 정책은
paired_abl_{A}s{1,2}_{cfg}_v5_26_10_19_22.csv (같은 시드·같은 반복 수).
"""

import csv
import os

import numpy as np
from scipy import stats

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FIG = os.path.join(ROOT, "figures"); TEX = os.path.join(ROOT, "docs", "tex")
REF = {"std": ("paired_std_SWa_v5_26_10_17_15.csv", 120), "d3": ("paired_d3_SWa_v5_26_10_17_15.csv", 80), "d5": ("paired_d5_SWa_v5_26_10_17_15.csv", 80)}
NAMES = [("A2", "Rule regularisation", "규칙 정규화"), ("A4", "Configuration randomisation", "구성 무작위화"),
         ("A3", "Entropy target annealing", "엔트로피 목표 감쇠"), ("A1", "Autoregressive decisions", "자기회귀 결정"),
         ("A5", "Residual penalty", "잔여 벌점")]


def load(p):
	r = list(csv.DictReader(open(p, encoding="utf-8")))
	return np.array([float(x["학습"]) if x["학습"] else np.nan for x in r])


def pstr(p):
	return "$<10^{-4}$" if p < 1e-4 else f"{p:.3f}" if p >= 0.001 else f"{p:.1e}"


def main():
	for lang in ("en", "ko"):
		lines = []
		for key, en, ko in NAMES:
			cells = []
			for cfg in ("std", "d3", "d5"):
				rb = load(os.path.join(FIG, REF[cfg][0]))[:REF[cfg][1]]
				sub = []
				for sd in (1, 2):
					b = load(os.path.join(FIG, f"paired_abl_{key}s{sd}_{cfg}_v5_26_10_19_22.csv"))
					d = b - rb; d = d[~np.isnan(d)]
					p = stats.wilcoxon(d).pvalue if len(d) > 5 and np.any(d != 0) else 1.0
					sub.append(f"${np.median(d):+.0f}$ ({pstr(p)})")
				cells.append(" / ".join(sub))
			lines.append(f"{en if lang == 'en' else ko} & " + " & ".join(cells) + " \\\\")
		head = ("Removed component & $4/50$ (120 runs) & $3/30$ (80) & $5/50$ (80) \\\\" if lang == "en"
		        else "제거한 요소 & $4/50$ (120회) & $3/30$ (80회) & $5/50$ (80회) \\\\")
		with open(os.path.join(TEX, f"tab_ablation_v5_{lang}_26_10_19_22.tex"), "w") as f:
			f.write("\\begin{tabular}{lrrr}\n\\toprule\n" + head + "\n\\midrule\n" + "\n".join(lines) + "\n\\bottomrule\n\\end{tabular}%\n")
		print(lang, len(lines), "행")


if __name__ == "__main__":
	main()
