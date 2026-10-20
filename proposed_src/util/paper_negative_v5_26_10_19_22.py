"""음성 결과 표(tab:negative)를 쌍별 CSV 에서 다시 만든다: 시드별 쌍별 중앙값 범위와 유의한 시드 수.

각 변형의 실행 이름과 구성(d3=3/30, d5=5/50, std=4/50)마다 행 수가 가장 많은 CSV 를 쓴다.
셀 = "중앙값 범위 (유의 시드 수/시드 수)", 유의 = 윌콕슨 p<0.05.
"""

import csv
import glob
import os

import numpy as np
from scipy import stats

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FIG = os.path.join(ROOT, "figures"); TEX = os.path.join(ROOT, "docs", "tex")
V = [("V1", ["RR3a", "RR3b"], "Relay-weighted rule regularisation ($\\times 3$)", "중계 가중 규칙 정규화 ($\\times 3$)", "relay-specific regularisation harms", "중계 특화 정규화는 해로움"),
     ("V2", ["PD2a", "PD2b", "PD6a", "PD6b"], "Per-drone relay credit (2.0 / 6.0)", "드론별 중계 크레딧 (2.0 / 6.0)", "relaying becomes the goal", "중계가 목적이 됨"),
     ("V3", [f"MH{i}" for i in range(1, 9)], "Checkpoint selection half on $3/30$", "체크포인트 선택 절반을 $3/30$으로", "improves only the standard config.", "표준 구성만 개선"),
     ("V4", ["RS2a", "RS2b"], "Zero-sum credit redistribution", "제로섬 크레딧 재분배", "same, larger", "같은 기제, 더 큼"),
     ("V5", ["JC1", "JC2", "JC3", "JC4"], "Joint-action critic", "결합 행동 critic", "target variance from sampled teammates", "표본된 동료 행동의 목표값 분산"),
     ("V6", ["NR1", "NR2"], "Nearest-first decision order", "가까운 드론부터 결정", "order is not the cause", "순서는 원인이 아님"),
     ("V7", ["SR1", "SR2"], "Re-decision after 60 stalled steps", "정체 60스텝 뒤 재결정", "irregular option lengths", "불규칙한 옵션 길이"),
     ("V8", ["MX1", "MX2", "MX3", "MX4"], "Monotone mixing critic (floor 0.5)", "단조 혼합 critic (하한 0.5)", "divergence vs.\\ credit on one axis", "발산 방지와 크레딧이 한 축"),
     ("V9", ["SQ1", "SQ2", "SQ3", "SQ4"], "Sequential single-agent credit", "순차 단일 에이전트 크레딧", "no contraction within a decision", "결정 안에 수축 없음"),
     ("V10", ["BC1a", "BC1w", "BC2a", "BC2w"], "Behaviour-cloning initialisation ($\\pm$ warm-up)", "행동 복제 초기화 (워밍업 $\\pm$)", "erased within 400 episodes", "400 에피소드 안에 지워짐"),
     ("V11", ["BN1", "BN2", "BN3", "BN4"], "Bottleneck-arrival penalty (0.15 / 0.25)", "병목 도달 시각 벌점 (0.15 / 0.25)", "reduces the mean, not the maximum", "최댓값이 아닌 평균을 줄임")]
def ref_csv(cfg):
	"""기준 SWa 의 튜닝 전 규칙(60/40) 측정: 26_10_17_15 재측정을 제외하고 행 수가 가장 많은 파일."""
	pat = {"d3": "paired_d3_", "d5": "paired_d5_", "std": "paired_std"}[cfg]
	fs = [f for f in glob.glob(os.path.join(FIG, "paired_*SWa*.csv")) if os.path.basename(f).startswith(pat)
	      and "R250" not in f and "R350" not in f and "esc" not in f and "sl_" not in f]
	old = [f for f in fs if "26_10_17_15" not in f]
	fs = sorted(old if old else fs, key=lambda f: (sum(1 for _ in open(f)), os.path.getmtime(f)))
	return fs[-1]


REF = {cfg: ref_csv(cfg) for cfg in ("d3", "d5", "std")}


def load(p):
	r = list(csv.DictReader(open(p, encoding="utf-8")))
	a = np.array([float(x["규칙+탈출"]) if x["규칙+탈출"] else np.nan for x in r])
	b = np.array([float(x["학습"]) if x["학습"] else np.nan for x in r])
	return a, b


def stat(p):
	a, b = load(p); d = b - a; d = d[~np.isnan(d)]
	pv = stats.wilcoxon(d).pvalue if len(d) > 5 and np.any(d != 0) else 1.0
	return float(np.median(d)), pv, len(d)


def best_csv(cfg, run):
	fs = set(glob.glob(os.path.join(FIG, f"paired_{cfg}_{run}_v5_*.csv")))
	fs = sorted(fs, key=lambda f: (sum(1 for _ in open(f)), os.path.getmtime(f)))
	return fs[-1] if fs else None


def cell(meds, sig):
	if not meds:
		return "---"
	lo, hi = min(meds), max(meds)
	rng = f"${lo:+.0f}$" if len(meds) == 1 else (f"${lo:+.0f}$, ${hi:+.0f}$" if len(meds) == 2 else f"${lo:+.0f}$ to ${hi:+.0f}$")
	return f"{rng} ({sig}/{len(meds)})"


def main():
	print("기준 CSV:", {k: os.path.basename(v) for k, v in REF.items()})
	for lang in ("en", "ko"):
		lines = []
		ref = []
		for cfg in ("d3", "d5", "std"):
			m, pv, n = stat(REF[cfg]); ref.append(f"${m:+.0f}$ ({'1' if pv < 0.05 else '0'}/1)")
		lines.append(("--- & Proposed configuration (reference) & " if lang == "en" else "--- & 제안 학습 구성 (기준) & ") + " & ".join(ref) + " & --- \\\\")
		for vid, runs, en, ko, men, mko in V:
			cells = []
			for cfg in ("d3", "d5", "std"):
				meds, sig = [], 0
				for r in runs:
					f = best_csv(cfg, r)
					if f is None:
						continue
					m, pv, n = stat(f); meds.append(m); sig += int(pv < 0.05)
				cells.append(cell(meds, sig))
			lines.append(f"{vid} & {en if lang == 'en' else ko} & " + " & ".join(cells) + f" & {men if lang == 'en' else mko} \\\\")
		head = ("\\# & Intervention (what changed) & $3/30$ & $5/50$ & $4/50$ & Mechanism of failure \\\\" if lang == "en"
		        else "\\# & 개입 (무엇을 바꿨나) & $3/30$ & $5/50$ & $4/50$ & 실패 기제 \\\\")
		with open(os.path.join(TEX, f"tab_negative_v5_{lang}_26_10_19_22.tex"), "w") as f:
			f.write("\\begin{tabular}{llrrrl}\n\\toprule\n" + head + "\n\\midrule\n" + "\n".join(lines) + "\n\\bottomrule\n\\end{tabular}%\n")
		print(lang, len(lines), "행")


if __name__ == "__main__":
	main()
