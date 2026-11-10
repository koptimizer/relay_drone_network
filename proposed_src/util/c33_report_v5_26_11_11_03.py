"""사이클 33 보고: 증류 정책들을 SWa·규칙과 쌍별로, 그리고 수리모형 최적값과 격자·연속 두 공간에서 비교한다.

표 1 (쌍별, 인스턴스별 중앙값 평균; SWa 대비 승–패 / 중앙 차 / p): 3/30, 5/50, 4/50 + 전이 6구성.
표 2 (수리모형 대비): 격자 결정 공간의 정확해 55건(최적 대비 중앙/평균 격차, 30회 중앙값)과 같은 40건의 연속 공간
측정(중계 격자 스냅, 최적 대비 평균 격차·완주) — 규칙·선견 규칙·빔 탐색·SWa 와 같은 줄에 둔다.
"""

import argparse
import csv
import glob
import os

import numpy as np
from scipy.stats import wilcoxon

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FIG = os.path.join(ROOT, "figures")
T = "v5_26_11_09_17"
MAIN = {"3/30": ("d3", "paired_d3_SWa_v5_26_10_17_15.csv", "paired_c32"), "5/50": ("d5", "paired_d5_SWa_v5_26_10_17_15.csv", "paired_c32"),
        "4/50": ("std", "paired_std_SWa_v5_26_10_17_15.csv", "paired_c32")}
XFER = {"3/20": ("d3m20", "paired_d3m20_SWa_v5_26_10_17_15.csv"), "4/80": ("d4m80", "paired_d4m80_SWa_v5_26_10_17_15.csv"),
        "6/80": ("d6m80", "paired_d6m80_SWa_v5_26_10_17_15.csv"), "8/150": ("d8m150", "paired_d8m150_SWa_v5_26_10_17_15.csv"),
        "10/300": ("d10m300", "paired_d10m300_SWa_v5_26_10_17_15.csv"), "12/400": ("d12m400", "paired_d12m400_SWa_v5_26_10_18_06.csv")}


def med(path, col="학습"):
	by, rule = {}, {}
	for r in csv.DictReader(open(path, encoding="utf-8")):
		s = int(r["seed"])
		if r.get(col):
			by.setdefault(s, []).append(float(r[col]))
		if r.get("규칙+탈출"):
			rule.setdefault(s, []).append(float(r["규칙+탈출"]))
	return {s: float(np.median(v)) for s, v in by.items()}, {s: float(np.median(v)) for s, v in rule.items()}


def cell(x, y):
	ks = [s for s in x if s in y]
	a = np.array([x[s] for s in ks]); b = np.array([y[s] for s in ks])
	p = wilcoxon(a, b).pvalue if np.any(a != b) else 1.0
	return f"{a.mean():.0f} ({int((a < b).sum())}-{int((a > b).sum())} / {np.median(a - b):+.0f} / {p:.2g})"


def lattice(run):
	opt = {}
	for f in ("lattice_eval3_m36_v5_26_10_27_00.csv", "lattice_eval3_m78_v5_26_10_27_00.csv"):
		for r in csv.DictReader(open(os.path.join(FIG, f))):
			opt[(r["dests"], r["seed"])] = float(r["opt"])
	L = {}
	for f in glob.glob(os.path.join(FIG, f"lattice_c32_{run}_m*_{T}.csv")):
		for r in csv.DictReader(open(f)):
			L[(r["dests"], r["seed"])] = r
	g = [(float(L[k]["learned_median"]) / opt[k] - 1) * 100 for k in L if L[k]["learned_median"]]
	g36 = [(float(L[k]["learned_median"]) / opt[k] - 1) * 100 for k in L if L[k]["learned_median"] and int(k[0]) <= 6]
	return (f"{np.median(g36):+.1f}% / {np.mean(g36):+.1f}%" if g36 else "---", f"{np.mean(g):+.1f}%" if g else "---")


def toy(run):
	f = os.path.join(FIG, f"toy_c33_{run}_{T}.csv")
	if not os.path.exists(f):
		return "---"
	R = list(csv.DictReader(open(f)))
	g = [(float(r["learned_median"]) / float(r["milp"]) - 1) * 100 for r in R if r["learned_median"]]
	comp = sum(int(r["learned_complete"].split("/")[0]) for r in R)
	return f"{np.mean(g):+.1f}% (완주 {comp}/{30 * len(R)})"


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--runs", default="DA2,DA3,DA4,DB1,DB2,DB3")
	A = p.parse_args()
	runs = A.runs.split(",")
	print("## 표 1 쌍별 (인스턴스별 중앙 makespan 평균; 괄호 = SWa 대비 승–패 / 중앙 차 / p)")
	for name, cfgs in (("주 구성", MAIN), ("전이 구성", XFER)):
		print(f"### {name}")
		hdr = "| 구성 | SWa | 규칙 | " + " | ".join(runs) + " |"
		print(hdr); print("|" + "---|" * (3 + len(runs)))
		for cfg, spec in cfgs.items():
			key, ref = spec[0], spec[1]
			sw, ru = med(os.path.join(FIG, ref))
			row = [cfg, f"{np.mean(list(sw.values())):.0f}", f"{np.mean(list(ru.values())):.0f}"]
			for run in runs:
				pre = "paired_c32" if cfg in MAIN else "paired_c32x"
				f = os.path.join(FIG, f"{pre}_{run}_{key}_{T}.csv")
				row.append(cell(med(f)[0], sw) if os.path.exists(f) else "---")
			print("| " + " | ".join(row) + " |")
	print("### 복구 덮어쓰기 (추론 시) 적용")
	for cfg, key, ref in (("5/50", "d5", "paired_d5_SWa_v5_26_10_17_15.csv"), ("4/80", "d4m80", "paired_d4m80_SWa_v5_26_10_17_15.csv"), ("6/80", "d6m80", "paired_d6m80_SWa_v5_26_10_17_15.csv")):
		sw, _ = med(os.path.join(FIG, ref))
		row = [cfg, f"{np.mean(list(sw.values())):.0f}"]
		for run in runs:
			f = os.path.join(FIG, f"paired_c32e_{run}_{key}_{T}.csv")
			row.append(cell(med(f)[0], sw) if os.path.exists(f) else "---")
		print("| " + " | ".join(row) + " |")
	print("\n## 표 2 수리모형 최적값 대비 (정확해 인스턴스, 드론 3)")
	print("| 방법 | 격자 결정 공간 M3–6 중앙 / 평균 | 격자 전체 47–55건 평균 | 연속 공간 M3–6 (중계 격자 스냅) 평균 |")
	print("|---|---|---|---|")
	print("| 최적 (정확해) | 0 | 0 | — |")
	print("| 빔 탐색 | +0.0% / +0.2% | +0.4% | — |")
	print("| 작업열 탐색 | — / +5.0% | +5.9% | — |")
	print("| 선견 규칙 | — / +12.4% | +14.3% | — |")
	f = os.path.join(FIG, f"toy_c33_SWa_{T}.csv")
	rc = "---"
	if os.path.exists(f):
		R = list(csv.DictReader(open(f)))
		rc = f"{np.mean([(float(r['heuristic']) / float(r['milp']) - 1) * 100 for r in R if r['heuristic']]):+.1f}% (완주 {sum(1 for r in R if r['heuristic'])}/{len(R)})"
	print(f"| 규칙 | +50.0% / +54.3% | +55.3% | {rc} |")
	for run in ["SWa"] + runs:
		if run == "SWa":
			g36, gall = "+51.9% / +122.0% (이상치 1건)", "+56.0%"
		else:
			g36, gall = lattice(run)
		print(f"| {run} | {g36} | {gall} | {toy(run)} |")


if __name__ == "__main__":
	main()
