"""사이클 32 최종 비교: 각 실행의 최고 가중치를 SWa 와 같은 인스턴스로 비교한다 (쌍별 3구성 + 격자 정확해).

셀 = 인스턴스별 학습 정책 중앙 makespan 의 평균, 승–패와 Wilcoxon p 는 SWa 원본 쌍별 파일과의 인스턴스 쌍.
격자 = 정확해 인스턴스에서 30회 중앙값의 최적 대비 격차 (중앙 / 평균).
"""

import argparse
import csv
import os

import numpy as np
from scipy.stats import wilcoxon

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FIG = os.path.join(ROOT, "figures")
REF = {"3/30": "paired_d3_SWa_v5_26_10_17_15.csv", "5/50": "paired_d5_SWa_v5_26_10_17_15.csv", "4/50": "paired_std_SWa_v5_26_10_17_15.csv"}
KEY = {"3/30": "d3", "5/50": "d5", "4/50": "std"}


def med(path):
	by = {}
	for r in csv.DictReader(open(path, encoding="utf-8")):
		v = by.setdefault(int(r["seed"]), [])
		if r["학습"]:
			v.append(float(r["학습"]))
	n = sum(len(v) for v in by.values())
	return {s: float(np.median(v)) for s, v in by.items() if v}, n


def lattice(run, tag):
	opt = {}
	for f in ("lattice_eval3_m36_v5_26_10_27_00.csv", "lattice_eval3_m78_v5_26_10_27_00.csv"):
		for r in csv.DictReader(open(os.path.join(FIG, f))):
			opt[(r["dests"], r["seed"])] = float(r["opt"])
	L = {}
	for part in ("m36", "m78"):
		f = os.path.join(FIG, f"lattice_c32_{run}_{part}_{tag}.csv")
		if os.path.exists(f):
			for r in csv.DictReader(open(f)):
				L[(r["dests"], r["seed"])] = r
	out = []
	for sel in (lambda k: int(k[0]) <= 6, lambda k: int(k[0]) >= 7):
		g = [(float(L[k]["learned_median"]) / opt[k] - 1) * 100 for k in L if sel(k) and L[k]["learned_median"]]
		out.append(f"{np.median(g):+.1f}% / {np.mean(g):+.1f}%" if g else "---")
	return out


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--runs", default="MA1,MA2,MA3,MA4,DA1,DA2")
	p.add_argument("--tag", default="v5_26_11_09_17")
	A = p.parse_args()
	ref = {c: med(os.path.join(FIG, f))[0] for c, f in REF.items()}
	print("실행 | " + " | ".join(f"{c}: 평균 (완주) 승-패 중앙차 p" for c in REF) + " | 격자 M3-6 중앙/평균 | 격자 M7-8 중앙/평균")
	sw = " | ".join(f"{c}: {np.mean(list(ref[c].values())):.0f}" for c in REF)
	print(f"SWa | {sw}")
	for run in A.runs.split(","):
		cells = []
		for c in REF:
			f = os.path.join(FIG, f"paired_c32_{run}_{KEY[c]}_{A.tag}.csv")
			if not os.path.exists(f):
				cells.append(f"{c}: ---")
				continue
			m, n = med(f)
			ks = [s for s in m if s in ref[c]]
			x = np.array([m[s] for s in ks]); y = np.array([ref[c][s] for s in ks])
			pv = wilcoxon(x, y).pvalue if np.any(x != y) else 1.0
			cells.append(f"{c}: {x.mean():.0f} ({n}) {int((x < y).sum())}-{int((x > y).sum())} {np.median(x - y):+.0f} p={pv:.2g}")
		print(f"{run} | " + " | ".join(cells) + " | " + " | ".join(lattice(run, A.tag)))


if __name__ == "__main__":
	main()
