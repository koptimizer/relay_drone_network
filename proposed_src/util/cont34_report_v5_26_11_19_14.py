"""사이클 34 보고: 연속 모형(κ=10, 5)의 최적값/하한과 연속 공간 제어기(규칙·선견 규칙·SWa·DA2)를 같은 인스턴스에서 비교한다.

행 = (κ, M) 조합 8개, 시드 5개 평균. 격차 = (제어기 makespan / 모형 값 − 1). 모형이 시간 제한에 걸리면 목적값(상한)과
하한을 따로 적고, 격차는 하한 기준으로도 낸다 (제어기 대 하한 격차는 참 격차의 상한이다).
"""

import csv
import glob
import json
import os

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FIG = os.path.join(ROOT, "figures")


def main():
	ctrl = {}
	f = os.path.join(FIG, "cont_ctrl_v5_26_11_19_14.csv")
	if os.path.exists(f):
		for r in csv.DictReader(open(f)):
			ctrl[(int(r["dests"]), int(r["seed"]))] = r
	res = {}
	for fj in glob.glob(os.path.join(FIG, "cont34_m*s*_k*.json")):
		d = json.load(open(fj))
		res[(d["kappa"], d["dests"], d["seed"])] = d
	print("| κ | M | 건수 | 증명 | 모형 목적 (상한) | 모형 하한 | 풀이 시간 | 규칙 | 선견 규칙 | SWa | DA2 |")
	print("|---|---|---|---|---|---|---|---|---|---|---|")
	rows = []
	for k in (10, 5):
		for M in (9, 10, 11, 12):
			ks = sorted(key for key in res if key[0] == k and key[1] == M)
			if not ks:
				print(f"| {k} | {M} | 0 | --- | --- | --- | --- | | | | |")
				continue
			ds = [res[key] for key in ks]
			obj = np.array([d["obj"] if d["obj"] is not None else np.nan for d in ds], dtype=float)
			bnd = np.array([d["bound"] if d["bound"] is not None else np.nan for d in ds], dtype=float)
			prov = sum(1 for d in ds if d["status"] == "optimal")
			sec = np.mean([d["solve_sec"] for d in ds])
			rep_ok = sum(1 for d in ds if d.get("replay") and d["replay"]["delivered"] == M and d["replay"]["comm_loss"] == 0)
			cells = []
			for name in ("rule", "look", "swa", "da2"):
				vals, gaps_o, gaps_b = [], [], []
				for key, d in zip(ks, ds):
					c = ctrl.get((M, key[2]))
					if not c or not c.get(name):
						continue
					v = float(c[name]); vals.append(v)
					if not np.isnan(obj[ks.index(key)]):
						gaps_o.append(v / obj[ks.index(key)] - 1)
					if not np.isnan(bnd[ks.index(key)]):
						gaps_b.append(v / bnd[ks.index(key)] - 1)
				if vals:
					cells.append(f"{np.mean(vals):.0f} (+{100 * np.mean(gaps_o):.0f}% / +{100 * np.mean(gaps_b):.0f}%)" if gaps_o else f"{np.mean(vals):.0f}")
				else:
					cells.append("---")
			print(f"| {k} | {M} | {len(ks)} | {prov}/{len(ks)} (재생 {rep_ok}) | {np.nanmean(obj):.0f} | {np.nanmean(bnd):.0f} | {sec:.0f}초 | " + " | ".join(cells) + " |")
			rows.append(dict(kappa=k, dests=M, n=len(ks), proven=prov, obj=np.nanmean(obj), bound=np.nanmean(bnd), sec=sec))
	print("\n괄호: 제어기 격차 (대 모형 목적값 / 대 모형 하한). 모형이 증명되면 둘이 같다.")
	# κ 비교: 같은 인스턴스에서 κ=5 가 κ=10 보다 얼마나 낮은가
	pairs = [(res[(10, M, s)]["obj"], res[(5, M, s)]["obj"]) for M in (9, 10, 11, 12) for s in range(1201, 1206)
	         if (10, M, s) in res and (5, M, s) in res and res[(10, M, s)]["obj"] and res[(5, M, s)]["obj"]]
	if pairs:
		a = np.array(pairs)
		print(f"κ=5 대 κ=10 (같은 인스턴스 {len(a)}건, 목적값): 평균 비율 {np.mean(a[:, 1] / a[:, 0]):.3f}, κ=5 가 낮은 건 {int(np.sum(a[:, 1] < a[:, 0]))}, 같은 건 {int(np.sum(a[:, 1] == a[:, 0]))}")


if __name__ == "__main__":
	main()
