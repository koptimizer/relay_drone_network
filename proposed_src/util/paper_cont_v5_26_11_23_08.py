"""논문 RQ2 연속 참조 표 생성 (사이클 35): 연속 위치·이산 시간 모형의 (κ, M) 8조합에 대한 상한·하한과 제어기 격차.

입력: figures/cont34p2_*.json (2차, 내접 여유 0.1 m 모형), figures/cont34_*.json (1차 하한), figures/cont_ctrl_v5_26_11_19_14.csv
(규칙·선견·SWa 전이·DA2), figures/cont_ctrl_sq_v5_26_11_23_08.csv (같은 분포에서 학습한 SQa·SQb; 없으면 빈 칸),
figures/cont_search_v5_26_11_23_12.csv (연속 공간 방문 순서·작업열 탐색, 동률 난수 10회 중앙값; 없으면 빈 칸).
출력: docs/tex/tab_cont_v5_en_26_11_23_08.tex, docs/tex/tab_cont_v5_ko_26_11_23_08.tex, 그리고 표준 출력의 요약.
같은 분포 정책은 학습 홀드아웃 점수(runs/<tag>/evals_<tag>.csv 의 최고 score)가 높은 시드를 쓴다.
"""

import csv
import glob
import json
import os

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FIG = os.path.join(ROOT, "figures")
TEX = os.path.join(ROOT, "docs", "tex")
T = "v5_26_11_23_08"
TX = "26_11_23_08"                                   # tex 파일 접미 (v5_ 는 파일명 앞에 이미 있음)


def load_models():
	res = {}
	for fj in glob.glob(os.path.join(FIG, "cont34p2_m*s*_k*.json")):
		d = json.load(open(fj)); key = (d["kappa"], d["dests"], d["seed"])
		f1 = os.path.join(FIG, f"cont34_m{d['dests']}s{d['seed']}_k{d['kappa']}.json")
		if os.path.exists(f1):
			o = json.load(open(f1))
			if o["bound"] is not None and (d["bound"] is None or o["bound"] > d["bound"]):
				d["bound"] = o["bound"]
		res[key] = d
	for key in sorted(res):                                  # κ=10 해는 κ=5 에서 실행 가능 → κ=5 의 유효 상한
		if key[0] == 5 and (10, key[1], key[2]) in res:
			o10 = res[(10, key[1], key[2])]["obj"]; d = res[key]
			if o10 is not None and (d["obj"] is None or o10 < d["obj"]):
				d["obj"] = o10
		d = res[key]
		d["proven"] = d["status"] == "optimal" or (d["obj"] is not None and d["bound"] is not None and d["obj"] - d["bound"] <= 1e-4 * d["obj"] + 1e-6)
	return res


def read_ctrl(name):
	f = os.path.join(FIG, name)
	if not os.path.exists(f):
		return {}
	return {(int(r["dests"]), int(r["seed"])): r for r in csv.DictReader(open(f))}


def best_seed(tags):
	best, bs = None, -1e9
	for tag in tags:
		f = os.path.join(ROOT, "runs", tag, f"evals_{tag}.csv")
		if not os.path.exists(f):
			continue
		sc = max(float(r["score"]) for r in csv.DictReader(open(f)))
		if sc > bs:
			best, bs = tag, sc
	return best, bs


def cell(vals, objs, bnds, proven_all):
	if not vals:
		return "---"
	v = np.mean(vals); go = 100 * np.mean([a / b - 1 for a, b in zip(vals, objs)]); gb = 100 * np.mean([a / b - 1 for a, b in zip(vals, bnds)])
	return f"{v:.0f} ({go:+.0f})" if proven_all else f"{v:.0f} ({go:+.0f}/{gb:+.0f})"


def main():
	res = load_models()
	base = read_ctrl("cont_ctrl_v5_26_11_19_14.csv")
	sq = read_ctrl(f"cont_ctrl_sq_{T}.csv")
	tag, sc = best_seed([f"{T}_SQa", f"{T}_SQb"])
	sqcol = tag.split("_")[-1] if tag else None
	print(f"같은 분포 정책: {tag} (홀드아웃 최고 점수 {sc:.1f})" if tag else "같은 분포 정책: 아직 없음")
	srch = read_ctrl("cont_search_v5_26_11_23_12.csv")
	cols = [("rule", base), ("look", base), ("vorder_med10", srch), ("taskseq_med10", srch), ("swa", base),
	        (sqcol, sq) if sqcol else (None, {}), ("da2", base)]
	rows_en, rows_ko, summary = [], [], []
	for k in (10, 5):
		for M in (9, 10, 11, 12):
			ks = sorted(key for key in res if key[0] == k and key[1] == M)
			ds = [res[key] for key in ks]
			obj = [d["obj"] for d in ds]; bnd = [d["bound"] for d in ds]; prov = sum(d["proven"] for d in ds)
			cells = []
			for name, src in cols:
				vals, o2, b2 = [], [], []
				for key, d in zip(ks, ds):
					r = src.get((M, key[2])) if name else None
					if r and r.get(name):
						vals.append(float(r[name])); o2.append(d["obj"]); b2.append(d["bound"])
				cells.append(cell(vals, o2, b2, prov == len(ks)))
			line = f"{k} & {M} & {prov}/{len(ks)} & {np.mean(obj):.0f} & {np.mean(bnd):.0f} & " + " & ".join(cells) + r" \\"
			rows_en.append(line); rows_ko.append(line)
			summary.append((k, M, prov, np.mean(obj), np.mean(bnd), cells))
	head_en = (r"\begin{table*}[t]" "\n" r"\centering" "\n"
	           r"\caption{Continuous-position, time-discretised reference model (period $\kappa$ steps, three drones, capacity 3, five instances per row) against the controllers run in the continuous space. UB and LB are the mean incumbent and bound after the two solver passes (one hour cold, two hours warm-started with a tightened horizon); ``proven'' counts instances solved to optimality. Controller cells give the mean makespan over the five instances and, in parentheses, the mean gap in percent to the model's UB and, when a row is not fully proven, to its LB (the true gap lies between the two). Learned policies are medians of 30 sampled runs per instance: ``transfer'' is the policy trained on 20--80 demand points with capacity 5, ``in-dist.'' the policy trained on the same distribution as these instances (8--16 demand points, capacity 3), ``distilled'' the policy distilled from the lookahead rule (an analysis variant, not a proposed controller). The two offline searches of Section~\ref{sec:search} are scored by the median over ten tie-breaking seeds of the projection of their best plan. Every model plan replays in the continuous environment with full delivery, no disconnection and 0.0\,m tracking error.}" "\n"
	           r"\label{tab:cont}" "\n" r"\scriptsize\setlength{\tabcolsep}{1.9pt}" "\n"
	           r"\begin{tabular}{rrrrrlllllll}" "\n" r"\toprule" "\n"
	           r" & & & & & \multicolumn{2}{c}{Online, learning-free} & \multicolumn{2}{c}{Offline search} & \multicolumn{2}{c}{Learned (proposed)} & Analysis \\" "\n"
	           r"\cmidrule(lr){6-7}\cmidrule(lr){8-9}\cmidrule(lr){10-11}\cmidrule(lr){12-12}" "\n"
	           r"$\kappa$ & $M$ & proven & UB & LB & Rule & Lookahead & Visit order & Task seq. & Transfer & In-dist. & Distilled \\" "\n" r"\midrule" "\n")
	head_ko = (r"\begin{table*}[t]" "\n" r"\centering" "\n"
	           r"\caption{연속 위치·이산 시간 참조 모형(기간 $\kappa$ 스텝, 드론 3대, 용량 3, 행당 인스턴스 5개)과 연속 공간에서 실행한 제어기의 비교. UB·LB 는 두 차례 풀이(1시간 초기 풀이, 단축 지평·시작해로 2시간) 뒤의 incumbent 와 하한의 평균이고 ``증명''은 최적성이 증명된 인스턴스 수다. 제어기 칸은 다섯 인스턴스의 평균 makespan 과 괄호 안에 모형 UB 대비 평균 격차(\%), 행이 모두 증명되지 않았으면 LB 대비 격차도 함께 적는다(참 격차는 둘 사이). 학습 정책은 인스턴스당 30회 표본의 중앙값이며, ``전이''는 수요지 20--80·용량 5 로 학습한 정책, ``같은 분포''는 이 인스턴스와 같은 분포(수요지 8--16·용량 3)로 학습한 정책, ``증류''는 선견 규칙을 증류한 정책(제안이 아닌 분석용 변형)이다. 두 오프라인 탐색(\ref{sec:search}절)은 최적 계획을 투영의 동률 난수 10개로 다시 굴린 중앙값이다. 모든 모형 계획은 연속 환경에서 전량 배송·단절 0·추종 오차 0.0\,m 로 재생된다.}" "\n"
	           r"\label{tab:cont}" "\n" r"\scriptsize\setlength{\tabcolsep}{1.9pt}" "\n"
	           r"\begin{tabular}{rrrrrlllllll}" "\n" r"\toprule" "\n"
	           r" & & & & & \multicolumn{2}{c}{온라인, 학습 없음} & \multicolumn{2}{c}{오프라인 탐색} & \multicolumn{2}{c}{학습 (제안)} & 분석 \\" "\n"
	           r"\cmidrule(lr){6-7}\cmidrule(lr){8-9}\cmidrule(lr){10-11}\cmidrule(lr){12-12}" "\n"
	           r"$\kappa$ & $M$ & 증명 & UB & LB & 규칙 & 선견 규칙 & 방문 순서 & 작업열 & 전이 & 같은 분포 & 증류 \\" "\n" r"\midrule" "\n")
	tail = "\n" r"\bottomrule" "\n" r"\end{tabular}" "\n" r"\end{table*}" "\n"
	open(os.path.join(TEX, f"tab_cont_v5_en_{TX}.tex"), "w").write(head_en + "\n".join(rows_en[:4]) + "\n" r"\midrule" "\n" + "\n".join(rows_en[4:]) + tail)
	open(os.path.join(TEX, f"tab_cont_v5_ko_{TX}.tex"), "w").write(head_ko + "\n".join(rows_ko[:4]) + "\n" r"\midrule" "\n" + "\n".join(rows_ko[4:]) + tail)
	print("| κ | M | 증명 | UB | LB | 규칙 | 선견 | 방문순서 | 작업열 | 학습(전이 SWa) | 학습(같은 분포) | 증류 DA2 |")
	print("|---|---|---|---|---|---|---|---|---|---|---|---|")
	for k, M, prov, ub, lb, cells in summary:
		print(f"| {k} | {M} | {prov}/5 | {ub:.0f} | {lb:.0f} | " + " | ".join(cells) + " |")
	# 시드별 같은 분포 정책 요약 (투명성)
	for s in ("SQa", "SQb"):
		vals = [(int(r["dests"]), int(r["seed"]), float(r[s])) for r in sq.values() if r.get(s)]
		if vals:
			g = [v / res[(10, M, sd)]["obj"] - 1 for M, sd, v in vals]
			print(f"{s}: 평균 makespan {np.mean([v for _, _, v in vals]):.0f}, κ=10 UB 대비 평균 격차 {100 * np.mean(g):+.0f}%, 완주 " + ",".join(sorted({r[s + '_complete'] for r in sq.values() if r.get(s)})))
	print("저장:", os.path.join(TEX, f"tab_cont_v5_en_{TX}.tex"), "및 ko")


if __name__ == "__main__":
	main()
