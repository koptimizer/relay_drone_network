"""사이클 12 평가 CSV 여섯 개를 읽어 간소 구성 채택 여부를 판단할 표를 만든다.

구성(표준·3/30·5/50)마다 자기회귀군과 간소군 CSV를 합쳐 한 줄씩 정리한다.
완주율과 완주 에피소드 makespan만 남기고, 측정 잡음 80스텝을 함께 표기한다.
"""

import csv
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
NOISE = 80
CFG = (("표준 4/50", "std"), ("미지 3/30", "d3"), ("미지 5/50", "d5"))
KEEP = ("규칙+교착", "SWa", "L1a", "L1b", "L2a", "L2b")


def label(m):
	"""CSV의 긴 방법 이름을 짧은 표시 이름으로 바꾼다."""
	if "교착 탈출" in m:
		return "규칙+교착"
	for k in ("SWa", "L1a", "L1b", "L2a", "L2b"):
		if k in m:
			return k
	return None


def read(path):
	"""한 평가 CSV에서 표시할 행만 뽑아 사전으로 돌려준다."""
	if not os.path.exists(path):
		return {}
	out = {}
	with open(path, encoding="utf-8") as f:
		for r in csv.DictReader(f):
			k = label(r["method"])
			if k:
				out[k] = (float(r["full_pct"]), r["makespan"], r["mk_sd"])
	return out


def main():
	"""여섯 CSV를 읽어 구성별 비교표를 출력한다."""
	stamp = sys.argv[1] if len(sys.argv) > 1 else "26_09_26_17"
	for name, tag in CFG:
		d = {}
		for grp in ("ar", "pl"):
			d.update(read(os.path.join(ROOT, "figures", f"eval_v5lean_{tag}_{grp}_{stamp}.csv")))
		if not d:
			print(f"[{name}] 결과 없음")
			continue
		print(f"\n[{name}]  (측정 잡음 +-{NOISE}스텝)")
		print(f"{'방법':<12}{'완주율':>8}{'makespan':>10}{'표준편차':>10}")
		for k in KEEP:
			if k in d:
				p, mk, sd = d[k]
				print(f"{k:<12}{p:>7.1f}%{mk:>10}{sd:>10}")
		base = d.get("SWa")
		lean = [d[k][1] for k in ("L2a", "L2b") if k in d and d[k][1] != "-"]
		if base and base[1] != "-" and lean:
			m = sum(float(x) for x in lean) / len(lean)
			gap = m - float(base[1])
			verdict = "동등" if abs(gap) <= NOISE else ("간소 우세" if gap < 0 else "기준 우세")
			print(f"  간소 평균 {m:.0f} 대 기준 {float(base[1]):.0f} -> 차이 {gap:+.0f} ({verdict})")


if __name__ == "__main__":
	main()
