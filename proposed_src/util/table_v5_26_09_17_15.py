"""평가 CSV들을 모아 3방법 비교표(완주율 / 완주 시 makespan)를 마크다운으로 찍는다."""
import csv
import glob
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
# (표시 이름, 드론, 목적지, CC, 롤아웃, 범위, CSV 후보 순서)
CFGS = [
	("4 / 50 · CC 고정", "안", 120, ["eval_v5hy2_std_26_09_17_10"]),
	("3 / 30 · CC 무작위", "안", 40, ["eval_v5hy2_d3_26_09_17_10"]),
	("5 / 50 · CC 무작위", "안", 40, ["eval_v5hy2_d5_26_09_17_10"]),
	("6 / 80 · CC 무작위", "안", 40, ["eval_v5hy2_d6_26_09_17_10"]),
	("3 / 50 · CC 무작위", "안", 20, ["eval_v5tbl_d3_m50_26_09_17_15"]),
	("3 / 80 · CC 무작위", "경계", 20, ["eval_v5tbl_d3_m80_26_09_17_15"]),
	("6 / 150 · CC 무작위", "밖", 20, ["eval_v5tbl_d6_m150_26_09_17_15", "eval_v5s3_extrap_d6_m150_26_09_16_13"]),
	("8 / 150 · CC 무작위", "밖", 20, ["eval_v5tbl_d8_m150_26_09_17_15", "eval_v5s3_extrap_d8_m150_26_09_16_13"]),
	("5 / 300 · CC 무작위", "밖", 20, ["eval_v5tbl_d5_m300_26_09_17_15"]),
	("8 / 300 · CC 무작위", "밖", 20, ["eval_v5tbl_d8_m300_26_09_17_15", "eval_v5s3_extrap_d8_m300_26_09_16_13"]),
	("10 / 300 · CC 무작위", "밖", 20, ["eval_v5tbl_d10_m300_26_09_17_15", "eval_v5s3_extrap_d10_m300_26_09_16_13"]),
]


def pick(rows, key):
	"""방법 이름으로 행을 고른다."""
	for r in rows:
		m = r["method"]
		if key == "rule" and "기하" in m:
			return r
		if key == "hybrid" and "혼합" in m and "탈출>1" in m:
			return r
		if key == "learned" and "s3b" in m and "혼합" not in m:
			return r
	return None


def cell(r):
	"""완주율 / makespan 셀."""
	if r is None:
		return "—"
	mk = r["makespan"] if r["makespan"] not in ("-", "") else "—"
	return f"{float(r['full_pct']):.0f}% / {mk}"


def main():
	"""표를 출력한다."""
	print("| 구성 (드론 / 목적지) | 학습 범위 | 롤아웃 | 기하 규칙 | 규칙 + 학습 보조 | **학습 단독** |")
	print("|---|---|---|---|---|---|")
	for name, rng, n, cands in CFGS:
		rows = []
		for c in cands:
			p = os.path.join(ROOT, "figures", c + ".csv")
			if os.path.exists(p):
				rows += list(csv.DictReader(open(p, encoding="utf-8")))
		if not rows:
			print(f"| {name} | {rng} | {n} | (측정 중) | (측정 중) | (측정 중) |")
			continue
		print(f"| {name} | {rng} | {n} | {cell(pick(rows, 'rule'))} | {cell(pick(rows, 'hybrid'))} | **{cell(pick(rows, 'learned'))}** |")


if __name__ == "__main__":
	main()
