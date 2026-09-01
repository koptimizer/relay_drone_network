"""학습 실행 상태를 한 화면으로 요약한다.

폰에서 바로 읽을 수 있게 좁은 폭으로 출력한다. 인자 없이 실행하면 전체 요약,
--watch를 주면 갱신될 때마다 다시 그린다.
"""

import argparse
import csv
import os
import subprocess
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def read_csv(path):
	"""CSV를 dict 리스트로 읽는다. 없거나 비었으면 빈 리스트."""
	if not os.path.exists(path):
		return []
	try:
		return list(csv.DictReader(open(path, encoding="utf-8")))
	except (OSError, csv.Error):
		return []


def running_tags():
	"""현재 실행 중인 학습 프로세스의 --tag 값 집합을 반환한다."""
	out = subprocess.run(["ps", "-eo", "args"], capture_output=True, text=True).stdout
	tags = set()
	for line in out.splitlines():
		if "pipeline/train_" in line and "--tag" in line:
			parts = line.split()
			tags.add(parts[parts.index("--tag") + 1])
	return tags


def fnum(row, key, default=None):
	"""CSV 셀을 float으로. 비었거나 없으면 default."""
	v = (row or {}).get(key, "")
	try:
		return float(v)
	except (TypeError, ValueError):
		return default


def run_summary(tag, live):
	"""한 실행의 진행 상황과 홀드아웃 최고 성능을 요약한다."""
	d = os.path.join(ROOT, "runs", tag)
	m = read_csv(os.path.join(d, f"metrics_{tag}.csv"))
	e = read_csv(os.path.join(d, f"evals_{tag}.csv"))
	if not m:
		return None

	ep = int(fnum(m[-1], "episode", 0))
	hrs = (fnum(m[-1], "wall_sec", 0) or 0) / 3600
	state = "실행중" if tag in live else "종료"

	line = f"{tag.replace('v2_26_08_25_14_', '').replace('v1_', 'v1-'):<16} {state:4} ep{ep:<5d} {hrs:4.1f}h"
	if e:
		best = max(e, key=lambda r: fnum(r, "delivered", -1))
		stale = len(e) - 1 - e.index(best)
		line += (f" | 최고 {fnum(best, 'delivered'):5.1f}@ep{best['episode']:<5}"
		         f" 최근 {fnum(e[-1], 'delivered'):5.1f} 정체 {stale:2d}")
	else:
		last = m[-1]
		line += f" | 학습지도 배송 {fnum(last, 'delivered', 0):.0f} (홀드아웃 평가 전)"
	return line


def show():
	"""전체 상태를 출력한다."""
	live = running_tags()
	runs = sorted(d for d in os.listdir(os.path.join(ROOT, "runs"))
	              if os.path.isdir(os.path.join(ROOT, "runs", d)))

	print(f"=== 학습 상태  {time.strftime('%m-%d %H:%M')}  (실행중 {len(live)}개) ===")
	for tag in runs:
		s = run_summary(tag, live)
		if s:
			print(s)

	fig = os.path.join(ROOT, "figures")
	bases = sorted((f for f in os.listdir(fig) if f.startswith("baseline_")),
	               key=lambda f: os.path.getmtime(os.path.join(fig, f))) if os.path.isdir(fig) else []
	if bases:
		rows = read_csv(os.path.join(fig, bases[-1]))
		print(f"\n=== 베이스라인  ({bases[-1]}) ===")
		for r in rows:
			print(f"{r['method']:<17} 배송 {float(r['delivered']):5.1f}/50"
			      f"  재적재 {float(r['reloads']):4.1f}  2홉+ {float(r['hop2plus']):.2f}")

	gpu = subprocess.run(
		["nvidia-smi", "--query-gpu=utilization.gpu,memory.used,memory.total",
		 "--format=csv,noheader"], capture_output=True, text=True).stdout.strip()
	if gpu:
		print(f"\nGPU {gpu}")


def main():
	"""인자를 읽어 1회 출력하거나 반복 감시한다."""
	p = argparse.ArgumentParser()
	p.add_argument("--watch", type=int, default=0, help="초 단위 주기로 반복 출력")
	args = p.parse_args()
	while True:
		show()
		if not args.watch:
			return
		time.sleep(args.watch)
		print()


if __name__ == "__main__":
	main()
