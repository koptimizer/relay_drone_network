"""v4 학습 5조합의 현재 상태를 한 줄씩 요약한다 (주기 보고용)."""

import glob
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TAGS = ["r_sw", "r_swc", "r_norel", "r_s2"]


def read_csv(path):
	"""CSV를 헤더 없는 행 리스트로 읽는다."""
	if not os.path.exists(path):
		return []
	with open(path, encoding="utf-8") as f:
		rows = [l.rstrip("\n").split(",") for l in f]
	return [r for r in rows[1:] if len(r) > 1 and r[0].isdigit()]


def summarize(tag):
	"""한 실행의 에피소드 수, 최고 홀드아웃, 최근 홀드아웃을 뽑는다."""
	full = f"v4_26_09_14_03_{tag}"
	d = os.path.join(ROOT, "runs", full)
	met = read_csv(os.path.join(d, f"metrics_{full}.csv"))
	ev = read_csv(os.path.join(d, f"evals_{full}.csv"))
	ep = met[-1][0] if met else "0"
	if not ev:
		return f"{tag:8s} ep={ep:<5s} 홀드아웃 아직"
	vals = [(float(r[1]), r[0]) for r in ev]
	best, best_ep = max(vals)
	last = vals[-1][0]
	stale = sum(1 for v, e in vals if int(e) > int(best_ep))
	return (f"{tag:8s} ep={ep:<5s} 최고={best:5.1f}@ep{best_ep:<5s} "
	        f"최근={last:5.1f} 정체={stale}/15")


def main():
	"""전체 상태를 출력한다."""
	n = subprocess.run("pgrep -fc 'tag v4_26_09_14_03_[r]_'", shell=True,
	                   capture_output=True, text=True).stdout.strip()
	out = [f"[v4 상태] 학습 프로세스 {n or 0}개"]
	for t in TAGS:
		out.append("  " + summarize(t))
	done = glob.glob(os.path.join(ROOT, "figures", "eval_v4_final_*.csv"))
	if done:
		out.append("  최종 평가 CSV 생성됨")
	print(" | ".join(out) if len(sys.argv) > 1 else "\n".join(out))


if __name__ == "__main__":
	main()
