#!/usr/bin/env bash
# 혼합 상위(기하 규칙 + 학습 교착 탈출)를 정식 표본으로 평가한다. 원본 s3b·규칙과 나란히.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5hy_26_09_17_08.log"
E="proposed_src/pipeline/eval_v5_26_09_15_23.py"
export OMP_NUM_THREADS=3 MKL_NUM_THREADS=3
L="--arch set --no-cluster-penalty --stochastic --straight-worker --max-steps 10000 --no-progress-limit 1000000000 --deadlock-limit 1000000000 --manager weights/v5_26_09_15_23_s3b/best_manager.pth --hybrid 60 100 --hybrid 30 60"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== 혼합 상위 평가 (a): 표준 4/50 CC고정, 60시드x2 ==="
python3 -u $E --n 60 --reps 2 $L --out eval_v5hy_std_26_09_17_08 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
for cfg in "3 30 501 d3" "5 50 701 d5" "6 80 601 d6"; do
	set -- $cfg
	say "=== 혼합 상위 평가: 드론 $1, 목적지 $2, CC 무작위, 40시드 ==="
	python3 -u $E --n 40 $L --num-drones $1 --num-dests $2 --random-cc --seed0 $3 --out eval_v5hy_$4_26_09_17_08 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
done
say "=== 완료 ==="
