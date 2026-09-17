#!/usr/bin/env bash
# 영구 이관 조건을 붙인 혼합 상위의 정식 평가. 표준 120 롤아웃 + 미지 구성 3종 40 롤아웃.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5hy2_26_09_17_10.log"
E="proposed_src/pipeline/eval_v5_26_09_15_23.py"
export OMP_NUM_THREADS=3 MKL_NUM_THREADS=3
L="--arch set --no-cluster-penalty --stochastic --straight-worker --max-steps 10000 --no-progress-limit 1000000000 --deadlock-limit 1000000000 --manager weights/v5_26_09_15_23_s3b/best_manager.pth --hybrid 60 100 0 300 --hybrid 60 100 1 200"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== 이관 혼합 평가 (a): 표준 4/50, 60시드x2 ==="
python3 -u $E --n 60 --reps 2 $L --out eval_v5hy2_std_26_09_17_10 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
for cfg in "3 30 501 d3" "5 50 701 d5" "6 80 601 d6"; do
	set -- $cfg
	say "=== 이관 혼합 평가: 드론 $1, 목적지 $2, CC 무작위, 40시드 ==="
	python3 -u $E --n 40 $L --num-drones $1 --num-dests $2 --random-cc --seed0 $3 --out eval_v5hy2_$4_26_09_17_10 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
done
say "=== 완료 ==="
