#!/usr/bin/env bash
# 학습 범위(드론 3-8, 목적지 20-80) 밖의 큰 구성에서 3단계 가중치를 추가 학습 없이 평가한다.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/extrap_v5_26_09_16_13.log"
E="proposed_src/pipeline/eval_v5_26_09_15_23.py"
MG="weights/v5_26_09_15_23_s3a/best_manager.pth weights/v5_26_09_15_23_s3b/best_manager.pth"
L="--arch set --no-cluster-penalty --stochastic --straight-worker --max-steps 10000 --no-progress-limit 1000000000 --deadlock-limit 1000000000 --random-cc --reps 1"
export OMP_NUM_THREADS=3 MKL_NUM_THREADS=3
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
for cfg in "6 150 801" "8 150 811" "10 300 821" "8 300 831"; do
	set -- $cfg
	say "=== 범위 밖 평가: 드론 $1, 목적지 $2, CC 무작위, 20시드 ==="
	python3 -u $E --n 20 $L --num-drones $1 --num-dests $2 --seed0 $3 --manager $MG --out eval_v5s3_extrap_d$1_m$2_26_09_16_13 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
done
say "=== 완료 ==="
