#!/usr/bin/env bash
# 3방법(기하 규칙 / 규칙+학습 보조 / 학습 단독) 비교표를 위해 빠진 칸을 채운다.
# 학습 범위 밖 4구성에는 혼합이 없었고, GIF로 보여준 3구성(3/50, 3/80, 5/300)은 정식 평가가 없었다.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5tbl_26_09_17_15.log"
E="proposed_src/pipeline/eval_v5_26_09_15_23.py"
export OMP_NUM_THREADS=3 MKL_NUM_THREADS=3
L="--arch set --no-cluster-penalty --stochastic --straight-worker --max-steps 10000 --no-progress-limit 1000000000 --deadlock-limit 1000000000 --random-cc --reps 1 --manager weights/v5_26_09_15_23_s3b/best_manager.pth --hybrid 60 100 1 200"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
for cfg in "3 50 901" "3 80 911" "6 150 801" "8 150 811" "5 300 921" "10 300 821" "8 300 831"; do
	set -- $cfg
	say "=== 표 채우기: 드론 $1, 목적지 $2, CC 무작위, 20시드 ==="
	python3 -u $E --n 20 $L --num-drones $1 --num-dests $2 --seed0 $3 --out eval_v5tbl_d$1_m$2_26_09_17_15 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
done
say "=== 완료 ==="
