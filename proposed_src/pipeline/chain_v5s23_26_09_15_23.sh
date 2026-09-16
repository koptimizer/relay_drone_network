#!/usr/bin/env bash
# v5 2단계(집합 구조, 고정 구성 4/50)와 3단계(집합 구조, 구성 무작위)를 시드 2개씩 병렬로 학습하고,
# 끝나면 표준 구성과 학습에서 본 적 없는 구성(드론 3/5/6대)으로 네 가중치를 모두 평가한다.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5s23_26_09_15_23.log"
T="proposed_src/pipeline/train_v5_26_09_15_23.py"
E="proposed_src/pipeline/eval_v5_26_09_15_23.py"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }

say "=== v5 2·3단계 학습 시작 ==="
setsid nohup python3 -u $T --tag v5_26_09_15_23_s2a --seed 1 >> runs/v5_s2a.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_15_23_s2b --seed 2 >> runs/v5_s2b.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_15_23_s3a --seed 1 --random-config >> runs/v5_s3a.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_15_23_s3b --seed 2 --random-config >> runs/v5_s3b.log 2>&1 &
sleep 10

say "=== 종료 대기 (최대 12시간) ==="
END=$(( $(date +%s) + 12*3600 ))
while true; do
	N=$(pgrep -fc "tag v5_26_09_15_23_[s]" || true)
	[ "${N:-0}" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; pkill -f "tag v5_26_09_15_23_[s]"; sleep 5; break; }
	sleep 180
done

export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
MG=""
for t in v5_26_09_15_23_s2a v5_26_09_15_23_s2b v5_26_09_15_23_s3a v5_26_09_15_23_s3b; do
	[ -f "weights/$t/best_manager.pth" ] && MG="$MG weights/$t/best_manager.pth"
done
L="--arch set --no-cluster-penalty --stochastic --straight-worker --max-steps 10000 --no-progress-limit 1000000000 --deadlock-limit 1000000000"

say "=== 평가 (a): 표준 구성 드론 4, 목적지 50, CC 고정, 60시드x2 ==="
python3 -u $E --n 60 --reps 2 $L --manager $MG --out eval_v5s23_std_26_09_15_23 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
say "=== 평가 (b): 미지 구성 드론 3, 목적지 30, CC 무작위, 40시드 ==="
python3 -u $E --n 40 --reps 1 $L --num-drones 3 --num-dests 30 --random-cc --seed0 501 --manager $MG --out eval_v5s23_d3_26_09_15_23 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
say "=== 평가 (c): 미지 구성 드론 6, 목적지 80, CC 무작위, 40시드 ==="
python3 -u $E --n 40 --reps 1 $L --num-drones 6 --num-dests 80 --random-cc --seed0 601 --manager $MG --out eval_v5s23_d6_26_09_15_23 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
say "=== 평가 (d): 미지 구성 드론 5, 목적지 50, CC 무작위, 40시드 ==="
python3 -u $E --n 40 --reps 1 $L --num-drones 5 --num-dests 50 --random-cc --seed0 701 --manager $MG --out eval_v5s23_d5_26_09_15_23 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
say "=== 완료 ==="
