#!/usr/bin/env bash
# v5 makespan 사이클: 3단계 가중치(s3b, ep1500)를 이어받아 시간 신호를 강화해 미세조정한다.
#   A: 완주 시점 보상 200 x (1 - 스텝/상한)      (시드 1, 2)
#   B: gamma 0.997 -> 0.999 (유효 지평 333 -> 1000) (시드 1, 2)
# 최고 체크포인트는 배송 + 속도 점수로 고른다. 끝나면 표준·3대·5대 구성에서 3단계 원본과 비교한다.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5mk_26_09_17_03.log"
T="proposed_src/pipeline/train_v5_26_09_15_23.py"
E="proposed_src/pipeline/eval_v5_26_09_15_23.py"
C="--random-config --init-from weights/v5_26_09_15_23_s3b/latest.pth --select makespan --max-episodes 800"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }

say "=== makespan 사이클 학습 시작 (A: 완주보상, B: gamma 0.999, 시드 2개씩) ==="
setsid nohup python3 -u $T --tag v5_26_09_17_03_A1 $C --complete-bonus 200 --seed 1 >> runs/v5mk_A1.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_17_03_A2 $C --complete-bonus 200 --seed 2 >> runs/v5mk_A2.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_17_03_B1 $C --gamma 0.999 --seed 1 >> runs/v5mk_B1.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_17_03_B2 $C --gamma 0.999 --seed 2 >> runs/v5mk_B2.log 2>&1 &
sleep 10

say "=== 종료 대기 (최대 10시간) ==="
END=$(( $(date +%s) + 10*3600 ))
while true; do
	N=$(pgrep -fc "tag v5_26_09_17_03_[AB]" || true)
	[ "${N:-0}" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; pkill -f "tag v5_26_09_17_03_[AB]"; sleep 5; break; }
	sleep 180
done

export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
MG="weights/v5_26_09_15_23_s3b/best_manager.pth"
for t in v5_26_09_17_03_A1 v5_26_09_17_03_A2 v5_26_09_17_03_B1 v5_26_09_17_03_B2; do
	[ -f "weights/$t/best_manager.pth" ] && MG="$MG weights/$t/best_manager.pth"
done
L="--arch set --no-cluster-penalty --stochastic --straight-worker --max-steps 10000 --no-progress-limit 1000000000 --deadlock-limit 1000000000"
say "=== 평가 (a): 표준 구성 4/50 CC고정, 60시드x2 ==="
python3 -u $E --n 60 --reps 2 $L --manager $MG --out eval_v5mk_std_26_09_17_03 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
say "=== 평가 (b): 드론 3, 목적지 30, CC 무작위, 40시드 ==="
python3 -u $E --n 40 $L --num-drones 3 --num-dests 30 --random-cc --seed0 501 --manager $MG --out eval_v5mk_d3_26_09_17_03 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
say "=== 평가 (c): 드론 5, 목적지 50, CC 무작위, 40시드 ==="
python3 -u $E --n 40 $L --num-drones 5 --num-dests 50 --random-cc --seed0 701 --manager $MG --out eval_v5mk_d5_26_09_17_03 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
say "=== 완료 ==="
