#!/usr/bin/env bash
# makespan 사이클 4: 막힘 페널티. 규칙 정규화 2.0 + 잔여 페널티 0.1 위에 투영이 잘라낸 변위에 팀 비용을 매긴다.
#   B01: 막힘 페널티 0.1 (시드 1, 2)    B03: 0.3 (시드 1, 2)
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5bp_26_09_18_16.log"
T="proposed_src/pipeline/train_v5_26_09_15_23.py"
E="proposed_src/pipeline/eval_v5_26_09_15_23.py"
C="--random-config --drones-range 3 6 --residual-penalty 0.1 --rule-reg 2.0 --select makespan --holdout-steps 3000 --eval-n 40 --max-episodes 1500"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== 막힘 페널티 사이클 학습 시작 (0.1 x2, 0.3 x2) ==="
setsid nohup python3 -u $T --tag v5_26_09_18_16_B01a $C --blocked-penalty 0.1 --seed 1 >> runs/v5bp_B01a.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_18_16_B01b $C --blocked-penalty 0.1 --seed 2 >> runs/v5bp_B01b.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_18_16_B03a $C --blocked-penalty 0.3 --seed 1 >> runs/v5bp_B03a.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_18_16_B03b $C --blocked-penalty 0.3 --seed 2 >> runs/v5bp_B03b.log 2>&1 &
sleep 10
say "=== 종료 대기 (최대 16시간) ==="
END=$(( $(date +%s) + 16*3600 ))
while true; do
	N=$(pgrep -fc "tag v5_26_09_18_16_[B]" || true)
	[ "${N:-0}" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; pkill -f "tag v5_26_09_18_16_[B]"; sleep 5; break; }
	sleep 180
done
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
L="--arch set --no-cluster-penalty --stochastic --straight-worker --max-steps 10000 --no-progress-limit 1000000000 --deadlock-limit 1000000000"
MG="weights/v5_26_09_18_00_R20a/best_manager.pth"
for t in v5_26_09_18_16_B01a v5_26_09_18_16_B01b v5_26_09_18_16_B03a v5_26_09_18_16_B03b; do
	[ -f "weights/$t/best_manager.pth" ] && MG="$MG weights/$t/best_manager.pth"; done
say "=== 평가 (a): 표준 4/50, 60시드x2 ==="
python3 -u $E --n 60 --reps 2 $L --manager $MG --out eval_v5bp_std_26_09_18_16 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
for cfg in "3 30 501 d3" "5 50 701 d5"; do
	set -- $cfg
	say "=== 평가: 드론 $1, 목적지 $2, CC 무작위, 40시드 ==="
	python3 -u $E --n 40 $L --num-drones $1 --num-dests $2 --random-cc --seed0 $3 --manager $MG --out eval_v5bp_$4_26_09_18_16 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
done
say "=== 완료 ==="
