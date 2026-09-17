#!/usr/bin/env bash
# makespan 사이클 3: 규칙 정규화 RL. actor 손실에 lambda x (규칙 행동의 NLL)을 더해 정책이 규칙 근처에서
# 출발·유지하되 Q가 강하게 반대할 때(교착)만 벗어나게 한다. 잔여 페널티 0.1 유지, 홀드아웃 40 인스턴스.
#   R05: lambda 0.5 (시드 1, 2)    R20: lambda 2.0 (시드 1, 2)
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5rr_26_09_18_00.log"
T="proposed_src/pipeline/train_v5_26_09_15_23.py"
E="proposed_src/pipeline/eval_v5_26_09_15_23.py"
C="--random-config --drones-range 3 6 --residual-penalty 0.1 --select makespan --holdout-steps 3000 --eval-n 40 --max-episodes 1500"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== 규칙 정규화 사이클 학습 시작 (lambda 0.5 x2, 2.0 x2) ==="
setsid nohup python3 -u $T --tag v5_26_09_18_00_R05a $C --rule-reg 0.5 --seed 1 >> runs/v5rr_R05a.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_18_00_R05b $C --rule-reg 0.5 --seed 2 >> runs/v5rr_R05b.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_18_00_R20a $C --rule-reg 2.0 --seed 1 >> runs/v5rr_R20a.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_18_00_R20b $C --rule-reg 2.0 --seed 2 >> runs/v5rr_R20b.log 2>&1 &
sleep 10
say "=== 종료 대기 (최대 16시간) ==="
END=$(( $(date +%s) + 16*3600 ))
while true; do
	N=$(pgrep -fc "tag v5_26_09_18_00_[R]" || true)
	[ "${N:-0}" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; pkill -f "tag v5_26_09_18_00_[R]"; sleep 5; break; }
	sleep 180
done
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
L="--arch set --no-cluster-penalty --stochastic --straight-worker --max-steps 10000 --no-progress-limit 1000000000 --deadlock-limit 1000000000"
MG="weights/v5_26_09_15_23_s3b/best_manager.pth"
for t in v5_26_09_18_00_R05a v5_26_09_18_00_R05b v5_26_09_18_00_R20a v5_26_09_18_00_R20b; do
	[ -f "weights/$t/best_manager.pth" ] && MG="$MG weights/$t/best_manager.pth"; done
say "=== 평가 (a): 표준 4/50, 60시드x2 ==="
python3 -u $E --n 60 --reps 2 $L --manager $MG --out eval_v5rr_std_26_09_18_00 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
for cfg in "3 30 501 d3" "5 50 701 d5"; do
	set -- $cfg
	say "=== 평가: 드론 $1, 목적지 $2, CC 무작위, 40시드 ==="
	python3 -u $E --n 40 $L --num-drones $1 --num-dests $2 --random-cc --seed0 $3 --manager $MG --out eval_v5rr_$4_26_09_18_00 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
done
say "=== 완료 ==="
