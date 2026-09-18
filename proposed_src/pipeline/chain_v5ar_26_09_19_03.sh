#!/usr/bin/env bash
# makespan 사이클 5: 편대 대형을 배울 수 있게 한다.
#   AR   : 자기회귀 상위 결정(먼 드론부터 순차, 앞 드론의 선택을 보고 결정)            (시드 1, 2)
#   ARC  : AR + 도달권 잠재 shaping 5 (중계로 사슬이 늘면 그 자리에서 팀 보상)          (시드 1, 2)
# 공통: 규칙 정규화 2.0 + 잔여 페널티 0.1, 드론 3-6, 홀드아웃 40 x 3000, 완주 우선 선택.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5ar_26_09_19_03.log"
T="proposed_src/pipeline/train_v5_26_09_15_23.py"
E="proposed_src/pipeline/eval_v5_26_09_15_23.py"
C="--random-config --drones-range 3 6 --residual-penalty 0.1 --rule-reg 2.0 --autoregressive --select makespan --holdout-steps 3000 --eval-n 40 --max-episodes 1500"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== 사이클 5 학습 시작 (AR x2, ARC x2) ==="
setsid nohup python3 -u $T --tag v5_26_09_19_03_ARa  $C --seed 1 >> runs/v5ar_ARa.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_19_03_ARb  $C --seed 2 >> runs/v5ar_ARb.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_19_03_ARCa $C --coverage-shaping 5.0 --seed 1 >> runs/v5ar_ARCa.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_19_03_ARCb $C --coverage-shaping 5.0 --seed 2 >> runs/v5ar_ARCb.log 2>&1 &
sleep 10
say "=== 종료 대기 (최대 18시간) ==="
END=$(( $(date +%s) + 18*3600 ))
while true; do
	N=$(pgrep -fc "tag v5_26_09_19_03_[A]" || true)
	[ "${N:-0}" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; pkill -f "tag v5_26_09_19_03_[A]"; sleep 5; break; }
	sleep 180
done
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
L="--arch set --autoregressive --no-cluster-penalty --stochastic --straight-worker --max-steps 10000 --no-progress-limit 1000000000 --deadlock-limit 1000000000"
MG=""
for t in v5_26_09_19_03_ARa v5_26_09_19_03_ARb v5_26_09_19_03_ARCa v5_26_09_19_03_ARCb; do
	[ -f "weights/$t/best_manager.pth" ] && MG="$MG weights/$t/best_manager.pth"; done
say "=== 평가 (a): 표준 4/50, 60시드x2 ==="
python3 -u $E --n 60 --reps 2 $L --manager $MG --out eval_v5ar_std_26_09_19_03 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
for cfg in "3 30 501 d3" "5 50 701 d5"; do
	set -- $cfg
	say "=== 평가: 드론 $1, 목적지 $2, CC 무작위, 40시드 ==="
	python3 -u $E --n 40 $L --num-drones $1 --num-dests $2 --random-cc --seed0 $3 --manager $MG --out eval_v5ar_$4_26_09_19_03 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
done
say "=== 완료 ==="
