#!/usr/bin/env bash
# 사이클 18: 결합 행동을 보는 critic.
#   사이클 13-16이 보상·선택·크레딧 층위에서 모두 실패했고, 진단은 "어느 드론이 중계할지를 가르칠
#   신호가 없다"였다. 현재 critic은 상태만 보고 드론별 Q를 내므로 중계의 가치(동료가 배송하는지에
#   달림)를 원리적으로 정할 수 없다. CTDE의 표준대로 critic이 결합 행동을 조건으로 받게 한다.
#   JC : --joint-critic (조리법은 SWa + 혼합 홀드아웃)   시드 1-4
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5jc_26_10_01_00.log"
T="proposed_src/pipeline/train_v5_26_09_15_23.py"
PR="proposed_src/util/paired_v5_26_09_27_08.py"
C="--random-config --drones-range 3 4 --residual-penalty 0.1 --rule-reg 2.0 --autoregressive --select makespan --speed-weight 40 --holdout-steps 3000 --eval-n 80 --ent-frac 0.6 --ent-frac-final 0.25 --ent-anneal-episodes 400 --holdout-mix --joint-critic"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== 사이클 18 학습 시작 (결합 critic, 시드 1-4) ==="
for s in 1 2 3 4; do
	setsid nohup python3 -u $T --tag v5_26_10_01_00_JC$s $C --seed $s >> runs/v5jc_JC$s.log 2>&1 &
	sleep 6
done
sleep 10
say "=== 종료 대기 (최대 20시간) ==="
END=$(( $(date +%s) + 20*3600 ))
while true; do
	N=$(pgrep -fc "tag v5_26_10_01_00_JC" || true)
	[ "${N:-0}" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; pkill -f "tag v5_26_10_01_00_JC"; sleep 5; break; }
	sleep 180
done
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
for s in 1 2 3 4; do
	W="weights/v5_26_10_01_00_JC$s/best_manager.pth"
	[ -f "$W" ] || continue
	say "=== 쌍별 3/30 JC$s ==="
	python3 -u $PR --autoregressive --manager $W --n 40 --reps 5 --num-drones 3 --num-dests 30 \
		--random-cc --seed0 501 --out paired_d3_JC${s}_v5_26_10_01_00 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"
	say "=== 쌍별 5/50 JC$s ==="
	python3 -u $PR --autoregressive --manager $W --n 40 --reps 5 --num-drones 5 --num-dests 50 \
		--random-cc --seed0 701 --out paired_d5_JC${s}_v5_26_10_01_00 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"
	say "=== 쌍별 표준 JC$s ==="
	python3 -u $PR --autoregressive --manager $W --n 60 --reps 2 \
		--out paired_std_JC${s}_v5_26_10_01_00 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"
done
say "=== 완료 ==="
