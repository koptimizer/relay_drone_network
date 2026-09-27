#!/usr/bin/env bash
# 사이클 13: 드론 3대 구성의 makespan 열세를 중계 배정 학습으로 좁힌다.
#   진단(26-09-27): 3/30에서 학습은 중계를 0.78대만 쓰고(규칙 1.39) 투영에 막힌 드론-스텝이
#   1,289 대 160, 정체율 0.315 대 0.081이다. 중계 배정만 규칙으로 덮으면 막힘이 60% 줄고
#   쌍별 중앙값이 54스텝 회복된다(p=0.053). 즉 원인은 중계의 가치를 학습하지 못한 것이다.
#   RR3/RR6 : 규칙 정규화에서 '규칙이 중계를 지시한 드론'의 가중치를 3배/6배로 (--relay-reg)
#   RG4     : 규칙 정규화를 균일하게 2.0 -> 4.0 (중계 특화가 아닌 대조군)
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5rc_26_09_27_11.log"
T="proposed_src/pipeline/train_v5_26_09_15_23.py"
E="proposed_src/pipeline/eval_v5_26_09_15_23.py"
PR="proposed_src/util/paired_v5_26_09_27_08.py"
C="--random-config --drones-range 3 4 --residual-penalty 0.1 --rule-reg 2.0 --autoregressive --select makespan --speed-weight 40 --holdout-steps 3000 --eval-n 80 --max-episodes 1500 --ent-frac 0.6 --ent-frac-final 0.25 --ent-anneal-episodes 400"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== 사이클 13 학습 시작 (RR3 x2 중계가중치 3, RG4 x2 균일 규칙정규화 4) ==="
setsid nohup python3 -u $T --tag v5_26_09_27_11_RR3a $C --relay-reg 3.0 --seed 1 >> runs/v5rc_RR3a.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_27_11_RR3b $C --relay-reg 3.0 --seed 2 >> runs/v5rc_RR3b.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_27_11_RG4a $C --rule-reg 4.0 --seed 1 >> runs/v5rc_RG4a.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_27_11_RG4b $C --rule-reg 4.0 --seed 2 >> runs/v5rc_RG4b.log 2>&1 &
sleep 10
say "=== 종료 대기 (최대 26시간) ==="
END=$(( $(date +%s) + 26*3600 ))
while true; do
	N=$(pgrep -fc "tag v5_26_09_27_11_R[RG]" || true)
	[ "${N:-0}" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; pkill -f "tag v5_26_09_27_11_R[RG]"; sleep 5; break; }
	sleep 180
done
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
# 1) 규칙과의 쌍별 비교 — 판정은 이 표로 한다 (중앙값·윌콕슨)
for r in RR3a RR3b RG4a RG4b; do
	W="weights/v5_26_09_27_11_$r/best_manager.pth"
	[ -f "$W" ] || continue
	say "=== 쌍별 표준 $r ==="
	python3 -u $PR --autoregressive --manager $W --n 60 --reps 2 \
		--out paired_std_${r}_v5_26_09_27_11 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"
	say "=== 쌍별 3/30 $r ==="
	python3 -u $PR --autoregressive --manager $W --n 40 --reps 1 --num-drones 3 --num-dests 30 \
		--random-cc --seed0 501 --out paired_d3_${r}_v5_26_09_27_11 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"
	say "=== 쌍별 5/50 $r ==="
	python3 -u $PR --autoregressive --manager $W --n 40 --reps 1 --num-drones 5 --num-dests 50 \
		--random-cc --seed0 701 --out paired_d5_${r}_v5_26_09_27_11 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"
done
# 2) 집계 평가 — 기록용, 베이스라인 다섯 종 포함
L="--arch set --autoregressive --no-cluster-penalty --stochastic --straight-worker --max-steps 10000 --no-progress-limit 1000000000 --deadlock-limit 1000000000"
MG=""
for t in v5_26_09_27_11_RR3a v5_26_09_27_11_RR3b v5_26_09_27_11_RG4a v5_26_09_27_11_RG4b v5_26_09_23_18_SWa; do
	[ -f "weights/$t/best_manager.pth" ] && MG="$MG weights/$t/best_manager.pth"; done
for cfg in "std 60 2 0 0 0" "d3 40 1 3 30 501" "d5 40 1 5 50 701"; do
	set -- $cfg
	tag=$1; n=$2; reps=$3; nd=$4; nm=$5; s0=$6
	EX=""; [ "$nd" != "0" ] && EX="--num-drones $nd --num-dests $nm --random-cc --seed0 $s0"
	say "=== 집계 평가 $tag ==="
	python3 -u $E --n $n --reps $reps $L $EX --manager $MG --out eval_v5rc_${tag}_26_09_27_11 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
done
say "=== 완료 ==="
