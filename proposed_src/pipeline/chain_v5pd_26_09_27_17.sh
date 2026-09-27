#!/usr/bin/env bash
# 사이클 14: 드론별 보상으로 중계 크레딧을 식별한다.
#   critic 진단(26-09-27): 드론별 Q의 평균만 팀 수익에 맞추므로 분해가 미결정이고,
#   critic 탐욕 정책은 actor 대비 40전 0승이다. "누가 중계해야 하는가"를 배울 신호가 없다.
#   개입: 배송이 일어나면 그 통신 경로 위의 중계 드론에게만 개별 보상을 주고(--relay-credit),
#         critic 회귀 목표를 드론별 보상으로 바꾼다(--per-drone-reward).
#   PD2 : 크레딧 2.0 (배송 팀보상 10의 20%)   PD6 : 크레딧 6.0 (60%)
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5pd_26_09_27_17.log"
T="proposed_src/pipeline/train_v5_26_09_15_23.py"
E="proposed_src/pipeline/eval_v5_26_09_15_23.py"
PR="proposed_src/util/paired_v5_26_09_27_08.py"
C="--random-config --drones-range 3 4 --residual-penalty 0.1 --rule-reg 2.0 --autoregressive --select makespan --speed-weight 40 --holdout-steps 3000 --eval-n 80 --max-episodes 1500 --ent-frac 0.6 --ent-frac-final 0.25 --ent-anneal-episodes 400 --per-drone-reward"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== 사이클 14 학습 시작 (PD2 x2 크레딧 2.0, PD6 x2 크레딧 6.0) ==="
setsid nohup python3 -u $T --tag v5_26_09_27_17_PD2a $C --relay-credit 2.0 --seed 1 >> runs/v5pd_PD2a.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_27_17_PD2b $C --relay-credit 2.0 --seed 2 >> runs/v5pd_PD2b.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_27_17_PD6a $C --relay-credit 6.0 --seed 1 >> runs/v5pd_PD6a.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_27_17_PD6b $C --relay-credit 6.0 --seed 2 >> runs/v5pd_PD6b.log 2>&1 &
sleep 10
say "=== 종료 대기 (최대 30시간) ==="
END=$(( $(date +%s) + 30*3600 ))
while true; do
	N=$(pgrep -fc "tag v5_26_09_27_17_PD[26]" || true)
	[ "${N:-0}" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; pkill -f "tag v5_26_09_27_17_PD[26]"; sleep 5; break; }
	sleep 180
done
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
for r in PD2a PD2b PD6a PD6b; do
	W="weights/v5_26_09_27_17_$r/best_manager.pth"
	[ -f "$W" ] || continue
	say "=== 쌍별 표준 $r ==="
	python3 -u $PR --autoregressive --manager $W --n 60 --reps 2 \
		--out paired_std_${r}_v5_26_09_27_17 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"
	say "=== 쌍별 3/30 $r ==="
	python3 -u $PR --autoregressive --manager $W --n 40 --reps 5 --num-drones 3 --num-dests 30 \
		--random-cc --seed0 501 --out paired_d3_${r}_v5_26_09_27_17 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"
	say "=== 쌍별 5/50 $r ==="
	python3 -u $PR --autoregressive --manager $W --n 40 --reps 5 --num-drones 5 --num-dests 50 \
		--random-cc --seed0 701 --out paired_d5_${r}_v5_26_09_27_17 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"
done
L="--arch set --autoregressive --no-cluster-penalty --stochastic --straight-worker --max-steps 10000 --no-progress-limit 1000000000 --deadlock-limit 1000000000"
MG=""
for t in v5_26_09_27_17_PD2a v5_26_09_27_17_PD2b v5_26_09_27_17_PD6a v5_26_09_27_17_PD6b v5_26_09_23_18_SWa; do
	[ -f "weights/$t/best_manager.pth" ] && MG="$MG weights/$t/best_manager.pth"; done
for cfg in "std 60 2 0 0 0" "d3 40 1 3 30 501" "d5 40 1 5 50 701"; do
	set -- $cfg
	tag=$1; n=$2; reps=$3; nd=$4; nm=$5; s0=$6
	EX=""; [ "$nd" != "0" ] && EX="--num-drones $nd --num-dests $nm --random-cc --seed0 $s0"
	say "=== 집계 평가 $tag ==="
	python3 -u $E --n $n --reps $reps $L $EX --manager $MG --out eval_v5pd_${tag}_26_09_27_17 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
done
say "=== 완료 ==="
