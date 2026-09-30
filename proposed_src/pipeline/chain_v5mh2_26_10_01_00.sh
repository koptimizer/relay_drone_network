#!/usr/bin/env bash
# 사이클 17: 사이클 15의 조리법(혼합 홀드아웃)으로 시드 4개를 더 돌린다.
#   목적: 표준 구성의 '동률'을 '우세'로 확정할 수 있는지 (사이클 15 갈래 평균 -0, MH1 -37).
#   최종 선택은 시드 601-640에서, 보고는 501-540에서 한다 (ckpt_sweep 참조).
# (아래 주석은 사이클 15의 것)
#   사이클 13(모방 가중치)과 14(드론별 크레딧)는 모두 3/30을 고치지 못했다. 특히 14는 중계를
#   서는 것 자체가 보상이 되어 5/50에서 115스텝 역전을 만들었다. 두 번의 방법 변경이 실패했으므로
#   이번에는 SWa 조리법 그대로 두고, 체크포인트 선택이 4/50만 보던 것을 3/30 절반으로 바꾼다.
#   홀드아웃 3/30 시드는 801-840으로 평가 시드(501-540)와 겹치지 않는다.
#   시드 4개를 돌려 실행 간 변동(3/30 쌍별 +23 ~ +56)의 좋은 끝을 뽑을 수 있는지 본다.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5mh2_26_10_01_00.log"
T="proposed_src/pipeline/train_v5_26_09_15_23.py"
E="proposed_src/pipeline/eval_v5_26_09_15_23.py"
PR="proposed_src/util/paired_v5_26_09_27_08.py"
C="--random-config --drones-range 3 4 --residual-penalty 0.1 --rule-reg 2.0 --autoregressive --select makespan --speed-weight 40 --holdout-steps 3000 --eval-n 80 --ent-frac 0.6 --ent-frac-final 0.25 --ent-anneal-episodes 400 --holdout-mix"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== 사이클 17 학습 시작 (혼합 홀드아웃, 시드 5-8) ==="
for s in 5 6 7 8; do
	setsid nohup python3 -u $T --tag v5_26_10_01_00_MH$s $C --seed $s >> runs/v5mh2_MH$s.log 2>&1 &
	sleep 6
done
sleep 10
say "=== 종료 대기 (최대 30시간) ==="
END=$(( $(date +%s) + 30*3600 ))
while true; do
	N=$(pgrep -fc "tag v5_26_10_01_00_MH" || true)
	[ "${N:-0}" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; pkill -f "tag v5_26_10_01_00_MH"; sleep 5; break; }
	sleep 180
done
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
for s in 5 6 7 8; do
	W="weights/v5_26_10_01_00_MH$s/best_manager.pth"
	[ -f "$W" ] || continue
	say "=== 쌍별 3/30 MH$s ==="
	python3 -u $PR --autoregressive --manager $W --n 40 --reps 5 --num-drones 3 --num-dests 30 \
		--random-cc --seed0 501 --out paired_d3_MH${s}_v5_26_10_01_00 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"
	say "=== 쌍별 5/50 MH$s ==="
	python3 -u $PR --autoregressive --manager $W --n 40 --reps 5 --num-drones 5 --num-dests 50 \
		--random-cc --seed0 701 --out paired_d5_MH${s}_v5_26_10_01_00 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"
	say "=== 쌍별 표준 MH$s ==="
	python3 -u $PR --autoregressive --manager $W --n 60 --reps 2 \
		--out paired_std_MH${s}_v5_26_10_01_00 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"
done
L="--arch set --autoregressive --no-cluster-penalty --stochastic --straight-worker --max-steps 10000 --no-progress-limit 1000000000 --deadlock-limit 1000000000"
MG=""
for t in v5_26_10_01_00_MH5 v5_26_10_01_00_MH6 v5_26_10_01_00_MH7 v5_26_10_01_00_MH8 v5_26_09_23_18_SWa; do
	[ -f "weights/$t/best_manager.pth" ] && MG="$MG weights/$t/best_manager.pth"; done
for cfg in "d3 40 1 3 30 501" "d5 40 1 5 50 701" "std 60 2 0 0 0"; do
	set -- $cfg
	tag=$1; n=$2; reps=$3; nd=$4; nm=$5; s0=$6
	EX=""; [ "$nd" != "0" ] && EX="--num-drones $nd --num-dests $nm --random-cc --seed0 $s0"
	say "=== 집계 평가 $tag ==="
	python3 -u $E --n $n --reps $reps $L $EX --manager $MG --out eval_v5mh2_${tag}_26_09_29_04 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
done
say "=== 완료 ==="
