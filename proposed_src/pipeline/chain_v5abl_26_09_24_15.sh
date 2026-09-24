#!/usr/bin/env bash
# 절제 실험 (v5.4 기준): 제안 구성에서 한 요소씩 빼고 같은 조건으로 학습·평가한다.
#   기준(SWa): 자기회귀 + 규칙 정규화 2.0 + 잔여 페널티 0.1 + 엔트로피 0.6->0.25@400
#              + 드론 3-4 무작위 구성 + 홀드아웃 80 x 3,000 + 선택 속도가중치 40
#   A1 자기회귀 제거      A2 규칙 정규화 제거      A3 엔트로피 감쇠 제거(0.6 고정)
#   A4 구성 무작위화 제거(고정 4/50)              A5 잔여 페널티 제거
# 시드 1을 먼저 5개 돌리고(1차), 이어 시드 2를 돌린다(2차). 각 차수는 5개 동시.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5abl_26_09_24_15.log"
T="proposed_src/pipeline/train_v5_26_09_15_23.py"
E="proposed_src/pipeline/eval_v5_26_09_15_23.py"
B="--random-config --drones-range 3 4 --residual-penalty 0.1 --rule-reg 2.0 --autoregressive --select makespan --speed-weight 40 --holdout-steps 3000 --eval-n 80 --max-episodes 1500 --ent-frac 0.6 --ent-frac-final 0.25 --ent-anneal-episodes 400"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }

launch(){   # $1 = 시드
	s=$1
	setsid nohup python3 -u $T --tag v5_26_09_24_15_A1s$s --random-config --drones-range 3 4 --residual-penalty 0.1 --rule-reg 2.0 --select makespan --speed-weight 40 --holdout-steps 3000 --eval-n 80 --max-episodes 1500 --ent-frac 0.6 --ent-frac-final 0.25 --ent-anneal-episodes 400 --seed $s >> runs/v5abl_A1s$s.log 2>&1 &
	sleep 5
	setsid nohup python3 -u $T --tag v5_26_09_24_15_A2s$s --random-config --drones-range 3 4 --residual-penalty 0.1 --autoregressive --select makespan --speed-weight 40 --holdout-steps 3000 --eval-n 80 --max-episodes 1500 --ent-frac 0.6 --ent-frac-final 0.25 --ent-anneal-episodes 400 --seed $s >> runs/v5abl_A2s$s.log 2>&1 &
	sleep 5
	setsid nohup python3 -u $T --tag v5_26_09_24_15_A3s$s --random-config --drones-range 3 4 --residual-penalty 0.1 --rule-reg 2.0 --autoregressive --select makespan --speed-weight 40 --holdout-steps 3000 --eval-n 80 --max-episodes 1500 --ent-frac 0.6 --seed $s >> runs/v5abl_A3s$s.log 2>&1 &
	sleep 5
	setsid nohup python3 -u $T --tag v5_26_09_24_15_A4s$s --num-drones 4 --num-dests 50 --residual-penalty 0.1 --rule-reg 2.0 --autoregressive --select makespan --speed-weight 40 --holdout-steps 3000 --eval-n 80 --max-episodes 1500 --ent-frac 0.6 --ent-frac-final 0.25 --ent-anneal-episodes 400 --seed $s >> runs/v5abl_A4s$s.log 2>&1 &
	sleep 5
	setsid nohup python3 -u $T --tag v5_26_09_24_15_A5s$s --random-config --drones-range 3 4 --rule-reg 2.0 --autoregressive --select makespan --speed-weight 40 --holdout-steps 3000 --eval-n 80 --max-episodes 1500 --ent-frac 0.6 --ent-frac-final 0.25 --ent-anneal-episodes 400 --seed $s >> runs/v5abl_A5s$s.log 2>&1 &
	sleep 10
}
wait_all(){ # $1 = 시드
	END=$(( $(date +%s) + 26*3600 ))
	while true; do
		N=$(pgrep -fc "tag v5_26_09_24_15_A[1-5]s$1" || true)
		[ "${N:-0}" -eq 0 ] && { say "시드 $1 학습 종료"; break; }
		[ "$(date +%s)" -ge "$END" ] && { say "시드 $1 시간 초과 — 중단"; pkill -f "tag v5_26_09_24_15_A[1-5]s$1"; sleep 5; break; }
		sleep 180
	done
}
say "=== 절제 실험 1차 (시드 1) 시작 ==="
launch 1; wait_all 1
say "=== 절제 실험 2차 (시드 2) 시작 ==="
launch 2; wait_all 2

export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
L="--arch set --no-cluster-penalty --stochastic --straight-worker --max-steps 10000 --no-progress-limit 1000000000 --deadlock-limit 1000000000"
# 자기회귀를 쓴 실행과 쓰지 않은 실행(A1)은 추론 방식이 달라 평가를 나눈다
MG_AR=""; MG_PL=""
for t in v5_26_09_24_15_A2s1 v5_26_09_24_15_A2s2 v5_26_09_24_15_A3s1 v5_26_09_24_15_A3s2 \
         v5_26_09_24_15_A4s1 v5_26_09_24_15_A4s2 v5_26_09_24_15_A5s1 v5_26_09_24_15_A5s2 \
         v5_26_09_23_18_SWa; do
	[ -f "weights/$t/best_manager.pth" ] && MG_AR="$MG_AR weights/$t/best_manager.pth"; done
for t in v5_26_09_24_15_A1s1 v5_26_09_24_15_A1s2; do
	[ -f "weights/$t/best_manager.pth" ] && MG_PL="$MG_PL weights/$t/best_manager.pth"; done
for cfg in "std 60 2 0 0 0" "d3 40 1 3 30 501" "d5 40 1 5 50 701"; do
	set -- $cfg
	tag=$1; n=$2; reps=$3; nd=$4; nm=$5; s0=$6
	EX=""; [ "$nd" != "0" ] && EX="--num-drones $nd --num-dests $nm --random-cc --seed0 $s0"
	say "=== 평가 $tag (자기회귀) ==="
	python3 -u $E --n $n --reps $reps $L --autoregressive $EX --manager $MG_AR --out eval_v5abl_${tag}_ar_26_09_24_15 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
	say "=== 평가 $tag (자기회귀 제거 A1) ==="
	python3 -u $E --n $n --reps $reps $L $EX --manager $MG_PL --out eval_v5abl_${tag}_pl_26_09_24_15 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
done
say "=== 완료 ==="
