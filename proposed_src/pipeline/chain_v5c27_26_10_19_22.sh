#!/usr/bin/env bash
# 사이클 27: 학습 정책의 편차 안정화 (전문가 피드백 26-10-19).
#   ESC1/ESC2: 휴리스틱과 같은 교착 탈출 덮어쓰기를 학습 루프에 포함해 학습 (--escape-in-loop), 배치도 덮어쓰기와 함께.
#   ENT1/ENT2: 엔트로피 목표 종단을 0.25 -> 0.10 log n 으로 낮춰 정책을 더 뾰족하게.
#   조리법은 SWa(사이클 11)와 같다. 예산 1,000 에피소드·인내 8. 감시 스크립트로 자동 재개. 학습 뒤 쌍별 평가까지 연쇄.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
SUP="proposed_src/pipeline/supervise_v5_26_10_13_12.sh"
PR="proposed_src/util/paired_v5_26_09_27_08.py"
LOG="$ROOT/runs/v5c27_26_10_19_22.log"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
C="--random-config --drones-range 3 4 --residual-penalty 0.1 --rule-reg 2.0 --autoregressive --select makespan --speed-weight 40 --holdout-steps 3000 --eval-n 80 --ent-frac 0.6 --ent-anneal-episodes 400"
say "=== 1단계: 학습 4실행 시작 ==="
setsid nohup bash $SUP v5_26_10_19_22_ESC1 $C --ent-frac-final 0.25 --escape-in-loop --seed 1 > /dev/null 2>&1 &
setsid nohup bash $SUP v5_26_10_19_22_ESC2 $C --ent-frac-final 0.25 --escape-in-loop --seed 2 > /dev/null 2>&1 &
setsid nohup bash $SUP v5_26_10_19_22_ENT1 $C --ent-frac-final 0.10 --seed 3 > /dev/null 2>&1 &
setsid nohup bash $SUP v5_26_10_19_22_ENT2 $C --ent-frac-final 0.10 --seed 4 > /dev/null 2>&1 &
sleep 60
while true; do
	n=0
	for r in ESC1 ESC2 ENT1 ENT2; do grep -q "감시 종료" "runs/sup_v5_26_10_19_22_${r}_supervisor.log" 2>/dev/null && n=$((n+1)); done
	[ "$n" -ge 4 ] && break
	sleep 300
done
say "=== 1단계 종료 (4실행 모두 감시 종료) ==="
pr(){ # out n reps nd nm seed0 extra...
	out=$1; n=$2; reps=$3; nd=$4; nm=$5; s0=$6; shift 6
	EX=""; [ "$nd" != "0" ] && EX="--num-drones $nd --num-dests $nm --random-cc --seed0 $s0"
	python3 -u $PR --autoregressive --manager $W --n $n --reps $reps $EX "$@" --out "$out" 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"
}
say "=== 2단계: 쌍별 평가 (4병렬) ==="
for r in ESC1 ESC2 ENT1 ENT2; do
	W="weights/v5_26_10_19_22_$r/best_manager.pth"
	[ -f "$W" ] || { say "가중치 없음: $r"; continue; }
	EXTRA=""; case $r in ESC*) EXTRA="--escape";; esac
	pr paired_c27_${r}_d3_v5_26_10_19_22 40 5 3 30 501 $EXTRA &
	pr paired_c27_${r}_d5_v5_26_10_19_22 40 5 5 50 701 $EXTRA &
	pr paired_c27_${r}_std_v5_26_10_19_22 60 4 0 0 0 $EXTRA &
	case $r in ESC*) pr paired_c27_${r}_d3_noesc_v5_26_10_19_22 40 5 3 30 501 & ;; esac
	wait
done
say "=== 완료 ==="
