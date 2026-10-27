#!/usr/bin/env bash
# 사이클 31b: 정책 반복(expert iteration) 선견 교사 (26-10-27).
#   사이클 31(LA)의 진단: 규칙이 뒤를 잇는다는 가정의 선견 라벨은 정책이 뒤를 이을 때 44%의 상태에서 손해였다
#   (라벨이 갈리는 445상태, 두 이득의 상관 r=0.02). 그래서 라벨을 현재 정책 연속으로 검증한다.
#   LP1·LP2: SWa 가중치에서 시작. 결정의 30%에서 (1) 규칙 연속 선견으로 정책 표본과 다른 대안 하나를 고르고
#            (2) 대안과 표본을 현재 정책 연속으로 공통 난수 6쌍씩 굴려 평균 20스텝 넘게 짧고 t<-2 일 때만 대안 라벨(가중 5).
#            그 밖은 SWa 와 같은 규칙 라벨.
#   CT1·CT2: 대조군. 같은 출발점·같은 설정에서 규칙 라벨만 (추가 학습 자체의 효과를 가른다).
#   공통: 엔트로피 비 0.25 고정(SWa 의 끝값), critic 예열 50 에피소드. 예산 1,000·인내 8 (기본값). 감시 스크립트.
#   학습 뒤: 표준 쌍별 평가(3/30, 5/50, 4/50) + 정확해 55건 격자 평가.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
SUP="proposed_src/pipeline/supervise_v5_26_10_13_12.sh"
PR="proposed_src/util/paired_v5_26_09_27_08.py"
LE="proposed_src/util/lattice_eval_v5_26_10_20_17.py"
LOG="$ROOT/runs/v5c31b_26_10_27_13.log"
T="v5_26_10_27_13"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
C="--random-config --drones-range 3 4 --residual-penalty 0.1 --rule-reg 2.0 --autoregressive --select makespan --speed-weight 40 --holdout-steps 3000 --eval-n 80 --ent-frac 0.25 --init-actor weights/v5_26_09_23_18_SWa/best_manager.pth --critic-warmup 50"
L="--teacher-lookahead --teacher-policy --la-workers 6 --la-k 3 --la-frac 0.3 --la-reps 6 --la-margin 20 --la-weight 5"
say "=== 1단계: 학습 4실행 시작 ==="
setsid nohup bash $SUP ${T}_LP1 $C $L --seed 1 > /dev/null 2>&1 &
setsid nohup bash $SUP ${T}_LP2 $C $L --seed 2 > /dev/null 2>&1 &
setsid nohup bash $SUP ${T}_CT1 $C --seed 1 > /dev/null 2>&1 &
setsid nohup bash $SUP ${T}_CT2 $C --seed 2 > /dev/null 2>&1 &
sleep 120
while true; do
	n=0
	for r in LP1 LP2 CT1 CT2; do
		if grep -q "감시 종료" "runs/sup_${T}_${r}_supervisor.log" 2>/dev/null \
		   || ! pgrep -f "[s]upervise_v5_26_10_13_12.sh ${T}_${r} " > /dev/null; then n=$((n+1)); fi
	done
	[ "$n" -ge 4 ] && break
	sleep 300
done
say "=== 1단계 종료 ==="
say "=== 2단계: 표준 쌍별 평가 ==="
pr(){ out=$1; n=$2; reps=$3; nd=$4; nm=$5; s0=$6
	EX=""; [ "$nd" != "0" ] && EX="--num-drones $nd --num-dests $nm --random-cc --seed0 $s0"
	python3 -u $PR --autoregressive --manager $W --n $n --reps $reps $EX --out "$out" 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"; }
for r in LP1 LP2 CT1 CT2; do
	W="weights/${T}_$r/best_manager.pth"
	[ -f "$W" ] || { say "가중치 없음: $r"; continue; }
	pr paired_c31b_${r}_d3_${T} 40 5 3 30 501 &
	pr paired_c31b_${r}_d5_${T} 40 5 5 50 701 &
	pr paired_c31b_${r}_std_${T} 60 4 0 0 0 &
	wait
done
say "=== 3단계: 정확해 인스턴스 격자 평가 ==="
for r in LP1 LP2 CT1 CT2; do
	W="weights/${T}_$r/best_manager.pth"
	[ -f "$W" ] || continue
	( python3 -u $LE --pattern "bfs2_n3m*_h98_26_10_21_03.json" --manager $W --reps 30 --out lattice_c31b_${r}_m36_${T} 2>&1 | grep -vE 'pygame|Hello|pkg' | tee -a "$LOG"
	  python3 -u $LE --pattern "bfs3_n3m[78]q2s*_26_10_21_06.json" --manager $W --reps 30 --out lattice_c31b_${r}_m78_${T} 2>&1 | grep -vE 'pygame|Hello|pkg' | tee -a "$LOG" ) &
done
wait
say "=== 완료 ==="
