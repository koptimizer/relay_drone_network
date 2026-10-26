#!/usr/bin/env bash
# 사이클 31: 선견 교사 학습 (26-10-27).
#   정규화 목표 = 규칙 위의 한 수 롤아웃 선견이 그 상태에서 고르는 배정 (--teacher-lookahead, 후보 3곳, 작업자 8).
#   선견은 정책과 같은 선택지 안에서 고르므로 라벨이 항상 행동 공간 안에 있다 (사이클 28의 실패 원인 회피).
#   나머지는 SWa 조리법. 예산 1,000 에피소드·인내 8 (기본값). 감시 스크립트로 자동 재개. LA1·LA2 두 시드.
#   학습 뒤: 표준 쌍별 평가(3/30, 5/50, 4/50) + 정확해 인스턴스 55건 격자 평가.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
SUP="proposed_src/pipeline/supervise_v5_26_10_13_12.sh"
PR="proposed_src/util/paired_v5_26_09_27_08.py"
LE="proposed_src/util/lattice_eval_v5_26_10_20_17.py"
LOG="$ROOT/runs/v5c31_26_10_27_00.log"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
C="--random-config --drones-range 3 4 --residual-penalty 0.1 --rule-reg 2.0 --autoregressive --select makespan --speed-weight 40 --holdout-steps 3000 --eval-n 80 --ent-frac 0.6 --ent-frac-final 0.25 --ent-anneal-episodes 400 --teacher-lookahead --la-workers 8 --la-k 3"
say "=== 1단계: 학습 2실행 시작 ==="
setsid nohup bash $SUP v5_26_10_27_00_LA1 $C --seed 1 > /dev/null 2>&1 &
setsid nohup bash $SUP v5_26_10_27_00_LA2 $C --seed 2 > /dev/null 2>&1 &
sleep 120
while true; do
	n=0
	for r in LA1 LA2; do
		if grep -q "감시 종료" "runs/sup_v5_26_10_27_00_${r}_supervisor.log" 2>/dev/null \
		   || ! pgrep -f "[s]upervise_v5_26_10_13_12.sh v5_26_10_27_00_${r} " > /dev/null; then n=$((n+1)); fi
	done
	[ "$n" -ge 2 ] && break
	sleep 300
done
say "=== 1단계 종료 ==="
say "=== 2단계: 표준 쌍별 평가 ==="
pr(){ out=$1; n=$2; reps=$3; nd=$4; nm=$5; s0=$6
	EX=""; [ "$nd" != "0" ] && EX="--num-drones $nd --num-dests $nm --random-cc --seed0 $s0"
	python3 -u $PR --autoregressive --manager $W --n $n --reps $reps $EX --out "$out" 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"; }
for r in LA1 LA2; do
	W="weights/v5_26_10_27_00_$r/best_manager.pth"
	[ -f "$W" ] || { say "가중치 없음: $r"; continue; }
	pr paired_c31_${r}_d3_v5_26_10_27_00 40 5 3 30 501 &
	pr paired_c31_${r}_d5_v5_26_10_27_00 40 5 5 50 701 &
	pr paired_c31_${r}_std_v5_26_10_27_00 60 4 0 0 0 &
	wait
done
say "=== 3단계: 정확해 인스턴스 격자 평가 ==="
for r in LA1 LA2; do
	W="weights/v5_26_10_27_00_$r/best_manager.pth"
	[ -f "$W" ] || continue
	( python3 -u $LE --pattern "bfs2_n3m*_h98_26_10_21_03.json" --manager $W --reps 30 --out lattice_c31_${r}_m36_v5_26_10_27_00 2>&1 | grep -vE 'pygame|Hello|pkg' | tee -a "$LOG"
	  python3 -u $LE --pattern "bfs3_n3m[78]q2s*_26_10_21_06.json" --manager $W --reps 30 --out lattice_c31_${r}_m78_v5_26_10_27_00 2>&1 | grep -vE 'pygame|Hello|pkg' | tee -a "$LOG" ) &
done
wait
say "=== 완료 ==="
