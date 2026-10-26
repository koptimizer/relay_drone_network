#!/usr/bin/env bash
# 사이클 30: 소규모 인스턴스 미세조정 (26-10-26).
#   SWa 가중치에서 시작해 드론 3·목적지 3-8(7 이상은 용량 2, 재적재 강제)·거점 무작위 분포로 이어서 학습한다.
#   홀드아웃은 격자 결정 공간(--holdout-lattice)에서 재므로 관문 지표 = 최종 지표(격자 makespan).
#   FT1·FT2: 규칙 정규화 2.0 (SWa 조리법), FL1·FL2: 규칙 정규화 0.5 (규칙에서 더 멀리 벗어날 수 있게).
#   예산 1,000 에피소드·인내 8 (기본값). 감시 스크립트로 자동 재개.
#   학습 뒤: 정확해 인스턴스 55건(M=3-8) 격자 평가 (30회 표본).
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
SUP="proposed_src/pipeline/supervise_v5_26_10_13_12.sh"
LE="proposed_src/util/lattice_eval_v5_26_10_20_17.py"
LOG="$ROOT/runs/v5c30_26_10_26_20.log"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
C="--random-config --drones-range 3 3 --dests-range 3 8 --reload-above 6 --holdout-lattice --eval-n 24 --residual-penalty 0.1 --autoregressive --select makespan --speed-weight 40 --holdout-steps 3000 --ent-frac 0.6 --ent-frac-final 0.25 --ent-anneal-episodes 400 --init-actor weights/v5_26_09_23_18_SWa/best_manager.pth"
say "=== 1단계: 학습 4실행 시작 ==="
setsid nohup bash $SUP v5_26_10_26_20_FT1 $C --rule-reg 2.0 --seed 1 > /dev/null 2>&1 &
setsid nohup bash $SUP v5_26_10_26_20_FT2 $C --rule-reg 2.0 --seed 2 > /dev/null 2>&1 &
setsid nohup bash $SUP v5_26_10_26_20_FL1 $C --rule-reg 0.5 --seed 3 > /dev/null 2>&1 &
setsid nohup bash $SUP v5_26_10_26_20_FL2 $C --rule-reg 0.5 --seed 4 > /dev/null 2>&1 &
sleep 120
while true; do
	n=0
	for r in FT1 FT2 FL1 FL2; do
		if grep -q "감시 종료" "runs/sup_v5_26_10_26_20_${r}_supervisor.log" 2>/dev/null \
		   || ! pgrep -f "[s]upervise_v5_26_10_13_12.sh v5_26_10_26_20_${r} " > /dev/null; then n=$((n+1)); fi
	done
	[ "$n" -ge 4 ] && break
	sleep 300
done
say "=== 1단계 종료 ==="
say "=== 2단계: 정확해 인스턴스 격자 평가 (M=3-8, 55건, 30회) ==="
for r in FT1 FT2 FL1 FL2; do
	W="weights/v5_26_10_26_20_$r/best_manager.pth"
	[ -f "$W" ] || { say "가중치 없음: $r"; continue; }
	( python3 -u $LE --pattern "bfs2_n3m*_h98_26_10_21_03.json" --manager $W --reps 30 --out lattice_c30_${r}_m36_v5_26_10_26_20 2>&1 | grep -vE 'pygame|Hello|pkg' | tee -a "$LOG"
	  python3 -u $LE --pattern "bfs3_n3m[78]q2s*_26_10_21_06.json" --manager $W --reps 30 --out lattice_c30_${r}_m78_v5_26_10_26_20 2>&1 | grep -vE 'pygame|Hello|pkg' | tee -a "$LOG" ) &
done
wait
say "=== 완료 ==="
