#!/usr/bin/env bash
# 사이클 22: 상위 결정을 단일 에이전트의 순차 결정으로 재정식화한다. 시드 4개, 동시 4실행.
#   왜: 드론 N대가 동시에 결정하면 critic이 팀 수익을 드론별로 쪼개야 하고, 그 분해를 식별하려는
#   시도가 세 번(14·18·20) 모두 실패했다. 순차 결정에서는 하위 결정 하나가 곧 단일 에이전트
#   전이이므로 분해가 필요 없다 — "어느 드론이 중계할지"가 그 차례의 행동가치로 직접 평가된다.
#   정책의 형태는 그대로라 학습 가중치가 기존 평가·쌍별 경로에서 그대로 돈다.
#   관문 두 번에서 기제 시험은 실패했다 (critic 탐욕 완주 4/20, 엔트로피 보정 후 2/20; 같은 시점
#   기존 집계는 19-20/20). 목표 Q는 발산하지 않지만 기존의 7배 수준에 머문다. 사전 확률은 낮게
#   보되, 채택 기준 2(3/30 쌍별)는 실제로 재서 판정한다 (26-10-11).
#   조리법은 SWa(사이클 11)와 같다. 예산 1,000 에피소드·인내 8.
#   채택 기준: (1) critic 탐욕이 actor에 20전 5승 이상, (2) 3/30 쌍별 중앙 +10 이하,
#              (3) 표준·5/50 악화 없음.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5c22_26_10_11_06.log"
T="proposed_src/pipeline/train_v5_26_09_15_23.py"
PR="proposed_src/util/paired_v5_26_09_27_08.py"
QP="proposed_src/util/qpick_v5_26_09_27_15.py"
C="--random-config --drones-range 3 4 --residual-penalty 0.1 --rule-reg 2.0 --autoregressive --select makespan --speed-weight 40 --holdout-steps 3000 --eval-n 80 --ent-frac 0.6 --ent-frac-final 0.25 --ent-anneal-episodes 400 --seq-decision"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== 사이클 22 학습 시작 (SQ1 SQ2 SQ3 SQ4, 순차 크레딧) ==="
for s in 1 2 3 4; do
	setsid nohup python3 -u $T --tag v5_26_10_11_06_SQ$s $C --seed $s >> runs/v5c22_SQ$s.log 2>&1 &
	sleep 6
done
sleep 10
say "=== 종료 대기 (최대 30시간) ==="
END=$(( $(date +%s) + 30*3600 ))
while true; do
	N=$(pgrep -fc "tag v5_26_10_11_06_" || true)
	[ "${N:-0}" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; pkill -f "tag v5_26_10_11_06_"; sleep 5; break; }
	sleep 180
done
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
for r in SQ1 SQ2 SQ3 SQ4; do
	[ -f "weights/v5_26_10_11_06_$r/latest.pth" ] || { say "latest 없음: $r"; continue; }
	say "=== critic 탐욕 대 actor $r (순차 탐욕, 3/30, 20롤아웃) ==="
	python3 -u $QP --dir weights/v5_26_10_11_06_$r --autoregressive --seq-decision --n 20 --reps 1 \
		--actor-latest 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"
done
for r in SQ1 SQ2 SQ3 SQ4; do
	W="weights/v5_26_10_11_06_$r/best_manager.pth"
	[ -f "$W" ] || { say "가중치 없음: $r"; continue; }
	for cfg in "d3 40 5 3 30 501" "d5 40 5 5 50 701" "std 60 2 0 0 0" "d3m20 40 2 3 20 901"; do
		set -- $cfg; tag=$1; n=$2; reps=$3; nd=$4; nm=$5; s0=$6
		EX=""; [ "$nd" != "0" ] && EX="--num-drones $nd --num-dests $nm --random-cc --seed0 $s0"
		say "=== 쌍별 $tag $r ==="
		python3 -u $PR --autoregressive --manager $W --n $n --reps $reps $EX \
			--out paired_${tag}_${r}_v5_26_10_11_06 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"
	done
done
say "=== 완료 ==="
