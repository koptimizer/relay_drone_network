#!/usr/bin/env bash
# 사이클 23: 규칙 행동으로 상위 정책을 지도학습(BC)해 초기화한 뒤 SAC로 미세조정한다. 시드 4개.
#   왜: 사이클 13-22의 여덟 개입은 모두 크레딧(가치 추정) 경로였고 모두 막혔다. 이것은 초기화
#   경로다. 관측에 규칙의 중계 배정을 맞출 정보가 있고(탐침 85%), BC로 그 정보를 정책에 먼저
#   심으면 탐색이 무작위가 아니라 규칙 근방에서 시작한다. 사이클 13(중계 모방 가중치)은 학습 중
#   정규화였고 이것은 출발점이라는 점이 다르다.
#   갈래 둘: BC 초기화만(a), BC 초기화 + critic 예열 50 에피소드(w). 무작위 critic이 BC 정책을
#   첫 갱신에서 밀어내는 것이 분명한 실패 모드이므로 예열을 함께 시험한다.
#   조리법은 SWa(사이클 11)와 같다. 예산 1,000 에피소드·인내 8.
#   채택 기준: (1) 3/30 쌍별 중앙 +10 이하, (2) 표준·5/50 악화 없음,
#              (3) 기제 확인 — 3/30 중계 대수가 규칙(1.39)에 가까워질 것 (기준 SWa 0.78).
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5c23_26_10_12_09.log"
T="proposed_src/pipeline/train_v5_26_09_15_23.py"
BC="proposed_src/pipeline/bc_pretrain_v5_26_10_12_09.py"
PR="proposed_src/util/paired_v5_26_09_27_08.py"
DG="proposed_src/util/diag_d3_v5_26_09_27_11.py"
C="--random-config --drones-range 3 4 --residual-penalty 0.1 --rule-reg 2.0 --autoregressive --select makespan --speed-weight 40 --holdout-steps 3000 --eval-n 80 --ent-frac 0.6 --ent-frac-final 0.25 --ent-anneal-episodes 400"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }

say "=== 1단계: 규칙 행동 지도학습 (BC) 2개 ==="
for b in 1 2; do
	python3 -u $BC --samples 40000 --epochs 12 --autoregressive --seed $b \
		--out weights/bc_v5_26_10_12_09_$b/actor.pth 2>&1 | grep -vE 'pkg|Hello' | tee -a "$LOG"
done
for b in 1 2; do
	[ -f "weights/bc_v5_26_10_12_09_$b/actor.pth" ] || { say "BC 실패: $b — 중단"; exit 1; }
done

say "=== 2단계: 미세조정 학습 시작 (BC1a BC2a BC1w BC2w) ==="
setsid nohup python3 -u $T --tag v5_26_10_12_09_BC1a $C --init-actor weights/bc_v5_26_10_12_09_1/actor.pth --seed 1 >> runs/v5c23_BC1a.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_10_12_09_BC2a $C --init-actor weights/bc_v5_26_10_12_09_2/actor.pth --seed 2 >> runs/v5c23_BC2a.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_10_12_09_BC1w $C --init-actor weights/bc_v5_26_10_12_09_1/actor.pth --critic-warmup 50 --seed 3 >> runs/v5c23_BC1w.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_10_12_09_BC2w $C --init-actor weights/bc_v5_26_10_12_09_2/actor.pth --critic-warmup 50 --seed 4 >> runs/v5c23_BC2w.log 2>&1 &
sleep 10
say "=== 종료 대기 (최대 30시간) ==="
END=$(( $(date +%s) + 30*3600 ))
while true; do
	N=$(pgrep -fc "tag v5_26_10_12_09_" || true)
	[ "${N:-0}" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; pkill -f "tag v5_26_10_12_09_"; sleep 5; break; }
	sleep 180
done
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2

say "=== 3단계: 기제 확인 — 3/30 중계 대수 (규칙 1.39, SWa 0.78) ==="
for r in BC1a BC2a BC1w BC2w; do
	W="weights/v5_26_10_12_09_$r/best_manager.pth"
	[ -f "$W" ] || continue
	python3 -u $DG "$W" "$r" 2>&1 | grep -vE 'pkg|Hello' | tee -a "$LOG"
done

say "=== 4단계: 쌍별 평가 ==="
for r in BC1a BC2a BC1w BC2w; do
	W="weights/v5_26_10_12_09_$r/best_manager.pth"
	[ -f "$W" ] || { say "가중치 없음: $r"; continue; }
	for cfg in "d3 40 5 3 30 501" "d5 40 5 5 50 701" "std 60 2 0 0 0" "d3m20 40 2 3 20 901"; do
		set -- $cfg; tag=$1; n=$2; reps=$3; nd=$4; nm=$5; s0=$6
		EX=""; [ "$nd" != "0" ] && EX="--num-drones $nd --num-dests $nm --random-cc --seed0 $s0"
		say "=== 쌍별 $tag $r ==="
		python3 -u $PR --autoregressive --manager $W --n $n --reps $reps $EX \
			--out paired_${tag}_${r}_v5_26_10_12_09 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"
	done
done
say "=== 완료 ==="
