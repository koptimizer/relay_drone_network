#!/usr/bin/env bash
# 사이클 20: 팀 값을 드론별 Q의 평균 대신 상태 의존 단조 가중 평균으로 합친다. 시드 4개, 동시 4실행.
#   왜: 평균은 합만 맞으면 어떤 분해든 손실이 같아 드론별 크레딧이 없다. critic 탐욕 정책이
#   actor에 40전 0승인 것이 그 증거다(26-09-27). 혼합망은 가중치가 상태마다 달라 분해가 식별되고,
#   가중치를 절댓값으로 쓰므로 결합 행동의 최선이 드론별 argmax와 같다는 성질은 유지된다.
#   혼합은 합 1로 정규화한 상태 의존 가중 평균이고 가중치에 1/(2n) 하한을 둔다. 초기값에서
#   정확히 평균과 같다. 하이퍼네트워크형(편향 항 있음)은 목표 Q가 80,000까지 발산해 폐기했고,
#   하한 없는 정규화형도 200에피소드 동안 꺾이지 않았다 (26-10-06 관문 두 번).
#   조리법은 SWa(사이클 11)와 같고 혼합 홀드아웃은 쓰지 않는다. 예산 1,000 에피소드·인내 8.
#   채택 기준: (1) critic 탐욕이 actor에 20전 5승 이상 — 크레딧이 생겼다는 직접 증거,
#              (2) 3/30 쌍별 중앙 +10 이하, (3) 표준·5/50 악화 없음.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5c20_26_10_06_09.log"
T="proposed_src/pipeline/train_v5_26_09_15_23.py"
PR="proposed_src/util/paired_v5_26_09_27_08.py"
QP="proposed_src/util/qpick_v5_26_09_27_15.py"
C="--random-config --drones-range 3 4 --residual-penalty 0.1 --rule-reg 2.0 --autoregressive --select makespan --speed-weight 40 --holdout-steps 3000 --eval-n 80 --ent-frac 0.6 --ent-frac-final 0.25 --ent-anneal-episodes 400 --mix-critic"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== 사이클 20 학습 시작 (MX1 MX2 MX3 MX4, 단조 혼합 critic) ==="
for s in 1 2 3 4; do
	setsid nohup python3 -u $T --tag v5_26_10_06_09_MX$s $C --seed $s >> runs/v5c20_MX$s.log 2>&1 &
	sleep 6
done
sleep 10
say "=== 종료 대기 (최대 30시간) ==="
END=$(( $(date +%s) + 30*3600 ))
while true; do
	N=$(pgrep -fc "tag v5_26_10_06_09_" || true)
	[ "${N:-0}" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; pkill -f "tag v5_26_10_06_09_"; sleep 5; break; }
	sleep 180
done
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
# 먼저 크레딧이 생겼는지 직접 본다 (채택 기준 1). critic 탐욕이 actor를 이기면 분해가 식별된 것이다.
for r in MX1 MX2 MX3 MX4; do
	[ -f "weights/v5_26_10_06_09_$r/latest.pth" ] || { say "latest 없음: $r"; continue; }
	say "=== critic 탐욕 대 actor $r (3/30, 20롤아웃) ==="
	python3 -u $QP --dir weights/v5_26_10_06_09_$r --autoregressive --mix-critic --n 20 --reps 1 \
		--actor-latest 2>&1 | grep -vE '^  |pkg|Hello' | tee -a "$LOG"
done
for r in MX1 MX2 MX3 MX4; do
	W="weights/v5_26_10_06_09_$r/best_manager.pth"
	[ -f "$W" ] || { say "가중치 없음: $r"; continue; }
	for cfg in "d3 40 5 3 30 501" "d5 40 5 5 50 701" "std 60 2 0 0 0" "d3m20 40 2 3 20 901"; do
		set -- $cfg; tag=$1; n=$2; reps=$3; nd=$4; nm=$5; s0=$6
		EX=""; [ "$nd" != "0" ] && EX="--num-drones $nd --num-dests $nm --random-cc --seed0 $s0"
		say "=== 쌍별 $tag $r ==="
		python3 -u $PR --autoregressive --manager $W --n $n --reps $reps $EX \
			--out paired_${tag}_${r}_v5_26_10_06_09 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"
	done
done
say "=== 완료 ==="
