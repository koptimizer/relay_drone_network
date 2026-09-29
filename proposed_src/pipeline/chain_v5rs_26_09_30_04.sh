#!/usr/bin/env bash
# 사이클 16: 사이클 14의 교정판 — 중계 크레딧을 얹지 말고 배송 보상에서 떼어 준다.
#   사이클 14는 중계에 보상을 얹어 중계가 목적이 됐고 세 구성 모두 나빠졌다. 총합이 보존되는
#   재분배라면 중계 자체는 이득이 아니고, 배송을 실제로 가능하게 한 경우에만 값이 생긴다.
#   관측 탐침(26-09-30): 규칙의 중계 배정은 현재 관측만으로 85% 맞힐 수 있다. 정보는 있고
#   그것을 쓰게 만드는 신호가 없다는 뜻이라, 손댈 곳은 가치 쪽이 맞다.
#   RS2 : --relay-share 2.0 --per-drone-reward   (가설)
#   PD0 : --per-drone-reward 만                  (대조군: 드론별 critic 자체가 해로운지 분리)
# 학습 예산은 새 기본값(상한 1000, 인내 8)을 쓴다 — 인자로 박지 않는다.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5rs_26_09_30_04.log"
T="proposed_src/pipeline/train_v5_26_09_15_23.py"
E="proposed_src/pipeline/eval_v5_26_09_15_23.py"
PR="proposed_src/util/paired_v5_26_09_27_08.py"
C="--random-config --drones-range 3 4 --residual-penalty 0.1 --rule-reg 2.0 --autoregressive --select makespan --speed-weight 40 --holdout-steps 3000 --eval-n 80 --ent-frac 0.6 --ent-frac-final 0.25 --ent-anneal-episodes 400 --per-drone-reward"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== 사이클 16 학습 시작 (RS2 x2 재분배 2.0, PD0 x2 대조군) ==="
setsid nohup python3 -u $T --tag v5_26_09_30_04_RS2a $C --relay-share 2.0 --seed 1 >> runs/v5rs_RS2a.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_30_04_RS2b $C --relay-share 2.0 --seed 2 >> runs/v5rs_RS2b.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_30_04_PD0a $C --seed 1 >> runs/v5rs_PD0a.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_30_04_PD0b $C --seed 2 >> runs/v5rs_PD0b.log 2>&1 &
sleep 10
say "=== 종료 대기 (최대 20시간) ==="
END=$(( $(date +%s) + 20*3600 ))
while true; do
	N=$(pgrep -fc "tag v5_26_09_30_04_[RP]" || true)
	[ "${N:-0}" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; pkill -f "tag v5_26_09_30_04_[RP]"; sleep 5; break; }
	sleep 180
done
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
for r in RS2a RS2b PD0a PD0b; do
	W="weights/v5_26_09_30_04_$r/best_manager.pth"
	[ -f "$W" ] || continue
	say "=== 쌍별 3/30 $r ==="
	python3 -u $PR --autoregressive --manager $W --n 40 --reps 5 --num-drones 3 --num-dests 30 \
		--random-cc --seed0 501 --out paired_d3_${r}_v5_26_09_30_04 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"
	say "=== 쌍별 5/50 $r ==="
	python3 -u $PR --autoregressive --manager $W --n 40 --reps 5 --num-drones 5 --num-dests 50 \
		--random-cc --seed0 701 --out paired_d5_${r}_v5_26_09_30_04 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"
	say "=== 쌍별 표준 $r ==="
	python3 -u $PR --autoregressive --manager $W --n 60 --reps 2 \
		--out paired_std_${r}_v5_26_09_30_04 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"
done
say "=== 완료 ==="
