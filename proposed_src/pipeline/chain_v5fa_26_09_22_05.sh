#!/usr/bin/env bash
# makespan 사이클 9: 감쇠를 빨리 끝내고, 드론이 빠듯한 구성을 집중 학습한다.
#   사이클 8 진단: 0.6 -> 0.15를 1,500 에피소드에 걸쳐 감쇠했더니 최고 체크포인트가
#   목표 0.48-0.51 구간(ep300-400)에서 나왔고, 좋은 구간(0.25-0.3)에 닿기 전에 조기 종료됐다.
#   FA   : 0.6 -> 0.25를 400 에피소드에 끝내고 유지, 드론 3-6   (시드 1, 2)
#   D34  : 같은 일정 + 드론 3-4만 학습 (3/30 격차 53%를 정조준)  (시드 1, 2)
# 공통: 자기회귀 + 규칙 정규화 2.0 + 잔여 페널티 0.1, 홀드아웃 80 인스턴스.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5fa_26_09_22_05.log"
T="proposed_src/pipeline/train_v5_26_09_15_23.py"
E="proposed_src/pipeline/eval_v5_26_09_15_23.py"
C="--random-config --residual-penalty 0.1 --rule-reg 2.0 --autoregressive --select makespan --holdout-steps 3000 --eval-n 80 --max-episodes 1500 --ent-frac 0.6 --ent-frac-final 0.25 --ent-anneal-episodes 400"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== 사이클 9 학습 시작 (FA x2, D34 x2) ==="
setsid nohup python3 -u $T --tag v5_26_09_22_05_FAa  $C --drones-range 3 6 --seed 1 >> runs/v5fa_FAa.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_22_05_FAb  $C --drones-range 3 6 --seed 2 >> runs/v5fa_FAb.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_22_05_D34a $C --drones-range 3 4 --seed 1 >> runs/v5fa_D34a.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_22_05_D34b $C --drones-range 3 4 --seed 2 >> runs/v5fa_D34b.log 2>&1 &
sleep 10
say "=== 종료 대기 (최대 20시간) ==="
END=$(( $(date +%s) + 20*3600 ))
while true; do
	N=$(pgrep -fc "tag v5_26_09_22_05_[FD]" || true)
	[ "${N:-0}" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; pkill -f "tag v5_26_09_22_05_[FD]"; sleep 5; break; }
	sleep 180
done
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
L="--arch set --autoregressive --no-cluster-penalty --stochastic --straight-worker --max-steps 10000 --no-progress-limit 1000000000 --deadlock-limit 1000000000"
MG=""
for t in v5_26_09_22_05_FAa v5_26_09_22_05_FAb v5_26_09_22_05_D34a v5_26_09_22_05_D34b; do
	[ -f "weights/$t/best_manager.pth" ] && MG="$MG weights/$t/best_manager.pth"; done
# 현 제안 가중치(사이클 8 ANNa)를 같은 표에 넣어 같은 조건에서 비교한다
[ -f "weights/v5_26_09_21_11_ANNa/best_manager.pth" ] && MG="$MG weights/v5_26_09_21_11_ANNa/best_manager.pth"
say "=== 평가 (a): 표준 4/50, 60시드x2 ==="
python3 -u $E --n 60 --reps 2 $L --manager $MG --out eval_v5fa_std_26_09_22_05 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
for cfg in "3 30 501 d3" "5 50 701 d5"; do
	set -- $cfg
	say "=== 평가: 드론 $1, 목적지 $2, CC 무작위, 40시드 ==="
	python3 -u $E --n 40 $L --num-drones $1 --num-dests $2 --random-cc --seed0 $3 --manager $MG --out eval_v5fa_$4_26_09_22_05 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
done
say "=== 완료 ==="
