#!/usr/bin/env bash
# makespan 사이클 11: 선택 점수 수정과 드론 가중 표본으로 두 구성의 장점을 합친다.
#   사이클 10: 드론 3대 전용 학습이 3/30 격차를 17% -> 10%로 줄였으나 표준 4/50은 사이클 9에 못 미쳤다.
#   또 선택 점수가 완주를 이진 문턱처럼 써서 850스텝짜리 체크포인트들이 버려졌다(--speed-weight로 수정).
#   W31 : 드론 3-4를 3:1로 표본 (3대 75%), 속도 가중치 40   (시드 1, 2)
#   SW   : 드론 3-4 균등 (사이클 9 구성), 속도 가중치 40     (시드 1, 2)
# SW는 선택 점수 수정만의 효과를 사이클 9와 직접 비교하는 대조군이다.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5w3_26_09_23_18.log"
T="proposed_src/pipeline/train_v5_26_09_15_23.py"
E="proposed_src/pipeline/eval_v5_26_09_15_23.py"
C="--random-config --drones-range 3 4 --residual-penalty 0.1 --rule-reg 2.0 --autoregressive --select makespan --speed-weight 40 --holdout-steps 3000 --eval-n 80 --max-episodes 1500 --ent-frac 0.6 --ent-frac-final 0.25 --ent-anneal-episodes 400"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== 사이클 11 학습 시작 (W31 x2, SW x2) ==="
setsid nohup python3 -u $T --tag v5_26_09_23_18_W31a $C --drone-weights 3 1 --seed 1 >> runs/v5w3_W31a.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_23_18_W31b $C --drone-weights 3 1 --seed 2 >> runs/v5w3_W31b.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_23_18_SWa  $C --seed 1 >> runs/v5w3_SWa.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_23_18_SWb  $C --seed 2 >> runs/v5w3_SWb.log 2>&1 &
sleep 10
say "=== 종료 대기 (최대 20시간) ==="
END=$(( $(date +%s) + 20*3600 ))
while true; do
	N=$(pgrep -fc "tag v5_26_09_23_18_[WS]" || true)
	[ "${N:-0}" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; pkill -f "tag v5_26_09_23_18_[WS]"; sleep 5; break; }
	sleep 180
done
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
L="--arch set --autoregressive --no-cluster-penalty --stochastic --straight-worker --max-steps 10000 --no-progress-limit 1000000000 --deadlock-limit 1000000000"
MG=""
for t in v5_26_09_23_18_W31a v5_26_09_23_18_W31b v5_26_09_23_18_SWa v5_26_09_23_18_SWb; do
	[ -f "weights/$t/best_manager.pth" ] && MG="$MG weights/$t/best_manager.pth"; done
# 사이클 10 최선(D3b)과 사이클 9 제안(D34a)을 같은 표에 넣어 비교하고 반복 측정도 겸한다
for t in v5_26_09_23_00_D3b v5_26_09_22_05_D34a; do
	[ -f "weights/$t/best_manager.pth" ] && MG="$MG weights/$t/best_manager.pth"; done
say "=== 평가 (a): 표준 4/50, 60시드x2 ==="
python3 -u $E --n 60 --reps 2 $L --manager $MG --out eval_v5w3_std_26_09_23_18 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
for cfg in "3 30 501 d3" "5 50 701 d5"; do
	set -- $cfg
	say "=== 평가: 드론 $1, 목적지 $2, CC 무작위, 40시드 ==="
	python3 -u $E --n 40 $L --num-drones $1 --num-dests $2 --random-cc --seed0 $3 --manager $MG --out eval_v5w3_$4_26_09_23_18 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
done
say "=== 완료 ==="
