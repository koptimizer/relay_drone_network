#!/usr/bin/env bash
# makespan 사이클 8: 엔트로피 목표를 더 낮추고, 감쇠로 탐색과 활용을 분리한다.
#   사이클 6에서 0.6 -> 0.3이 표준 makespan을 1,067 -> 955로 내렸고 0.3이 0.45보다 좋았다.
#   사이클 7의 정체 전환 보상은 역효과였으므로 끈다. 외삽과 감쇠 두 갈래를 같이 본다.
#   L015 : 엔트로피 목표 0.15 고정            (시드 1, 2)
#   ANN  : 0.6 -> 0.15 선형 감쇠              (시드 1, 2)
# 공통: 자기회귀 + 규칙 정규화 2.0 + 잔여 페널티 0.1, 드론 3-6.
# 홀드아웃을 40 -> 80 인스턴스로 늘린다. 홀드아웃 885가 정식 979로, 901이 1,008-1,124로
# 뒤집히는 선택 잡음이 반복됐다 (26-09-20, 26-09-21).
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5ent2_26_09_21_11.log"
T="proposed_src/pipeline/train_v5_26_09_15_23.py"
E="proposed_src/pipeline/eval_v5_26_09_15_23.py"
C="--random-config --drones-range 3 6 --residual-penalty 0.1 --rule-reg 2.0 --autoregressive --select makespan --holdout-steps 3000 --eval-n 80 --max-episodes 1500"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== 사이클 8 학습 시작 (L015 x2, ANN x2) ==="
setsid nohup python3 -u $T --tag v5_26_09_21_11_L015a $C --ent-frac 0.15 --seed 1 >> runs/v5e2_L015a.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_21_11_L015b $C --ent-frac 0.15 --seed 2 >> runs/v5e2_L015b.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_21_11_ANNa $C --ent-frac 0.6 --ent-frac-final 0.15 --seed 1 >> runs/v5e2_ANNa.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_21_11_ANNb $C --ent-frac 0.6 --ent-frac-final 0.15 --seed 2 >> runs/v5e2_ANNb.log 2>&1 &
sleep 10
say "=== 종료 대기 (최대 20시간) ==="
END=$(( $(date +%s) + 20*3600 ))
while true; do
	N=$(pgrep -fc "tag v5_26_09_21_11_[LA]" || true)
	[ "${N:-0}" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; pkill -f "tag v5_26_09_21_11_[LA]"; sleep 5; break; }
	sleep 180
done
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
L="--arch set --autoregressive --no-cluster-penalty --stochastic --straight-worker --max-steps 10000 --no-progress-limit 1000000000 --deadlock-limit 1000000000"
MG=""
for t in v5_26_09_21_11_L015a v5_26_09_21_11_L015b v5_26_09_21_11_ANNa v5_26_09_21_11_ANNb; do
	[ -f "weights/$t/best_manager.pth" ] && MG="$MG weights/$t/best_manager.pth"; done
# 사이클 6의 제안 가중치를 같은 표에 넣어 직접 비교한다
[ -f "weights/v5_26_09_19_12_E03b/best_manager.pth" ] && MG="$MG weights/v5_26_09_19_12_E03b/best_manager.pth"
say "=== 평가 (a): 표준 4/50, 60시드x2 ==="
python3 -u $E --n 60 --reps 2 $L --manager $MG --out eval_v5ent2_std_26_09_21_11 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
for cfg in "3 30 501 d3" "5 50 701 d5"; do
	set -- $cfg
	say "=== 평가: 드론 $1, 목적지 $2, CC 무작위, 40시드 ==="
	python3 -u $E --n 40 $L --num-drones $1 --num-dests $2 --random-cc --seed0 $3 --manager $MG --out eval_v5ent2_$4_26_09_21_11 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
done
say "=== 완료 ==="
