#!/usr/bin/env bash
# makespan 사이클 6: 강제 무작위성을 걷어내고 탈출을 정책이 배우게 한다.
#   26-09-19 온도 프로브: 엔트로피 목표 0.6·log n 아래서 alpha가 2~3에 머물러 정책이 흐트러지고,
#   추론 온도를 내리면 교착이 드러난다 (탈출이 무작위성에 의존). 정체 신호는 이미 self 관측에 있다.
#   E03  : 엔트로피 목표 0.3·log n   (시드 1, 2)
#   E045 : 엔트로피 목표 0.45·log n  (시드 1, 2)
# 공통: 사이클 5의 ARb 구성 (자기회귀 + 규칙 정규화 2.0 + 잔여 페널티 0.1, 드론 3-6, 홀드아웃 40 x 3000).
# 사이클 5 체인이 끝난 뒤(GPU 확보) 자동 시작한다.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5ent_26_09_19_12.log"
PREV="$ROOT/runs/chain_v5ar_26_09_19_03.log"
T="proposed_src/pipeline/train_v5_26_09_15_23.py"
E="proposed_src/pipeline/eval_v5_26_09_15_23.py"
C="--random-config --drones-range 3 6 --residual-penalty 0.1 --rule-reg 2.0 --autoregressive --select makespan --holdout-steps 3000 --eval-n 40 --max-episodes 1500"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== 사이클 5 체인 종료 대기 ==="
while ! grep -q "=== 완료" "$PREV" && pgrep -f "chain_v5ar_26_09_19_0[3]" >/dev/null; do sleep 300; done
say "=== 사이클 6 학습 시작 (E03 x2, E045 x2) ==="
setsid nohup python3 -u $T --tag v5_26_09_19_12_E03a  $C --ent-frac 0.3  --seed 1 >> runs/v5ent_E03a.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_19_12_E03b  $C --ent-frac 0.3  --seed 2 >> runs/v5ent_E03b.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_19_12_E045a $C --ent-frac 0.45 --seed 1 >> runs/v5ent_E045a.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_19_12_E045b $C --ent-frac 0.45 --seed 2 >> runs/v5ent_E045b.log 2>&1 &
sleep 10
say "=== 종료 대기 (최대 18시간) ==="
END=$(( $(date +%s) + 18*3600 ))
while true; do
	N=$(pgrep -fc "tag v5_26_09_19_12_[E]" || true)
	[ "${N:-0}" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; pkill -f "tag v5_26_09_19_12_[E]"; sleep 5; break; }
	sleep 180
done
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
L="--arch set --autoregressive --no-cluster-penalty --stochastic --straight-worker --max-steps 10000 --no-progress-limit 1000000000 --deadlock-limit 1000000000"
MG=""
for t in v5_26_09_19_12_E03a v5_26_09_19_12_E03b v5_26_09_19_12_E045a v5_26_09_19_12_E045b; do
	[ -f "weights/$t/best_manager.pth" ] && MG="$MG weights/$t/best_manager.pth"; done
say "=== 평가 (a): 표준 4/50, 60시드x2 ==="
python3 -u $E --n 60 --reps 2 $L --manager $MG --out eval_v5ent_std_26_09_19_12 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
for cfg in "3 30 501 d3" "5 50 701 d5"; do
	set -- $cfg
	say "=== 평가: 드론 $1, 목적지 $2, CC 무작위, 40시드 ==="
	python3 -u $E --n 40 $L --num-drones $1 --num-dests $2 --random-cc --seed0 $3 --manager $MG --out eval_v5ent_$4_26_09_19_12 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
done
say "=== 완료 ==="
