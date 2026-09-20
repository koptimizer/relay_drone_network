#!/usr/bin/env bash
# makespan 사이클 7: 무작위성 없이 탈출을 배우게 한다.
#   사이클 6(엔트로피 목표 0.3/0.45)은 속도를 970-1,040으로 당겼으나 완주가 0.93-1.00 사이를 오갔다.
#   정체 중(이동 실현율 EMA<0.3, 목표 20스텝 이상)인 드론이 목표를 바꾸면 팀 보상 0.5를 준다.
#   S03  : 엔트로피 0.3  + 정체 전환 보상 0.5 (시드 1, 2)
#   S045 : 엔트로피 0.45 + 정체 전환 보상 0.5 (시드 1, 2)
# 공통: 사이클 5의 ARb 구성. 사이클 6 체인이 끝난 뒤(GPU 확보) 자동 시작한다.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5esc_26_09_20_06.log"
PREV="$ROOT/runs/chain_v5ent_26_09_19_12.log"
T="proposed_src/pipeline/train_v5_26_09_15_23.py"
E="proposed_src/pipeline/eval_v5_26_09_15_23.py"
C="--random-config --drones-range 3 6 --residual-penalty 0.1 --rule-reg 2.0 --autoregressive --select makespan --holdout-steps 3000 --eval-n 40 --max-episodes 1500"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== 사이클 6 체인 종료 대기 ==="
while ! grep -q "=== 완료" "$PREV" && pgrep -f "chain_v5ent_26_09_19_1[2]" >/dev/null; do sleep 300; done
say "=== 사이클 7 학습 시작 (S03 x2, S045 x2) ==="
setsid nohup python3 -u $T --tag v5_26_09_20_06_S03a  $C --ent-frac 0.3  --stall-switch-bonus 0.5 --seed 1 >> runs/v5esc_S03a.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_20_06_S03b  $C --ent-frac 0.3  --stall-switch-bonus 0.5 --seed 2 >> runs/v5esc_S03b.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_20_06_S045a $C --ent-frac 0.45 --stall-switch-bonus 0.5 --seed 1 >> runs/v5esc_S045a.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_20_06_S045b $C --ent-frac 0.45 --stall-switch-bonus 0.5 --seed 2 >> runs/v5esc_S045b.log 2>&1 &
sleep 10
say "=== 종료 대기 (최대 18시간) ==="
END=$(( $(date +%s) + 18*3600 ))
while true; do
	N=$(pgrep -fc "tag v5_26_09_20_06_[S]" || true)
	[ "${N:-0}" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; pkill -f "tag v5_26_09_20_06_[S]"; sleep 5; break; }
	sleep 180
done
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
L="--arch set --autoregressive --no-cluster-penalty --stochastic --straight-worker --max-steps 10000 --no-progress-limit 1000000000 --deadlock-limit 1000000000"
MG=""
for t in v5_26_09_20_06_S03a v5_26_09_20_06_S03b v5_26_09_20_06_S045a v5_26_09_20_06_S045b; do
	[ -f "weights/$t/best_manager.pth" ] && MG="$MG weights/$t/best_manager.pth"; done
say "=== 평가 (a): 표준 4/50, 60시드x2 ==="
python3 -u $E --n 60 --reps 2 $L --manager $MG --out eval_v5esc_std_26_09_20_06 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
for cfg in "3 30 501 d3" "5 50 701 d5"; do
	set -- $cfg
	say "=== 평가: 드론 $1, 목적지 $2, CC 무작위, 40시드 ==="
	python3 -u $E --n 40 $L --num-drones $1 --num-dests $2 --random-cc --seed0 $3 --manager $MG --out eval_v5esc_$4_26_09_20_06 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
done
say "=== 완료 ==="
