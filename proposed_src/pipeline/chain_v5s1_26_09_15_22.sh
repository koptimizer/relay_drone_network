#!/usr/bin/env bash
# v5 1단계: 인스턴스 무작위화 + 후보 다양화 + 목표 유지 + 무배송 500 종료.
# 시드 2개를 병렬로 돌리고, 끝나면 (a) 표준 구성 (b) 학습에서 본 적 없는 구성으로 평가한다.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5s1_26_09_15_22.log"
T="proposed_src/pipeline/train_v5_26_09_15_22.py"
E="proposed_src/pipeline/eval_v5_26_09_15_22.py"
W="weights/v4_26_09_14_03_w/best_worker.pth"
C="--comm-range 300 --no-cluster-penalty --hl-every 20 --eval-n 12 --stage manager --worker-ckpt $W --rule-manager chain --max-episodes 1500"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }

say "=== v5 1단계 학습 시작 (시드 1, 2) ==="
setsid nohup python3 -u $T --tag v5_26_09_15_22_s1 $C --seed 1 >> runs/v5s1_s1.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_15_22_s2 $C --seed 2 >> runs/v5s1_s2.log 2>&1 &
sleep 10

say "=== 종료 대기 (최대 10시간) ==="
END=$(( $(date +%s) + 10*3600 ))
while true; do
	N=$(pgrep -fc "tag v5_26_09_15_22_[s]" || true)
	[ "${N:-0}" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; pkill -f "tag v5_26_09_15_22_[s]"; sleep 5; break; }
	sleep 180
done

export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
CK=""; MG=""
for t in v5_26_09_15_22_s1 v5_26_09_15_22_s2; do
	[ -f "weights/$t/best_manager.pth" ] && { CK="$CK weights/$t/best_worker.pth"; MG="$MG weights/$t/best_manager.pth"; }
done
L="--no-cluster-penalty --stochastic --straight-worker --max-steps 10000 --no-progress-limit 1000000000 --deadlock-limit 1000000000"

say "=== 평가 (a): 표준 구성 — 드론 4, 목적지 50, CC 고정, 60시드x2 ==="
python3 -u $E --n 60 --reps 2 $L --worker $CK --manager $MG --out eval_v5s1_std_26_09_15_22 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"

say "=== 평가 (b): 미지 구성 — 목적지 30, CC 무작위, 40시드 ==="
python3 -u $E --n 40 --reps 1 $L --num-dests 30 --random-cc --seed0 501 --worker $CK --manager $MG --out eval_v5s1_d30cc_26_09_15_22 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"

say "=== 평가 (c): 미지 구성 — 목적지 80, CC 무작위, 40시드 ==="
python3 -u $E --n 40 --reps 1 $L --num-dests 80 --random-cc --seed0 601 --worker $CK --manager $MG --out eval_v5s1_d80cc_26_09_15_22 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
say "=== 완료 ==="
