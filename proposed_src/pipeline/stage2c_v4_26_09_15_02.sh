#!/usr/bin/env bash
# 복귀 마스킹 수정 후의 2단계 재실행.
# '적재 가득한데 복귀'가 항상 허용되어 정책이 그 무의미 행동으로 수렴했다
# (ep350에서 상위 결정의 51%, 배송 7.25 -> 차단 시 21.75).
# r_ 접두는 이 수정 이후의 실행을 뜻한다.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v4_26_09_14_03.log"
T="proposed_src/pipeline/train_v4_26_09_14_03.py"
W="weights/v4_26_09_14_03_w/best_worker.pth"
C="--comm-range 300 --no-cluster-penalty --hl-every 20 --eval-n 12 --seed 1"
M="--stage manager --worker-ckpt $W --rule-manager chain --max-episodes 1500 --straight-worker"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }

say "=== 2단계 재실행 (복귀 마스킹 수정, 4조합) ==="
setsid nohup python3 -u $T --tag v4_26_09_14_03_r_sw   $C $M >> runs/r_sw.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v4_26_09_14_03_r_swc  $C $M --incomplete-penalty 0.05 >> runs/r_swc.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v4_26_09_14_03_r_norel $C $M --no-relay >> runs/r_norel.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v4_26_09_14_03_r_s2   $C $M --seed 2 >> runs/r_s2.log 2>&1 &
sleep 10

say "=== 종료 대기 (최대 10시간) ==="
END=$(( $(date +%s) + 10*3600 ))
while true; do
	N=$(pgrep -fc "tag v4_26_09_14_03_[r]_" || true)
	[ "${N:-0}" -eq 0 ] && { say "2단계 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; pkill -f "tag v4_26_09_14_03_[r]_"; sleep 5; break; }
	sleep 180
done

say "=== 3단계: 확정 평가 (60시드 x 2회, 샘플링) ==="
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
CK=""; MG=""
for t in v4_26_09_14_03_r_sw v4_26_09_14_03_r_swc v4_26_09_14_03_r_norel v4_26_09_14_03_r_s2; do
	[ -f "weights/$t/best_manager.pth" ] && { CK="$CK weights/$t/best_worker.pth"; MG="$MG weights/$t/best_manager.pth"; }
done
python3 -u proposed_src/pipeline/eval_v4_26_09_14_03.py --n 60 --reps 2 --no-cluster-penalty \
	--stochastic --max-steps 10000 --no-progress-limit 1000000000 --deadlock-limit 1000000000 \
	--worker $CK --manager $MG --out eval_v4_final_26_09_15_02 2>&1 \
	| grep -v -e pkg -e Hello | tee -a "$LOG"
say "=== 완료 ==="
