#!/usr/bin/env bash
# 옵션 단위(semi-MDP) 상위 전이를 적용한 뒤의 2단계 재실행.
# 이전 실행은 상위 행동을 1스텝 전이로 저장해 중계처럼 지연 보상이 나는 행동을
# 원리적으로 평가하지 못했다. 그 결과 중계 선택률이 0으로 수렴하고 성능이 무너졌다.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v4_26_09_14_03.log"
T="proposed_src/pipeline/train_v4_26_09_14_03.py"
W="weights/v4_26_09_14_03_w/best_worker.pth"
C="--comm-range 300 --no-cluster-penalty --hl-every 20 --eval-n 12 --seed 1"
M="--stage manager --worker-ckpt $W --rule-manager chain --max-episodes 1500"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }

say "=== 2단계 재실행 (옵션 단위 전이 적용, 4조합) ==="
setsid nohup python3 -u $T --tag v4_26_09_14_03_o_sw   $C $M --straight-worker >> runs/o_sw.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v4_26_09_14_03_o_swc  $C $M --straight-worker --incomplete-penalty 0.05 >> runs/o_swc.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v4_26_09_14_03_o_base $C $M >> runs/o_base.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v4_26_09_14_03_o_ctrl $C $M --rule-manager plain --straight-worker >> runs/o_ctrl.log 2>&1 &
sleep 10

say "=== 종료 대기 (최대 10시간) ==="
END=$(( $(date +%s) + 10*3600 ))
while true; do
	N=$(pgrep -fc "tag v4_26_09_14_03_[o]_" || true)
	[ "${N:-0}" -eq 0 ] && { say "2단계 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; pkill -f "tag v4_26_09_14_03_[o]_"; sleep 5; break; }
	sleep 180
done

say "=== 3단계: 확정 평가 (60시드 x 2회, 샘플링) ==="
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
CK=""; MG=""
for t in v4_26_09_14_03_o_sw v4_26_09_14_03_o_swc v4_26_09_14_03_o_base v4_26_09_14_03_o_ctrl; do
	[ -f "weights/$t/best_manager.pth" ] && { CK="$CK weights/$t/best_worker.pth"; MG="$MG weights/$t/best_manager.pth"; }
done
python3 -u proposed_src/pipeline/eval_v4_26_09_14_03.py --n 60 --reps 2 --no-cluster-penalty \
	--stochastic --max-steps 10000 --no-progress-limit 1000000000 --deadlock-limit 1000000000 \
	--worker $CK --manager $MG --out eval_v4_final_26_09_14_03 2>&1 \
	| grep -v -e pkg -e Hello | tee -a "$LOG"
say "=== 완료 ==="
