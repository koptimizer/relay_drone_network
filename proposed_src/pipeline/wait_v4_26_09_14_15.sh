#!/usr/bin/env bash
# v4 2단계 5조합이 모두 끝나기를 기다렸다가 확정 평가를 돌린다.
#   base/comp/ctrl : 학습 하위 위에서 상위 학습
#   sw/swcomp      : 직진 하위로 고정하고 상위만 학습 (하위 성능 교란 제거)
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v4_26_09_14_03.log"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }

say "=== 2단계 종료 대기 (5조합, 최대 10시간) ==="
END=$(( $(date +%s) + 10*3600 ))
while true; do
	N=$(ps -ef | grep -c "[t]rain_v4_26_09_14_03.py")
	[ "$N" -eq 0 ] && { say "2단계 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; \
		ps -ef | grep "[t]rain_v4_26_09_14_03.py" | awk '{print $2}' | xargs -r kill; sleep 5; break; }
	sleep 180
done

say "=== 3단계: 확정 평가 (60시드 x 2회, 샘플링, 완주 측정) ==="
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
CK=""; MG=""
for t in v4_26_09_14_03_m_base v4_26_09_14_03_m_comp v4_26_09_14_03_m_ctrl \
         v4_26_09_14_03_m_sw v4_26_09_14_03_m_swcomp; do
	[ -f "weights/$t/best_manager.pth" ] && { CK="$CK weights/$t/best_worker.pth"; MG="$MG weights/$t/best_manager.pth"; }
done
python3 -u proposed_src/pipeline/eval_v4_26_09_14_03.py --n 60 --reps 2 --no-cluster-penalty \
	--stochastic --max-steps 10000 --no-progress-limit 1000000000 --deadlock-limit 1000000000 \
	--worker $CK --manager $MG --out eval_v4_final_26_09_14_03 2>&1 \
	| grep -v -e pkg -e Hello | tee -a "$LOG"
say "=== 완료 ==="
