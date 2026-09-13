#!/usr/bin/env bash
# 확률 샘플링 추론이 교착을 없애는 것이 20 롤아웃에서 관찰되어, 정식 표본으로 확인한다.
# 최선 조합(term)과 이전 챔피언(26_09_04_15)을 결정론 / 샘플링 두 방식으로 각각 잰다.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/stoch_26_09_12_19.log"
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
E="python3 -u proposed_src/pipeline/eval_v3_26_09_11_11.py"
CK=""; MG=""
for t in v3_26_09_12_09_term v3_26_09_04_15_m_s1 v3_26_09_04_15_m_s2; do
	CK="$CK weights/$t/best_worker.pth"; MG="$MG weights/$t/best_manager.pth"; done
L="--max-steps 30000 --no-progress-limit 4000 --deadlock-limit 300"

say "=== 1/2: 여유 예산 120 롤아웃, 결정론 추론 (대조군) ==="
$E --n 60 --reps 2 --no-cluster-penalty $L --worker $CK --manager $MG \
	--out eval_v3_det120_26_09_12_19 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"

say "=== 2/2: 여유 예산 120 롤아웃, 확률 샘플링 추론 ==="
$E --n 60 --reps 2 --no-cluster-penalty $L --stochastic --worker $CK --manager $MG \
	--out eval_v3_stoch120_26_09_12_19 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
say "=== 완료 ==="
