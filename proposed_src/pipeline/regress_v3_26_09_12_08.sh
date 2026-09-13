#!/usr/bin/env bash
# 긴 에피소드 전환이 회귀인지 가리기 위해, 이전 v3 체크포인트를 이번 조합과
# 똑같은 프로토콜로 두 예산에서 다시 잰다. 이어서 여유 예산 표본을 키운다.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/regress_26_09_12_08.log"
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
E="python3 -u proposed_src/pipeline/eval_v3_26_09_11_11.py"
# 이전 챔피언 (모두 43차원 관측) + 이번 최선 조합을 한 표에 놓는다
P="v3_26_09_04_15_m_s1 v3_26_09_04_15_m_s2 v3_26_09_07_17_m_s1 v3_26_09_07_17_m_s2 v3_26_09_10_12_g99"
CK=""; MG=""
for t in $P; do CK="$CK weights/$t/best_worker.pth"; MG="$MG weights/$t/best_manager.pth"; done

say "=== 1/3: 이전 챔피언, 좁은 예산 (무배송 200, 300 롤아웃) ==="
$E --n 60 --reps 5 --no-cluster-penalty --max-steps 10000 --no-progress-limit 200 \
	--deadlock-limit 100 --worker $CK --manager $MG \
	--out eval_v3_prior_tight_26_09_12_08 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"

say "=== 2/3: 이전 챔피언, 여유 예산 (무배송 4000, 120 롤아웃) ==="
$E --n 60 --reps 2 --no-cluster-penalty --max-steps 30000 --no-progress-limit 4000 \
	--deadlock-limit 300 --worker $CK --manager $MG \
	--out eval_v3_prior_loose_26_09_12_08 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"

say "=== 3/3: 이번 조합, 여유 예산 표본 확대 (120 롤아웃) ==="
CK2=""; MG2=""
for t in v3_26_09_10_12_g99 v3_26_09_10_12_s1k v3_26_09_10_12_mk_ep850 v3_26_09_11_11_mk2k; do
	CK2="$CK2 weights/$t/best_worker.pth"; MG2="$MG2 weights/$t/best_manager.pth"; done
$E --n 60 --reps 2 --no-cluster-penalty --max-steps 30000 --no-progress-limit 4000 \
	--deadlock-limit 300 --worker $CK2 --manager $MG2 \
	--out eval_v3_loose120_26_09_12_08 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
say "=== 완료 ==="
