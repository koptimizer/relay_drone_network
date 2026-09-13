#!/usr/bin/env bash
# 학습 두 건이 모두 끝나기를 기다렸다가 평가 두 종류를 이어서 돌린다.
#   1) 동일 조건 비교 — 기존 상한(무배송 200)으로 네 조합의 배송량을 맞대어 잰다
#   2) makespan 측정 — 여유 상한(무배송 4000)으로 전량 완주 여부와 완주 시간을 잰다
# 여유 상한 롤아웃은 에피소드가 10배 이상 길어 롤아웃 수를 따로 줄여 잡는다.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_26_09_11_11.log"
# 작은 MLP를 배치 4로 도는 평가라 스레드를 늘려도 이득이 없고 학습만 굶는다.
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }

say "=== 종료 대기 (최대 20시간) ==="
END=$(( $(date +%s) + 20*3600 ))
while true; do
	N=$(ps -ef | grep -c "[p]ipeline/train_v3")
	[ "$N" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; \
		ps -ef | grep -E "[p]ipeline/train_v3|[s]upervise_v3" | awk '{print $2}' | xargs -r kill; sleep 5; break; }
	sleep 180
done

CK=""; MG=""
for t in v3_26_09_10_12_g99 v3_26_09_10_12_s1k v3_26_09_10_12_mk v3_26_09_10_12_mk_ep850 v3_26_09_11_11_mk2k; do
	[ -f "weights/$t/best_worker.pth" ] && { CK="$CK weights/$t/best_worker.pth"; MG="$MG weights/$t/best_manager.pth"; }
done

say "=== 평가 1/2: 동일 조건 비교 (300 롤아웃) ==="
python3 -u proposed_src/pipeline/eval_v3_26_09_11_11.py --n 60 --reps 5 --no-cluster-penalty \
	--max-steps 10000 --no-progress-limit 200 --deadlock-limit 100 \
	--worker $CK --manager $MG --out eval_v3_ablate_26_09_11_11 2>&1 \
	| grep -v -e pkg -e Hello | tee -a "$LOG"

say "=== 평가 2/2: makespan 측정 (여유 상한, 25 롤아웃) ==="
python3 -u proposed_src/pipeline/eval_v3_26_09_11_11.py --n 25 --reps 1 --no-cluster-penalty \
	--max-steps 30000 --no-progress-limit 4000 --deadlock-limit 300 \
	--worker $CK --manager $MG --out eval_v3_makespan_26_09_11_11 2>&1 \
	| grep -v -e pkg -e Hello | tee -a "$LOG"
say "=== 완료 ==="
