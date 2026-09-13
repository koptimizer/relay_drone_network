#!/usr/bin/env bash
# 세 조합을 병렬 학습 → 전부 종료 대기 → 300 롤아웃 평가까지 자동 실행한다.
#   g99 : 상한 10000, gamma 0.99   → 상한 10배의 효과만 분리
#   s1k : 상한  1000, gamma 0.997  → gamma 상향의 효과만 분리
#   mk  : 상한 10000, gamma 0.99, 무배송 한도 800 → 전량 완주(makespan) 달성 목표
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_26_09_10_12.log"
SUP="proposed_src/pipeline/supervise_v3_26_09_07_17.sh"
W="weights/v3_26_09_07_17_w/best_worker.pth"
C="--comm-range 300 --no-cluster-penalty --hl-every 20 --eval-n 12 --stage manager --worker-ckpt $W"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }

say "=== 학습 시작 (3조합) ==="
setsid nohup $SUP v3_26_09_10_12_g99 $C --seed 1 --max-steps 10000 --gamma 0.99  --no-progress-limit 200 >/dev/null 2>&1 &
sleep 10
setsid nohup $SUP v3_26_09_10_12_s1k $C --seed 1 --max-steps 1000  --gamma 0.997 --no-progress-limit 200 >/dev/null 2>&1 &
sleep 10
setsid nohup $SUP v3_26_09_10_12_mk  $C --seed 1 --max-steps 10000 --gamma 0.99  --no-progress-limit 800 >/dev/null 2>&1 &
sleep 10

say "=== 종료 대기 (최대 14시간) ==="
END=$(( $(date +%s) + 14*3600 ))
while true; do
	N=$(ps -ef | grep -c "[p]ipeline/train_v3")
	[ "$N" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; \
		ps -ef | grep -E "[p]ipeline/train_v3|[s]upervise_v3" | awk '{print $2}' | xargs -r kill; sleep 5; break; }
	sleep 180
done

say "=== 최종 평가 ==="
CK=""; MG=""
for t in v3_26_09_10_12_g99 v3_26_09_10_12_s1k v3_26_09_10_12_mk; do
	[ -f "weights/$t/best_worker.pth" ] && { CK="$CK weights/$t/best_worker.pth"; MG="$MG weights/$t/best_manager.pth"; }
done
python3 -u proposed_src/pipeline/eval_v3_26_09_07_17.py --n 60 --reps 5 --no-cluster-penalty \
	--worker $CK --manager $MG --out eval_v3_ablate_26_09_10_12 2>&1 \
	| grep -v -e pkg -e Hello | tee -a "$LOG"
say "=== 완료 ==="
