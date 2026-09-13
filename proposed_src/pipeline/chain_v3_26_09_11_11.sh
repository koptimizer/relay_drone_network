#!/usr/bin/env bash
# 재부팅으로 끊긴 mk 학습을 재개하고, 전량 완주(makespan) 전용 조합을 함께 돌린 뒤
# 네 조합 전부를 makespan 측정이 가능한 여유 상한으로 평가한다.
#   mk   : 상한 10000, gamma 0.99, 무배송 800  → ep1101부터 재개 (기존 최고 26.9)
#   mk2k : 상한 10000, gamma 0.99, 무배송 2000 → 무배송 한도만 올려 완주를 노림
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_26_09_11_11.log"
SUP="proposed_src/pipeline/supervise_v3_26_09_07_17.sh"
W="weights/v3_26_09_07_17_w/best_worker.pth"
C="--comm-range 300 --no-cluster-penalty --hl-every 20 --eval-n 12 --stage manager --worker-ckpt $W"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }

say "=== 학습 시작 (mk 재개 + mk2k 신규) ==="
setsid nohup $SUP v3_26_09_10_12_mk $C --seed 1 --max-steps 10000 --gamma 0.99 \
	--no-progress-limit 800 --max-episodes 1900 --resume >/dev/null 2>&1 &
sleep 10
setsid nohup $SUP v3_26_09_11_11_mk2k $C --seed 1 --max-steps 10000 --gamma 0.99 \
	--no-progress-limit 2000 --warmup-no-progress 2000 --max-episodes 1000 >/dev/null 2>&1 &
sleep 10

say "=== 종료 대기 (최대 20시간) ==="
END=$(( $(date +%s) + 20*3600 ))
while true; do
	N=$(ps -ef | grep -c "[p]ipeline/train_v3")
	[ "$N" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; \
		ps -ef | grep -E "[p]ipeline/train_v3|[s]upervise_v3" | awk '{print $2}' | xargs -r kill; sleep 5; break; }
	sleep 180
done

say "=== 최종 평가 (makespan 측정용 여유 상한) ==="
CK=""; MG=""
for t in v3_26_09_10_12_g99 v3_26_09_10_12_s1k v3_26_09_10_12_mk v3_26_09_11_11_mk2k; do
	[ -f "weights/$t/best_worker.pth" ] && { CK="$CK weights/$t/best_worker.pth"; MG="$MG weights/$t/best_manager.pth"; }
done
python3 -u proposed_src/pipeline/eval_v3_26_09_11_11.py --n 60 --reps 5 --no-cluster-penalty \
	--max-steps 30000 --no-progress-limit 4000 --deadlock-limit 300 \
	--worker $CK --manager $MG --out eval_v3_ablate_26_09_11_11 2>&1 \
	| grep -v -e pkg -e Hello | tee -a "$LOG"
say "=== 완료 ==="
