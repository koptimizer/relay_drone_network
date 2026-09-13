#!/usr/bin/env bash
# 2단계 학습(시드 2개) → 종료 대기 → 300 롤아웃 평가까지 자동으로 이어서 실행한다.
# 이전 두 사이클에서 단계 사이 연결을 걸지 않아 학습이 멈춰 있던 일을 막기 위한 것이다.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_26_09_09_16.log"
SUP="proposed_src/pipeline/supervise_v3_26_09_07_17.sh"
W="weights/v3_26_09_07_17_w/best_worker.pth"
C="--comm-range 300 --no-cluster-penalty --hl-every 20 --eval-n 12"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }

say "=== 2단계 학습 시작 (시드 2개) ==="
for s in 1 2; do
	setsid nohup $SUP "v3_26_09_07_17_m_s$s" --stage manager $C --worker-ckpt "$W" --seed "$s" >/dev/null 2>&1 &
	sleep 10
done

say "=== 종료 대기 (최대 12시간) ==="
END=$(( $(date +%s) + 12*3600 ))
while true; do
	N=$(ps -ef | grep -c "[p]ipeline/train_v3")
	[ "$N" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; \
		ps -ef | grep -E "[p]ipeline/train_v3|[s]upervise_v3" | awk '{print $2}' | xargs -r kill; sleep 5; break; }
	sleep 120
done

say "=== 최종 평가 (60 인스턴스 × 5회) ==="
CK=""; MG=""
for t in v3_26_09_07_17_m_s1 v3_26_09_07_17_m_s2; do
	[ -f "weights/$t/best_worker.pth" ] && { CK="$CK weights/$t/best_worker.pth"; MG="$MG weights/$t/best_manager.pth"; }
done
python3 -u proposed_src/pipeline/eval_v3_26_09_07_17.py --n 60 --reps 5 --no-cluster-penalty \
	--worker $CK --manager $MG --out eval_v3_longep_26_09_07_17 2>&1 \
	| grep -v -e pkg -e Hello | tee -a "$LOG"
say "=== 완료 ==="
