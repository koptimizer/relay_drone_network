#!/usr/bin/env bash
# 사이클 17 학습이 끝나는 즉시 사이클 18을 시작한다 (평가는 CPU라 GPU가 빈다).
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5jc_26_10_01_00.log"
echo "[$(date '+%F %T')] 사이클 17 학습 종료 대기 중" | tee -a "$LOG"
END=$(( $(date +%s) + 30*3600 ))
while true; do
	N=$(pgrep -fc "tag v5_26_10_01_00_MH" || true)
	[ "${N:-0}" -eq 0 ] && break
	[ "$(date +%s)" -ge "$END" ] && { echo "[$(date '+%F %T')] 대기 시간 초과" | tee -a "$LOG"; exit 1; }
	sleep 120
done
echo "[$(date '+%F %T')] 사이클 17 학습이 끝났다 — 사이클 18 시작" | tee -a "$LOG"
exec "$ROOT/proposed_src/pipeline/chain_v5jc_26_10_01_00.sh"
