#!/usr/bin/env bash
# v5 학습 프로세스가 죽으면 latest.pth에서 이어서 자동 재시작한다.
# CUDA illegal memory access는 이 환경에서 산발적으로 발생하는 GPU 레벨 이벤트다
# (26-08-27 세 실행 동시 사망, 26-09-22 ep1384, 26-10-13 세 실행 동시 사망 ep24-25).
# 사용: supervise_v5_26_10_13_12.sh <tag> [학습 스크립트 인자...]
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TAG="$1"; shift
TRAIN="$ROOT/proposed_src/pipeline/train_v5_26_09_15_23.py"
LOG="$ROOT/runs/sup_$TAG.log"
SUP="$ROOT/runs/sup_${TAG}_supervisor.log"
MAX_RETRY=20
for ((i = 0; i <= MAX_RETRY; i++)); do
	if [ "$i" -eq 0 ]; then RESUME=""; else RESUME="--resume"; fi
	echo "[$(date '+%F %T')] 시도 $i 시작 $RESUME" >> "$SUP"
	python3 -u "$TRAIN" --tag "$TAG" $RESUME "$@" >> "$LOG" 2>&1
	code=$?
	echo "[$(date '+%F %T')] 종료 코드 $code" >> "$SUP"
	[ "$code" -eq 0 ] && { echo "[$(date '+%F %T')] 정상 종료. 감시 종료." >> "$SUP"; break; }
	if [ "$code" -eq 130 ] || [ "$code" -eq 143 ]; then
		echo "[$(date '+%F %T')] 외부 종료 신호. 감시 종료." >> "$SUP"; break
	fi
	echo "[$(date '+%F %T')] 비정상 종료 → 30초 후 재개" >> "$SUP"
	sleep 30
done
