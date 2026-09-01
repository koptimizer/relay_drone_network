#!/usr/bin/env bash
# 학습 프로세스가 죽으면 latest.pth에서 이어서 자동 재시작한다.
# CUDA illegal memory access처럼 산발적으로 발생하는 GPU 오류로 밤사이 실행을 잃지 않기 위함.
# 사용: supervise_v2_26_08_25_14.sh <tag> [학습 스크립트 인자...]
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TAG="$1"; shift
TRAIN="$ROOT/proposed_src/pipeline/train_v2_26_08_25_14.py"
LOG="$ROOT/runs/$TAG/train_$TAG.log"
SUP="$ROOT/runs/$TAG/supervisor_$TAG.log"
MAX_RETRY=20
mkdir -p "$ROOT/runs/$TAG"

for ((i = 0; i <= MAX_RETRY; i++)); do
	if [ "$i" -eq 0 ]; then RESUME=""; else RESUME="--resume"; fi
	echo "[$(date '+%F %T')] 시도 $i 시작 $RESUME" >> "$SUP"
	python3 -u "$TRAIN" --tag "$TAG" $RESUME "$@" >> "$LOG" 2>&1
	code=$?
	echo "[$(date '+%F %T')] 종료 코드 $code" >> "$SUP"

	# 정상 종료(조기 종료 포함)이면 감시를 끝낸다
	if [ "$code" -eq 0 ]; then
		echo "[$(date '+%F %T')] 정상 종료. 감시 종료." >> "$SUP"
		break
	fi
	# 사용자가 kill한 경우(SIGTERM/SIGINT)도 재시작하지 않는다
	if [ "$code" -eq 130 ] || [ "$code" -eq 143 ]; then
		echo "[$(date '+%F %T')] 외부 종료 신호. 감시 종료." >> "$SUP"
		break
	fi
	echo "[$(date '+%F %T')] 비정상 종료 → 30초 후 재개" >> "$SUP"
	sleep 30
done
