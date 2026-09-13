#!/usr/bin/env bash
# v3(26_09_04_15) 학습 감시 — 비정상 종료 시 latest.pth에서 자동 재개한다.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TAG="$1"; shift
TRAIN="$ROOT/proposed_src/pipeline/train_v3_26_09_07_17.py"
LOG="$ROOT/runs/$TAG/train_$TAG.log"; SUP="$ROOT/runs/$TAG/supervisor_$TAG.log"
mkdir -p "$ROOT/runs/$TAG"
for ((i = 0; i <= 20; i++)); do
	[ "$i" -eq 0 ] && RESUME="" || RESUME="--resume"
	echo "[$(date '+%F %T')] 시도 $i $RESUME" >> "$SUP"
	python3 -u "$TRAIN" --tag "$TAG" $RESUME "$@" >> "$LOG" 2>&1
	c=$?; echo "[$(date '+%F %T')] 종료 코드 $c" >> "$SUP"
	{ [ "$c" -eq 0 ] || [ "$c" -eq 130 ] || [ "$c" -eq 143 ]; } && break
	echo "[$(date '+%F %T')] 비정상 종료 → 30초 후 재개" >> "$SUP"; sleep 30
done
