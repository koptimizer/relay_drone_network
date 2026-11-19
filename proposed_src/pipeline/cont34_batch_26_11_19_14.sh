#!/usr/bin/env bash
# 사이클 34 배치: 연속 모형을 κ ∈ {10, 5} × M ∈ {9, 10, 11, 12} × 시드 1201–1205 (40건) 에 대해 SCIP 로 푼다.
#   드론 3, 용량 3 (M=9 는 재적재 불필요, 10–12 는 필수), 32각형 바깥 근사, 시간 제한 3,600초, 단일 스레드 10개 병렬.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
mkdir -p runs/cont34
LOG="$ROOT/runs/cont34/batch.log"
echo "[$(date '+%F %T')] 시작" >> "$LOG"
jobs=""
for k in 10 5; do for M in 9 10 11 12; do for s in 1201 1202 1203 1204 1205; do jobs="$jobs $k:$M:$s"; done; done; done
echo $jobs | tr ' ' '\n' | xargs -P 10 -I{} bash -c '
	IFS=: read k M s <<< "{}"
	out=cont34_m${M}s${s}_k${k}
	[ -f figures/$out.json ] && exit 0
	python3 -u proposed_src/pipeline/cont_milp_v5_26_11_19_14.py --solver scip --norm poly --ngon 32 --num-dests $M --seed $s --kappa $k --capacity 3 --time-limit 3600 --threads 1 --out $out > runs/cont34/$out.log 2>&1
	echo "[$(date "+%F %T")] 완료 $out: $(grep "\[결과\]" runs/cont34/$out.log | cut -c1-160)" >> '"$LOG"'
'
echo "[$(date '+%F %T')] === 완료 ===" >> "$LOG"
