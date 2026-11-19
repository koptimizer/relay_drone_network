#!/usr/bin/env bash
# 사이클 34 배치 (Gurobi, 정확 MISOCP): κ ∈ {10, 5} × M ∈ {9, 10, 11, 12} × 시드 1201–1205 (40건).
#   드론 3, 용량 3, 64각형 바깥 근사 (반지름 오차 0.12%), 시간 제한 3,600초, 4 스레드 × 4 병렬. 라이선스 갱신(26-11-19) 뒤 SCIP 배치를 대체한다.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
mkdir -p runs/cont34
LOG="$ROOT/runs/cont34/batch.log"
echo "[$(date '+%F %T')] Gurobi 64각형 배치 시작" >> "$LOG"
jobs=""
for k in 10 5; do for M in 9 10 11 12; do for s in 1201 1202 1203 1204 1205; do jobs="$jobs $k:$M:$s"; done; done; done
echo $jobs | tr ' ' '\n' | xargs -P 4 -I{} bash -c '
	IFS=: read k M s <<< "{}"
	out=cont34_m${M}s${s}_k${k}
	[ -f figures/$out.json ] && exit 0
	python3 -u proposed_src/pipeline/cont_milp_v5_26_11_19_14.py --solver gurobi --norm poly --ngon 64 --num-dests $M --seed $s --kappa $k --capacity 3 --time-limit 3600 --threads 4 --out $out > runs/cont34/$out.log 2>&1
	echo "[$(date "+%F %T")] 완료 $out: $(grep "\[결과\]" runs/cont34/$out.log | cut -c1-170)" >> '"$LOG"'
'
echo "[$(date '+%F %T')] === 완료 ===" >> "$LOG"
