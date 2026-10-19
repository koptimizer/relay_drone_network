#!/usr/bin/env bash
# RQ2 참조 (재적재 필요 크기): 볼록 껍질 가지치기 격자 MILP, M=6·8 각 10 인스턴스, M=10 4 인스턴스. 시간 제한 3600s, 2병렬 x 4스레드.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
M="proposed_src/pipeline/milp_v5_26_09_21_03.py"; LOG="$ROOT/runs/milp/hull_batch_26_10_19_22.log"; mkdir -p runs/milp
run_one(){ m=$1; s=$2; tl=$3; out="milph_n3m${m}s${s}_26_10_19_22"
	[ -f "figures/$out.json" ] && { echo "skip $out"; return; }
	python3 -u "$M" --num-drones 3 --num-dests "$m" --seed "$s" --time-limit "$tl" --threads 4 --quiet --hull-auto --out "$out" > "runs/milp/$out.log" 2>&1
	echo "[$(date '+%F %T')] done M=$m seed=$s: $(python3 -c "
import json;d=json.load(open('figures/$out.json'));print('nodes',d['nodes'],'steps',d['milp_steps'],'gap',d['gap'],'bound',d['bound_steps'],'rule',d['rule_makespan'],'sec',d['solve_sec'])" 2>/dev/null || echo 'json 없음')"; }
export -f run_one; export M
{ for s in $(seq 901 910); do echo "6 $s 3600"; done; for s in $(seq 901 910); do echo "8 $s 3600"; done; for s in $(seq 901 904); do echo "10 $s 3600"; done; } | xargs -P 2 -L 1 bash -c 'run_one $0 $1 $2' 2>&1 | tee -a "$LOG"
echo "MILPH_ALLDONE $(date '+%F %T')" | tee -a "$LOG"
