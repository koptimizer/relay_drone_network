#!/usr/bin/env bash
# RQ2: 격자 MILP 참조해를 크기당 여러 인스턴스에서, 더 긴 시간 제한으로 구한다 (gap 축소).
#   드론 3대: 목적지 4·6·8 각 10 인스턴스(시드 901-910), 목적지 10은 4 인스턴스(901-904).
#   시간 제한 4곳 1800s, 6곳 3600s, 8곳 7200s, 10곳 7200s. 3개 병렬(6스레드씩).
#   기존 기록: 4곳 600s에서 gap 42%, 6곳 1800s에서 44%, 10곳 5400s에서 해 없음.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
M="proposed_src/pipeline/milp_v5_26_09_21_03.py"
LOG="$ROOT/runs/milp/batch_26_10_17_15.log"
mkdir -p runs/milp
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
Q="$ROOT/runs/milp/queue_26_10_17_15.txt"
: > "$Q"
for s in $(seq 901 910); do echo "4 $s 1800" >> "$Q"; done
for s in $(seq 901 910); do echo "6 $s 3600" >> "$Q"; done
for s in $(seq 901 910); do echo "8 $s 7200" >> "$Q"; done
for s in $(seq 901 904); do echo "10 $s 7200" >> "$Q"; done
say "=== MILP 배치 시작: $(wc -l < "$Q")건, 3병렬 ==="
run_one(){
	m=$1; s=$2; tl=$3
	out="milp_n3m${m}s${s}_26_10_17_15"
	[ -f "figures/$out.json" ] && { echo "[$(date '+%F %T')] 건너뜀(존재): $out"; return; }
	echo "[$(date '+%F %T')] 시작 M=$m seed=$s tl=$tl"
	python3 -u "$M" --num-drones 3 --num-dests "$m" --seed "$s" --time-limit "$tl" --threads 6 --quiet \
		--out "$out" > "runs/milp/$out.log" 2>&1
	echo "[$(date '+%F %T')] 종료 M=$m seed=$s: $(python3 -c "
import json;d=json.load(open('figures/$out.json'));print('steps',d['milp_steps'],'gap',d['gap'],'bound',d['bound_steps'],'rule',d['rule_makespan'],'sec',d['solve_sec'])" 2>/dev/null || echo 'json 없음')"
}
export -f run_one; export M
cat "$Q" | xargs -P 3 -L 1 bash -c 'run_one $0 $1 $2' 2>&1 | tee -a "$LOG"
say "=== MILP 배치 완료 ==="
