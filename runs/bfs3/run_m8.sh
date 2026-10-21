#!/usr/bin/env bash
cd /home/kopt/drone
B=proposed_src/pipeline/lattice_bfs_cargo_v5_26_10_21_05.py
one(){ s=$1; out="bfs3_n3m8q2s${s}_26_10_21_06"
	[ -f figures/$out.json ] && { echo "skip $s"; return; }
	python3 -u $B --num-dests 8 --capacity 2 --seed $s --max-periods 50 --out $out > runs/bfs3/$out.log 2>&1
	echo "[$(date '+%T')] M8 s$s: $(grep '결과\|Error\|Memory' runs/bfs3/$out.log | cut -c1-80)"
}
export -f one; export B
for s in $(seq 901 905); do echo "$s"; done | xargs -P 3 -L 1 bash -c 'one $0'
echo "BFS3_M8_ALLDONE $(date '+%T')"
