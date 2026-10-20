#!/usr/bin/env bash
cd /home/kopt/drone
B=proposed_src/pipeline/lattice_bfs_v5_26_10_19_22.py
one(){ m=$1; s=$2; out="bfs2_n3m${m}s${s}_h98_26_10_21_03"
	python3 -u $B --num-dests $m --seed $s --out $out > runs/bfs2/$out.log 2>&1
	echo "[$(date '+%T')] $out: $(grep '결과\|해 없음' runs/bfs2/$out.log | cut -c1-90)"
}
export -f one; export B
for m in 3 4 5 6; do for s in $(seq 901 910); do echo "$m $s"; done; done | xargs -P 4 -L 1 bash -c 'one $0 $1'
echo "BFS2_ALLDONE $(date '+%T')"
