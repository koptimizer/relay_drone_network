#!/usr/bin/env bash
# RQ2 정확해: BFS 로 M=3,4,5 x 시드 901-910 x 격자 간격 98.2 / 69.4 / 49.1
cd /home/kopt/drone
B=proposed_src/pipeline/lattice_bfs_v5_26_10_19_22.py
one(){ m=$1; s=$2; h=$3; tag=$(printf "%.0f" $h)
	out="bfs_n3m${m}s${s}_h${tag}_26_10_19_22"
	[ -f figures/$out.json ] && { echo "skip $out"; return; }
	python3 -u $B --num-dests $m --seed $s --spacing $h --out $out > runs/bfs/$out.log 2>&1
	echo "[$(date '+%T')] done $out: $(grep '결과' runs/bfs/$out.log | cut -c1-120)"
}
export -f one; export B
for h in 98.2 69.4 49.1; do for m in 3 4 5; do for s in $(seq 901 910); do echo "$m $s $h"; done; done; done | xargs -P 2 -L 1 bash -c 'one $0 $1 $2'
echo "BFS_ALLDONE $(date '+%T')"
