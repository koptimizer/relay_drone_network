#!/usr/bin/env bash
# M=6 정확해 (h=98). 용량 비추적 완화이므로 해 복원 후 드론별 배송 수 <= Q 를 확인한다.
cd /home/kopt/drone
B=proposed_src/pipeline/lattice_bfs_v5_26_10_19_22.py
one(){ s=$1; out="bfs_n3m6s${s}_h98_26_10_19_22"
	[ -f figures/$out.json ] && { echo "skip $out"; return; }
	python3 -u $B --num-dests 6 --seed $s --out $out > runs/bfs6/$out.log 2>&1
	echo "[$(date '+%T')] done $out: $(grep '결과\|해 없음' runs/bfs6/$out.log | cut -c1-110)"
}
export -f one; export B
for s in $(seq 901 910); do echo "$s"; done | xargs -P 3 -L 1 bash -c 'one $0'
echo "BFS6_ALLDONE $(date '+%T')"
