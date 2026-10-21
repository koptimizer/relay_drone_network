#!/usr/bin/env bash
cd /home/kopt/drone
PR=proposed_src/util/paired_v5_26_09_27_08.py
one(){ r=$1; cfg=$2
	W="weights/v5_26_09_24_15_$r/best_manager.pth"; AR="--autoregressive"; case $r in A1*) AR="";; esac
	case $cfg in d3) EX="--num-drones 3 --num-dests 30 --random-cc --seed0 501 --n 40 --reps 2";; d5) EX="--num-drones 5 --num-dests 50 --random-cc --seed0 701 --n 40 --reps 2";; std) EX="--n 60 --reps 2";; esac
	nice -n 10 python3 -u $PR $AR --manager $W $EX --out paired_abl_${r}_${cfg}_v5_26_10_19_22 > runs/abl27/${r}_${cfg}.log 2>&1
	echo "[$(date '+%T')] done $r $cfg: $(grep -a '윌콕슨\|쌍별 차이' runs/abl27/${r}_${cfg}.log | tr '\n' ' ' | cut -c1-200)"
}
export -f one; export PR
for r in A1s1 A1s2 A2s1 A2s2 A3s1 A3s2 A4s1 A4s2 A5s1 A5s2; do for c in d3 std d5; do echo "$r $c"; done; done | xargs -P 4 -L 1 bash -c 'one $0 $1'
echo "ABL_ALLDONE $(date '+%T')"
