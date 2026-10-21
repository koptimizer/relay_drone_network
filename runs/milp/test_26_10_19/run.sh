#!/usr/bin/env bash
cd /home/kopt/drone
M=proposed_src/pipeline/milp_v5_26_09_21_03.py
run(){ s=$1; v=$2; shift 2; python3 -u $M --num-drones 3 --num-dests 4 --seed $s --time-limit 600 --threads 4 --quiet --out "milp_test_m4s${s}_${v}_26_10_19" "$@" > runs/milp/test_26_10_19/m4s${s}_${v}.log 2>&1; echo "[$(date '+%T')] done $s $v: $(python3 -c "import json;d=json.load(open('figures/milp_test_m4s${s}_${v}_26_10_19.json'));print('nodes',d['nodes'],'steps',d['milp_steps'],'gap',d['gap'],'bound',d['bound_steps'],'sec',d['solve_sec'])")"; }
export -f run; export M
printf "901 base\n901 hull --hull-auto\n901 hullmc --hull-auto --multi-commodity\n901 mc --multi-commodity\n903 base\n903 hull --hull-auto\n903 hullmc --hull-auto --multi-commodity\n903 mc --multi-commodity\n" | xargs -P 4 -L 1 bash -c 'run $0 $1 $2 $3'
echo "ALLDONE $(date '+%T')"
