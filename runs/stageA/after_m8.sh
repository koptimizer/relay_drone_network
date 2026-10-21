#!/usr/bin/env bash
# M=8 정확해 5건이 모두 나오면 1단계 측정(규칙·학습·복구, 탐색 휴리스틱)을 이어서 건다.
cd /home/kopt/drone
while [ "$(ls figures/bfs3_n3m8q2s90[1-5]_26_10_21_06.json 2>/dev/null | wc -l)" -lt 5 ]; do
	[ "$(pgrep -fc '[l]attice_bfs_cargo')" = "0" ] && break
	sleep 60
done
echo "[$(date '+%T')] M=8 정확해 $(ls figures/bfs3_n3m8q2s90[1-5]_26_10_21_06.json 2>/dev/null | wc -l)/5 — 1단계 측정 시작"
python3 -u proposed_src/util/lattice_eval_v5_26_10_20_17.py --pattern "bfs3_n3m8q2s90[1-5]_26_10_21_06.json" --reps 30 --out lattice_m8q2_SWa_v5_26_10_21_17 > runs/stageA/le_m8.log 2>&1 &
python3 -u proposed_src/util/metaheur_v5_26_10_20_17.py --pattern "bfs3_n3m8q2s90[1-5]_26_10_21_06.json" --lattice --algo ils --iters 600 --restarts 2 --out meta_m8q2_v5_26_10_21_17 > runs/stageA/meta_m8.log 2>&1 &
wait
echo "[$(date '+%T')] M=8 1단계 측정 완료"
