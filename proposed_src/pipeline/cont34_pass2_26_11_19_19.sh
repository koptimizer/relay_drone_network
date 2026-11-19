#!/usr/bin/env bash
# 사이클 34 2차 풀이: 1차 배치(=== 완료 ===)를 기다린 뒤, 40건 전부를 내접 여유 0.1 m 모형으로 incumbent 기준 지평 단축(T=⌈obj/κ⌉+σ)
#   + 시작해 + MIPFocus 3 으로 2시간·4스레드씩 4개 병렬로 다시 푼다. κ=10 먼저. κ=5 는 κ=10 해가 더 좋으면 그것을 세분해 시작해로 쓴다
#   (거친 κ 의 실행 가능해는 세분 모형에서도 실행 가능). 결과 cont34p2_* (보고 스크립트가 병합).
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/cont34/pass2.log"
until grep -q "=== 완료 ===" runs/cont34/batch.log; do sleep 120; done
echo "[$(date '+%F %T')] 1차 배치 종료 확인, 2차 풀이 시작" >> "$LOG"
python3 - <<'PY' > runs/cont34/pass2_jobs.txt
import json, glob, os
R = {}
for f in glob.glob("figures/cont34_m*s*_k*.json"):
	d = json.load(open(f)); R[(d["kappa"], d["dests"], d["seed"])] = (d, os.path.basename(f))
rows = []
for (k, M, s), (d, fn) in R.items():
	start = fn                                    # 모형이 바뀌어(내접 여유 0.1 m) 증명된 건도 다시 푼다 — 시작해 덕에 빨리 끝난다
	if k == 5 and (10, M, s) in R:
		d10, fn10 = R[(10, M, s)]
		if d10["obj"] is not None and (d["obj"] is None or d10["obj"] < d["obj"]):
			start = fn10
	rows.append((-k, M, s, f"{k}:{M}:{s}:{start}"))
print("\n".join(r[3] for r in sorted(rows)))
PY
echo "[$(date '+%F %T')] 대상 $(wc -l < runs/cont34/pass2_jobs.txt)건" >> "$LOG"
xargs -P 4 -I{} bash -c '
	IFS=: read k M s js <<< "{}"
	out=cont34p2_m${M}s${s}_k${k}
	[ -f figures/$out.json ] && exit 0
	python3 -u proposed_src/pipeline/cont_milp_v5_26_11_19_14.py --solver gurobi --norm poly --ngon 64 --num-dests $M --seed $s --kappa $k --capacity 3 --time-limit 7200 --threads 4 --mip-focus 3 --start-json $js --tight-horizon --grb-log runs/cont34/${out}_grb.log --out $out > runs/cont34/$out.log 2>&1
	echo "[$(date "+%F %T")] 완료 $out: $(grep "\[결과\]" runs/cont34/$out.log | cut -c1-170)" >> '"$LOG"'
' < runs/cont34/pass2_jobs.txt
echo "[$(date '+%F %T')] === 2차 완료 ===" >> "$LOG"
