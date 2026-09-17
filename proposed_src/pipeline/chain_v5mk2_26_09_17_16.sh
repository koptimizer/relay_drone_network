#!/usr/bin/env bash
# makespan 사이클 2: 처음부터 학습, 홀드아웃 3000스텝·완주 우선 선택, 드론 3-6 비중.
#   A  : 잔여 페널티 0.1                      (시드 1, 2)
#   AB : 잔여 페널티 0.1 + 규칙 조언 관측       (시드 1, 2)
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5mk2_26_09_17_16.log"
T="proposed_src/pipeline/train_v5_26_09_15_23.py"
E="proposed_src/pipeline/eval_v5_26_09_15_23.py"
C="--random-config --drones-range 3 6 --residual-penalty 0.1 --select makespan --holdout-steps 3000 --max-episodes 1500"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== makespan 사이클 2 학습 시작 (A x2, AB x2) ==="
setsid nohup python3 -u $T --tag v5_26_09_17_16_A1  $C --seed 1 >> runs/v5mk2_A1.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_17_16_A2  $C --seed 2 >> runs/v5mk2_A2.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_17_16_AB1 $C --rule-obs --seed 1 >> runs/v5mk2_AB1.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_17_16_AB2 $C --rule-obs --seed 2 >> runs/v5mk2_AB2.log 2>&1 &
sleep 10
say "=== 종료 대기 (최대 14시간) ==="
END=$(( $(date +%s) + 14*3600 ))
while true; do
	N=$(pgrep -fc "tag v5_26_09_17_16_[A]" || true)
	[ "${N:-0}" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; pkill -f "tag v5_26_09_17_16_[A]"; sleep 5; break; }
	sleep 180
done
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
L="--arch set --no-cluster-penalty --stochastic --straight-worker --max-steps 10000 --no-progress-limit 1000000000 --deadlock-limit 1000000000"
for grp in "A:v5_26_09_17_16_A1 v5_26_09_17_16_A2:" "AB:v5_26_09_17_16_AB1 v5_26_09_17_16_AB2:--advice"; do
	IFS=: read -r name tags flag <<< "$grp"
	MG="weights/v5_26_09_15_23_s3b/best_manager.pth"; [ "$name" = "AB" ] && MG=""
	for t in $tags; do [ -f "weights/$t/best_manager.pth" ] && MG="$MG weights/$t/best_manager.pth"; done
	say "=== 평가 $name (a): 표준 4/50, 60시드x2 ==="
	python3 -u $E --n 60 --reps 2 $L $flag --manager $MG --out eval_v5mk2_${name}_std_26_09_17_16 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
	for cfg in "3 30 501 d3" "5 50 701 d5"; do
		set -- $cfg
		say "=== 평가 $name: 드론 $1, 목적지 $2, CC 무작위, 40시드 ==="
		python3 -u $E --n 40 $L $flag --num-drones $1 --num-dests $2 --random-cc --seed0 $3 --manager $MG --out eval_v5mk2_${name}_$4_26_09_17_16 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
	done
done
say "=== 완료 ==="
