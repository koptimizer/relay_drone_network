#!/usr/bin/env bash
# 사이클 12: 절제 연구가 지목한 불필요 요소를 실제로 뺀 간소 구성을 검증한다.
#   절제 결과: 잔여 페널티는 해롭다(표준 -28, 3/30 -37). 자기회귀는 구성에 따라 효과가 뒤집히고
#   학습·추론이 3배 느리다. 규칙 정규화·엔트로피 감쇠·구성 무작위화는 유지한다.
#   L1 : 잔여 페널티 제거, 자기회귀 유지            (시드 1, 2)
#   L2 : 잔여 페널티 + 자기회귀 모두 제거 (간소)    (시드 1, 2)
# L2가 기준과 동등하면 제안 방법이 3배 빨라지므로 실무적으로 큰 이득이다.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5lean_26_09_26_17.log"
T="proposed_src/pipeline/train_v5_26_09_15_23.py"
E="proposed_src/pipeline/eval_v5_26_09_15_23.py"
C="--random-config --drones-range 3 4 --rule-reg 2.0 --select makespan --speed-weight 40 --holdout-steps 3000 --eval-n 80 --max-episodes 1500 --ent-frac 0.6 --ent-frac-final 0.25 --ent-anneal-episodes 400"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== 사이클 12 학습 시작 (L1 x2 자기회귀 유지, L2 x2 간소) ==="
setsid nohup python3 -u $T --tag v5_26_09_26_17_L1a $C --autoregressive --seed 1 >> runs/v5lean_L1a.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_26_17_L1b $C --autoregressive --seed 2 >> runs/v5lean_L1b.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_26_17_L2a $C --seed 1 >> runs/v5lean_L2a.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v5_26_09_26_17_L2b $C --seed 2 >> runs/v5lean_L2b.log 2>&1 &
sleep 10
say "=== 종료 대기 (최대 26시간) ==="
END=$(( $(date +%s) + 26*3600 ))
while true; do
	N=$(pgrep -fc "tag v5_26_09_26_17_L[12]" || true)
	[ "${N:-0}" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; pkill -f "tag v5_26_09_26_17_L[12]"; sleep 5; break; }
	sleep 180
done
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
L="--arch set --no-cluster-penalty --stochastic --straight-worker --max-steps 10000 --no-progress-limit 1000000000 --deadlock-limit 1000000000"
MG_AR=""; MG_PL=""
for t in v5_26_09_26_17_L1a v5_26_09_26_17_L1b v5_26_09_23_18_SWa; do
	[ -f "weights/$t/best_manager.pth" ] && MG_AR="$MG_AR weights/$t/best_manager.pth"; done
for t in v5_26_09_26_17_L2a v5_26_09_26_17_L2b; do
	[ -f "weights/$t/best_manager.pth" ] && MG_PL="$MG_PL weights/$t/best_manager.pth"; done
for cfg in "std 60 2 0 0 0" "d3 40 1 3 30 501" "d5 40 1 5 50 701"; do
	set -- $cfg
	tag=$1; n=$2; reps=$3; nd=$4; nm=$5; s0=$6
	EX=""; [ "$nd" != "0" ] && EX="--num-drones $nd --num-dests $nm --random-cc --seed0 $s0"
	say "=== 평가 $tag (자기회귀 L1 + 기준) ==="
	python3 -u $E --n $n --reps $reps $L --autoregressive $EX --manager $MG_AR --out eval_v5lean_${tag}_ar_26_09_26_17 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
	say "=== 평가 $tag (간소 L2) ==="
	python3 -u $E --n $n --reps $reps $L $EX --manager $MG_PL --out eval_v5lean_${tag}_pl_26_09_26_17 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"
done
say "=== 완료 ==="
