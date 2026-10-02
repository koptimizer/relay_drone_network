#!/usr/bin/env bash
# 사이클 19: 결정 구조 두 가지를 SWa 조리법 위에서 시험한다 (각 시드 2개, 동시 4실행).
#   NR : --ar-near         자기회귀 순서를 CC에서 가까운 드론부터 (중계는 가까운 드론이 서므로 먼저 정한다)
#   SR : --stall-redecide 60  부분 교착 60스텝마다 전 드론 재결정 (학습·평가 동일, 규칙 베이스라인은 그대로)
#   조리법은 SWa(사이클 11)와 같고 --holdout-mix는 쓰지 않는다 (SWa +18/+40/-54 와 직접 비교).
#   예산: 기본값 1,000 에피소드 · 인내 8 (--max-episodes 박지 않음).
#   채택 기준: NR은 3/30 +10 이하, SR은 3대 꼬리 최댓값이 규칙 2배 이하 — 둘 다 표준·5/50 악화 없음.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5c19_26_10_02_12.log"
T="proposed_src/pipeline/train_v5_26_09_15_23.py"
PR="proposed_src/util/paired_v5_26_09_27_08.py"
C="--random-config --drones-range 3 4 --residual-penalty 0.1 --rule-reg 2.0 --autoregressive --select makespan --speed-weight 40 --holdout-steps 3000 --eval-n 80 --ent-frac 0.6 --ent-frac-final 0.25 --ent-anneal-episodes 400"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== 사이클 19 학습 시작 (NR1 NR2 SR1 SR2) ==="
for s in 1 2; do
	setsid nohup python3 -u $T --tag v5_26_10_02_12_NR$s $C --ar-near --seed $s >> runs/v5c19_NR$s.log 2>&1 &
	sleep 6
	setsid nohup python3 -u $T --tag v5_26_10_02_12_SR$s $C --stall-redecide 60 --seed $s >> runs/v5c19_SR$s.log 2>&1 &
	sleep 6
done
sleep 10
say "=== 종료 대기 (최대 30시간) ==="
END=$(( $(date +%s) + 30*3600 ))
while true; do
	N=$(pgrep -fc "tag v5_26_10_02_12_" || true)
	[ "${N:-0}" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; pkill -f "tag v5_26_10_02_12_"; sleep 5; break; }
	sleep 180
done
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
for r in NR1 NR2 SR1 SR2; do
	W="weights/v5_26_10_02_12_$r/best_manager.pth"
	[ -f "$W" ] || { say "가중치 없음: $r"; continue; }
	X=""; [ "${r:0:2}" = "NR" ] && X="--ar-near"; [ "${r:0:2}" = "SR" ] && X="--stall-redecide 60"
	for cfg in "d3 40 5 3 30 501" "d5 40 5 5 50 701" "std 60 2 0 0 0" "d3m20 40 2 3 20 901"; do
		set -- $cfg; tag=$1; n=$2; reps=$3; nd=$4; nm=$5; s0=$6
		EX=""; [ "$nd" != "0" ] && EX="--num-drones $nd --num-dests $nm --random-cc --seed0 $s0"
		say "=== 쌍별 $tag $r ($X) ==="
		python3 -u $PR --autoregressive $X --manager $W --n $n --reps $reps $EX \
			--out paired_${tag}_${r}_v5_26_10_02_12 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"
	done
done
say "=== 완료 ==="
