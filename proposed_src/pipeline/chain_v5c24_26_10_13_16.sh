#!/usr/bin/env bash
# 사이클 24: 목적함수를 makespan 쪽으로 돌린다 — 병목 ETA 페널티. 시드 4개(가중치 2수준), 감시 아래.
#   왜: 사이클 13-23의 아홉 개입(정보·크레딧·결정구조·초기화)이 모두 실패했고, 95% 정확한 규칙
#   모방 초기화조차 400 에피소드 안에 지워졌다. 남은 설명은 목적함수다. 보상은 배송 +10 중심이고
#   유효 지평 333스텝인데 에피소드는 600-900스텝이다. 잔여 페널티는 미배송 '수'를 벌해 평균 완료
#   시간을 줄이지만, makespan은 마지막 배송 하나만 본다.
#   병목 ETA = 가장 늦게 닿을 미배송 목적지의 예상 도달 스텝. 실현 이동 비율로 나누므로 투영에
#   막힌 드론은 늦은 것으로 계산되고, 중계로 동료를 풀어 주면 즉시 줄어든다.
#   관문 1(기하 거리만 쓴 ETA)은 중계 비중이 대조군과 같아 실패했다. 막힘을 보게 고친 관문 2에서
#   처음으로 기제 신호가 나왔다 — 대조군은 0.266에서 0.211로 감쇠하는데 페널티를 주면 0.242·0.252로
#   유지되고 가중치에 단조다 (26-10-13).
#   조리법은 SWa(사이클 11)와 같다. 예산 1,000 에피소드·인내 8.
#   채택 기준: (1) 3/30 쌍별 중앙 +10 이하, (2) 완주 100% 유지, (3) 표준·5/50 악화 없음,
#              (4) 기제 — 3/30 중계 대수가 SWa 0.78에서 규칙 1.39 쪽으로 이동.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5c24_26_10_13_16.log"
SV="proposed_src/pipeline/supervise_v5_26_10_13_12.sh"
PR="proposed_src/util/paired_v5_26_09_27_08.py"
DG="proposed_src/util/diag_d3_v5_26_09_27_11.py"
C="--random-config --drones-range 3 4 --residual-penalty 0.1 --rule-reg 2.0 --autoregressive --select makespan --speed-weight 40 --holdout-steps 3000 --eval-n 80 --ent-frac 0.6 --ent-frac-final 0.25 --ent-anneal-episodes 400"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== 사이클 24 학습 시작 (BN1 BN2 가중치 0.15, BN3 BN4 가중치 0.25) — 감시 아래 ==="
setsid nohup $SV v5_26_10_13_16_BN1 $C --bottleneck-penalty 0.15 --seed 1 > /dev/null 2>&1 &
sleep 6
setsid nohup $SV v5_26_10_13_16_BN2 $C --bottleneck-penalty 0.15 --seed 2 > /dev/null 2>&1 &
sleep 6
setsid nohup $SV v5_26_10_13_16_BN3 $C --bottleneck-penalty 0.25 --seed 3 > /dev/null 2>&1 &
sleep 6
setsid nohup $SV v5_26_10_13_16_BN4 $C --bottleneck-penalty 0.25 --seed 4 > /dev/null 2>&1 &
sleep 15
say "=== 종료 대기 (최대 30시간) — 감시 스크립트가 CUDA 사고 시 자동 재개한다 ==="
END=$(( $(date +%s) + 30*3600 ))
while true; do
	N=$(pgrep -fc "supervise_v5_26_10_13_12.sh v5_26_10_13_16_" || true)
	[ "${N:-0}" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; pkill -f "supervise_v5_26_10_13_12.sh v5_26_10_13_16_"; pkill -f "tag v5_26_10_13_16_"; sleep 5; break; }
	sleep 180
done
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
say "=== 기제 확인 — 3/30 중계 대수 (규칙 1.39, SWa 0.78) ==="
for r in BN1 BN2 BN3 BN4; do
	W="weights/v5_26_10_13_16_$r/best_manager.pth"
	[ -f "$W" ] || continue
	python3 -u $DG "$W" "$r" 2>&1 | grep -vE 'pkg|Hello' | tee -a "$LOG"
done
say "=== 쌍별 평가 ==="
for r in BN1 BN2 BN3 BN4; do
	W="weights/v5_26_10_13_16_$r/best_manager.pth"
	[ -f "$W" ] || { say "가중치 없음: $r"; continue; }
	for cfg in "d3 40 5 3 30 501" "d5 40 5 5 50 701" "std 60 2 0 0 0" "d3m20 40 2 3 20 901"; do
		set -- $cfg; tag=$1; n=$2; reps=$3; nd=$4; nm=$5; s0=$6
		EX=""; [ "$nd" != "0" ] && EX="--num-drones $nd --num-dests $nm --random-cc --seed0 $s0"
		say "=== 쌍별 $tag $r ==="
		python3 -u $PR --autoregressive --manager $W --n $n --reps $reps $EX \
			--out paired_${tag}_${r}_v5_26_10_13_16 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"
	done
done
say "=== 완료 ==="
