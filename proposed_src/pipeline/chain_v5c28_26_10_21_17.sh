#!/usr/bin/env bash
# 사이클 28: 탐색 휴리스틱 모방 학습 (26-10-21).
#   교사 풀(300 인스턴스: 40% M=5-12 Q=2, 60% M=13-60 Q=5, N=3-4, 무작위 거점)에 ILS 방문 순서를 구해 두고,
#   ST1·ST2: 정규화 목표 = 교사(ILS 순서)가 그 상태에서 내릴 옵션 (--teacher)
#   RT1·RT2: 같은 풀, 정규화 목표 = 기존 규칙 (분포 변화만의 효과를 가르는 대조군)
#   나머지는 SWa 조리법. 예산 1,000 에피소드·인내 8 (기본값). 감시 스크립트로 자동 재개.
#   학습 뒤: 재적재 인스턴스(M=7·8, Q=2, 각 5건) 격자 평가 + 표준 쌍별 평가(3/30, 5/50, 4/50).
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
SUP="proposed_src/pipeline/supervise_v5_26_10_13_12.sh"
PR="proposed_src/util/paired_v5_26_09_27_08.py"
LE="proposed_src/util/lattice_eval_v5_26_10_20_17.py"
LOG="$ROOT/runs/v5c28_26_10_21_17.log"
POOL="figures/teacher_pool_v5_26_10_21_17_shard*.json"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== 0단계: 교사 풀 대기 ==="
while [ "$(ls $POOL 2>/dev/null | wc -l)" -lt 14 ]; do sleep 120; done
say "교사 풀 준비: $(python3 -c "import json,glob;r=[x for f in glob.glob('$POOL') for x in json.load(open(f))];g=[x for x in r if x.get('teacher_ms')];import numpy as np;print(len(r),'건, 교사/규칙 평균',round(float(np.mean([x['teacher_ms']/x['rule_ms'] for x in g if x['rule_ms']])),3))")"
C="--residual-penalty 0.1 --rule-reg 2.0 --autoregressive --select makespan --speed-weight 40 --holdout-steps 3000 --eval-n 80 --ent-frac 0.6 --ent-frac-final 0.25 --ent-anneal-episodes 400 --pool $POOL"
say "=== 1단계: 학습 4실행 시작 ==="
setsid nohup bash $SUP v5_26_10_21_17_RT1 $C --seed 1 > /dev/null 2>&1 &
setsid nohup bash $SUP v5_26_10_21_17_RT2 $C --seed 2 > /dev/null 2>&1 &
setsid nohup bash $SUP v5_26_10_21_17_ST1 $C --teacher --seed 3 > /dev/null 2>&1 &
setsid nohup bash $SUP v5_26_10_21_17_ST2 $C --teacher --seed 4 > /dev/null 2>&1 &
sleep 120
while true; do
	n=0
	for r in RT1 RT2 ST1 ST2; do grep -q "감시 종료" "runs/sup_v5_26_10_21_17_${r}_supervisor.log" 2>/dev/null && n=$((n+1)); done
	[ "$n" -ge 4 ] && break
	sleep 300
done
say "=== 1단계 종료 ==="
say "=== 2단계: 재적재 인스턴스 격자 평가 (M=7·8, Q=2, 각 5건) ==="
for r in RT1 RT2 ST1 ST2; do
	W="weights/v5_26_10_21_17_$r/best_manager.pth"
	[ -f "$W" ] || { say "가중치 없음: $r"; continue; }
	python3 -u $LE --pattern "bfs3_n3m[78]q2s90[1-5]_26_10_21_06.json" --manager $W --reps 30 \
		--out lattice_c28_${r}_v5_26_10_21_17 2>&1 | grep -vE 'pygame|Hello|pkg' | tee -a "$LOG" &
done
wait
say "=== 3단계: 표준 쌍별 평가 ==="
pr(){ out=$1; n=$2; reps=$3; nd=$4; nm=$5; s0=$6
	EX=""; [ "$nd" != "0" ] && EX="--num-drones $nd --num-dests $nm --random-cc --seed0 $s0"
	python3 -u $PR --autoregressive --manager $W --n $n --reps $reps $EX --out "$out" 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"; }
for r in RT1 RT2 ST1 ST2; do
	W="weights/v5_26_10_21_17_$r/best_manager.pth"
	[ -f "$W" ] || continue
	pr paired_c28_${r}_d3_v5_26_10_21_17 40 5 3 30 501 &
	pr paired_c28_${r}_d5_v5_26_10_21_17 40 5 5 50 701 &
	pr paired_c28_${r}_std_v5_26_10_21_17 60 4 0 0 0 &
	wait
done
say "=== 완료 ==="
