#!/usr/bin/env bash
# 사이클 35: 연속 모형 인스턴스(드론 3·용량 3·수요지 9–12·거점 고정)와 같은 분포(수요지 8–16, 용량 3, 거점 무작위)에서
#   학습 관리자를 새로 학습(SQa·SQb, SWa 조리법 그대로, 예산 1,000 에피소드·인내 8)하고, 사이클 34 의 20개 인스턴스에서
#   30회 표본 중앙값으로 잰 뒤 논문 RQ2 연속 참조 표를 만든다. 학습 2개 병렬(GPU 공유) → 평가 → 표. 사람 개입 없이 끝까지 간다.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
SUP="proposed_src/pipeline/supervise_v5_26_10_13_12.sh"
LOG="$ROOT/runs/v5c35_26_11_23_08.log"
T="v5_26_11_23_08"
C="--random-config --drones-range 3 3 --dests-range 8 16 --capacity 3 --residual-penalty 0.1 --rule-reg 2.0 --autoregressive --select makespan --speed-weight 40 --holdout-steps 3000 --eval-n 80 --max-episodes 1000 --patience 8 --ent-frac 0.6 --ent-frac-final 0.25 --ent-anneal-episodes 400"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== 1단계: SQa·SQb 학습 (같은 분포: 드론 3, 수요지 8–16, 용량 3) ==="
( bash $SUP ${T}_SQa $C --seed 1 ) &
( bash $SUP ${T}_SQb $C --seed 2 ) &
wait
for r in SQa SQb; do say "  $r: $(tail -n 1 runs/sup_${T}_${r}_supervisor.log) | 최고 홀드아웃 $(awk -F, 'NR>1 && $10==1 {s=$9; e=$1} END {print s" (ep"e")"}' runs/${T}_${r}/evals_${T}_${r}.csv 2>/dev/null)"; done
say "=== 1단계 종료 ==="
say "=== 2단계: 20개 인스턴스 연속 평가 (SQa·SQb, 30회 중앙값) ==="
python3 -u proposed_src/util/cont_ctrl_v5_26_11_19_14.py --no-rule --weights SQa=weights/${T}_SQa/best_manager.pth,SQb=weights/${T}_SQb/best_manager.pth --out cont_ctrl_sq_${T} --workers 10 2>&1 | grep -vE "pkg_resources|pygame|Hello" | tail -n 22 | tee -a "$LOG"
say "=== 3단계: 보고·논문 표 ==="
python3 proposed_src/util/paper_cont_v5_26_11_23_08.py 2>&1 | tee -a "$LOG"
say "=== 완료 ==="
