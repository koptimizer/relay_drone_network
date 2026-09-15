#!/usr/bin/env bash
# v4 전체 사이클: 하위 재학습 -> 상위 학습(절제 3조합 병렬) -> 확정 평가.
# v3 하위는 중계 목표를 겪은 적이 없어 기하 규칙과 조합하면 36.8에 그쳤다(직진 하위 45.1).
# 그래서 1단계에서 중계를 쓰는 규칙 상위로 하위부터 다시 배운다.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v4_26_09_14_03.log"
TRAIN="proposed_src/pipeline/train_v4_26_09_14_03.py"
C="--comm-range 300 --no-cluster-penalty --hl-every 20 --eval-n 12 --seed 1"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }

say "=== 1단계: 하위 재학습 (규칙 상위 = 기하 중계) ==="
mkdir -p runs/v4_26_09_14_03_w
python3 -u $TRAIN --tag v4_26_09_14_03_w --stage worker --rule-manager chain \
	$C --max-episodes 1200 >> runs/v4_26_09_14_03_w/train.log 2>&1
say "1단계 종료 — $(tail -2 runs/v4_26_09_14_03_w/train.log | head -1)"

W="weights/v4_26_09_14_03_w/best_worker.pth"
if [ ! -f "$W" ]; then say "오류: 하위 가중치가 없다. 중단."; exit 1; fi

say "=== 2단계: 상위 학습 3조합 병렬 ==="
M="--stage manager --worker-ckpt $W --rule-manager chain --max-episodes 1500"
# base : 중계 행동만. incomplete 페널티 없음
# comp : 잔여 목적지 페널티로 '끝내는 것'을 보상 — 목적함수 불일치 교정
# noreal: 중계 행동을 규칙 상위에서 빼고 학습 (v3 통제군)
setsid nohup python3 -u $TRAIN --tag v4_26_09_14_03_m_base $C $M \
	>> runs/v4_26_09_14_03_m_base.log 2>&1 &
sleep 10
setsid nohup python3 -u $TRAIN --tag v4_26_09_14_03_m_comp $C $M --incomplete-penalty 0.05 \
	>> runs/v4_26_09_14_03_m_comp.log 2>&1 &
sleep 10
setsid nohup python3 -u $TRAIN --tag v4_26_09_14_03_m_ctrl $C $M --rule-manager plain \
	>> runs/v4_26_09_14_03_m_ctrl.log 2>&1 &
sleep 10

say "=== 2단계 종료 대기 (최대 10시간) ==="
END=$(( $(date +%s) + 10*3600 ))
while true; do
	N=$(ps -ef | grep -c "[t]rain_v4_26_09_14_03.py --tag v4_26_09_14_03_m")
	[ "$N" -eq 0 ] && { say "2단계 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; \
		ps -ef | grep "[t]rain_v4_26_09_14_03.py" | awk '{print $2}' | xargs -r kill; sleep 5; break; }
	sleep 180
done

say "=== 3단계: 확정 평가 (60시드 x 2회, 샘플링, 완주 측정) ==="
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
CK=""; MG=""
for t in v4_26_09_14_03_m_base v4_26_09_14_03_m_comp v4_26_09_14_03_m_ctrl; do
	[ -f "weights/$t/best_worker.pth" ] && { CK="$CK weights/$t/best_worker.pth"; MG="$MG weights/$t/best_manager.pth"; }
done
python3 -u proposed_src/pipeline/eval_v4_26_09_14_03.py --n 60 --reps 2 --no-cluster-penalty \
	--stochastic --max-steps 10000 --no-progress-limit 1000000000 --deadlock-limit 1000000000 \
	--worker $CK --manager $MG --out eval_v4_final_26_09_14_03 2>&1 \
	| grep -v -e pkg -e Hello | tee -a "$LOG"
say "=== 완료 ==="
