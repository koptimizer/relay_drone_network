#!/usr/bin/env bash
# 26_09_04_15 -> 26_09_07_17 에서 배송이 27 -> 15~18 로 무너진 원인을 가린다.
# 학습에 영향을 주는 변경은 셋뿐이고(드론별 정지 지표는 계측 전용), 이를 하나씩 켜고 끈다.
#   base : 셋 다 이전 상태  (상한 1000, 조기종료 없음, 도달보너스 매스텝)  -> 27 재현 기대
#   arv  : 도달보너스만 현행(목표당 1회)
#   term : 조기종료만 현행(교착 100 / 무배송 200)
#   keep : 나머지는 현행이고 도달보너스만 이전 상태 (26_09_07_17 에서 하나만 뺀 것)
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/ablate_26_09_12_09.log"
SUP="proposed_src/pipeline/supervise_v3_26_09_07_17.sh"
W="weights/v3_26_09_04_15_w/best_worker.pth"
C="--comm-range 300 --no-cluster-penalty --hl-every 20 --eval-n 12 --stage manager"
C="$C --worker-ckpt $W --seed 1 --gamma 0.997 --max-episodes 1500"
NOTERM="--deadlock-limit 999999 --no-progress-limit 999999 --warmup-no-progress 999999"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }

say "=== 절제 학습 시작 (4조합) ==="
setsid nohup $SUP v3_26_09_12_09_base $C --max-steps 1000  $NOTERM --arrive-every >/dev/null 2>&1 &
sleep 10
setsid nohup $SUP v3_26_09_12_09_arv  $C --max-steps 1000  $NOTERM              >/dev/null 2>&1 &
sleep 10
setsid nohup $SUP v3_26_09_12_09_term $C --max-steps 1000  --deadlock-limit 100 --no-progress-limit 200 --arrive-every >/dev/null 2>&1 &
sleep 10
setsid nohup $SUP v3_26_09_12_09_keep $C --max-steps 10000 --deadlock-limit 100 --no-progress-limit 200 --arrive-every >/dev/null 2>&1 &
sleep 10

say "=== 종료 대기 (최대 14시간) ==="
END=$(( $(date +%s) + 14*3600 ))
while true; do
	N=$(ps -ef | grep -c "[p]ipeline/train_v3")
	[ "$N" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; \
		ps -ef | grep -E "[p]ipeline/train_v3|[s]upervise_v3" | awk '{print $2}' | xargs -r kill; sleep 5; break; }
	sleep 180
done

say "=== 평가: 이전 챔피언과 같은 좁은 예산 300 롤아웃 ==="
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
CK="weights/v3_26_09_04_15_m_s1/best_worker.pth"; MG="weights/v3_26_09_04_15_m_s1/best_manager.pth"
for t in v3_26_09_12_09_base v3_26_09_12_09_arv v3_26_09_12_09_term v3_26_09_12_09_keep; do
	[ -f "weights/$t/best_worker.pth" ] && { CK="$CK weights/$t/best_worker.pth"; MG="$MG weights/$t/best_manager.pth"; }
done
python3 -u proposed_src/pipeline/eval_v3_26_09_11_11.py --n 60 --reps 5 --no-cluster-penalty \
	--max-steps 10000 --no-progress-limit 200 --deadlock-limit 100 \
	--worker $CK --manager $MG --out eval_v3_ablate3_26_09_12_09 2>&1 \
	| grep -v -e pkg -e Hello | tee -a "$LOG"
say "=== 완료 ==="
