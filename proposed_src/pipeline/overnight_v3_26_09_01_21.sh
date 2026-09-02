#!/usr/bin/env bash
# 밤사이 v3 결과를 확실히 검증한다.
#
# 지금까지 모든 설정을 1회씩만 학습했으므로 학습 자체의 분산을 모른다.
# 1) 2단계(상위 학습)를 서로 다른 시드 3개로 재학습해 재현성을 확인하고
# 2) 3단계(동시 학습)를 1개 시드로 재학습해 붕괴가 재현되는지 확인한 뒤
# 3) 모든 최적 정책을 대규모(인스턴스 60 × 5회 = 롤아웃 300개)로 평가한다.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT" || exit 1

SUP="proposed_src/pipeline/supervise_v3_26_08_31_19.sh"
W2="weights/v3_26_08_31_19_m_hl20"
WK="weights/v3_26_08_31_19_w_nocl/best_worker.pth"
LOG="$ROOT/runs/overnight_26_09_01_21.log"
COMMON="--comm-range 300 --no-cluster-penalty --hl-every 20 --eval-n 12"

say() { echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }

say "=== 1단계: 2단계 재학습 3시드 + 3단계 재학습 1시드 (병렬) ==="
for s in 1 2 3; do
	setsid nohup $SUP "v3_26_09_01_m2_s${s}" --stage manager $COMMON \
		--worker-ckpt "$WK" --seed "$s" >/dev/null 2>&1 &
	sleep 10
done
setsid nohup $SUP "v3_26_09_01_j_s1" --stage joint $COMMON \
	--worker-ckpt "$W2/best_worker.pth" --manager-ckpt "$W2/best_manager.pth" \
	--seed 1 >/dev/null 2>&1 &
sleep 10
say "4개 실행 기동 완료"

say "=== 2단계: 전부 종료될 때까지 대기 (최대 9시간) ==="
DEADLINE=$(( $(date +%s) + 9*3600 ))
while true; do
	N=$(ps -ef | grep -c "[p]ipeline/train_v3" )
	[ "$N" -eq 0 ] && { say "모든 학습 종료"; break; }
	[ "$(date +%s)" -ge "$DEADLINE" ] && { say "시간 초과 — 남은 $N개 중단"; \
		ps -ef | grep "[p]ipeline/train_v3" | awk '{print $2}' | xargs -r kill; \
		ps -ef | grep "[s]upervise_v3" | awk '{print $2}' | xargs -r kill; sleep 5; break; }
	sleep 120
done

say "=== 3단계: 대규모 평가 (인스턴스 60 × 5회 = 롤아웃 300개) ==="
CK=""; MG=""
for t in m_hl20:v3_26_08_31_19_m_hl20 m2_s1:v3_26_09_01_m2_s1 m2_s2:v3_26_09_01_m2_s2 \
         m2_s3:v3_26_09_01_m2_s3 j_s1:v3_26_09_01_j_s1; do
	d="weights/${t#*:}"
	if [ -f "$d/best_worker.pth" ] && [ -f "$d/best_manager.pth" ]; then
		CK="$CK $d/best_worker.pth"; MG="$MG $d/best_manager.pth"
		say "  평가 대상 추가: ${t%%:*}"
	fi
done

python3 -u proposed_src/pipeline/eval_v3_26_08_31_19.py \
	--n 60 --reps 5 --no-cluster-penalty \
	--worker $CK --manager $MG \
	--out eval_v3_overnight_26_09_01_21 2>&1 | grep -v -e pkg -e Hello | tee -a "$LOG"

say "=== 완료 ==="
