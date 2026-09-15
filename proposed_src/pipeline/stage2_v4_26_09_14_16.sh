#!/usr/bin/env bash
# v4 2단계 5조합을 띄우고, 모두 끝나면 확정 평가까지 이어서 돌린다.
# 홀드아웃을 샘플링 평가로 고친 뒤의 재실행이다 (argmax는 같은 가중치에서 11배 낮게 나왔다).
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v4_26_09_14_03.log"
T="proposed_src/pipeline/train_v4_26_09_14_03.py"
W="weights/v4_26_09_14_03_w/best_worker.pth"
C="--comm-range 300 --no-cluster-penalty --hl-every 20 --eval-n 12 --seed 1"
M="--stage manager --worker-ckpt $W --rule-manager chain --max-episodes 1500"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }

say "=== 2단계 재기동 (5조합, 홀드아웃 샘플링 평가) ==="
setsid nohup python3 -u $T --tag v4_26_09_14_03_m_base $C $M >> runs/v4_26_09_14_03_m_base.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v4_26_09_14_03_m_comp $C $M --incomplete-penalty 0.05 >> runs/v4_26_09_14_03_m_comp.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v4_26_09_14_03_m_ctrl $C $M --rule-manager plain >> runs/v4_26_09_14_03_m_ctrl.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v4_26_09_14_03_m_sw $C $M --straight-worker >> runs/v4_26_09_14_03_m_sw.log 2>&1 &
sleep 6
setsid nohup python3 -u $T --tag v4_26_09_14_03_m_swcomp $C $M --straight-worker --incomplete-penalty 0.05 >> runs/v4_26_09_14_03_m_swcomp.log 2>&1 &
sleep 10

say "=== 종료 대기 (최대 10시간) ==="
END=$(( $(date +%s) + 10*3600 ))
while true; do
	N=$(pgrep -fc "tag v4_26_09_14_03_m_" || true)
	[ "${N:-0}" -eq 0 ] && { say "2단계 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; \
		pkill -f "tag v4_26_09_14_03_m_"; sleep 5; break; }
	sleep 180
done

say "=== 3단계: 확정 평가 (60시드 x 2회, 샘플링) ==="
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
CK=""; MG=""
for t in v4_26_09_14_03_m_base v4_26_09_14_03_m_comp v4_26_09_14_03_m_ctrl \
         v4_26_09_14_03_m_sw v4_26_09_14_03_m_swcomp \
         v4_26_09_14_03_m_rh05 v4_26_09_14_03_m_rh10; do
	[ -f "weights/$t/best_manager.pth" ] && { CK="$CK weights/$t/best_worker.pth"; MG="$MG weights/$t/best_manager.pth"; }
done
python3 -u proposed_src/pipeline/eval_v4_26_09_14_03.py --n 60 --reps 2 --no-cluster-penalty \
	--stochastic --max-steps 10000 --no-progress-limit 1000000000 --deadlock-limit 1000000000 \
	--worker $CK --manager $MG --out eval_v4_final_26_09_14_03 2>&1 \
	| grep -v -e pkg -e Hello | tee -a "$LOG"
say "=== 완료 ==="
