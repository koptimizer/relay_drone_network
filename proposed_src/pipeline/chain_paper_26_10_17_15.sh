#!/usr/bin/env bash
# 논문 측정 연쇄 (사이클 26). 휴리스틱 파라미터가 확정된 뒤 돈다. GPU 불필요.
#   1) 휴리스틱이 (60,40,shuffle)에서 바뀌었으면 모든 쌍별 비교를 새 휴리스틱으로 재측정
#      (표준 480, 3/30 200, 5/50 200, 3/20·4/80·6/80·8/150·10/300 각 80).
#   2) RQ3 통신 반경 전이: R=250·350에서 3/30·4/50·5/50 쌍별 (R=300은 1의 결과를 쓴다).
#   3) RQ2 토이 비교: MILP 참조 인스턴스에서 휴리스틱·학습 (MILP 배치가 끝난 뒤).
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_paper_26_10_17_15.log"
PR="proposed_src/util/paired_v5_26_09_27_08.py"
W="weights/v5_26_09_23_18_SWa/best_manager.pth"
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
ESC=$(python3 -c "import sys; sys.path.insert(0,'proposed_src'); from pipeline.common_v5_26_09_15_22 import ESC_K,ESC_HOLD,ESC_MODE; print(ESC_K,ESC_HOLD,ESC_MODE)")
say "=== 휴리스틱 확정 파라미터: $ESC ==="
pr(){ # tag n reps nd nm seed0 extra...
	tag=$1; n=$2; reps=$3; nd=$4; nm=$5; s0=$6; shift 6
	EX=""; [ "$nd" != "0" ] && EX="--num-drones $nd --num-dests $nm --random-cc --seed0 $s0"
	python3 -u $PR --autoregressive --manager $W --n $n --reps $reps $EX "$@" --out "$tag" 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"
}
if [ "$ESC" != "60 40 shuffle" ]; then
	say "=== 1) 휴리스틱 변경 — 쌍별 전체 재측정 (4병렬) ==="
	pr paired_std_SWa_v5_26_10_17_15 60 8 0 0 0 &
	pr paired_d3_SWa_v5_26_10_17_15 40 5 3 30 501 &
	pr paired_d5_SWa_v5_26_10_17_15 40 5 5 50 701 &
	pr paired_d3m20_SWa_v5_26_10_17_15 40 2 3 20 901 &
	wait
	pr paired_d4m80_SWa_v5_26_10_17_15 40 2 4 80 901 &
	pr paired_d6m80_SWa_v5_26_10_17_15 40 2 6 80 901 &
	pr paired_d8m150_SWa_v5_26_10_17_15 40 2 8 150 901 &
	pr paired_d10m300_SWa_v5_26_10_17_15 40 2 10 300 901 &
	wait
else
	say "=== 1) 휴리스틱 유지 — 기존 쌍별 결과를 그대로 쓴다 ==="
fi
say "=== 2) RQ3 통신 반경 전이 (R=250, 350) ==="
for R in 250 350; do
	pr paired_R${R}_330_SWa_v5_26_10_17_15 40 5 3 30 501 --comm-range $R &
	pr paired_R${R}_550_SWa_v5_26_10_17_15 40 5 5 50 701 --comm-range $R &
	pr paired_R${R}_450_SWa_v5_26_10_17_15 60 2 0 0 0 --comm-range $R &
	wait
done
say "=== 3) RQ2 토이 비교 — MILP 배치 종료 대기 ==="
while pgrep -f "milp_batch_26_10_17_15.sh" > /dev/null; do sleep 300; done
python3 -u proposed_src/util/toy_compare_v5_26_10_17_15.py 2>&1 | grep -vE 'pkg|Hello' | tee -a "$LOG"
say "=== 완료 ==="
