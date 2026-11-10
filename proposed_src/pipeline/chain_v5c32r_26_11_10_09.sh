#!/usr/bin/env bash
# 사이클 32r: DA2 재현 (26-11-10).
#   DA2 의 최고는 반복 1 (β=1, 교사 상태만, 정제 라벨: 이탈 여유폭 10스텝·후보당 공통 난수 2회, 보류 10% 조기 종료)이었다.
#   그 절차를 학습 시드 3·4 로 다시 돌려(DA3·DA4, 병렬, 작업자 각 9), 반복 1 가중치를 검증 통과 여부와 무관하게
#   쌍별 3구성 + 격자 55건으로 잰다. 갈래당 시드 하나의 차이를 신호로 쓰지 않는다는 규칙 (§9).
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
PR="proposed_src/util/paired_v5_26_09_27_08.py"
LE="proposed_src/util/lattice_eval_v5_26_10_20_17.py"
DI="proposed_src/pipeline/distill_v5_26_11_09_17.py"
LOG="$ROOT/runs/v5c32r_26_11_10_09.log"
T="v5_26_11_09_17"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== 1단계: DA3·DA4 증류 (반복 1 절차) ==="
for s in 3 4; do
	( python3 -u $DI --tag ${T}_DA$s --seed $s --betas 1.0 --episodes 96 --workers 9 --k-cand 3 --horizon 1000 --max-steps 1200 --train-steps 3000 >> runs/sup_${T}_DA$s.log 2>&1; echo "[$(date '+%F %T')] 증류 감시 종료 (코드 $?)" >> runs/sup_${T}_DA$s.log ) &
done
wait
say "=== 1단계 종료: $(grep -h '증류 종료' runs/sup_${T}_DA3.log runs/sup_${T}_DA4.log | tr '\n' ' ') ==="
say "=== 2단계: 반복 1 가중치의 쌍별 평가 + 격자 평가 ==="
pr(){ out=$1; n=$2; reps=$3; nd=$4; nm=$5; s0=$6
	EX=""; [ "$nd" != "0" ] && EX="--num-drones $nd --num-dests $nm --random-cc --seed0 $s0"
	python3 -u $PR --autoregressive --manager $W --n $n --reps $reps $EX --out "$out" 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"; }
for r in DA3 DA4; do
	W="weights/${T}_$r/manager_it1.pth"
	[ -f "$W" ] || { say "가중치 없음: $r"; continue; }
	pr paired_c32_${r}_d3_${T} 40 5 3 30 501 &
	pr paired_c32_${r}_d5_${T} 40 5 5 50 701 &
	pr paired_c32_${r}_std_${T} 60 4 0 0 0 &
	( python3 -u $LE --pattern "bfs2_n3m*_h98_26_10_21_03.json" --manager $W --reps 30 --out lattice_c32_${r}_m36_${T} > /dev/null 2>&1
	  python3 -u $LE --pattern "bfs3_n3m[78]q2s*_26_10_21_06.json" --manager $W --reps 30 --out lattice_c32_${r}_m78_${T} > /dev/null 2>&1 ) &
	wait
done
say "=== 3단계: SWa 대비 요약 ==="
python3 proposed_src/util/c32_summary_v5_26_11_09_17.py --runs DA2,DA3,DA4 2>&1 | tee -a "$LOG"
say "=== 완료 ==="
