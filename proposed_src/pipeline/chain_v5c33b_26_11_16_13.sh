#!/usr/bin/env bash
# 사이클 33b: 균형 혼합 증류 (26-11-16).
#   사이클 32(DA, 드론 3–4·수요지 20–80)는 3대 구성 개선이 재현됐고 5/50 은 소폭 열세, 사이클 33(DB, 3–6·20–150)은 큰
#   인스턴스가 결정의 72–78% 를 차지해 그 이득까지 잃었다. 여기서는 DA 절차를 유지하면서
#   (1) 3–4대·20–80 96 에피소드 + 5–6대·30–100 32 에피소드를 섞고, (2) 학습 표본 추출을 에피소드별 균형 가중으로 하여
#   큰 인스턴스의 비중을 에피소드 수(25%)로 제한하고, (3) 보류 조기 종료 인내 6(1,200스텝)·최대 5,000스텝으로 더 오래 학습한다.
#   시드 8·9·10 (DC1–3), 작업자 6, 수집 데이터 저장. 평가는 사이클 33 과 같다.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
PR="proposed_src/util/paired_v5_26_09_27_08.py"
LE="proposed_src/util/lattice_eval_v5_26_10_20_17.py"
TC="proposed_src/util/toy_compare_v5_26_10_17_15.py"
DI="proposed_src/pipeline/distill_v5_26_11_09_17.py"
LOG="$ROOT/runs/v5c33b_26_11_16_13.log"
T="v5_26_11_09_17"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== 1단계: DC1·DC2·DC3 증류 (균형 혼합) ==="
for s in 8 9 10; do
	r="DC$((s-7))"
	( python3 -u $DI --tag ${T}_$r --seed $s --betas 1.0 --mix "3,4,20,80,96;5,6,30,100,32" --balanced --patience 6 --workers 6 --k-cand 3 --horizon 1000 --max-steps 2000 --train-steps 5000 --save-data >> runs/sup_${T}_$r.log 2>&1; echo "[$(date '+%F %T')] 증류 감시 종료 (코드 $?)" >> runs/sup_${T}_$r.log ) &
done
wait
say "=== 1단계 종료: $(grep -h '증류 종료' runs/sup_${T}_DC1.log runs/sup_${T}_DC2.log runs/sup_${T}_DC3.log | tr '\n' ' ') ==="
say "=== 2단계: 반복 1 가중치 평가 ==="
pr(){ out=$1; n=$2; reps=$3; nd=$4; nm=$5; s0=$6; shift 6
	EX=""; [ "$nd" != "0" ] && EX="--num-drones $nd --num-dests $nm --random-cc --seed0 $s0"
	python3 -u $PR --autoregressive --manager $W --n $n --reps $reps $EX "$@" --out "$out" 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"; }
for r in DC1 DC2 DC3; do
	W="weights/${T}_$r/manager_it1.pth"
	[ -f "$W" ] || { say "가중치 없음: $r"; continue; }
	pr paired_c32_${r}_d3_${T} 40 5 3 30 501 &
	pr paired_c32_${r}_d5_${T} 40 5 5 50 701 &
	pr paired_c32_${r}_std_${T} 60 4 0 0 0 &
	( python3 -u $LE --pattern "bfs2_n3m*_h98_26_10_21_03.json" --manager $W --reps 30 --out lattice_c32_${r}_m36_${T} > /dev/null 2>&1
	  python3 -u $LE --pattern "bfs3_n3m[78]q2s*_26_10_21_06.json" --manager $W --reps 30 --out lattice_c32_${r}_m78_${T} > /dev/null 2>&1
	  python3 -u $TC --pattern "bfs2_n3m*_h98_26_10_21_03.json" --manager $W --reps 30 --lattice-snap --out toy_c33_${r}_${T} > /dev/null 2>&1 ) &
	wait
	pr paired_c32x_${r}_d3m20_${T} 40 2 3 20 901 &
	pr paired_c32x_${r}_d4m80_${T} 40 2 4 80 901 &
	pr paired_c32x_${r}_d6m80_${T} 40 2 6 80 901 &
	pr paired_c32x_${r}_d8m150_${T} 40 2 8 150 901 &
	pr paired_c32x_${r}_d10m300_${T} 40 2 10 300 901 &
	pr paired_c32x_${r}_d12m400_${T} 40 2 12 400 901 &
	wait
	pr paired_c32e_${r}_d4m80_${T} 40 2 4 80 901 --escape &
	pr paired_c32e_${r}_d6m80_${T} 40 2 6 80 901 --escape &
	pr paired_c32e_${r}_d5_${T} 40 5 5 50 701 --escape &
	wait
	say "--- $r 평가 완료"
done
say "=== 3단계: 요약 ==="
python3 proposed_src/util/c33_report_v5_26_11_11_03.py --runs DA2,DA3,DA4,DC1,DC2,DC3 2>&1 | tee -a "$LOG"
say "=== 완료 ==="
