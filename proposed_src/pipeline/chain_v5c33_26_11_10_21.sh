#!/usr/bin/env bash
# 사이클 33: 증류 데이터 확장 (26-11-10).
#   사이클 32 의 정제 라벨 증류(교사 상태만, β=1, 이탈 여유폭 10·공통 난수 2회, 보류 10% 조기 종료)는 3대 구성에서 재현되는
#   개선을 냈지만, 수집 구성이 드론 3–4·수요지 20–80 이라 5/50 에서 약간 느리고 4/80·6/80 에 꼬리가 생겼다.
#   여기서는 수집 구성을 드론 3–6·수요지 20–150(상한 2,000스텝)으로 넓혀 같은 절차를 시드 5·6·7 로 돌린다 (DB1–3).
#   교사 에피소드 시간: 6/150 약 26분, 5/100 약 11분, 3–4대 4–8분 → 128 에피소드 / 작업자 6 ≈ 4–6시간.
#   평가: 반복 1 가중치로 쌍별 3구성 + 전이 6구성 + 격자 정확해 55건 + 정확해 인스턴스 연속 공간(중계 격자 스냅) + 복구 덮어쓰기(4/80·6/80).
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
PR="proposed_src/util/paired_v5_26_09_27_08.py"
LE="proposed_src/util/lattice_eval_v5_26_10_20_17.py"
TC="proposed_src/util/toy_compare_v5_26_10_17_15.py"
DI="proposed_src/pipeline/distill_v5_26_11_09_17.py"
LOG="$ROOT/runs/v5c33_26_11_10_21.log"
T="v5_26_11_09_17"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== 1단계: DB1·DB2·DB3 증류 (드론 3–6, 수요지 20–150) ==="
for s in 5 6 7; do
	r="DB$((s-4))"
	( python3 -u $DI --tag ${T}_$r --seed $s --betas 1.0 --episodes 128 --workers 6 --k-cand 3 --horizon 1000 --max-steps 2000 --train-steps 3000 --drones-range 3 6 --dests-range 20 150 >> runs/sup_${T}_$r.log 2>&1; echo "[$(date '+%F %T')] 증류 감시 종료 (코드 $?)" >> runs/sup_${T}_$r.log ) &
done
wait
say "=== 1단계 종료: $(grep -h '증류 종료' runs/sup_${T}_DB1.log runs/sup_${T}_DB2.log runs/sup_${T}_DB3.log | tr '\n' ' ') ==="
say "=== 2단계: 반복 1 가중치 평가 ==="
pr(){ out=$1; n=$2; reps=$3; nd=$4; nm=$5; s0=$6; shift 6
	EX=""; [ "$nd" != "0" ] && EX="--num-drones $nd --num-dests $nm --random-cc --seed0 $s0"
	python3 -u $PR --autoregressive --manager $W --n $n --reps $reps $EX "$@" --out "$out" 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"; }
for r in DB1 DB2 DB3; do
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
say "=== 3단계: SWa 대비 요약 ==="
python3 proposed_src/util/c32_summary_v5_26_11_09_17.py --runs DA2,DA3,DA4,DB1,DB2,DB3 2>&1 | tee -a "$LOG"
say "=== 완료 ==="
