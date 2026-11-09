#!/usr/bin/env bash
# 사이클 32: 학습을 makespan 에 맞추기 (26-11-09).
#   진단: 기존 보상은 재적재 보너스(+30)가 느린 정책에 상을 주고 시간 항이 수익의 약 4%뿐이다 (3/30 12건: SWa 가
#   규칙보다 58스텝 느린데 수익은 +8.9). 새 보상(스텝당 -0.1 + 잠재 기반 보강, 잠재 = -0.1 x 남은 시간 추정)은 12건 모두
#   makespan 순위와 수익 순위가 일치했다. 검증도 바꿨다: 80건 x 1회 홀드아웃 대신 3구성 x 20건 x 3회 쌍별 비율
#   (SWa 기준 1.005, 알려진 퇴보 체크포인트 1.06–1.14 를 가른다).
#   1단계 (병렬):
#     MA1·MA2: SWa 조리법 + makespan 정렬 보상(gamma 0.999) + 쌍별 검증, 규칙 정규화 2.0, 시드 1·2 (처음부터)
#     MA3·MA4: 같고 규칙 정규화 0.5 (규칙에서 더 멀리 벗어날 수 있게)
#     DA1→DA2: 선견 규칙 증류 (DAgger, 지도학습만), SWa 에서 시작, 시드 1·2 를 차례로 (메모리 여유를 위해 순차, 작업자 10)
#     교사 성능 (낮은 우선순위, 2프로세스): 매 결정 선견·토글 없음·후보 3 의 3/30·5/50 쌍별 성능 (증류의 상한 참고)
#   검증은 환경 난수까지 공통 시드로 고정한다 (26-11-09 수정: 고치기 전 같은 가중치가 0.998–1.043 으로 흔들림).
#   최고 갱신 여유폭 1% (한 번 검증의 잡음 약 2%).
#   2단계: 각 최고 가중치의 쌍별 평가 (3/30, 5/50, 4/50) + 격자 정확해 55건
#   3단계: SWa 대비 요약표
#   예산: SAC 1,000 에피소드·인내 8 (기본값), 감시 스크립트.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
SUP="proposed_src/pipeline/supervise_v5_26_10_13_12.sh"
PR="proposed_src/util/paired_v5_26_09_27_08.py"
LE="proposed_src/util/lattice_eval_v5_26_10_20_17.py"
DI="proposed_src/pipeline/distill_v5_26_11_09_17.py"
RC="proposed_src/util/rule2_cont_v5_26_10_26_20.py"
LOG="$ROOT/runs/v5c32_26_11_09_17.log"
T="v5_26_11_09_17"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
[ -f figures/val_ref_${T}.json ] || { say "검증 기준값 없음 — 중단"; exit 1; }
C="--random-config --drones-range 3 4 --residual-penalty 0.1 --autoregressive --select makespan --speed-weight 40 --holdout-steps 3000 --eval-n 80 --ent-frac 0.6 --ent-frac-final 0.25 --ent-anneal-episodes 400 --makespan-reward --gamma 0.999 --val-paired --val-samples 3 --val-margin 1.0"
say "=== 1단계: SAC 4실행 + 증류 2실행 + 교사 평가 시작 ==="
setsid nohup bash $SUP ${T}_MA1 $C --rule-reg 2.0 --seed 1 > /dev/null 2>&1 &
setsid nohup bash $SUP ${T}_MA2 $C --rule-reg 2.0 --seed 2 > /dev/null 2>&1 &
setsid nohup bash $SUP ${T}_MA3 $C --rule-reg 0.5 --seed 1 > /dev/null 2>&1 &
setsid nohup bash $SUP ${T}_MA4 $C --rule-reg 0.5 --seed 2 > /dev/null 2>&1 &
DCMD="--betas 1.0,0.5,0.25,0.0,0.0,0.0,0.0,0.0 --episodes 96 --workers 10 --k-cand 3 --horizon 1000 --max-steps 1200 --train-steps 3000"
setsid nohup bash -c "for s in 1 2; do for a in 1 2; do python3 -u $DI --tag ${T}_DA\$s --seed \$s $DCMD >> runs/sup_${T}_DA\$s.log 2>&1 && break; echo \"[\$(date '+%F %T')] 증류 비정상 종료, 재시도 \$a\" >> runs/sup_${T}_DA\$s.log; sleep 30; done; echo \"[\$(date '+%F %T')] 증류 감시 종료\" >> runs/sup_${T}_DA\$s.log; done" > /dev/null 2>&1 &
for c in "3 30 501" "5 50 701"; do
	set -- $c
	setsid nohup nice -n 10 python3 -u $RC --num-drones $1 --num-dests $2 --seed0 $3 --n 40 --random-cc --methods la --la-every 0 --k-cand 3 --no-toggle --horizon 1000 --out expert_c32_d$1_${T} > runs/expert_c32_d$1.log 2>&1 &
done
sleep 120
while true; do
	n=0
	for r in MA1 MA2 MA3 MA4; do
		if grep -q "감시 종료" "runs/sup_${T}_${r}_supervisor.log" 2>/dev/null \
		   || ! pgrep -f "[s]upervise_v5_26_10_13_12.sh ${T}_${r} " > /dev/null; then n=$((n+1)); fi
	done
	for r in DA1 DA2; do
		if grep -q "증류 감시 종료" "runs/sup_${T}_${r}.log" 2>/dev/null; then n=$((n+1)); fi
	done
	[ "$n" -ge 6 ] && break
	sleep 300
done
say "=== 1단계 종료 ==="
say "=== 2단계: 쌍별 평가 + 격자 평가 ==="
pr(){ out=$1; n=$2; reps=$3; nd=$4; nm=$5; s0=$6
	EX=""; [ "$nd" != "0" ] && EX="--num-drones $nd --num-dests $nm --random-cc --seed0 $s0"
	python3 -u $PR --autoregressive --manager $W --n $n --reps $reps $EX --out "$out" 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"; }
for r in MA1 MA2 MA3 MA4 DA1 DA2; do
	W="weights/${T}_$r/best_manager.pth"
	[ -f "$W" ] || { say "가중치 없음: $r"; continue; }
	pr paired_c32_${r}_d3_${T} 40 5 3 30 501 &
	pr paired_c32_${r}_d5_${T} 40 5 5 50 701 &
	pr paired_c32_${r}_std_${T} 60 4 0 0 0 &
	( python3 -u $LE --pattern "bfs2_n3m*_h98_26_10_21_03.json" --manager $W --reps 30 --out lattice_c32_${r}_m36_${T} > /dev/null 2>&1
	  python3 -u $LE --pattern "bfs3_n3m[78]q2s*_26_10_21_06.json" --manager $W --reps 30 --out lattice_c32_${r}_m78_${T} > /dev/null 2>&1 ) &
	wait
done
say "=== 3단계: SWa 대비 요약 ==="
python3 proposed_src/util/c32_summary_v5_26_11_09_17.py 2>&1 | tee -a "$LOG"
say "=== 완료 ==="
