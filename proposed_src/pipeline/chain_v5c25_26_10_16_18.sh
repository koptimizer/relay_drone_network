#!/usr/bin/env bash
# 사이클 25: 연결성 투영을 '같은 방향 축소'에서 '통신 원판 경계 당김'으로 바꾸고 재학습한다.
#   왜: 중계를 늘리려는 열 개입(13-24)이 모두 실패했다. 측정된 근접 원인은 중계 대수가 아니라
#   막힌 이동이다 — 3/30에서 드론-시간의 32%가 투영 축소로 사라진다(규칙은 9%). 정책의 선호를
#   바꾸는 대신 중계가 적어도 비용이 작게 만든다. 규칙은 이미 막힘이 적어 이득이 작고 제안은
#   커서, 이득이 비대칭으로 제안 쪽에 쌓인다.
#   영 샷 측정(옛 동역학에서 학습한 SWa를 새 동역학에서 평가): 3/30 +40 -> +21, 5/50 -54 -> -52,
#   3/20 +36 -> +40, 그런데 표준 4/50은 +0 -> +31로 악화. 정책이 옛 동역학에 맞춰져 있으므로
#   재학습이 정식 시험이다 (26-10-16).
#   조리법은 SWa(사이클 11)와 같고 --slide만 더한다. 예산 1,000 에피소드·인내 8. 감시 아래.
#   채택 기준: (1) 3/30 쌍별 중앙 +10 이하, (2) 표준 동률 회복(+10 이하), (3) 완주 100%,
#              (4) 5/50 우위 유지.
#   주의: 막힘 카운터는 두 투영 사이에 뜻이 다르다(접선 이동도 일부 막힘으로 집계). 비교는
#   makespan과 정체율로 한다. 규칙 베이스라인도 같은 동역학에서 다시 재므로 비교는 공정하다.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
LOG="$ROOT/runs/chain_v5c25_26_10_16_18.log"
SV="proposed_src/pipeline/supervise_v5_26_10_13_12.sh"
PR="proposed_src/util/paired_v5_26_09_27_08.py"
DG="proposed_src/util/diag_d3_v5_26_09_27_11.py"
C="--random-config --drones-range 3 4 --residual-penalty 0.1 --rule-reg 2.0 --autoregressive --select makespan --speed-weight 40 --holdout-steps 3000 --eval-n 80 --ent-frac 0.6 --ent-frac-final 0.25 --ent-anneal-episodes 400 --slide"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "=== 사이클 25 학습 시작 (SL1-SL4, 경계 당김 투영) — 감시 아래 ==="
for s in 1 2 3 4; do
	setsid nohup $SV v5_26_10_16_18_SL$s $C --seed $s > /dev/null 2>&1 &
	sleep 6
done
sleep 15
say "=== 종료 대기 (최대 30시간) ==="
END=$(( $(date +%s) + 30*3600 ))
while true; do
	N=$(pgrep -fc "supervise_v5_26_10_13_12.sh v5_26_10_16_18_" || true)
	[ "${N:-0}" -eq 0 ] && { say "학습 종료"; break; }
	[ "$(date +%s)" -ge "$END" ] && { say "시간 초과 — 중단"; pkill -f "supervise_v5_26_10_13_12.sh v5_26_10_16_18_"; pkill -f "tag v5_26_10_16_18_"; sleep 5; break; }
	sleep 180
done
export OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
say "=== 기제 확인 — 3/30 중계 대수·정체율 (경계 당김) ==="
for r in SL1 SL2 SL3 SL4; do
	W="weights/v5_26_10_16_18_$r/best_manager.pth"
	[ -f "$W" ] || continue
	python3 -u $DG "$W" "$r" slide 2>&1 | grep -vE 'pkg|Hello' | tee -a "$LOG"
done
say "=== 쌍별 평가 (두 정책 모두 경계 당김) ==="
for r in SL1 SL2 SL3 SL4; do
	W="weights/v5_26_10_16_18_$r/best_manager.pth"
	[ -f "$W" ] || { say "가중치 없음: $r"; continue; }
	for cfg in "d3 40 5 3 30 501" "d5 40 5 5 50 701" "std 60 2 0 0 0" "d3m20 40 2 3 20 901"; do
		set -- $cfg; tag=$1; n=$2; reps=$3; nd=$4; nm=$5; s0=$6
		EX=""; [ "$nd" != "0" ] && EX="--num-drones $nd --num-dests $nm --random-cc --seed0 $s0"
		say "=== 쌍별 $tag $r ==="
		python3 -u $PR --autoregressive --slide --manager $W --n $n --reps $reps $EX \
			--out paired_sl_${tag}_${r}_v5_26_10_16_18 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"
	done
done
say "=== 완료 ==="
