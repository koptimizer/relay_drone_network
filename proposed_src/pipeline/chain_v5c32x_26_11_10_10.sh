#!/usr/bin/env bash
# 사이클 32x: 증류 정책(DA2·DA3·DA4)을 논문의 나머지 6구성에서 잰다 (26-11-10).
#   3/20, 4/80, 6/80, 8/150, 10/300, 12/400 — SWa 와 같은 쌍별 규약 (시드 901–940, 2회, 무작위 거점).
#   증류는 드론 3–4대 데이터로만 했으므로 큰 편대로의 전이를 확인해야 제안 정책 교체를 판단할 수 있다.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT" || exit 1
PR="proposed_src/util/paired_v5_26_09_27_08.py"
LOG="$ROOT/runs/v5c32x_26_11_10_10.log"
T="v5_26_11_09_17"
say(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
pr(){ out=$1; nd=$2; nm=$3
	python3 -u $PR --autoregressive --manager $W --n 40 --reps 2 --num-drones $nd --num-dests $nm --random-cc --seed0 901 --out "$out" 2>&1 | grep -vE '^  rep|pkg|Hello' | tee -a "$LOG"; }
say "=== 6구성 쌍별 평가 시작 ==="
for r in DA2 DA3 DA4; do
	W="weights/${T}_$r/best_manager.pth"
	[ -f "$W" ] || { say "가중치 없음: $r"; continue; }
	pr paired_c32x_${r}_d3m20_${T} 3 20 &
	pr paired_c32x_${r}_d4m80_${T} 4 80 &
	pr paired_c32x_${r}_d6m80_${T} 6 80 &
	pr paired_c32x_${r}_d8m150_${T} 8 150 &
	pr paired_c32x_${r}_d10m300_${T} 10 300 &
	pr paired_c32x_${r}_d12m400_${T} 12 400 &
	wait
	say "--- $r 완료"
done
say "=== 완료 ==="
