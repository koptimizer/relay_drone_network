#!/usr/bin/env bash
# 연구 사이클 완료 시 저장소를 정리하고 커밋·태그·푸시한다.
# 사용: release_cycle.sh <버전태그> "<10단어 이하 설명>" [--dry-run]
#   예: release_cycle.sh v3_26_08_31_19 "hierarchical MARL, matches greedy with lower variance"
#
# 버전에 종속되지 않는 운영 스크립트이므로 접미를 붙이지 않는다.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT" || exit 1

VER="${1:-}"
MSG="${2:-}"
DRY="${3:-}"

if [ -z "$VER" ] || [ -z "$MSG" ]; then
	echo "사용: release_cycle.sh <버전태그> \"<10단어 이하 설명>\" [--dry-run]"
	echo "  예: release_cycle.sh v3_26_08_31_19 \"hierarchical MARL, matches greedy baseline\""
	exit 1
fi

# CLAUDE.md 규칙: 태그는 '버전넘버 + YY-MM-DD-HH', 설명은 10단어 이하
TAG="$(echo "$VER" | sed 's/_/-/3; s/_/-/3; s/_/-/3' | sed 's/\(v[0-9]*\)_/\1_/')"
TAG="$(echo "$VER" | awk -F'_' '{printf "%s_%s-%s-%s-%s", $1, $2, $3, $4, $5}')"
WORDS=$(echo "$MSG" | wc -w)
if [ "$WORDS" -gt 10 ]; then
	echo "오류: 설명이 ${WORDS}단어입니다. CLAUDE.md 규칙상 10단어 이하여야 합니다."
	exit 1
fi

echo "=== 1. 학습 진행 중인지 확인 ==="
RUNNING=$(pgrep -fc "pipeline/train_v" || true)
if [ "$RUNNING" -gt 0 ]; then
	echo "경고: 학습 프로세스 ${RUNNING}개가 실행 중입니다. 지표가 갱신 중일 수 있습니다."
fi

echo "=== 2. 용량 점검 ==="
# best_*.pth 외의 체크포인트는 .gitignore가 이미 제외한다. 여기서는 실제 결과만 확인한다.
STAGED_MB=$(git ls-files -o -c --exclude-standard -z 2>/dev/null \
	| du -ch --files0-from=- 2>/dev/null | tail -1 | cut -f1)
echo "추적 대상 총 용량: ${STAGED_MB:-알수없음}"

BIG=$(git ls-files -o -c --exclude-standard | while read -r f; do
	[ -f "$f" ] && [ "$(stat -c%s "$f")" -gt 52428800 ] && echo "$f"
done)
if [ -n "$BIG" ]; then
	echo "오류: 50MB 초과 파일이 추적 대상에 있습니다. .gitignore를 확인하세요."
	echo "$BIG"
	exit 1
fi

echo "=== 3. 변경 사항 ==="
git add -A
CHANGED=$(git diff --cached --name-only | wc -l)
if [ "$CHANGED" -eq 0 ]; then
	echo "커밋할 변경이 없습니다."
	exit 0
fi
git diff --cached --stat | tail -5
echo "  파일 ${CHANGED}개 변경"

NEW_PTH=$(git diff --cached --name-only --diff-filter=A | grep -c '\.pth$' || true)
echo "  새로 추가되는 가중치: ${NEW_PTH}개 (best_*.pth만 추적됨)"

if [ "$DRY" = "--dry-run" ]; then
	echo "=== dry-run: 여기서 중단합니다 ==="
	git reset -q
	exit 0
fi

echo "=== 4. 커밋 ==="
git -c commit.gpgsign=false commit -q -m "$(printf '%s: %s\n\n연구 사이클 완료 시점의 코드·지표·문서 스냅샷.\n상세 이력은 docs/VERSIONS.md 참조.\n\nCo-Authored-By: Claude Opus 5 <noreply@anthropic.com>' "$VER" "$MSG")"
git log --oneline -1

echo "=== 5. 태그 ==="
if git rev-parse "$TAG" >/dev/null 2>&1; then
	echo "태그 $TAG 가 이미 존재합니다. 태그 없이 푸시합니다."
	TAG=""
else
	git tag -a "$TAG" -m "$MSG"
	echo "태그 생성: $TAG"
fi

echo "=== 6. 푸시 ==="
if ! git push -q origin main 2>&1; then
	echo "오류: main 푸시 실패. 인증 또는 원격 상태를 확인하세요."
	exit 1
fi
[ -n "$TAG" ] && git push -q origin "$TAG"
echo "완료: $(git rev-parse --short HEAD) → origin/main${TAG:+ , 태그 $TAG}"
