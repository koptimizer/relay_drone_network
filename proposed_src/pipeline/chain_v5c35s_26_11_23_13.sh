#!/bin/bash
# 사이클 35 보조: 연속 공간 방문 순서·작업열 탐색 (20 인스턴스) → 논문 표 재생성
cd /home/kopt/drone
LOG=runs/v5c35s_26_11_23_13.log
echo "[$(date '+%F %T')] === 1단계: 연속 공간 탐색 (방문 순서 3x400, 작업열 3x1000) ===" >> $LOG
python3 -u proposed_src/util/cont_search_v5_26_11_23_12.py --workers 6 --out cont_search_v5_26_11_23_12 >> $LOG 2>&1
echo "[$(date '+%F %T')] === 2단계: 논문 표 재생성 ===" >> $LOG
python3 -u proposed_src/util/paper_cont_v5_26_11_23_08.py >> $LOG 2>&1
echo "[$(date '+%F %T')] === 완료 ===" >> $LOG
