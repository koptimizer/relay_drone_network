#!/bin/bash
# 사이클 35 보조: 탐색이 끝나면 참조 인스턴스에서 선견 규칙의 인스턴스당 시간을 잰다 (4 작업자)
cd /home/kopt/drone
LOG=runs/v5c35t_26_11_23_13.log
until grep -q "=== 완료 ===" runs/v5c35s_26_11_23_13.log; do sleep 60; done
echo "[$(date '+%F %T')] === 선견 규칙 시간 측정 ===" >> $LOG
python3 -u proposed_src/util/cont_ctrl_v5_26_11_19_14.py --weights "" --workers 4 --out cont_ctrl_time_v5_26_11_23_13 >> $LOG 2>&1
echo "[$(date '+%F %T')] === 완료 ===" >> $LOG
