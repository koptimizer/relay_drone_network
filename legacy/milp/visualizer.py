import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import matplotlib.patches as patches
import numpy as np
import math

def visualize_gurobi_solution(m, data, x_vars, v_vars, y_vars, z_vars):
    """
    Gurobi 최적해를 바탕으로 시간에 따른 드론 라우팅 및 통신 반경을 시각화합니다.
    """
    # 파라미터 및 노드 좌표 추출
    coords = data["node_coordinates"]
    T_max = data["parameters"]["T_max"]
    P_0 = 1000.0  # 이전 코드의 초기 신호 강도
    B_low = data["parameters"]["B_low"]
    
    # 최소 통신 가능 반경 계산 (P_0 / d^2 = B_low  => d = sqrt(P_0 / B_low))
    comm_radius = math.sqrt(P_0 / B_low)

    # 1. 시점 t별 드론의 위치 및 릴레이 활성화 상태 파싱
    # drone_pos[t][k] = (x, y)
    drone_pos = {t: {} for t in range(1, T_max + 1)}
    active_relays = {t: [] for t in range(1, T_max + 1)}

    for t in range(1, T_max + 1):
        # 릴레이 상태 파싱
        for r in data["sets"]["R"]:
            if y_vars[r, t].X > 0.5:
                active_relays[t].append(r)
                
        # 드론 위치 파싱
        for k in data["sets"]["K"]:
            # 노드에 체공 중인 경우
            hovering = False
            for i in data["sets"]["N"]:
                if v_vars[i, k, t].X > 0.5:
                    drone_pos[t][k] = coords[i]
                    hovering = True
                    break
            
            # 이동 중인 경우 (보간 계산)
            if not hovering:
                # 과거에 출발하여 현재 t 시점에 이동 중인 경로 탐색
                for i in data["sets"]["N"]:
                    for j in data["sets"]["N"]:
                        if i != j:
                            tau_ij = data["matrices"]["tau_ij"][i][j]
                            for start_t in range(max(1, t - tau_ij + 1), t + 1):
                                if x_vars[i, j, k, start_t].X > 0.5:
                                    # i에서 j로 가는 중
                                    progress = (t - start_t) / tau_ij
                                    x_curr = coords[i][0] + (coords[j][0] - coords[i][0]) * progress
                                    y_curr = coords[i][1] + (coords[j][1] - coords[i][1]) * progress
                                    drone_pos[t][k] = (x_curr, y_curr)
                                    break

    # 2. Matplotlib Figure 및 Slider 세팅
    fig, ax = plt.subplots(figsize=(10, 8))
    plt.subplots_adjust(bottom=0.2)
    
    ax_slider = plt.axes([0.15, 0.05, 0.7, 0.03])
    time_slider = Slider(ax_slider, 'Time (t)', 1, T_max, valinit=1, valstep=1)

    def update(val):
        t = int(time_slider.val)
        ax.clear()
        
        # 기본 그리드 및 축 설정
        ax.set_xlim(-25, 25)
        ax.set_ylim(-25, 25)
        ax.grid(True, linestyle='--', alpha=0.5)
        ax.set_title(f"Disaster Drone Routing & Comm Network - Time: {t}", fontsize=14)
        
        # 3. 통신 반경 그리기 (제어 센터 & 활성화된 릴레이)
        # 제어 센터 반경 (파란색)
        cc_circle = patches.Circle(coords['0'], comm_radius, color='blue', alpha=0.1)
        ax.add_patch(cc_circle)
        
        # 활성 릴레이 반경 (초록색)
        for r in active_relays[t]:
            r_circle = patches.Circle(coords[r], comm_radius, color='green', alpha=0.15)
            ax.add_patch(r_circle)

        # 4. 정적 노드 그리기
        # 제어센터
        ax.scatter(*coords['0'], c='blue', s=200, marker='s', label='Control Center')
        ax.text(coords['0'][0], coords['0'][1]+1.5, '0', fontsize=12, ha='center')
        
        # 목적지
        for c in data["sets"]["C"]:
            ax.scatter(*coords[c], c='red', s=150, marker='*', label='Destination' if c == data["sets"]["C"][0] else "")
            ax.text(coords[c][0], coords[c][1]+1.5, c, fontsize=12, ha='center')
            
        # 릴레이 (비활성/활성 구분)
        for r in data["sets"]["R"]:
            is_active = r in active_relays[t]
            color = 'green' if is_active else 'lightgray'
            size = 100 if is_active else 50
            ax.scatter(*coords[r], c=color, s=size, marker='^', label='Relay Node' if r == data["sets"]["R"][0] else "")
            ax.text(coords[r][0], coords[r][1]+1.5, r, fontsize=10, ha='center')

        # 5. 드론 위치 및 통신 상태 그리기
        for k in data["sets"]["K"]:
            if k in drone_pos[t]:
                px, py = drone_pos[t][k]
                
                # 상하역(정밀제어) 상태 확인
                is_unloading = z_vars[k, t].X > 0.5 if (k, t) in z_vars else False
                d_color = 'orange' if is_unloading else 'black'
                
                ax.scatter(px, py, c=d_color, s=80, marker='o', edgecolors='white', zorder=5)
                ax.text(px, py-2, k, fontsize=10, ha='center', weight='bold')

        # 중복 범례 제거
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys(), loc='upper right')

    # 초기 화면 렌더링
    update(1)
    time_slider.on_changed(update)
    plt.show()

# 사용 예시: 기존 모델의 m.optimize() 이후에 아래와 같이 변수를 넘겨서 호출
# visualize_gurobi_solution(m, data, x, v, y, z)