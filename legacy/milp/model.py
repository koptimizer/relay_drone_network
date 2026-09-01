import math
import json
import random
import gurobipy as gp
from gurobipy import GRB
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import matplotlib.patches as patches
import numpy as np

# ==========================================
# 1. 시나리오 기반 소형 인스턴스 생성기 (Peripheral Origin)
# ==========================================
def generate_simplified_small_instance():
    """
    [요청 시나리오 반영] 거점이 존재하지 않고, 중앙 센터가 오른쪽 하단 주변부에 위치하며, 
    왼쪽에 넓게 목적지가 존재하는 Homogeneous 군집 드론 최적화 문제 인스턴스를 생성합니다.
    """
    
    # 1. 하드코딩 파라미터 설정
    instance_size = "simplified_small"
    num_drones = 2
    # [시나리오] 고정 릴레이 노드(R)는 0개
    num_relay = 0
    num_dest = 2
    # 타임 스텝 수 (넉넉하게)
    T_max = 30
    # 지도 영역 크기
    grid_size = 20

    random.seed(42) # 재현성

    # 2. 노드 생성 및 좌표 할당
    # [시나리오] 제어 센터는 원점이 아닌 지도 오른쪽 주변부 (예: GridSize/2, 0)
    nodes = {'0': (grid_size, 0)} 
    dest_nodes = [f'C{i+1}' for i in range(num_dest)]
    relay_nodes = [] 
    
    # [시나리오] 목적지 좌표: 그리드 왼쪽 영역에 넓게 분포 유도
    # X 좌표 범위: (-GridSize, 0), Y 좌표 범위: (-GridSize, GridSize)
    for c in dest_nodes:
        nodes[c] = (round(random.uniform(-grid_size, 0), 3), 
                    round(random.uniform(-grid_size, grid_size), 3))

    all_nodes = list(nodes.keys())
    
    # 3. 글로벌 물리 파라미터 설정
    speed = 5.0 # 드론 이동 속도 (단위 거리 / 1 타임 스텝)
    P_0 = 1000.0 # 송신 출력 상수 (통신 반경 원점 복귀용)
    alpha = 2.0 # 경로 손실 지수
    B_low = 1.0 # 이동 중 필요 대역폭
    B_high = 5.0 # 상하역 중 필요 대역폭
    S_c = 2 # 목적지별 상하역 소요 타임 스텝 수

    # 4. 거리, 이동 시간(\tau_{ij}), 통신 용량(C_{ij}) 행렬 계산
    tau_matrix = {}
    capacity_matrix = {}
    
    for i in all_nodes:
        tau_matrix[i] = {}
        capacity_matrix[i] = {}
        for j in all_nodes:
            if i == j:
                tau_matrix[i][j] = 0
                capacity_matrix[i][j] = 9999.0 # 자기 자신 무한대 통신
            else:
                dist = math.dist(nodes[i], nodes[j])
                tau_matrix[i][j] = math.ceil(dist / speed)
                # 통신 용량 (거리에 따른 감쇄)
                capacity_matrix[i][j] = round(P_0 / (dist ** alpha), 3)

    # 5. 결과 통합 딕셔너리 구성 (gurobipy 입력용)
    instance_data = {
        "instance_size": instance_size,
        "sets": {
            "N": all_nodes,       # 전체 노드 세트
            "C": dest_nodes,    # 목적지 노드 세트
            "R": relay_nodes,   # 고정 릴레이 노드 세트 (빈 리스트)
            "K": [f'K{i+1}' for i in range(num_drones)], # 통합 군집 드론 세트 ('K1', 'K2')
            "T": list(range(1, T_max + 1)) # 시간 스텝 세트
        },
        "parameters": {
            "B_low": B_low,
            "B_high": B_high,
            "S_c": S_c,
            "T_max": T_max,
            "epsilon": 0.001   # 목적함수 내 체공 시간 페널티 가중치
        },
        "matrices": {
            "tau_ij": tau_matrix, # 노드 i->j 이동 시간 행렬 (\tau_{ij})
            "C_ij": capacity_matrix # 노드 i->j 통신 용량 행렬 (C_{ij})
        },
        "node_coordinates": nodes # (X, Y) 좌표 행렬
    }
    
    # gurobipy 호환성: matrices 내부 딕셔너리의 키를 tuple 형태로 변환
    # (JSON 저장 시에는 다시 string으로 바뀌지만, 메모리 내에서는 tuple이 편리함)
    for matrix_name in ["tau_ij", "C_ij"]:
        formatted_matrix = {}
        for i in all_nodes:
            for j in all_nodes:
                formatted_matrix[i, j] = instance_data["matrices"][matrix_name][i][j]
        instance_data["matrices"][matrix_name] = formatted_matrix

    return instance_data

# ==========================================
# 2. 통합 군집 드론 수리모형 솔버 (Homogeneous Swarm)
# ==========================================
class UnifiedSwarmRoutingSolver:
    def __init__(self, instance_data):
        """
        인스턴스 데이터를 로드하고 동질적 군집 모델을 초기화합니다.
        """
        self.data = instance_data
        self.sets = self.data["sets"]
        self.params = self.data["parameters"]
        self.matrices = self.data["matrices"]
        self.coords = self.data["node_coordinates"]

        # 드론별 목적지 단순 할당
        dest_nodes = self.sets["C"]
        drone_nodes = self.sets["K"]
        self.k_dest_map = {drone_nodes[i]: dest_nodes[i % len(dest_nodes)] for i in range(len(drone_nodes))}
        
        self.model = gp.Model(f"Unified_Swarm_Routing_{self.data['instance_size']}")
        self.vars = {}

    def build_model(self):
        """
        모든 드론이 배송과 중계를 겸하는 선형 수리모형을 빌드합니다.
        """
        m = self.model
        N = self.sets["N"]
        K = self.sets["K"]
        T = self.sets["T"]
        tau = self.matrices["tau_ij"]
        cap = self.matrices["C_ij"]
        
        # --- 의사결정 변수 ---
        # x: 드론 k가 t에 i->j 출발
        x = m.addVars([(i, j, k, t) for i in N for j in N for k in K for t in T if i != j], vtype=GRB.BINARY, name="x")
        # v: 드론 k가 t에 노드 i에 상주
        v = m.addVars([(i, k, t) for i in N for k in K for t in T], vtype=GRB.BINARY, name="v")
        # z: 드론 k가 t에 상하역(High-BW) 중
        z = m.addVars([(k, t) for k in K for t in T], vtype=GRB.BINARY, name="z")
        
        # [핵심 로직] 통신 흐름 변수 f[i,j,k_target,t]: 드론 k_target을 위해 i->j로 흐르는 데이터량
        # Sink는 목적지가 아닌 "드론 k_target 본인의 현재 위치"
        f = m.addVars([(i, j, k, t) for i in N for j in N for k in K for t in T if i != j], vtype=GRB.CONTINUOUS, lb=0, name="f")
        
        C_k = m.addVars(K, vtype=GRB.CONTINUOUS, name="C_k")
        C_max = m.addVar(vtype=GRB.CONTINUOUS, name="C_max")

        self.vars.update({'x': x, 'v': v, 'z': z, 'f': f, 'C_k': C_k, 'C_max': C_max})

        # --- 목적함수 ---
        # Makespan 최소화 + 공중 체공 시간(비행 비용) 최소화 (ASAP 최우선)
        flight_cost = gp.quicksum(v[i, k, t] for i in N if i != '0' for k in K for t in T)
        epsilon = self.params["epsilon"]
        m.setObjective(C_max + epsilon * flight_cost, GRB.MINIMIZE)

        # --- 제약식 ---
        # 제약 1: Makespan
        for k in K:
            m.addConstr(C_max >= C_k[k], name=f"makespan_{k}")
            for t in T:
                m.addConstr(C_k[k] >= t * z[k, t], name=f"completion_{k}_{t}")

        # 제약 2: [요청사항 완벽 반영] 모든 드론은 t=1에 반드시 제어 센터('0')에서 시작
        for k in K:
            m.addConstr(v['0', k, 1] + gp.quicksum(x['0', j, k, 1] for j in N if j != '0') == 1, name=f"start_at_center_{k}")
            for i in N:
                if i != '0': 
                    m.addConstr(v[i, k, 1] == 0, name=f"no_start_outside_{i}_{k}")

        # 제약 3: 시간-공간 흐름 보존 (t > 1)
        T_max = self.params["T_max"]
        for k in K:
            for i in N:
                for t in range(2, T_max + 1):
                    # 과거 출발 도착 흐름
                    arrival = gp.quicksum(x[j, i, k, t - tau[j, i]] for j in N if j != i and t - tau[j, i] >= 1)
                    # 현재 출발 흐름
                    departure = gp.quicksum(x[i, j, k, t] for j in N if j != i)
                    m.addConstr(v[i, k, t] == v[i, k, t - 1] + arrival - departure, name=f"ts_balance_{i}_{k}_{t}")

        # 제약 4: 목적지 상하역 소요 시간 및 정밀제어 활성화
        S_c = self.params["S_c"]
        for k in K:
            dest = self.k_dest_map[k]
            m.addConstr(gp.quicksum(v[dest, k, t] for t in T) >= S_c, name=f"visit_{k}")
            m.addConstr(gp.quicksum(z[k, t] for t in T) == S_c, name=f"unload_time_{k}")
            for t in T:
                m.addConstr(z[k, t] <= v[dest, k, t], name=f"z_act_{k}_{t}")

        # 제약 5 & 6 [기존 오류 수정]: 동적 데이터 흐름 및 상호 릴레이 제약
        # 비선형 b[k,t] 변수를 없애고 드론의 상태(v, z)로 직접 Sink 흐름 설정
        B_low = self.params["B_low"]
        B_high = self.params["B_high"]
        
        for k_target in K:
            dest = self.k_dest_map[k_target]
            for t in T:
                # 제어 센터('0')는 k_target이 요구하는 대역폭만큼 생성 (Source)
                # SinkReq_k_t = B_low + (B_high-B_low)*z[k_target,t]
                
                # 목적지('dest') 흐름 보존 (Sink)
                in_flow_dest = gp.quicksum(f[j, dest, k_target, t] for j in N if j != dest)
                out_flow_dest = gp.quicksum(f[dest, j, k_target, t] for j in N if j != dest)
                # 목적지 도착 시: z=1이면 B_high 소모, z=0이면 B_low 소모
                m.addConstr(in_flow_dest - out_flow_dest == B_low * v[dest, k_target, t] + (B_high - B_low) * z[k_target, t], 
                            name=f"snk_dest_{k_target}_{t}")
                
                # Source ('0') 흐름 보존
                in_flow_src = gp.quicksum(f[j, '0', k_target, t] for j in N if j != '0')
                out_flow_src = gp.quicksum(f['0', j, k_target, t] for j in N if j != '0')
                # Source는 총 Sink 요구량 생성
                m.addConstr(out_flow_src - in_flow_src == B_low + (B_high - B_low) * z[k_target, t], 
                            name=f"src_flow_{k_target}_{t}")
                
                # 목적지 이외의 일반 노드 (Relay or Intermediate Sink)
                for i in N:
                    if i != '0' and i != dest:
                        in_flow = gp.quicksum(f[j, i, k_target, t] for j in N if j != i)
                        out_flow = gp.quicksum(f[i, j, k_target, t] for j in N if j != i) # <-- N으로 정상화
                        # k_target 본인이 체공 중이면 B_low 소모, 아니면 중계(in=out)
                        m.addConstr(in_flow - out_flow == B_low * v[i, k_target, t], name=f"snk_mid_{i}_{k_target}_{t}")

        # 제약 7 [핵심 수정]: 물리적 채널 용량 및 노드 활성화 동기화 (Homogeneous Relay)
        # i와 j 노드 양쪽에 '어떤 군집 드론이든' 체공 중이어야만 통신망이 유도됨
        for t in T:
            for i in N:
                for j in N:
                    if i != j:
                        # 통신 i<->j 사이의 총 데이터 흐름 (양방향 합)
                        total_flow = gp.quicksum(f[i, j, k, t] + f[j, i, k, t] for k in K)
                        
                        # 제약 센터('0')는 드론 없이도 활성
                        if i == '0':
                            # j 노드에는 드론이 있어야 함
                            m.addConstr(total_flow <= cap[i, j] * gp.quicksum(v[j, k, t] for k in K), name=f"cap_src_in_{i}_{j}_{t}")
                        elif j == '0':
                            # i 노드에는 드론이 있어야 함
                            m.addConstr(total_flow <= cap[i, j] * gp.quicksum(v[i, k, t] for k in K), name=f"cap_src_out_{i}_{j}_{t}")
                        else:
                            # 둘 다 군집 드론이어야 함 (P2P 통신)
                            m.addConstr(total_flow <= cap[i, j] * gp.quicksum(v[i, k, t] for k in K), name=f"cap_P2P_i_{i}_{j}_{t}")
                            m.addConstr(total_flow <= cap[i, j] * gp.quicksum(v[j, k, t] for k in K), name=f"cap_P2P_j_{i}_{j}_{t}")

    def export_lp_and_solve(self):
        """
        모델 구조를 .lp 파일로 저장하고 최적화를 수행합니다.
        """
        self.model.update()
        lp_filename = "Unified_Swarm_Model.lp"
        self.model.write(lp_filename)
        print(f"\n[성공] 동질적 군집 수리모형 구조가 '{lp_filename}' 파일로 저장되었습니다.")

        self.model.optimize()
        return self.model.Status == GRB.OPTIMAL

# ==========================================
# 3. 통합 군집 드론 비주얼라이저 (P2P Comm Visualization)
# ==========================================
class SwarmVisualizer:
    def __init__(self, solver_instance):
        """
        최적화가 완료된 솔버 인스턴스를 입력받아 시각화 데이터를 구성합니다.
        """
        self.solver = solver_instance
        self.data = solver_instance.data
        self.sets = solver_instance.sets
        self.vars = solver_instance.vars
        
        # 통신 반경 역산 (Low 대역폭 만족 최대 거리)
        P_0 = 1000.0  # 송신 출력 상수
        B_low = self.solver.params["B_low"]
        self.comm_radius = math.sqrt(P_0 / B_low)

    def process_solution_data(self):
        """
        Gurobi 최적해 변수(BINARY)를 시간 t별 (x, y) 좌표 및 상태 데이터로 전처리합니다.
        """
        T_max = self.solver.params["T_max"]
        K = self.sets["K"]
        coords = self.solver.coords
        
        # drone_pos[t][k] = (x, y), unloading_status[t][k] = True/False
        self.drone_pos = {t: {} for t in self.sets["T"]}
        self.unloading_status = {t: {k: False for k in K} for t in self.sets["T"]}

        x_v, v_v, z_v = self.vars['x'], self.vars['v'], self.vars['z']
        
        for t in self.sets["T"]:
            for k in K:
                # 상하역 상태 확인
                if z_v[k, t].X > 0.5:
                    self.unloading_status[t][k] = True
                    
                hovering = False
                for i in self.sets["N"]:
                    if v_v[i, k, t].X > 0.5:
                        self.drone_pos[t][k] = coords[i]
                        hovering = True
                        break
                
                # 이동 중 (좌표 보간)
                if not hovering:
                    for i in self.sets["N"]:
                        for j in self.sets["N"]:
                            if i != j:
                                tau_ij = self.solver.matrices["tau_ij"][i, j]
                                for start_t in range(max(1, t - tau_ij + 1), t + 1):
                                    if x_v[i, j, k, start_t].X > 0.5:
                                        progress = (t - start_t) / tau_ij
                                        x_c = coords[i][0] + (coords[j][0] - coords[i][0]) * progress
                                        y_c = coords[i][1] + (coords[j][1] - coords[i][1]) * progress
                                        self.drone_pos[t][k] = (x_c, y_c)
                                        break

    def run_animation(self):
        """
        Matplotlib 슬라이더 애니메이션을 실행합니다.
        """
        self.process_solution_data()

        coords = self.solver.coords
        T_max = self.solver.params["T_max"]
        dest_nodes = self.sets["C"]
        drone_nodes = self.sets["K"]
        
        fig, ax = plt.subplots(figsize=(10, 8))
        plt.subplots_adjust(bottom=0.2)
        ax_slider = plt.axes([0.15, 0.05, 0.7, 0.03])
        time_slider = Slider(ax_slider, 'Time (t)', 1, T_max, valinit=1, valstep=1)

        def update(val):
            t = int(time_slider.val)
            ax.clear()
            # [시나리오] 주변부 센터 및 왼쪽 목적지에 맞게 축 설정
            grid_size = 20
            ax.set_xlim(-25, 25)
            ax.set_ylim(-25, 25)
            ax.grid(True, linestyle='--', alpha=0.5)
            ax.set_title(f"Dynamic Swarm Relay Delivery (Homogeneous) - Time: {t}", fontsize=12)

            # 통신 반경 원 그리기 (원활도 검증)
            # 센터 반경 (파란색)
            cc_circle = patches.Circle(coords['0'], self.comm_radius, color='blue', alpha=0.07, label='Center Boundary')
            ax.add_patch(cc_circle)
            
            # [핵심] 군집 드론 자체의 통신 반경 (초록색) - P2P 중계를 가시화
            for k in drone_nodes:
                if k in self.drone_pos[t]:
                    pos = self.drone_pos[t][k]
                    # 제어 센터('0')에 복귀해 대기 중인 상태가 아닐 때만 그림
                    if math.dist(pos, coords['0']) > 0.1:
                        d_circle = patches.Circle(pos, self.comm_radius, color='green', alpha=0.12, label='Swarm P2P Bounds' if k == drone_nodes[0] else "")
                        ax.add_patch(d_circle)

            # 노드 마킹
            # 제어센터 (네모) - 오른쪽 주변부
            ax.scatter(*coords['0'], c='blue', s=200, marker='s', zorder=3, label='Center')
            ax.text(coords['0'][0], coords['0'][1]+1.5, 'Center', fontsize=12, ha='center')
            
            # 목적지 (별) - 왼쪽
            for c in dest_nodes:
                ax.scatter(*coords[c], c='red', s=150, marker='*', zorder=3, label='Destination' if c == dest_nodes[0] else "")
                ax.text(coords[c][0], coords[c][1]+1.5, c, fontsize=12, ha='center')

            # 이동 드론 및 상하역 상태 그리기 (모두 동일 능력)
            for k in drone_nodes:
                if k in self.drone_pos[t]:
                    px, py = self.drone_pos[t][k]
                    
                    is_unloading = self.unloading_status[t][k]
                    d_color = 'orange' if is_unloading else 'black' # 상하역 중엔 오렌지색
                    
                    ax.scatter(px, py, c=d_color, s=100, marker='o', edgecolors='white', zorder=5)
                    bw_label = 'High' if is_unloading else 'Low'
                    ax.text(px, py-1.8, f"{k}({bw_label})", fontsize=9, ha='center', weight='bold')

            # 범례 위치 조정
            handles, labels = ax.get_legend_handles_labels()
            by_label = dict(zip(labels, handles))
            ax.legend(by_label.values(), by_label.keys(), loc='upper right')

        # 초기 렌더링
        update(1)
        time_slider.on_changed(update)
        plt.show()

# ==========================================
# 4. 메인 실행부
# ==========================================
if __name__ == "__main__":
    # 1. 시나리오 기반 소형 인스턴스 생성
    print("\n--- [시나리오 반영] 거점 없는 소형 인스턴스 생성 중 ---")
    small_instance_data = generate_simplified_small_instance()
    print(f"생성 완료. 제어 센터 좌표: {small_instance_data['node_coordinates']['0']}")

    # 2. 통합 군집 드론 솔버 객체 생성 및 실행
    solver = UnifiedSwarmRoutingSolver(small_instance_data)
    solver.build_model()
    
    if solver.model.Status == gp.GRB.INFEASIBLE:
        print("\n[진단] 모델이 비현실적입니다. IIS를 계산합니다.")
        
        # 2. IIS 계산 실행
        solver.model.computeIIS()
        
        # 3. IIS 결과를 텍스트 파일(.ilp)로 저장
        iis_filename = "irreducible_infeasible_subsystem.ilp"
        solver.model.write(iis_filename)
        
        print(f"[알림] 충돌하는 제약식 집합(IIS)이 '{iis_filename}' 파일로 저장되었습니다.")
        print("       이 파일을 메모장 등으로 열어 어떤 제약식들이 충돌하는지 확인하세요.")
    
    else:

        print("\n--- 동질적 군집 P2P 라우팅 최적화 시작 ---")
        if solver.export_lp_and_solve():
            print("최적해 도출 성공! 비주얼라이저를 구동합니다.")
            # 3. 비주얼라이저 객체 생성 및 애니메이션 실행
            visualizer = SwarmVisualizer(solver)
            visualizer.run_animation()
        else:
            print("\n최적해를 찾지 못했습니다. Status Code:", solver.model.Status)