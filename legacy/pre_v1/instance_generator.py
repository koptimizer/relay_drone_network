import numpy as np

def generate_disaster_map(num_dests=50, filename="disaster_map.npy"):
    # 1. 제어 센터 위치 (오른쪽 끝 중앙)
    cc_pos = np.array([950.0, 500.0])
    
    # 2. 목적지 위치 (제어 센터보다 왼쪽 영역에 골고루 분포)
    # X: 50~850, Y: 50~950 사이에서 50개 생성
    dests_pos = np.random.uniform(low=[50, 50], high=[850, 950], size=(num_dests, 2))
    
    # 데이터를 딕셔너리 형태로 묶어서 저장
    map_data = {
        'cc_pos': cc_pos,
        'dests_pos': dests_pos
    }
    
    np.save(filename, map_data)
    print(f"지도 생성이 완료되었습니다: {filename}")

if __name__ == "__main__":
    generate_disaster_map()