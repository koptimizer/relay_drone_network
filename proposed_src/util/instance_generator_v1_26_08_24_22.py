"""재난 배송 인스턴스(제어 센터 + 목적지 좌표) 생성기 (v1, 26-08-24-22).

instance_generator.py를 분리한 것으로 생성 규칙 변경은 없다.
기본 저장 경로만 CWD 기준에서 util 폴더 기준으로 바로잡았다.
"""

import os

import numpy as np

DEFAULT_MAP = os.path.join(os.path.dirname(os.path.abspath(__file__)), "disaster_map.npy")


def sample_instance(num_dests=50, seed=None):
	"""제어 센터와 목적지 좌표를 생성해 (cc_pos, dests_pos)로 반환한다."""
	rng = np.random.default_rng(seed)
	# 1. 제어 센터 위치 (오른쪽 끝 중앙)
	cc_pos = np.array([950.0, 500.0])
	# 2. 목적지 위치 (제어 센터보다 왼쪽 영역에 골고루 분포)
	# X: 50~850, Y: 50~950 사이에서 num_dests개 생성
	dests_pos = rng.uniform(low=[50, 50], high=[850, 950], size=(num_dests, 2))
	return cc_pos, dests_pos


def generate_disaster_map(num_dests=50, filename=DEFAULT_MAP, seed=None):
	"""제어 센터 1곳과 목적지 num_dests곳의 좌표를 npy로 저장한다."""
	cc_pos, dests_pos = sample_instance(num_dests, seed)

	# 데이터를 딕셔너리 형태로 묶어서 저장
	map_data = {
		'cc_pos': cc_pos,
		'dests_pos': dests_pos
	}

	np.save(filename, map_data)
	print(f"지도 생성이 완료되었습니다: {filename}")


if __name__ == "__main__":
	generate_disaster_map()
