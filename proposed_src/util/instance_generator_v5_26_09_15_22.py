"""재난 배송 인스턴스 생성기 (v5, 26_09_15_22) — 도달 가능성 제약과 구성 무작위화.

v2 대비 두 가지가 다르다.
1) 모든 목적지가 드론 N대로 닿을 수 있는 거리(N x beta x 통신반경) 안에 있도록 보장한다.
   드론 3대에서는 기존 인스턴스 전부가 도달 불가 목적지를 포함했다(200/200).
2) 제어 센터 위치·목적지 수·드론 수를 인자로 받아 학습 중 무작위화할 수 있다.
기본 인자(드론 4대, CC 고정, 목적지 50)에서는 v2와 같은 시드가 같은 인스턴스를 낸다.
"""

import os

import numpy as np

DEFAULT_MAP = os.path.join(os.path.dirname(os.path.abspath(__file__)), "disaster_map.npy")
V2_CC = np.array([950.0, 500.0])
V2_LOW, V2_HIGH = np.array([50.0, 50.0]), np.array([850.0, 950.0])


def reach_limit(num_drones, comm_range=300.0, beta=0.9):
	"""드론 N대가 체인을 이뤄 닿을 수 있는 제어 센터로부터의 최대 거리."""
	return num_drones * beta * comm_range


def sample_instance(num_dests=50, seed=None, num_drones=4, comm_range=300.0, beta=0.9,
                    cc_pos=None, map_size=1000.0, margin=50.0, min_dist=60.0):
	"""제어 센터와 목적지 좌표를 생성해 (cc_pos, dests_pos)로 반환한다."""
	rng = np.random.default_rng(seed)
	random_cc = cc_pos is not None and isinstance(cc_pos, str) and cc_pos == "random"
	if random_cc:
		# 제어 센터를 지도 안 어디든 둔다. 목적지 영역도 지도 전체가 된다.
		cc = rng.uniform(low=[margin, margin], high=[map_size - margin] * 2)
		low, high = np.array([margin, margin]), np.array([map_size - margin] * 2)
	else:
		cc = V2_CC.copy() if cc_pos is None else np.asarray(cc_pos, dtype=float)
		low, high = V2_LOW, V2_HIGH
	limit = reach_limit(num_drones, comm_range, beta)
	dests = rng.uniform(low=low, high=high, size=(num_dests, 2))
	# 닿을 수 없거나 제어 센터에 붙어 있는 목적지만 골라 다시 뽑는다. 기본 구성에서는
	# 위반이 없어 v2와 같은 좌표가 나온다.
	for _ in range(10000):
		d = np.linalg.norm(dests - cc, axis=1)
		bad = np.flatnonzero((d > limit) | (d < min_dist))
		if len(bad) == 0:
			break
		dests[bad] = rng.uniform(low=low, high=high, size=(len(bad), 2))
	else:
		raise RuntimeError(f"도달 가능한 목적지 {num_dests}곳을 만들지 못했다 (한계 {limit:.0f})")
	return cc, dests


def sample_config(rng, drones=(3, 8), dests=(20, 80), random_cc=True):
	"""학습용으로 드론 수·목적지 수를 뽑고, 그에 맞는 인스턴스를 만든다."""
	n = int(rng.integers(drones[0], drones[1] + 1))
	m = int(rng.integers(dests[0], dests[1] + 1))
	seed = int(rng.integers(0, 2 ** 31 - 1))
	cc, d = sample_instance(m, seed=seed, num_drones=n, cc_pos="random" if random_cc else None)
	return n, m, cc, d


def generate_disaster_map(num_dests=50, filename=DEFAULT_MAP, seed=None):
	"""제어 센터 1곳과 목적지 num_dests곳의 좌표를 npy로 저장한다."""
	cc_pos, dests_pos = sample_instance(num_dests, seed)
	np.save(filename, {'cc_pos': cc_pos, 'dests_pos': dests_pos})
	print(f"지도 생성이 완료되었습니다: {filename}")


if __name__ == "__main__":
	generate_disaster_map()
