"""v3 상위 가중치를 v4의 확장된 관측 차원으로 이식한다.

manager_obs가 43에서 49로 늘었으므로 첫 층 가중치에 0열 6개를 덧붙인다.
새 채널의 초기 기여가 0이라 이식 직후 v4 정책은 v3와 같은 함수에서 출발한다.
"""

import argparse
import os
import sys

import torch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def transplant(src, dst, old_dim, new_dim):
	"""첫 층 가중치를 0-패딩해 넓히고 나머지 층은 그대로 옮긴다."""
	sd = torch.load(src, map_location="cpu")
	key = "net.0.weight"
	w = sd[key]
	if w.shape[1] == new_dim:
		print(f"이미 {new_dim}차원이다. 그대로 복사한다.")
	else:
		assert w.shape[1] == old_dim, f"입력 차원이 {w.shape[1]}로 예상({old_dim})과 다르다"
		wide = torch.zeros(w.shape[0], new_dim, dtype=w.dtype)
		wide[:, :old_dim] = w
		sd[key] = wide
	os.makedirs(os.path.dirname(dst), exist_ok=True)
	torch.save(sd, dst)
	return sd


def verify(src, dst, old_dim, new_dim, n_act, trials=64):
	"""같은 43차원 입력에 대해 이식 전후 출력이 일치하는지 확인한다."""
	from model.hier_net_v3_26_08_31_19 import ManagerActor
	old = ManagerActor(old_dim, n_act)
	old.load_state_dict(torch.load(src, map_location="cpu"))
	new = ManagerActor(new_dim, n_act)
	new.load_state_dict(torch.load(dst, map_location="cpu"))
	old.eval(), new.eval()
	g = torch.Generator().manual_seed(0)
	x = torch.randn(trials, old_dim, generator=g)
	xp = torch.cat([x, torch.zeros(trials, new_dim - old_dim)], dim=1)
	m = torch.ones(trials, n_act, dtype=torch.bool)
	with torch.no_grad():
		_a1, p1, _l1 = old(x, m)
		_a2, p2, _l2 = new(xp, m)
	gap = (p1 - p2).abs().max().item()
	return gap


def main():
	"""v3 상위 가중치를 v4 차원으로 넓히고 동치성을 검증한다."""
	p = argparse.ArgumentParser()
	p.add_argument("--src", default="weights/v3_26_09_04_15_m_s1")
	p.add_argument("--dst", default="weights/v4_26_09_14_03_seed")
	p.add_argument("--old-dim", type=int, default=43)
	p.add_argument("--new-dim", type=int, default=49)
	p.add_argument("--n-act", type=int, default=7)
	a = p.parse_args()

	src_m = os.path.join(ROOT, a.src, "best_manager.pth")
	dst_m = os.path.join(ROOT, a.dst, "best_manager.pth")
	transplant(src_m, dst_m, a.old_dim, a.new_dim)

	src_w = os.path.join(ROOT, a.src, "best_worker.pth")
	dst_w = os.path.join(ROOT, a.dst, "best_worker.pth")
	os.makedirs(os.path.dirname(dst_w), exist_ok=True)
	torch.save(torch.load(src_w, map_location="cpu"), dst_w)   # 하위는 차원 불변

	gap = verify(src_m, dst_m, a.old_dim, a.new_dim, a.n_act)
	print(f"상위 이식: {a.old_dim} -> {a.new_dim}차원")
	print(f"하위 복사: 차원 불변 (16)")
	print(f"동치성 검증: 같은 입력에 대한 행동 확률 최대 오차 {gap:.3e}")
	if gap > 1e-6:
		print("경고: 이식 전후 출력이 다르다. 0-패딩이 잘못됐을 수 있다.")
		sys.exit(1)
	print(f"통과 — v4는 v3와 동일한 정책에서 출발한다. 저장: {a.dst}")


if __name__ == "__main__":
	main()
