"""체크포인트를 훑어 홀드아웃 인스턴스에서의 성능 곡선을 만든다.

학습 곡선은 고정 학습 지도 기준이라 일반화 성능을 말해주지 않는다.
처음 보는 인스턴스에서 학습량 대비 성능이 어떻게 변하는지를 직접 측정한다.
"""

import argparse
import csv
import os
import sys
from multiprocessing import Pool

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MAX_STEPS = 1000


def make_env(legacy, relay_w):
	"""구환경(legacy) 또는 신환경 인스턴스를 만든다."""
	if legacy:
		path = os.path.join(ROOT, "legacy", "pre_v1", "CTDERL.py")
		head = open(path, encoding="utf-8").read().partition("# --- 4. Training Loop ---")[0]
		ns = {}
		exec(head, ns)
		return ns["DisasterRelayDroneEnv"]()
	from env.disaster_relay_env_v1_26_08_25_04 import DisasterRelayDroneEnv
	return DisasterRelayDroneEnv(relay_w=relay_w)


def hop_depth(pos, cc, R):
	"""각 드론의 제어 센터까지 홉 수 (1=직접 연결, -1=단절)."""
	n = len(pos)
	h = np.full(n, -1)
	frontier = [i for i in range(n) if np.linalg.norm(pos[i] - cc) <= R]
	for i in frontier:
		h[i] = 1
	d = np.linalg.norm(pos[:, None, :] - pos[None, :, :], axis=2)
	k = 1
	while frontier:
		nxt = [i for i in range(n) if h[i] == -1 and any(d[i, j] <= R for j in frontier)]
		for i in nxt:
			h[i] = k + 1
		frontier = nxt
		k += 1
	return h


def job(task):
	"""체크포인트 하나를 seeds개 인스턴스에서 평가하고 요약 지표를 반환한다."""
	from model.sac_net_v1_26_08_24_22 import SACActor
	from util.instance_generator_v1_26_08_24_22 import sample_instance
	torch.set_num_threads(1)
	ckpt, ep, legacy, relay_w, seeds = task

	env = make_env(legacy, relay_w)
	actor = SACActor(env.reset().shape[-1], 2)
	actor.load_state_dict(torch.load(ckpt, map_location="cpu"))
	actor.eval()

	D, I, H, RE, CL = [], [], [], [], []
	for s in seeds:
		env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=s)
		obs = env.reset()
		deliv = np.zeros(env.num_drones, dtype=int)
		prev = env.dests_active.copy()
		hops, reach, cl, step, done = [], 0.0, 0, 0, False
		while not done and step < MAX_STEPS:
			with torch.no_grad():
				a, _ = actor(torch.FloatTensor(obs), deterministic=True)
			a = a.numpy()
			obs, _s, _r, done = env.step(
				np.stack([(a[:, 0] + 1) * np.pi, (a[:, 1] + 1) * 0.5], axis=1))
			new = prev & ~env.dests_active
			if new.any():
				near = np.linalg.norm(
					env.drones_pos[:, None, :] - env.dests_pos[None, new, :], axis=2)
				for c in range(new.sum()):
					deliv[int(np.argmin(near[:, c]))] += 1
			prev = env.dests_active.copy()
			hops.append(hop_depth(env.drones_pos, env.cc_pos, env.comm_range))
			reach = max(reach, float(np.max(np.linalg.norm(env.drones_pos - env.cc_pos, axis=1))))
			if not env.comm_status.all():
				cl = 1
			step += 1
		D.append(int(env.num_dests - env.dests_active.sum()))
		I.append(int((deliv == 0).sum()))
		H.append(float(np.mean(np.array(hops) >= 2)))
		RE.append(reach)
		CL.append(cl)
	return {
		"episode": ep, "delivered": float(np.mean(D)), "delivered_sd": float(np.std(D)),
		"idle_drones": float(np.mean(I)), "hop2plus": float(np.mean(H)),
		"max_reach": float(np.mean(RE)), "comm_loss": int(sum(CL)), "n": len(seeds),
	}


def main():
	"""한 실행의 체크포인트들을 훑어 결과를 CSV로 저장한다."""
	p = argparse.ArgumentParser()
	p.add_argument("--tag", required=True)
	p.add_argument("--every", type=int, default=200, help="체크포인트 샘플 간격 (에피소드)")
	p.add_argument("--n", type=int, default=8, help="체크포인트당 평가 인스턴스 수")
	p.add_argument("--relay-w", type=float, default=0.0)
	p.add_argument("--legacy", action="store_true")
	p.add_argument("--workers", type=int, default=8)
	args = p.parse_args()

	wdir = os.path.join(ROOT, "weights", args.tag)
	eps = sorted(int(f[12:-4]) for f in os.listdir(wdir) if f.startswith("sac_actor_ep"))
	eps = [e for e in eps if e % args.every == 0] or eps[:: max(1, len(eps) // 20)]
	seeds = list(range(101, 101 + args.n))   # 학습에 쓰지 않은 시드 대역
	tasks = [(os.path.join(wdir, f"sac_actor_ep{e}.pth"), e, args.legacy, args.relay_w, seeds)
	         for e in eps]

	print(f"{args.tag}: 체크포인트 {len(tasks)}개 × 인스턴스 {args.n}개", flush=True)
	with Pool(args.workers) as pool:
		rows = pool.map(job, tasks)

	out = os.path.join(ROOT, "figures", f"sweep_{args.tag}.csv")
	os.makedirs(os.path.dirname(out), exist_ok=True)
	with open(out, "w", newline="", encoding="utf-8") as f:
		w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
		w.writeheader()
		w.writerows(rows)
	best = max(rows, key=lambda r: r["delivered"])
	print(f"최고: ep{best['episode']} 배송 {best['delivered']:.1f}±{best['delivered_sd']:.1f} "
	      f"미활동 {best['idle_drones']:.2f} 2홉+ {best['hop2plus']:.2f}")
	print(f"최종: ep{rows[-1]['episode']} 배송 {rows[-1]['delivered']:.1f}")
	print(f"저장: {out}")


if __name__ == "__main__":
	main()
