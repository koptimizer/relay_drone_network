"""규칙 기반 베이스라인을 제안 방법과 동일한 환경·인스턴스에서 평가한다.

통신 연결은 환경이 하드 제약으로 강제하므로 규칙 정책도 그대로 넣을 수 있다.
"RL이 뻔한 방법을 이기는가"에 답하기 위한 도구다.
"""

import argparse
import csv
import os
import sys

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v2_26_08_25_14 import DisasterRelayDroneEnv
from model.sac_net_v2_26_08_25_14 import SACActor
from util.instance_generator_v2_26_08_25_14 import sample_instance

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MAX_STEPS = 1000


def to_action(vec):
	"""방향 벡터를 환경 행동(각도, 속도비)으로 바꾼다. 영벡터면 정지."""
	n = np.linalg.norm(vec)
	if n < 1e-9:
		return np.array([0.0, 0.0])
	return np.array([np.arctan2(vec[1], vec[0]) % (2 * np.pi), 1.0])


def greedy(env, n_relay=0):
	"""적재 중이면 최근접 미배송지로, 비었으면 제어 센터로 향한다.

	n_relay > 0이면 앞의 n_relay대를 중계 전용으로 고정하고, 제어 센터에서 미배송지
	무게중심 방향으로 통신 반경의 0.9배 간격에 배치한다 (Steiner 배치의 단순 근사).
	"""
	acts = np.zeros((env.num_drones, 2))
	active = env.dests_pos[env.dests_active]

	if n_relay > 0 and len(active) > 0:
		direction = active.mean(axis=0) - env.cc_pos
		norm = np.linalg.norm(direction)
		unit = direction / norm if norm > 1e-9 else np.array([1.0, 0.0])

	for i in range(env.num_drones):
		if env.drones_timer[i] > 0:
			continue
		if i < n_relay and len(active) > 0:
			anchor = env.cc_pos + unit * (0.9 * env.comm_range * (i + 1))
			anchor = np.clip(anchor, 0, env.map_size[0])
			gap = anchor - env.drones_pos[i]
			acts[i] = to_action(gap) if np.linalg.norm(gap) > 15.0 else np.array([0.0, 0.0])
		elif env.drones_capacity[i] > 0 and len(active) > 0:
			tgt = active[np.argmin(np.linalg.norm(active - env.drones_pos[i], axis=1))]
			acts[i] = to_action(tgt - env.drones_pos[i])
		else:
			acts[i] = to_action(env.cc_pos - env.drones_pos[i])
	return acts


def random_policy(env, rng):
	"""균등 무작위 방향과 속도. 성능의 바닥값을 준다."""
	return np.stack([rng.uniform(0, 2 * np.pi, env.num_drones),
	                 rng.uniform(0, 1, env.num_drones)], axis=1)


def rollout(env, policy):
	"""한 에피소드를 끝까지 굴리고 지표를 반환한다."""
	env.reset()
	done, step = False, 0
	while not done and step < MAX_STEPS:
		_o, _s, _r, done = env.step(policy(env))
		step += 1
	return env.episode_stats()


def make_actor(ckpt, obs_dim, device, mode="angle"):
	"""체크포인트에서 결정론적 정책 함수를 만든다."""
	actor = SACActor(obs_dim, 2).to(device)
	actor.load_state_dict(torch.load(ckpt, map_location=device))
	actor.eval()

	def policy(env):
		with torch.no_grad():
			a, _ = actor(torch.FloatTensor(env._get_local_obs()).to(device), deterministic=True)
		a = a.cpu().numpy()
		if mode == "vector":
			return a
		return np.stack([(a[:, 0] + 1) * np.pi, (a[:, 1] + 1) * 0.5], axis=1)
	return policy


def main():
	"""규칙 베이스라인과(있으면) 학습 정책을 동일 인스턴스에서 비교한다."""
	p = argparse.ArgumentParser()
	p.add_argument("--n", type=int, default=30, help="평가 인스턴스 수")
	p.add_argument("--seed0", type=int, default=101)
	p.add_argument("--ckpt", nargs="*", default=[], help="비교할 학습 체크포인트 경로")
	p.add_argument("--n-peers", type=int, default=2)
	p.add_argument("--comm-range", type=float, default=300.0)
	p.add_argument("--projection", choices=["cancel", "scale"], default="scale")
	p.add_argument("--modes", nargs="*", default=[], help="각 --ckpt의 action_mode (angle/vector)")
	p.add_argument("--out", default="baseline_v2_26_08_25_14")
	args = p.parse_args()

	rng = np.random.default_rng(0)
	methods = [
		("random", lambda e: random_policy(e, rng), 2),
		("greedy", lambda e: greedy(e, 0), 2),
		("greedy + 1 relay", lambda e: greedy(e, 1), 2),
		("greedy + 2 relay", lambda e: greedy(e, 2), 2),
	]
	for i, c in enumerate(args.ckpt):
		mode = args.modes[i] if i < len(args.modes) else "angle"
		methods.append((os.path.basename(os.path.dirname(c)).replace("v2_26_08_25_14_", ""),
		                None, args.n_peers, c, mode))

	rows = []
	for m in methods:
		name, fn, npeers = m[0], m[1], m[2]
		mode = m[4] if len(m) > 4 else "angle"
		env = DisasterRelayDroneEnv(n_peers=npeers, comm_range=args.comm_range, action_mode=mode,
		                            projection=args.projection)
		if fn is None:
			fn = make_actor(m[3], env.reset().shape[-1], torch.device("cpu"), mode)
		stats = []
		for s in range(args.seed0, args.seed0 + args.n):
			env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=s)
			stats.append(rollout(env, fn))
		g = lambda k: np.mean([st[k] for st in stats])
		rows.append({
			"method": name,
			"delivered": round(g("delivered"), 2),
			"delivered_sd": round(float(np.std([st["delivered"] for st in stats])), 2),
			"completion_rate": round(g("completion_rate"), 4),
			"idle_drones": round(g("idle_drones"), 2),
			"reloads": round(g("reloads"), 2),
			"hop2plus": round(g("hop2plus"), 3),
			"max_reach": round(g("max_reach"), 0),
			"blocked": round(g("blocked"), 0),
			"comm_loss": int(sum(st["comm_loss"] for st in stats)),
			"full_completion": sum(1 for st in stats if st["makespan"] is not None),
		})
		print(f"  {name}: 배송 {rows[-1]['delivered']}/50 완료", flush=True)

	cols = list(rows[0].keys())
	w = {c: max(len(c), max(len(str(r[c])) for r in rows)) for c in cols}
	print(f"\n홀드아웃 {args.n}개 인스턴스 (시드 {args.seed0}~{args.seed0 + args.n - 1}), 통신반경 {args.comm_range:.0f}\n")
	print(" | ".join(c.ljust(w[c]) for c in cols))
	print("-" * (sum(w.values()) + 3 * len(cols)))
	for r in rows:
		print(" | ".join(str(r[c]).ljust(w[c]) for c in cols))

	os.makedirs(os.path.join(ROOT, "figures"), exist_ok=True)
	out = os.path.join(ROOT, "figures", f"{args.out}.csv")
	with open(out, "w", newline="", encoding="utf-8") as f:
		wr = csv.DictWriter(f, fieldnames=cols)
		wr.writeheader()
		wr.writerows(rows)
	print(f"\n저장: {out}")


if __name__ == "__main__":
	main()
