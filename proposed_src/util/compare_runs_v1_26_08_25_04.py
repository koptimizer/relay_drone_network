"""발표 자료(pre-v1) 모델과 v1 실행들을 동일 인스턴스에서 비교 평가한다.

구·신 환경은 보상 체계가 다르므로 보상이 아니라 환경 결과 지표(배송 수, 미활동 드론,
통신 단절, makespan, 홉 분포)로만 비교한다. 결과는 표와 figures/ 그래프로 남긴다.
"""

import argparse
import csv
import os
import sys

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v1_26_08_24_22 import DisasterRelayDroneEnv
from model.sac_net_v1_26_08_24_22 import SACActor
from util.instance_generator_v1_26_08_24_22 import sample_instance

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MAX_STEPS = 1000


def load_legacy_env():
	"""legacy/pre_v1/CTDERL.py의 구환경 클래스를 학습 루프 제외하고 로드한다."""
	path = os.path.join(ROOT, "legacy", "pre_v1", "CTDERL.py")
	head = open(path, encoding="utf-8").read().partition("# --- 4. Training Loop ---")[0]
	ns = {}
	exec(head, ns)
	return ns["DisasterRelayDroneEnv"]


def hop_depth(pos, cc, R):
	"""각 드론의 제어 센터까지 홉 수를 반환한다 (1=직접 연결, -1=단절)."""
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


def rollout(env, actor, seed):
	"""한 인스턴스를 굴리고 구·신 환경 공통의 결과 지표를 반환한다."""
	env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=seed)
	obs = env.reset()
	deliv = np.zeros(env.num_drones, dtype=int)
	prev_active = env.dests_active.copy()
	hops, step, done, comm_loss = [], 0, False, 0

	while not done and step < MAX_STEPS:
		with torch.no_grad():
			a, _ = actor(torch.FloatTensor(obs), deterministic=True)
		a = a.numpy()
		acts = np.stack([(a[:, 0] + 1) * np.pi, (a[:, 1] + 1) * 0.5], axis=1)
		obs, _s, _r, done = env.step(acts)

		# 구환경에는 드론별 배송 카운터가 없으므로 목적지 활성 상태 변화로 역산한다
		newly = prev_active & ~env.dests_active
		if newly.any():
			near = np.linalg.norm(
				env.drones_pos[:, None, :] - env.dests_pos[None, newly, :], axis=2
			)
			for c in range(newly.sum()):
				deliv[int(np.argmin(near[:, c]))] += 1
		prev_active = env.dests_active.copy()

		hops.append(hop_depth(env.drones_pos, env.cc_pos, env.comm_range))
		if not env.comm_status.all():
			comm_loss = 1
		step += 1

	H = np.array(hops)
	delivered = int(env.num_dests - env.dests_active.sum())
	return {
		"delivered": delivered,
		"idle_drones": int((deliv == 0).sum()),
		"deliv_std": float(deliv.std()),
		"steps": step,
		"makespan": step if delivered == env.num_dests else None,
		"comm_loss": comm_loss,
		"hop1": float(np.mean(H == 1)),
		"hop2plus": float(np.mean(H >= 2)),
	}


def evaluate(label, ckpt, legacy, n):
	"""체크포인트를 n개 인스턴스에서 평가하고 에피소드별 지표 리스트를 반환한다."""
	Env = load_legacy_env() if legacy else DisasterRelayDroneEnv
	env = Env()
	actor = SACActor(env.reset().shape[-1], 2)
	actor.load_state_dict(torch.load(ckpt, map_location="cpu"))
	actor.eval()
	rows = [rollout(env, actor, seed=i) for i in range(1, n + 1)]
	print(f"  {label}: {n}개 인스턴스 완료", flush=True)
	return rows


def summarize(label, rows):
	"""평가 결과를 한 줄 요약 dict로 집계한다."""
	done = [r["makespan"] for r in rows if r["makespan"] is not None]
	return {
		"run": label,
		"배송": f'{np.mean([r["delivered"] for r in rows]):.1f}/50',
		"완료율": f'{np.mean([r["delivered"] for r in rows]) / 50 * 100:.0f}%',
		"미활동드론": f'{np.mean([r["idle_drones"] for r in rows]):.2f}',
		"드론간편차": f'{np.mean([r["deliv_std"] for r in rows]):.2f}',
		"통신단절": f'{sum(r["comm_loss"] for r in rows)}/{len(rows)}',
		"2홉이상비율": f'{np.mean([r["hop2plus"] for r in rows]) * 100:.0f}%',
		"전량완주": f'{len(done)}/{len(rows)}',
		"makespan": f'{np.mean(done):.0f}' if done else "-",
	}


def latest_ckpt(tag):
	"""해당 실행 태그에서 에피소드 번호가 가장 큰 actor 체크포인트 경로를 반환한다."""
	d = os.path.join(ROOT, "weights", tag)
	fs = [f for f in os.listdir(d) if f.startswith("sac_actor_ep")]
	return os.path.join(d, max(fs, key=lambda f: int(f[12:-4])))


def main():
	"""pre-v1과 지정한 v1 실행들을 동일 인스턴스에서 비교하고 결과를 저장한다."""
	p = argparse.ArgumentParser()
	p.add_argument("--n", type=int, default=20, help="평가 인스턴스 수")
	p.add_argument("--runs", nargs="*", default=["v1_26_08_24_22", "v1_26_08_25_04"])
	p.add_argument("--pre", default="v1_26_06_16_13/sac_actor_ep5000.pth")
	p.add_argument("--out", default="v1_26_08_25_04")
	args = p.parse_args()

	print(f"동일 인스턴스 {args.n}개에서 결정론적 평가", flush=True)
	results = {}
	pre_path = os.path.join(ROOT, "weights", args.pre)
	if os.path.exists(pre_path):
		results["pre-v1 (발표자료 ep5000)"] = evaluate("pre-v1", pre_path, True, args.n)
	for tag in args.runs:
		if os.path.isdir(os.path.join(ROOT, "weights", tag)):
			ck = latest_ckpt(tag)
			results[f"{tag} ({os.path.basename(ck)[:-4]})"] = evaluate(tag, ck, False, args.n)

	summ = [summarize(k, v) for k, v in results.items()]
	cols = list(summ[0].keys())
	w = {c: max(len(c), max(len(str(s[c])) for s in summ)) for c in cols}
	print("\n" + " | ".join(c.ljust(w[c]) for c in cols))
	print("-" * (sum(w.values()) + 3 * len(cols)))
	for s in summ:
		print(" | ".join(str(s[c]).ljust(w[c]) for c in cols))

	os.makedirs(os.path.join(ROOT, "figures"), exist_ok=True)
	out = os.path.join(ROOT, "figures", f"compare_{args.out}.csv")
	with open(out, "w", newline="", encoding="utf-8") as f:
		wr = csv.DictWriter(f, fieldnames=cols)
		wr.writeheader()
		wr.writerows(summ)
	print(f"\n저장: {out}")


if __name__ == "__main__":
	main()
