"""critic 점수 결정: 학습된 twin-Q critic 으로 드론별 옵션을 점수화해 가장 높은 옵션을 고른다 (롤아웃 없는 선견).

학습 체크포인트(latest.pth)의 critic 두 벌의 최솟값을 쓰고, 행동 마스크로 불가능한 옵션을 가린다.
결정당 비용은 학습 정책과 같은 ms 수준이다. 격자 결정 공간(정확해 55건)과 연속 공간(3/30·4/50·5/50)에서 잰다.
"""

import argparse
import csv
import glob
import json
import os
import sys
import time

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
from model.hier_net_v5_26_09_15_23 import SetManagerTwinQ
from pipeline.common_v5_26_09_15_22 import action_mask, chain_escape_manager, escape_wrap, straight_worker
from pipeline.milp_v5_26_09_21_03 import build_nodes
from util.instance_generator_v5_26_09_15_22 import sample_instance
from util.lattice_eval_v5_26_10_20_17 import hop_table, run as lat_run
from util.paired_v5_26_09_27_08 import learned

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def critic_fn(ckpt, dev, k=7):
	"""critic 으로 드론별 argmax 옵션을 고르는 결정 함수."""
	ck = torch.load(os.path.join(ROOT, ckpt), map_location=dev, weights_only=False)
	q = SetManagerTwinQ(k).to(dev)
	q.load_state_dict(ck["critic"])
	q.eval()

	def fn(e):
		o = {kk: torch.as_tensor(v, device=dev).unsqueeze(0) for kk, v in e.manager_set_obs().items()}
		m = action_mask(e, dev).unsqueeze(0)
		with torch.no_grad():
			q1, q2 = q(o)
			v = torch.minimum(q1, q2)[0]
		v = torch.where(m[0], v, torch.full_like(v, -1e9))
		return v.argmax(1).cpu().numpy()
	return fn


def critic_cand_fn(ckpt, dev, k=7, k_cand=3):
	"""선견과 같은 후보 집합(규칙 배정 + 드론 하나를 가까운 후보로 덮어쓰기)을 critic 평균 Q 로 점수화해 고른다."""
	ck = torch.load(os.path.join(ROOT, ckpt), map_location=dev, weights_only=False)
	q = SetManagerTwinQ(k).to(dev)
	q.load_state_dict(ck["critic"])
	q.eval()
	base = chain_escape_manager()

	def fn(e):
		N = e.num_drones
		acts0 = np.asarray(base(e)).astype(np.int64)
		o = {kk: torch.as_tensor(v, device=dev).unsqueeze(0) for kk, v in e.manager_set_obs().items()}
		m = action_mask(e, dev).unsqueeze(0)
		with torch.no_grad():
			q1, q2 = q(o)
			v = torch.minimum(q1, q2)[0]
		v = torch.where(m[0], v, torch.full_like(v, -1e9)).cpu().numpy()
		opts = [acts0]
		for i in range(N):
			if e.drones_capacity[i] == 0:
				continue
			for c in range(min(k_cand, e.n_cand)):
				if e.candidates(i)[c] >= 0 and c != acts0[i]:
					a = acts0.copy(); a[i] = c; opts.append(a)
		sc = [float(np.mean([v[i, a[i]] for i in range(N)])) for a in opts]
		return opts[int(np.argmax(sc))]
	return fn


def run_cont(env, mf, seed, cc_mode, hl_every=20):
	env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=seed, num_drones=env.num_drones,
	                                            comm_range=env.comm_range, cc_pos=cc_mode)
	env.reset()
	env.set_goals(mf(env))
	done = False
	while not done and env.current_step < env.max_steps:
		_o, _w, _r, done = env.step(straight_worker(env))
		if env.current_step % hl_every == 0 or env.goal_invalid().any():
			env.set_goals(mf(env))
	return env.makespan


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--ckpt", default="weights/v5_26_09_23_18_SWa/latest.pth")
	p.add_argument("--mode", choices=["lattice", "cont"], default="lattice")
	p.add_argument("--pattern", default="bfs2_n3m*_h98_26_10_21_03.json")
	p.add_argument("--num-drones", type=int, default=3)
	p.add_argument("--num-dests", type=int, default=30)
	p.add_argument("--seed0", type=int, default=501)
	p.add_argument("--n", type=int, default=40)
	p.add_argument("--random-cc", action="store_true")
	p.add_argument("--out", default="critic_pick_v5_26_10_27_00")
	p.add_argument("--cand", action="store_true", help="critic 으로 선견 후보 집합(규칙 + 단일 덮어쓰기)만 점수화한다")
	A_ = p.parse_args()
	torch.set_num_threads(2)
	dev = torch.device("cpu")
	cf = critic_cand_fn(A_.ckpt, dev) if A_.cand else critic_fn(A_.ckpt, dev)
	cfe = escape_wrap(cf)
	rows = []
	if A_.mode == "lattice":
		for f in sorted(glob.glob(os.path.join(ROOT, "figures", A_.pattern))):
			d = json.load(open(f))
			N, M, seed, kappa = int(d["drones"]), int(d["dests"]), int(d["seed"]), int(d["kappa"])
			env = DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=10000,
			                            deadlock_limit=10 ** 9, no_progress_limit=10 ** 9, num_drones=N, num_dests=M)
			if "capacity" in d:
				env.max_capacity = int(d["capacity"])
			env.reset()
			cc, dests = sample_instance(M, seed=seed, num_drones=N, comm_range=300.0, cc_pos=None)
			ell, sp = env.max_speed * kappa, float(d["spacing"])
			pos, _nd = build_nodes(cc, dests, ell, sp, 300.0, N, hull_margin=sp / np.sqrt(2.0))
			Dm = np.linalg.norm(pos[:, None, :] - pos[None, :, :], axis=2)
			A = [np.flatnonzero(Dm[i] <= ell + 1e-6) for i in range(len(pos))]
			hop = hop_table(A, len(pos))
			t0 = time.time()
			c = lat_run(env, cf, seed, pos, A, kappa, hop)
			ce = lat_run(env, cfe, seed, pos, A, kappa, hop)
			rows.append(dict(drones=N, dests=M, seed=seed, opt=d["milp_steps"], critic=c, critic_esc=ce, sec=round(time.time() - t0, 1)))
			print(f"M={M} s{seed}: 최적 {d['milp_steps']:.0f} | critic {c} | critic+복구 {ce}", flush=True)
	else:
		cc_mode = "random" if A_.random_cc else None
		for seed in range(A_.seed0, A_.seed0 + A_.n):
			env = DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=10000,
			                            deadlock_limit=10 ** 9, no_progress_limit=10 ** 9,
			                            num_drones=A_.num_drones, num_dests=A_.num_dests)
			env.reset()
			c = run_cont(env, cf, seed, cc_mode); ce = run_cont(env, cfe, seed, cc_mode)
			rows.append(dict(drones=A_.num_drones, dests=A_.num_dests, seed=seed, critic=c, critic_esc=ce))
			print(f"{A_.num_drones}/{A_.num_dests} s{seed}: critic {c} | critic+복구 {ce}", flush=True)
	out = os.path.join(ROOT, "figures", f"{A_.out}.csv")
	with open(out, "w", newline="", encoding="utf-8") as fh:
		w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
	print("저장:", out, flush=True)


if __name__ == "__main__":
	main()
