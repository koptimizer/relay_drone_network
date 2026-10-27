"""선견 라벨 검증: 선견이 고른 배정이 '뒤를 학습 정책이 이을 때'에도 규칙 배정보다 나은가 (사이클 31 진단).

학습 정책(SWa)으로 3/30 인스턴스를 굴리며 결정 시점의 상태를 떠 두고, 그 상태에서
  - 규칙 라벨 (사슬 규칙의 배정)과 선견 라벨 (규칙 롤아웃으로 고른 배정, 학습 스크립트의 교사와 같은 후보 집합)을 만든다.
  - 두 라벨이 다른 상태만 남겨, 각 라벨을 건 뒤 (a) 규칙(+복구)으로 끝까지, (b) 학습 정책으로 끝까지 굴린 makespan 을 잰다.
선견 라벨은 (a) 에서는 정의상 이득이다. (b) 에서도 이득이면 라벨은 학습 정책에게도 좋은 표적이고, 아니면
'뒤를 규칙이 잇는다'는 가정에 묶인 표적이라는 가설을 지지한다.
"""

import argparse
import copy
import csv
import multiprocessing as mp
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
HL, HOR, LA_K = 20, 3000, 3


def cont(env, mf, horizon):
	"""현재 상태에서 결정 함수 mf 로 끝까지 (또는 horizon 스텝까지) 굴려 값(미완주면 상한 + 50 x 남은 수)을 돌려준다."""
	from pipeline.common_v5_26_09_15_22 import straight_worker
	t0, done = env.current_step, False
	while not done and env.current_step < min(env.max_steps, t0 + horizon):
		_o, _w, _r, done = env.step(straight_worker(env))
		if env.current_step % HL == 0 or env.goal_invalid().any():
			env.set_goals(mf(env))
	if env.makespan is not None:
		return float(env.makespan)
	return float(env.current_step) + 50.0 * float(env.dests_active.sum())


def labels(env):
	"""(규칙 배정, 선견 배정) 을 (acts, dests) 쌍으로 돌려준다. 학습 스크립트의 act_lookahead 와 같은 후보 집합."""
	from pipeline.common_v5_26_09_15_22 import chain_escape_manager, chain_manager
	N = env.num_drones
	acts0 = chain_manager(env).astype(np.int64)
	d0 = np.array([int(env.candidates(i)[a]) if 0 <= a < env.n_cand else -1 for i, a in enumerate(acts0)])
	opts = [(acts0, None)]
	for i in range(N):
		if env.drones_capacity[i] == 0:
			continue
		for j in [int(x) for x in env.candidates(i)[:LA_K] if x >= 0]:
			if j == d0[i] or j in d0:
				continue
			acts = acts0.copy(); dests = -np.ones(N, dtype=np.int64); acts[i], dests[i] = 0, j
			opts.append((acts, dests))
	vals = []
	for a, d in opts:
		e = copy.deepcopy(env); e.set_goals(a, d)
		vals.append(cont(e, chain_escape_manager(), 1000))
	return opts[0], opts[int(np.argmin(vals))], int(np.argmin(vals))


def work(args):
	"""인스턴스 하나: 학습 정책 궤적에서 상태를 떠 라벨 쌍을 평가한다."""
	seed, every, reps, manager = args
	import torch
	torch.set_num_threads(1)
	from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
	from pipeline.common_v5_26_09_15_22 import chain_escape_manager, straight_worker
	from util.instance_generator_v5_26_09_15_22 import sample_instance
	from util.paired_v5_26_09_27_08 import learned
	lf = learned(manager, torch.device("cpu"), True)
	rng = np.random.default_rng(seed)
	np.random.seed(seed)
	env = DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=10000,
	                            deadlock_limit=10 ** 9, no_progress_limit=10 ** 9, num_drones=3, num_dests=30)
	env.cc_pos, env.dests_pos = sample_instance(30, seed=seed, num_drones=3, comm_range=300.0, cc_pos="random")
	env.reset()
	env.set_goals(lf(env))
	rows, k, done = [], 0, False
	while not done and env.current_step < 3000:
		_o, _w, _r, done = env.step(straight_worker(env))
		if env.current_step % HL == 0 or env.goal_invalid().any():
			k += 1
			if k % every == 0 and env.dests_active.sum() > 2:
				(ar, dr), (al, dl), idx = labels(env)
				if idx > 0:
					res = {}
					for nm, (a, d) in (("rule", (ar, dr)), ("la", (al, dl))):
						for cn, mf_f in (("rule", chain_escape_manager), ("pol", lambda: lf)):
							v = []
							for _ in range(reps):
								e = copy.deepcopy(env); e.set_goals(a, d)
								v.append(cont(e, mf_f(), HOR))
							res[f"{nm}_{cn}"] = float(np.mean(v))
					rows.append(dict(seed=seed, step=env.current_step, served=int(30 - env.dests_active.sum()), **res))
			env.set_goals(lf(env))
	return rows


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--seed0", type=int, default=501)
	p.add_argument("--n", type=int, default=20)
	p.add_argument("--every", type=int, default=3, help="학습 정책 궤적의 몇 번째 결정 시점마다 상태를 뜰지")
	p.add_argument("--reps", type=int, default=4)
	p.add_argument("--workers", type=int, default=6)
	p.add_argument("--manager", default="weights/v5_26_09_23_18_SWa/best_manager.pth")
	p.add_argument("--out", default="la_label_probe_v5_26_10_27_13")
	A = p.parse_args()
	with mp.get_context("fork").Pool(A.workers) as pool:
		parts = pool.map(work, [(s, A.every, A.reps, A.manager) for s in range(A.seed0, A.seed0 + A.n)])
	rows = [r for part in parts for r in part]
	out = os.path.join(ROOT, "figures", f"{A.out}.csv")
	with open(out, "w", newline="", encoding="utf-8") as fh:
		w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
	from scipy import stats
	R = {k: np.array([r[k] for r in rows]) for k in ("rule_rule", "la_rule", "rule_pol", "la_pol")}
	d_rule = R["la_rule"] - R["rule_rule"]; d_pol = R["la_pol"] - R["rule_pol"]
	print(f"상태 {len(rows)} (라벨이 다른 상태만, 인스턴스 {A.n})")
	print(f"뒤를 규칙이 이을 때: 선견 라벨 - 규칙 라벨 = 평균 {d_rule.mean():+.1f}, 중앙 {np.median(d_rule):+.1f}, 선견이 나은 비율 {np.mean(d_rule < 0):.2f}, p={stats.wilcoxon(d_rule).pvalue:.2g}")
	print(f"뒤를 학습 정책이 이을 때: 선견 라벨 - 규칙 라벨 = 평균 {d_pol.mean():+.1f}, 중앙 {np.median(d_pol):+.1f}, 선견이 나은 비율 {np.mean(d_pol < 0):.2f}, p={stats.wilcoxon(d_pol).pvalue:.2g}")
	print("저장:", out)


if __name__ == "__main__":
	main()
