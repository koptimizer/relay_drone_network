"""정체가 길어질 때만 상위 행동을 높은 온도로 다시 뽑아 드문 장기 정체를 없애는지 본다.

같은 학습 정책의 분포를 그대로 쓰고 추론 절차만 바꾸므로 규칙을 섞지 않는다(학습 단독 유지).
설정마다 롤아웃별 makespan을 모아 중앙값과 꼬리를 함께 비교한다.
"""

import argparse
import csv
import os
import sys

import numpy as np
import torch
from scipy import stats

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
from model.hier_net_v5_26_09_15_23 import SetManagerActor
from pipeline.common_v5_26_09_15_22 import action_mask, chain_escape_manager, straight_worker
from util.instance_generator_v5_26_09_15_22 import sample_instance

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def learned(ckpt, dev, ar, k_stall=0, temp=1.0):
	"""집합 상위를 결정 함수로 감싼다. 정체가 k_stall 이상이면 온도 temp로 다시 뽑는다."""
	m = SetManagerActor(autoregressive=ar).to(dev)
	m.load_state_dict(torch.load(os.path.join(ROOT, ckpt), map_location=dev))
	m.eval()
	hit = {"n": 0}

	def fn(e):
		o = {k: torch.as_tensor(v, device=dev).unsqueeze(0) for k, v in e.manager_set_obs().items()}
		with torch.no_grad():
			act, probs, _ = m(o, action_mask(e, dev).unsqueeze(0))
		if k_stall and e.deadlock_run >= k_stall and temp != 1.0:
			p = probs[0].clamp_min(0.0) ** (1.0 / temp)
			p = p / p.sum(-1, keepdim=True).clamp_min(1e-12)
			act = torch.distributions.Categorical(probs=p).sample().unsqueeze(0)
			hit["n"] += 1
		return act[0].cpu().numpy()
	return fn, hit


def run(env, mf, hl_every, seed, cc):
	"""한 인스턴스를 굴려 makespan(미완주면 None)을 돌려준다."""
	env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=seed, num_drones=env.num_drones,
	                                            comm_range=env.comm_range, cc_pos=cc)
	env.reset()
	env.set_goals(mf(env))
	done, t = False, 0
	while not done and t < env.max_steps:
		_o, _wr, _tr, done = env.step(straight_worker(env))
		t += 1
		if t % hl_every == 0 or env.goal_invalid().any():
			env.set_goals(mf(env))
	s = env.episode_stats()
	return s["makespan"] if s["makespan"] else None


def main():
	"""여러 (정체 문턱, 온도) 설정을 같은 롤아웃에서 비교한다."""
	p = argparse.ArgumentParser()
	p.add_argument("--manager", default="weights/v5_26_09_23_18_SWa/best_manager.pth")
	p.add_argument("--autoregressive", action="store_true")
	p.add_argument("--n", type=int, default=60)
	p.add_argument("--reps", type=int, default=2)
	p.add_argument("--seed0", type=int, default=101)
	p.add_argument("--num-drones", type=int, default=4)
	p.add_argument("--num-dests", type=int, default=50)
	p.add_argument("--random-cc", action="store_true")
	p.add_argument("--settings", nargs="*", default=["0:1", "60:2", "60:5", "120:2", "120:5"],
	               help="정체문턱:온도 목록. 0:1은 개입 없음")
	p.add_argument("--rule", action="store_true", help="규칙+탈출도 함께 측정")
	p.add_argument("--out", default="stall_temp_v5_26_09_27_09")
	args = p.parse_args()

	torch.set_num_threads(2)
	dev = torch.device("cpu")
	env = DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=10000,
	                            deadlock_limit=10 ** 9, no_progress_limit=10 ** 9,
	                            num_drones=args.num_drones, num_dests=args.num_dests)
	env.reset()
	cc = "random" if args.random_cc else None
	seeds = [(r, s) for r in range(args.reps) for s in range(args.seed0, args.seed0 + args.n)]
	res, hits = {}, {}
	pol = []
	for sp in args.settings:
		k, t = sp.split(":")
		pol.append((f"정체>{k} 온도{t}" if int(k) else "개입 없음", int(k), float(t)))
	for name, k, t in pol:
		mf, hit = learned(args.manager, dev, args.autoregressive, k, t)
		res[name] = [run(env, mf, 20, s, cc) for _r, s in seeds]
		hits[name] = hit["n"]
		print(f"  {name}: 완주 {sum(1 for x in res[name] if x)}/{len(seeds)} 개입 {hit['n']}회", flush=True)
	if args.rule:
		res["규칙+탈출"] = [run(env, chain_escape_manager(60, 40, "shuffle"), 20, s, cc) for _r, s in seeds]
		print(f"  규칙+탈출: 완주 {sum(1 for x in res['규칙+탈출'] if x)}/{len(seeds)}", flush=True)

	base = np.array([x if x else np.nan for x in res[pol[0][0]]], dtype=float)
	print(f"\n{'설정':<18}{'완주':>8}{'중앙값':>9}{'절단평균':>10}{'평균':>8}{'최대':>8}{'2000초과':>10}{'개입':>7}")
	for name in res:
		v = np.array([x if x else np.nan for x in res[name]], dtype=float)
		ok = ~np.isnan(v)
		print(f"{name:<18}{int(ok.sum()):>4}/{len(v):<3}{np.median(v[ok]):>9.0f}"
		      f"{stats.trim_mean(v[ok], 0.1):>10.0f}{v[ok].mean():>8.0f}{v[ok].max():>8.0f}"
		      f"{int((v[ok] > 2000).sum()):>10}{hits.get(name, 0):>7}")
		m = ok & ~np.isnan(base)
		if name != pol[0][0] and m.sum() > 5:
			d = v[m] - base[m]
			print(f"{'':<18}개입 없음 대비 쌍별 중앙 {np.median(d):+.0f} "
			      f"승 {int((d < 0).sum())} 패 {int((d > 0).sum())} p={stats.wilcoxon(d).pvalue:.4f}")

	out = os.path.join(ROOT, "figures", f"{args.out}.csv")
	with open(out, "w", newline="", encoding="utf-8") as f:
		wr = csv.writer(f)
		wr.writerow(["rep", "seed"] + list(res.keys()))
		for i, (r, s) in enumerate(seeds):
			wr.writerow([r, s] + [res[k][i] for k in res])
	print(f"\n저장: {out}")


if __name__ == "__main__":
	main()
