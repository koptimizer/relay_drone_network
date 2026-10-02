"""같은 인스턴스에서 규칙+탈출과 학습 정책의 makespan을 쌍으로 비교한다.

완주한 에피소드만 평균하면 어려운 인스턴스를 포기한 쪽이 유리해 보이는 편향이 생긴다.
두 방법이 모두 완주한 부분집합에서 쌍별 차이와 부호 검정을 내어 그 편향을 걷어낸다.
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


def learned(ckpt, dev, ar, ar_near=False):
	"""집합 상위 가중치를 표본 추출 결정 함수로 감싼다 (평가 규약과 동일)."""
	m = SetManagerActor(autoregressive=ar, ar_near=ar_near).to(dev)
	m.load_state_dict(torch.load(os.path.join(ROOT, ckpt), map_location=dev))
	m.eval()

	def fn(e):
		o = {k: torch.as_tensor(v, device=dev).unsqueeze(0) for k, v in e.manager_set_obs().items()}
		with torch.no_grad():
			act, _p, _ = m(o, action_mask(e, dev).unsqueeze(0))
		return act[0].cpu().numpy()
	return fn


def run(env, mf, hl_every, seed, cc_mode):
	"""한 인스턴스를 굴려 makespan(미완주면 None)을 돌려준다."""
	env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=seed, num_drones=env.num_drones,
	                                            comm_range=env.comm_range, cc_pos=cc_mode)
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
	"""두 방법을 같은 시드 목록에서 굴려 쌍별 비교표를 출력하고 CSV로 남긴다."""
	p = argparse.ArgumentParser()
	p.add_argument("--manager", default="weights/v5_26_09_23_18_SWa/best_manager.pth")
	p.add_argument("--autoregressive", action="store_true")
	p.add_argument("--ar-near", action="store_true", help="가까운 드론부터 결정하도록 학습한 가중치")
	p.add_argument("--stall-redecide", type=int, default=0, help="학습 정책에만 적용하는 정체 재결정 스텝 (학습과 같은 값)")
	p.add_argument("--n", type=int, default=60)
	p.add_argument("--reps", type=int, default=2)
	p.add_argument("--seed0", type=int, default=101)
	p.add_argument("--num-drones", type=int, default=4)
	p.add_argument("--num-dests", type=int, default=50)
	p.add_argument("--random-cc", action="store_true")
	p.add_argument("--max-steps", type=int, default=10000)
	p.add_argument("--hl-every", type=int, default=20)
	p.add_argument("--out", default="paired_v5_26_09_27_08")
	args = p.parse_args()

	torch.set_num_threads(2)
	dev = torch.device("cpu")
	env = DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=args.max_steps,
	                            deadlock_limit=10 ** 9, no_progress_limit=10 ** 9,
	                            num_drones=args.num_drones, num_dests=args.num_dests)
	env.reset()
	cc = "random" if args.random_cc else None
	pol = (("규칙+탈출", chain_escape_manager(60, 40, "shuffle")), ("학습", learned(args.manager, dev, args.autoregressive, args.ar_near)))
	rec = []
	for rep in range(args.reps):
		for s in range(args.seed0, args.seed0 + args.n):
			row = {"rep": rep, "seed": s}
			for nm, mf in pol:
				env.stall_redecide = args.stall_redecide if nm == "학습" else 0   # 규칙 베이스라인은 그대로
				row[nm] = run(env, mf, args.hl_every, s, cc)
			rec.append(row)
			print(f"  rep{rep} seed{s}: 규칙 {rec[-1]['규칙+탈출']} 학습 {rec[-1]['학습']}", flush=True)

	a = np.array([r["규칙+탈출"] if r["규칙+탈출"] else np.nan for r in rec], dtype=float)
	b = np.array([r["학습"] if r["학습"] else np.nan for r in rec], dtype=float)
	both = ~np.isnan(a) & ~np.isnan(b)
	print(f"\n롤아웃 {len(rec)}개 (시드 {args.seed0}~{args.seed0 + args.n - 1} x {args.reps}회, "
	      f"드론 {args.num_drones} 목적지 {args.num_dests})")
	print(f"완주: 규칙+탈출 {int(np.sum(~np.isnan(a)))} · 학습 {int(np.sum(~np.isnan(b)))} · 둘 다 {int(both.sum())}")
	print(f"\n[편향된 비교] 각자 완주한 것만 평균")
	print(f"  규칙+탈출 {np.nanmean(a):.0f}   학습 {np.nanmean(b):.0f}   차이 {np.nanmean(b) - np.nanmean(a):+.0f}")
	d = b[both] - a[both]
	print(f"\n[쌍별 비교] 둘 다 완주한 {both.sum()}개")
	print(f"  규칙+탈출 {a[both].mean():.0f}   학습 {b[both].mean():.0f}   쌍별 차이 {d.mean():+.0f} (중앙값 {np.median(d):+.0f})")
	print(f"  학습이 빠른 경우 {int((d < 0).sum())} · 느린 경우 {int((d > 0).sum())} · 동점 {int((d == 0).sum())}")
	w = stats.wilcoxon(d) if both.sum() > 5 and np.any(d != 0) else None
	if w:
		print(f"  윌콕슨 부호순위 p={w.pvalue:.4f}")
	only_b = ~np.isnan(b) & np.isnan(a)
	if only_b.any():
		print(f"\n규칙만 실패한 {int(only_b.sum())}개에서 학습의 makespan: {b[only_b].mean():.0f} "
		      f"(범위 {b[only_b].min():.0f}-{b[only_b].max():.0f})")
	only_a = np.isnan(b) & ~np.isnan(a)
	if only_a.any():
		print(f"학습만 실패한 {int(only_a.sum())}개에서 규칙의 makespan: {a[only_a].mean():.0f}")

	out = os.path.join(ROOT, "figures", f"{args.out}.csv")
	with open(out, "w", newline="", encoding="utf-8") as f:
		wr = csv.DictWriter(f, fieldnames=list(rec[0].keys()))
		wr.writeheader()
		wr.writerows(rec)
	print(f"\n저장: {out}")


if __name__ == "__main__":
	main()
