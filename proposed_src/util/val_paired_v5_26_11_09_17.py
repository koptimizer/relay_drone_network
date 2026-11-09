"""쌍별 검증: 고정 기준 정책(SWa)과 같은 인스턴스에서 비교하는 체크포인트 선택 지표 (사이클 32, 26-11-09).

이전 홀드아웃(4/50 80건, 인스턴스당 표본 1회, 완주율 + 평균 스텝 점수)은 같은 가중치를 다시 재도 2점 넘게
흔들렸고, 쌍별 평가가 p<1e-4 로 잡은 55–78스텝 퇴보를 가리지 못했다 (사이클 31b). 여기서는 최종 판정과 같은
세 구성(3/30, 5/50, 4/50)에서 시드가 겹치지 않는 각 20건을 R 회씩 굴려, 인스턴스별 중앙 makespan 을 기준
정책의 중앙값으로 나눈 비율의 평균을 쓴다 (낮을수록 좋다, 미완주는 상한 3,000). 표본마다 공통 난수 시드를
고정해 체크포인트 사이 비교 잡음을 줄인다. 기준값은 한 번 R_ref 회로 구해 JSON 으로 둔다.

python3 val_paired_v5_26_11_09_17.py --build-ref  : 기준값 생성 (CPU 병렬)
python3 val_paired_v5_26_11_09_17.py --weights W  : 가중치 하나를 채점
"""

import argparse
import json
import multiprocessing as mp
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CFG = [(3, 30, 1501, "random"), (5, 50, 1701, "random"), (4, 50, 1101, None)]
N_PER, CAP, HL = 20, 3000, 20
REF_PATH = os.path.join(ROOT, "figures", "val_ref_v5_26_11_09_17.json")


def instances():
	"""검증 인스턴스 목록 (드론, 수요지, 시드, 거점 모드). 최종 평가 시드(501–540, 701–740, 101–160)와 겹치지 않는다."""
	return [(n, m, s0 + k, cc) for n, m, s0, cc in CFG for k in range(N_PER)]


def make_env(n, m):
	from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
	return DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=CAP,
	                             deadlock_limit=10 ** 9, no_progress_limit=10 ** 9, num_drones=n, num_dests=m)


def rollout(env, policy, seed, cc_mode, crn):
	"""인스턴스 하나를 공통 난수 시드 crn 으로 굴려 makespan(미완주면 None)을 돌려준다."""
	import torch
	from pipeline.common_v5_26_09_15_22 import straight_worker
	from util.instance_generator_v5_26_09_15_22 import sample_instance
	env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=seed, num_drones=env.num_drones,
	                                            comm_range=env.comm_range, cc_pos=cc_mode)
	env.reset()
	np.random.seed(crn)
	torch.manual_seed(crn)
	# 환경은 투영의 동률 깨기에 자체 난수 생성기를 쓴다 (전역 시드와 무관). 이것도 같은 시드로 다시 만들어야
	# 같은 가중치가 같은 값을 낸다 (고치기 전: 같은 SWa 가 0.998–1.043 으로 흔들림, 26-11-09)
	env.rng = np.random.default_rng(crn)
	env.set_goals(policy(env))
	done = False
	while not done and env.current_step < CAP:
		_o, _w, _r, done = env.step(straight_worker(env))
		if env.current_step % HL == 0 or env.goal_invalid().any():
			env.set_goals(policy(env))
	return env.makespan


def crn_seed(k, r):
	return 7_000_000 + 1000 * k + r


def evaluate(policy, R, ks=None):
	"""policy(env)->행동 으로 검증 인스턴스를 R 회씩 굴려 인스턴스별 makespan 목록을 돌려준다 (직렬)."""
	insts = instances()
	ks = range(len(insts)) if ks is None else ks
	out = {}
	envs = {}
	for k in ks:
		n, m, seed, cc = insts[k]
		if (n, m) not in envs:
			envs[(n, m)] = make_env(n, m)
		out[k] = [rollout(envs[(n, m)], policy, seed, cc, crn_seed(k, r)) for r in range(R)]
	return out


def summarize(runs, ref):
	"""인스턴스별 중앙 makespan / 기준 중앙값 비율의 평균과 구성별 값, 완주율."""
	med = {k: float(np.median([x if x is not None else CAP for x in v])) for k, v in runs.items()}
	full = float(np.mean([x is not None for v in runs.values() for x in v]))
	ratio = {k: med[k] / ref[str(k)] for k in med}
	insts = instances()
	by = {}
	for k, r in ratio.items():
		n, m = insts[k][0], insts[k][1]
		by.setdefault(f"{n}/{m}", []).append(r)
	return dict(ratio=float(np.mean(list(ratio.values()))), full=full, mean_med=float(np.mean(list(med.values()))),
	            **{f"r{c.replace('/', '_')}": float(np.mean(v)) for c, v in by.items()})


def load_ref(path=REF_PATH):
	with open(path) as f:
		return json.load(f)["ref_median"]


def _job(args):
	"""병렬 작업자: 가중치 파일의 정책으로 인스턴스 k 를 R 회 굴린다."""
	weights, k, R = args
	import torch
	torch.set_num_threads(1)
	from util.paired_v5_26_09_27_08 import learned
	lf = learned(weights, torch.device("cpu"), True)
	return k, evaluate(lf, R, ks=[k])[k]


def evaluate_parallel(weights, R, workers=8):
	insts = instances()
	with mp.get_context("fork").Pool(workers) as pool:
		res = pool.map(_job, [(weights, k, R) for k in range(len(insts))])
	return dict(res)


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--build-ref", action="store_true")
	p.add_argument("--weights", default="weights/v5_26_09_23_18_SWa/best_manager.pth")
	p.add_argument("--R", type=int, default=8)
	p.add_argument("--workers", type=int, default=12)
	A = p.parse_args()
	os.chdir(ROOT)
	runs = evaluate_parallel(A.weights, A.R, A.workers)
	if A.build_ref:
		ref = {str(k): float(np.median([x if x is not None else CAP for x in v])) for k, v in runs.items()}
		with open(REF_PATH, "w") as f:
			json.dump(dict(weights=A.weights, R=A.R, instances=instances(), ref_median=ref,
			               full=float(np.mean([x is not None for v in runs.values() for x in v]))), f, indent=1)
		print(f"기준값 저장: {REF_PATH} | 완주 {np.mean([x is not None for v in runs.values() for x in v]):.3f}")
		for c in ("3/30", "5/50", "4/50"):
			ks = [k for k, i in enumerate(instances()) if f"{i[0]}/{i[1]}" == c]
			print(f"  {c}: 기준 중앙 makespan 평균 {np.mean([ref[str(k)] for k in ks]):.0f}")
	else:
		s = summarize(runs, load_ref())
		print(json.dumps(s, ensure_ascii=False))


if __name__ == "__main__":
	main()
