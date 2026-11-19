"""사이클 34 1차(통신 여유 0·이동 바깥 근사) 해 40건을 고친 재생기(매듭 사이 등속 직선 이동)로 다시 재생해 실패 원인을 가른다.

원인 1 — 옛 재생기의 비동기 도착(짧은 구간 드론이 먼저 멈춤)으로 매듭 사이 거리가 볼록성 논거를 벗어난 것 (재생기 결함).
원인 2 — 다각형 꼭짓점 방향 링크가 정확히 R 에 놓여 환경의 ≤R 판정에 부동소수 오차로 걸린 것 (모형 여유 0 의 결함).
결과는 각 JSON 의 replay_uniform 에 저장한다 (원래 replay 는 그대로 둔다).
"""

import glob
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def main():
	from pipeline.cont_milp_v5_26_11_19_14 import load_plan, replay, rule_makespan
	from util.instance_generator_v5_26_09_15_22 import sample_instance
	rows = []
	for fj in sorted(glob.glob(os.path.join(ROOT, "figures", "cont34_m*s*_k*.json"))):
		d = json.load(open(fj))
		if d.get("obj") is None:
			continue
		N, M, Q, R, kappa = d["drones"], d["dests"], d["capacity"], 300.0, d["kappa"]
		cc, dests = sample_instance(M, seed=d["seed"], num_drones=N, comm_range=R, cc_pos=None)
		_d, P, serv, rel, _k = load_plan(os.path.basename(fj), kappa)
		_ms, v = rule_makespan(cc, dests, N, Q, R, max_steps=10)
		rep = replay(P, cc, dests, N, Q, R, kappa, v, serv, rel)
		# 계획 매듭 자체가 환경 판정(≤R)으로 연결되는지: 꼭짓점 링크(정확히 R) 검사
		from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
		env = DisasterRelayDroneEnv(map_path="", comm_range=R, cluster_penalty=False, max_steps=10, deadlock_limit=10 ** 9, no_progress_limit=10 ** 9, num_drones=N, num_dests=M)
		env.cc_pos, env.dests_pos = cc.copy(), dests.copy()
		env.reset()
		bad_knots = [t for t in range(P.shape[0]) if not env._bfs_connected(P[t]).all()]
		d["replay_uniform"] = dict(rep, knots_over_R=len(bad_knots))
		json.dump(d, open(fj, "w"), ensure_ascii=False, indent=1)
		old = d["replay"]
		rows.append((os.path.basename(fj), old["delivered"], old["max_dev"], rep["delivered"], rep["max_dev"], len(bad_knots), M))
		print(f"{os.path.basename(fj):28s} 옛 재생 {old['delivered']}/{M} 오차 {old['max_dev']:6.1f}m | 등속 재생 {rep['delivered']}/{M} 오차 {rep['max_dev']:6.1f}m | R 초과 매듭 {len(bad_knots)}", flush=True)
	n = len(rows)
	print(f"\n요약 {n}건: 옛 재생 전량 배송 {sum(1 for r in rows if r[1] == r[6])} / 등속 재생 전량 배송 {sum(1 for r in rows if r[3] == r[6])} | "
	      f"옛 오차>5m {sum(1 for r in rows if r[2] > 5)}건 → 등속 오차>5m {sum(1 for r in rows if r[4] > 5)}건 | R 초과 매듭 있는 해 {sum(1 for r in rows if r[5] > 0)}건")


if __name__ == "__main__":
	main()
