"""연속 공간에서 규칙 / 따라가기 중계 / 선견 규칙을 쌍별 평가 인스턴스(3/30, 5/50 등)로 잰다.

격자 평가에서 효과를 본 두 변경(따라가기 중계, 한 수 선견)이 제어기가 설계된 연속 공간과 큰 편대에서도
유효한지 본다. 롤아웃은 상한 horizon 스텝까지 굴리고, 미완주면 상한 + 50 x 남은 수요지 수로 값을 매긴다.
"""

import argparse
import copy
import csv
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
from pipeline.common_v5_26_09_15_22 import chain_escape_manager, straight_worker
from util.instance_generator_v5_26_09_15_22 import sample_instance
from util.rule2_v5_26_10_26_20 import follow_relays, resolved, rule_setter

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def continue_cont(env, setter, hl_every, horizon):
	"""현재 상태에서 연속 공간으로 끝까지 (또는 horizon 까지) 굴린다."""
	t0 = env.current_step
	done = False
	while not done and env.current_step < min(env.max_steps, t0 + horizon):
		_o, _w, _r, done = env.step(straight_worker(env))
		if env.current_step % hl_every == 0 or env.goal_invalid().any():
			setter(env)
	if env.makespan is not None:
		return float(env.makespan)
	return float(env.current_step) + 50.0 * float(env.dests_active.sum())


def lookahead_cont(hl_every, horizon, k_cand=5, follow=False, commit=False, log=None, la_every=0):
	"""연속 공간 한 수 선견 설정자 (격자판과 같은 후보 열거: 드론 i 에 후보 j, 중계 선행 토글).

	la_every > 0 이면 선견 평가는 그 주기(또는 목표 무효화 시)에만 하고 사이는 규칙(+따라가기)으로 잇는다.
	따라가기를 켜면 중계 선행 토글은 의미가 없으므로 열거에서 뺀다.
	"""
	base = chain_escape_manager()
	st = {"pin": None, "last": -10 ** 9}
	relay_opts = (False,) if follow else (False, True)

	def decide(env):
		N = env.num_drones
		acts0 = np.asarray(base(env)).copy()
		d0 = resolved(env, acts0)
		opts = [(acts0, None, bool(env.relay_lookahead), None)]
		for i in range(N):
			if env.drones_capacity[i] == 0:
				continue
			for j in [int(x) for x in env.candidates(i)[:k_cand] if x >= 0]:
				if j == d0[i] or j in d0:
					continue
				acts = acts0.copy(); dests = -np.ones(N, dtype=int)
				acts[i], dests[i] = 0, j
				for la in relay_opts:
					opts.append((acts, dests, la, (i, j) if commit else None))
		for la in relay_opts:
			if la != bool(env.relay_lookahead):
				opts.append((acts0, None, la, None))
		best, bv = None, None
		for k, (acts, dests, la, pin) in enumerate(opts):
			e = copy.deepcopy(env)
			e.relay_lookahead = la
			e.set_goals(acts, dests)
			if follow:
				follow_relays(e)
			v = continue_cont(e, rule_setter(chain_escape_manager(), follow, pin), hl_every, horizon)
			if bv is None or v < bv:
				best, bv = k, v
		acts, dests, la, pin = opts[best]
		if log is not None:
			log.append((env.current_step, len(opts), best, bv))
		env.relay_lookahead = la
		env.set_goals(acts, dests)
		if follow:
			follow_relays(env)
		st["pin"] = pin

	def setter(env):
		pin = st["pin"]
		if env.current_step <= 1:
			st["last"] = -10 ** 9
		if commit and pin is not None and env.dests_active[pin[1]] and env.drones_capacity[pin[0]] > 0 \
		   and env.goal_kind[pin[0]] == 0 and env.goal_dest[pin[0]] == pin[1]:
			rule_setter(base, follow, pin)(env)
			return
		if la_every > 0 and env.current_step - st["last"] < la_every and not env.goal_invalid().any():
			rule_setter(base, follow, None)(env)
			return
		st["last"] = env.current_step
		decide(env)
	return setter


def run(env, setter, seed, cc_mode, hl_every):
	"""한 인스턴스를 연속 공간으로 굴려 makespan(미완주면 None)을 돌려준다."""
	env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=seed, num_drones=env.num_drones,
	                                            comm_range=env.comm_range, cc_pos=cc_mode)
	env.reset()
	env.relay_lookahead = False
	setter(env)
	done = False
	while not done and env.current_step < env.max_steps:
		_o, _w, _r, done = env.step(straight_worker(env))
		if env.current_step % hl_every == 0 or env.goal_invalid().any():
			setter(env)
	return env.makespan


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--num-drones", type=int, default=3)
	p.add_argument("--num-dests", type=int, default=30)
	p.add_argument("--seed0", type=int, default=501)
	p.add_argument("--n", type=int, default=40)
	p.add_argument("--random-cc", action="store_true")
	p.add_argument("--max-steps", type=int, default=10000)
	p.add_argument("--horizon", type=int, default=3000)
	p.add_argument("--hl-every", type=int, default=20)
	p.add_argument("--methods", default="rule,follow,la_follow")
	p.add_argument("--la-every", type=int, default=100, help="선견 평가 주기 (스텝). 0이면 모든 결정 시점")
	p.add_argument("--k-cand", type=int, default=3)
	p.add_argument("--out", default="rule2_cont_v5_26_10_26_20")
	A_ = p.parse_args()
	cc = "random" if A_.random_cc else None
	rows = []
	for seed in range(A_.seed0, A_.seed0 + A_.n):
		env = DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=A_.max_steps,
		                            deadlock_limit=10 ** 9, no_progress_limit=10 ** 9,
		                            num_drones=A_.num_drones, num_dests=A_.num_dests)
		env.reset()
		row = dict(drones=A_.num_drones, dests=A_.num_dests, seed=seed)
		for m in A_.methods.split(","):
			t0 = time.time()
			if m == "rule":
				s = rule_setter(chain_escape_manager(), False)
			elif m == "follow":
				s = rule_setter(chain_escape_manager(), True)
			else:
				s = lookahead_cont(A_.hl_every, A_.horizon, k_cand=A_.k_cand, follow="follow" in m,
				                   commit="commit" in m, la_every=A_.la_every)
			row[m] = run(env, s, seed, cc, A_.hl_every)
			row[f"{m}_sec"] = round(time.time() - t0, 1)
		rows.append(row)
		print(f"{A_.num_drones}/{A_.num_dests} s{seed}: " + " | ".join(f"{m} {row[m]} ({row[f'{m}_sec']}s)" for m in A_.methods.split(",")), flush=True)
	out = os.path.join(ROOT, "figures", f"{A_.out}.csv")
	with open(out, "w", newline="", encoding="utf-8") as fh:
		w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
	print("저장:", out, flush=True)


if __name__ == "__main__":
	main()
