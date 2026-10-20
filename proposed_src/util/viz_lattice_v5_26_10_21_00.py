"""M=6 인스턴스에서 네 방법(최적 계획·규칙·탐색 휴리스틱·학습)의 궤적을 같은 시계로 GIF 애니메이션한다.

네 방법 모두 모형과 같은 결정 공간(기간 경계 노드 위치·A(i) 이동·기간 서비스)에서 돌며, 각 패널에
현재 스텝, 완료된 makespan, 누적 이동거리, 배송 수를 표기한다. 평가가 동등한지 눈으로 검증하는 용도다.
"""

import argparse
import csv
import glob
import json
import os
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle
import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
from pipeline.common_v5_26_09_15_22 import chain_escape_manager, escape_wrap
from pipeline.milp_v5_26_09_21_03 import build_nodes
from util.instance_generator_v5_26_09_15_22 import sample_instance
from util.lattice_eval_v5_26_10_20_17 import hop_table, lattice_step, serve_at_boundary
from util.metaheur_v5_26_10_20_17 import search, seq_actions
from util.paired_v5_26_09_27_08 import learned

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FIG = os.path.join(ROOT, "figures")
plt.rcParams.update({"font.size": 8, "font.family": "DejaVu Sans"})
C = ["#1d6d9c", "#b04236", "#2d7a5c", "#8a5fb0"]


def audit(trace, pos, A, cc, R, kappa):
	"""기록한 기간 경계 궤적이 수리모형의 실행 가능해인지 검사한다 (동등성 검증).

	(1) 모든 경계 위치가 격자 노드인가, (2) 기간 이동이 A(i) 안인가, (3) 경계에서 연결인가,
	(4) 서비스가 한 기간을 통째로 쓰는가. 네 방법 모두 통과해야 최적값이 공통 하한이 된다.
	"""
	from util.lattice_eval_v5_26_10_20_17 import persist_connected
	bad = {"node": 0, "move": 0, "conn": 0}
	prev = None
	for nodes in trace:
		if nodes is None:
			continue
		if prev is not None:
			for i, (u, v) in enumerate(zip(prev, nodes)):
				if v not in A[u]:
					bad["move"] += 1
			if not persist_connected(prev, nodes, pos, cc, R):
				bad["conn"] += 1
		P = pos[nodes]
		Q = np.vstack([cc[None, :], P])
		Dm = np.linalg.norm(Q[:, None, :] - Q[None, :, :], axis=2) <= R + 1e-6
		seen, stack = {0}, [0]
		while stack:
			u = stack.pop()
			for v in np.flatnonzero(Dm[u]):
				if v not in seen:
					seen.add(int(v)); stack.append(int(v))
		if len(seen) != len(Q):
			bad["conn"] += 1
		prev = nodes
	return bad


def record(env, setter, seed, pos, A, hop, kappa, plan=None, max_steps=4000):
	"""한 방법을 격자 모드로 굴리며 스텝별 (위치, 미배송 마스크)를 기록한다."""
	env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=seed, num_drones=env.num_drones,
	                                            comm_range=env.comm_range, cc_pos=None)
	env.reset()
	env.no_project = env.no_service = True
	env.relay_nodes = pos
	node = (plan[0].copy() if plan is not None else np.zeros(env.num_drones, dtype=int))
	env.drones_pos[:] = pos[node]
	if setter is not None:
		setter(env)
	frames = [(env.drones_pos.copy(), env.dests_active.copy())]
	trace = [node.copy()]
	dist = np.zeros(env.num_drones)
	t, p = 0, 0
	while env.makespan is None and t < max_steps:
		if plan is not None:
			serve_at_boundary(env, node, node, kappa)
			if env.makespan is not None:
				frames.append((env.drones_pos.copy(), env.dests_active.copy()))
				break
			p += 1
			if p >= plan.shape[0]:
				break
			tgt = np.where(env.drones_timer > 0, node, plan[p])
		else:
			serve_at_boundary(env, node, node, kappa)
			if env.makespan is not None:
				frames.append((env.drones_pos.copy(), env.dests_active.copy()))
				break
			tgt = lattice_step(env, node, pos, A, hop, env.cc_pos, env.comm_range)
		dist += np.linalg.norm(pos[tgt] - pos[node], axis=1)
		stp = (pos[tgt] - pos[node]) / (kappa * env.max_speed)
		for _ in range(kappa):
			env.step(stp)
			t += 1
			if setter is not None and (t % 20 == 0 or env.goal_invalid().any()):
				setter(env)
			frames.append((env.drones_pos.copy(), env.dests_active.copy()))
		node = tgt
		trace.append(node.copy())
		env.drones_pos[:] = pos[node]
		frames[-1] = (env.drones_pos.copy(), env.dests_active.copy())
	return frames, env.makespan, float(dist.sum()), int(env.num_dests - env.dests_active.sum()), trace


def main():
	p_ = argparse.ArgumentParser()
	p_.add_argument("--dests", type=int, default=6)
	p_.add_argument("--seeds", type=int, nargs="*", default=list(range(901, 911)))
	p_.add_argument("--manager", default="weights/v5_26_09_23_18_SWa/best_manager.pth")
	p_.add_argument("--stride", type=int, default=2, help="몇 스텝마다 한 프레임")
	p_.add_argument("--out", default="viz_m6_26_10_21_00")
	A_ = p_.parse_args()
	torch.set_num_threads(2)
	lf = learned(A_.manager, torch.device("cpu"), True)
	meta = {}
	for f in sorted(glob.glob(os.path.join(ROOT, "runs", "meta", "m*.log"))):
		for ln in open(f):
			m = re.match(r"M=\s*(\d+) s(\d+): 최적 (\d+) \| 탐색 (\d+)", ln)
			if m:
				meta[(int(m.group(1)), int(m.group(2)))] = int(m.group(4))
	os.makedirs(os.path.join(FIG, "viz"), exist_ok=True)
	rows = []
	for seed in A_.seeds:
		jf = os.path.join(FIG, f"bfs2_n3m{A_.dests}s{seed}_h98_26_10_21_03.json")
		d = json.load(open(jf))
		N, M, kappa = int(d["drones"]), int(d["dests"]), int(d["kappa"])
		plan = np.load(jf.replace(".json", "_plan.npy"), allow_pickle=True).item()["plan"]
		env = DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=4000,
		                            deadlock_limit=10 ** 9, no_progress_limit=10 ** 9, num_drones=N, num_dests=M)
		env.reset()
		cc, dests = sample_instance(M, seed=seed, num_drones=N, comm_range=300.0, cc_pos=None)
		ell, sp = env.max_speed * kappa, float(d["spacing"])
		pos, _nd = build_nodes(cc, dests, ell, sp, 300.0, N, hull_margin=sp / np.sqrt(2.0))
		D = np.linalg.norm(pos[:, None, :] - pos[None, :, :], axis=2)
		A = [np.flatnonzero(D[i] <= ell + 1e-6) for i in range(len(pos))]
		hop = hop_table(A, len(pos))
		runs = []
		runs.append(("optimal plan (MILP/BFS)",) + record(env, None, seed, pos, A, hop, kappa, plan=plan))
		runs.append(("rule",) + record(env, (lambda e, mf=chain_escape_manager(): e.set_goals(mf(e))), seed, pos, A, hop, kappa))
		order, bv, _n = search(env, seed, (pos, A, hop), kappa, 600, np.random.default_rng(7), algo="ils")
		env.max_steps = 4000
		st = {"esc": 0, "n": 0}
		runs.append(("search heuristic (ILS)",) + record(env, (lambda e: e.set_goals(*seq_actions(e, order, st))), seed, pos, A, hop, kappa))
		runs.append(("learned policy",) + record(env, (lambda e: e.set_goals(lf(e))), seed, pos, A, hop, kappa))
		nf = max(len(r[1]) for r in runs)
		idx = list(range(0, nf, A_.stride)) + [nf - 1]
		fig, axes = plt.subplots(1, 4, figsize=(12.0, 3.5))
		lim = [min(pos[:, 0].min(), cc[0]) - 80, max(pos[:, 0].max(), cc[0]) + 80,
		       min(pos[:, 1].min(), cc[1]) - 80, max(pos[:, 1].max(), cc[1]) + 80]
		arts = []
		for k, (ax, (nm, fr, ms, dist, dl, _tr)) in enumerate(zip(axes, runs)):
			ax.set_aspect("equal"); ax.set_xlim(lim[0], lim[1]); ax.set_ylim(lim[2], lim[3])
			ax.set_xticks([]); ax.set_yticks([])
			ax.scatter(pos[1 + M:, 0], pos[1 + M:, 1], s=3, color="#dddddd", zorder=0)
			ax.scatter([cc[0]], [cc[1]], s=90, marker="s", color="#222", zorder=5)
			sd = ax.scatter(dests[:, 0], dests[:, 1], s=46, marker="x", lw=1.6, color="#222", zorder=4)
			dr = ax.scatter(np.zeros(N), np.zeros(N), s=90, zorder=6, edgecolor="k", lw=0.6, c=C[:N])
			ln = [ax.plot([], [], color="#1d6d9c", lw=1.1, zorder=3)[0] for _ in range(N + 1)]
			disc = [Circle((0, 0), 300.0, fill=True, alpha=0.05, color="#1d6d9c", lw=0) for _ in range(N)]
			for c in disc:
				ax.add_patch(c)
			ttl = ax.set_title("", fontsize=8.5, loc="left")
			arts.append((sd, dr, ln, disc, ttl, fr, ms, dist, dl, nm))
		fig.suptitle(f"$M$ = {M}, 3 drones, instance seed {seed} — all methods in the model's decision space", fontsize=9.5)

		def upd(fi):
			out = []
			for (sd, dr, ln, disc, ttl, fr, ms, dist, dl, nm) in arts:
				j = min(fi, len(fr) - 1)
				P, act = fr[j]
				dr.set_offsets(P)
				sd.set_color(np.where(act, "#222222", "#cccccc"))
				nodes = np.vstack([cc[None, :], P])
				e = 0
				for a in range(len(nodes)):
					for b in range(a + 1, len(nodes)):
						if np.linalg.norm(nodes[a] - nodes[b]) <= 300.0 and e < len(ln):
							ln[e].set_data([nodes[a, 0], nodes[b, 0]], [nodes[a, 1], nodes[b, 1]]); e += 1
				for q in range(e, len(ln)):
					ln[q].set_data([], [])
				for c, q in zip(disc, P):
					c.center = tuple(q)
				done = "" if ms is None else f"  makespan {ms}"
				ttl.set_text(f"{nm}\nstep {min(fi, len(fr)-1)}{done}   dist {dist:.0f} m   served {int((~act).sum())}/{M}")
				out += [sd, dr, ttl] + ln + disc
			return out

		ani = FuncAnimation(fig, upd, frames=idx, blit=False, interval=70)
		out = os.path.join(FIG, "viz", f"{A_.out}_s{seed}.gif")
		ani.save(out, writer=PillowWriter(fps=14), dpi=72)
		plt.close(fig)
		r = {"seed": seed}
		for (nm, fr, ms, dist, dl, _tr) in runs:
			r[nm + " makespan"] = ms
			r[nm + " dist"] = round(dist)
			r[nm + " served"] = dl
		rows.append(r)
		for nm, _f, ms, dist, dl, tr in runs:
			b = audit(tr, pos, A, cc, 300.0, kappa)
			r[nm + " audit"] = f"move{b['move']}/conn{b['conn']}"
			print(f"    검증 {nm:24s}: 이동 위반 {b['move']}, 연결 위반 {b['conn']}", flush=True)
		print(f"seed {seed}: " + " | ".join(f"{nm} ms={ms} d={dist:.0f} {dl}/{M}" for nm, _f, ms, dist, dl, _t in runs), flush=True)
		print("  저장:", out, flush=True)
	with open(os.path.join(FIG, "viz", f"{A_.out}.csv"), "w", newline="", encoding="utf-8") as fh:
		w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)


if __name__ == "__main__":
	main()
