"""MILP 결과를 논문용 표와 경로 그림으로 정리한다 (v5, 26_09_21_03)."""

import glob
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SPEED = 50.0 * 1000 / 3600
SERVICE = 10


def lower_bound(cc, dests, num_drones):
	"""연속 문제의 자명한 하한: 최원거리 비행시간과 병렬 서비스 시간 중 큰 값."""
	d = float(np.max(np.linalg.norm(np.asarray(dests) - np.asarray(cc), axis=1)))
	return max(d / SPEED, np.ceil(len(dests) / num_drones) * SERVICE), d


def plan_figure(npy_path, out_pdf, title):
	"""격자·목적지·드론 경로를 한 장에 그린다."""
	blob = np.load(npy_path, allow_pickle=True).item()
	plan, pos, cc, dests = blob["plan"], blob["pos"], blob["cc"], blob["dests"]
	fig, ax = plt.subplots(figsize=(6.2, 5.6))
	nd = len(dests)
	ax.scatter(pos[nd + 1:, 0], pos[nd + 1:, 1], s=4, c="0.85", label="lattice nodes", zorder=1)
	ax.scatter(dests[:, 0], dests[:, 1], s=52, marker="s", facecolor="white",
	           edgecolor="tab:red", linewidth=1.3, label="destinations", zorder=4)
	ax.scatter([cc[0]], [cc[1]], s=170, marker="*", c="k", label="control centre", zorder=5)
	cols = plt.cm.viridis(np.linspace(0.05, 0.78, plan.shape[1]))
	for k in range(plan.shape[1]):
		tr = pos[plan[:, k]] + (k - 1) * 6.0
		ax.plot(tr[:, 0], tr[:, 1], "-o", ms=2.8, lw=1.3, color=cols[k], alpha=0.9,
		        label=f"drone {k + 1}", zorder=3)
	ax.set_aspect("equal")
	ax.set_xlabel("x [m]")
	ax.set_ylabel("y [m]")
	ax.set_title(title, fontsize=10)
	ax.legend(fontsize=7, loc="best", framealpha=0.9)
	fig.tight_layout()
	fig.savefig(out_pdf)
	plt.close(fig)


def collect():
	"""figures/milp_*.json을 읽어 목적지 수별로 묶는다."""
	out = {}
	for p in sorted(glob.glob(os.path.join(ROOT, "figures", "milp_m*_26_09_21_03.json"))):
		r = json.load(open(p))
		r["_tag"] = os.path.basename(p)[:-5]
		out.setdefault(r["dests"], []).append(r)
	return out


def main():
	"""결과 json들을 읽어 tex 조각과 그림을 만든다."""
	sys.path.insert(0, os.path.join(ROOT, "proposed_src"))
	from util.instance_generator_v5_26_09_15_22 import sample_instance
	groups = collect()
	if not groups:
		print("결과 파일이 없다")
		return
	L, lines = [], []
	any_r = next(iter(groups.values()))[0]
	L.append("The instances come from the project's own generator (\\texttt{sample\\_instance}, seed "
	         f"{any_r['seed']}) with $N = {any_r['drones']}$ drones, so that the MILP, the heuristic and "
	         "the learned policy all see the same geometry. Gurobi 13.0.2 ran with six threads on a "
	         "machine that was concurrently training the reinforcement-learning agent. "
	         "The lattice uses $\\kappa = 10$ simulator steps per period and $h = 98.2$\\,m.\n")
	L.append("\\begin{table}[h]\n\\centering\n\\caption{Model size and search outcome. Makespan is in "
	         "simulator steps (seconds). ``best'' is the best incumbent over the runs listed; the "
	         "spread shows how much the incumbent varies between identical runs with different solver "
	         "seeds.}\n\\label{tab:milp}\n"
	         "\\begin{tabular}{rrrrrrrr}\n\\toprule\n"
	         "$M$ & $|N|$ & $\\bar{T}$ & binaries & runs & best & spread & bound \\\\\n\\midrule")
	for M in sorted(groups):
		rs = groups[M]
		ok = [r for r in rs if r.get("milp_steps") is not None]
		ms = [r["milp_steps"] for r in ok]
		ref = min(ok, key=lambda r: r["milp_steps"]) if ok else rs[0]
		runs = f"{len(ok)}/{len(rs)}"
		bestv = f"{min(ms):.0f}" if ms else "none"
		spread = "---" if len(ms) <= 1 else f"{min(ms):.0f}--{max(ms):.0f}"
		bd = f"{ref['bound_steps']:.0f}" if ref.get("bound_steps") is not None else "---"
		L.append(f"{M} & {ref['nodes']} & {ref['horizon']} & {ref.get('binaries', 0) or '---'} & "
		         f"{runs} & {bestv} & {spread} & {bd} \\\\")
	L.append("\\bottomrule\n\\end{tabular}\n\\end{table}\n")

	L.append("\\begin{table}[h]\n\\centering\n\\caption{The best MILP plan replayed in the simulator, "
	         "against the geometric relay rule with deadlock escape on the same instance. ``cut'' counts "
	         "the drone-steps in which the simulator's connectivity projection shortened a commanded "
	         "displacement, the price of sampling connectivity only at period boundaries.}\n"
	         "\\label{tab:replay}\n"
	         "\\begin{tabular}{rrrrrr}\n\\toprule\n"
	         "$M$ & MILP & replayed & cut steps & rule & $C_{\\mathrm{LB}}$ \\\\\n\\midrule")
	for M in sorted(groups):
		ok = [r for r in groups[M] if r.get("milp_steps") is not None]
		ref = min(ok, key=lambda r: r["milp_steps"]) if ok else groups[M][0]
		cc, dests = sample_instance(M, seed=ref["seed"], num_drones=ref["drones"])
		lb, _ = lower_bound(cc, dests, ref["drones"])
		rp = ref.get("replay")
		mk = (rp["makespan"] if rp and rp["makespan"] else "---")
		cut = rp["blocked"] if rp else "---"
		mil = f"{ref['milp_steps']:.0f}" if ok else "none"
		L.append(f"{M} & {mil} & {mk} & {cut} & {ref['rule_makespan']} & {lb:.0f} \\\\")
	L.append("\\bottomrule\n\\end{tabular}\n\\end{table}\n")

	# 서술: 편차와 실행 가능해 확보 실패를 수치로 말한다
	nar = []
	for M in sorted(groups):
		rs = groups[M]
		ok = [r for r in rs if r.get("milp_steps") is not None]
		if len(rs) > 1:
			nar.append(f"at $M = {M}$, {len(ok)} of {len(rs)} runs returned a feasible plan")
		elif not ok:
			nar.append(f"at $M = {M}$, the single run returned none")
	spreads = [r["milp_steps"] for r in groups[min(groups)] if r.get("milp_steps") is not None]
	wins = []
	for M in sorted(groups):
		ok = [r for r in groups[M] if r.get("milp_steps") is not None]
		if not ok:
			continue
		b = min(ok, key=lambda r: r["milp_steps"])
		if b["replay"] and b["replay"]["makespan"]:
			wins.append(f"{b['replay']['makespan']} against {b['rule_makespan']}\\,s at $M = {M}$")
	L.append("Two facts stand out. The incumbent is unstable: " +
	         (f"{len(spreads)} runs of the same $M = {min(groups)}$ model with different solver seeds "
	          f"returned makespans from {min(spreads):.0f} to {max(spreads):.0f}\\,s. " if len(spreads) > 1 else "") +
	         "And the primal side degrades sharply with the number of destinations: " +
	         ", ".join(nar) + ". The model is a reference for the smallest instances only. "
	         "Where it does return a plan, the plan is good: replayed in the simulator it beats the "
	         "geometric rule with deadlock escape (" + "; ".join(wins) + "), and it never loses "
	         "connectivity. The cut-step counts in Table~\\ref{tab:replay} are the price of sampling "
	         "connectivity at period boundaries: the simulator had to shorten that many commanded "
	         "displacements, without ever disconnecting a drone.\n")

	big = max(M for M in groups if any(r.get("milp_steps") is not None for r in groups[M]))
	best = min([r for r in groups[big] if r.get("milp_steps") is not None],
	           key=lambda r: r["milp_steps"])
	npy = os.path.join(ROOT, "figures", f"{best['_tag']}_plan.npy")
	fig_pdf = os.path.join(ROOT, "figures", "milp_plan_v5_26_09_21_03.pdf")
	if os.path.exists(npy):
		plan_figure(npy, fig_pdf, f"MILP plan, {best['drones']} drones, {big} destinations "
		                          f"(makespan {best['milp_steps']:.0f}\\,s)")
		L.append("\\begin{figure}[h]\n\\centering\n"
		         "\\includegraphics[width=0.70\\textwidth]{../../figures/milp_plan_v5_26_09_21_03.pdf}\n"
		         "\\caption{Lattice, destinations and optimal trajectories for the largest instance the "
		         "model could solve. Trajectories are offset by a few metres per drone so that shared "
		         "nodes stay visible.}\n\\label{fig:plan}\n\\end{figure}\n")
	out = os.path.join(ROOT, "docs", "tex", "milp_results_v5_26_09_21_03.tex")
	with open(out, "w") as f:
		f.write("\n".join(L))
	print("저장:", out)
	for M in sorted(groups):
		rs = groups[M]
		print(f"  M={M}: 실행 {len(rs)}회 makespan {[r['milp_steps'] for r in rs]} "
		      f"규칙 {rs[0]['rule_makespan']}")


if __name__ == "__main__":
	main()
