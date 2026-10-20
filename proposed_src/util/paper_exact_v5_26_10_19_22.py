"""RQ2 자료: BFS 정확해(격자 간격 3종) 대 두 제어기, 그리고 재적재 크기의 MILP 참조 표와 그림을 만든다.

입력: figures/bfs_n3m{M}s{seed}_h{98,69,49}_26_10_19_22.json, figures/toy_bfs_v5_26_10_19_22.csv,
      figures/milph_n3m{M}s{seed}_26_10_19_22.json, figures/toy_milph_v5_26_10_19_22.csv
출력: docs/tex/tab_exact_v5_{en,ko}_26_10_19_22.tex, tab_solver_v5_{en,ko}_26_10_19_22.tex, figures/fig_exact_v5_26_10_19_22.pdf
"""

import argparse
import csv
import glob
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FIG = os.path.join(ROOT, "figures"); TEX = os.path.join(ROOT, "docs", "tex")
plt.rcParams.update({"font.size": 8, "pdf.fonttype": 42, "font.family": "DejaVu Sans"})
C_H, C_L, C_E = "#b04236", "#1d6d9c", "#2d7a5c"
HS = (98, 69, 49)


def pstr(p):
	return "$<10^{-4}$" if p < 1e-4 else f"{p:.3f}" if p >= 0.001 else f"{p:.1e}"


def load_bfs():
	"""(M, seed) -> {h: steps} 와 재생 결과."""
	out = {}
	for f in glob.glob(os.path.join(FIG, "bfs_n3m*_h*_26_10_19_22.json")):
		d = json.load(open(f)); key = (int(d["dests"]), int(d["seed"]))
		h = int(round(float(d["spacing"])))
		out.setdefault(key, {})[h] = dict(steps=d["milp_steps"], sec=d["solve_sec"], nodes=d["nodes"],
		                                  replay=(d["replay"] or {}).get("makespan") if d.get("replay") else None)
	return out


def load_toy(name):
	p = os.path.join(FIG, name)
	if not os.path.exists(p):
		return {}
	return {(int(r["dests"]), int(r["seed"])): r for r in csv.DictReader(open(p, encoding="utf-8"))}


def converged(o, tol=0.03):
	"""세 격자 간격의 최적값이 tol 안에서 일치하면 그 인스턴스는 격자가 수렴한 것으로 본다."""
	v = [o[h]["steps"] for h in HS if h in o]
	return len(v) == len(HS) and (max(v) - min(v)) <= tol * min(v)


def tab_exact(bfs, toy, lang):
	"""격자가 수렴한 M<=5 인스턴스와 최적이 증명된 M=6 인스턴스를 한 표에 모은다 (인스턴스별)."""
	H = {"en": "$M$ & inst. & " + " & ".join(f"$h{{=}}{h}$" for h in HS) + " & heuristic & learned & learned+rec. & heur. & learned & l.+rec. \\\\",
	     "ko": "$M$ & 번호 & " + " & ".join(f"$h{{=}}{h}$" for h in HS) + " & 휴리스틱 & 학습 & 학습+복구 & 휴리 & 학습 & 학습+복구 \\\\"}
	lines, rh, rl, re = [], [], [], []
	for M in sorted({k[0] for k in bfs}):
		for k in sorted(k for k in bfs if k[0] == M):
			if not converged(bfs[k]) or k not in toy:
				continue
			o = float(bfs[k][HS[-1]]["steps"]); t = toy[k]
			h, l, e = float(t["heuristic"]), float(t["learned_median"]), float(t["esc_median"])
			rh.append(h / o); rl.append(l / o); re.append(e / o)
			lines.append(f"{M} & {k[1] - 900} & " + " & ".join(f"{bfs[k][x]['steps']:.0f}" for x in HS)
			             + f" & {h:.0f} & {l:.0f} & {e:.0f} & {h/o:.2f} & {l/o:.2f} & {e/o:.2f} \\\\")
	# M=6: 최적이 증명된 MILP 인스턴스 (격자 하나뿐이라 수렴 확인 없음)
	milp = load_toy("toy_milph_v5_26_10_19_22.csv")
	for f in sorted(glob.glob(os.path.join(FIG, "milph_n3m6s*_26_10_19_22.json"))):
		d = json.load(open(f))
		if not d["milp_steps"] or d["gap"] is None or d["gap"] > 1e-6:
			continue
		k = (6, int(d["seed"]))
		if k not in milp:
			continue
		o = float(d["milp_steps"]); t = milp[k]
		h, l, e = float(t["heuristic"]), float(t["learned_median"]), float(t["esc_median"])
		rh.append(h / o); rl.append(l / o); re.append(e / o)
		lines.append(f"6 & {k[1] - 900} & {o:.0f} & --- & --- & {h:.0f} & {l:.0f} & {e:.0f} & {h/o:.2f} & {l/o:.2f} & {e/o:.2f} \\\\")
	mean = ("\\midrule\n\\multicolumn{6}{l}{" + ("mean ratio over the " if lang == "en" else "표의 ")
	        + f"{len(rh)}" + (" instances}" if lang == "en" else "개 인스턴스 평균 비율}")
	        + f" & & & {np.mean(rh):.2f} & {np.mean(rl):.2f} & {np.mean(re):.2f} \\\\")
	with open(os.path.join(TEX, f"tab_exact_v5_{lang}_26_10_19_22.tex"), "w") as f:
		f.write("\\begin{tabular}{rr" + "r" * len(HS) + "rrrrrr}\n\\toprule\n"
		        + ("& & \\multicolumn{3}{c}{lattice optimum (steps)} & \\multicolumn{3}{c}{makespan (steps)} & \\multicolumn{3}{c}{ratio to optimum} \\\\\n"
		           if lang == "en" else
		           "& & \\multicolumn{3}{c}{격자 최적값 (스텝)} & \\multicolumn{3}{c}{makespan (스텝)} & \\multicolumn{3}{c}{최적값 대비 비율} \\\\\n")
		        + "\\cmidrule(lr){3-5}\\cmidrule(lr){6-8}\\cmidrule(lr){9-11}\n"
		        + H[lang] + "\n\\midrule\n" + "\n".join(lines) + "\n" + mean + "\n\\bottomrule\n\\end{tabular}%\n")
	print(f"tab_exact({lang}):", len(lines), "행")


def tab_solver(lang):
	"""재적재 크기: 볼록 껍질 격자 MILP 가 돌려준 실행 가능해/증명 최적/평균 갭/평균 시간, 제어기 대 최선 해."""
	toy = load_toy("toy_milph_v5_26_10_19_22.csv")
	rows = {}
	for f in glob.glob(os.path.join(FIG, "milph_n3m*_26_10_19_22.json")):
		d = json.load(open(f)); rows.setdefault(int(d["dests"]), []).append(d)
	H = {"en": "$M$ & instances & feasible & optimal & mean gap & mean time (min) & heuristic / best & learned / best & learned + recovery / best \\\\",
	     "ko": "$M$ & 인스턴스 & 실행 가능해 & 최적 증명 & 평균 갭 & 평균 시간 (분) & 휴리스틱 / 최선 & 학습 / 최선 & 학습 + 복구 / 최선 \\\\"}
	lines = []
	for M in sorted(rows):
		L = rows[M]; feas = [d for d in L if d["milp_steps"]]
		opt = [d for d in feas if d["gap"] is not None and d["gap"] < 1e-6]
		gaps = [d["gap"] for d in feas if d["gap"] is not None]
		rh, rl, re = [], [], []
		for d in feas:
			k = (M, int(d["seed"]))
			if k in toy and toy[k]["heuristic"]:
				rh.append(float(toy[k]["heuristic"]) / d["milp_steps"])
			if k in toy and toy[k]["learned_median"]:
				rl.append(float(toy[k]["learned_median"]) / d["milp_steps"])
			if k in toy and toy[k].get("esc_median"):
				re.append(float(toy[k]["esc_median"]) / d["milp_steps"])
		cells = [str(M), str(len(L)), str(len(feas)), str(len(opt)),
		         f"{100*np.mean(gaps):.0f}\\%" if gaps else "---", f"{np.mean([d['solve_sec'] for d in L])/60:.0f}",
		         f"{np.mean(rh):.2f}" if rh else "---", f"{np.mean(rl):.2f}" if rl else "---", f"{np.mean(re):.2f}" if re else "---"]
		lines.append(" & ".join(cells) + " \\\\")
	with open(os.path.join(TEX, f"tab_solver_v5_{lang}_26_10_19_22.tex"), "w") as f:
		f.write("\\begin{tabular}{rrrrrrrrr}\n\\toprule\n" + H[lang] + "\n\\midrule\n" + "\n".join(lines) + "\n\\bottomrule\n\\end{tabular}%\n")
	print(f"tab_solver({lang}):", len(lines), "행")


def fig_exact(bfs, toy):
	"""M별 패널: 인스턴스 1..10 의 격자 최적값(간격 3종), 휴리스틱, 학습 중앙값(IQR), 학습+복구 중앙값."""
	Ms = sorted({k[0] for k in bfs})
	fig, axes = plt.subplots(1, len(Ms), figsize=(7.2, 2.6))
	axes = np.atleast_1d(axes)
	mk = {98: ("s", 0.35), 69: ("s", 0.6), 49: ("s", 1.0)}
	for ax, M in zip(axes, Ms):
		keys = sorted(k for k in bfs if k[0] == M)
		x = np.arange(1, len(keys) + 1)
		for h in HS:
			y = [bfs[k].get(h, {}).get("steps", np.nan) for k in keys]
			ax.plot(x, y, marker=mk[h][0], ms=4, lw=0.8, color="#333", alpha=mk[h][1], label=f"lattice optimum, $h={h}$ m")
		hv = [float(toy[k]["heuristic"]) if k in toy and toy[k]["heuristic"] else np.nan for k in keys]
		ax.plot(x, hv, "o", ms=4, color=C_H, label="heuristic")
		lm = [float(toy[k]["learned_median"]) if k in toy and toy[k]["learned_median"] else np.nan for k in keys]
		q1 = [float(toy[k]["learned_q1"]) if k in toy and toy[k]["learned_q1"] else np.nan for k in keys]
		q3 = [float(toy[k]["learned_q3"]) if k in toy and toy[k]["learned_q3"] else np.nan for k in keys]
		ax.errorbar(x + 0.15, lm, yerr=[np.array(lm) - np.array(q1), np.array(q3) - np.array(lm)], fmt="^", ms=4, color=C_L, lw=0.8, capsize=1.5, label="learned, median (IQR of 30)")
		le = [float(toy[k]["esc_median"]) if k in toy and toy[k].get("esc_median") else np.nan for k in keys]
		ax.plot(x - 0.15, le, "v", ms=4, color=C_E, label="learned + recovery, median")
		ax.set_title(f"$M$ = {M}", fontsize=8); ax.set_xticks(x)
		ax.set_xticklabels([f"{i}" + ("*" if converged(bfs[k]) else "") for i, k in zip(x, keys)], fontsize=7)
		ax.set_xlabel("instance (* : lattice converged)", fontsize=7)
	axes[0].set_ylabel("makespan (steps)")
	h, l = axes[0].get_legend_handles_labels()
	fig.legend(h, l, loc="upper center", ncol=3, frameon=False, fontsize=6.5, bbox_to_anchor=(0.5, 1.02))
	fig.tight_layout(rect=(0, 0, 1, 0.9))
	out = os.path.join(FIG, "fig_exact_v5_26_10_19_22.pdf"); fig.savefig(out); fig.savefig(out.replace(".pdf", ".png"), dpi=160); plt.close(fig)
	print("저장:", out)


def main():
	bfs = load_bfs(); toy = load_toy("toy_bfs_v5_26_10_19_22.csv")
	print("BFS 인스턴스", len(bfs), "| toy 행", len(toy))
	for lang in ("en", "ko"):
		tab_exact(bfs, toy, lang); tab_solver(lang)
	if toy:
		fig_exact(bfs, toy)


if __name__ == "__main__":
	main()
