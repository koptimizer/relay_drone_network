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


def tab_exact(bfs, toy, lang):
	"""M별: 최적값 평균(간격 3종), 제어기/최적 비율(가장 촘촘한 격자 기준), 쌍별 검정."""
	H = {"en": ("$M$ & $n$ & " + " & ".join(f"$h={h}$" for h in HS) + " & heuristic & learned (median) & learned + recovery & $p$ (heur.\\ vs.\\ learned) \\\\",),
	     "ko": ("$M$ & $n$ & " + " & ".join(f"$h={h}$" for h in HS) + " & 휴리스틱 & 학습 (중앙값) & 학습 + 복구 & $p$ (휴리스틱 대 학습) \\\\",)}
	lines = []
	for M in sorted({k[0] for k in bfs}):
		keys = sorted(k for k in bfs if k[0] == M and all(h in bfs[k] for h in HS) and k in toy)
		if not keys:
			continue
		opt = {h: np.array([bfs[k][h]["steps"] for k in keys], dtype=float) for h in HS}
		fine = HS[-1]
		heur = np.array([float(toy[k]["heuristic"]) if toy[k]["heuristic"] else np.nan for k in keys])
		lm = np.array([float(toy[k]["learned_median"]) if toy[k]["learned_median"] else np.nan for k in keys])
		le = np.array([float(toy[k]["esc_median"]) if toy[k].get("esc_median") else np.nan for k in keys])
		rh, rl, re = heur / opt[fine], lm / opt[fine], le / opt[fine]
		both = ~np.isnan(heur) & ~np.isnan(lm)
		p = stats.wilcoxon(lm[both] - heur[both]).pvalue if both.sum() > 5 and np.any(lm[both] != heur[both]) else np.nan
		cells = [str(M), str(len(keys))] + [f"{opt[h].mean():.0f}" for h in HS]
		cells += [f"{np.nanmean(rh):.3f}", f"{np.nanmean(rl):.3f}", f"{np.nanmean(re):.3f}" if not np.all(np.isnan(re)) else "---",
		          pstr(p) if not np.isnan(p) else "---"]
		lines.append(" & ".join(cells) + " \\\\")
	with open(os.path.join(TEX, f"tab_exact_v5_{lang}_26_10_19_22.tex"), "w") as f:
		f.write("\\begin{tabular}{rr" + "r" * len(HS) + "rrrr}\n\\toprule\n" + H[lang][0] + "\n\\midrule\n" + "\n".join(lines) + "\n\\bottomrule\n\\end{tabular}%\n")
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
		ax.set_title(f"$M$ = {M}", fontsize=8); ax.set_xticks(x); ax.set_xlabel("instance")
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
