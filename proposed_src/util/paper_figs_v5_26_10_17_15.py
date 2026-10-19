"""논문 그림 생성 (사이클 26). 쌍별 CSV·절제 CSV·MILP JSON에서 그린다.

사용: python paper_figs_v5_26_10_17_15.py [--tag 26_10_17_15] [--suffix <쌍별 CSV 접미>]
그림은 figures/fig_*_v5_<tag>.pdf 로 저장되며 docs/tex 에서 \\includegraphics 로 읽는다.
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

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FIG = os.path.join(ROOT, "figures")
plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
                     "pdf.fonttype": 42, "font.family": "DejaVu Sans"})
C_H, C_L = "#b04236", "#1d6d9c"


def paired(path):
	"""쌍별 CSV -> (규칙 배열, 학습 배열) with NaN for incomplete."""
	r = list(csv.DictReader(open(path, encoding="utf-8")))
	a = np.array([float(x["규칙+탈출"]) if x["규칙+탈출"] else np.nan for x in r])
	b = np.array([float(x["학습"]) if x["학습"] else np.nan for x in r])
	return a, b


def fig_regime(tag, files):
	"""쌍별 중앙 차이 대 드론 수. files: [(label, N, path), ...]."""
	rows = []
	for lab, n, path in files:
		if not os.path.exists(path):
			continue
		a, b = paired(path)
		both = ~np.isnan(a) & ~np.isnan(b)
		d = b[both] - a[both]
		lo, hi = np.percentile(d, [25, 75])
		rows.append((lab, n, float(np.median(d)), lo, hi, int((d < 0).sum()), int((d > 0).sum())))
	rows.sort(key=lambda r: (r[1], r[0]))
	fig, ax = plt.subplots(figsize=(6.2, 3.3))
	x = np.arange(len(rows))
	med = [r[2] for r in rows]
	err = [[r[2] - r[3] for r in rows], [r[4] - r[2] for r in rows]]
	cols = [C_H if m > 0 else C_L for m in med]
	ax.bar(x, med, color=cols, alpha=0.85, width=0.65)
	ax.errorbar(x, med, yerr=err, fmt="none", ecolor="#333", elinewidth=0.8, capsize=2)
	ax.axhline(0, color="#333", lw=0.8)
	ax.set_xticks(x)
	ax.set_xticklabels([f"{r[0]}\n{r[5]}–{r[6]}" for r in rows], fontsize=8)
	ax.set_ylabel("paired median makespan difference\n(learned − heuristic, steps)")
	ax.set_xlabel("drones / demand points  (wins–losses for the learned policy)")
	ax.text(0.99, 0.97, "heuristic faster ↑", transform=ax.transAxes, va="top", ha="right", fontsize=8, color=C_H)
	ax.text(0.99, 0.03, "learned faster ↓", transform=ax.transAxes, va="bottom", ha="right", fontsize=8, color=C_L)
	fig.tight_layout()
	out = os.path.join(FIG, f"fig_regime_v5_{tag}.pdf")
	fig.savefig(out); plt.close(fig)
	print("저장:", out)


def fig_cdf(tag, files):
	"""세 구성의 makespan 누적분포와 쌍별 차이 히스토그램."""
	files = [(lab, p) for lab, p in files if os.path.exists(p)]
	fig, axes = plt.subplots(2, len(files), figsize=(2.3 * len(files) + 0.6, 4.2))
	if len(files) == 1:
		axes = axes.reshape(2, 1)
	for j, (lab, p) in enumerate(files):
		a, b = paired(p)
		allv = np.concatenate([a[~np.isnan(a)], b[~np.isnan(b)]])
		xmax = float(np.percentile(allv, 98))
		for arr, col, nm in ((a, C_H, "heuristic"), (b, C_L, "learned")):
			v = np.sort(arr[~np.isnan(arr)])
			axes[0, j].step(v, np.arange(1, len(v) + 1) / len(arr), where="post", color=col, lw=1.2, label=nm)
			axes[0, j].axvline(np.median(v), color=col, ls=":", lw=0.8)
		axes[0, j].set_xlim(float(allv.min()) * 0.9, xmax)
		axes[0, j].set_title(lab, fontsize=9)
		axes[0, j].set_xlabel("makespan (steps, to 98th pct.)")
		if j == 0:
			axes[0, j].set_ylabel("fraction of rollouts")
			axes[0, j].legend(frameon=False, fontsize=7, loc="lower right")
		both = ~np.isnan(a) & ~np.isnan(b)
		d = b[both] - a[both]
		lim = 400.0
		dc = np.clip(d, -lim, lim)
		axes[1, j].hist(dc, bins=np.linspace(-lim, lim, 33), color="#777", alpha=0.85)
		axes[1, j].axvline(0, color="#333", lw=0.8)
		axes[1, j].axvline(np.median(d), color="#000", ls="--", lw=0.9)
		axes[1, j].set_xlabel("learned − heuristic (steps)")
		over = int((np.abs(d) > lim).sum())
		right = np.median(d) <= 0
		axes[1, j].text(0.98 if right else 0.02, 0.95, f"median {np.median(d):+.0f}\n{over} pairs beyond ±400",
		                transform=axes[1, j].transAxes, ha="right" if right else "left", va="top", fontsize=7)
		if j == 0:
			axes[1, j].set_ylabel("pairs")
	fig.tight_layout()
	out = os.path.join(FIG, f"fig_cdf_v5_{tag}.pdf")
	fig.savefig(out); plt.close(fig)
	print("저장:", out)


def fig_heur(tag, pattern):
	"""휴리스틱 절제 열지도: 설정별 완주 합과 중앙 makespan 합."""
	rows = []
	for f in sorted(glob.glob(pattern)):
		rows += list(csv.DictReader(open(f, encoding="utf-8")))
	if not rows:
		print("절제 CSV 없음"); return
	cfgs = sorted(set(r["config"] for r in rows))
	ks, hs = [30, 60, 90, 120], [20, 40, 80]
	fig, axes = plt.subplots(1, 2, figsize=(6.4, 2.6))
	for ai, mode in enumerate(("shuffle", "retreat")):
		Z = np.full((len(hs), len(ks)), np.nan)
		Cc = np.zeros((len(hs), len(ks)))
		for i, h in enumerate(hs):
			for j, k in enumerate(ks):
				sel = [r for r in rows if r["mode"] == mode and int(r["k_stall"]) == k and int(r["hold"]) == h]
				if len(sel) == len(cfgs):
					Z[i, j] = sum(int(r["median"]) for r in sel if r["median"])
					Cc[i, j] = sum(int(r["complete"]) for r in sel)
		im = axes[ai].imshow(Z, cmap="viridis_r", aspect="auto")
		axes[ai].set_xticks(range(len(ks))); axes[ai].set_xticklabels(ks)
		axes[ai].set_yticks(range(len(hs))); axes[ai].set_yticklabels(hs)
		axes[ai].set_xlabel("stall threshold $k_{stall}$ (steps)")
		axes[ai].set_ylabel("hold $h_{hold}$ (steps)")
		axes[ai].set_title("lead swap" if mode == "shuffle" else "retreat", fontsize=9)
		for i in range(len(hs)):
			for j in range(len(ks)):
				if not np.isnan(Z[i, j]):
					axes[ai].text(j, i, f"{int(Z[i, j])}\n{int(Cc[i, j])}", ha="center", va="center", fontsize=6.5, color="w")
	fig.colorbar(im, ax=axes, label="sum of median makespan (3 configs)", shrink=0.8)
	out = os.path.join(FIG, f"fig_heur_v5_{tag}.pdf")
	fig.savefig(out, bbox_inches="tight"); plt.close(fig)
	print("저장:", out)


def fig_milp(tag, csv_path):
	"""MILP 참조 인스턴스별 세 방법의 값."""
	if not os.path.exists(csv_path):
		print("toy CSV 없음"); return
	rows = list(csv.DictReader(open(csv_path, encoding="utf-8")))
	Ms = sorted(set(int(r["dests"]) for r in rows))
	fig, axes = plt.subplots(1, len(Ms), figsize=(1.9 * len(Ms) + 0.8, 2.8), sharey=False)
	if len(Ms) == 1:
		axes = [axes]
	for ax, M in zip(axes, Ms):
		sel = sorted([r for r in rows if int(r["dests"]) == M], key=lambda r: int(r["seed"]))
		x = np.arange(len(sel))
		milp = [float(r["replay"]) if r["replay"] else np.nan for r in sel]
		heur = [float(r["heuristic"]) if r["heuristic"] else np.nan for r in sel]
		lm = [float(r["learned_median"]) if r["learned_median"] else np.nan for r in sel]
		lo = [float(r["learned_q1"]) if r.get("learned_q1") else np.nan for r in sel]
		hi = [float(r["learned_q3"]) if r.get("learned_q3") else np.nan for r in sel]
		ax.plot(x, milp, "s", color="#444", ms=4, label="MILP plan (replayed)")
		ax.plot(x, heur, "o", color=C_H, ms=4, label="heuristic")
		ax.errorbar(x, lm, yerr=[np.array(lm) - np.array(lo), np.array(hi) - np.array(lm)], fmt="^", color=C_L,
		            ms=4, elinewidth=0.6, capsize=1.5, label="learned median (IQR of 30 samples)")
		ax.set_title(f"$M={M}$", fontsize=9)
		ax.set_xlabel("instance")
		ax.set_xticks(x); ax.set_xticklabels([str(k + 1) for k in range(len(sel))], fontsize=6)
	axes[0].set_ylabel("makespan (steps)")
	h, l = axes[0].get_legend_handles_labels()
	fig.legend(h, l, frameon=False, fontsize=7, loc="upper center", ncol=3, bbox_to_anchor=(0.5, 1.02))
	fig.tight_layout(rect=(0, 0, 1, 0.93))
	out = os.path.join(FIG, f"fig_milp_v5_{tag}.pdf")
	fig.savefig(out); plt.close(fig)
	print("저장:", out)


def fig_range(tag, files):
	"""통신 반경별 쌍별 중앙 차이: 구성마다 한 패널, 막대 부호로 누가 빠른지, IQR 수염, 승-패 표기."""
	files = [(R, lab, p) for R, lab, p in files if os.path.exists(p)]
	if not files:
		print("반경 CSV 없음"); return
	labs = sorted(set(l for _, l, _ in files))
	Rs = sorted(set(R for R, _, _ in files))
	fig, axes = plt.subplots(1, len(labs), figsize=(5.0, 2.5), sharey=True)
	ymin, ymax = 0, 0
	for ax, lab in zip(axes, labs):
		comp = {}
		for R in Rs:
			p = [p for r, l, p in files if r == R and l == lab]
			if not p:
				continue
			a, b = paired(p[0]); both = ~np.isnan(a) & ~np.isnan(b)
			d = b[both] - a[both]
			med = float(np.median(d)); q1, q3 = np.percentile(d, [25, 75])
			col = C_H if med > 0 else C_L
			ax.bar(R, med, width=32, color=col, alpha=0.85 if R == 300 else 0.55, edgecolor=col, lw=0.8)
			ax.plot([R, R], [q1, q3], color="#333", lw=0.9)
			w, l = int((d < 0).sum()), int((d > 0).sum())
			ax.text(R, (q3 if med > 0 else q1) + (6 if med > 0 else -6), f"{w}–{l}", ha="center",
			        va="bottom" if med > 0 else "top", fontsize=6.2, color="#333")
			comp[R] = f"{int((~np.isnan(a)).sum())}/{int((~np.isnan(b)).sum())}"
			ymin, ymax = min(ymin, q1), max(ymax, q3)
		ax.axhline(0, color="#333", lw=0.8)
		ax.axvspan(284, 316, color="#ddd", alpha=0.5, lw=0, zorder=0)
		ax.set_xticks(Rs); ax.set_xticklabels([str(r) for r in Rs], fontsize=7)
		import matplotlib.transforms as mtr
		tr = mtr.blended_transform_factory(ax.transData, ax.transAxes)
		for r in Rs:
			ax.text(r, -0.16, comp.get(r, ""), transform=tr, ha="center", va="top", fontsize=5.3, color="#555")
		ax.set_xlim(Rs[0] - 35, Rs[-1] + 35)
		ax.set_title(f"$N/M$ = {lab}", fontsize=8)
		ax.tick_params(axis="y", labelsize=7)
		ax.set_xlabel("$R$ (m)", fontsize=7, labelpad=13)
	axes[0].set_ylabel("paired median difference (steps)\nlearned − heuristic", fontsize=7)
	pad = 0.22 * (ymax - ymin)
	axes[0].set_ylim(ymin - pad, ymax + pad)
	for ax in axes:
		ax.text(0.02, 0.97, "heuristic faster", transform=ax.transAxes, fontsize=5.8, color=C_H, va="top")
		ax.text(0.02, 0.03, "learned faster", transform=ax.transAxes, fontsize=5.8, color=C_L, va="bottom")
	from matplotlib.patches import Patch
	fig.legend(handles=[Patch(color="#ddd", label="training radius 300 m"),
	                    plt.Line2D([], [], color="#333", lw=0.9, label="interquartile range; label = wins–losses of the learned policy;\nsmall numbers = completions heuristic/learned")],
	           loc="lower center", ncol=2, frameon=False, fontsize=6.3, bbox_to_anchor=(0.5, -0.03))
	fig.tight_layout(rect=(0, 0.08, 1, 1))
	out = os.path.join(FIG, f"fig_range_v5_{tag}.pdf")
	fig.savefig(out); fig.savefig(out.replace(".pdf", ".png"), dpi=160); plt.close(fig)
	print("저장:", out)


def fig_latency(tag):
	"""상위 결정 1회 지연 (측정값은 speed 스크립트 기록)."""
	N = [3, 4, 6, 8]
	rule = [0.05, 0.08, 0.10, 0.12]
	ind = [1.27, 1.52, 1.94, 2.45]
	ar = [2.70, 3.52, 5.32, 7.35]
	fig, ax = plt.subplots(figsize=(4.2, 2.7))
	ax.plot(N, rule, "o-", color=C_H, label="heuristic")
	ax.plot(N, ind, "s--", color=C_L, alpha=0.6, label="learned, independent")
	ax.plot(N, ar, "^-", color=C_L, label="learned, autoregressive")
	ax.set_yscale("log")
	ax.set_xlabel("drones"); ax.set_ylabel("decision latency (ms)")
	ax.set_xticks(N)
	ax.legend(frameon=False, fontsize=8)
	fig.tight_layout()
	out = os.path.join(FIG, f"fig_latency_v5_{tag}.pdf")
	fig.savefig(out); plt.close(fig)
	print("저장:", out)


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--tag", default="26_10_17_15")
	p.add_argument("--suffix", default="", help="쌍별 CSV 접미 (재측정본). 비우면 기존 파일을 쓴다")
	A = p.parse_args()
	F = lambda name: os.path.join(FIG, name)
	if A.suffix:
		sz = [("3/20", 3, F(f"paired_d3m20_SWa_{A.suffix}.csv")), ("3/30", 3, F(f"paired_d3_SWa_{A.suffix}.csv")),
		      ("4/50", 4, F(f"paired_std_SWa_{A.suffix}.csv")), ("4/80", 4, F(f"paired_d4m80_SWa_{A.suffix}.csv")),
		      ("5/50", 5, F(f"paired_d5_SWa_{A.suffix}.csv")), ("6/80", 6, F(f"paired_d6m80_SWa_{A.suffix}.csv")),
		      ("8/150", 8, F(f"paired_d8m150_SWa_{A.suffix}.csv")), ("10/300", 10, F(f"paired_d10m300_SWa_{A.suffix}.csv")),
		      ("12/400", 12, F("paired_d12m400_SWa_v5_26_10_18_06.csv"))]
	else:
		sz = [("3/20", 3, F("paired_size_d3_m20_v5_26_10_02_11.csv")), ("3/30", 3, F("paired_d3_rep5_v5_26_09_27_11.csv")),
		      ("4/50", 4, F("paired_std480_SWa_v5_26_10_02_12.csv")), ("4/80", 4, F("paired_size_d4_m80_v5_26_10_02_11.csv")),
		      ("5/50", 5, F("paired_d5_rep5_v5_26_09_27_11.csv")), ("6/80", 6, F("paired_size_d6_m80_v5_26_10_02_11.csv")),
		      ("8/150", 8, F("paired_size_d8_m150_v5_26_10_02_11.csv")), ("10/300", 10, F("paired_size_d10_m300_v5_26_10_02_11.csv"))]
	fig_regime(A.tag, sz)
	fig_cdf(A.tag, [(l, p) for l, n, p in sz if l in ("3/30", "4/50", "5/50")])
	fig_heur(A.tag, F("heur_ablate_v5_26_10_17_15_*.csv"))
	fig_milp(A.tag, F("toy_compare_v5_26_10_17_15.csv"))
	lp = {l: p for l, n, p in sz}
	base = {"3/30": lp["3/30"], "4/50": lp["4/50"], "5/50": lp["5/50"]}
	rf = []
	for lab in ("3/30", "4/50", "5/50"):
		for R in (250, 300, 350):
			rf.append((R, lab, base[lab] if R == 300 else F(f"paired_R{R}_{lab.replace('/', '')}_SWa_v5_26_10_17_15.csv")))
	fig_range(A.tag, rf)
	fig_latency(A.tag)


if __name__ == "__main__":
	main()
