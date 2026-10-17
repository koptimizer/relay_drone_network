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
	ax.set_xticklabels([r[0] for r in rows])
	ax.set_ylabel("paired median makespan difference\n(learned − heuristic, steps)")
	ax.set_xlabel("drones / destinations")
	for i, r in enumerate(rows):
		ax.annotate(f"{r[5]}–{r[6]}", (i, r[4] if r[2] >= 0 else r[3]),
		            textcoords="offset points", xytext=(0, 4 if r[2] >= 0 else -10), ha="center", fontsize=7, color="#555")
	ax.text(0.01, 0.97, "heuristic faster ↑", transform=ax.transAxes, va="top", fontsize=8, color=C_H)
	ax.text(0.01, 0.03, "learned faster ↓", transform=ax.transAxes, va="bottom", fontsize=8, color=C_L)
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
		for arr, col, nm in ((a, C_H, "heuristic"), (b, C_L, "learned")):
			v = np.sort(arr[~np.isnan(arr)])
			axes[0, j].step(v, np.arange(1, len(v) + 1) / len(arr), where="post", color=col, lw=1.2, label=nm)
			axes[0, j].axvline(np.median(v), color=col, ls=":", lw=0.8)
		axes[0, j].set_title(lab, fontsize=9)
		axes[0, j].set_xlabel("makespan (steps)")
		if j == 0:
			axes[0, j].set_ylabel("fraction of rollouts")
			axes[0, j].legend(frameon=False, fontsize=7, loc="lower right")
		both = ~np.isnan(a) & ~np.isnan(b)
		d = b[both] - a[both]
		axes[1, j].hist(d, bins=30, color="#777", alpha=0.85)
		axes[1, j].axvline(0, color="#333", lw=0.8)
		axes[1, j].axvline(np.median(d), color="#000", ls="--", lw=0.9)
		axes[1, j].set_xlabel("learned − heuristic (steps)")
		axes[1, j].text(0.98, 0.95, f"median {np.median(d):+.0f}", transform=axes[1, j].transAxes,
		                ha="right", va="top", fontsize=8)
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
		lo = [float(r["learned_min"]) if r["learned_min"] else np.nan for r in sel]
		hi = [float(r["learned_max"]) if r["learned_max"] else np.nan for r in sel]
		ax.plot(x, milp, "s", color="#444", ms=4, label="MILP plan (replayed)")
		ax.plot(x, heur, "o", color=C_H, ms=4, label="heuristic")
		ax.errorbar(x, lm, yerr=[np.array(lm) - np.array(lo), np.array(hi) - np.array(lm)], fmt="^", color=C_L,
		            ms=4, elinewidth=0.6, capsize=1.5, label="learned median (min–max)")
		ax.set_title(f"$M={M}$", fontsize=9)
		ax.set_xlabel("instance")
		ax.set_xticks(x); ax.set_xticklabels([r["seed"] for r in sel], fontsize=6, rotation=90)
	axes[0].set_ylabel("makespan (steps)")
	axes[0].legend(frameon=False, fontsize=6.5, loc="upper left")
	fig.tight_layout()
	out = os.path.join(FIG, f"fig_milp_v5_{tag}.pdf")
	fig.savefig(out); plt.close(fig)
	print("저장:", out)


def fig_range(tag, files):
	"""통신 반경별 쌍별 중앙 차이. files: [(R, label, path)]."""
	files = [(R, lab, p) for R, lab, p in files if os.path.exists(p)]
	if not files:
		print("반경 CSV 없음"); return
	labs = sorted(set(l for _, l, _ in files))
	Rs = sorted(set(R for R, _, _ in files))
	fig, ax = plt.subplots(figsize=(4.6, 2.8))
	for li, lab in enumerate(labs):
		xs, ys = [], []
		for R in Rs:
			p = [p for r, l, p in files if r == R and l == lab]
			if not p:
				continue
			a, b = paired(p[0]); both = ~np.isnan(a) & ~np.isnan(b)
			xs.append(R); ys.append(float(np.median(b[both] - a[both])))
		ax.plot(xs, ys, "o-", label=lab, lw=1.2, ms=4)
	ax.axhline(0, color="#333", lw=0.8)
	ax.set_xlabel("communication radius $R$ (m)")
	ax.set_ylabel("paired median difference\n(learned − heuristic, steps)")
	ax.legend(frameon=False, fontsize=8)
	fig.tight_layout()
	out = os.path.join(FIG, f"fig_range_v5_{tag}.pdf")
	fig.savefig(out); plt.close(fig)
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
		      ("8/150", 8, F(f"paired_d8m150_SWa_{A.suffix}.csv")), ("10/300", 10, F(f"paired_d10m300_SWa_{A.suffix}.csv"))]
	else:
		sz = [("3/20", 3, F("paired_size_d3_m20_v5_26_10_02_11.csv")), ("3/30", 3, F("paired200_d3_SWa_v5_26_09_27_08.csv")),
		      ("4/50", 4, F("paired_std480_SWa_v5_26_10_02_12.csv")), ("4/80", 4, F("paired_size_d4_m80_v5_26_10_02_11.csv")),
		      ("5/50", 5, F("paired200_d5_SWa_v5_26_09_27_08.csv")), ("6/80", 6, F("paired_size_d6_m80_v5_26_10_02_11.csv")),
		      ("8/150", 8, F("paired_size_d8_m150_v5_26_10_02_11.csv")), ("10/300", 10, F("paired_size_d10_m300_v5_26_10_02_11.csv"))]
	fig_regime(A.tag, sz)
	fig_cdf(A.tag, [(l, p) for l, n, p in sz if l in ("3/30", "4/50", "5/50")])
	fig_heur(A.tag, F("heur_ablate_v5_26_10_17_15_*.csv"))
	fig_milp(A.tag, F("toy_compare_v5_26_10_17_15.csv"))
	fig_range(A.tag, [(R, lab, F(f"paired_R{R}_{lab.replace('/', '')}_SWa_v5_26_10_17_15.csv"))
	                  for R in (250, 300, 350) for lab in ("3/30", "4/50", "5/50")])
	fig_latency(A.tag)


if __name__ == "__main__":
	main()
