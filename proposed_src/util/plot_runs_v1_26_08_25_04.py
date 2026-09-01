"""학습 실행들의 지표 추이와 홀드아웃 비교를 figures/에 그린다.

한글 폰트가 없는 환경이므로 축·범례는 영문으로 쓴다.
보상은 실행마다 보상 함수가 달라 비교 불가하므로 그리지 않는다.
"""

import argparse
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FIG = os.path.join(ROOT, "figures")
LABEL = {
	"v1_26_08_24_22": "Run A (approved fixes)",
	"v1_26_08_25_04": "Run C (+relay potential, bugfix)",
}


def load(tag):
	"""실행 태그의 metrics CSV를 컬럼별 배열 dict로 읽는다."""
	path = os.path.join(ROOT, "runs", tag, f"metrics_{tag}.csv")
	rows = list(csv.DictReader(open(path, encoding="utf-8")))
	out = {}
	for k in rows[0]:
		out[k] = np.array([float(r[k]) if r[k] not in ("", "nan") else np.nan for r in rows])
	return out


def smooth(y, w):
	"""이동평균으로 곡선을 평활한다."""
	if len(y) < w:
		return y
	return np.convolve(y, np.ones(w) / w, mode="valid")


def curve(ax, data, key, w, title, ylabel):
	"""실행별 지표 추이를 한 축에 겹쳐 그린다."""
	for tag, d in data.items():
		if key not in d or np.all(np.isnan(d[key])):
			continue
		y = smooth(d[key], w)
		ax.plot(np.arange(len(y)) + w, y, label=LABEL.get(tag, tag), lw=1.4)
	ax.set_title(title, fontsize=11)
	ax.set_xlabel("episode")
	ax.set_ylabel(ylabel)
	ax.grid(alpha=0.3)


def main():
	"""추이 그래프와 홀드아웃 비교 막대그래프를 저장한다."""
	p = argparse.ArgumentParser()
	p.add_argument("--runs", nargs="*", default=["v1_26_08_24_22", "v1_26_08_25_04"])
	p.add_argument("--smooth", type=int, default=100)
	p.add_argument("--tag", default="v1_26_08_25_04")
	args = p.parse_args()

	os.makedirs(FIG, exist_ok=True)
	data = {t: load(t) for t in args.runs if os.path.exists(
		os.path.join(ROOT, "runs", t, f"metrics_{t}.csv"))}

	fig, axes = plt.subplots(2, 2, figsize=(11, 7))
	curve(axes[0][0], data, "delivered", args.smooth,
	      "Deliveries per episode (of 50)", "destinations served")
	curve(axes[0][1], data, "hop2plus", args.smooth,
	      "Relay chain formation (hop depth >= 2)", "fraction of drone-steps")
	curve(axes[1][0], data, "idle_drones", args.smooth,
	      "Idle drones (0 deliveries)", "count of 4")
	curve(axes[1][1], data, "max_reach", args.smooth,
	      "Max distance reached from control center", "distance")
	axes[1][1].axhline(500, ls="--", c="gray", lw=1)
	axes[1][1].annotate("direct comm range (500)", (0.02, 0.06),
	                    xycoords="axes fraction", fontsize=8, color="gray")
	axes[0][0].legend(fontsize=8)
	fig.suptitle("Training progression — v1 runs", fontsize=12)
	fig.tight_layout()
	out1 = os.path.join(FIG, f"training_curves_{args.tag}.png")
	fig.savefig(out1, dpi=140)
	print(f"저장: {out1}")

	# 홀드아웃 비교 막대
	cmp_path = os.path.join(FIG, f"compare_{args.tag}.csv")
	if os.path.exists(cmp_path):
		rows = list(csv.DictReader(open(cmp_path, encoding="utf-8")))
		names = [r["run"].split(" (")[0] for r in rows]
		deliv = [float(r["배송"].split("/")[0]) for r in rows]
		idle = [float(r["미활동드론"]) for r in rows]
		hop = [float(r["2홉이상비율"].rstrip("%")) for r in rows]

		fig2, ax = plt.subplots(1, 3, figsize=(12, 3.6))
		for a, v, t, yl in [(ax[0], deliv, "Deliveries (of 50)", "destinations"),
		                    (ax[1], idle, "Idle drones (lower better)", "count of 4"),
		                    (ax[2], hop, "Relay chain usage", "% drone-steps at hop>=2")]:
			bars = a.bar(range(len(v)), v, color=["#888", "#4878a8", "#c04a4a"][:len(v)])
			a.set_xticks(range(len(v)))
			a.set_xticklabels(names, fontsize=7, rotation=12)
			a.set_title(t, fontsize=10)
			a.set_ylabel(yl, fontsize=8)
			a.bar_label(bars, fmt="%.1f", fontsize=8)
			a.grid(alpha=0.3, axis="y")
		fig2.suptitle("Held-out evaluation on 20 unseen instances (deterministic policy)", fontsize=11)
		fig2.tight_layout()
		out2 = os.path.join(FIG, f"holdout_compare_{args.tag}.png")
		fig2.savefig(out2, dpi=140)
		print(f"저장: {out2}")


def plot_sweep(tags, tag):
	"""홀드아웃 성능이 학습량에 따라 어떻게 변하는지와 홉-배송 상관을 그린다."""
	data = {}
	for t in tags:
		f = os.path.join(FIG, f"sweep_{t}.csv")
		if os.path.exists(f):
			data[t] = list(csv.DictReader(open(f, encoding="utf-8")))
	if not data:
		return

	fig, ax = plt.subplots(1, 3, figsize=(13, 3.8))
	for t, rows in data.items():
		ep = [int(r["episode"]) for r in rows]
		dl = [float(r["delivered"]) for r in rows]
		sd = [float(r["delivered_sd"]) for r in rows]
		ax[0].errorbar(ep, dl, yerr=sd, lw=1.4, capsize=2, label=LABEL.get(t, t))
		ax[1].plot(ep, [float(r["hop2plus"]) for r in rows], lw=1.4, label=LABEL.get(t, t))
		ax[2].scatter([float(r["hop2plus"]) for r in rows], dl, s=22, alpha=0.75,
		              label=LABEL.get(t, t))

	ax[0].set_title("Held-out deliveries vs training length", fontsize=10)
	ax[0].set_xlabel("training episode")
	ax[0].set_ylabel("deliveries (of 50)")
	ax[0].legend(fontsize=7)

	ax[1].set_title("Relay chain usage vs training length", fontsize=10)
	ax[1].set_xlabel("training episode")
	ax[1].set_ylabel("fraction of drone-steps at hop>=2")

	allh = [float(r["hop2plus"]) for rows in data.values() for r in rows]
	alld = [float(r["delivered"]) for rows in data.values() for r in rows]
	rho = np.corrcoef(allh, alld)[0, 1]
	ax[2].set_title(f"Relay usage vs deliveries (r = {rho:+.2f})", fontsize=10)
	ax[2].set_xlabel("fraction of drone-steps at hop>=2")
	ax[2].set_ylabel("deliveries (of 50)")

	for a in ax:
		a.grid(alpha=0.3)
	fig.suptitle("Held-out evaluation across checkpoints (8 unseen instances each)", fontsize=11)
	fig.tight_layout()
	out = os.path.join(FIG, f"holdout_sweep_{tag}.png")
	fig.savefig(out, dpi=140)
	print(f"저장: {out}")


if __name__ == "__main__":
	main()
