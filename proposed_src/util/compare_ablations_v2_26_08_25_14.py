"""격리 실험 실행들을 홀드아웃 평가 로그 기준으로 비교한다.

각 실행은 기준선 대비 한 가지 조건만 다르므로, 최고 성능·정점 도달 시점·홉 지표 유지 여부를
나란히 놓으면 어느 가설이 지지되는지 읽을 수 있다.
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
	"base": "baseline (v1 setup)",
	"e1_alpha": "E1: alpha floor 0.05",
	"e2_relay": "E2: relay-weighted credit",
	"e3_obs": "E3: observe all 3 peers",
}


def load(tag, prefix):
	"""실행의 홀드아웃 평가 로그를 읽는다."""
	p = os.path.join(ROOT, "runs", f"{prefix}_{tag}", f"evals_{prefix}_{tag}.csv")
	if not os.path.exists(p):
		return None
	rows = list(csv.DictReader(open(p, encoding="utf-8")))
	return rows or None


def summarize(tag, rows):
	"""최고 성능과 그 시점, 마지막 평가값, 홉 지표를 요약한다."""
	dl = [float(r["delivered"]) for r in rows]
	i = int(np.argmax(dl))
	return {
		"run": LABEL.get(tag, tag),
		"평가횟수": len(rows),
		"최고배송": f'{dl[i]:.1f}±{float(rows[i]["delivered_sd"]):.1f}',
		"정점ep": rows[i]["episode"],
		"정점 2홉+": f'{float(rows[i]["hop2plus"]):.2f}',
		"정점 도달": f'{float(rows[i]["max_reach"]):.0f}',
		"정점 미활동": f'{float(rows[i]["idle_drones"]):.2f}',
		"최종배송": f'{dl[-1]:.1f}',
		"최종 2홉+": f'{float(rows[-1]["hop2plus"]):.2f}',
		"단절": sum(int(r["comm_loss"]) for r in rows),
	}


def main():
	"""요약 표를 출력하고 홀드아웃 추이 그래프를 저장한다."""
	p = argparse.ArgumentParser()
	p.add_argument("--prefix", default="v2_26_08_25_14")
	p.add_argument("--tags", nargs="*", default=["base", "e1_alpha", "e2_relay", "e3_obs"])
	args = p.parse_args()

	data = {t: load(t, args.prefix) for t in args.tags}
	data = {t: v for t, v in data.items() if v}
	if not data:
		print("아직 홀드아웃 평가 기록이 없습니다.")
		return

	summ = [summarize(t, r) for t, r in data.items()]
	cols = list(summ[0].keys())
	w = {c: max(len(c), max(len(str(s[c])) for s in summ)) for c in cols}
	print(" | ".join(c.ljust(w[c]) for c in cols))
	print("-" * (sum(w.values()) + 3 * len(cols)))
	for s in summ:
		print(" | ".join(str(s[c]).ljust(w[c]) for c in cols))

	fig, ax = plt.subplots(1, 3, figsize=(13, 3.8))
	for t, rows in data.items():
		ep = [int(r["episode"]) for r in rows]
		ax[0].plot(ep, [float(r["delivered"]) for r in rows], lw=1.5, marker="o",
		           ms=3, label=LABEL.get(t, t))
		ax[1].plot(ep, [float(r["hop2plus"]) for r in rows], lw=1.5, marker="o", ms=3)
		ax[2].plot(ep, [float(r["idle_drones"]) for r in rows], lw=1.5, marker="o", ms=3)
	for a, t, yl in [(ax[0], "Held-out deliveries", "of 50"),
	                 (ax[1], "Relay chain usage", "fraction at hop>=2"),
	                 (ax[2], "Idle drones", "count of 4")]:
		a.set_title(t, fontsize=10)
		a.set_xlabel("training episode")
		a.set_ylabel(yl, fontsize=8)
		a.grid(alpha=0.3)
	ax[0].legend(fontsize=7)
	fig.suptitle("v2 step 2 — collapse-cause isolation (one condition changed each)", fontsize=11)
	fig.tight_layout()
	os.makedirs(FIG, exist_ok=True)
	out = os.path.join(FIG, f"ablation_{args.prefix}.png")
	fig.savefig(out, dpi=140)
	print(f"\n저장: {out}")


if __name__ == "__main__":
	main()
