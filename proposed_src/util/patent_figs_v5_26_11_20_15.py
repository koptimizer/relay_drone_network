"""발명내용설명서 보완용 도면 생성 (도1–도5, 수식1) — docs/pptx/patent_*.png.

도1 시스템 구성도, 도2 장치 블록도, 도3 운영 방법 흐름도, 도4 중계 슬롯 기하와 연결성 투영, 도5 계층형 학습 제어기 구조, 수식1 핵심 수식.
참조 부호는 설명서 본문과 같다 (100 플랫폼, 110 통신·네트워크 인터페이스부, 120 프로세서/메모리부, 121–124 모듈, 130 DB부, 200 거점, 300 드론, 400 수요지, 500 관리자 단말, 600 외부 데이터).
"""

import glob
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch, Polygon, Rectangle

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUT = os.path.join(ROOT, "docs", "pptx")
TAG = "v5_26_11_20_15"
for f in glob.glob(os.path.expanduser("~/.fonts/NotoSansKR*.ttf")):
	fm.fontManager.addfont(f)
plt.rcParams.update({"font.family": "Noto Sans KR", "font.size": 10, "axes.unicode_minus": False, "mathtext.fontset": "dejavusans"})
INK, GREY, LIGHT = "#1a1a1a", "#666666", "#e8e8e8"


def box(ax, x, y, w, h, text, num=None, fs=10, fc="white", lw=1.2, round_=True, bold=False):
	"""참조 부호가 붙은 상자를 그린다."""
	st = "round,pad=0.02,rounding_size=0.15" if round_ else "square,pad=0.02"
	ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle=st, fc=fc, ec=INK, lw=lw))
	label = text if num is None else f"{text}\n({num})"
	ax.text(x + w / 2, y + h / 2, label, ha="center", va="center", fontsize=fs, color=INK, fontweight="bold" if bold else "normal", linespacing=1.3)


def arrow(ax, p, q, style="-|>", lw=1.2, ls="-", color=INK, shrink=2, cs="arc3,rad=0"):
	ax.add_patch(FancyArrowPatch(p, q, arrowstyle=style, mutation_scale=12, lw=lw, ls=ls, color=color, shrinkA=shrink, shrinkB=shrink, connectionstyle=cs))


def drone(ax, x, y, color, size=12, label=None, num=None, dy=-26):
	ax.plot(x, y, marker="X", ms=size, mec=INK, mfc=color, mew=0.8, zorder=5)
	if label:
		ax.annotate(f"{label}\n({num})" if num else label, (x, y), xytext=(0, dy), textcoords="offset points", ha="center", va="top" if dy < 0 else "bottom", fontsize=9)


def fig1():
	"""도1 — 드론 릴레이 네트워크 기반 광역 물류 시스템 구성도 (평면 개념도)."""
	fig, ax = plt.subplots(figsize=(10, 6.2))
	R = 300
	cc = np.array([150, 300])
	relay1, relay2, lead = np.array([420, 330]), np.array([690, 360]), np.array([935, 395])
	dest_far, dest_near = np.array([960, 400]), np.array([300, 520])
	for p in (cc, relay1, relay2, lead):
		ax.add_patch(Circle(p, R, fc="#dfe9f5", ec="#4a78b5", lw=0.8, ls="--", alpha=0.45, zorder=1))
	# 통신 음영 지역 (파손된 기지국)
	ax.add_patch(Polygon([[560, 60], [640, 40], [720, 70], [760, 160], [700, 230], [600, 220], [540, 150]], closed=True, fc="#d9d9d9", ec=GREY, hatch="///", lw=0.8, zorder=2))
	ax.text(650, 135, "통신 음영 지역\n(기지국 파손·험지)", ha="center", va="center", fontsize=9, color=INK, zorder=3)
	# 링크
	for a, b in ((cc, relay1), (relay1, relay2), (relay2, lead)):
		ax.plot([a[0], b[0]], [a[1], b[1]], color="#c0392b", lw=2.0, zorder=4)
	ax.text(285, 345, "1홉", fontsize=9, color="#c0392b"); ax.text(555, 370, "2홉", fontsize=9, color="#c0392b"); ax.text(812, 400, "3홉", fontsize=9, color="#c0392b")
	# 거점
	ax.add_patch(Rectangle(cc - [22, 22], 44, 44, fc="#f4d03f", ec=INK, lw=1.2, zorder=5))
	ax.text(cc[0], cc[1] - 40, "중앙 물류 센터·관제 거점\n(200)", ha="center", va="top", fontsize=9)
	# 드론
	drone(ax, *relay1, "#e67e22", label="중계 드론", num="320a")
	drone(ax, *relay2, "#e67e22", label="중계 드론", num="320b")
	drone(ax, *lead, "#2e86c1", label="배송 드론", num="310", dy=-30)
	drone(ax, 150, 470, "#2e86c1", label="배송 드론(근거리)", num="310'", dy=-18)
	# 수요지
	for d, lab in ((dest_far, "원거리 수요지 (400a)"), (dest_near, "수요지 (400b)"), (np.array([80, 120]), "수요지 (400c)"), (np.array([880, 150]), "수요지 (400d)")):
		ax.plot(*d, marker="^", ms=11, mec=INK, mfc="#27ae60", zorder=5)
		if lab.startswith("수요지 (400b)"):
			ax.text(d[0] + 18, d[1], lab, ha="left", va="center", fontsize=8.5)
		else:
			ax.text(d[0], d[1] + 16, lab, ha="center", va="bottom", fontsize=8.5)
	# 플랫폼·관리자
	box(ax, 20, 560, 260, 70, "광역 드론 물류 플랫폼\n(통신·네트워크 I/F, 프로세서/메모리, DB)", num="100", fs=8.5, fc="#fdfefe")
	box(ax, 320, 570, 150, 50, "관리자 단말", num="500", fs=9)
	arrow(ax, (150, 560), (150, 325), ls="--", color=GREY)
	arrow(ax, (280, 595), (320, 595), style="<|-|>", color=GREY)
	ax.annotate("제어 신호·상태 데이터\n(다중 홉 중계)", (560, 300), ha="center", fontsize=8.5, color="#c0392b")
	ax.annotate("통신 반경 R", (cc[0] + R * 0.72, cc[1] - R * 0.72), fontsize=8.5, color="#4a78b5")
	ax.set_xlim(-20, 1080); ax.set_ylim(0, 650); ax.set_aspect("equal"); ax.axis("off")
	ax.set_title("도 1. 드론 릴레이 네트워크를 이용한 광역 드론 물류 시스템의 구성도", fontsize=11, pad=8)
	fig.savefig(os.path.join(OUT, f"patent_fig1_{TAG}.png"), dpi=300, bbox_inches="tight"); plt.close(fig)


def fig2():
	"""도2 — 플랫폼(장치) 블록도."""
	fig, ax = plt.subplots(figsize=(11, 7.2))
	ax.add_patch(FancyBboxPatch((0.6, 0.4), 8.2, 6.2, boxstyle="round,pad=0.02,rounding_size=0.2", fc="#fbfbfb", ec=INK, lw=1.6))
	ax.text(0.8, 6.38, "광역 드론 물류 플랫폼 (100)", fontsize=11, fontweight="bold", va="center")
	box(ax, 0.9, 1.0, 1.6, 4.9, "통신 및\n네트워크\n인터페이스부", num="110", fs=10, fc="#eaf2fb")
	ax.text(1.7, 1.25, "드론 링크\n(다중 홉 중계)\n거점 링크\n외부 API", ha="center", va="bottom", fontsize=8, color=GREY)
	ax.add_patch(FancyBboxPatch((2.9, 1.0), 4.2, 4.9, boxstyle="round,pad=0.02,rounding_size=0.15", fc="#f4f9f1", ec=INK, lw=1.2))
	ax.text(5.0, 5.62, "프로세서 / 메모리부 (120)", ha="center", fontsize=10, fontweight="bold")
	box(ax, 3.1, 4.55, 1.85, 0.8, "전처리 모듈", num="121", fs=9)
	box(ax, 5.15, 4.55, 1.85, 0.8, "중계 통신 네트워크\n모델 모듈", num="122", fs=8.5)
	ax.add_patch(FancyBboxPatch((3.1, 1.85), 3.9, 2.45, boxstyle="round,pad=0.02,rounding_size=0.1", fc="white", ec=INK, lw=1.0))
	ax.text(5.05, 4.1, "릴레이 경로 최적화 모델 모듈 (123)", ha="center", fontsize=9, fontweight="bold")
	box(ax, 3.15, 3.05, 1.2, 0.85, "목표 선택\n정책부", num="123a", fs=7.6)
	box(ax, 4.5, 3.05, 1.2, 0.85, "중계 슬롯\n배정부", num="123b", fs=7.6)
	box(ax, 5.85, 3.05, 1.1, 0.85, "연결성\n투영기", num="123c", fs=7.6)
	box(ax, 3.55, 2.0, 1.45, 0.8, "롤아웃 선견\n평가기", num="123d", fs=7.6)
	box(ax, 5.15, 2.0, 1.45, 0.8, "교착 복구부", num="123e", fs=7.6)
	arrow(ax, (4.35, 3.47), (4.5, 3.47)); arrow(ax, (5.7, 3.47), (5.85, 3.47))
	arrow(ax, (4.27, 2.8), (3.75, 3.05), color=GREY); arrow(ax, (5.87, 2.8), (3.95, 3.05), color=GREY, cs="arc3,rad=0.15")
	box(ax, 3.1, 1.1, 3.9, 0.6, "관리 모니터링 모듈 (대시보드·임계치 알림)", num="124", fs=8.5)
	box(ax, 7.3, 1.0, 1.35, 4.9, "데이터\n베이스부", num="130", fs=10, fc="#fdf2e9")
	ax.text(7.97, 1.25, "드론 상태\n거점·물동량\n기상·지형\n배송 이력\n모델 가중치", ha="center", va="bottom", fontsize=7.8, color=GREY)
	arrow(ax, (2.5, 3.5), (2.9, 3.5), style="<|-|>"); arrow(ax, (7.1, 3.5), (7.3, 3.5), style="<|-|>")
	arrow(ax, (4.0, 4.55), (4.0, 4.3)); arrow(ax, (6.0, 4.55), (6.0, 4.3))
	arrow(ax, (5.05, 1.85), (5.05, 1.7))
	# 외부 요소
	box(ax, 9.3, 5.0, 1.6, 0.9, "드론군\n(300)", fs=9.5, fc="#eef7ff")
	box(ax, 9.3, 3.6, 1.6, 0.9, "물류 센터·거점\n(200)", fs=9, fc="#fff9e6")
	box(ax, 9.3, 2.2, 1.6, 0.9, "외부 데이터\n(기상·지형 API)\n(600)", fs=8, fc="#f3f3f3")
	box(ax, 9.3, 0.8, 1.6, 0.9, "관리자 단말\n(500)", fs=9.5, fc="#f3f3f3")
	for y in (5.45, 4.05, 2.65, 1.25):
		arrow(ax, (8.8, y if y < 3 else 3.55 + (y - 3.55) * 0.0 + (y - 3.55)), (9.3, y), style="<|-|>" if y != 2.65 else "-|>", cs="arc3,rad=0.0")
	ax.text(9.3, 6.1, "외부 구성 요소", fontsize=9, color=GREY)
	ax.set_xlim(0.4, 11.1); ax.set_ylim(0.3, 6.8); ax.axis("off")
	ax.set_title("도 2. 광역 드론 물류 플랫폼(장치)의 블록도", fontsize=11, pad=6)
	fig.savefig(os.path.join(OUT, f"patent_fig2_{TAG}.png"), dpi=300, bbox_inches="tight"); plt.close(fig)


def fig3():
	"""도3 — 운영 방법 흐름도."""
	fig, ax = plt.subplots(figsize=(8.5, 12.5))
	steps = [("S110", "데이터 수집\n(드론 상태·거점 물동량·기상/지형)"), ("S120", "전처리·상태 갱신\n(결측 보정, 좌표 통일, 특징 생성)"),
	         ("S130", "통신 그래프 $G_t$ 구성, 거점까지의 홉 깊이 계산\n(BFS, 반경 R 이내 간선)"),
	         ("S140", "결정 시점인가?\n(주기 $H$ 스텝 또는 목표 무효)"),
	         ("S150", "드론별 옵션 유효성 마스크 계산\n(후보 수요지 k개·복귀·중계)"),
	         ("S160", "목표 선택: 학습 관리자(자기회귀) / 규칙 / 롤아웃 선견"),
	         ("S170", "중계 슬롯 배정 (전선까지의 선분 위 등간격)"),
	         ("S180", "하위 비행 명령 (목표 방향 직진, 서비스 중 정지)"),
	         ("S190", "연결성 투영 (연결이 끊기는 변위를 이분 탐색으로 축소)"),
	         ("S200", "상태 전이·서비스 (하역 S스텝, 거점 재적재)"),
	         ("S210", "정체·부분 교착 감시 → 임계 초과 시 복구 기동"),
	         ("S220", "모니터링·알림 (연결 상태, 중계 부하, 단절 예측)"),
	         ("S230", "모든 수요지 배송 완료?")]
	y0, dy, w, h = 11.6, 0.9, 6.4, 0.62
	ys = {}
	for i, (num, txt) in enumerate(steps):
		y = y0 - i * dy
		ys[num] = y
		if num in ("S140", "S230"):
			ax.add_patch(Polygon([[1.1, y + h / 2], [1.1 + w / 2, y + h / 2 + 0.42], [1.1 + w, y + h / 2], [1.1 + w / 2, y + h / 2 - 0.42]], closed=True, fc="#fff9e6", ec=INK, lw=1.2))
			ax.text(1.1 + w / 2, y + h / 2, txt, ha="center", va="center", fontsize=8.8)
		else:
			ax.add_patch(FancyBboxPatch((1.1, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.1", fc="white", ec=INK, lw=1.2))
			ax.text(1.1 + w / 2, y + h / 2, txt, ha="center", va="center", fontsize=8.8)
		ax.text(0.95, y + h / 2 + (0.32 if num in ("S140", "S230") else 0.0), num, ha="right", va="center", fontsize=9, fontweight="bold")
		if i > 0:
			ax.add_patch(FancyArrowPatch((1.1 + w / 2, y0 - (i - 1) * dy), (1.1 + w / 2, y + h), arrowstyle="-|>", mutation_scale=12, lw=1.1, color=INK, shrinkA=0, shrinkB=0))
	# 분기: S140 아니오 → S180
	xr = 1.1 + w + 0.35
	ax.plot([1.1 + w, xr], [ys["S140"] + h / 2] * 2, color=INK, lw=1.1)
	ax.plot([xr, xr], [ys["S140"] + h / 2, ys["S180"] + h / 2], color=INK, lw=1.1)
	ax.add_patch(FancyArrowPatch((xr, ys["S180"] + h / 2), (1.1 + w, ys["S180"] + h / 2), arrowstyle="-|>", mutation_scale=12, lw=1.1, color=INK, shrinkA=0, shrinkB=0))
	ax.text(xr + 0.08, ys["S140"] + h / 2 + 0.1, "아니오\n(이전 목표 유지)", fontsize=8, va="bottom")
	ax.text(1.1 + w / 2 + 0.1, ys["S140"] - 0.2, "예", fontsize=8.5)
	# S230 아니오 → S110 (왼쪽 루프)
	xl = 0.35
	ax.plot([1.1, xl], [ys["S230"] + h / 2] * 2, color=INK, lw=1.1)
	ax.plot([xl, xl], [ys["S230"] + h / 2, ys["S110"] + h * 0.85], color=INK, lw=1.1)
	ax.add_patch(FancyArrowPatch((xl, ys["S110"] + h * 0.85), (1.1, ys["S110"] + h * 0.85), arrowstyle="-|>", mutation_scale=12, lw=1.1, color=INK, shrinkA=0, shrinkB=0))
	ax.text(xl - 0.06, (ys["S230"] + ys["S110"]) / 2, "아니오 (다음 스텝으로)", fontsize=8, va="center", ha="right", rotation=90)
	# 종료
	ax.add_patch(FancyBboxPatch((1.1 + w / 2 - 1.1, ys["S230"] - 0.95), 2.2, 0.55, boxstyle="round,pad=0.02,rounding_size=0.3", fc=LIGHT, ec=INK, lw=1.2))
	ax.text(1.1 + w / 2, ys["S230"] - 0.675, "종료 (makespan 기록)", ha="center", va="center", fontsize=9)
	ax.add_patch(FancyArrowPatch((1.1 + w / 2, ys["S230"] + h / 2 - 0.42), (1.1 + w / 2, ys["S230"] - 0.4), arrowstyle="-|>", mutation_scale=12, lw=1.1, color=INK, shrinkA=0, shrinkB=0))
	ax.text(1.1 + w / 2 + 0.1, ys["S230"] - 0.2, "예", fontsize=8.5)
	# 시작
	ax.add_patch(FancyBboxPatch((1.1 + w / 2 - 0.8, y0 + h + 0.35), 1.6, 0.5, boxstyle="round,pad=0.02,rounding_size=0.3", fc=LIGHT, ec=INK, lw=1.2))
	ax.text(1.1 + w / 2, y0 + h + 0.6, "시작 (임무 적재)", ha="center", va="center", fontsize=9)
	ax.add_patch(FancyArrowPatch((1.1 + w / 2, y0 + h + 0.35), (1.1 + w / 2, y0 + h), arrowstyle="-|>", mutation_scale=12, lw=1.1, color=INK, shrinkA=0, shrinkB=0))
	ax.set_xlim(0, 9.6); ax.set_ylim(ys["S230"] - 1.2, y0 + h + 1.1); ax.axis("off")
	ax.set_title("도 3. 드론 릴레이 네트워크 운영 방법의 흐름도", fontsize=11, pad=4)
	fig.savefig(os.path.join(OUT, f"patent_fig3_{TAG}.png"), dpi=300, bbox_inches="tight"); plt.close(fig)


def fig4():
	"""도4 — (a) 중계 슬롯 기하, (b) 연결성 투영."""
	fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 5.6), gridspec_kw=dict(width_ratios=[1.35, 1]))
	R, beta = 300, 0.9
	# (a) 슬롯
	cc = np.array([0.0, 0.0]); f = np.array([900.0, 130.0])
	d = np.linalg.norm(f - cc); n = int(np.ceil(d / (beta * R))) - 1
	a1.plot([cc[0], f[0]], [cc[1], f[1]], color=GREY, lw=1.0, ls="--", zorder=3)
	a1.add_patch(Circle(cc, R, fc="#dfe9f5", ec="#4a78b5", lw=0.8, ls="--", alpha=0.35))
	a1.add_patch(Rectangle(cc - [18, 18], 36, 36, fc="#f4d03f", ec=INK, lw=1.2, zorder=5))
	a1.text(cc[0], cc[1] - 45, "거점 c (200)", ha="center", va="top", fontsize=9)
	a1.plot(*f, marker="^", ms=12, mec=INK, mfc="#27ae60", zorder=5); a1.text(f[0], f[1] + 28, "전선 f\n(선두의 목표, 400a)", ha="center", va="bottom", fontsize=9)
	for k in range(1, n + 1):
		sk = cc + (k / (n + 1)) * (f - cc)
		a1.add_patch(Circle(sk, R, fc="#dfe9f5", ec="#4a78b5", lw=0.8, ls="--", alpha=0.35))
		a1.plot(*sk, marker="o", ms=13, mfc="white", mec="#c0392b", mew=1.6, zorder=6)
		a1.text(sk[0], sk[1] - 40, f"슬롯 $s_{k}$", ha="center", va="top", fontsize=9, color="#c0392b")
	s1 = cc + (1 / (n + 1)) * (f - cc)
	drone(a1, s1[0] - 30, s1[1] + 130, "#e67e22")
	a1.annotate("", xy=s1 + [-4, 16], xytext=(s1[0] - 26, s1[1] + 115), arrowprops=dict(arrowstyle="-|>", color="#e67e22", lw=1.5))
	a1.text(s1[0] - 250, s1[1] + 170, "중계 드론 (320):\n가장 가까운 빈 슬롯으로", ha="left", va="bottom", fontsize=9)
	a1.text(450, -215, f"$n=\\min(\\lceil d/(\\beta R)\\rceil-1,\\ N-1)={n}$   (d = {d:.0f} m, R = {R} m, β = {beta}),      $s_k=c+\\frac{{k}}{{n+1}}(f-c)$", ha="center", va="top", fontsize=9.5)
	a1.set_xlim(-340, 1240); a1.set_ylim(-320, 430); a1.set_aspect("equal"); a1.axis("off")
	a1.set_title("(a) 중계 슬롯의 기하", fontsize=10)
	# (b) 투영
	pj = np.array([0.0, 0.0]); pi = np.array([200.0, -40.0])
	a2.add_patch(Circle(pj, R, fc="#dfe9f5", ec="#4a78b5", lw=1.0, ls="--", alpha=0.5))
	a2.text(-120, 255, "드론 j 의 통신 반경 R", fontsize=9, color="#4a78b5")
	drone(a2, *pj, "#e67e22", label="드론 j (중계, 정지)", dy=-18)
	drone(a2, *pi, "#2e86c1", label="드론 i", dy=-18)
	u = np.array([0.95, 0.31]); u = u / np.linalg.norm(u); D = 140.0
	want = pi + D * u
	lam = max(l for l in np.linspace(0, 1, 4001) if np.linalg.norm(pi + l * D * u - pj) <= R)
	got = pi + lam * D * u
	a2.annotate("", xy=want, xytext=pi, arrowprops=dict(arrowstyle="-|>", color=GREY, lw=1.5, ls="--"))
	a2.plot(*want, marker="X", ms=11, mfc="white", mec=GREY, mew=1.2)
	a2.annotate("명령 변위 $\\Delta_i$ (과장)\n→ 거리 > R, 연결 끊김", xy=want, xytext=(300, 150), fontsize=8.5, color=GREY, arrowprops=dict(arrowstyle="-", color=GREY, lw=0.8))
	a2.annotate("", xy=got, xytext=pi, arrowprops=dict(arrowstyle="-|>", color="#c0392b", lw=2.2))
	a2.plot(*got, marker="X", ms=11, mfc="#2e86c1", mec=INK, mew=0.8, zorder=6)
	a2.annotate("투영 변위 $\\lambda^{*}\\Delta_i$\n→ 거리 = R 에서 멈춤", xy=got, xytext=(290, -190), fontsize=8.5, color="#c0392b", arrowprops=dict(arrowstyle="-", color="#c0392b", lw=0.8))
	a2.plot([pj[0], got[0]], [pj[1], got[1]], color="#c0392b", lw=1.0, ls=":")
	a2.text(-310, -300, "$\\lambda^{*}=\\max\\{\\lambda\\in[0,1]:\\ G(p+\\lambda\\Delta)\\ \\mathrm{connected}\\}$\n이분 탐색, 다른 드론의 배율은 고정. 직전 위치는 항상 연결 상태이므로 λ=0 은 늘 실행 가능", ha="left", va="bottom", fontsize=8.8)
	a2.set_xlim(-330, 560); a2.set_ylim(-320, 330); a2.set_aspect("equal"); a2.axis("off")
	a2.set_title("(b) 연결성 투영에 의한 이동 제한", fontsize=10)
	fig.suptitle("도 4. 중계 슬롯의 기하와 연결성 투영", fontsize=11, y=0.98)
	fig.savefig(os.path.join(OUT, f"patent_fig4_{TAG}.png"), dpi=300, bbox_inches="tight"); plt.close(fig)


def fig5():
	"""도5 — 계층형 학습 제어기와 학습 구조."""
	fig, ax = plt.subplots(figsize=(12.5, 7.4))
	# 상위 관리자
	ax.add_patch(FancyBboxPatch((0.3, 3.3), 7.6, 3.9, boxstyle="round,pad=0.02,rounding_size=0.2", fc="#f4f9f1", ec=INK, lw=1.4))
	ax.text(0.5, 7.0, "상위: 목표 선택 정책 (학습 관리자, 123a) — 결정 시점마다 드론별 옵션 1개", fontsize=10, fontweight="bold", va="center")
	cols = [0.75, 1.95, 3.15, 4.35]; w = 1.05
	ents = ["동료 드론\n(N−1 × 9 특징)", "자기 상태\n(10 특징)", "후보 수요지\n(k=5 × 6 특징)", "중계 슬롯\n(4 특징)"]
	for x, t in zip(cols, ents):
		box(ax, x, 6.0, w, 0.75, t, fs=7.8, fc="#eaf2fb")
		box(ax, x, 5.15, w, 0.5, "MLP (128×2)", fs=7.8)
		arrow(ax, (x + w / 2, 6.0), (x + w / 2, 5.65))
		ax.plot([x + w / 2, x + w / 2], [5.15, 4.9], color=INK, lw=1.1)
	ax.plot([cols[0] + w / 2, 5.25], [4.9, 4.9], color=INK, lw=1.1)
	arrow(ax, (5.25, 4.9), (5.6, 5.3))
	box(ax, 5.6, 4.9, 2.1, 1.85, "어텐션 풀링\n(질의 = 자기 임베딩,\n4 헤드 × 2 층)\n+ 융합 MLP → $h_i$", fs=8.3, fc="#fff9e6")
	box(ax, 5.3, 3.45, 2.4, 0.75, "포인터 헤드: 후보 5개 점수 $[h_i; e_{cand}]$\n고정 헤드: 복귀·중계 점수", fs=7.6)
	box(ax, 3.0, 3.45, 2.1, 0.75, "유효성 마스크 →\n소프트맥스 (7 옵션)", fs=7.8, fc="#fff9e6")
	box(ax, 0.75, 3.45, 2.05, 0.75, "자기회귀 결정\n(거점에서 먼 드론부터)", fs=7.8)
	arrow(ax, (6.65, 4.9), (6.5, 4.2)); arrow(ax, (5.3, 3.82), (5.1, 3.82)); arrow(ax, (3.0, 3.82), (2.8, 3.82))
	ax.plot([0.75, 0.5, 0.5, 0.62], [3.82, 3.82, 6.37, 6.37], color=GREY, lw=1.0)
	arrow(ax, (0.62, 6.37), (0.75, 6.37), color=GREY)
	ax.text(0.45, 5.0, "선택 역할을 동료 특징에 기록", fontsize=7, color=GREY, rotation=90, va="center", ha="right")
	# 하위
	ax.add_patch(FancyBboxPatch((0.3, 0.6), 7.6, 2.1, boxstyle="round,pad=0.02,rounding_size=0.2", fc="#fbfbfb", ec=INK, lw=1.4))
	ax.text(0.5, 2.5, "하위: 매 스텝 (학습 없음)", fontsize=10, fontweight="bold", va="center")
	box(ax, 0.5, 0.9, 2.1, 1.1, "중계 슬롯 배정부 (123b) +\n목표 방향 직진 비행\n(서비스 중 정지)", fs=8.3)
	box(ax, 2.9, 0.9, 2.1, 1.1, "연결성 투영기 (123c)\n$G_t$ 연결 유지,\n이분 탐색 배율", fs=8.5, fc="#fdf2e9")
	box(ax, 5.3, 0.9, 2.3, 1.1, "환경 전이 / 드론군\n(이동, 하역, 재적재,\n정체 감시)", fs=8.5)
	arrow(ax, (2.6, 1.45), (2.9, 1.45)); arrow(ax, (5.0, 1.45), (5.3, 1.45))
	arrow(ax, (1.775, 3.45), (1.775, 2.7), lw=1.6); ax.text(1.9, 3.05, "옵션 (H = 20 스텝 유지)", fontsize=8.5)
	# 관측·전이 되먹임
	ax.plot([7.6, 8.25, 8.25], [1.45, 1.45, 6.4], color=INK, lw=1.2); arrow(ax, (8.25, 6.4), (7.9, 6.4))
	ax.text(8.17, 4.3, "관측 (상태 갱신)", fontsize=8, rotation=90, va="center", ha="right")
	ax.plot([7.6, 8.6, 8.6], [1.6, 1.6, 6.22], color=GREY, lw=1.1); arrow(ax, (8.6, 6.22), (9.0, 6.22), color=GREY)
	ax.text(8.7, 4.3, "전이 (관측·옵션·수익)", fontsize=8, rotation=90, va="center", ha="left", color=GREY)
	# 학습 (우측)
	ax.add_patch(FancyBboxPatch((9.0, 0.6), 3.3, 6.6, boxstyle="round,pad=0.02,rounding_size=0.2", fc="#f3f3f3", ec=INK, lw=1.4))
	ax.text(9.2, 7.0, "학습 (중앙 집중 학습·분산 실행)", fontsize=10, fontweight="bold", va="center")
	box(ax, 9.2, 5.75, 2.9, 0.95, "재생 버퍼 (옵션 전이)\n관측·옵션·누적 수익·$\\gamma^{\\Delta t}$", fs=8)
	box(ax, 9.2, 4.55, 2.9, 0.95, "twin-Q critic\n(드론 임베딩 간 어텐션,\n팀 보상 회귀)", fs=8, fc="#fdf2e9")
	box(ax, 9.2, 3.35, 2.9, 0.95, "이산 SAC actor 갱신\n$\\alpha\\log\\pi - Q$ + 엔트로피 목표 감쇠\n+ 규칙 모방 CE 항", fs=7.6)
	box(ax, 9.2, 2.15, 2.9, 0.95, "검증·조기 종료\n(홀드아웃 80 인스턴스, 인내 8)", fs=8)
	box(ax, 9.2, 0.85, 2.9, 1.05, "선택적 증류 (DAgger):\n롤아웃 선견 교사(123d)의 라벨로\n관리자 미세조정", fs=7.6, fc="#eaf2fb")
	for y in (5.75, 4.55, 3.35, 2.15):
		arrow(ax, (10.65, y), (10.65, y - 0.25))
	ax.set_xlim(0.1, 12.5); ax.set_ylim(0.4, 7.5); ax.axis("off")
	ax.set_title("도 5. 계층형 학습 제어기의 구조와 학습 흐름", fontsize=11, pad=6)
	fig.savefig(os.path.join(OUT, f"patent_fig5_{TAG}.png"), dpi=300, bbox_inches="tight"); plt.close(fig)


def eq1():
	"""수식1 — 연결성, 목적, 투영, 슬롯, 보상, 참조 모형 핵심식 (mathtext: \\leq, \\geq 사용, 한글은 수식 밖)."""
	fig, ax = plt.subplots(figsize=(11, 9.6)); ax.axis("off")
	lines = [
		("(1) 통신 그래프와 연결성", r"$G_t=(V,E_t),\ \ V=\{c\}\cup\{1,\dots,N\},\ \ E_t=\{(a,b):\ \|p_a(t)-p_b(t)\|\leq R\}$", "모든 t 에서 G_t 가 연결될 것 (모든 드론이 거점 c 까지 다중 홉 경로를 가짐)"),
		("(2) 목적 (배송 완료 시간 최소화)", r"$\min\ C,\qquad C=\max_{j\in\{1,\dots,M\}} t_j^{\mathrm{deliv}}$", "마지막 수요지의 배송 완료 스텝. 모든 수요지 배송과 (1)의 연결성 제약 아래"),
		("(3) 중계 슬롯의 수와 위치", r"$n=\min\left(\left\lceil \frac{\|f-c\|}{\beta R}\right\rceil-1,\ N-1\right),\qquad s_k=c+\frac{k}{n+1}(f-c),\ \ k=1,\dots,n,\qquad \beta=0.9$", "f: 전선(선두의 목표 또는 가장 먼 비중계 드론 위치), β: 슬롯 여유 계수"),
		("(4) 연결성 투영 (드론 m 의 변위 축소)", r"$\lambda_m^{*}=\max\{\lambda\in[0,1]:\ G(p+\lambda\Delta_m)\ \mathrm{connected}\},\qquad p_m(t+1)=p_m(t)+\lambda_m^{*}\Delta_m,\ \ \|\Delta_m\|\leq v$", "다른 드론의 배율은 고정한 채 이분 탐색. 직전 위치는 항상 연결 상태이므로 λ=0 이 항상 실행 가능"),
		("(5) 팀 보상 (학습용) 과 옵션 수익", r"$r_t=10\,n_t^{\mathrm{deliv}}+30\,n_t^{\mathrm{reload}}-0.01N-0.1N\cdot u_t/M,\qquad R^{\mathrm{opt}}_t=\sum_{\tau=t}^{t+\Delta t-1}\gamma^{\tau-t}r_\tau,\ \ \gamma=0.997$", "u_t: 미배송 수요지 수, Δt: 옵션 유지 스텝 수(결정 시점 간격)"),
		("(6) 정체·부분 교착 판정과 복구 발동", r"$\rho_i(t)=\frac{\langle\Delta_i^{\mathrm{eff}},\Delta_i^{\mathrm{cmd}}\rangle}{\|\Delta_i^{\mathrm{cmd}}\|^{2}},\qquad \mathrm{stall}_i\Leftrightarrow\rho_i<0.1,\qquad \delta_{t+1}=\delta_t+1\ \mathrm{if}\ |\{i:\mathrm{moving},\ \mathrm{not\ stall}_i\}|\leq 1\ \mathrm{else}\ 0,\qquad \delta_t\geq k_{stall}\Rightarrow\mathrm{recovery}$", "ρ_i: 실현 이동 비율(명령 변위 대비 실제 변위), δ_t: 연속 부분 교착 스텝 수. 복구는 h_hold 회의 결정 시점 동안 선두 교대 또는 후퇴로 옵션을 덮어쓴다 (일 실시예 k_stall=120, h_hold=20)"),
		("(7) 모니터링 지표 (관리 모니터링 모듈)", r"$m_{ab}(t)=R-d_{ab}(t),\qquad \tau_{ab}(t)=\frac{R-d_{ab}(t)}{\max(\dot d_{ab}(t),\varepsilon)},\qquad \ell_i(t)=W^{-1}\,\Sigma_{\tau=t-W+1}^{\,t}\,\mathbf{1}[\mathrm{role}_i(\tau)=\mathrm{relay}],\qquad h_i(t)=\mathrm{BFS\ depth}$", "m: 링크 여유, τ: 단절 예상 시간(상대 속도로 멀어질 때), ℓ: 창 W 에서의 중계 부하, h: 거점까지의 홉 깊이. 임계치 초과 시 중계 보강·체공·역할 교대·복구 알림"),
	]
	y = 0.985
	for title, expr, note in lines:
		if title:
			ax.text(0.01, y, title, fontsize=10.5, fontweight="bold", va="top"); y -= 0.04
		ax.text(0.03, y, expr, fontsize=10.5, va="top"); y -= 0.05
		ax.text(0.05, y, note, fontsize=8.8, va="top", color=GREY); y -= 0.045
	ax.text(0.01, 0.01, "기호: p_i(t) 드론 i 의 위치, c 거점, R 통신 반경, v 스텝당 최대 이동 거리, N 드론 수, M 수요지 수, Δ 변위, d_ab 두 정점 사이 거리, k_stall·h_hold 복구 임계치와 지속 횟수, W 모니터링 창 길이.", fontsize=8.5, va="bottom", color=GREY)
	ax.set_title("수식 1. 핵심 수식 (연결성·목적·슬롯·투영·보상·교착 판정·모니터링 지표)", fontsize=11)
	fig.savefig(os.path.join(OUT, f"patent_eq1_{TAG}.png"), dpi=300, bbox_inches="tight"); plt.close(fig)


if __name__ == "__main__":
	import sys
	only = sys.argv[1:] or ["fig1", "fig2", "fig3", "fig4", "fig5", "eq1"]
	for name in only:
		globals()[name]()
	print("saved:", sorted(os.path.basename(f) for f in glob.glob(os.path.join(OUT, f"patent_*_{TAG}.png"))))
