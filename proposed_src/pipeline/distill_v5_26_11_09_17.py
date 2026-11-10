"""선견 규칙 증류 (DAgger, 사이클 32, 26-11-09).

교사 = 규칙 위의 한 수 롤아웃 선견을 매 결정 시점마다 하는 정책 (후보: 규칙 배정 + 드론 하나를 가까운 후보 k 곳으로
바꾼 배정, 각 후보를 규칙으로 끝까지 굴려 가장 짧은 것). 중계 선행 토글은 정책의 옵션으로 표현할 수 없으므로 뺀다.
학생 = 제안 정책(SWa)과 같은 구조의 집합 상위 정책, SWa 가중치에서 시작.

사이클 31 과 다른 점: 선견 라벨을 SAC 의 정규화 항으로 넣지 않고, 강화학습 갱신 없이 교사 정책 전체를 지도학습으로
흉내 낸다 (교차 엔트로피, 자기회귀 교사 강제). 반복 i 에서 학생(확률 1-beta_i)과 교사(beta_i)가 섞여 굴린 궤적의
모든 결정 시점에 교사 라벨을 달아 모으고(DAgger), 모은 전체로 다시 학습한다. 반복마다 쌍별 검증(val_paired)으로
최고 가중치를 고른다 (관문 지표 = 최종 지표).
"""

import argparse
import copy
import json
import multiprocessing as mp
import os
import sys
import time

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv, N_MAX
from model.hier_net_v5_26_09_15_23 import SetManagerActor
from pipeline.common_v5_26_09_15_22 import action_mask, chain_escape_manager, straight_worker
from util.instance_generator_v5_26_09_15_22 import sample_config

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
K, HL = 7, 20


def pad_obs(o, n):
	"""집합 관측을 N_MAX 드론으로 패딩하고 drone_mask 를 붙인다 (학습 스크립트와 같다)."""
	C = o["cand"].shape[1]
	out = dict(self=np.zeros((N_MAX, 10), np.float32), cand=np.zeros((N_MAX, C, 6), np.float32),
	           cand_mask=np.zeros((N_MAX, C), bool), peer=np.zeros((N_MAX, N_MAX, 9), np.float32),
	           peer_mask=np.zeros((N_MAX, N_MAX), bool), relay=np.zeros((N_MAX, 4), np.float32),
	           drone_mask=np.zeros(N_MAX, bool))
	out["self"][:n], out["cand"][:n], out["cand_mask"][:n] = o["self"], o["cand"], o["cand_mask"]
	out["peer"][:n, :n], out["peer_mask"][:n, :n], out["relay"][:n] = o["peer"], o["peer_mask"], o["relay"]
	out["drone_mask"][:n] = True
	return out


def padded_mask(env):
	m = action_mask(env, torch.device("cpu")).numpy()
	mp_ = np.zeros((N_MAX, K), bool)
	mp_[:env.num_drones] = m
	mp_[env.num_drones:, K - 2] = True
	return mp_


def rule_value(env, horizon):
	"""현재 상태에서 규칙(+복구)으로 끝까지 (또는 horizon 스텝까지) 굴린 값 (미완주면 상한 + 50 x 남은 수)."""
	base = chain_escape_manager()
	t0, done = env.current_step, False
	while not done and env.current_step < min(env.max_steps, t0 + horizon):
		_o, _w, _r, done = env.step(straight_worker(env))
		if env.current_step % HL == 0 or env.goal_invalid().any():
			env.set_goals(base(env))
	if env.makespan is not None:
		return float(env.makespan)
	return float(env.current_step) + 50.0 * float(env.dests_active.sum())


def expert_label(env, base, k_cand, horizon, margin=0.0, reps=1):
	"""교사의 옵션 라벨 (드론별 0..K-1): 규칙 배정과 드론 하나를 가까운 후보로 바꾼 배정 중 규칙 롤아웃이 가장 짧은 것.

	reps > 1 이면 후보마다 공통 난수 reps 회로 굴려 평균하고, 규칙 배정보다 margin 스텝 넘게 짧을 때만 그 후보를 고른다
	(그 밖은 규칙 배정). 같은 상태에서 동률 깨기 난수만 바꿔도 단일 롤아웃 라벨은 33%가 바뀌고, 이탈 이득은 중앙
	4스텝이다 (26-11-09 측정) — 라벨 대부분이 잡음이라 DA2 부터 정제한다.
	"""
	N = env.num_drones
	acts0 = np.asarray(base(env)).astype(np.int64).copy()
	cand = [np.asarray(env.candidates(i)) for i in range(N)]
	d0 = [int(cand[i][a]) if 0 <= a < env.n_cand else -1 for i, a in enumerate(acts0)]
	opts = [acts0]
	for i in range(N):
		if env.drones_capacity[i] == 0:
			continue
		for c in range(min(k_cand, env.n_cand)):
			j = int(cand[i][c])
			if j < 0 or j == d0[i] or j in d0:
				continue
			a = acts0.copy(); a[i] = c
			opts.append(a)
	vals = []
	seeds = [int(env.rng.integers(1 << 31)) for _ in range(reps)] if reps > 1 else [None]
	for a in opts:
		vs = []
		for sd in seeds:
			e = copy.deepcopy(env)
			if sd is not None:
				e.rng = np.random.default_rng(sd)
			e.set_goals(a)
			vs.append(rule_value(e, horizon))
		vals.append(float(np.mean(vs)))
	best = int(np.argmin(vals))
	if vals[0] - vals[best] <= margin:
		best = 0
	return opts[best], len(opts)


def episode(args):
	"""작업자: 학생·교사 혼합으로 한 에피소드를 굴리며 모든 결정 시점에 교사 라벨을 단다."""
	seed, wpath, beta, k_cand, horizon, max_steps = args[:6]
	margin, reps = (args[6], args[7]) if len(args) >= 8 else (0.0, 1)
	drones, dests = (args[8], args[9]) if len(args) >= 10 else ((3, 4), (20, 80))
	torch.set_num_threads(1)
	rng = np.random.default_rng(seed)
	np.random.seed(seed % (2 ** 31))
	torch.manual_seed(seed)
	n, m, cc, d = sample_config(rng, drones, dests, random_cc=True)
	env = DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=max_steps,
	                            deadlock_limit=10 ** 9, no_progress_limit=10 ** 9, num_drones=n, num_dests=m)
	env.cc_pos, env.dests_pos = cc, d
	env.reset()
	env.rng = np.random.default_rng(seed)
	net = None
	if beta < 1.0:
		net = SetManagerActor(autoregressive=True)
		net.load_state_dict(torch.load(wpath, map_location="cpu"))
		net.eval()
	base = chain_escape_manager()
	samples, n_opts, n_exp = [], 0, 0

	def decide():
		nonlocal n_opts, n_exp
		lab, no = expert_label(env, base, k_cand, horizon, margin, reps)
		n_opts += no
		o = pad_obs(env.manager_set_obs(), n)
		mk = padded_mask(env)
		lp = np.full(N_MAX, K - 2, dtype=np.int64)
		lp[:n] = lab
		samples.append((o, mk, lp))
		if net is None or rng.random() < beta:
			env.set_goals(lab); n_exp += 1
		else:
			with torch.no_grad():
				oo = {k: torch.as_tensor(v).unsqueeze(0) for k, v in o.items()}
				a, _p, _ = net(oo, torch.as_tensor(mk).unsqueeze(0))
			env.set_goals(a[0, :n].numpy())

	t0 = time.time()
	decide()
	done = False
	while not done and env.current_step < max_steps:
		_o, _w, _r, done = env.step(straight_worker(env))
		if not done and (env.current_step % HL == 0 or env.goal_invalid().any()):
			decide()
	info = dict(n=n, m=m, makespan=env.makespan, steps=env.current_step, decisions=len(samples),
	            opts=n_opts / max(1, len(samples)), expert_frac=n_exp / max(1, len(samples)), sec=time.time() - t0)
	return samples, info


def stack(data):
	keys = list(data[0][0].keys())
	O = {k: np.stack([x[0][k] for x in data]) for k in keys}
	return O, np.stack([x[1] for x in data]), np.stack([x[2] for x in data])


def ce_eval(actor, data, dev, cap=6000):
	"""보류 데이터의 교차 엔트로피와 라벨 일치율 (자기회귀 교사 강제, 일치율은 argmax)."""
	if not data:
		return None, None
	O, Mk, Y = stack(data[:cap])
	tot, ok_sum, hit = 0.0, 0.0, 0.0
	with torch.no_grad():
		for i in range(0, len(Y), 1024):
			o = {k: torch.as_tensor(v[i:i + 1024], device=dev) for k, v in O.items()}
			mk = torch.as_tensor(Mk[i:i + 1024], device=dev); y = torch.as_tensor(Y[i:i + 1024], device=dev)
			_a, p, lp = actor(o, mk, given=y)
			ok = o["drone_mask"].float() * mk.gather(-1, y.unsqueeze(-1)).squeeze(-1).float()
			tot += float(-(lp.gather(-1, y.unsqueeze(-1)).squeeze(-1) * ok).sum())
			hit += float(((p.argmax(-1) == y).float() * ok).sum())
			ok_sum += float(ok.sum())
	return tot / max(1.0, ok_sum), hit / max(1.0, ok_sum)


def train(actor, data, steps, batch, lr, dev, rng, val=None, check=200, patience=3):
	"""모은 전체 데이터로 교차 엔트로피 학습 (자기회귀 교사 강제). (평균 손실, 라벨 일치율) 을 돌려준다.

	val (보류 에피소드의 표본) 을 주면 check 스텝마다 보류 손실을 재어 가장 낮을 때의 가중치로 되돌리고, patience 번
	연속 나아지지 않으면 멈춘다. DA1(조기 종료 없음)은 학습 데이터 일치 0.998 대 새 상태 일치 0.78 로 외워 버렸다.
	"""
	O, Mk, Y = stack(data)
	S = len(Y)
	opt = torch.optim.Adam(actor.parameters(), lr=lr)
	actor.train()
	losses, accs = [], []
	best_v, best_sd, bad = None, None, 0
	if val:
		actor.eval(); best_v, _ = ce_eval(actor, val, dev); actor.train()
		best_sd = {k: v.detach().clone() for k, v in actor.state_dict().items()}
	for st in range(steps):
		idx = rng.integers(S, size=min(batch, S))
		o = {k: torch.as_tensor(v[idx], device=dev) for k, v in O.items()}
		mk = torch.as_tensor(Mk[idx], device=dev)
		y = torch.as_tensor(Y[idx], device=dev)
		_a, p, lp = actor(o, mk, given=y)
		ll = lp.gather(-1, y.unsqueeze(-1)).squeeze(-1)
		ok = o["drone_mask"].float() * mk.gather(-1, y.unsqueeze(-1)).squeeze(-1).float()
		loss = -(ll * ok).sum() / ok.sum().clamp_min(1.0)
		opt.zero_grad()
		loss.backward()
		torch.nn.utils.clip_grad_norm_(actor.parameters(), 1.0)
		opt.step()
		losses.append(float(loss.detach()))
		accs.append(float(((p.argmax(-1) == y).float() * ok).sum() / ok.sum().clamp_min(1.0)))
		if val and (st + 1) % check == 0:
			actor.eval(); v, _ = ce_eval(actor, val, dev); actor.train()
			if v < best_v - 1e-4:
				best_v, best_sd, bad = v, {k: t.detach().clone() for k, t in actor.state_dict().items()}, 0
			else:
				bad += 1
				if bad >= patience:
					break
	if val:
		actor.load_state_dict(best_sd)
	actor.eval()
	TRAIN_INFO["steps"] = st + 1
	return float(np.mean(losses[-200:])), float(np.mean(accs[-200:]))


TRAIN_INFO = {"steps": 0}


def val_job(args):
	"""검증 작업자 (spawn): 가중치 파일의 정책으로 검증 인스턴스 k 를 R 회 굴린다."""
	wpath, k, R = args
	torch.set_num_threads(1)
	from util.paired_v5_26_09_27_08 import learned
	from util.val_paired_v5_26_11_09_17 import evaluate
	lf = learned(wpath, torch.device("cpu"), True)
	return k, evaluate(lf, R, ks=[k])[k]


def main():
	p = argparse.ArgumentParser()
	p.add_argument("--tag", required=True)
	p.add_argument("--seed", type=int, default=1)
	p.add_argument("--init", default="weights/v5_26_09_23_18_SWa/best_manager.pth")
	p.add_argument("--betas", default="1.0,0.5,0.25,0.0,0.0")
	p.add_argument("--episodes", type=int, default=40, help="반복당 에피소드 수")
	p.add_argument("--workers", type=int, default=6)
	p.add_argument("--k-cand", type=int, default=2)
	p.add_argument("--horizon", type=int, default=1000)
	p.add_argument("--max-steps", type=int, default=1200)
	p.add_argument("--train-steps", type=int, default=2000)
	p.add_argument("--batch", type=int, default=256)
	p.add_argument("--lr", type=float, default=1e-4)
	p.add_argument("--val-samples", type=int, default=3)
	p.add_argument("--label-margin", type=float, default=10.0,
	               help="교사가 규칙 배정에서 벗어나려면 이만큼(스텝) 넘게 짧아야 한다. 26-11-09 DA2 부터 기본 10 — DA1 은 0")
	p.add_argument("--label-reps", type=int, default=2, help="교사 후보마다 공통 난수 롤아웃 수 (평균). DA2 부터 기본 2 — DA1 은 1")
	p.add_argument("--drones-range", type=int, nargs=2, default=[3, 4], help="수집 에피소드의 드론 수 범위 (사이클 33: 3 6)")
	p.add_argument("--dests-range", type=int, nargs=2, default=[20, 80], help="수집 에피소드의 수요지 수 범위 (사이클 33: 20 150)")
	p.add_argument("--holdout-frac", type=float, default=0.1,
	               help="에피소드의 이 비율을 보류해 학습 조기 종료에 쓴다 (0 이면 끔). 26-11-09 DA2 부터 기본 0.1 — DA1 은 끈 상태로 돌았다")
	A = p.parse_args()
	os.chdir(ROOT)
	from util.val_paired_v5_26_11_09_17 import instances, load_ref, summarize
	W_DIR = os.path.join(ROOT, "weights", A.tag); os.makedirs(W_DIR, exist_ok=True)
	R_DIR = os.path.join(ROOT, "runs", A.tag); os.makedirs(R_DIR, exist_ok=True)
	dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
	torch.manual_seed(A.seed)
	rng = np.random.default_rng(A.seed)
	actor = SetManagerActor(autoregressive=True).to(dev)
	actor.load_state_dict(torch.load(os.path.join(ROOT, A.init), map_location=dev))
	actor.eval()
	wpath = os.path.join(R_DIR, "student.pth")
	ref = load_ref()
	ctx = mp.get_context("spawn")
	pool = ctx.Pool(A.workers)
	log = open(os.path.join(R_DIR, f"distill_{A.tag}.jsonl"), "a")

	def save_student():
		tmp = wpath + ".tmp"
		torch.save({k: v.detach().cpu() for k, v in actor.state_dict().items()}, tmp)
		os.replace(tmp, wpath)

	def validate(tag):
		save_student()
		res = dict(pool.map(val_job, [(wpath, k, A.val_samples) for k in range(len(instances()))]))
		sm = summarize(res, ref)
		print(f"  [검증 {tag}] 기준 대비 비율 {sm['ratio']:.3f} | 3/30 {sm['r3_30']:.3f} 5/50 {sm['r5_50']:.3f} "
		      f"4/50 {sm['r4_50']:.3f} | 완주 {sm['full']:.3f}", flush=True)
		return sm

	print(f"증류 {A.tag} | 구성 드론 {A.drones_range} 수요지 {A.dests_range} 상한 {A.max_steps} | 교사: 규칙 위 한 수 롤아웃 선견 (후보 {A.k_cand}곳, 토글 없음, 지평 {A.horizon}, "
	      f"이탈 여유폭 {A.label_margin}스텝, 후보당 롤아웃 {A.label_reps}회) | "
	      f"학생 초기 {A.init} | beta {A.betas} | 반복당 {A.episodes} 에피소드 | 학습 최대 {A.train_steps} 스텝 | "
	      f"보류 {A.holdout_frac} (조기 종료 {'켬' if A.holdout_frac > 0 else '끔'})", flush=True)
	sm0 = validate("초기(SWa)")
	best, best_it = sm0["ratio"], 0
	torch.save(actor.state_dict(), os.path.join(W_DIR, "best_manager.pth"))
	log.write(json.dumps(dict(it=0, **sm0)) + "\n"); log.flush()
	data, held = [], []
	hrng = np.random.default_rng(A.seed + 777)
	for it, beta in enumerate([float(x) for x in A.betas.split(",")], start=1):
		t0 = time.time()
		save_student()
		seeds = [int(A.seed * 1_000_000 + it * 10_000 + e) for e in range(A.episodes)]
		res = pool.map(episode, [(s, wpath, beta, A.k_cand, A.horizon, A.max_steps, A.label_margin, A.label_reps,
		                          tuple(A.drones_range), tuple(A.dests_range)) for s in seeds])
		for smp, _info in res:
			if A.holdout_frac > 0 and hrng.random() < A.holdout_frac:
				held += smp
			else:
				data += smp
		infos = [x[1] for x in res]
		done = [x for x in infos if x["makespan"]]
		print(f"반복 {it} (beta {beta}): 에피소드 {len(infos)} | 완주 {len(done)}/{len(infos)} | 결정 {sum(x['decisions'] for x in infos)} "
		      f"(후보 평균 {np.mean([x['opts'] for x in infos]):.1f}) | 데이터 {len(data)} (보류 {len(held)}) | 수집 {time.time() - t0:.0f}초", flush=True)
		vs = [held[i] for i in rng.permutation(len(held))[:6000]] if held else None
		loss, acc = train(actor, data, A.train_steps, A.batch, A.lr, dev, rng, val=vs)
		hv, ha = ce_eval(actor, vs, dev) if vs else (None, None)
		print(f"  학습: {TRAIN_INFO['steps']} 스텝, 손실 {loss:.3f}, 라벨 일치 {acc:.3f}"
		      + (f" | 보류 손실 {hv:.3f}, 보류 일치 {ha:.3f}" if vs else ""), flush=True)
		torch.save(actor.state_dict(), os.path.join(W_DIR, f"manager_it{it}.pth"))
		sm = validate(f"반복 {it}")
		is_best = sm["ratio"] < best - 0.005
		if is_best:
			best, best_it = sm["ratio"], it
			torch.save(actor.state_dict(), os.path.join(W_DIR, "best_manager.pth"))
		log.write(json.dumps(dict(it=it, beta=beta, data=len(data), held=len(held), loss=loss, acc=acc, held_loss=hv, held_acc=ha,
		                          steps=TRAIN_INFO["steps"], is_best=int(is_best), **sm,
		                          collect=[dict(n=x["n"], m=x["m"], makespan=x["makespan"], decisions=x["decisions"]) for x in infos])) + "\n")
		log.flush()
		print(f"  최고 비율 {best:.3f} @반복 {best_it}{'  ← 갱신' if is_best else ''}", flush=True)
	pool.close()
	print(f"증류 종료. 최고 비율 {best:.3f} @반복 {best_it}", flush=True)


if __name__ == "__main__":
	main()
