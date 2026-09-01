"""CTDE-SAC 학습 루프 (v2, 26_08_25_14).

v1 대비 핵심 변화: 학습 중 홀드아웃 인스턴스로 주기 평가해 최고 정책을 보존하고,
개선이 멈추면 조기 종료한다. v1은 좋은 정책을 만든 뒤 계속 학습해 스스로 망가뜨렸다.
"""

import argparse
import csv
import os
import sys
import time

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.tensorboard import SummaryWriter

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v2_26_08_25_14 import DisasterRelayDroneEnv
from model.sac_net_v2_26_08_25_14 import SACActor, SACCritic
from pipeline.replay_v2_26_08_25_14 import EfficientReplayBuffer, RewardNormalizer
from util.instance_generator_v2_26_08_25_14 import sample_instance


def to_env_action(a, mode):
	"""actor의 tanh 출력을 환경 행동으로 바꾼다."""
	if mode == "vector":
		return a
	return np.stack([(a[:, 0] + 1) * np.pi, (a[:, 1] + 1) * 0.5], axis=1)

BATCH_SIZE    = 512
WARMUP_STEPS  = 5000
POLICY_DELAY  = 2
TAU           = 0.001
MAX_STEPS     = 1000
CKPT_EVERY    = 10

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

_p = argparse.ArgumentParser()
_p.add_argument("--tag", default="v2_26_08_25_14", help="실행 태그")
_p.add_argument("--relay-w", type=float, default=0.0, help="중계 potential 쉐이핑 가중치")
_p.add_argument("--max-episodes", type=int, default=100000)
_p.add_argument("--eval-every", type=int, default=50, help="홀드아웃 평가 주기 (에피소드)")
_p.add_argument("--eval-n", type=int, default=6, help="평가 인스턴스 수")
_p.add_argument("--eval-seed0", type=int, default=201, help="평가 시드 시작값 (학습과 분리)")
_p.add_argument("--patience", type=int, default=15, help="개선 없이 견딜 평가 횟수")
_p.add_argument("--resume", action="store_true", help="latest.pth에서 이어서 학습")
# --- 단계 2 격리 실험 플래그: 각각 하나의 가설만 검증한다 ---
_p.add_argument("--alpha-min", type=float, default=0.0, help="E1 탐색 붕괴: alpha 하한")
_p.add_argument("--relay-credit", action="store_true", help="E2 보상 구조: 팀 보상 중계 가중 재분배")
_p.add_argument("--n-peers", type=int, default=2, help="E3 관측 한계: 관측할 동료 수")
_p.add_argument("--comm-range", type=float, default=300.0, help="통신 반경 (v2 기준 300)")
_p.add_argument("--projection", choices=["cancel", "scale"], default="scale",
                help="연결 제약 위반 시 이동 전체 취소(cancel) 또는 최대 안전 변위로 축소(scale)")
_p.add_argument("--action-mode", choices=["angle", "vector"], default="angle",
                help="angle=v1 방식, vector=속도 벡터 직접 출력")
ARGS = _p.parse_args()

RUN_TAG    = ARGS.tag
LOG_DIR    = os.path.join(ROOT, "runs", RUN_TAG)
WEIGHT_DIR = os.path.join(ROOT, "weights", RUN_TAG)
CSV_PATH   = os.path.join(LOG_DIR, f"metrics_{RUN_TAG}.csv")
EVAL_PATH  = os.path.join(LOG_DIR, f"evals_{RUN_TAG}.csv")

CSV_COLS = [
	"episode", "steps", "reward", "delivered", "completion_rate", "makespan",
	"idle_drones", "deliv_min", "deliv_max", "deliv_std", "reloads", "blocked",
	"hop2plus", "max_reach", "t_last_deliv", "comm_loss", "alpha",
	"loss_critic", "loss_actor", "env_steps", "wall_sec",
]
EVAL_COLS = ["episode", "delivered", "delivered_sd", "idle_drones", "hop2plus",
             "max_reach", "comm_loss", "is_best", "env_steps", "wall_sec"]


def holdout_eval(env, actor, device, n, seed0):
	"""학습에 쓰지 않은 인스턴스에서 결정론적 정책을 굴려 일반화 성능을 잰다."""
	D, I, H, R, C = [], [], [], [], 0
	for s in range(seed0, seed0 + n):
		env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=s)
		obs = env.reset()
		done, step = False, 0
		while not done and step < MAX_STEPS:
			with torch.no_grad():
				a, _ = actor(torch.FloatTensor(obs).to(device), deterministic=True)
			a = a.cpu().numpy()
			obs, _s, _r, done = env.step(to_env_action(a, env.action_mode))
			step += 1
		st = env.episode_stats()
		D.append(st["delivered"])
		I.append(st["idle_drones"])
		H.append(st["hop2plus"])
		R.append(st["max_reach"])
		C += st["comm_loss"]
	return {"delivered": float(np.mean(D)), "delivered_sd": float(np.std(D)),
	        "idle_drones": float(np.mean(I)), "hop2plus": float(np.mean(H)),
	        "max_reach": float(np.mean(R)), "comm_loss": C}


if __name__ == "__main__":
	kw = dict(relay_w=ARGS.relay_w, n_peers=ARGS.n_peers, relay_credit=ARGS.relay_credit,
	          comm_range=ARGS.comm_range, action_mode=ARGS.action_mode,
	          projection=ARGS.projection)
	env = DisasterRelayDroneEnv(**kw)
	eval_env = DisasterRelayDroneEnv(**kw)   # 학습 상태를 건드리지 않는 별도 인스턴스
	env.reset()

	obs_dim = env._get_local_obs().shape[-1]
	state_dim = env._get_global_state().shape[0]
	act_dim = 2
	total_act_dim = act_dim * env.num_drones

	device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
	print(f"Using device: {device}  tag={RUN_TAG}", flush=True)
	print(f"설정: relay_w={ARGS.relay_w} alpha_min={ARGS.alpha_min} "
	      f"relay_credit={ARGS.relay_credit} n_peers={ARGS.n_peers} "
	      f"comm_range={ARGS.comm_range} action_mode={ARGS.action_mode} "
	      f"projection={ARGS.projection}", flush=True)
	print(f"obs_dim={obs_dim}, state_dim={state_dim}, total_act_dim={total_act_dim}", flush=True)
	print(f"조기 종료: {ARGS.eval_every}ep마다 홀드아웃 {ARGS.eval_n}개 평가, "
	      f"patience={ARGS.patience}회", flush=True)

	actor = SACActor(obs_dim, act_dim).to(device)
	critic = SACCritic(state_dim, total_act_dim).to(device)
	critic_t = SACCritic(state_dim, total_act_dim).to(device)
	critic_t.load_state_dict(critic.state_dict())

	a_opt = optim.Adam(actor.parameters(), lr=3e-4)
	c_opt = optim.Adam(critic.parameters(), lr=3e-4)
	log_alpha = torch.tensor(np.log(0.1), requires_grad=True, device=device)
	alpha_opt = optim.Adam([log_alpha], lr=3e-4)
	target_entropy = -float(total_act_dim) * 0.6

	buffer = EfficientReplayBuffer(env.num_drones, obs_dim, state_dim, act_dim, capacity=100000)
	reward_norm = RewardNormalizer()

	# 재개: 리플레이 버퍼는 복원하지 않으므로 WARMUP만큼 다시 채운 뒤 학습이 재개된다
	start_ep, resumed = 1, None
	latest = os.path.join(WEIGHT_DIR, "latest.pth")
	if ARGS.resume and os.path.exists(latest):
		resumed = torch.load(latest, map_location=device, weights_only=False)
		actor.load_state_dict(resumed["actor"])
		critic.load_state_dict(resumed["critic"])
		critic_t.load_state_dict(resumed["critic_t"])
		a_opt.load_state_dict(resumed["a_opt"])
		c_opt.load_state_dict(resumed["c_opt"])
		with torch.no_grad():
			log_alpha.copy_(resumed["log_alpha"].to(device))
		start_ep = int(resumed["episode"]) + 1
		print(f"재개: ep{start_ep}부터 (이전 최고 {resumed.get('best_score', -1):.1f}"
		      f"@ep{resumed.get('best_ep', 0)})", flush=True)

	os.makedirs(LOG_DIR, exist_ok=True)
	os.makedirs(WEIGHT_DIR, exist_ok=True)
	writer = SummaryWriter(log_dir=LOG_DIR)
	def truncate_from(path, ep):
		"""재개 시 되감긴 구간의 기록을 지워 CSV에 중복 행이 남지 않게 한다."""
		if not os.path.exists(path):
			return
		rows = list(csv.DictReader(open(path, encoding="utf-8")))
		keep = [r for r in rows if int(r["episode"]) < ep]
		with open(path, "w", newline="", encoding="utf-8") as f:
			w = csv.DictWriter(f, fieldnames=rows[0].keys() if rows else CSV_COLS)
			w.writeheader()
			w.writerows(keep)

	if resumed:
		truncate_from(CSV_PATH, start_ep)
		truncate_from(EVAL_PATH, start_ep)

	mode = "a" if resumed else "w"
	csv_file = open(CSV_PATH, mode, newline="", encoding="utf-8")
	csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_COLS)
	ev_file = open(EVAL_PATH, mode, newline="", encoding="utf-8")
	ev_writer = csv.DictWriter(ev_file, fieldnames=EVAL_COLS)
	if not resumed:
		csv_writer.writeheader()
		ev_writer.writeheader()

	total_env_steps, update_count = 0, 0
	last_c_loss = last_a_loss = float("nan")
	last_c_grad = last_a_grad = float("nan")
	best_score = resumed.get("best_score", -1.0) if resumed else -1.0
	best_ep = resumed.get("best_ep", 0) if resumed else 0
	stale = resumed.get("stale", 0) if resumed else 0
	t0 = time.time()

	for episode in range(start_ep, ARGS.max_episodes + 1):
		local_obs = env.reset()
		global_state = env._get_global_state()
		ep_reward, step, done = 0.0, 0, False

		while not done and step < MAX_STEPS:
			with torch.no_grad():
				a_tanh, _ = actor(torch.FloatTensor(local_obs).to(device))
				actions = a_tanh.cpu().numpy()
				env_acts = to_env_action(actions, ARGS.action_mode)

			next_local_obs, next_global_state, rewards, done = env.step(env_acts)
			buffer.push(local_obs, global_state, actions.flatten(), rewards,
			            next_local_obs, next_global_state, float(done))

			if len(buffer) > WARMUP_STEPS:
				b_o, b_s, b_a, b_r, b_no, b_ns, b_d = buffer.sample(BATCH_SIZE, device=device)
				team_r = b_r.mean(dim=1, keepdim=True)
				reward_norm.update(team_r)
				team_r_norm = reward_norm.normalize(team_r)
				alpha = log_alpha.exp().detach()

				with torch.no_grad():
					na, nlp = actor(b_no)
					na_j = na.flatten(start_dim=1)
					nlp_j = nlp.sum(dim=1)
					tq1, tq2 = critic_t(b_ns, na_j)
					y = team_r_norm + 0.99 * (1 - b_d) * (torch.min(tq1, tq2) - alpha * nlp_j)

				q1, q2 = critic(b_s, b_a)
				loss_c = nn.MSELoss()(q1, y) + nn.MSELoss()(q2, y)
				c_opt.zero_grad()
				loss_c.backward()
				c_grad_norm = torch.nn.utils.clip_grad_norm_(critic.parameters(), 1.0)
				c_opt.step()
				last_c_loss, last_c_grad = loss_c.item(), c_grad_norm.item()
				update_count += 1

				if update_count % POLICY_DELAY == 0:
					ca, clp = actor(b_o)
					ca_j = ca.flatten(start_dim=1)
					clp_j = clp.sum(dim=1)
					loss_a = (alpha * clp_j - critic(b_s, ca_j)[0]).mean()
					a_opt.zero_grad()
					loss_a.backward()
					a_grad_norm = torch.nn.utils.clip_grad_norm_(actor.parameters(), 1.0)
					a_opt.step()
					last_a_loss, last_a_grad = loss_a.item(), a_grad_norm.item()

					alpha_loss = -(log_alpha * (clp_j + target_entropy).detach()).mean()
					alpha_opt.zero_grad()
					alpha_loss.backward()
					alpha_opt.step()
					if ARGS.alpha_min > 0.0:
						with torch.no_grad():
							log_alpha.clamp_(min=float(np.log(ARGS.alpha_min)))

				for t, p in zip(critic_t.parameters(), critic.parameters()):
					t.data.copy_((1 - TAU) * t.data + TAU * p.data)

				if total_env_steps % 50 == 0:
					writer.add_scalar('Loss/Critic', last_c_loss, total_env_steps)
					writer.add_scalar('GradNorm/Critic', last_c_grad, total_env_steps)
					writer.add_scalar('Loss/Actor', last_a_loss, total_env_steps)
					writer.add_scalar('GradNorm/Actor', last_a_grad, total_env_steps)
					writer.add_scalar('Alpha', alpha.item(), total_env_steps)
					writer.add_scalar('Reward/RewardNorm_Mean', reward_norm.mean, total_env_steps)

			local_obs, global_state = next_local_obs, next_global_state
			ep_reward += rewards.mean()
			step += 1
			total_env_steps += 1

		st = env.episode_stats()
		for k, v in [('Reward/Episode', ep_reward), ('Steps/Episode', step),
		             ('Delivery/Completed', st["delivered"]),
		             ('Delivery/CompletionRate', st["completion_rate"]),
		             ('Delivery/IdleDrones', st["idle_drones"]),
		             ('Delivery/PerDroneStd', st["deliv_std"]),
		             ('Delivery/Reloads', st["reloads"]),
		             ('Comm/BlockedMoves', st["blocked"]),
		             ('Comm/LossTerminated', st["comm_loss"]),
		             ('Relay/Hop2PlusRatio', st["hop2plus"]),
		             ('Relay/MaxReach', st["max_reach"])]:
			writer.add_scalar(k, v, episode)
		if st["makespan"] is not None:
			writer.add_scalar('Delivery/Makespan', st["makespan"], episode)

		csv_writer.writerow({
			"episode": episode, "steps": step, "reward": round(float(ep_reward), 4),
			"delivered": st["delivered"], "completion_rate": round(st["completion_rate"], 4),
			"makespan": st["makespan"] if st["makespan"] is not None else "",
			"idle_drones": st["idle_drones"], "deliv_min": st["deliv_min"],
			"deliv_max": st["deliv_max"], "deliv_std": round(st["deliv_std"], 4),
			"reloads": st["reloads"], "blocked": st["blocked"],
			"hop2plus": round(st["hop2plus"], 4), "max_reach": round(st["max_reach"], 1),
			"t_last_deliv": st["t_last_deliv"], "comm_loss": st["comm_loss"],
			"alpha": round(float(log_alpha.exp().item()), 6),
			"loss_critic": round(last_c_loss, 6), "loss_actor": round(last_a_loss, 6),
			"env_steps": total_env_steps, "wall_sec": round(time.time() - t0, 1),
		})
		csv_file.flush()

		print(f"Ep {episode} | Step {step} | R {ep_reward:8.2f} | "
		      f"배송 {st['delivered']:2d}/50 | 미활동 {st['idle_drones']} | "
		      f"2홉+ {st['hop2plus']:.2f} | 도달 {st['max_reach']:4.0f} | "
		      f"차단 {st['blocked']:4d}", flush=True)

		if episode % CKPT_EVERY == 0:
			torch.save(actor.state_dict(), os.path.join(WEIGHT_DIR, f"sac_actor_ep{episode}.pth"))
			# 크래시 대비 전체 상태. 임시 파일에 쓴 뒤 교체해 중간에 죽어도 파일이 깨지지 않는다.
			tmp = os.path.join(WEIGHT_DIR, "latest.tmp")
			torch.save({"episode": episode, "actor": actor.state_dict(),
			            "critic": critic.state_dict(), "critic_t": critic_t.state_dict(),
			            "a_opt": a_opt.state_dict(), "c_opt": c_opt.state_dict(),
			            "log_alpha": log_alpha.detach().cpu(),
			            "best_score": best_score, "best_ep": best_ep, "stale": stale}, tmp)
			os.replace(tmp, os.path.join(WEIGHT_DIR, "latest.pth"))

		# --- 홀드아웃 평가 · 최고 정책 보존 · 조기 종료 ---
		if episode % ARGS.eval_every == 0 and len(buffer) > WARMUP_STEPS:
			actor.eval()
			ev = holdout_eval(eval_env, actor, device, ARGS.eval_n, ARGS.eval_seed0)
			actor.train()

			is_best = ev["delivered"] > best_score
			if is_best:
				best_score, best_ep, stale = ev["delivered"], episode, 0
				torch.save({"episode": episode, "score": best_score,
				            "actor": actor.state_dict(), "critic": critic.state_dict(),
				            "critic_t": critic_t.state_dict(), "a_opt": a_opt.state_dict(),
				            "c_opt": c_opt.state_dict(), "log_alpha": log_alpha.detach().cpu()},
				           os.path.join(WEIGHT_DIR, "best.pth"))
				torch.save(actor.state_dict(), os.path.join(WEIGHT_DIR, "best_actor.pth"))
			else:
				stale += 1

			for k, v in [('Holdout/Delivered', ev["delivered"]),
			             ('Holdout/IdleDrones', ev["idle_drones"]),
			             ('Holdout/Hop2PlusRatio', ev["hop2plus"]),
			             ('Holdout/MaxReach', ev["max_reach"])]:
				writer.add_scalar(k, v, episode)
			ev_writer.writerow({"episode": episode, **{k: round(v, 4) for k, v in ev.items()},
			                    "is_best": int(is_best), "env_steps": total_env_steps,
			                    "wall_sec": round(time.time() - t0, 1)})
			ev_file.flush()

			print(f"  [홀드아웃] 배송 {ev['delivered']:.1f}±{ev['delivered_sd']:.1f} "
			      f"미활동 {ev['idle_drones']:.2f} 2홉+ {ev['hop2plus']:.2f} "
			      f"도달 {ev['max_reach']:.0f} | 최고 {best_score:.1f}@ep{best_ep} "
			      f"| 정체 {stale}/{ARGS.patience}{'  ← 최고 갱신' if is_best else ''}",
			      flush=True)

			if stale >= ARGS.patience:
				print(f"\n조기 종료: {ARGS.patience}회 연속 개선 없음. "
				      f"최고 정책은 ep{best_ep} (홀드아웃 배송 {best_score:.1f}/50), "
				      f"{WEIGHT_DIR}/best_actor.pth", flush=True)
				break

	writer.close()
	csv_file.close()
	ev_file.close()
	print(f"Training finished. best={best_score:.1f} @ ep{best_ep}", flush=True)
