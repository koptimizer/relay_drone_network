"""CTDE-SAC 학습 루프 (v1, 26_08_24_22).

에피소드 단위 배송/균형/연결 지표를 TensorBoard와 CSV로 함께 남긴다.
CKPT_EVERY 에피소드마다 actor 가중치와 재개용 전체 상태를 저장한다.
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

from env.disaster_relay_env_v1_26_08_24_22 import DisasterRelayDroneEnv
from model.sac_net_v1_26_08_24_22 import SACActor, SACCritic
from pipeline.replay_v1_26_08_24_22 import EfficientReplayBuffer, RewardNormalizer

BATCH_SIZE    = 512   # 256→512: 보상 분산 감소, 그래디언트 안정화
WARMUP_STEPS  = 5000  # 256→5000: 다양한 초기 샘플 확보 후 학습 시작
POLICY_DELAY  = 2     # Critic 2번 업데이트 당 Actor 1번 (TD3 스타일)
TAU           = 0.001 # 0.005→0.001: 타깃 네트워크 느리게 추적 → Critic 스파이크 완화
MAX_STEPS     = 1000  # 에피소드 스텝 상한
MAX_EPISODES  = 100000
CKPT_EVERY    = 10    # 가중치 저장 주기 (에피소드)

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 기본값은 26_08_24_22 실행과 동일하다 (relay_w=0.0).
_p = argparse.ArgumentParser()
_p.add_argument("--tag", default="v1_26_08_24_22", help="실행 태그 (runs/·weights/ 하위 폴더명)")
_p.add_argument("--relay-w", type=float, default=0.0, help="중계 potential 쉐이핑 가중치")
ARGS = _p.parse_args()

RUN_TAG    = ARGS.tag
LOG_DIR    = os.path.join(ROOT, "runs", RUN_TAG)
WEIGHT_DIR = os.path.join(ROOT, "weights", RUN_TAG)
CSV_PATH   = os.path.join(LOG_DIR, f"metrics_{RUN_TAG}.csv")

CSV_COLS = [
	"episode", "steps", "reward", "delivered", "completion_rate", "makespan",
	"idle_drones", "deliv_min", "deliv_max", "deliv_std", "reloads", "blocked",
	"comm_loss", "alpha", "loss_critic", "loss_actor", "env_steps", "wall_sec",
]


if __name__ == "__main__":
	env = DisasterRelayDroneEnv(relay_w=ARGS.relay_w)
	env.reset()

	obs_dim = env._get_local_obs().shape[-1]      # 19
	state_dim = env._get_global_state().shape[0]  # 66
	act_dim = 2
	total_act_dim = act_dim * env.num_drones       # 8

	device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
	print(f"Using device: {device}  tag={RUN_TAG}  relay_w={ARGS.relay_w}", flush=True)
	print(f"obs_dim={obs_dim}, state_dim={state_dim}, total_act_dim={total_act_dim}", flush=True)

	actor = SACActor(obs_dim, act_dim).to(device)
	critic = SACCritic(state_dim, total_act_dim).to(device)
	critic_t = SACCritic(state_dim, total_act_dim).to(device)
	critic_t.load_state_dict(critic.state_dict())

	a_opt = optim.Adam(actor.parameters(), lr=3e-4)
	c_opt = optim.Adam(critic.parameters(), lr=3e-4)
	log_alpha = torch.tensor(np.log(0.1), requires_grad=True, device=device)
	alpha_opt = optim.Adam([log_alpha], lr=3e-4)

	# Alpha 그래프: 탐색이 너무 빨리 수렴 → target_entropy를 0.6배로 완화
	target_entropy = -float(total_act_dim) * 0.6

	buffer = EfficientReplayBuffer(env.num_drones, obs_dim, state_dim, act_dim, capacity=100000)
	reward_norm = RewardNormalizer()

	os.makedirs(LOG_DIR, exist_ok=True)
	os.makedirs(WEIGHT_DIR, exist_ok=True)
	writer = SummaryWriter(log_dir=LOG_DIR)
	csv_file = open(CSV_PATH, "w", newline="", encoding="utf-8")
	csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_COLS)
	csv_writer.writeheader()

	total_env_steps = 0
	update_count = 0
	# 마지막 관측 손실값 — Actor는 POLICY_DELAY 주기로만 갱신되므로 따로 들고 있는다.
	# (구버전은 로깅 조건 두 개의 주기가 영원히 어긋나 Actor 지표가 한 번도 기록되지 않았다)
	last_c_loss = last_a_loss = float("nan")
	last_c_grad = last_a_grad = float("nan")
	t0 = time.time()

	for episode in range(1, MAX_EPISODES + 1):
		local_obs = env.reset()
		global_state = env._get_global_state()
		ep_reward, step, done = 0.0, 0, False

		while not done and step < MAX_STEPS:
			with torch.no_grad():
				a_tanh, _ = actor(torch.FloatTensor(local_obs).to(device))
				actions = a_tanh.cpu().numpy()  # (4, 2)
				env_acts = np.stack(
					[(actions[:, 0] + 1) * np.pi, (actions[:, 1] + 1) * 0.5], axis=1
				)

			next_local_obs, next_global_state, rewards, done = env.step(env_acts)

			buffer.push(
				local_obs, global_state, actions.flatten(),
				rewards,
				next_local_obs, next_global_state, float(done)
			)

			if len(buffer) > WARMUP_STEPS:
				b_o, b_s, b_a, b_r, b_no, b_ns, b_d = buffer.sample(BATCH_SIZE, device=device)

				team_r = b_r.mean(dim=1, keepdim=True)  # (batch, 1)
				reward_norm.update(team_r)
				team_r_norm = reward_norm.normalize(team_r)

				alpha = log_alpha.exp().detach()

				with torch.no_grad():
					na, nlp = actor(b_no)
					na_j = na.flatten(start_dim=1)    # (batch, 8)
					nlp_j = nlp.sum(dim=1)            # (batch, 1)
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

				# Policy delay: Critic 2회 업데이트당 Actor 1회 업데이트
				if update_count % POLICY_DELAY == 0:
					ca, clp = actor(b_o)
					ca_j = ca.flatten(start_dim=1)    # (batch, 8)
					clp_j = clp.sum(dim=1)            # (batch, 1)
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

				# TAU 상수 사용: 느린 타깃 추적으로 Q-값 발산 억제
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
		writer.add_scalar('Reward/Episode', ep_reward, episode)
		writer.add_scalar('Steps/Episode', step, episode)
		writer.add_scalar('Delivery/Completed', st["delivered"], episode)
		writer.add_scalar('Delivery/CompletionRate', st["completion_rate"], episode)
		writer.add_scalar('Delivery/IdleDrones', st["idle_drones"], episode)
		writer.add_scalar('Delivery/PerDroneMin', st["deliv_min"], episode)
		writer.add_scalar('Delivery/PerDroneMax', st["deliv_max"], episode)
		writer.add_scalar('Delivery/PerDroneStd', st["deliv_std"], episode)
		writer.add_scalar('Delivery/Reloads', st["reloads"], episode)
		writer.add_scalar('Comm/BlockedMoves', st["blocked"], episode)
		writer.add_scalar('Comm/LossTerminated', st["comm_loss"], episode)
		if st["makespan"] is not None:
			writer.add_scalar('Delivery/Makespan', st["makespan"], episode)

		csv_writer.writerow({
			"episode": episode, "steps": step, "reward": round(float(ep_reward), 4),
			"delivered": st["delivered"], "completion_rate": round(st["completion_rate"], 4),
			"makespan": st["makespan"] if st["makespan"] is not None else "",
			"idle_drones": st["idle_drones"], "deliv_min": st["deliv_min"],
			"deliv_max": st["deliv_max"], "deliv_std": round(st["deliv_std"], 4),
			"reloads": st["reloads"], "blocked": st["blocked"], "comm_loss": st["comm_loss"],
			"alpha": round(float(log_alpha.exp().item()), 6),
			"loss_critic": round(last_c_loss, 6), "loss_actor": round(last_a_loss, 6),
			"env_steps": total_env_steps, "wall_sec": round(time.time() - t0, 1),
		})
		csv_file.flush()

		print(
			f"Ep {episode} | Step {step} | R {ep_reward:8.2f} | "
			f"배송 {st['delivered']:2d}/50 | 미활동드론 {st['idle_drones']} | "
			f"재적재 {st['reloads']} | 이동차단 {st['blocked']:4d} | 단절 {st['comm_loss']}",
			flush=True
		)

		if episode % CKPT_EVERY == 0:
			torch.save(actor.state_dict(), os.path.join(WEIGHT_DIR, f"sac_actor_ep{episode}.pth"))
			torch.save({
				"episode": episode,
				"actor": actor.state_dict(),
				"critic": critic.state_dict(),
				"critic_t": critic_t.state_dict(),
				"a_opt": a_opt.state_dict(),
				"c_opt": c_opt.state_dict(),
				"log_alpha": log_alpha.detach().cpu(),
			}, os.path.join(WEIGHT_DIR, "latest.pth"))

	writer.close()
	csv_file.close()
	print("Training Complete.", flush=True)
