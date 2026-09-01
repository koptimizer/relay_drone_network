import numpy as np
from gymnasium import spaces
import torch
import torch.nn as nn
import torch.optim as optim
import pygame
import os
import warnings
from torch.utils.tensorboard import SummaryWriter

warnings.filterwarnings("ignore", category=UserWarning, module="pygame")

BATCH_SIZE    = 512   # 256→512: 보상 분산 감소, 그래디언트 안정화
WARMUP_STEPS  = 5000  # 256→5000: 다양한 초기 샘플 확보 후 학습 시작
POLICY_DELAY  = 2     # Critic 2번 업데이트 당 Actor 1번 (TD3 스타일)
TAU           = 0.001 # 0.005→0.001: 타깃 네트워크 느리게 추적 → Critic 스파이크 완화


# --- 1. Reward Normalizer ---
# Critic 스파이크 원인: 보상 범위가 -100 ~ +40으로 극단적.
# 샘플링 후 Bellman 타깃 계산 시 정규화하여 Q-값 스케일을 안정화한다.
# 수집 시점이 아닌 학습 시점에 정규화하므로 환경 탐색 행동에 영향을 주지 않는다.
class RewardNormalizer:
    def __init__(self):
        self.mean  = 0.0
        self.var   = 1.0
        self.count = 1e-4   # 0 나눗셈 방지를 위한 초기값

    def update(self, x: torch.Tensor):
        """배치 통계로 running mean/var 갱신 (Welford 방식)."""
        x_np = x.detach().cpu().numpy().ravel()
        n = len(x_np)
        delta = x_np.mean() - self.mean
        total = self.count + n
        self.mean += delta * n / total
        self.var   = (self.var * self.count + x_np.var() * n
                      + delta ** 2 * self.count * n / total) / total
        self.count = total

    def normalize(self, x: torch.Tensor) -> torch.Tensor:
        std = (self.var ** 0.5 + 1e-8)
        return torch.clamp((x - self.mean) / std, -10.0, 10.0)


# --- 2. Efficient Replay Buffer ---
# Fix 2: rewards를 (capacity, num_drones) shape으로 저장해 개별 기여도 보존
class EfficientReplayBuffer:
    def __init__(self, num_drones, obs_dim, state_dim, action_dim, capacity=100000):
        self.capacity = capacity
        self.num_drones = num_drones
        self.ptr = 0
        self.size = 0

        self.obs = np.zeros((capacity, num_drones, obs_dim), dtype=np.float32)
        self.state = np.zeros((capacity, state_dim), dtype=np.float32)
        self.actions = np.zeros((capacity, num_drones * action_dim), dtype=np.float32)
        self.rewards = np.zeros((capacity, num_drones), dtype=np.float32)  # Fix 2
        self.next_obs = np.zeros((capacity, num_drones, obs_dim), dtype=np.float32)
        self.next_state = np.zeros((capacity, state_dim), dtype=np.float32)
        self.dones = np.zeros((capacity, 1), dtype=np.float32)

    def push(self, obs, state, action, reward, next_obs, next_state, done):
        self.obs[self.ptr] = obs
        self.state[self.ptr] = state
        self.actions[self.ptr] = action
        self.rewards[self.ptr] = reward   # Fix 2: (num_drones,) 배열 그대로 저장
        self.next_obs[self.ptr] = next_obs
        self.next_state[self.ptr] = next_state
        self.dones[self.ptr, 0] = done
        self.ptr = (self.ptr + 1) % self.capacity
        self.size = min(self.size + 1, self.capacity)

    def sample(self, batch_size, device):
        ind = np.random.randint(0, self.size, size=batch_size)
        return (
            torch.FloatTensor(self.obs[ind]).to(device),
            torch.FloatTensor(self.state[ind]).to(device),
            torch.FloatTensor(self.actions[ind]).to(device),
            torch.FloatTensor(self.rewards[ind]).to(device),   # (batch, num_drones)
            torch.FloatTensor(self.next_obs[ind]).to(device),
            torch.FloatTensor(self.next_state[ind]).to(device),
            torch.FloatTensor(self.dones[ind]).to(device)
        )

    def __len__(self):
        return self.size


# --- 2. Environment ---
class DisasterRelayDroneEnv:
    def __init__(self, map_path="disaster_map.npy", render_mode=None):
        self.num_drones = 4
        self.num_dests = 50
        self.map_size = (1000, 1000)
        self.max_capacity = 5
        self.comm_range = 500.0
        self.max_speed = 50.0 * (1000 / 3600)
        self.unload_time = 10
        self.interaction_radius = 25.0
        self.shape_reward_scale = 0.010

        if os.path.exists(map_path):
            map_data = np.load(map_path, allow_pickle=True).item()
            self.cc_pos = map_data['cc_pos']
            self.dests_pos = map_data['dests_pos']
            print(f"지도를 성공적으로 로드했습니다: {map_path}")
        else:
            self.cc_pos = np.array([950.0, 500.0])
            self.dests_pos = np.random.uniform(low=[50, 50], high=[850, 950], size=(self.num_dests, 2))
            print("지도를 찾을 수 없어 임시 지도를 생성했습니다.")

        self.action_space = spaces.Box(
            low=np.array([0.0, 0.0], dtype=np.float32),
            high=np.array([2 * np.pi, 1.0], dtype=np.float32),
            dtype=np.float32
        )

        self.render_mode = render_mode
        self.screen = None
        self.clock = None
        self.render_fps = 120
        self.radius_surface = None
        self.font = None

    def reset(self):
        self.dests_active = np.ones(self.num_dests, dtype=bool)
        self.drones_pos = np.tile(self.cc_pos, (self.num_drones, 1))
        self.drones_capacity = np.full(self.num_drones, self.max_capacity)
        self.drones_timer = np.zeros(self.num_drones)
        self.comm_status = np.ones(self.num_drones, dtype=bool)
        self.prev_target_dists = self._get_target_dists()
        self.current_step = 0
        return self._get_local_obs()

    def _get_target_dists(self):
        if np.any(self.dests_active):
            active_dests = self.dests_pos[self.dests_active]
            dist_to_dests = np.linalg.norm(
                active_dests[None, :, :] - self.drones_pos[:, None, :], axis=2
            )
            nearest_dists = np.min(dist_to_dests, axis=1)
        else:
            nearest_dists = np.zeros(self.num_drones)
        return np.where(
            self.drones_capacity > 0,
            nearest_dists,
            np.linalg.norm(self.drones_pos - self.cc_pos, axis=1)
        )

    # Fix 3: BFS 로직을 헬퍼로 분리해 코드 중복 제거
    def _bfs_connected(self, positions):
        dist_to_cc = np.linalg.norm(positions - self.cc_pos, axis=1)
        visited = dist_to_cc <= self.comm_range

        drone_dists = np.linalg.norm(
            positions[:, None, :] - positions[None, :, :], axis=2
        )
        adjacency = drone_dists <= self.comm_range
        np.fill_diagonal(adjacency, False)

        while True:
            new_visited = visited | (adjacency.dot(visited.astype(int)) > 0)
            if np.array_equal(new_visited, visited):
                break
            visited = new_visited
        return visited

    def _check_connectivity(self):
        self.comm_status = self._bfs_connected(self.drones_pos)

    def step(self, actions):
        self.current_step += 1
        rewards = np.zeros(self.num_drones)
        team_reward = 0.0

        # 이동 제안
        proposed_pos = np.copy(self.drones_pos)
        intended_to_move = np.zeros(self.num_drones, dtype=bool)
        for i in range(self.num_drones):
            if self.drones_timer[i] > 0:
                self.drones_timer[i] -= 1
            else:
                intended_to_move[i] = True
                angle, speed_ratio = actions[i][0], actions[i][1]
                speed = speed_ratio * self.max_speed
                proposed_pos[i] += np.array([np.cos(angle) * speed, np.sin(angle) * speed])
                proposed_pos[i] = np.clip(proposed_pos[i], 0, self.map_size[0])

        # Fix 3: BFS 헬퍼를 재사용한 예방적 연결성 검사
        valid_movement = intended_to_move.copy()
        for _ in range(self.num_drones):
            temp_pos = np.where(valid_movement[:, None], proposed_pos, self.drones_pos)
            visited = self._bfs_connected(temp_pos)
            all_safe = True
            for i in range(self.num_drones):
                if not visited[i] and valid_movement[i]:
                    valid_movement[i] = False
                    all_safe = False
            if all_safe:
                break

        for i in range(self.num_drones):
            if intended_to_move[i]:
                if valid_movement[i]:
                    self.drones_pos[i] = proposed_pos[i]
                else:
                    rewards[i] -= 0.1

        # Fix 3: 단일 comm_status 업데이트 (중복 BFS 제거)
        self._check_connectivity()

        # 연결 단절 안전망 (예방 실패 시에만 발동)
        episode_terminated_by_comm_loss = False
        for i in range(self.num_drones):
            if not self.comm_status[i]:
                rewards[i] -= 100.0
                episode_terminated_by_comm_loss = True

        if episode_terminated_by_comm_loss:
            if self.render_mode == "human":
                self.render()
            return self._get_local_obs(), self._get_global_state(), rewards, True

        # 이동 보상 (포텐셜 기반 쉐이핑)
        curr_target_dists = self._get_target_dists()
        for i in range(self.num_drones):
            if self.drones_timer[i] == 0:
                dist_diff = np.clip(
                    self.prev_target_dists[i] - curr_target_dists[i],
                    -self.max_speed, self.max_speed
                )
                if self.drones_capacity[i] == 0:
                    rewards[i] += dist_diff * (self.shape_reward_scale * 5.0)
                else:
                    rewards[i] += dist_diff * self.shape_reward_scale
            rewards[i] -= 0.001

            # 배달
            if self.drones_capacity[i] > 0 and self.drones_timer[i] == 0:
                for j in range(self.num_dests):
                    if (self.dests_active[j] and
                            np.linalg.norm(self.drones_pos[i] - self.dests_pos[j]) <= self.interaction_radius):
                        self.dests_active[j] = False
                        self.drones_capacity[i] -= 1
                        self.drones_timer[i] = self.unload_time
                        team_reward += 10.0
                        break

            # Fix 8: 보급 보상 +50 → +30 (조기 귀환 유인 완화)
            elif self.drones_capacity[i] == 0 and self.drones_timer[i] == 0:
                if np.linalg.norm(self.drones_pos[i] - self.cc_pos) <= self.interaction_radius:
                    self.drones_capacity[i] = self.max_capacity
                    self.drones_timer[i] = self.unload_time
                    team_reward += 30.0
                    print("드론이 본부에 돌아와 보급을 받았습니다!")

        # Fix 4, 5: 릴레이 간격 보상 — 용량 조건 제거, 크기 강화
        for i in range(self.num_drones):
            for j in range(i + 1, self.num_drones):
                dist = np.linalg.norm(self.drones_pos[i] - self.drones_pos[j])
                if dist < 150.0:
                    rewards[i] -= 0.05   # Fix 5: -0.01 → -0.05
                    rewards[j] -= 0.05
                elif 300.0 <= dist <= 480.0:
                    rewards[i] += 0.05   # Fix 4: +0.01 → +0.05
                    rewards[j] += 0.05

        rewards += team_reward
        done = not np.any(self.dests_active)
        self.prev_target_dists = self._get_target_dists()
        if self.render_mode == "human":
            self.render()
        return self._get_local_obs(), self._get_global_state(), rewards, done

    # Fix 6: 관측치에 두 번째 이웃 드론 추가 (15 → 19차원)
    # 새 구조: [cc_dist, my_comm, my_cap, is_delivering,       # 4
    #           target_dist, target_dir(2),                     # 3
    #           peer1(dist, dir_x, dir_y, comm),                # 4
    #           peer2(dist, dir_x, dir_y, comm),                # 4
    #           drone_id one-hot]                               # 4  → 합계 19
    def _get_local_obs(self):
        obs = []
        for i in range(self.num_drones):
            cc_dist = np.linalg.norm(self.drones_pos[i] - self.cc_pos) / self.map_size[0]
            my_comm = 1.0 if self.comm_status[i] else 0.0
            my_cap = self.drones_capacity[i] / self.max_capacity
            is_delivering = 1.0 if self.drones_capacity[i] > 0 else 0.0
            target_dist = self.prev_target_dists[i] / self.map_size[0]

            if self.drones_capacity[i] > 0:
                active_dests = self.dests_pos[self.dests_active]
                if len(active_dests) > 0:
                    raw_vec = (
                        active_dests[np.argmin(np.linalg.norm(active_dests - self.drones_pos[i], axis=1))]
                        - self.drones_pos[i]
                    )
                else:
                    raw_vec = np.array([0.0, 0.0])
            else:
                raw_vec = self.cc_pos - self.drones_pos[i]

            dist_raw = np.linalg.norm(raw_vec)
            target_dir = (raw_vec / dist_raw) if dist_raw > 0 else np.array([0.0, 0.0])

            # Fix 6: 거리 순 정렬 후 가장 가까운 2대 관측
            other_dists = sorted(
                [(np.linalg.norm(self.drones_pos[i] - self.drones_pos[j]), j)
                 for j in range(self.num_drones) if j != i]
            )
            peer_features = []
            for k in range(2):
                nj = other_dists[k][1]
                peer_raw_vec = self.drones_pos[nj] - self.drones_pos[i]
                peer_dist_raw = other_dists[k][0]
                peer_dist = peer_dist_raw / self.map_size[0]
                peer_dir = (peer_raw_vec / peer_dist_raw) if peer_dist_raw > 0 else np.array([0.0, 0.0])
                peer_comm = 1.0 if self.comm_status[nj] else 0.0
                peer_features.extend([peer_dist, peer_dir[0], peer_dir[1], peer_comm])

            drone_id = np.zeros(self.num_drones)
            drone_id[i] = 1.0

            agent_obs = np.array([
                cc_dist, my_comm, my_cap, is_delivering,
                target_dist, target_dir[0], target_dir[1],
                *peer_features,
                *drone_id
            ])
            obs.append(agent_obs)
        return np.array(obs)

    def _get_global_state(self):
        return np.concatenate((
            self.drones_pos.flatten() / self.map_size[0],
            self.comm_status,
            self.drones_capacity / self.max_capacity,
            self.dests_active
        ))

    def render(self):
        if self.render_mode != "human":
            return
        if self.screen is None:
            pygame.init()
            pygame.font.init()
            self.screen = pygame.display.set_mode(self.map_size)
            self.clock = pygame.time.Clock()
            self.radius_surface = pygame.Surface(self.map_size, pygame.SRCALPHA)
            self.font = pygame.font.SysFont('Arial', 14, bold=True)

        self.screen.fill((255, 255, 255))
        self.radius_surface.fill((0, 0, 0, 0))
        pygame.draw.circle(self.radius_surface, (0, 0, 255, 30), self.cc_pos.astype(int), int(self.comm_range))
        self.screen.blit(self.radius_surface, (0, 0))

        for i in range(self.num_drones):
            pygame.draw.circle(self.screen, (0, 255, 0), self.drones_pos[i].astype(int), int(self.comm_range), 2)
        for i in range(self.num_drones):
            if np.linalg.norm(self.drones_pos[i] - self.cc_pos) <= self.comm_range:
                pygame.draw.line(self.screen, (150, 150, 150), self.cc_pos, self.drones_pos[i], 2)
            for j in range(i + 1, self.num_drones):
                if np.linalg.norm(self.drones_pos[i] - self.drones_pos[j]) <= self.comm_range:
                    pygame.draw.line(self.screen, (150, 150, 150), self.drones_pos[i], self.drones_pos[j], 2)

        for j in range(self.num_dests):
            if self.dests_active[j]:
                pygame.draw.rect(self.screen, (255, 0, 0), (*self.dests_pos[j] - 5, 10, 10))

        pygame.draw.circle(self.screen, (0, 0, 255), self.cc_pos.astype(int), 15)
        for i in range(self.num_drones):
            color = (139, 69, 19) if self.drones_capacity[i] == 0 else (0, 200, 0)
            if not self.comm_status[i]:
                color = (0, 0, 0)
            pygame.draw.circle(self.screen, color, self.drones_pos[i].astype(int), 12)
            if self.drones_timer[i] > 0:
                pygame.draw.circle(self.screen, (255, 215, 0), self.drones_pos[i].astype(int), 16, 3)
            cap_text = self.font.render(str(int(self.drones_capacity[i])), True, (255, 255, 255))
            self.screen.blit(cap_text, cap_text.get_rect(center=self.drones_pos[i].astype(int)))

        pygame.display.flip()
        self.clock.tick(self.render_fps)


# --- 3. SAC Networks ---
# Fix 1: Self-Attention Actor를 진정한 CTDE MLP로 교체.
# 각 드론이 자신의 local obs만으로 독립적으로 행동을 결정한다.
# 이웃 정보는 환경 관측치(peer1/peer2 특징)에 이미 인코딩되어 있으므로
# MLP만으로도 릴레이 협력 행동을 학습할 수 있다.
class SACActor(nn.Module):
    def __init__(self, obs_dim, action_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 256), nn.LayerNorm(256), nn.ReLU(),
            nn.Linear(256, 256), nn.LayerNorm(256), nn.ReLU(),
        )
        self.mu = nn.Linear(256, action_dim)
        self.log_std = nn.Linear(256, action_dim)

    def forward(self, obs):
        # obs: (..., obs_dim) — 배치/드론 차원에 무관하게 마지막 차원에만 적용
        x = self.net(obs)
        mu = self.mu(x)
        log_std = torch.clamp(self.log_std(x), -20, 2)
        std = torch.exp(log_std)
        dist = torch.distributions.Normal(mu, std)
        z = dist.rsample()
        a = torch.tanh(z)
        lp = (dist.log_prob(z) - torch.log(1 - a.pow(2) + 1e-6)).sum(-1, keepdim=True)
        return a, lp


class SACCritic(nn.Module):
    def __init__(self, state_dim, act_dim):
        super().__init__()
        self.q1 = nn.Sequential(
            nn.Linear(state_dim + act_dim, 256), nn.ReLU(),
            nn.Linear(256, 256), nn.ReLU(),
            nn.Linear(256, 1)
        )
        self.q2 = nn.Sequential(
            nn.Linear(state_dim + act_dim, 256), nn.ReLU(),
            nn.Linear(256, 256), nn.ReLU(),
            nn.Linear(256, 1)
        )

    def forward(self, s, a):
        sa = torch.cat([s, a], dim=1)
        return self.q1(sa), self.q2(sa)


# --- 4. Training Loop ---
if __name__ == "__main__":
    env = DisasterRelayDroneEnv()
    env.reset()

    obs_dim = env._get_local_obs().shape[-1]      # 19
    state_dim = env._get_global_state().shape[0]  # 66
    act_dim = 2
    total_act_dim = act_dim * env.num_drones       # 8

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")
    print(f"obs_dim={obs_dim}, state_dim={state_dim}, total_act_dim={total_act_dim}")

    actor = SACActor(obs_dim, act_dim).to(device)
    critic = SACCritic(state_dim, total_act_dim).to(device)
    critic_t = SACCritic(state_dim, total_act_dim).to(device)
    critic_t.load_state_dict(critic.state_dict())

    a_opt = optim.Adam(actor.parameters(), lr=3e-4)
    c_opt = optim.Adam(critic.parameters(), lr=3e-4)
    log_alpha = torch.tensor(np.log(0.1), requires_grad=True, device=device)
    alpha_opt = optim.Adam([log_alpha], lr=3e-4)

    # Alpha 그래프: 탐색이 너무 빨리 수렴 → target_entropy를 0.6배로 완화
    # -8 → -4.8: 정책이 더 오래 다양한 행동을 유지하도록 유도
    target_entropy = -float(total_act_dim) * 0.6

    buffer = EfficientReplayBuffer(env.num_drones, obs_dim, state_dim, act_dim, capacity=100000)
    reward_norm = RewardNormalizer()  # Critic 스파이크 완화용 보상 정규화기

    writer = SummaryWriter(log_dir="runs/DisasterRelay_MARL")
    os.makedirs("weights", exist_ok=True)

    total_env_steps = 0
    update_count = 0  # Policy delay 카운터

    for episode in range(1, 5001):
        local_obs = env.reset()
        global_state = env._get_global_state()
        ep_reward, step, done = 0.0, 0, False

        while not done and step < 1000:
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

            # WARMUP_STEPS 이후 학습 시작: 버퍼에 다양한 샘플이 쌓인 뒤 업데이트
            if len(buffer) > WARMUP_STEPS:
                b_o, b_s, b_a, b_r, b_no, b_ns, b_d = buffer.sample(BATCH_SIZE, device=device)

                team_r = b_r.mean(dim=1, keepdim=True)  # (batch, 1)

                # 보상 정규화: -100 ~ +40 범위를 [-10, 10]으로 압축
                # Bellman 타깃 스케일 안정화 → Critic 스파이크 완화
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

                update_count += 1

                # Policy delay: Critic 2회 업데이트당 Actor 1회 업데이트
                # Actor가 불안정한 Q값을 추적하는 것을 방지
                if update_count % POLICY_DELAY == 0:
                    ca, clp = actor(b_o)
                    ca_j = ca.flatten(start_dim=1)    # (batch, 8)
                    clp_j = clp.sum(dim=1)            # (batch, 1)
                    loss_a = (alpha * clp_j - critic(b_s, ca_j)[0]).mean()
                    a_opt.zero_grad()
                    loss_a.backward()
                    a_grad_norm = torch.nn.utils.clip_grad_norm_(actor.parameters(), 1.0)
                    a_opt.step()

                    alpha_loss = -(log_alpha * (clp_j + target_entropy).detach()).mean()
                    alpha_opt.zero_grad()
                    alpha_loss.backward()
                    alpha_opt.step()

                # TAU 상수 사용: 느린 타깃 추적으로 Q-값 발산 억제
                for t, p in zip(critic_t.parameters(), critic.parameters()):
                    t.data.copy_((1 - TAU) * t.data + TAU * p.data)

                if total_env_steps % 50 == 0:
                    writer.add_scalar('Loss/Critic', loss_c.item(), total_env_steps)
                    writer.add_scalar('GradNorm/Critic', c_grad_norm.item(), total_env_steps)
                    if update_count % POLICY_DELAY == 0:
                        writer.add_scalar('Loss/Actor', loss_a.item(), total_env_steps)
                        writer.add_scalar('GradNorm/Actor', a_grad_norm.item(), total_env_steps)
                    writer.add_scalar('Alpha', alpha.item(), total_env_steps)
                    writer.add_scalar('Reward/RewardNorm_Mean', reward_norm.mean, total_env_steps)

            local_obs, global_state = next_local_obs, next_global_state
            ep_reward += rewards.mean()
            step += 1
            total_env_steps += 1

        print(f"Ep {episode} | Step {step} | Reward: {ep_reward:.2f}")
        writer.add_scalar('Reward/Episode', ep_reward, episode)
        writer.add_scalar('Steps/Episode', step, episode)

        if episode % 100 == 0:
            torch.save(actor.state_dict(), f"weights/sac_actor_ep{episode}.pth")

    writer.close()
    print("Training Complete.")