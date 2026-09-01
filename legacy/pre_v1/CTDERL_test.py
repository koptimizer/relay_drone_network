import glob
import os
import numpy as np
import torch


def load_ctderl_module(path):
    with open(path, 'r', encoding='utf-8') as f:
        source = f.read()
    head, _sep, _tail = source.partition('# --- 4. Training Loop ---')
    if not _sep:
        raise ValueError('CTDERL.py에서 학습 루프 시작선을 찾을 수 없습니다.')
    module = {}
    exec(head, module)
    return module


def main():
    module = load_ctderl_module(os.path.join(os.path.dirname(__file__), 'CTDERL.py'))
    DisasterRelayDroneEnv = module['DisasterRelayDroneEnv']
    SACActor = module['SACActor']

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f'Using device: {device}')

    # obs_dim을 환경에서 자동 계산 (CTDERL.py와 동일한 방식)
    _probe_env = DisasterRelayDroneEnv()
    obs_dim = _probe_env.reset().shape[-1]  # 19
    act_dim = 2
    print(f'obs_dim={obs_dim}')
    actor = SACActor(obs_dim, act_dim).to(device)

    weight_files = sorted(glob.glob(os.path.join(os.path.dirname(__file__), 'weights', 'sac_actor_ep4800.pth')))
    if not weight_files:
        raise FileNotFoundError('weights 디렉토리에서 sac_actor_ep*.pth 파일을 찾을 수 없습니다.')

    model_path = weight_files[-1]
    print(f'Loading actor weights from: {model_path}')
    actor.load_state_dict(torch.load(model_path, map_location=device))
    actor.eval()

    env = DisasterRelayDroneEnv(render_mode="human")
    test_scores = []
    n_tests = 50

    for test_idx in range(1, n_tests + 1):
        local_obs = env.reset()
        total_reward = 0.0
        step = 0
        done = False

        while not done and step < 1000:
            with torch.no_grad():
                action_tensor, _ = actor(torch.FloatTensor(local_obs).to(device))
                actions = action_tensor.cpu().numpy()
            env_acts = np.stack([(actions[:, 0] + 1) * np.pi, (actions[:, 1] + 1) * 0.5], axis=1)
            local_obs, _global_state, rewards, done = env.step(env_acts)
            total_reward += np.mean(rewards)
            step += 1

        test_scores.append(total_reward)
        print(f'Test {test_idx:02d}: score = {total_reward:.2f}, steps = {step}')

    avg_score = float(np.mean(test_scores))
    print('---')
    print(f'Average score over {n_tests} tests: {avg_score:.2f}')


if __name__ == '__main__':
    main()
