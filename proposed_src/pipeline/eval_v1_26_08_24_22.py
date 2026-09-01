"""학습된 actor 가중치로 배송 성능을 평가하는 스크립트 (v1, 26_08_24_22).

결정론적 행동으로 여러 인스턴스를 롤아웃하고 배송률/makespan/균형 지표를 집계한다.
--render를 주면 pygame 시뮬레이션으로 기동을 육안 검증할 수 있다.
"""

import argparse
import os
import sys

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.disaster_relay_env_v1_26_08_24_22 import DisasterRelayDroneEnv
from model.sac_net_v1_26_08_24_22 import SACActor
from util.instance_generator_v1_26_08_24_22 import sample_instance

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MAX_STEPS = 1000


def rollout(env, actor, device, deterministic=True):
	"""한 에피소드를 끝까지 굴리고 (총보상, 에피소드 지표)를 반환한다."""
	obs = env.reset()
	total, step, done = 0.0, 0, False
	while not done and step < MAX_STEPS:
		with torch.no_grad():
			a, _ = actor(torch.FloatTensor(obs).to(device), deterministic=deterministic)
			a = a.cpu().numpy()
		acts = np.stack([(a[:, 0] + 1) * np.pi, (a[:, 1] + 1) * 0.5], axis=1)
		obs, _state, rewards, done = env.step(acts)
		total += float(np.mean(rewards))
		step += 1
	return total, env.episode_stats()


def main():
	"""체크포인트를 불러와 n개 인스턴스에서 평가하고 요약 통계를 출력한다."""
	p = argparse.ArgumentParser()
	p.add_argument("--run", default="v1_26_08_24_22", help="weights 하위 실행 태그")
	p.add_argument("--ckpt", default="", help="체크포인트 파일명 (기본: 최신 에피소드)")
	p.add_argument("--n", type=int, default=20, help="평가 인스턴스 수")
	p.add_argument("--render", action="store_true", help="pygame 시각화")
	p.add_argument("--stochastic", action="store_true", help="평균 대신 정책 샘플링 사용")
	args = p.parse_args()

	wdir = os.path.join(ROOT, "weights", args.run)
	if args.ckpt:
		ckpt = os.path.join(wdir, args.ckpt)
	else:
		files = [f for f in os.listdir(wdir) if f.startswith("sac_actor_ep")]
		if not files:
			raise FileNotFoundError(f"{wdir}에 sac_actor_ep*.pth가 없습니다.")
		ckpt = os.path.join(wdir, max(files, key=lambda f: int(f[12:-4])))

	device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
	print(f"device={device}  checkpoint={ckpt}", flush=True)

	env = DisasterRelayDroneEnv(render_mode="human" if args.render else None)
	obs_dim = env.reset().shape[-1]
	actor = SACActor(obs_dim, 2).to(device)
	actor.load_state_dict(torch.load(ckpt, map_location=device))
	actor.eval()

	rows = []
	for i in range(1, args.n + 1):
		# 인스턴스마다 다른 지도를 써야 정책의 일반화를 측정할 수 있다
		env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=i)
		total, st = rollout(env, actor, device, deterministic=not args.stochastic)
		rows.append(st | {"reward": total})
		print(
			f"[{i:02d}] R {total:8.2f} | 배송 {st['delivered']:2d}/50 | "
			f"makespan {st['makespan'] if st['makespan'] else '-':>5} | "
			f"미활동드론 {st['idle_drones']} | 단절 {st['comm_loss']}",
			flush=True
		)

	done_all = [r["makespan"] for r in rows if r["makespan"] is not None]
	print("---")
	print(f"인스턴스 {len(rows)}개 평균")
	print(f"  총보상          : {np.mean([r['reward'] for r in rows]):.2f}")
	print(f"  배송 완료율     : {np.mean([r['completion_rate'] for r in rows]) * 100:.1f}%")
	print(f"  전량 배송 성공  : {len(done_all)}/{len(rows)}"
	      + (f" (평균 makespan {np.mean(done_all):.0f} step)" if done_all else ""))
	print(f"  미활동 드론 수  : {np.mean([r['idle_drones'] for r in rows]):.2f}")
	print(f"  드론간 배송 편차: {np.mean([r['deliv_std'] for r in rows]):.2f}")
	print(f"  통신 단절 발생  : {sum(r['comm_loss'] for r in rows)}/{len(rows)}")


if __name__ == "__main__":
	main()
