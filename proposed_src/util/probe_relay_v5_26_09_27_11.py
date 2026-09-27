"""중계 부족이 3/30 열세의 원인인지 확인하는 진단 탐침 (제안 아님).

학습 정책의 결정에서 '중계 역할 배정'만 규칙의 것으로 덮어쓰고 나머지는 그대로 둔다.
중계만 규칙 수준으로 늘렸을 때 makespan이 회복되면 크레딧 할당 수정이 값어치가 있다.
"""
import os, sys
import numpy as np, torch
from scipy import stats
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv
from model.hier_net_v5_26_09_15_23 import SetManagerActor
from pipeline.common_v5_26_09_15_22 import action_mask, chain_escape_manager, chain_manager, straight_worker
from util.instance_generator_v5_26_09_15_22 import sample_instance

R=os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
def learned_fn(ck, dev, ar):
	"""집합 상위 가중치를 표본 추출 결정 함수로 감싼다."""
	m=SetManagerActor(autoregressive=ar).to(dev); m.load_state_dict(torch.load(os.path.join(R,ck), map_location=dev)); m.eval()
	def fn(e):
		o={k: torch.as_tensor(v, device=dev).unsqueeze(0) for k,v in e.manager_set_obs().items()}
		with torch.no_grad():
			act,_p,_=m(o, action_mask(e,dev).unsqueeze(0))
		return act[0].cpu().numpy()
	return fn

def overlay(lf, mode):
	"""규칙이 중계를 지시한 드론만 규칙 행동으로 덮어쓴다."""
	def fn(e):
		a=lf(e).copy()
		r=chain_manager(e)
		K=e.n_cand+1                      # 중계 행동 인덱스
		m=action_mask(e, torch.device('cpu')).numpy()
		for i in range(e.num_drones):
			if mode=='force' and r[i]==K and m[i,K]:
				a[i]=K
			if mode=='both' and m[i,r[i]]:
				a[i]=r[i] if (r[i]==K or a[i]==K) else a[i]
		return a
	return fn

def run(env, mf, seed):
	"""한 인스턴스를 굴려 makespan과 막힘 지표를 돌려준다."""
	env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=seed, num_drones=env.num_drones,
	                                            comm_range=env.comm_range, cc_pos="random")
	env.reset(); env.set_goals(mf(env))
	done,t=False,0
	while not done and t<env.max_steps:
		_o,_w,_r,done=env.step(straight_worker(env)); t+=1
		if t%20==0 or env.goal_invalid().any():
			env.set_goals(mf(env))
	s=env.episode_stats()
	return (s['makespan'] if s['makespan'] else None), s['blocked'], s['stall_ratio']

dev=torch.device('cpu'); torch.set_num_threads(2)
env=DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=10000,
                          deadlock_limit=10**9, no_progress_limit=10**9, num_drones=3, num_dests=30)
env.reset()
lf=learned_fn('weights/v5_26_09_23_18_SWa/best_manager.pth', dev, True)
pol=(('학습 그대로', lf), ('중계만 규칙', overlay(lf,'force')), ('규칙+탈출', chain_escape_manager(60,40,'shuffle')))
res={}
for nm,mf in pol:
	out=[run(env,mf,s) for s in range(501,541)]
	res[nm]=np.array([x[0] if x[0] else np.nan for x in out])
	bl=np.mean([x[1] for x in out]); st=np.mean([x[2] for x in out])
	v=res[nm][~np.isnan(res[nm])]
	print(f"{nm:<12} 완주 {len(v)}/40  중앙값 {np.median(v):>5.0f}  절단평균 {stats.trim_mean(v,.1):>5.0f}  막힘 {bl:>6.0f}  정체율 {st:.3f}")
base=res['학습 그대로']; 
for nm in ('중계만 규칙','규칙+탈출'):
	m=~np.isnan(base)&~np.isnan(res[nm]); d=res[nm][m]-base[m]
	print(f"{nm} 대 학습 그대로: 쌍별 중앙 {np.median(d):+.0f} 승 {int((d<0).sum())} 패 {int((d>0).sum())} p={stats.wilcoxon(d).pvalue:.4f}")
