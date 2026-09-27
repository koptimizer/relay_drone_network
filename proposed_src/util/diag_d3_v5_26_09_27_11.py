"""3/30 구성에서 규칙과 학습 정책의 행동 구성을 비교해 왜 규칙이 빠른지 찾는다."""
import os, sys
import numpy as np, torch
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from env.disaster_relay_env_v5_26_09_15_22 import DisasterRelayDroneEnv, GOAL_RELAY, GOAL_RETURN, GOAL_DELIVER
from model.hier_net_v5_26_09_15_23 import SetManagerActor
from pipeline.common_v5_26_09_15_22 import action_mask, chain_escape_manager, straight_worker
from util.instance_generator_v5_26_09_15_22 import sample_instance

R=os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
def learned(ck, dev, ar):
	"""집합 상위 가중치를 표본 추출 결정 함수로 감싼다."""
	m=SetManagerActor(autoregressive=ar).to(dev); m.load_state_dict(torch.load(os.path.join(R,ck), map_location=dev)); m.eval()
	def fn(e):
		o={k: torch.as_tensor(v, device=dev).unsqueeze(0) for k,v in e.manager_set_obs().items()}
		with torch.no_grad():
			act,_p,_=m(o, action_mask(e,dev).unsqueeze(0))
		return act[0].cpu().numpy()
	return fn

def run(env, mf, seed):
	"""한 인스턴스를 굴리며 역할 구성과 지표를 함께 모은다."""
	env.cc_pos, env.dests_pos = sample_instance(env.num_dests, seed=seed, num_drones=env.num_drones,
	                                            comm_range=env.comm_range, cc_pos="random")
	env.reset(); env.set_goals(mf(env))
	acc={'relay':0.0,'ret':0.0,'dest':0.0,'n':0,'switch':0,'steps':0}
	prev=env.goal_kind.copy()
	done,t=False,0
	while not done and t<env.max_steps:
		acc['relay']+=float(np.sum(env.goal_kind==GOAL_RELAY)); acc['ret']+=float(np.sum(env.goal_kind==GOAL_RETURN))
		acc['dest']+=float(np.sum(env.goal_kind==GOAL_DELIVER)); acc['n']+=1
		_o,_w,_r,done=env.step(straight_worker(env)); t+=1
		if t%20==0 or env.goal_invalid().any():
			env.set_goals(mf(env))
			acc['switch']+=int(np.sum(env.goal_kind!=prev)); prev=env.goal_kind.copy()
	s=env.episode_stats(); acc['steps']=s['steps']
	return s, acc

dev=torch.device('cpu'); torch.set_num_threads(2)
env=DisasterRelayDroneEnv(map_path="", comm_range=300.0, cluster_penalty=False, max_steps=10000,
                          deadlock_limit=10**9, no_progress_limit=10**9, num_drones=3, num_dests=30)
env.reset()
pol=(('규칙+탈출', chain_escape_manager(60,40,'shuffle')),
     ('학습 SWa', learned('weights/v5_26_09_23_18_SWa/best_manager.pth', dev, True)))
print(f"{'정책':<12}{'makespan':>9}{'중계대수':>9}{'복귀':>7}{'배송':>7}{'막힘':>8}{'정체율':>8}{'2홉+':>7}{'재적재':>7}{'목표변경':>9}")
for nm,mf in pol:
	MK=[];RL=[];RT=[];DS=[];BL=[];ST=[];HP=[];RE=[];SW=[]
	for s in range(501,541):
		st,ac=run(env,mf,s)
		if st['makespan']: MK.append(st['makespan'])
		RL.append(ac['relay']/ac['n']); RT.append(ac['ret']/ac['n']); DS.append(ac['dest']/ac['n'])
		BL.append(st['blocked']); ST.append(st['stall_ratio']); HP.append(st['hop2plus']); RE.append(st['reloads'])
		SW.append(ac['switch']/max(1,ac['steps']/20))
	print(f"{nm:<12}{np.median(MK):>9.0f}{np.mean(RL):>9.2f}{np.mean(RT):>7.2f}{np.mean(DS):>7.2f}"
	      f"{np.mean(BL):>8.0f}{np.mean(ST):>8.3f}{np.mean(HP):>7.2f}{np.mean(RE):>7.1f}{np.mean(SW):>9.2f}")
