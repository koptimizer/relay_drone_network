# 문헌 검토: 릴레이 드론 네트워크 기반 다중 드론 배송의 MARL 접근

**Slug:** `marl-relay-drone-delivery` · **작성일:** 2026-08-24 · **대상 버전:** v1 (26_08_24_22)
**상태:** researcher 2축 조사 → verifier 인용·URL 검증 → reviewer 적대적 감사까지 완료, 지적사항 반영본.
일부 주장은 이후 학습 실험으로 반증되어 정정했다(§2.2). 미해결 항목은 §7 참조.

본 문서는 `docs/recent_study_before_260824.pdf`(2026 한국통신학회 하계학술대회 발표)의 후속 작업을
위한 것으로, 두 가지 목적을 가진다. (1) 발표에서 제기한 한계를 해결할 구현 근거를 모으고,
(2) 발표의 선행연구 위치 설정이 방어 가능한지 적대적으로 검증한다.

---

## 0. 조사 범위와 방법

두 축으로 나누어 조사했다.

- **알고리즘 축**: CTDE 다중 에이전트 SAC의 critic 보상 입력, 하드 제약 강제 방식(마스킹/투영 대
  페널티), 연결성 제약 하 다중 로봇 RL, 보상 쉐이핑 병리, GNN 관측 인코더, 각도 행동 파라미터화.
- **도메인 축**: 발표의 기존 인용 검증·확장, 2022년 이후 최신 연구, 갭 주장의 적대적 검증,
  평가 지표와 베이스라인 관행, 공개 벤치마크 존재 여부.

수집은 WebSearch/WebFetch 기반이며, 인용 그래프 탐색 도구(`alpha` CLI)는 환경에 설치되어 있지 않아
사용하지 못했다. 이 제약은 §7의 미해결 질문에 영향을 준다. 초록만 확인한 자료는 본문에 명시했다.

---

## 1. 가장 중요한 발견: 갭 주장을 좁혀야 한다

발표 자료 8페이지는 "중계 드론 네트워크를 접목한 드론 배송 라우팅 연구는 매우 미비한 상황"이라고
적었다. **이 표현은 현재 형태로는 방어할 수 없다.** 인접한 두 문헌이 개념 공간의 대부분을 이미
점유하고 있다.

**(a) 임무 수행 ↔ 통신 중계 역할 전환은 다중 로봇 탐사에서 2009년부터 발표된 아이디어다.**
De Hoog, Cameron & Visser(2009)는 로봇을 explorer와 relayer로 분류하고 지정 회합점으로 조율했고
[10, 이차출처], Cesare et al.(ICRA 2015)은 explore/meet/sacrifice/**relay** 네 상태를 정의해
relay 역할 로봇이 착륙·정지하여 중계 노드로 기능하게 했다 [10, 11]. 역할 전환이라는 기제
수준에서는 본 연구의 구상과 겹친다(단, [11]은 초록 수준 확인이며, 아래 표가 보이듯 통신·적재·
목적함수 세 축 모두에서 본 연구와 다르다). 최근의 PRoID(2026)는 동일 로봇이 탐사와 중계를 겸하며
정보 전달률 예측치를 비교해 역할을 전환한다 [12, 초록].

**(b) 임무 UAV / 중계 UAV의 MARL 공동 최적화도 이미 존재한다.**
MUTTO(2024)는 Relay UAV와 Mission UAV의 궤적·송신전력을 MARL로 공동 최적화하며 지상관제소까지
다중 홉 전송을 다루고, **역할별로 분리된 보상 함수**를 쓴다 [14, 초록]. MAEN(MobiCom 2024) [15, 초록],
MRLMN [16], 그리고 발표가 이미 인용한 Ding et al.(2022) [7]이 같은 계열이다.

**실제로 비어 있는 지점.** 조사 범위 내에서, 동질 편대가 (i) 실제 물자를 적재해 다수 수요지에
배송하고, (ii) 모든 에이전트가 제어 센터에 항시 연결되도록 다중 홉 중계 사슬을 형성하며,
(iii) **배송 makespan을 최소화**하는 문제를 함께 다룬 논문은 찾지 못했다. 세 문헌이 깔끔하게
갈린다.

| 문헌군 | 통신 | 적재/재적재 | 목적함수 |
|---|---|---|---|
| 배송 라우팅 (Murray[1, 2]/Agatz[3]/Dorling[4] 계열) | 무료·미모델링 | 있음 | makespan/비용 |
| 중계·연결성 (Zeng[5]/Yanmaz[6]/Ding[7]/MUTTO[14]) | 핵심 | 없음 | throughput/coverage |
| 역할전환 탐사 (De Hoog/Cesare[11]/PRoID[12]) | 핵심 | 없음 | 정보 획득량 |

참고로 동일 드론이 배송과 부차 임무(도로 교통 센싱 등)를 동시 수행하는 **이중임무 편대** 연구도
있으나 [20], 부차 임무가 통신 중계가 아니라 센싱이라는 점에서 본 연구의 설정과는 다르다. 다만
"배송당 최대 우회 예산" 구조는 중계 의무와 배송의 트레이드오프를 정식화하는 방식으로 참고할
만하다.

**논문에 쓸 권장 표현.** "거의 연구되지 않았다"가 아니라 다음처럼 좁혀 쓰는 것이 정직하고
오히려 더 강한 주장이 된다.

> 임무 수행과 통신 중계 사이의 역할 전환은 다중 로봇 탐사 [11, 12]와 임무/중계 UAV 공동 최적화
> [6, 14]에서 확립된 개념이다. **저자들이 조사한 범위에서는(to the best of our knowledge)**
> 이를 **용량 제약이 있고, 재적재를 위한 반복 왕복이 필요하며, makespan을 최소화하는 배송
> 문제**로 옮긴 사례가 확인되지 않는다. 이 설정에서는 중계
> 의무가 적재 처리량과 직접 경쟁하며, 편대는 보급을 위해 거점으로 반복 복귀해야 한다.

### 발표 자료에서 발견된 인용 오류 4건

1. **6페이지 mFSTSP 저자 오기.** "The multiple flying sidekicks TSP: Parcel delivery with multiple
   drones (Murray & Chu, 2015)"라고 적혀 있으나, 해당 제목은 **Murray & Raj (2020)**, TR-C
   110:368–398이다 [2]. **Murray & Chu (2015)**는 다른 논문으로, 단일 드론 FSTSP/PDSTSP를 제안한
   TR-C 54:86–109이다 [1]. 20페이지 참고문헌은 2020년으로 올바르나 슬라이드 본문만 틀렸다.
   더구나 6페이지 설명("1대의 트럭이 1대의 드론을 사이드킥으로")은 2020년 논문(이종 편대)이 아니라
   2015년 논문을 서술한 것이다. **두 편 모두 인용하는 것이 맞다.**
2. **Dorling 연도 불일치.** 6페이지는 2017, 20페이지는 2016. 저널 게재는 IEEE TSMC 47(1),
   2017이고 프리프린트가 2016이다 [4]. 하나로 통일할 것.
3. **Yanmaz (2022) 중복 등재.** 20페이지 참고문헌 [2]와 [8]에 같은 항목이 두 번 있다.
4. **미인용 참고문헌.** Panda et al.(2019) [8]과 Yin et al.(2023) [9]이 참고문헌에만 있고 본문
   6–8페이지에서 인용되지 않는다. 쓰거나 빼야 한다.

또한 갭 논증 전체가 의존하는 **Choi & Cheong (2025)**(한국통신학회 학술대회논문집 94–95쪽)는
공개 URL을 찾지 못했다. 국내 학술대회 2쪽 초록이므로, 이것 하나에 갭 주장의 근거를 싣는 것은
심사에서 취약하다. 원문을 확보해 실제 정식화 내용을 확인해야 한다.

---

## 2. 설계 근거: 우리 구현 결정과 문헌 대조

### 2.1 연결성 — 보상 페널티가 아니라 하드 제약으로 (v1에서 적용 완료)

이번 조사에서 가장 정량적으로 강한 근거가 나온 항목이다.

Huang & Ontañón [32]은 μRTS에서 invalid action **masking**과 invalid action **penalty**를 통제
비교했다. 맵이 커지면 격차가 극적이다.

| 맵 크기 | 페널티 방식 (episodic return) | 마스킹 방식 |
|---|---|---|
| 4×4 | 40.0 | ~40.0 |
| 10×10 | 0.5 | ~40.0 |
| 24×24 | 0.5 | ~40.0 |

첫 보상을 찾기까지의 탐색 비용도 페널티는 전체 학습의 3.43%, 마스킹은 맵 크기와 무관하게 약
0.06%였다 [32]. 이론적으로도 마스킹은 편향을 만들지 않는다 — 로짓에 적용된 마스크는 상태 의존
미분가능 함수이므로 정책 경사 정리가 그대로 성립한다(Proposition 1) [32].

연속 행동에서의 대응물은 Dalal et al.의 **safety layer** [33]로, 안전 제약을 만족하는 최소 섭동을
해석적으로 구해 행동을 보정하며 **학습 중 제약 위반 0**을 주장한다 [33, 초록 수준].

**soft 방식의 한계를 보여주는 직접 증거**도 있다. Li, Jie, Kong & Cheng의 연결성 유지 연구 [35]는 그래프
라플라시안의 second-smallest eigenvalue(Fiedler value) $\lambda_2$를 CMDP 비용으로 쓰는데,
3대 로봇 실험 결과가 다음과 같다.

| 방법 | 과제 성공률 | 연결 유지율 | 이동 시간 |
|---|---|---|---|
| CPO | 0.05 | 0.97 | 16.68 |
| TRPO+BC | 0.90 | 0.30 | 7.85 |
| CPO+BC (제안) | 0.87 | 0.73 | 7.28 |

**어떤 soft 방법도 두 마리 토끼를 잡지 못한다.** 연결성을 97%로 올리면 과제 성공률이 5%로
붕괴한다 [35]. CLAUDE.md 2절이 "모든 드론은 임무 수행 간 통신 활성화 상태여야 한다"를 **가정**으로
못박은 이상, 이를 보상으로 다루는 것은 가정과 구현의 불일치다.

**중요한 보정 — 투영만으로는 부족하다.** Action projection 연구 [34]는 **action aliasing**을
지적한다. 여러 위반 행동이 같은 보정 행동으로 접히면 그래디언트 정보가 소실되어 critic이
평탄해진다("flat-lining critic"). 이 논문이 제시하는 완화책이 **보정 크기에 비례하는 작은
페널티를 함께 주는 것**이다 [34]. 즉 문헌의 답은 "페널티 대신 마스킹"이 아니라 **"투영 + 작은
비례 페널티"**다.

> **v1 반영 상태:** `_enforce_connectivity()`로 전원 연결이 될 때까지 원인 이동을 되돌린다.
> 단절이 구조적으로 불가능해지는 것은 검증했다(귀납: reset 시 전원이 제어 센터에 있어 연결
> 상태이고, 모든 이동을 되돌리면 직전 위치로 돌아가므로 루프는 반드시 연결 상태에서 종료한다.
> 지도 경계 클리핑은 검사 *이전*에 적용되고 검사와 대입이 같은 배열을 쓰므로 불변식이 유지된다).
>
> 다만 **처음 구현에는 두 가지 결함이 있었고 26_08_25_04에서 고쳤다.**
> (1) 동점일 때 가장 낮은 인덱스의 이동을 취소해서, 사슬을 붙잡는 앵커 드론이 페널티를 독식했다
> (20 에피소드 전부에서 0번 드론이 앵커, 평균 165회 차단). 정책이 공유 MLP + `drone_id` one-hot을
> 쓰므로 "0번은 구조적으로 손해"를 학습하게 된다. 동점을 무작위로 깨도록 바꿔 앵커 역할이
> 분산됨을 확인했다([4, 8, 6, 2] / 20ep).
> (2) 페널티가 -0.1 고정이었다. [34]가 권하는 것은 **보정 크기에 비례하는** 페널티이므로
> 취소된 변위에 비례하도록 바꿨다. 부수 효과로, 속도 0을 명령해 중계 대기 중인 드론은 보정량이
> 0이라 더 이상 처벌받지 않는다(이전에는 `timer==0`인 모든 드론이 처벌 대상이었다).
>
> 남은 차이: 우리는 이동을 **전부 취소**(all-or-nothing)하는 반면 [33]은 **최소 섭동**을 구한다.
> 최소 섭동(연결을 유지하는 최대 스텝까지만 축소)으로 바꾸면 aliasing이 더 줄어든다 — 다음 반복.

### 2.2 보상 설계 — 스텝당 양의 보상은 완주를 방해한다 (v1에서 적용 완료)

Ng, Harada & Russell [36]의 결과: 쉐이핑 항이 $F(s,a,s') = \gamma\Phi(s') - \Phi(s)$ 형태이면
최적 정책이 보존되며, **이 형태는 충분조건일 뿐 아니라 필요조건**이다. 즉 이 형태로 쓸 수 없는
쉐이핑 항은 최적 정책을 바꿀 수 있다 [36, 이차출처를 통한 정리 확인].

같은 논문이 든 고전적 실패 사례가 우리 문제와 정확히 같은 구조다 — 목표 접근에 보상을 주자
자전거가 목표 주위를 작은 원으로 맴돌고, 공 접촉에 보상을 주자 축구 에이전트가 공 옆에서 진동하듯
반복 접촉한다 [36, 38 경유]. CoastRunners에서 녹색 블록 통과 보상이 경주 대신 제자리 순환을
유도한 것도 같은 계열이다 [38]. 모바일 로봇 안전성 테스트 사례에서도 유사한 보상 해킹이
관찰되었고, 누적 보상 대신 에피소드 최대 보상으로 바꾸자 해당 현상이 사라졌다고 보고한다 [39].

**우리 구현에 대한 산술.** 구버전은 잘 벌어진 드론 쌍마다 양쪽에 스텝당 +0.05를 주었고, 시간
페널티는 드론당 -0.001이었다. 드론 4대면 6쌍이므로 이상적 대형에서 스텝당 총 +0.60이 4대에
분배되어 드론당 약 +0.15, **시간 페널티의 약 150배**다. 상한이 없으므로 배송하지 않고 대형만
유지하는 것이 강한 국소 최적이 된다. 다만 **지배 전략은 아니다** — 스텝 상한이 1000이므로
대형 유지의 상한은 드론당 약 +149인 반면 50건 완주는 약 +800이다(reviewer 감사에서 산출).
따라서 정확한 표현은 "완주보다 유리하다"가 아니라 "완주 경로를 발견하기 전에 빠지기 쉬운
국소 최적"이다. (산술은 소스 코드로부터의 추론이고,
이것이 해당하는 병리 **패턴**이 문헌의 주장이다.)

> **v1 반영 상태와 그 실패 (26_08_24_22 → 26_08_25_04):** 산개 보상을 제거하고 시간 페널티를
> -0.001 → -0.01로 올렸다. farming은 사라졌으나 **이 판단은 실험으로 반증되었다.**
> 당시 근거는 "연결이 하드 제약이므로 릴레이 사슬은 구조적으로 유지된다"였는데, 이는
> **제약 만족과 목표 달성을 혼동한 것**이다. 하드 제약은 사슬이 *끊어지는 것*만 막고 사슬을
> *만들도록* 유도하지 않는다. 제어 센터 옆에 뭉쳐 있어도 제약은 완벽히 만족된다
> (무작위 정책 12,000스텝: 홉 분포 {1: 1.0}, 위반 0으로 재현).
>
> 결과: 정책이 1홉 구성으로 붕괴했다. ep500에서 2홉 46%·3홉+ 21%였던 것이 ep2500에는
> 1홉 92%·3홉+ 0%가 되었다. 목적지 50곳 중 제어 센터 직접 반경 안에 있는 것은 15곳뿐이므로
> 배송이 그 상한에 붙어 정체했다.
>
> **수정(26_08_25_04):** 산개 유인을 potential 형태로 되살렸다.
> $\Phi(s) = w \cdot |\{(i,j) : 300 \le d_{ij} \le 480\}|$, $F = \Phi(s') - \Phi(s)$.
> 에피소드 총합이 $\Phi(\text{끝})-\Phi(\text{처음})$으로 telescoping되어 유계이므로 farming이
> 불가능하고(대형 50스텝 유지 = 시간 페널티만 누적), 사슬을 새로 뻗을 때만 1회성 보상이 나온다.
>
> **거리 쉐이핑 항에 대한 주의.** `dist_diff * scale`에 $\gamma$만 붙이면 정확한 PBRS가 된다는
> 서술은 과소평가였다. potential 자체가 목적지 소비에 따라 변하고, clip과 `timer==0` 게이팅이
> telescoping을 깨뜨린다(300스텝에 138.27의 누출로 실측). 정확한 불변성을 원하면 potential을
> 상태의 함수로 재정의해야 한다.

참고로 Devlin & Kudenko [37]는 학습 중 변하는 동적 potential로 확장하면서, 다중 에이전트에서의
보장은 정책 불변성이 아니라 **consistent Nash equilibria의 보존**임을 밝혔다 [37, 초록 수준].
우리처럼 다중 에이전트인 경우 이 구분을 인지해야 한다.

### 2.3 행동 파라미터화 — 각도 스칼라는 불연속 표현 (v1 미적용, 최우선 과제)

Zhou et al. [43]은 신경망의 회전 표현 연속성을 다루면서 **SO(2)를 정확히 우리 사례로 논한다**.
2차원 회전을 각도 $\theta \in [0, 2\pi]$ 스칼라로 표현하면 항등원에서 불연속이 발생한다 — 한쪽
방향극한은 0, 반대쪽은 $2\pi$로 정의되지 않는다 [43]. 연속 표현은 **첫 열벡터
$[\cos\theta, \sin\theta]^T$**다 [43]. 실험적으로도 불연속 표현은 연속 표현 대비 평균 오차가
6~14배 높았다(3D 회전 기준) [43].

우리 코드는 `env_acts = (a+1)*π`로 tanh 스칼라를 $[0, 2\pi]$에 사상한다. 물리적으로 동일한 헤딩
0과 $2\pi$가 행동 범위의 양 끝에 놓이므로, pre-tanh 가우시안 정책은 두 헤딩에 동시에 확률질량을
둘 수 없고 반드시 모든 중간 헤딩을 경유해야 한다.

**권장 대안(강도 순).**
1. **최소 변경**: actor가 2개 값을 내고 이를 방향 벡터로 해석(정규화 후 속도 곱).
2. **본 문제에 더 적합**: **속도 벡터** $(v_x, v_y)$를 직접 출력. 헤딩과 속도를 하나의 연속·특이점
   없는 파라미터화로 합치고 속도 헤드를 없앤다. §2.1의 연결성 투영도 벡터 공간에서 자연스럽다
   (각도 공간에서는 어색하다).
3. **보장이 가장 강한 변경**: **헤딩 이산화 + 실행불가 헤딩 마스킹**. 마스킹의 불편향 증명이
   사는 이산 행동 설정으로 들어간다 [32]. Kanervisto et al. [45]은 "연속 행동이 이산 행동보다
   학습이 어렵고 아예 학습을 막기도 하며, 이산화하면 성능이 눈에 띄게 개선된다"고 보고한다
   [45, 초록·검색요약 수준].
4. **정책 분포를 행동 다양체에 맞추는 변경**: Gaussian 대신 행동 공간의 위상에 맞는 분포를 쓰는
   방향도 있다 — 3D 회전에서는 Bingham 분포가 Gaussian 파라미터화보다 낫다고 보고된다 [44].
   2D 헤딩의 유사물은 원환(circle) 위의 von Mises 분포이나, 이번 조사에서 2D 사례를 직접
   다룬 문헌은 찾지 못했다(원리만 [44]에서 원용).

### 2.4 Critic의 보상 입력 — 팀 보상 평균화 (v1 미적용)

현재 코드는 `team_r = b_r.mean(dim=1)`로 드론별 보상을 평균해 단일 스칼라를 Bellman 타깃에 쓴다.

VDN [30]은 단일 팀 보상이 두 가지 명명된 실패를 낳는다고 지적하며 도입됐다 —
**lazy agent 문제**(한 에이전트가 유용한 정책을 배우면 다른 에이전트는 자신의 탐색이 팀 보상을
낮추므로 학습을 단념)와 **spurious reward**(팀 보상을 유발하지 않은 행동이 강화됨) [30].
QPLEX [29]와 같은 후속 value-decomposition 계열도 같은 credit assignment 문제를 다룬다.

**완화 요인으로 볼 여지가 있으나 근거가 약해 채택하지 않는다.** joint action을 학습하는 critic이
있으면 팀 보상의 영향이 작다는 취지의 서술이 [31]에 있다고 조사 단계에서 기록되었으나, 이는 본문
직접 인용이 아니라 검색 요약 수준이고 해당 초록에서 확인되지 않는다(신뢰도 중). 더구나 **MAPPO의
critic은 joint action을 받는 Q가 아니라 상태 가치 V(s)**이므로, 이 논거를 우리 joint-action critic의
근거로 그대로 옮길 수 없다. MAPPO 인용 [28]을 이 주장의 보강으로 다는 것도 부적절하다.

**그러나 같은 출처가 경고한다.** 공유 보상 방식은 **환경 규모가 커질수록** 개별 보상 방식 대비
열화하며, 격자 크기가 일정 수준을 넘으면 수렴에 실패한다고 보고한다. 격차는 과제 밀도가 높을수록
작다 [31]. **1000×1000 맵에 목적지 50개, 드론 4대는 저밀도·대규모 영역**, 즉 공유 보상이 가장
불리하다고 보고된 영역이다.

추가로, 환경이 이미 `rewards += team_reward`로 모든 드론에 팀 항을 더한 뒤 학습 루프가 다시
평균을 내므로, 팀 항은 온전히 남는 반면 **개별 쉐이핑/페널티 신호만 1/N로 희석된다.**
(이는 두 파일을 함께 읽은 추론이며 소스의 주장이 아니다.)

**권장 경로.** FACMAC [26]이 가장 위험이 낮은 상위 호환이다. 단일 $Q(s, a_{joint})$ 대신
에이전트별 효용 $Q_i(s, a_i)$를 학습하고 mixing network로 $Q_{tot}$를 구성하되, **현재의
joint-action 정책 경사는 그대로 유지**한다. 실제로 우리 actor 업데이트는 이미 FACMAC식이다 —
모든 드론의 현재 정책 행동을 critic에 넣는다. 즉 **FACMAC의 정책 경사는 있고 factored critic만
없는 상태**다. 이는 MADDPG[25]와도 다르다 — MADDPG는 에이전트마다 별도 critic을 두고 각자의
개별 보상으로 학습하는 반면, FACMAC은 팀 보상 구조를 유지한 채 critic만 factorize한다. SAC에
특화된 사례로는 mSAC[27]가 있다 — decomposed Q network와 counterfactual advantage로 credit
assignment를 부분적으로 완화한다고 보고하나, 그 결과는 이산 행동 공간(SMAC) 기준이라 본
연속 행동 설정에 그대로 옮기긴 어렵다.

### 2.5 관측 설계

연결성 유지 연구 [35]의 관측 구성: 2D 레이저 스캔 90개, 현재 속도, **로봇 로컬 프레임으로 변환된**
상대 목표 위치, **자기 기준 상대** 팀 위치. 보상은 $r_g + r_c$로, 전원 도달 시 +100,
그 외에는 $10 \times (d_{prev} - d_{curr})$, 충돌 시 -100 [35]. 하이퍼파라미터는 $\gamma$=0.99,
비용 할인 0.999, $\lambda_e$=0.1, $\eta$=0.01, 제약 임계 d=0.1 [35].

MRLMN [16]의 UAV 관측에는 **이진 연결 상태 $c^{UAV}(t)$가 명시적으로 포함**된다. 보상은 팀 항에
에이전트별 항을 더한 형태 $R^u = R + \alpha_1 R^u_{conn} + \alpha_2 R^u_{RE}$이며
**$\alpha_1 = 1$, $\alpha_2 = 3$** — 즉 중계 기여를 직접 연결 기여의 3배로 가중한다. UAV를 기지국
거리 분위수로 4개 그룹으로 나눠 그룹별로 가중치를 달리한다 [16]. **역할 분화 보상 분해**의 구체적
사례로, 우리의 배송자/중계자 역할 분담에 직접 적용 가능하다.

**시사점.** 우리 관측은 이미 자기 통신 상태와 상대 좌표를 포함하지만, **홉 수(hop count)나
$\lambda_2$ 같은 네트워크 위상 특징이 없다.** [35]와 [16]은 모두 연결 관련 특징을 관측에 명시
주입한다. 또한 [35]의 쉐이핑 계수 $10 \times$ 대 종단 $\pm 100$의 비율과 비교하면, 우리의
`dist_diff * 0.010` 대 배송 +10은 상대적으로 쉐이핑이 훨씬 약하다.

### 2.6 GNN 인코더 — 기대를 낮춰야 한다

DGN [40]은 가장 구체적인 아키텍처 명세를 준다. 에이전트가 노드, 노드 특징은 로컬 관측의 인코딩,
간선은 **거리 기반**으로 동적으로 변하며 이웃 수 **K=3** 고정. 합성곱 커널은 **8-head** dot-product
attention, Q 네트워크는 **모든 합성곱 층의 특징을 연결**해서 받는다. 시간적 관계 정규화로
$t$와 $t+1$의 어텐션 분포 간 KL 손실을 보조로 더한다 [40]. 성능은 30×30 격자에서 Battle 0.91
(MFQ 0.70, CommNet 0.03, DQN -0.03), Routing(N=20) 0.49 (MFQ 0.18) 등 [40].

**핵심 시사점: 보고된 GNN 이득은 에이전트 20대 이상 영역에 집중되어 있다** [40, 42]. 드론이
4대뿐인 우리 설정에서 K=3은 사실상 완전그래프이므로, GNN이 주는 것은 확장성이 아니라
순열 불변성과 동적 간선 구조 정도다. 이전에 self-attention actor를 MLP로 되돌린 결정은
N=4에서는 합리적이었다고 볼 근거가 된다(추론).

**GNN을 쓴다면 대상은 peer가 아니라 목적지여야 한다.** UPDeT [41]의 entity-set 표현이
"목적지 50개 + 드론 4대 + 거점 1개"라는 가변 길이 엔티티 집합을 인코딩하는 올바른 방식이다
[41, 초록 수준].

현재 관측에 대한 정확한 진단은 다음과 같다. `peer1/peer2` 슬롯은 거리순으로 정렬되어 있으므로
그 자체로 순열 불변성을 깨지 않는다(정준 순서 부여). 순열 불변성을 실제로 깨는 것은 **`drone_id`
one-hot**이다. 그리고 이 관측의 진짜 결함은 따로 있다 — **각 드론이 3명의 동료 중 2명만 본다.**
드론 4대로 3홉 사슬을 만들려면 세 동료의 상대 배치를 동시에 알아야 하는데, 현재 관측으로는
3자 중계 관계를 표현할 수 없다. 1홉 붕괴의 구조적 원인 중 하나일 수 있다(추론).

**CLAUDE.md 3절의 GNN 방향에 대한 경고.** 배송+중계 설정에서의 GNN 시공간 관측 인코딩에 대한
검증된 선행 사례는 찾지 못했으나, **그래프 어텐션 인코더 자체는 2021–2023년 이후 다중 에이전트
라우팅에서 표준**이므로 [17, 40] 이를 novelty로 주장해서는 안 된다.

---

## 3. v1 구현 진단 대조표

| 항목 | 문헌이 지적하는 문제 | 근거 | v1 상태 |
|---|---|---|---|
| 통신 단절 -100 + 강제 종료 | 대형 종단 페널티는 탐색·가치학습을 망친다. CMDP 경로도 연결률 73%에서 정체 | [32][34][35] | **해결** — 하드 제약. 구버전 결함(하역 중 드론 유기 시 회피 불가능한 -100)을 재현·해소 확인 |
| 산개 보상 +0.05/쌍/스텝 대 시간 -0.001 | 비-potential 스텝당 양의 보상 → 순환·완주 회피 | [36][38] | **1차 시도 실패 → 재수정** — 단순 제거는 1홉 붕괴를 낳았다(§2.2). potential 형태로 대체 |
| 배송 완료·makespan 미측정 | makespan은 이 분야의 표준 목적함수이자 보고 지표 | [17][18][21] | **부분** — 배송 수·균형은 해결. makespan은 50곳 전량 완주 시에만 기록되어 현 성능대에서는 사실상 미기록(생존 편향). `t_last_deliv`를 추가로 기록 |
| 중계 도달 범위 미측정 | 하드 제약에서 AUR은 항상 1.0이라 무의미 (§4.1) | 실측 | **해결(26_08_25_04)** — 홉 깊이 2+ 비율, 최대 도달 거리 추가 |
| 차단 페널티가 앵커 드론에 편중 | 중계 역할 수행자가 구조적으로 처벌 | 실측 | **해결(26_08_25_04)** — 타이브레이크 무작위화 |
| 차단 페널티가 고정값 | [34]는 보정 크기 비례를 권고 | [34] | **해결(26_08_25_04)** — 변위 비례로 변경 |
| 각도 스칼라 헤딩 | SO(2) 불연속 표현 | [43] | **미해결 — 최우선** |
| `team_r.mean()` | 개별 신호 1/N 희석, 저밀도·대규모에서 공유 보상 열화 | [30][31] | 미해결 |
| 단일 $Q(s,a_{joint})$ | FACMAC factored critic이 연속 다중 에이전트에서 우위 | [26] | 미해결 |
| 관측에 위상 특징 없음 | 연결 상태·홉수·$\lambda_2$를 명시 주입하는 것이 관행 | [16][35] | 부분 (자기 연결 상태만) |
| 균일 보상(역할 미분화) | 중계 기여를 3배 가중하는 역할 분화 사례 | [16] | 미해결 |
| 이동 전체 취소(all-or-nothing) | 최소 섭동 투영이 aliasing을 줄임 | [33][34] | 미해결 |

---

## 4. 평가 설계

### 4.1 채택할 지표

**배송 측** — makespan(에이전트별 완료시간의 최댓값)이 표준이며 ScheduleNet과 iMTSP가 직접
보고한다 [17][18]. 보고 형식은 **참조해 대비 비율**이 가장 깔끔하다. ScheduleNet은 LKH3를 1.00으로
두고 상대 makespan을 보고한다 [17]. 계산 시간은 인스턴스 크기 대비 로그축으로 함께 보고한다 [17].

**통신 측** — 주의가 필요하다. **Available UAV Ratio**($\frac{1}{U}\sum_u c_u^{UAV}$) [16]는
연결을 soft하게 다루는 MRLMN에서는 의미가 있으나, **우리처럼 연결을 하드 제약으로 강제하면
정의상 항상 1.0이 되어 아무 정보도 주지 않는다**(무작위·적대적 정책 모두에서 1.0000으로 실측).
실제로 이 때문에 1홉 붕괴가 2500 에피소드 동안 발견되지 않았다 — 계획된 지표가 전부
"끊겼는가"만 물었고 "얼마나 멀리 뻗었는가"를 묻지 않았다.

따라서 하드 제약 설정에서 보고해야 할 통신 측 지표는 **도달 범위**다: 홉 깊이 2 이상인
드론-스텝 비율, 에피소드 최대 도달 거리, 그리고 제어 센터 직접 반경 밖 목적지의 배송 비율.
AUR은 제약이 실제로 지켜졌는지 확인하는 sanity check로만 쓴다.

**드론 간 작업 분배 공정성** — 이번 조사 범위에서 Jain index류 공정성 지표를 보고한 문헌은 찾지
못했다. min-max/makespan 목적이 이 분야의 암묵적 공정성 대리지표다. CLAUDE.md 3절이
"학습 불균형"을 관찰된 실패로 명시한 만큼 드론별 작업량 분산을 명시 보고하는 것은 의미가 있으나,
**novelty가 아니라 진단적 보고로 제시해야 한다.**

**추론 지연** — RL 라우팅 논문들은 종단 해결 시간을 보고한다. iMTSP는 1000-도시 인스턴스에
1.35–4.85초를 보고한다 [18]. 실시간 제어 우위를 주장하려면 **스텝당 forward-pass 지연을 ms 단위로**
보고해야 하며, 이는 더 엄격하고 덜 보고되는 수치다.

### 4.2 구현해야 할 베이스라인

문헌 관행상 필수 비교군은 다음과 같다 [17][18][19].

1. **LKH3** — min-max mTSP에서 사실상 최적의 대리 [17].
2. **Google OR-Tools** 라우팅 [17][18].
3. **Gurobi/CPLEX exact MILP** — 소규모 인스턴스 한정 [17].
4. **다른 RL** — ScheduleNet [17], DAN, MAPPO 등.
5. **비학습 중계 베이스라인** — Yanmaz의 Steiner point 기반 중계 배치 [6]는 강한 비학습 대조군이며,
   인용만 하지 말고 **구현해서 비교해야 한다.** RELINK [10, 이차출처]도 실시간으로 도는 최적화
   기반 대안이므로 "MILP는 느리다"로 뭉뚱그릴 수 없다.

**중요한 단서 — 위 1~4는 head-to-head 비교군이 아니다.** LKH3·OR-Tools·CPLEX·ScheduleNet은
모두 용량 제약, 재적재, 하역 시간, 통신 연결이 **없는** min-max mTSP를 푼다. 따라서 이들은
"제약을 모두 풀었을 때의 하한"을 주는 **완화 참조 상계(relaxed reference bound)**로만 쓸 수 있고,
같은 문제를 푸는 경쟁자로 제시하면 안 된다. 실제로 같은 문제를 놓고 겨룰 수 있는 비학습
대조군은 5번(Yanmaz의 Steiner point 중계 배치 [6])뿐이므로, §6 로드맵에서 7순위로 미뤄둔 것은
재고해야 한다.

**ScheduleNet(AAMAS 2023) [17]은 반드시 인용해야 할 누락 문헌이다.** min-max mTSP를 에피소드 보상
MDP로 정식화하고 type-aware graph attention으로 푼다 — 목적함수가 우리와 동일하고 통신 제약만
없으므로, **"중계를 무시하면 어떻게 되는가"의 자연스러운 ablation**이다.

### 4.3 MILP 비교 대상 규모 — 인용 가능한 수치

- **트럭+드론 1대 정확해 한계:** Dell'Amico et al.은 벤치마크를 **고객 10명 72개 인스턴스**와
  **고객 9명 100개 인스턴스**로 구성하며 "이보다 큰 인스턴스는 증명된 최적해를 구하는 것 자체가
  이미 어려워서 선택하지 않았다"고 명시한다. Gurobi 9.1.1, i5-7200U에서 **수 초~2시간** [21].
- **min-max mTSP + CPLEX:** ScheduleNet 표에서 CPLEX 증명 최적해는 eil51(m=2), eil76(m=2)에만
  표시되고, 더 큰 m과 berlin52/rat99는 best-known 상계만 보고된다 [17]. 즉 드론도 통신도 없는
  순수 min-max mTSP조차 **도시 50–75개, 에이전트 2대**를 넘으면 정확해가 어렵다.
- **OR-Tools 열화:** LKH3 대비 평균 makespan 격차가 30–200 도시에서 1.12, {300×30, 500×50,
  700×50}에서 3.45로 커진다 [17]. iMTSP는 OR-Tools가 600 도시 이상에서 300초 내 수렴하지 못한다고
  보고한다 [18].

**정직한 프레이밍(CLAUDE.md 7절).** 이 수치들은 "정확해 MILP는 확장되지 않는다"를 정당화하지만
**"휴리스틱이 약하다"를 정당화하지 않는다.** LKH3는 최적에 가깝고 정확해법보다 빠르며,
ScheduleNet의 최고 설정도 **LKH3보다 4% 나쁘다** [17]. 우리가 주장할 수 있는 것은 해의 품질 우위가
아니라 **(i) 연결 그래프가 동적으로 변하는 상황에서의 온라인 재결정**과 **(ii) 스텝당 지연**이다.

### 4.4 벤치마크 — 부정적 결과

**우리 문제에 맞는 공개 벤치마크는 존재하지 않는다.** "disaster drone logistics benchmark"류 검색은
컴퓨터 비전 데이터셋(FloodNet, 3DAeroRelief, ResQ-UAV, CODrone)만 반환하며, 이는 피해 판독·인물
탐지용 영상이지 라우팅 인스턴스가 아니다. 수요량·드론 용량·재적재 시간·통신 반경 모델을 함께
담은 인스턴스 라이브러리는 찾지 못했다.

**권장 완화책:** 기존 공개 라우팅 인스턴스(Murray & Chu 10-고객 집합, Agatz 생성기)에 **통신 반경
모델을 덧씌워** 인스턴스를 만들면, 배송 기하는 표준이고 연결성 층만 새로워진다. 그러면
"중계 제약 없음" ablation이 발표된 makespan과 직접 비교 가능해진다. 생성기와 인스턴스를 논문과
함께 공개하는 것은 현재 비어 있는 니치다.

아래 공개 라우팅 인스턴스 목록은 트럭-드론 라우팅 통합 프레임워크 논문의 저장소 목록과도
일치한다 [22].

| 활용 가능한 공개 인스턴스 | URL |
|---|---|
| Murray & Chu FSTSP/PDSTSP + 최적해 | http://www.or.unimore.it/site/home/online-resources/exact-models-for-the-fstsp.html |
| mFSTSP 인스턴스 + 솔버 코드 | https://github.com/optimatorlab/mFSTSP |
| Agatz TSP-D 생성기 | https://doi.org/10.5281/zenodo.1204676 |
| Sacramento VRP-D | https://zenodo.org/records/2572764 |
| Poikonen min-makespan | https://mario.ruthmair.at/?page_id=226 |
| mTSPLib (LKH3/CPLEX 참조값) | [17]에 표로 정리됨 |

시뮬레이터로는 gym-pybullet-drones [23]가 물리 수준이며, 점질량 운동학 추상화에는 과하다.
시뮬레이터 선택 근거를 대려면 최근의 체계적 비교 조사를 참고할 수 있다 [24]. 우리 Pygame 2D
환경은 합리적 선택이나, 정직한 표현은 "표준 벤치마크"가 아니라
**"자체 환경, 오픈소스 공개"**다.

---

## 5. 통신 모델에 대한 경고

우리 환경은 반경 500m의 **디스크 모델**(거리 이내면 연결)을 쓴다. PropEM-L [13]은 학습된 전파
모델로 중계 노드를 배치하며, 디스크/반경 모델이 강한 단순화임을 보인다 [13, 이차출처].
심사자가 반드시 물을 지점이므로, 가정으로 명시하고 한계 절에서 다루는 편이 낫다.

한편 legacy MILP 모델(`legacy/milp/model.py`)은 이미 $P_0/d^\alpha$ 경로손실과 대역폭 용량으로
통신을 모델링하고 있었다. RL 환경이 이보다 단순한 모델을 쓰는 것은 일관성 문제이며, 베이스라인
비교 시 두 모델을 맞춰야 한다.

---

## 6. 구현 우선순위 로드맵

문헌 근거의 강도와 비용 대비 효과 순이다.

1. **행동 파라미터화를 속도 벡터로 변경** [43][45]. 근거가 명확하고 변경 범위가 작으며, §2.1의
   최소 섭동 투영과도 자연스럽게 결합된다.
2. **연결성 투영을 최소 섭동으로 정교화** [33][34]. 전체 취소 대신 연결을 유지하는 최대 스텝까지
   축소.
3. **쉐이핑 항을 정확한 potential 형태로 정리**($\gamma\Phi(s')-\Phi(s)$) [36].
4. **관측에 위상 특징 추가** — 제어 센터까지의 홉 수, 그래프 $\lambda_2$ [16][35].
5. **FACMAC factored critic** [26]. joint-action 정책 경사는 유지.
6. **역할 분화 보상 가중** [16].
7. **베이스라인 구현** — 먼저 LKH3/OR-Tools(배송만), 그다음 Yanmaz식 Steiner 중계 배치 [6][17].
8. **entity-set 인코더로 목적지 50개 인코딩** [41]. peer 간 메시지 패싱이 아니라 목적지 쪽.

---

## 7. 미해결 질문과 조사의 한계

1. **Choi & Cheong (2025) 원문 미확보.** 갭 논증이 의존하는 국내 학술대회 초록의 공개 URL을 찾지
   못했다. KICS/DBpia 접근 권한으로 실제 정식화를 확인해야 한다.
2. **전방 인용 탐색 미수행.** Cesare et al. 2015 [11]과 Yanmaz 2022 [6]를 인용한 후속 논문들이
   정확히 일치하는 선행연구가 있을 가능성이 가장 높은 지점이다. 인용 그래프 도구를 쓸 수 없어
   완료하지 못했다. **novelty를 주장해 투고하기 전에 반드시 수행할 것.**
3. **비영어권 문헌 미조사.** 중국어 OR 저널, 국내 저널, 국방 물류 보고서는 도달하지 못했다.
4. **초록만 확인한 자료 다수.** 출판사 차단(ScienceDirect, ACM DL, IEEE Xplore)으로 MUTTO [14],
   MAEN [15], Peng et al. [19] 등은 본문을 읽지 못했다. 논문에 인용할 때 본문 재확인 필요.
5. **인용 저자명 전수 재확인 필요.** reviewer 감사에서 [35]의 저자가 "Feng et al."로 잘못
   적혀 있던 것이 발견되었다(실제: Li, Jie, Kong & Cheng). 읽지 않은 다른 논문의 저자명이
   조사 단계에서 옮겨 붙은 것으로 보인다. 저널 투고 전 45개 출처의 저자·연도·게재지를 전수
   재확인해야 한다.
6. **N=4에서 GNN 이득 여부는 미검증 가설.** 문헌의 이득이 N≥20에 집중된다는 것은 소스의 주장이나,
   "따라서 N=4에서는 무의미하다"는 우리의 추론이다. 직접 ablation으로 확인해야 한다.

---

## Sources

**도메인 · 선행연구**

1. Murray & Chu (2015), *The flying sidekick TSP*, TR-C 54:86–109 — https://doi.org/10.1016/j.trc.2015.03.005
2. Murray & Raj (2020), *The multiple flying sidekicks TSP*, TR-C 110:368–398 — https://doi.org/10.1016/j.trc.2019.11.003
3. Agatz, Bouman & Schmidt (2018), Transportation Science 52(4):965–981 — https://doi.org/10.1287/trsc.2017.0791
4. Dorling, Heinrichs, Messier & Magierowski (2017), IEEE TSMC 47(1):70–85 — https://arxiv.org/abs/1608.02305
5. Zeng, Zhang & Lim (2016), IEEE TCOM 64(12):4983–4996 — https://doi.org/10.1109/TCOMM.2016.2611512
6. Yanmaz (2022), Ad Hoc Networks 128:102800 — https://www.sciencedirect.com/science/article/abs/pii/S1570870522000178
7. Ding, Chen, Wu, Liu, Gao & Shen (2022), IEEE TVT 71(9):10059–10072 — https://ieeexplore.ieee.org/document/9795858/
8. Panda, Das, Sen & Arif (2019), IEEE Access 7:102985–102999 — https://ieeexplore.ieee.org/document/8778653/
9. Yin, Yang, Yu, Chen & Wang (2023), IEEE T-IV 8(4):2983–2997 — https://www.researchgate.net/publication/367194502_An_Air-to-Ground_Relay_Communication_Planning_Method_for_UAVs_Swarm_Applications
10. *Multi-Robot Cooperative Exploration Survey* (2025), arXiv:2503.07278 — https://arxiv.org/abs/2503.07278
11. Cesare, Skeele, Yoo, Zhang & Hollinger (2015), ICRA 2015 — https://doi.org/10.1109/ICRA.2015.7139494
12. PRoID (2026), arXiv:2604.10433 — https://arxiv.org/abs/2604.10433
13. Clark et al. (2022), *PropEM-L*, arXiv:2205.01267 — https://arxiv.org/abs/2205.01267
14. *MUTTO*, Vehicular Communications (2024) — https://www.sciencedirect.com/science/article/abs/pii/S1570870524002130
15. *MAEN*, ACM MobiCom '24 — https://dl.acm.org/doi/10.1145/3636534.3694730
16. Xu et al., *MRLMN*, arXiv:2505.08448 — https://arxiv.org/abs/2505.08448
17. Park, Kwon & Park (2023), *ScheduleNet*, AAMAS 2023 — https://www.ifaamas.org/Proceedings/aamas2023/pdfs/p878.pdf
18. Guo, Ren & Wang (2024), *iMTSP*, arXiv:2405.00285 — https://arxiv.org/abs/2405.00285
19. Peng, Wang, Yin & Cheng (2025), TR-E 195 — https://ideas.repec.org/a/eee/transe/v195y2025ics1366554525000158.html
20. Zhu, Xu, Malikopoulos & Geroliminis (2026), arXiv:2604.02471 — https://arxiv.org/abs/2604.02471
21. Dell'Amico, Montemanni & Novellani (2021), arXiv:2107.13275 — https://arxiv.org/abs/2107.13275
22. Xu & Carlsson (2026), arXiv:2602.20310 — https://arxiv.org/abs/2602.20310
23. gym-pybullet-drones — https://github.com/utiasDSL/gym-pybullet-drones
24. *Survey of Simulators for Aerial Robots*, arXiv:2311.02296 — https://arxiv.org/abs/2311.02296

**알고리즘 · 학습 방법**

25. Lowe et al. (2017), *MADDPG*, NeurIPS 2017 — https://arxiv.org/pdf/1706.02275
26. Peng et al. (2021), *FACMAC*, NeurIPS 2021 — https://arxiv.org/abs/2003.06709
27. Pu et al., *Decomposed Soft Actor-Critic* — https://arxiv.org/abs/2104.06655
28. Yu et al., *The Surprising Effectiveness of PPO in Cooperative Multi-Agent Games* — https://arxiv.org/abs/2103.01955
29. Wang et al. (2021), *QPLEX*, ICLR 2021 — https://arxiv.org/abs/2008.01062
30. Sunehag et al., *Value-Decomposition Networks* — https://arxiv.org/abs/1706.05296
31. Radke et al., *Towards a Better Understanding of Learning with Multiagent Teams* — https://arxiv.org/abs/2306.16205
32. Huang & Ontañón, *A Closer Look at Invalid Action Masking in Policy Gradient Algorithms* — https://arxiv.org/abs/2006.14171
33. Dalal et al., *Safe Exploration in Continuous Action Spaces* — https://arxiv.org/abs/1801.08757
34. *Safe RL using Action Projection: Safeguard the Policy or the Environment?*, arXiv:2509.12833 — https://arxiv.org/html/2509.12833v1
35. Li, Jie, Kong & Cheng, *Decentralized Global Connectivity Maintenance for Multi-Robot Navigation: A RL Approach*, arXiv:2109.08536 — https://arxiv.org/abs/2109.08536
36. Ng, Harada & Russell (1999), *Policy Invariance Under Reward Transformations*, ICML 1999 — https://people.eecs.berkeley.edu/~russell/papers/icml99-shaping.pdf
37. Devlin & Kudenko (2012), *Dynamic Potential-Based Reward Shaping*, AAMAS 2012 — https://eprints.whiterose.ac.uk/id/eprint/75121/
38. Weng, *Reward Hacking in Reinforcement Learning* — https://lilianweng.github.io/posts/2024-11-28-reward-hacking/
39. Huck et al., *RL for Safety Testing: Lessons from A Mobile Robot Case Study* — https://arxiv.org/abs/2311.02907
40. Jiang, Dun, Huang & Lu, *Graph Convolutional Reinforcement Learning* — https://arxiv.org/abs/1810.09202
41. Hu, Zhu et al. (2021), *UPDeT*, ICLR 2021 — https://arxiv.org/abs/2101.08001
42. Sun, Shen & How, *Scaling Up Multiagent Reinforcement Learning for Robotic Systems: Learn an Adaptive Sparse Communication Graph* — https://arxiv.org/abs/2003.01040
43. Zhou, Barnes, Lu, Yang & Li (2019), *On the Continuity of Rotation Representations in Neural Networks*, CVPR 2019 — https://arxiv.org/abs/1812.07035
44. James & Abbeel, *Bingham Policy Parameterization for 3D Rotations in RL* — https://arxiv.org/abs/2202.03957
45. Kanervisto, Scheller & Hautamäki (2020), *Action Space Shaping in Deep RL*, IEEE CoG 2020 — https://arxiv.org/abs/2004.00980

**직접 확인하지 못한 이차출처** (본문에 표기): De Hoog, Cameron & Visser (2009),
*Role-based autonomous multi-robot exploration* — [10]의 참고문헌에서 인용.
Xia et al. (2023), *RELINK*, IEEE RA-L 8(12):8152–8159 — [10]의 참고문헌에서 인용.
