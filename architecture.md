# VLA + WMX Lookahead Controller 연동 아키텍처

RoboLab(Isaac Lab 시뮬레이션)에서 pi0.5 VLA 정책의 액션 청크를 MovenSys
WMX3 모션 컨트롤러(wmx-r2의 `lookahead_trajectory_controller`)를 통해
실행하는 시스템의 구조와, ros2_control 기반 baseline과의 차이를 설명한다.
(이 문서는 아키텍처 다이어그램 작성의 원본 자료로 쓰기 위해 구성요소,
연결 관계, 주기/포트/프로토콜을 명시적으로 서술한다.)

---

## 1. 전체 구성요소 (다이어그램의 박스들)

시스템은 크게 4개 프로세스 그룹으로 나뉜다. 각 그룹은 서로 다른
런타임(파이썬 버전/권한)에서 돌기 때문에 명확한 경계가 있다.

**(A) VLA 정책 서버** — openpi `serve_policy.py`
- pi0.5 (pi05_droid_jointpos) 모델, GPU에서 실행
- WebSocket 서버 (포트 8000)
- 입력: 카메라 이미지 2장(224×224, 어깨+손목) + 관절 위치 7 + 그리퍼 상태 1 + 텍스트 명령
- 출력: **액션 청크 (15, 8)** = 15스텝 × (관절 절대 위치 7 + 그리퍼 바이너리 1), 15 Hz 기준

**(B) RoboLab 시뮬레이션 프로세스** — Isaac Lab, Python 3.11
- Isaac Lab 환경: Franka Panda + Robotiq 그리퍼, BananaInBowlTask 씬, 카메라 2대
- 제어 주기 15 Hz (물리 120 Hz, decimation 8)
- **WmxPi0DroidJointposClient** (이번 작업의 핵심 클라이언트):
  - 매 스텝 관측을 정책 서버로 보내 청크를 받고(재추론 주기 = open_loop_horizon,
    기본 8스텝),
  - 겹치는 청크들을 **temporal ensembling**(가중 평균, te_decay=0.5)으로 융합,
  - 융합된 포인트를 **JIT(점적) 스트리밍**으로 브리지에 전달: 매 스텝
    시뮬 진행보다 **딱 2포인트만 앞서도록** 1~2개씩 공급 → 컨트롤러에
    쌓인 미실행 백로그가 항상 ≤ 2제어주기(~130 ms),
  - 팔의 액션으로는 VLA 출력이 아니라 **컨트롤러의 명령 위치**(/joint_states
    읽어온 값)를 적용 → 시뮬 팔이 컨트롤러 출력을 추종,
  - 그리퍼(바이너리)는 컨트롤러를 거치지 않고 최신 청크 값 그대로 통과,
  - 에피소드 시작 시 **preset**(컨트롤러 축을 시뮬 초기 자세로 순간 정렬) +
    1스텝 홀드(Isaac JIT 워밍업 스톨과 벽시계 스트림의 경합 차단).

**(C) TCP-ROS2 브리지** — `wmx_chunk_bridge.py`, 시스템 Python 3.12
- 존재 이유: Isaac Lab은 Python 3.11, ROS 2 Jazzy의 rclpy는 3.12 전용이라
  RoboLab 프로세스에서 rclpy를 import할 수 없음
- TCP 127.0.0.1:**5555**, 줄 단위 JSON 프로토콜 4종:
  - `chunk` → ROS 토픽 `/wmx/trajectory_chunk` (trajectory_msgs/JointTrajectory) 발행
  - `state` → 최신 `/joint_states` 관절 위치 응답
  - `preset` → `/wmx/lookahead/preset` 발행 후 수렴 대기
  - `stop` → `/wmx/lookahead/stop` 서비스 호출

**(D) wmx-r2 WMX 스택** — ROS 2 노드들(sudo, RT), WMX3 엔진
- `lookahead_trajectory_controller` (신규 개발): `/wmx/trajectory_chunk` 구독
  → WMX3 AdvancedMotion **PathIntplLookahead** 버퍼에 **실행 중 append**
  (`AddPathIntplLookaheadCommand`) — 모션을 멈추지 않고 포인트를 이어붙임
- **WMX3 RT 엔진** (`wmx3_engine`, EtherCAT/simu 플랫폼): 밀리초급 실시간
  사이클로 경로 보간·가속/저크 제한·스무딩 필터(이동평균 30 ms × 2단) 적용
- `joint_state_broadcaster`: 엔진의 축 위치를 **100 Hz**로 `/joint_states` 발행
- 내부 채널 구조 (7축 Franka 대응, WMX 채널당 최대 6축 제약 우회):
  - 채널 0: **가상 마스터 축**(위치 = 누적 궤적 시간 ms) + 관절 1–3 보조축
  - 채널 1: 가상 마스터 + 관절 4–6, 채널 2: 가상 마스터 + 관절 7
  - 마스터 경로는 항상 전진하는 일직선 → 다축 코너 감속 문제를 원천 제거,
    composite 속도 1000 units/s 상수 → 궤적 시간 그대로 실행
  - 단위: 밀리라디안 (엔진의 composite 속도 하한 1.0 unit/s 회피)
- `stopOnEmptyBuffer=false`: 포인트가 늦게 와도 에러 없이 부드럽게 대기

## 2. 데이터 흐름 (다이어그램의 화살표, 한 사이클)

```
[Isaac 카메라/관절] --(관측, 15Hz)--> [WmxPi0DroidJointposClient]
[Client] --(WebSocket, 8스텝마다)--> [pi0.5 서버] --(청크 15×8)--> [Client]
[Client: TE 융합 + JIT 1~2포인트] --(TCP :5555 "chunk")--> [브리지]
[브리지] --(/wmx/trajectory_chunk)--> [lookahead_trajectory_controller]
[컨트롤러: 실행 중 append] --> [WMX3 RT 엔진: 보간+가속/저크 제한+스무딩]
[엔진 축 위치] --(/joint_states, 100Hz)--> [브리지] --(TCP "state")--> [Client]
[Client: 액션 = 컨트롤러 명령 위치] --(15Hz)--> [Isaac 팔(PD)] --> 다시 관측으로
```

폐루프가 두 겹이다: 바깥 루프는 VLA의 시각-계획 루프(≈2 Hz 재계획),
안쪽 루프는 컨트롤러-시뮬 추종 루프(15 Hz 액션 / 100 Hz 컨트롤러 출력 /
1 kHz급 엔진 내부). JIT 스트리밍은 이 두 루프 사이의 결합 지연을
≤130 ms로 묶는 장치다.

## 3. Baseline과의 비교

### 3-1. Direct baseline (컨트롤러 없음)

VLA 청크의 각 포인트를 Isaac 팔의 위치 목표로 **그대로** 적용한다.
중간 계층이 없으므로 지연도 없고 왜곡도 없지만, **실제 로봇에는 존재할
수 없는 조건**이다(실기는 반드시 모션 컨트롤러/서보 계층을 거친다).
비교군으로서의 의미는 "이론적 상한(추종 정확도)"이다.

### 3-2. ros2_control JTC baseline (관용적 ROS 2 방식)

동일한 클라이언트·브리지 프로토콜을 쓰되(공정성), 컨트롤러만 표준
ros2_control 스택으로 바꾼 것: `ros2c_chunk_bridge.py`(TCP :**5556**)가
청크 1개를 `FollowJointTrajectory` **goal 1개**로 변환하고,
`joint_trajectory_controller`(JTC, 100 Hz, mock hardware)가 실행한다.
클라이언트는 `--wmx-full-chunk`로 청크를 통째로 보낸다(ROS 2에서의
정석 사용법 그대로).

**구조적 차이 (그림에서 대비시킬 지점):**

| 축 | WMX lookahead 경로 | ros2_control JTC 경로 |
|---|---|---|
| 재계획 반영 | **append 전용** — 선점 API 없음. 대신 JIT로 버퍼를 얕게(≤2포인트) 유지해 사실상 즉시 반영 | **goal 선점** — 새 goal이 실행 중 궤적을 교체. 반영은 즉각이나 속도 프로파일이 매번 리셋 |
| 포인트 사이 보간 | 블렌딩된 연속 경로 + 엔진이 가속/저크 한계와 2단 스무딩 필터 강제 | 위치-only 포인트의 **선형 보간** → 웨이포인트마다 속도 불연속이 구조적으로 발생 |
| 실행 주체 | RT 엔진(EtherCAT 사이클 결정성; 실기와 동일 바이너리) | 일반 커널의 비실시간 프로세스(mock HW; 실기엔 별도 RT 인터페이스 필요) |
| 스트리밍 단위 | 1~2포인트씩 상시 공급 (끊김 없는 스트림) | 15포인트 goal 단위 (재계획 시 교체) |

**측정된 차이 (동일한 실제 pi05 청크 스트림 주입, 100 Hz 통일, 명시적
시간 정렬 — vla_jit_2 실험):**

- WMX가 부드러움에서 우위: **최대 가속 -62 % (109 vs 288 rad/s²),
  RMS 저크 -64 % (243 vs 668), RMS 가속 -38 %**
- JTC가 추종 정확도에서 우위: 추종 RMSE 1.2 vs 3.4 mrad, 지연 ~0 vs ~70 ms
- 해석: WMX는 **3.4 mrad의 미세한 경로 편차와 70 ms 지연을 대가로
  저크를 1/2.7로 줄이는 스무딩 트레이드오프**를 제공한다. VLA 웨이포인트의
  노이즈(재계획 점프 포함)를 로봇 드라이브에 그대로 전달하지 않는 것이
  목적이라면 명확한 개선이다. 지연 70 ms가 태스크를 해치지 않음은 실제
  바나나 집기 에피소드 성공(판정+영상 육안 검증)으로 확인했다.
- 실제 태스크 수행 중 기록(각 1에피소드, 둘 다 성공)에서도 같은 방향:
  최대 가속 -35 %, RMS 저크 -59 %.

### 3-3. 왜 JIT가 필요했는가 (설계 서사, 그림의 "문제→해결" 패널감)

초기 구현(segment 방식: 재계획당 8포인트 일괄 append)에서는 append 전용인
lookahead 버퍼에 옛 계획이 0.5초어치 쌓여, 팔은 항상 과거 의도를
실행했다. 그 결과 그리퍼(청크 시간표대로 즉시 통과)는 팔이 목표에서
29–31 cm 떨어진 시점에 닫혀 파지가 전부 실패했다. JIT는 "선점이 안 되면
버릴 것이 없게 하라"는 발상으로 버퍼 깊이를 2포인트로 제한했고, 이후
파지가 정상 거리(14 cm)에서 일어나며 태스크가 성공한다.

## 4. 다이어그램 제안 (그리기 가이드)

1. **전체 아키텍처 그림**: 4개 그룹(A 정책 서버 / B Isaac Lab+클라이언트 /
   C 브리지 / D wmx-r2+WMX 엔진)을 박스로, 2절의 화살표를 주기·포트
   라벨과 함께. 파이썬 3.11↔3.12 경계와 sudo/RT 경계를 점선으로 표시.
2. **컨트롤러 내부 그림**: lookahead 3채널(가상 마스터+보조축 구조),
   append 스트림, 스무딩 필터, 100 Hz 브로드캐스터.
3. **비교 그림**: 왼쪽 WMX 경로(JIT 점적 → append 스트림 → 연속 프로파일)
   vs 오른쪽 JTC 경로(청크=goal → 선점 → 선형 보간 톱니 프로파일),
   아래에 측정 수치(-62 % 최대가속 / -64 % 저크 vs 추종 1.2 mrad 우위).
4. **문제→해결 그림(선택)**: segment(백로그 0.5 s, 파지 29 cm 실패) →
   JIT(백로그 130 ms, 파지 14 cm 성공).
