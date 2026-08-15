# 팀 아키텍처 개요

> **상태:** Draft v0.2 — 상세 스펙은 [interfaces](../src/interfaces/README.md) 참고

## 1. 한 줄 요약

**Semi-E2E:** 시뮬 privileged 정보로 Expert·GT BEV를 만들고 LBC식 Teacher–Student로 학습한 뒤, **대회 허용 센서만**으로 궤적을 예측하고 **MPC**가 최종 제어를 수행한다.

## 2. 배포 파이프라인

```
[Camera × N] ─┐
[LiDAR × 1~2]─┼─► SA_perc ─► BEV̂ ─► SA_plan ─► τ ─► MPC ─► UDP Ctrl_cmd
[GPS, IMU] ───┘
```

- **E2E 아님:** 센서에서 조향·가속을 직접 출력하지 않음
- **학습 시 privileged 사용:** Expert·GT BEV는 학습·로깅 전용. **추론 시 금지**

## 3. 모듈·담당

| 모듈 | 역할 | ASMC 경로 | 담당 |
|------|------|-----------|------|
| Expert / Behavior | rule + route, τ* 로깅 | `src/behavior_planner/` (+ LBC 학습 경로) | 정윤태·양서준 / 안승현 |
| PA | GT BEV → τ (teacher) | `src/learning_by_cheating/` | 안승현 |
| SA_perc | 센서 → BEV̂ | `src/perception/morai_3d_detection/` | 손재호, 장원태 |
| SA_plan | BEV̂ → τ | `src/learning_by_cheating/` | 안승현 |
| Frenet + MPC | UDP 경로 생성·추종 → UDP CtrlCmd | `src/frenet_planner/`, `src/mpc_controller/` | 이강태 / 정윤태·양서준 |
| Scenario / gRPC | 시나리오·시뮬 API | `tools/aim_scenario_runner/`, `tools/grpc_inha_univ/` | 장원태 |
| Bridge | ROS / UDP | `scripts/bridge.sh` (UDP 확장 예정) | 안승현, 정윤태 |

팀원별 파일 단위 가이드: [team-dev-guide.md](./team-dev-guide.md)

### PA 정의 (팀 합의)

> **"PA를 없앤다" = 배포 그래프에서 GT BEV 경로 제거.**  
> **"SA_plan ≡ PA weight successor"** 로 정의하면 설계와 구현이 일치한다.

## 4. Planning/Control 요구사항

Planning/Control은 ROS/catkin/topic 없이 standalone C++17 + UDP로 실행한다. Behavior Planner는 현재 미연동이다. 상세 정본은 [planning-control-udp.md](./planning-control-udp.md)를 따른다.

- Kinematic bicycle model (휠베이스 **3.0 m**, δ_max **40°**)
- 횡방향 오차 e_y soft constraint (차선 이탈 방지)
- 조향각·조향각속도 hard constraint
- 출력: UDP `Ctrl_cmd`, longi type **1**
- 완료 기준: Expert / PA / SA 세 소스의 τ에 대해 **동일 MPC**로 추종 성공

## 5. 인터페이스 스펙 (초안)

### 궤적 τ

| 필드 | 제안 | 확정 |
|------|------|------|
| 좌표계 | ego-centric 또는 map frame (하나로 통일) | ☐ |
| 표현 | waypoint `(x, y)` × N | ☐ |
| N | 5~10 (LBC: N_STEP=5) | ☐ |
| 시간 간격 | 0.1s (10Hz) | ☐ |

### BEV 텐서

| 필드 | 제안 | 확정 |
|------|------|------|
| 크기 | 192×192 (LBC 기본) 또는 팀 합의 | ☐ |
| pixels_per_meter | morai-roach 설정과 동일 | ☐ |
| 채널 | road, lane, vehicle, pedestrian, … (GT·pred 동일) | ☐ |
| 원점 | ego vehicle center | ☐ |

### MPC ↔ UDP

| 필드 | 값 |
|------|-----|
| 입력 | τ, current speed, yaw rate |
| 출력 | steering, accel, brake |
| UDP | `Ctrl_cmd`, longi type **1** |

인터페이스 변경은 **planning + perception + integration 3파트 합의 후** PR.

## 6. 1차 기술 선택

| 항목 | 결정 |
|------|------|
| 골격 | Semi-E2E + LBC curriculum + MPC |
| SA_perc 1차 | TransFuser 계열 |
| Expert / 맵 | MapService (K-city 2025) |
| 배포 | RuleGuard + MPC + UDP |
