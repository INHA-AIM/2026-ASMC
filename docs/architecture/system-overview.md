# 팀 아키텍처 개요

> **문서 역할:** Working Overview — 현재 전체 구조를 짧게 연결하는 문서
> **문서 허브:** [README.md](../README.md)
> **상태:** Working Overview v0.4 — 상세: [운영 가이드](../operations/README.md) · [인지 융합 로드맵](../packages/asmc_perception/roadmap.md)

## 1. 한 줄 요약

**Hybrid modular (rule-first):** 룰+MPC 뼈대에 camera lane/drivable, MGeo map prior와 LiDAR occupancy를 출처가 분리된 ego-BEV에서 결합한다. 시뮬 경계는 허용 **UDP**, 모듈 내부는 **ROS**다. 세부 인터페이스는 camera live와 map pilot 결과를 확인한 뒤 확정하며 Semi-E2E는 후속 연구로 둔다.

## 2. 배포 파이프라인 (Phase B 목표)

```text
[Camera ×3] ── UDP bridge ─ camera segmentation ─ camera probability ─┐
[MGeo bake] ─ localization/crop ─ map road/lane/route/stopline ───────┼─ ego-BEV
[LiDAR VLP16] ─ UDP bridge ─ detection/tracking/occupancy ────────────┘
[GPS/IMU/status] ─ localization + blackout dead reckoning ────────────┘

ego-BEV → lane/topology + safe drivable + object/lane context
        → Behavior + Frenet → trajectory → MPC → Ctrl UDP
```

## 3. 현재 개발 단계

| 순서 | 현재 방향 |
|---:|---|
| 1 | 음영구간 3-camera shared lane-family v3·판제 인수 완료 |
| 2 | MGeo/node-link bake 자산 통합과 local crop·좌표계 검증 |
| 3 | camera v3와 map crop 실증을 바탕으로 공통 BEV/interface 합의 |
| 4 | lane topology, LiDAR occupancy와 GPS blackout 대응 연결 |
| 5 | 10월 대회판 호환·안정화 이식과 필요 시 camera domain 보강 |

과거 Phase 0/Wave 1/2 이식 과정은 [integration-plan.md](../archive/integration/wave1-wave2-plan.md)에 기록으로
보존한다. 현재 세부 gate는 [인지 융합 로드맵](../packages/asmc_perception/roadmap.md) §10을 따른다.

## 4. 모듈·담당

| 모듈 | 역할 | ASMC 경로 | 담당 |
|------|------|-----------|------|
| Expert / Behavior | rule + route, τ* 로깅 | `src/behavior_planner/` (+ LBC 학습 경로) | 정윤태·양서준 / 안승현 |
| LBC teacher BEV | privileged simulator state → GT BEV | `tools/research/lbc_teacher_bev/` | 안승현 |
| 차선·주행가능영역·eval | 카메라 학습·live 추론·음영 lane-family v3 | `src/asmc_perception/` runtime + offline/training | 안승현 |
| MGeo map prior | current Node/Link vector 후보·offline bake, pose 기반 local crop | `src/asmc_hd_map/` 후보와 기존 `src/asmc_hd_map/development/won_tae_link_node/` | 안승현·장원태 협의 |
| 공통 BEV·차로 문맥 | map/camera/LiDAR layer와 object association | `src/asmc_perception/` camera layer; LiDAR handoff 후 공통 계약 | 인지·맵·LiDAR 공동 |
| LiDAR 장애물 | clustering·OBB·tracking·카메라 보조 분류 | `src/lidar+sensorfusion/` | 손재호, 정윤태 |
| PA/SA_plan 구 학습 실험 | 현재 runtime 미사용 | Git history | 안승현 |
| Behavior + Frenet + MPC | ROS command→trajectory→control; bridge가 Ctrl UDP 전송 | `src/behavior_planner/`, `src/frenet_planner/`, `src/mpc_controller/` | 정윤태·양서준·이강태 |
| Scenario / gRPC | 시나리오·시뮬 API | `tools/aim_scenario_runner/`, `tools/grpc_inha_univ/` | 장원태 |
| Bridge | MORAI UDP 경계·내부 ROS 변환 | `src/asmc_bridge/` 단일 패키지 | 이강태, 안승현 |

## 5. Planning/Control

- Bicycle 3.0 m, δ_max 40°. MPC는 `/ctrl_cmd`, bridge가 UDP **cmd type 1 + ctrl mode 2**를 소유한다.

Planning/Control은 ROS1/catkin runtime과 transport-independent C++ core로 구성된다. 현재 v2
Behavior→Frenet→MPC 경로와 알려진 본선 준비 gap은
[Planning/Control 개발 가이드](../packages/planning-control/README.md), 실행은
[운영 규약](../operations/planning-control.md)을 따른다.

## 6. 인터페이스

현재 확정 계약과 초안을 섞지 않는다. 작업 중 camera/BEV 이름은
[인지 융합 로드맵](../packages/asmc_perception/roadmap.md) §6, 합의된 계약은
`src/interfaces/README.md`에서 관리한다. 공용 interface 변경은 **Path B**다.

## 7. 기술 선택

| 항목 | 결정 |
|------|------|
| 골격 | Hybrid modular + MPC |
| 네트워크 | **UDP 경계 + ROS 내부** (권장; mpc 이견 회의 후) |
| camera/map/LiDAR | 출처별 layer를 보존하고 실증 후 fusion 규칙 확정 |
| 인터페이스 | provisional topic만 문서화, camera live/map pilot 뒤 합의 |
| 개발판→대회판 | 9월 26.R1.h3 기능 완성, 10월 한시적 호환·안정화 브랜치 후보 |
| Semi-E2E | Phase C |
