# 팀 아키텍처 개요

> **상태:** Draft v0.2.1 — 상세: 상위 [architecture.md](../../docs/competition/architecture.md) · [network-udp-ros.md](./network-udp-ros.md) · [perception-agentic-loop.md](./perception-agentic-loop.md)

## 1. 한 줄 요약

**Hybrid modular (rule-first):** 룰+MPC 뼈대. 안승현 인지 **후보**(차선·3캠 BEV·bake·Loc-Aid·sync)는 성능으로 채택. Det3D·TL. 시뮬 **UDP** + 내부 **ROS** ([network-udp-ros.md](./network-udp-ros.md)). Semi-E2E는 Phase C.

## 2. 배포 파이프라인 (Phase B 목표)

```
[Camera ×3+1] ─┐
[LiDAR VLP16] ─┼─► Sensor bridge ─┬─► Det3D / TL / Loc-Aid / (BEV assist)
[GPS, IMU] ────┘                  │
     UDP Ego ◄──► udp_bridge ─────┴─► Behavior FSM + Frenet → τ → MPC → Ctrl UDP
[MGeo bake] ◄── Loc-Aid pose (crop)
```

## 3. Phase 0 / Wave 2

| Phase | 내용 |
|-------|------|
| 0 (~8/7) | 개인 레포 기능 완성; ASMC=문서·안승현 인지 후보·eval |
| **1 (8/7~)** | Wave 2 이식 — [integration-plan.md](./integration-plan.md) |

## 4. 모듈·담당

| 모듈 | 역할 | ASMC 경로 | 담당 |
|------|------|-----------|------|
| Expert / Behavior | rule + route, τ* 로깅 | `src/behavior_planner/` (+ LBC 학습 경로) | 정윤태·양서준 / 안승현 |
| PA | GT BEV → τ (teacher) | `src/learning_by_cheating/` | 안승현 |
| 차선·주행가능영역·eval | 카메라 인지·평가 | `src/perception/drivable_bev/`, `tools/perception_eval/` | 안승현 |
| Camera Det3D | SparseDrive식 카메라 3D 검출 연구 | `src/perception/morai_3d_detection/` | 손재호 |
| LiDAR 장애물 | clustering·OBB·tracking·카메라 보조 분류 | `src/lidar+sensorfusion/` | 손재호, 정윤태 |
| SA_plan | BEV̂ → τ | `src/learning_by_cheating/` | 안승현 |
| Frenet + MPC | UDP 경로 생성·추종 → UDP CtrlCmd | `src/frenet_planner/`, `src/mpc_controller/` | 이강태 / 정윤태·양서준 |
| Scenario / gRPC | 시나리오·시뮬 API | `tools/aim_scenario_runner/`, `tools/grpc_inha_univ/` | 장원태 |
| Bridge | ROS / UDP | `scripts/rosbridge.sh`, `scripts/udp_bridge.sh` | 이강태, 안승현 |

## 5. Planning/Control

- Bicycle 3.0 m, δ_max 40°, UDP Ctrl_cmd **cmd type 1 + ctrl mode 2**

Planning/Control은 ROS/catkin/topic 없이 standalone C++17 + UDP로 실행한다. Behavior Planner는 현재 미연동이다. 상세 정본은 [planning-control-udp.md](./planning-control-udp.md)를 따른다.

## 6. 인터페이스

τ · tracks · TL · Loc-Aid pose — architecture §9. 변경은 **Path B**.

## 7. 기술 선택

| 항목 | 결정 |
|------|------|
| 골격 | Hybrid modular + MPC |
| 네트워크 | **UDP 경계 + ROS 내부** (권장; mpc 이견 회의 후) |
| Loc-Aid / bake / 3캠 BEV / 차선 | **후보** (동등, 성능으로 채택) |
| Semi-E2E | Phase C |
