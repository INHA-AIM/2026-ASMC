# 시스템 구조

현재 구조는 ROS1 Noetic 기반의 모듈식 인지·Behavior/Frenet·MPC입니다. MORAI 경계는 UDP이며
개발용 gRPC·Capture·GT는 학습·검증 도구에 한정합니다. 현재 팀 소유 패키지는
[저장소 지도](repository-layout.md), 담당은 [ownership](../onboarding/ownership.md)을 봅니다.

```text
MORAI 허용 센서 → asmc_bridge
  RGB 3뷰 → asmc_camera YOLO → asmc_lidar 3뷰 fusion (PR #122 기본)
  LiDAR + GPS/IMU/속도 → asmc_lidar 검출·추적·전달 필터
  GPS/IMU → planning_inputs → /planning/vehicle/state
  camera4 → asmc_camera ResNet → 신호등 typed 관측
  RGB 3뷰 → TwinLite → model-grid 투영 → 음영 boundary_family

/perception/lidar/planning_observations_map → planning_inputs → /planning/objects
차량 상태·객체·신호·local map → asmc_planner(Behavior → Execution → Frenet)
  → /planning/trajectory → asmc_controller → /ctrl_cmd/normal
음영 제어 어댑터 → /ctrl_cmd/jamming
  → asmc_planner 제어 선택기 → /ctrl_cmd → UDP bridge → MORAI
```

## 현재 실행과 목표를 구분한다

| 경계 | 현재 구현 | 남은 작업 |
|---|---|---|
| 카메라 객체 | E8 기본·사전학습 선택, TensorRT, 원본16종/소비3그룹, 3뷰 관측 | 장원태 detect 추가학습; 승현 3뷰 fusion. 3뷰 관측이 곧 3뷰 fusion은 아님 |
| 차선 | TwinLite TensorRT·전방544×960/측방480×640, model-grid→shared family | 음영 ODD에서 판제 소비·fallback 공동 확인; 순간 공급 공백 제한 추적 |
| LiDAR | `asmc_lidar` 검출·OBB·추적·3뷰 fusion·HD맵/상부 전달 필터, typed 판제 입력. 구 Main/Msg 폐기 | 승현 3뷰 연결 유지·음영 판제 기반, 재호 터널 통합·OBB/관측 위치·추적/속도. [M0–M5](../../src/asmc_lidar/docs/improvement-milestones.md) |
| 판제 | `asmc_planner` 단일 패키지, LiDAR 입력 통합 실행, MPC·재밍 선택기 | 전 구간·조건별 반복 수락, 카메라 그룹/음영 소비 정책은 판제 담당 |
| HD Map | `asmc_hd_map` global/local·mission metadata 제공 | 판제 local map copy와 출처·route parity 확인; 자동 `/hd_map/*` 전환은 아님 |
| 상태·좌표 | bridge `/vehicle/state`와 판제 전용 `/planning/vehicle/state` 분리 | 두 토픽을 같은 추정기로 오인하지 않음; 공용 geodesy/IMU 선택 코어와 패키지별 추정 정책을 구분 |
| 현장 | local UDP profile·LAN2LAN 설정과 검사 도구 | 실제 주최 환경·두 PC 네트워크·재시작·장시간 주행 수락 |

## 최신 팀 변경

[PR #89](https://github.com/INHAautonav/2026-ASMC/pull/89)는 차량·LiDAR의 true ENU→UTM 방향 정합,
GPS 위치/속도 보정, MPC 조향 반응 지연·변화 제한, 제어 선택기 watchdog과 조건부 속도 정책을 반영했습니다.
메시지·토픽 이름은 유지됩니다. 방향 보정은 두 소비 경로에 함께 적용하며 중복 회전하지 않습니다.
일반 주행 요청58km/h는 실제 최고속도 보장이 아닙니다. 신호·장애물·합류가 우선하며,
구간별 성과·남은 회귀 실패는 [판제 운영](../operations/planning-control.md)에 구분합니다.

## 대회 마무리 우선순위

먼저 단일 producer·정확한 좌표/시각·실패 시 정지·전 구간 반복주행을 고정합니다.
패키지 이전과 모델 개선은 그 기준선을 보존한 작은 변경으로 진행합니다.
새 BEV 융합·seg 객체 학습·연구 모델 비교는 필수 주행 경로를 지연시키지 않는 후속입니다.
담당별 결과·수락 기준은 [통합 로드맵](consolidation-roadmap.md)에 있습니다.
