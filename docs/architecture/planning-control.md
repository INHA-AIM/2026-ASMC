# Planning/Control 구조

현재 production planner는 `asmc_planner` 하나입니다. Behavior → Execution → Frenet은
내부 C++ 구조·직접 호출이며 MPC는 `asmc_controller`에 남습니다.
이전 `behavior_planner`·`frenet_planner` ROS 패키지는 삭제됐지만 C++/파라미터 namespace는 보존합니다.

## 데이터 흐름과 책임

```text
/gps + /imu → planning_inputs → /planning/vehicle/state
Main /last_lidar + /lidar/motion_debug → planning_inputs → /planning/objects
상태·객체·신호·map copy → asmc_planner(Behavior10Hz / Planning20Hz)
  → /planning/trajectory → MPC20Hz → /ctrl_cmd/normal
음영 camera family/BBox → 재밍 어댑터 → /ctrl_cmd/jamming
  → 제어 선택기 → /ctrl_cmd → bridge
```

| 모듈 | 책임 | 경계 |
|---|---|---|
| planning_inputs | GPS/IMU 상태, exact-time LiDAR wire/진단 변환 | `/planning/*` 정본 입력. bridge `/vehicle/state`와 별도 추정기 |
| Behavior | route·규정·signal/object 문맥, 기동·목표·transaction | trajectory·actuator 생성은 소유하지 않음 |
| Frenet | 기동 검증·후보·충돌/kinematic 검사·feedback | 전달된 OBB를 center/size로 임의 재구성하지 않음 |
| MPC | 최신 차량 상태에서 trajectory 추종 | ego_at_plan은 provenance, 현재 x0가 아님 |
| 제어 선택기 | NORMAL/JAMMING 선택·watchdog·최종 단일 `/ctrl_cmd` | 선택 입력250ms 단절/무효 모드는 제동 |
| 재밍 어댑터 | family/BBox 기반 음영 후보·조향 | read-only monitor와 제어 launch는 서로 다름 |

통합 launch는 LiDAR 입력이 기본입니다. `planner_v2.launch` 단독의 기본은 개발용 `/Object_topic`이므로
sensor-only 실행에는 detected 입력·판제 상태/객체 토픽을 명시합니다.
통합 launch가 모델·bridge·Main을 모두 띄우는 것은 아닙니다. [실행 순서](../operations/commands.md)를 따릅니다.

## 좌표·시간·권한

상태·객체·trajectory는 map frame과 SI 단위를 사용합니다. 판제 차량 상태와 Main의 map 객체는
true ENU→UTM grid yaw 보정을 일치시켜야 합니다. 이미 grid yaw인 producer에는 중복 보정하지 않습니다.
source stamp와 steady receive freshness는 별개이며 실제 입력 중단을 재발행으로 가리지 않습니다.
그룹 이름은 물체의 이동 가능성이고 실제 운동 판정·velocity validity는 추적 근거입니다.

일반 구간은 local HD맵 경로, 차선 주행 소비는 음영 ODD입니다. 카메라가 구간을 판정하거나
controller mode를 정하지 않습니다. `asmc_hd_map` 제품과 판제 map copy의 parity는 별도 확인합니다.

## 최신 #89와 남은 검증

#89는 일반 KEEP의 조건부58km/h, 지정7개 도로의 KEEP 곡선 설정, MPC0.15초 조향 지연·속도별
조향 변화 한계, GPS 보정과 제어 watchdog을 반영했습니다. 신호·STOP·FOLLOW·위험·합류가 우선합니다.
세부 설정·rollback은 [Planner README](../../src/asmc_planner/README.md),
[Controller README](../../src/asmc_controller/README.md), [판제 운영](../operations/planning-control.md)을 봅니다.

PR 보고의 `roundabout_execution_test` stopline 전 commit 실패가 남아 있습니다.
단위/부분 주행 결과를 모든 maneuver·대회 전체 수락으로 확장하지 않습니다.
현재 실제 gate는 소스·등록 테스트·반복 주행으로 확인하며 옛 문서의 MERGE 미구현 문구를 적용하지 않습니다.

[리팩토링 경계](planning-control-refactoring.md) ·
[FSM/Frenet v2 계약](../contracts/planning-fsm-frenet-v2.md) ·
[음영 인계](../handoffs/camera-to-planning.md)
