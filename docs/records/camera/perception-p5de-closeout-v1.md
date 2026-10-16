# P5-D/E perception consumer·운영 closeout v1

> **문서 상태:** Active Closeout Record
>
> **기준일:** 2026-09-26
>
> **기계 정본:** `config/contracts/perception_runtime_acceptance_v1.yaml`

## 1. 닫은 범위

P5-C에서 동결한 기존 topic과 ROS1 message MD5를 바꾸지 않고 실제 consumer 소유권과 운영
진입점을 닫았다. LiDAR nested workspace, LAN2LAN 현장 설정과 web console은 이 범위에 포함하지
않는다.

| 경계 | closeout 결정 |
|---|---|
| 신호등 producer | `asmc_perception`이 유일한 운영 소유자 |
| 판제 신호등 입력 | typed `/perception/traffic_light/detection`이 정본 |
| 신호등 호환 출력 | `/traffic_light/state`·`detail` 유지, 모든 consumer typed 전환 뒤 삭제 검토 |
| 신호등 fail-safe | typed를 한 번 받은 뒤 invalid/stale이면 UNKNOWN, legacy로 우회 금지 |
| planning launch | producer 기본값 `false`; 독립 호환 시험에서만 `traffic_light:=true` |
| camera lane 입력 | typed `/perception/lane/boundary_family`가 정본 |
| lane 호환 출력 | 개별 `boundary_observations`는 기본-OFF |
| lane adapter | read-only reference로 유지, 판제 담당자가 흡수·삭제 시점 결정 |
| debug strip | 0~15 m 수락 시각화일 뿐 planning input이나 family 유효거리 제한이 아님 |

## 2. 인수 증거

### 신호등

- 같은 입력 순서에서 기존·canonical producer의 String/typed/source header/stale/recovery parity:
  `config/baselines/perception_traffic_light_parity_v1.yaml`
- 격리된 ROS decision loop의 green KEEP → red STOP → green KEEP:
  `config/baselines/perception_traffic_light_closed_loop_v1.yaml`
- 26.R1 제어 시뮬레이터의 Behavior→Frenet→MPC→UDP 실제 정지·재출발:
  `config/baselines/perception_traffic_light_vehicle_closed_loop_v1.yaml`

물리 차량 gate는 제어와 teleport를 수행하는 opt-in 시험이므로 일상 CI나 closeout 재검증에서
자동 실행하지 않는다. 정적 contract와 격리 decision loop를 기본 회귀로 사용한다.

### Lane family

대표 result bag은 복사하지 않고 `$ASMC_DATA` 상대 경로와 hash만 고정한다.

```text
rosbags/results/20260913_195606_kcity2025_shadow_static_avoid_return_sunny_1pm_wheel/
twinlite_v6_shadow_boundary_family_candidate_v3_profile_v1_20260915/bags/
twinlite_v6_shadow_boundary_family_candidate_v3_profile_v1_20260915_0.bag
```

| 항목 | 값 |
|---|---:|
| duration | 28.230 s |
| SHA-256 | `7ab5c78147651c5bcdd77272589988309d3ba653237729e09ab6ed253872ded8` |
| `/perception/lane/boundary_family` | 293 messages, 10.379 Hz |
| type / ROS1 MD5 | `asmc_msgs/LaneBoundaryFamily` / `70ac79f98ca1244c782ea4067becd6af` |

이 과거 A/B result bag에는 legacy `boundary_observations`도 같은 293개가 들어 있다. 이는 현재
runtime에서 legacy 출력이 필요하다는 뜻이 아니다. 현재 handoff bag profile은 typed family를
필수로, legacy observations를 선택 항목으로 둔다. adapter profile도 family와 adapter diagnostics만
필수로 요구한다.

2026-09-26 격리 ROS master에서 이 bag을 4배속 재생해 reference adapter 출력을 다시 녹화했다.
입력 family 293개 전부가 재생됐고, adapter는 accepted strip MarkerArray 282개와 diagnostics
113개를 발행했다. marker 수가 입력보다 적은 것은 invalid/freshness/support gate에서 그릴 수
없는 frame을 표시 가능한 차선으로 꾸미지 않는 fail-closed 동작이다.

## 3. 최종 운영 경계

| 작업 | 정본 |
|---|---|
| MORAI UDP와 Hz 관찰 | `./scripts/udp_bridge.sh`를 별도 foreground terminal에서 실행 |
| live·bag perception | `asmc-ros-noetic`에서 `roslaunch asmc_perception perception.launch` 직접 실행 |
| 학습·모델 검증 | `asmc-perception-train`; host ROS graph에 연결하지 않음 |
| 신호등 포함 perception | perception launch에 `start_traffic_light:=true` |
| camera rqt | `/perception/camera/debug/mosaic/compressed` |
| BEV rqt | `/perception/bev/debug/comparison/compressed` |
| lane 판제 검수 | `roslaunch lane_boundary_adapter shadow_boundary_family_candidate.launch` |
| lane RViz | 자동 실행, fixed frame `base_link`; headless는 `rviz:=false` |
| runtime graph 수락 | `./scripts/verify_perception.sh`와 필요한 명시적 consumer 옵션 |

모든 구성요소를 함께 띄운 최종 graph 검사는 다음과 같다.

```bash
./scripts/verify_perception.sh \
  --traffic-light --visualization --require-planning --require-lane-adapter
```

verifier는 프로세스를 자동 시작하지 않는다. 각 producer와 consumer가 이미 실행 중일 때만
node/type/단일 publisher/subscriber/rate/stamp/frame/staleness를 15초간 검사한다.

## 4. 검증과 다음 경계

정적 closeout validator는 P5-C/D/E 상태, consumer 정책, planning launch의 중복 producer 방지,
대표 family 메시지 수, bag profile, container와 rqt/RViz 진입점을 함께 검사한다. 이 closeout은
topic rename이나 message 수정이 아니므로 판제의 기존 canonical 구독 경로는 유지된다.

같은 날 격리 신호등 decision loop도 재실행해 typed GREEN `KEEP` → RED `STOP` → GREEN
`KEEP`과 `passed=true`를 확인했다. 이 시험은 trajectory/controller/UDP를 발행하지 않는다.

| 회귀 묶음 | 결과 |
|---|---:|
| P5 closeout·facade 정적 unit | 31 passed |
| `asmc_perception` catkin | 205 passed |
| `behavior_planner` catkin | 62 passed |
| `lane_boundary_adapter` catkin | 56 passed |
| `data_collection` Python | 31 passed |
| perception topic/contract/parity audit unit | 8 passed |

perception contract, P5-A topic policy, traffic-light artifact parity와 문서 migration 검사도 모두
통과했고 `asmc_msgs/msg` 변경은 없다.

남은 일은 다음 단계로 넘긴다.

- web console: 실제 원격 다중 사용자 수요가 생길 때까지 보류
- traffic-light legacy String: named consumer가 모두 typed로 전환된 뒤 제거 gate 수행
- `lane_boundary_adapter`: 판제가 freshness/support 변환을 흡수하고 parity를 통과한 뒤 판제 판단으로 제거
- LiDAR/fusion: 담당자 코드와 데이터의 기준선 계측부터 별도 단계로 시작
- LAN2LAN: package 통합 안정 지점과 팀원 준비 뒤 실제 두 PC 리허설
