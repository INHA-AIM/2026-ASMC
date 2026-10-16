# D2 Planning/Control·handoff 문서 통합 기록

> **상태:** 2026-09-22 통합 완료
>
> **범위:** 현재 Behavior/Frenet/MPC runtime, refactoring 기준과 파트 간 handoff

## 결과

이번 묶음에서는 단순한 링크 정리보다 중요한 현재상태 불일치를 발견했다. 기존 운영 문서는
Planning/Control을 standalone UDP-only로 설명했지만 현재 source와 package manifest는 이미
ROS1/catkin runtime으로 전환돼 있었다.

현재 실행 정본을 source·launch에 맞게 교정하고, 이전 UDP 문서 68행은
[standalone UDP 기준선](../archive/standalone-udp-baseline.md)에 내용 손실 없이 보존했다.
Planning/Control package-group guide와 handoff index를 추가했으며 기존 상세 설계·handoff
1,034행은 축약하지 않았다.

## 근거로 확인한 현재 구현

- `frenet_planner`와 `mpc_controller` package.xml은 `asmc_msgs`, `morai_msgs`, `roscpp`를 사용한다.
- 두 CMake는 ROS node와 transport-independent core를 build한다.
- Behavior v2는 `/behavior/command_v2`, Frenet은 `/planning/trajectory`와
  `/planning/frenet_feedback_v2`, MPC는 `/ctrl_cmd`를 발행한다.
- `autonomy_no_bridges_v2.launch`가 Behavior, Frenet, MPC와 state를 연결한다.
- 현재 개발 object 경로는 `/Object_topic`과 임시 `/perception/objects` adapter다.
- Behavior는 아직 package-local link/node/route JSON을 읽고 `/hd_map/*`를 직접 소비하지 않는다.

따라서 MORAI 외부 경계는 UDP지만 Planning/Control 내부를 standalone UDP라고 부를 수 없다.

## 원문과 현재 행선지

| 기존 정보 | 고유 내용 | 현재 행선지 | 처리 |
|---|---|---|---|
| 옛 `operations/planning-control.md` | standalone binary, `:9300`, 직접 Ctrl UDP와 당시 gate | `archive/planning/standalone-udp-baseline.md` | 원문 보존, historical 표시 |
| `packages/planning-control/refactoring.md` | state ownership, dependency, provenance, anti-pattern, test gate | 같은 경로 | 전체 본문 보존, 현재 build/count만 교정 |
| `handoffs/planning/behavior-legacy.md` | v1 3-package 통합 설계와 당시 결정 근거 | 같은 경로 | 전체 본문 보존, legacy banner 교정 |
| `handoffs/perception/shadow-camera.md` | camera lane/BEV→planning 인수 조건·수치 | 같은 경로 | 전체 본문 보존, index 연결 |
| `handoffs/perception/camera-object-detection.md` | detector→LiDAR model·class·학습 인수 | 같은 경로 | 전체 본문 보존, index 연결 |
| `src/planning_control/lane_boundary_adapter/README.md` | read-only adapter 정책과 평가 기록 | 같은 경로 | 전체 본문 보존, planning guide 연결 |
| `contracts/planning/fsm-frenet-v2.md` | v2 frozen transaction·rollout 상세 2,305행 | 계약 정본 유지 | 본문 미변경 |
| `archive/planning/d1-legacy-behavior-boundary.md` | legacy policy와 계산 자산 분류 424행 | 역사 분석 유지 | 본문 미변경 |

루트·architecture·operations·interface·data collection의 짧은 안내에서는 현재 ROS node/topic
명칭만 교정했다. 이전 판단 근거와 실험 수치를 줄이는 통합은 하지 않았다. 앞선 D2 보고서에
남은 standalone 전제에는 후속 교정 주석을 달아 기록 당시 판단과 현재 사실을 모두 보존했다.

## 새 진입점

| 문서 | 역할 |
|---|---|
| [Planning/Control 개발 가이드](../../architecture/planning-control.md) | package 책임, current/legacy 경계와 변경 gate |
| [Planning/Control 운영 규약](../../operations/planning-control.md) | build, launch, topic 관측과 fail-safe |
| [Handoff index](../../handoffs/README.md) | active/deployment/legacy 상태와 인수 문서 탐색 |
| [Planner README](../../../src/asmc_planner/README.md) | v1/v2 I/O, local map와 실행 |
| [MPC README](../../../src/asmc_controller/README.md) | I/O, state provenance와 fail-safe |

## 명확히 한 제한

- v2 ROS chain은 구현됐지만 `/Object_topic` oracle 때문에 현재 개발 launch는 field-safe가 아니다.
- Behavior의 local map copy와 완성된 `asmc_hd_map`은 아직 runtime에서 통합되지 않았다.
- camera lane adapter는 read-only이며 제어 권한이 없다.
- LANE_CHANGE는 제한된 validation scope이고 MERGE production gate는 닫혀 있다.
- `scripts/build_planning_control.sh`와 `run_planner_controller.sh`는 현재 source에 맞지 않는
  standalone helper이며 Phase 5 정리 대상이다.
- v1 제거는 v2 full-stack과 rollback 검증 뒤에만 가능하다.

## 변경하지 않은 것

- Behavior/Frenet/MPC source, launch, config와 algorithm parameter
- `scripts/build_ws.sh`의 동작과 인자 전달 방식(설명 주석·완료 문구만 현재 구조에 맞게 교정)
- ROS message 정의·MD5, topic name과 timeout
- 상세 v2 contract, refactoring 판단과 날짜별 handoff 수치
- privileged object adapter와 package-local map의 실제 동작
- bridge/network profile과 simulator sensor 설정

## 검증

- package.xml, CMake, launch, source publisher/subscriber와 문서를 대조
- ROS Noetic container에서 `asmc_msgs`, `morai_msgs`, `vehicle_state`, Behavior, Frenet, MPC
  관련 package build 성공
- Behavior gtest 29건, Frenet CTest 25건, MPC gtest 53건(5개 binary) 모두 통과
- 변경 Markdown 28개의 로컬 target·anchor 검사 통과
- `git diff --check`
- 빈 `docs/` 디렉터리 0개 확인
- 검증 후 catkin whitelist를 기존 `asmc_msgs;asmc_hd_map`으로 복원

다음 D2 묶음은 `records/`와 `archive/`다. 이번에 보존한 UDP baseline과 기존 날짜별 기록의
index·상태를 정리하되 상세 분석은 삭제하지 않는다.
