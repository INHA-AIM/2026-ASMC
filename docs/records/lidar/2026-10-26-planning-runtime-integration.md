# 새 LiDAR 판제 연결과 최신 main 통합 검증

## 기준과 실행 확인

- 기능 확인 기준: `b6776464`. 사용자가 대회용 MolitComp03에서 기본 bridge 조향값
  `invert_steer=false`와 새 LiDAR 입력으로 주행이 동작함을 확인했다.
- 같은 시점의 읽기 전용 조회: `Simulator_v.S4.251001.MolitComp03_Window`, local UDP,
  `planning_objects_adapter/source_policy=asmc_lidar`, `/perception/lidar/tracks_map` 소비.
- 주행 거리·전 미션·장애물 조건과 LAN2LAN은 이 사용자 확인에서 계측하지 않았다.
- 통합 기준: main `c98db3c6`(PR #109), 통합 커밋 `f974a9a2`. 최신 노란불·제동 정책과
  기본 `state_source=competition`을 유지했다. 문서 충돌은 새 LiDAR 명령과 최신 상태 입력 설명을
  모두 보존해 해결했다. 사용자 주행 확인과 통합 후 자동 검증을 구분한다.
- 공용 조향 기본값 false를 유지한다. 별도로 확인한26R1 개발 조합만 true를 명시한다.

## 격리 검증 결과

컨테이너 `asmc-ros-noetic`에서 `/tmp/asmc-lidar-planning-build`와
`/tmp/asmc-lidar-planning-devel`을 사용했다. 운영 build/devel과 노드를 교체하거나 재시작하지 않았다.
ROS 시험은 별도 master와 합성 입력을 사용했고 실제 MORAI bridge를 시작하지 않았다.

| 검사 | 결과 |
|---|---|
| 공용 메시지·LiDAR·카메라·판제 입력·재밍 어댑터·플래너·제어기 빌드 | 통과, Main/Msg 없는 outer workspace |
| LiDAR CTest | 34/34 통과 |
| 필수 pipeline gate | 34/34 실행·통과 확인, 누락/생략 없음 |
| 판제 입력 CTest | 6/6 통과 |
| 제어기 CTest | 18/18 통과 |
| 플래너 CTest | 최초62/64 → 시험 조건 수정 후 **64/64 통과**, 252.24초 |
| 실제 LiDAR→공용 추적 관측→판제 직접 입력 | 별도 master에서16개 확인: GPS0/복구·IMU 중단/복구·reset·재시작·상태/벡터·단일 생산자 |
| 통합 launch의 ROS 경계 | 별도 master에서11개 확인: Behavior/Frenet/MPC 연결, 구 wire·validated 복구 |
| 구 Main/Msg 빌드·링크 | 별도 build/devel에서 통과, 링크29개 검사. 새 private 추적 라이브러리 자동 연결 없음 |
| 공용 메시지 직렬화 | 5개 통과, 기존 메시지 MD5 유지 |
| UDP 패킷/상태·동시 진단 | 13개/15개 통과 |
| 문서·공용/브릿지·런타임 계약 unittest | 35개 통과 |
| 인지 계약·토픽 정책·runtime acceptance unittest | 27개 통과 |
| 판제 launch·객체 필터 계약 | 각각2개 통과 |
| Main 카메라 추론 소유권 | 4개 통과 |
| 웹 | Node20에서 LiDAR68개·판제 motion 표시 검사 통과 |
| 문서 보존·링크·명령 | 동결5개/history, launch27개·명령108개 검사 통과 |

Node20은 CI의 판제 dashboard 검사 환경과 맞췄다. 테스트 도구의 구 Node10 문법 오류를 제품
회귀 실패로 집계하지 않았으며 시스템 Node를 교체하지 않았다. CI cache/concurrency의 실제
시간 개선과 hosted CI 결과는 로컬 결과와 별도로 확인한다.

## 최초 통합에서 발견한 플래너 회귀

1. `command_avoid_target_test`: 대상 소실 grace·PREPARE 후보 등116개 내부 확인 실패.
2. `roundabout_execution_test`: `safe distant candidate cannot commit six metres before the stopline` 확인 실패.

시험 수정 전 `8382abed`에서 다음 범위는 `origin/main`과 차이가 없음을 확인했다.

```bash
git diff --quiet origin/main -- \
  src/asmc_planner/CMakeLists.txt src/asmc_planner/src \
  src/asmc_planner/tests src/asmc_planner/config src/asmc_controller R_KR_PG_KATRI
```

실행 바이너리도 격리 devel의 최신 `asmc_planner_frenet_core`, execution/signal 라이브러리를
로드함을 확인했다. 이번 PR에서 회피·로터리 알고리즘이나 해당 테스트의 기대값을 바꾸지 않았다.
최신 main과 동일한 입력 범위에서 관측돼 최초 게시에서는 PR을 Draft로 유지했다.
이후 사용자의 CI 수정 요청에 따라 아래와 같이 시험 조건과 안전 계약을 대조했다.

## CI 실패와 시험 조건 수정

PR #111의 `8382abed`에 대한 hosted 실행 `40256063843`에서 실제로 실패한 것은
`planning_filter_test`의 guardrail 두 사례다. 위 로컬 플래너 두 실패와 별개다.
`__FILE__`이 `../../src/planning_inputs/test/...` 같은 상대경로로 컴파일되면 실행 디렉토리에서
catalog를 찾지 못했다. CMake가 시험 전용 절대 source 경로를 전달하도록 수정하고,
관련 없는 `/tmp`에서 실행해 28개 사례 통과를 확인했다. 운영 catalog와 필터는 수정하지 않았다.

로컬 플래너 시험은 다음 조건을 명시했다.

- `command_avoid_target_test`는 합성 lateral-offset 장면과 transaction/lifecycle 검사다.
  실제 K-City 경계 검사와 분리해 이 fixture에서만 지도·로터리를 비활성화한다.
  실제 지도 제약은 기존 `avoid_road_constraints_test`에서 그대로 검사한다.
- 같은 fixture의 미관측 유예는 기존 검사 의도인 5회로 고정한다. 운영 YAML의 정적60회·동적5회는
  별도 사례에서 마지막 허용 회차와 다음 회차의 `TARGET_OBJECT_LOST`를 각각 검사한다.
- 합성 회피 명령에 6m/s 요청 속도를 명시한다. 이를 생략하면 현재 운영 순항 속도가 적용돼
  6m/s에서 시작한 기존 장면과 다른 가속·충돌 후보가 된다. 정지로 확정한 합성 객체에는
  `LOW_THREAT`를 명시하며, 미확정 관측을 운영 코드에서 정지로 바꾸지 않는다.
- 과거 정차 탈출 사례의 시작 이격은0.451457m, 네 후보의 두 번째 후속 샘플 이격은
  0.446377–0.446379m였다. 현재 0.45m 최단거리 안전 여유 밖에서 시작해 나중에 진입하므로
  recovery 예외를 적용할 수 없다. 기존의 성공 기대를 `WAITING`·후보 무효·정확한 blocker 검사로
  바꾸고, 시작 이격과 후속 진입을 독립적인 polygon 거리 검사로 확인한다. 안전 여유는 줄이지 않는다.
- 로터리6m 전 정차 상태에서는 출발 후보가 horizon에 들어오지 않을 수 있다. READY/COMMIT 차단을
  후보 존재와 분리하고, 1m/s 접근에서는 실제 유효 후보가 있어도 조기 COMMIT할 수 없음을 추가 검사한다.

회피·충돌·로터리의 운영 코드와 설정은 이 수정에서 변경하지 않았다. CI에는 회피 fixture,
로터리 COMMIT, 실제 지도 제약, polygon 충돌의 네 CTest를 추가해 회귀를 다시 확인하도록 했다.
기존 실패 기록은 최초 상태의 근거로 보존하며 후속 검사 결과와 혼동하지 않는다.

최종 격리 검사에서는 플래너64/64·판제 입력6/6·별도 디렉토리의 필터28개가 통과했다.
문서·계약 unittest35개와 launch·필터 소유권 검사4개도 통과했다. 운영 build/devel·실행 노드는
교체하지 않았다. Hosted 결과는 [PR #111의 checks](https://github.com/INHAautonav/2026-ASMC/pull/111/checks)에서
최신 revision 기준으로 확인하며 로컬 통과와 구분한다.

기존26R1 제한 대조의 입력 freshness 정지1회도 원인 미확정으로 보존한다. 이후 통과 사례나
대회용 사용자 기능 확인으로 면제하지 않는다. timeout 확대·무효 입력의 빈 결과 대체는 하지 않았다.

## 다음 확인과 복구

공동 전 미션 인수·센서 중단/복구·속도 정확도·LAN2LAN은 별도다. 구 Main 복구 경로와
통합 기본 `validated_lidar`를 유지하며, 새 연결은 `lidar_source_policy:=asmc_lidar`로 선택한다.
3뷰 fusion·GPS 없는 터널 속도/벽/천장·구 패키지 폐기는 이번 완료 범위에 포함하지 않는다.

[현재 실행·복구 기준선](../../../src/asmc_lidar/docs/planning-runtime.md) ·
[전체 이관 인계](../../handoffs/lidar-migration.md)
