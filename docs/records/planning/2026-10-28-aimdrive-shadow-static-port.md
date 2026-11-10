# GPS anchor와 DR로 연결한 map 정적 회피 실험

> 2026-10-29 통합: 링크만 현재 localization 이관 경로로 연결했다. 아래 실험 날짜·SHA·과거 명칭은 유지한다.

2026-10-28. [측위·재사용 분석](2026-10-28-aimdrive-shadow-map-analysis.md)의 첫 구현이다.
기준은 `feat/shadow-local-footprint`의 `6f416d46`이고 구현은 별도
`research/shadow-aimdrive-localization-20261028` worktree에 있다.
다른 local 작업과 PR127 작업의 파일·index·실행 컨테이너를 변경하지 않았다.

**기존 Frenet map 정적 회피의 SHIFT/PASS/RETURN을 연결했다.** 합성 이상 추종에서는
15km/h 초기 조건으로 우회·복귀가 가능했고, 실제 bag 입력에서도 GPS 무효 이후
DR 기반 map 경로가 local MPC까지 전달됐다. 실제 시뮬레이터의 15km/h 장애물 회피나
119m 터널 전체 폐루프 주행을 수락한 결과는 아니다.

## 연결한 코드

- [anchor.hpp](../../../src/asmc_planner/src/shadow_localization/anchor.hpp): native GPS/IMU rear-axle
  상태와 같은 source 시각의 상대 odometry를 맞춘다. 최근 1초의 map-minus-odometry
  변환을 평균하고 GPS 소실 뒤 고정한다. AIM식 anchor 구조를 현재 C++ 적분에 연결했으며
  AIMDrive Python 패키지나 외부 구현을 복사하지 않았다.
- GPS 입력은 별도 `/planning/shadow_map/gps_state`의 valid `SOURCE_GPS_IMU`만 받는다.
  기존 adapter의 0.35m GPS lever 제거, true ENU→UTM grid 보정과 정상 GPS 필터를 유지한다.
  `/Ego_topic`에서는 기존 local node와 동일하게 차속만 사용한다. GT 자차 위치·heading은
  측위 입력으로 사용하지 않는다.
- [static_planner.cpp](../../../src/asmc_planner/src/shadow_localization/static_planner.cpp): 공개
  `PrepareAvoid`/`CommitAvoid`/`PlanCommittedAvoid` 경계를 사용한다. 전체 Behavior FSM을
  시작하는 대신 이 실험이 정적 대상 선택과 기동 수명주기를 소유한다.
- 대상은 서로 다른 관측 4개 이상, 0.5초 이상, 자차 이동 보상 후 중심 변화 0.15m 이내와
  크기 안정성을 요구한다. 절대속도 unavailable만으로 정지로 간주하지 않는다. 정지 근거는
  연구용 LOW_THREAT 판단이고 `velocity_unavailable`는 보존한다. 명시적 동적 객체나
  사람·개가 가까운 앞 구간에 있으면 이 정적 전용 경로는 거부한다.
- 모든 객체의 8코너를 유지해 같은 anchor로 map 좌표로 옮긴다. 지도 밖이라는 이유로
  local 객체를 삭제하지 않는다. 경로는 다시 현재 local frame으로 변환하고 기존
  `CheckFootprint`/`CheckSweep`로 OBB·관측 노면을 검사한다. 검사된 관측 범위 끝에서
  정지 가능한 속도로 [공통 시간화](../../../src/asmc_planner/src/local/planner.cpp)를 적용한다.
- native 이동 horizon이 끝났다는 이유만으로 복귀 도중 정지시키지 않도록, 전체 suffix가
  검증됐고 끝 속도가 양수인 경우에만 곡률을 잇는 짧은 연장을 검사한다. 연장에는 native
  map의 차체 포함 검사와 local sweep을 모두 적용한다. 거부된 지점이나 native 정지점을
  넘어 연장하지 않는다. local 검사·변환이 성공한 후보만 COMMIT한다.

## 벽·천장 전처리와 기본 동작의 경계

LiDAR 소스·설정은 변경하지 않았다. 추정 pose를 LiDAR의 `map_pose.valid`나 원본 `/gps`에
주입하지 않으므로 실제 GPS 무효가 계속 터널 전처리를 선택한다. 기존 canonical map 객체
토픽의 GPS 무효 동작도 그대로다. 연구용 map 객체는 planner 내부에만 존재한다.

기본 `local.launch`는 새 옵션이 꺼져 있고 기존 planner를 사용한다. 공유한 부분은
검증된 local 경로의 시간화·제동 처리이며 기존 local core 회귀시험으로 확인했다.
이 실험은 기존의 OBB 침범 비용 기반 recovery를 켜지 않는다. 별도 가림 보조를 요청한
경우에만 기존 `near_blind_assist` 규칙과 5km/h 상한을 사용하고 `assisted`를 기록한다.
지도 정보가 가려진 공간의 관측 근거를 대신하지 않는다.

motion epoch 변경, GPS의 큰 위치/방향 innovation, anchor 부재, 시간·거리 한계에서는
anchor/기동을 거부하거나 초기화한다. hold 상한 120초·250m는 정확도 보장이 아니다.
GPS 복구 시 원본 AIM 서비스의 offset smoothing 전체를 이식한 것은 아니며, 복구 전후의
고속 경로 연속성은 추가 수락 대상이다. 실제로 유지되는 가짜 OBB도 이 측위로 해결되지 않는다.

## 검사 결과

[수치·로그 SHA 요약](2026-10-28-aimdrive-shadow-static-validation.json)을 함께 보존한다.

| 검사 | 결과와 범위 |
|---|---|
| Anchor/정적 core | 7.2km/h cap 16개, 15km/h 초기 조건 16개, 반대 방향 15km/h 17개 assertion 통과. 미래 anchor·epoch·GPS jump·시간/거리 한계·동적 객체 거부 포함. |
| 15km/h 합성 우회 | SHIFT/PASS/RETURN 이후 본차로 오차 약 0.246m 이내. 검사된 최소 local 차체 여유 약 0.570m/0.597m. 기록 센서나 실제 MPC 추종 오차를 포함한 수치가 아니다. |
| 터널 route | 기존 35개 통과. GPS 정상/무효 전환, 벽/천장, 보호 객체, 좌표 변환, 경계·잔여 구조물 검사 포함. |
| 터널 filter core | 기존 37개 통과. |
| 기존 local planner·MPC 경계 | core 회귀 및 private ROS master의 메시지·stale·중복 제어 생산자 검사 통과. |
| 엄격한 bag 재생 r02 | 약 46초 입력에서 3초 뒤 GPS 216개를 무효화. DR 판단은 수행됐지만 ego 가림 영역 때문에 유효 경로 0개. 엄격한 관측 검사가 map 정보로 우회되지 않음을 확인했다. |
| 가림 보조 bag 재생 r03 | 같은 기록/마스킹에서 유효 경로 431개, 그중 DR 399개. 모두 가림 보조이며 선택 속도 최대 5km/h. MPC의 경로 수신·계산 확인, 제어 publisher 0개. |

bag 입력은 `local1-20261028-drive20/sensors_0.bag`다. 기존 분석에서 이동 35.07m,
기록 최고속도 약 3.73km/h인 자료다. 이번 bag 결과를 15km/h 주행 결과와 합치지 않는다.
bag의 정적 회피 기동 수락도 아니다. 이 bag에서는 KEEP 경로를 확인했고 정적 우회·복귀는
별도의 합성 이상 추종 시험으로 검사했다. 원본 bag은 수정하지 않았으며 재생 중 GPS
메시지만 메모리에서 무효화했다. recorded header/receipt 시각을 사용하고 acquisition
clock을 복원했다고 주장하지 않는다. 공급 공백과 시작/종료의 거부도 그대로 기록했다.

## 실행과 재현

연구용 진입점은 [shadow_map_static.launch](../../../src/asmc_planner/launch/shadow_localization_static.launch)다.
다음은 제어 없는 preview이며 센서 bridge나 인지 노드를 추가로 시작하지 않는다.

```bash
roslaunch asmc_planner shadow_map_static.launch \
  run_id:=<registered-new-run-id> enable_control:=false
```

출력은 `/planning/shadow_map/{odometry,trajectory,path,trace,diagnostics}`로 분리된다.
기존 local 좌표계와 `LocalTrajectory`의 stamp/epoch/dt 계약을 유지하며 MPC 하나를 재사용한다.
제어를 켜는 실험은 기존 selector의 JAMMING slot을 사용하므로 NORMAL·기존 local 제어기를
동시에 실행하지 않는다. 제어기의 기존 소유권 검사가 중복 발행을 거부한다.

순수 계산·제어 경계 시험:

```bash
rosrun asmc_planner shadow_map_test
rosrun asmc_planner shadow_map_test 15
rosrun asmc_planner shadow_map_test 15 left
python3 tools/scen_runner/test_local_boundary.py
```

bag 재생은 [replay_shadow_map.py](../../../tools/perception/replay_localization.py)를 사용했다.
새 run을 D에 등록하고 입력 bag SHA·설정 SHA·source revision/dirty patch를 보존한 뒤,
별도 `--network none` 컨테이너에서 실행한다. 아래 경로는 컨테이너 `/storage` 계약이다.

```bash
python3 tools/perception/replay_shadow_map.py \
  /storage/artifacts/local1-20261028-drive20/sensors_0.bag \
  /storage/artifacts/<registered-new-run-id> \
  --run-id <registered-new-run-id> \
  --run-manifest <registered-run-manifest.json> \
  --config <registered-config.json> --gps-blackout-after 3
```

가림 보조 비교에는 등록 설정의 `near_blind_assist=true`와 CLI
`--near-blind-assist`가 모두 필요하다. 기존 run ID와 결과 폴더를 재사용하지 않는다.
재생기는 private ROS master와 제어 꺼짐 stack만 시작하고 자신이 시작한 프로세스만 종료한다.
다른 ROS master·시뮬레이터·GPU 인지·기존 컨테이너는 재시작하지 않았다.

실제 결과는 D `artifacts/aimdrive-static-port-20261028-r{01,02,03}`에 각각 있다. r01은 재생기
context 토픽 연결 오류로 입력 재생 전 종료한 실패 근거다. 수정 후의 r02/r03과 함께
보존했으며 입력 manifest·설정·dirty source patch는
`manifests/aimdrive-static-preparation/<run-id>`에 있다.

후속: [무장애 위치 추종의 실제 주행 체크포인트](2026-10-28-aimdrive-shadow-position-checkpoint.md).
