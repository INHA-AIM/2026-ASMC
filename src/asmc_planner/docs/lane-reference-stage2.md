# 차선변경 후 참조선 인계 시험

> 2026-10-26 main 통합: 현재 실행은 [통합 안내](main-integration-20261026.md)를 따른다.
> 아래 날짜별 Main/nested workspace 기록과 구 입력 전환 옵션은 당시 검증 이력이다.
> 현행 입력은 asmc_lidar의 typed observation이며 별도 MORAI 변환기를 시작하지 않는다.


상위 계약은 [고주로 전환 계약](../../../docs/contracts/planning-highway-transition-v1.md),
지도·참조선 기반 API는 [1단계](./lane-reference-stage1.md)를 따른다.
이 단계는 **전용 시험 실행에서 LC → 참조선 인계 → KEEP/FOLLOW**를 연결한다.
일반 `autonomy_no_bridges_v2.launch`의 Behavior·미션 판단과 주행 경로는 유지한다.

이 시험에서 확인하려는 행동은 다음 세 가지다.

1. 기존 차로 A408에서 옆 차로 A434로 이동한다.
2. 목표 차로 안에 들어와 방향까지 안정되면, 그 차로 중심선을 새 주행 기준으로 삼는다.
3. 이후 KEEP은 그 차로를 계속 따라가고, FOLLOW는 그 차로의 앞차를 본다.

기본 preview는 차량을 움직이지 않는다. 실제 이동 시험에는 `enable_control:=true`와
준비 완료 후의 `arm` 호출이 필요하다. 합류 가능 시점이나 추월 여부의 자동 판단은 후속 단계다.

## 구현 범위

`Execution/LaneReferenceExecution`이 현재 Frenet core와 준비된 목표 core를
소유한다. 목표 core는 새로 초기화한 인스턴스에만 `SetPreparedReference`를
적용한다. 지도 polygon union·참조선 생성·두 core 초기화는 비동기 준비 작업이며
매 planning tick에 반복하지 않는다.

인계 전에는 기존 참조선의 LC를 계속 계획한다. 목표 차로에서 위치·heading·
횡속도·차체 점유가 안정화된 뒤, 같은 최신 ego/객체 snapshot으로 목표 참조선의
KEEP/FOLLOW를 실제 계산한다. 기존 충돌 검사에 더해 도로 영역·실선·유효 s
범위를 검사하고, 전환 궤적과 후속 궤적의 같은 1초 구간에서 위치·heading·곡률·
속도·가속도 차이를 검사한다. 두 core의 sampling dt도 같아야 한다.

ROS Trajectory 변환까지 성공하면 기존 CommandExecutionManager의 완료
전이를 시도한다. 그 전이가 성공한 같은 callback에서 core·ego/object 투영·
leader·다음 궤적을 가진 owner를 교체하고 ref_epoch를 한 번 증가시킨다.
입력 sequence/stamp나 지도가 바뀌거나 완료 전이가 거부되면 기존 owner와
LC 잠금을 유지한다. 후속 궤적을 새 시각으로 포장해 재사용하지 않는다.

인계 뒤에는 매번 현재 map pose와 객체로 KEEP/FOLLOW를 새로 계획한다.
선행차는 새 참조선 corridor와 OBB가 겹치는 전방 vehicle에서 선택하고, 충돌
검사에는 선택되지 않은 객체도 모두 전달한다. KEEP/FOLLOW/STOP 또는 leader가
바뀌면 command_id를 새로 발급한다. 같은 ID의 목표를 고치지 않는다.

| 값 | 시험에서의 의미 |
|---|---|
| `route_s` | 원래 미션 참조선에 현재 map pose를 투영한 위치; 진단용으로 따로 유지 |
| `lane_s/lane_d` | 채택한 차로 참조선의 ego 좌표; 후속 계획·객체 투영에 사용 |
| `ref_epoch` | 시작 1, 인계 성공 때만 2; 재시도/거부 때는 증가하지 않음 |
| `COMPLETED` | 인계한 LC command의 성공; 다음 cycle부터 새 KEEP/FOLLOW command |

`route_s`를 실제 CP/속도 구간의 기존 Behavior 소비자에 연결한 단계는 아니다.
이 시험은 BehaviorInput을 띄우지 않는다. 일반 runtime에 붙일 때는 미션 판단과
활성 차로 판단을 구분해서 모든 소비자를 연결해야 한다.

## 시험 조건과 한계

`config/reference_handover_trial.yaml`의 A408 → 인접 A434는 **선정한 시험 fixture**다.
차로 선택 정책을 이 ID로 하드코딩한 것이 아니다. build/hash를 고정하며 지도
개정 때에는 임의로 hash만 바꾸지 말고 해당 fixture를 재검증한다.

- 속도는 20 km/h, 시험 노드는 25 km/h 초과 설정을 거부한다.
- arm·READY·LC 확정 직전에 같은 객체 조건을 검사한다. UNKNOWN/정적 클래스이며
  운동 상태가 LOW_THREAT로 안정된 객체는, 실제 OBB의 XY convex hull이 출발 차로와
  모든 목표 차로 polygon에 겹치지 않을 때만 시작을 허용한다. 경계 접촉도 겹침이다.
  지도나 객체를 중심점·고정 거리로 대체하지 않으며 차로 전체를 검사하므로, 시험
  이동 거리 밖이더라도 같은 차로의 구조물은 보수적으로 시작을 막는다.
- 차량·보행자·알 수 없는 클래스, UNKNOWN/DYNAMIC_PENDING/DYNAMIC 운동 상태,
  유효하지 않은 객체 geometry는 차로 밖에 있어도 시작을 막는다. MORAI의 주변
  차량을 제거한 시험장을 전제로 하며 교통 gap을 승인하는 정책은 아니다.
  LOW_THREAT 분류가 보존한 raw 속도는 별도 임계값으로 재분류하지 않는다.
- 이 허용 검사는 객체 배열을 제거·수정하지 않는다. 차로 밖에서 허용한 구조물도
  모든 계획·후속 참조선 인계의 기존 충돌 검사에 그대로 전달한다.
- 유한한 목표 참조선 끝에서는 기존 STOP 계획을 사용한다. 이 시험용 끝 감속은
  고주로 필수 복귀/차로 감소 정책의 구현이 아니다.
- 자동 합류 gap 승인, 후방 접근 차량 예측, 선택적 추월, recovery/abort,
  차로 sequence 자동 선택, 고속 주행 안전 검증은 아직 포함하지 않는다.
- 목표 참조선에는 원래 미션의 회전교차로 s 구간을 복사하지 않는다. 일반 주행의
  회전교차로 설정을 수정한 것이 아니다.
- 정렬·안정화·연속성 수치는 초기 시험값이다. 실제 MPC 추종이 검증된 한계가 아니다.

## 빌드

호스트 repository에서 작업 브랜치를 갱신한다. main 변경이나 PR 생성은 필요 없다.

```bash
git fetch origin
git switch feature/highway-transition-contract
git pull --ff-only
```

ROS 컨테이너에서:

```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_ENABLE_TESTING=ON
source devel/setup.bash
./devel/lib/asmc_planner/lane_reference_test
./devel/lib/asmc_planner/lane_reference_execution_test
./devel/lib/asmc_planner/hd_lane_map_product_test
```

## 실행

HD-map publisher는 metadata에 고정된 공식 경로 원본을
`/data/R_KR_PG_K-city_2025/2026_molit_comp_global_path.txt`에서 요구한다.
새 컨테이너/데이터 볼륨에서는 launch 전에 한 번 준비한다.

```bash
cd /root/ws
python3 tools/planning/prepare_reference_handover_data.py
```

도구는 설정된 Frenet 경로에서 공식 원본의 줄바꿈·EOF 형식을 복원하되,
metadata의 원본 byte 수와 SHA-256이 정확히 일치할 때만 새 파일을 만든다.
좌표·지도·metadata를 수정하지 않으며 기존의 다른 파일을 덮어쓰지 않는다.
경로를 편집해 원본과 다르면 `--source /실제/공식원본.txt`를 지정한다.
데이터가 다른 위치에 있으면 도구에 `--data-root /실제/data`를 주고 launch에도
`data_root:=/실제/data`를 함께 지정한다. `ASMC_DATA` 환경 변수만 바꾸는 것으로
launch의 명시적인 `/data` 설정을 바꿀 수는 없다. 원본 검사를 끄지 않는다.

현재 터미널 1 UDP bridge, 2 camera perception, 3 Main LiDAR,
4 camera bbox detector는 그대로 사용한다. **터미널 5의 일반 주행 launch를
종료한 뒤** 아래 시험 launch로 교체한다. 지도 publisher·planning input adapter를
포함하므로 일반 planner와 동시에 실행하지 않는다.

먼저 제어 출력 없는 preview:

```bash
roslaunch asmc_planner reference_handover_trial.launch
```

MORAI에서 주변 차량이 없는 A408의 초반 직선에 차를 놓고 차로 진행 방향을
맞춘다. 목표 도착 거리 25 m와 이후 정지 여유가 필요하므로 차로 끝에서 arm하지
않는다. `HD-map target reference prepared` 로그 후 별도 터미널에서:

```bash
rosservice call /reference_handover_trial/arm "{}"
rostopic echo /planning/reference_handover/diagnostics
```

`arm`이 거부되면 이제 사유를 구분해 반환한다. `map_preparing`은 준비 대기,
`map_preparation_failed`는 지도 처리 실패, `inputs_not_ready`는 입력 유효성 문제다.
`ego_footprint_outside_source_lane`은 차량 전체가 지정한 A408 안에 들어오지 않은
상태다. `trial_object_blocks_start`는 객체 조건 때문에 거절된 상태이며 `id`와
`reason`을 함께 반환한다. `overlaps_source_target_corridor`는 차로 점유,
`motion_not_settled_low_threat`는 미확정/동적 운동 상태,
`traffic_actor_or_unknown_class`는 차량·보행자/알 수 없는 클래스다.
`trial_already_armed`이면 새 시험을 위해 launch를 재시작한다.

preview는 고정된 차량을 움직이지 않는다. 실제 도착·인계를 보려면 preview를
종료하고 같은 fixture에서 제어 시험을 실행한 후 위 arm을 다시 호출한다.

```bash
roslaunch asmc_planner reference_handover_trial.launch enable_control:=true
```

이때 controller와 control selector도 실행한다. 기존 GpsModeSource가
`/planning/mode`를 발행하므로 selector가 NORMAL 제어를 전달할 수 있다.
이 시험에 재밍 주행 실행기는 포함하지 않는다. arm 전과 무효 입력 때에는
정지 궤적을 발행한다. 한 번 인계한 뒤 재시험하려면 launch를 재시작한다.

| 출력 | preview | 제어 시험 |
|---|---|---|
| trajectory | `/planning/reference_handover/trajectory_preview` | `/planning/trajectory` |
| command / feedback | `/planning/reference_handover/command`, `/planning/reference_handover/feedback` | 동일 |
| diagnostics | `/planning/reference_handover/diagnostics` | 동일 |
| control | 발행하지 않음 | `/ctrl_cmd/normal` → selector → `/ctrl_cmd` |

일반 대시보드의 Behavior 표시를 이 시험의 FSM 상태로 해석하지 않는다. 시험
feedback의 LC 완료, 이후 KEEP/FOLLOW, diagnostics의 ref_epoch=2와 lane_d,
실제 차로 유지·조향 연속성을 함께 확인한다. lane 끝 STOP은 예상한 동작이다.

이미지·point cloud 없이 필요한 출력만 녹화한다. 60초마다 분할하며 실제 용량은
검출 객체 수와 궤적 길이에 따라 달라진다.

```bash
cd /root/ws
mkdir -p diagnostics
rosbag record --lz4 --split --duration=60 -O diagnostics/reference_handover \
  /gps /planning/vehicle/state /planning/objects /planning/trajectory \
  /planning/reference_handover/trajectory_preview \
  /planning/reference_handover/command /planning/reference_handover/feedback \
  /planning/reference_handover/diagnostics /planning/mode /ctrl_cmd /rosout_agg
```

## 검증 결과와 남은 확인

2026-10-15, 브랜치 기반 `11120b5` 및 같은 지도·Frenet 설정:

| 확인 | 결과와 범위 |
|---|---|
| 기존 native 지도·정렬 검사 | 63 checks, 0 failures |
| 실제 Frenet core를 연결한 실행 단위 검사 | 60 checks, 0 failures; 준비/완료 거부, snapshot 변경, epoch 단일 증가, KEEP 유지, 목표 차로 FOLLOW, stale/지도 변경/충돌/연속성 거부, 실행 상태 재결합 |
| 기존 093505 / 131053 / 140026 bag | A408 자세 63 / 116 / 118개, 합계 297개; 목표 A434 기하 도착 오판 0건, 첫 bag 2개 invalid |
| 실제 HD map A408→A434 추가 native 확인 | 생성 궤적의 다음 표본을 오차 없이 따른다는 가정에서 인계 후 KEEP 지속, epoch=2; MPC/센서/ROS closed-loop 시험 아님 |
| 공식 경로 준비 도구 | LF/CRLF/EOF 차이 복원 후 공식 SHA·254056 bytes 일치; 재실행, 원본 보존, 다른 기존 파일 보존, 좌표 변경 거부 확인 |

bag 검사는 기존 `check_lane_reference.py`로 같은 기하 core를 다시 실행한 결과다.
과거 bag에는 LC 주행이 없어 실제 완료·추종이나 동적 충돌 검증의 근거로 삼지
않는다. 객체 source stamp만 사용한 기하 검사이며 전체 인지 replay가 아니다.

이 host에는 ROS Noetic/catkin이 없어 **새 ROS 실행 파일의 전체 build와 MORAI
주행은 아직 미실행**이다. native Frenet 전체 library와 두 검사 실행 파일을 직접
컴파일했다. ROS 메시지/launch 인자를 대조했지만 이를 catkin build 성공으로
대체하지 않는다. 사용자 컨테이너에서 빌드 후 빈 차로 시험이 다음 수락 조건이다.

후속 사용자 Noetic 로그에서 `reference_handover_trial` 기동을 확인했다.
당시 HD-map publisher가 `/data`의 공식 경로 파일 누락으로 종료한 문제에 대해
위 사전 준비 도구와 `data_root` launch 인자를 추가했다. MORAI에서 실제 인계·
추종이 성공했다는 결과는 아직 없다.

## Boost 1.71의 지도 준비 실패 회귀 수정

같은 compact map의 102개 주행가능영역을 전역 union으로 합치면 Boost 1.71에서
`(-57.1401, -136.002)`의 self-intersection 오류가 재현됐고, Boost 1.89에서는
통과했다. 원본 영역은 각각 유효하며, 문제는 새 소비 코드가 만든 전역 합집합이었다.

이제 원본 polygon을 각각 검증해서 보관한다. 한 영역이 차체 전체를 포함하면
즉시 판정하고, 여러 영역을 걸칠 때만 차체 bounding box와 겹치는 영역들을
지역적으로 합쳐 **차체 전체**를 검사한다. 각 지역 결과도 유효성을 확인하며,
유효하지 않은 결과로 통과시키지 않는다. 오래된 integer rescaling은 이 구현
파일의 지역 연산에서 사용하지 않는다. 원본 지도·차폭·여유거리·공용 메시지는 유지한다.
실제 틈을 메우는 buffer나 작은 잔여 면적을 무시하는 허용값은 추가하지 않았다.
같은 불변 영역·같은 polygon 집합의 지역 결과는 thread별 한 개만 재사용한다.
새 지도/다른 polygon 집합이면 다시 계산하고, 차량 위치나 포함 판정은 캐시하지 않는다.

원본 좌표 그대로의 지도 product 회귀 검사를 build에 추가했다. Boost 1.71과
1.89에서 지도/정렬, 실제 product, 참조선 실행 검사를 각각 수행하며, 좁은 틈·
영역 연결부·내부 섬·점에서 만나는 구멍·원본 polygon 유효성 거부를 포함한다.
2026-10-16, 같은 host에서 각 Boost 헤더 버전으로 빌드한 최종 결과:

| 검사 | Boost 1.71 | Boost 1.89 |
|---|---:|---:|
| 지도·정렬·영역 경계·캐시 검사 | 78 / 78 | 78 / 78 |
| 실제 compact product 검사 | 8 / 8 | 8 / 8 |
| 실제 Frenet core의 참조선 실행 검사 | 60 / 60 | 60 / 60 |

A408→A434의 이상적 추종 시험도 두 버전에서 인계 후 KEEP과 epoch=2를 확인했다.
이는 Boost 버전 호환성을 검사한 native 빌드이며 Noetic 전체 컨테이너 재실행은 아니다.
이 수정도 ROS 전체 실행·MORAI closed-loop 성공을 대신하지 않는다.

일반 주행 cycle, Behavior, controller, 인지 producer, 정본 지도, 기존 주행 YAML과
공용 메시지는 변경하지 않았다. rollback은 이 단계의 커밋을 revert하고 일반
주행 launch를 다시 사용한다.

## 객체 0개 시작 조건의 보완

사용자 Noetic 실행에서 지도 준비와 arm 성공을 확인한 뒤, 다음 제어 실행은
`empty_road_trial_requires_zero_objects`로 거절됐다. 보고된 ID 1104의 OBB는 실제
A408과 A434 영역에서 각각 약 12.10 m, 8.61 m 떨어져 있었지만, 기존 arm·READY·
LC 확정 조건은 객체 개수만 검사했다. 이 거리를 새 임계값으로 사용하지 않는다.

이제 세 지점 모두 위 시험 객체 조건을 사용한다. 실제 compact map과 보고된
8개 corner로 차로 밖 허용을 재현했고, 같은 OBB를 목표 차로로 옮긴 경우와
DYNAMIC_PENDING으로 바꾼 경우는 계속 거절되는 것을 검사했다. 별도로 출발/목표
차로 점유, 경계 접촉, 차량·보행자, 미확정 운동 상태, 잘못된 geometry와
오목한 차로 polygon을 검사한다.

같은 ID 1104를 제거하지 않고 실제 Frenet LC·인계·후속 KEEP에 계속 전달한
native 이상적 추종에서는 tick 72에 ref_epoch=2가 됐고, 이후 KEEP의 최대
횡오차는 약 0.0122 m였다. 이는 생성 궤적을 오차 없이 따라가는 시험이며,
수정 후 ROS 전체 build와 MORAI MPC 추종 확인은 사용자 컨테이너에서 필요하다.

Boost 1.71과 1.89 각각에서 지도·정렬 78개, 실제 product 11개, 실행·객체 승인
82개 검사가 모두 통과했다. 이 객체 조건은 전용 시험에만 적용하며, 일반 주행의
객체 필터나 회피·FOLLOW·MERGE 승인 조건으로 사용하지 않는다.
