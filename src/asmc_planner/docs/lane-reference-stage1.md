# 차로 참조선 기반 단계와 오프라인 검사

상위 설계는 [고주로 전환 계약](../../../docs/contracts/planning-highway-transition-v1.md)이다.
이 단계는 HD map을 받아 목표 차로 참조선을 준비하고, 인계 전 기하 조건을
독립적으로 검사한다. 자동 합류·차선변경, 실시간 참조선 교체와 제어 출력은
활성화하지 않는다. 기존 production cycle·일반 STOP·회전교차로는 그대로다.

후속 연결은 [2단계 전용 참조선 인계 시험](./lane-reference-stage2.md)에 구현했다.
아래 결과·제한은 1단계 오프라인 API의 범위이며 일반 주행에는 두 단계 모두
자동으로 적용하지 않는다.

## 구현 경계

| API | 역할 | 아직 하지 않는 일 |
|---|---|---|
| `Map/HdLaneMap` | 불변 전역 지도, 차로 연결·인접 관계, 원본 도로 polygon 집합, 실제 실선 geometry 보관 | route/CP 기반 목표 차로 선택 |
| `Parser/HdMapLaneInput` | 기존 ROS HD-map 메시지의 frame/build/hash·완전성·수량 확인 후 native snapshot 변환 | 구독·실시간 snapshot 교체 |
| `Frenet/Reference/LaneReference` | 명시한 lane sequence 연결, 기존 `BuildRefLine` 재사용, 도로·도색·곡률 검사 | 횡이동 궤적·동적 gap 판단 |
| `Execution/ReferenceHandover` | command/ref epoch 결합, 최신 자세·객체 시각, 목표 차로 정렬·차체 점유·안정화 시간 검사 | public READY/COMPLETED, 실제 상태 인계 |
| `lane_reference_audit` / `check_lane_reference.py` | 같은 C++ core에 지도·합성 자세·기존 bag 자세 입력 | ROS 제어·궤적 발행, MPC closed-loop 시험 |

`HdMapMeta`와 세 계층 `LaneArray`, `LaneMarkingArray`, `DrivableAreaArray`가
모두 complete global snapshot이고 build/map/hash/frame이 일치해야 한다.
부분 crop, 누락된 연결, 비대칭 연결, 중복 ID, 비유한 geometry, provisional
도로는 거절한다. 실패한 새 snapshot이 기존 불변 snapshot을 수정하지 않는다.

분기에서는 `successors.front()`를 택하지 않는다. 호출자가 미션과 일치하는
lane sequence를 명시해야 한다. 현재 API 자체에는 CP 통과·횡단 허용 구간·
관측 범위·고주로 합류 승인 정책이 없다.

## 영역과 도색 검사

- 참조선 중심은 요청한 차로 polygon union 안에 있어야 한다.
- 차체 전체는 검증된 `DrivableArea` union 안에 있어야 한다. 네 모서리만
  확인하지 않으므로 차체 내부의 작은 비주행 영역도 검출한다.
- 차로 분류 polygon의 합류부 틈을 곧바로 물리적인 벽으로 취급하지 않는다.
  실제 목표 차로 인계 조건은 더 엄격하게 **차체 전체의 목표 차로 점유**를 요구한다.
- 단일 `left/right_boundary_id` 대신 모든 실제 실선 polyline과 도색 폭을 검사한다.
  정지선과 가상선은 실선 차로 벽으로 취급하지 않는다. 점선이라는 사실만으로
  차선변경을 허용하지 않으며 횡단 가능 정책은 다음 단계에서 검사한다.
- 기존 곡률·횡가속도 상한은 낮추지 않았다. 곡률에는 표본의 kappa와 인접
  heading 변화율을 함께 사용한다. `curveSpeedLimit()`은 정적 상한이며 명령 속도가 아니다.
- 국소 reference 양끝에서 차체가 넘어가는 구간은 사용 가능 s 범위에서 제외한다.

참조선 정적 검사는 거리·heading 간격을 제한한 표본 검사다. 연속 시간의
전체 swept-volume 충돌 안전이나 실제 제어 추종을 보증하지 않는다.

## 인계 전 기하 조건

`ProjectOnLaneReference`는 ego·객체의 물리적인 map 좌표에서 새 s/d를 계산한다.
이전 ref의 s_hint·target_d를 복사하지 않는다. 투영 성공만으로 목표 차로
점유가 확인된 것은 아니다. 객체 속도는 차체 heading과 독립적인 map vx/vy
overload로 투영한다. ego용 speed/yaw overload로 횡단 객체의 속도를 대신하지 않는다.

`ReferenceHandoverGate::begin`에 command_id, source ref epoch, 불변 target과
최소 도착 s를 고정한다. 동일 ID 갱신은 목표나 epoch를 바꾸거나 안정화 시간을
초기화할 수 없다. 반복 발행된 같은 source pose는 시간을 누적하지 않는다.
오래된 ego/객체 입력, pose 공백·역행, map/command/epoch 불일치는 안정화
이력을 끊는다. target 위치·heading·횡속도와 차체 점유를 일정 시간 만족하면
`GEOMETRY_ELIGIBLE`을 반환한다.

이 값은 **public READY/COMPLETED가 아니다**. 최신 동적 충돌 검사, 후속 주행
궤적과 전환 궤적의 연속성, ego/객체/leader/ref의 원자적 교체가 추가로 필요하다.

## 빌드와 실행

ROS Noetic workspace에서 다음처럼 빌드한다. 새 core는 Boost.Geometry header를
사용하며 `package.xml`에 `boost` 의존성을 선언했다. 기존 `Reference.cpp`는
기존 Frenet library 소유를 유지한다.

```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_ENABLE_TESTING=ON
source devel/setup.bash
./devel/lib/asmc_planner/lane_reference_test
./devel/lib/asmc_planner/hd_map_lane_input_test
```

실제 지도에서 목표 차로와 A445→A153 연결을 각각 확인한다.

```bash
python3 tools/planning/check_lane_reference.py \
  --binary devel/lib/asmc_planner/lane_reference_audit \
  --target-lanes lane:A2256W000434 --synthetic-target

python3 tools/planning/check_lane_reference.py \
  --binary devel/lib/asmc_planner/lane_reference_audit \
  --target-lanes lane:A2256W000445 lane:A2256W000153 --synthetic-target
```

기존 bag에서는 원래 차로 A408 구간의 실제 ego 자세를 A434 참조선에 넣는다.
이는 아직 옆 차로를 달리는 차를 도착 완료로 오인하지 않는지 확인하는 시험이다.
bag 경로는 보유 파일로 지정하고 여러 번 `--bag`을 줄 수 있다.

```bash
python3 tools/planning/check_lane_reference.py \
  --binary devel/lib/asmc_planner/lane_reference_audit \
  --target-lanes lane:A2256W000434 --source-link A2256W000408 \
  --bag diagnostics/curve_avoid_diag.bag --synthetic-target
```

기본 입력은 정본 compact map·reviewed markings·현재 Frenet YAML이다.
`--map`, `--markings`, `--params`로 다른 파일을 지정할 수 있으며, markings의
원본 byte SHA가 compact map의 source SHA와 다르면 거절한다. Noetic에서는
설치된 `rosbag`을 사용하고, ROS 없는 host에서는 선택적으로 `rosbags`를 쓴다.
`--report` 출력은 생성물이며 git에 추가하지 않는다.

## 확인한 결과와 한계

2026-10-15, main `fcfe8c5`의 지도·파라미터를 입력했다.
지도 build는 `kcity2025_static_junction_widened_drivable_v6`, 70 lanes / 102 areas다.

| 시험 | 결과 |
|---|---|
| native core 단위 검사 | 63 checks, 0 failures; 직선·곡선 정렬, 부분/불일치 지도, 섬·실선·차체 점유, 독립 객체 속도, stale/중복 시각 포함 |
| A434 참조선 | 145.000 m, 최대 곡률 약 0.001575 /m, 정적 검사 통과 |
| A445→A153 참조선 | 218.999 m, 최대 곡률 약 0.065952 /m, 정적 검사 통과 |
| A434·A445→A153의 target 정렬 합성 자세 | 각각 30개 중 초기 5개 안정화, 이후 25개 geometry eligible |
| 기존 093505 / 131053 / 140026 bag의 A408 ego 자세 | 각각 63 / 116 / 118개, 합계 297개; 목표 A434 도착 오판 0건 |
| 위 기록의 입력 유효성 | 첫 bag 2개 invalid, 나머지는 aligning; 객체 source stamp는 `/last_lidar`에서 확인 |

합성 자세는 차량 운동 시뮬레이션이 아니다. bag 시험도 고정된 과거 자세를
새 core에 넣은 것이며 새 차선변경의 실제 성공·후방 gap 안전을 뜻하지 않는다.
객체 시각의 freshness를 확인했지만 이 도구에서 객체 전체 충돌 예측을 하지 않는다.
A445→A153의 정적 곡률 상한으로 계산한 속도는 약 22.6 km/h이며, 기존 주행
속도·경로·제어를 수정한 결과가 아니다.

이 host에는 ROS Noetic/catkin이 없어 ROS adapter test와 전체 catkin build는
실행하지 못했다. 실제 ROS 메시지 필드·enum을 대조하고 테스트를 등록했으며,
native C++ core·CLI는 g++로 직접 빌드해 검사했다. GitHub CI나 MORAI 주행을
통과했다고 해석하지 않는다.

초기 구현의 전역 polygon union과 참조선 준비에는 이 host에서 초 단위 시간이 걸렸다.
후속 수정은 전역 union을 제거하고 원본 polygon을 보관한다. 차체가 한 영역에
포함되면 바로 판정하고, 여러 영역을 걸치는 경우에만 차체 주변 영역을 합쳐
차체 전체 포함 여부를 검사한다. Boost 1.71의 integer rescaling도 이 구현 파일의
지역 연산에서 사용하지 않는다. 원본과 생성 결과의 기하 유효성 검사는 유지한다.
전체 지도 검사와 참조선 생성은 준비 단계에서 수행해 불변 결과를 재사용한다.
차체 주변 영역 조회는 planning cycle의 검사다. 실제 runtime 연결 전 대상 PC에서
준비 시간·검사 비용을 측정한다.

## 다음 연결 단계

1. MissionRoute는 유지하고 ActiveLaneReference와 ref epoch의 단일 소유자를 둔다.
2. 빈 목표 차로에서 최신 ego/객체를 새 ref에 재투영하고 leader와 후속 KEEP/FOLLOW를 준비한다.
3. 전환 전후 궤적의 같은 시각 위치·heading·곡률·속도·가속도와 동적 안전성을 검사한다.
4. 모두 통과할 때 reference·ego/object 좌표·leader·다음 궤적을 함께 적용하고 COMPLETED를 발행한다.
5. 비활성 회귀와 빈 차로 closed-loop를 통과한 뒤 실제 고주로 gap 승인 정책을 연결한다.

지도/참조선 생성 실패가 기존 궤적 발행을 막지 않는 구조를 유지한다.
부분 인계나 과거 s/d 재사용으로 실제 차선변경 연결을 대신하지 않는다.
