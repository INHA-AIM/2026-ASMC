# 판제가 사용하는 LiDAR 실행 정본

B4에서 `asmc_lidar`를 판제·수집·시나리오의 단일 LiDAR 실행으로 확정했다. 구 Main/Msg·wire bridge·
motion JSON 결합과 전용 필터/빌드 경로를 제거한다. 사용자와 판제가 새 경로 주행을 확인했으며,
전체 미션·터널 구조물 제거·속도 정확도 수락은 이후 공동 작업이다.

## 실행

Noetic 컨테이너의 `/root/ws`에서 outer workspace만 빌드하고 source한다. 일반 빌드는
LiDAR 처리 노드의 필수 의존성이 빠지면 실패한다. 수신 라이브러리만 필요할 때는 개발용으로
`ASMC_LIDAR_PIPELINE=OFF`와 `ASMC_LIDAR_PIPELINE_REQUIRED=OFF`를 둘 다 명시한다.

```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
./scripts/build_ws.sh -j2 -l2
source devel/setup.bash
```

아래 각 행은 별도 터미널이다. 브릿지·카메라·LiDAR는 각 한 세트만 실행한다.

```bash
./scripts/udp_bridge.sh
roslaunch asmc_camera camera.launch
roslaunch asmc_lidar lidar.launch
roslaunch asmc_planner autonomy_no_bridges_v2.launch
```

마지막 명령은 차량 제어를 포함한다. 판제 launch는 센서·추론 생산자를 시작하지 않는다.
`fusion:=true`, `debug:=false`, `rviz:=false`가 LiDAR 기본이다. 단일 비교는 `fusion:=false`,
RViz 추가는 `rviz:=true`, 병행 비교는 `namespace:=/migration/lidar`를 사용한다.

웹의 단계별 점군을 보려면 기존 LiDAR 실행 대신 `roslaunch asmc_lidar lidar.launch debug:=true`로
시작하고 별도 터미널에서 `roslaunch asmc_lidar view.launch` 후 `http://127.0.0.1:8091/`에 접속한다.
`view.launch`는 구독만 하며 새 처리 노드를 시작하지 않는다. debug OFF에서도 판제 출력은 제공된다.

`debug`는 중간 점군·웹 scan 표현만 제어한다. 검출·추적·카메라 fusion·HD맵/높이 전달 필터와
공용 `tracks_local/map` 발행은 계속 동작한다. 웹·trace·RViz를 실행하지 않아도 판제의
`planning_observations_map` 구독만으로 새 객체가 공급된다. debug OFF 웹에서 단계별 점군이 없는 것은 정상이다.
2026-10-26 시험은 이 판제 단독 수요와 GPS/IMU 중단·복귀·reset/restart를 실제 ROS로 확인했다.
운영 읽기 확인과 새 폐루프 주행의 구분은 [CI·디버그 검증 기록](../../../docs/records/repository/2026-10-26-ci-runtime.md)에 있다.

## 토픽과 공용 메시지

| 토픽 (`/perception/lidar/` 아래) | 타입·용도 |
|---|---|
| `planning_observations_map` | `asmc_msgs/LidarPlanningObservation`, 판제용 KF 속도 품질·운동 상태·관측 식별자 |
| `tracks_local`, `tracks_map` | `asmc_msgs/LidarTrackingObservation`, 기하·속도·4단계 운동 상태·관측 식별자 |
| `observations_local`, `observations_map` | `asmc_msgs/LidarObservation`, RViz 등 기하·식별 소비 |
| `objects_local`, `objects_map` | `asmc_msgs/LidarObjectArray`, 웹 상태 등 배열 소비 |
| `diagnostics`, `trace`, `debug/*` | 진단·분석·표시. 판제 정본 입력으로 결합하지 않음 |

모든 출력은 같은 전달 필터 결과에서 만들며 구독 수요에 따라 직렬화한다. 단계별 노드는 추가하지 않는다.
판제는 `planning_observations_map`을 queue1로 받아 `/planning/objects`로 변환한다.
PR #114에서 검증한 `LidarPlanningObservation`의 KF 속도 품질·미확정 속도 기하 보존과
세션별 private ID 분리를 유지한다. 별도 companion 실행·source 선택·JSON 중계는 없다.
`tracks_local/map`은 기존 `LidarTrackingObservation` 계약과 classifier 적용 추정값을 유지하는
분석용 출력이며, 판제용 KF 속도와 서로 다른 추정값이다. topic 이름만 remap해 두 타입을 섞지 않는다.
기존 공용 메시지 MD5는 유지하며 PR #114의 private `velocity_unavailable` 필드 때문에 전체 재빌드한다.

## 좌표·시간·무효 처리

- local은 후륜축 `base_link`다. 내부 수평화 좌표와 구분하고 중심·8코너·속도를 같은 scan context로 변환한다.
- map 객체는 scan 시각의 유효 pose에서만 제공한다. GPS0/무효/stale에서는 `POSE_UNAVAILABLE`로 알리고 local 기하를 유지한다.
- global Frenet에 local 토픽을 이름만 remap하지 않는다. 음영구간 local/family 소비와 행동 전환은 판제가 담당한다.
- source age300ms·미래 허용50ms, session/generation·설정/보정 ID·stamp/sequence 순서와 tracking 정렬을 검사한다.
- 정상 `OBSERVED_EMPTY`는 새 빈 배열로 전달한다. 무효·stale·pose 부족는 발행을 보류하며 정상 빈 결과를 만들지 않는다.
- `/planning_objects_adapter/diagnostics`는1Hz로 수신/발행/거부와 사유를 제공한다. 기존 timeout/장애 행동은 판제가 소유한다.

## 속도와 분류의 의미

판제용 속도는 PR #114의 평면 KF 상대속도에 같은 scan의 ego 병진 속도를 한 번 보상한 map 추정값이다.
이미 회전 정렬된 tracker 상태에 `omega × p`를 다시 더하지 않는다. 초기/누락 추적·부적절한 dt·pose
근거 부족이면 속도는 무효이며, 유효한 현재 기하는 `velocity_unavailable=true`·UNKNOWN으로 보존한다.
이때 private vx/vy의0은 점유 표현 자리값이며 측정된 정지가 아니다. 판제의 기존 불확실성/정지 정책을 따른다.
UNKNOWN/LOW_THREAT/DYNAMIC_PENDING/DYNAMIC을 구분하고 측정된 LOW_THREAT만 기존 정지 예측을 쓴다.
새 session/generation은 raw track ID를 private ID로 새로 매핑해 판제 캐시가 이전 객체와 혼동하지 않게 한다.
속도 정확도·후진·터널 입력의 추가 수락은 후속이며 일반 tracking 출력의 의미는 바꾸지 않았다.

카메라 vehicle/dynamic_obstacle/static_obstacle 그룹은 실제 운동 상태와 별개다. 원본 문자열은
공용 관측에 있고 private numeric enum은 vehicle/person/unknown만 표현한다. unknown도 충돌 기하를 유지한다.

## 전달 필터와 책임

HD맵 v7·안쪽10cm·점군3점/10% AND OBB10%·높이 ROI/상부 제외, 관측 품질 최소0.10m·확인3/해제5와
기존 근접 보존을 유지한다. 유효한 최신 카메라 vehicle/dynamic 매칭은 우선 보호한다. 지도/pose 무효는
지도 제외를 중단한다. 최종 kept ID를 모든 배열·관측·추적 관측에 동일 적용하고 tracker·원본 OBB는 보존한다.

구 도로 mask·시설물 중심 목록·사각 구조물 blacklist·OBB clipping은 다시 도입하지 않는다.
판제 입력에서 classifier·필터를 두 번째 실행하지 않는다. TTC·추종·정지·회피·센서 장애 대응은 판제에 유지한다.
[필터 소유권](filter-migration.md)과 [공동 작업표](migration-tasks.md)를 따른다.

## 최신 main 통합

PR #114의 속도 품질·GPS/재밍 복구를 이 정본에 통합했다. 판제 담당의 변경 대응표와
이번 재빌드·debug OFF 제한 주행은 [통합 기록](../../../docs/records/lidar/2026-10-26-pr114-canonical-integration.md)에 있다.

## 검증과 복구

[정본화 검증 기록](../../../docs/records/lidar/runtime-canonicalization.md)에 클린 빌드·출력 대조·실제 ROS·CI를 기록한다.
[이전 실행·주행·CI 증거](../../../docs/records/lidar/planning-runtime-through-pr111.md)는 당시 값으로 보존한다.
구 실행 복구는 삭제 전 `b2e8329f1cb71f11150d5125335e01c5b8d252c5`의 별도 checkout을 사용한다.
현재 checkout에 구 overlay를 source하거나 새 메시지를 구 wire 이름으로 remap하지 않는다.

## 배포와 분석 도구

기존 인지/판제 launch는 소유자가 정상 종료한 뒤 최신 소스를 빌드하고 각 터미널에서 Noetic과
outer setup을 다시 source한다. 구 nested setup을 source하던 개인 shell 설정도 제거한다.
자동으로 운영 노드를 종료하거나 구 build/devel/install 잔류 파일을 일괄 삭제하지 않는다.

속도 분석은 `tools/analyze_velocity_bag.py`와 `tools/measure_velocity_response.py`가 공용 관측의
실제 속도·validity를 직접 읽는다. Marker 문자열이나 진단 JSON을 결합하지 않는다. `/Object_topic`은
개발 평가 GT이며 운영 인지 입력으로 사용하지 않는다. 오프라인 분석의 numpy/matplotlib·ROS bag 환경은
별도 준비하고 결과는 고유 run ID·등록한 D 경로에 저장한다. 과거 결과를 덮어쓰지 않는다.

## HD맵 launch의 역할

`competition_map.launch`는 지도 publisher와 lane/context 조회를 시작하고 `rviz:=true`일 때 GUI를
추가한다. 기본 `publish_local:=false`라 큰 local crop은 끈다. `/hd_map/global/*`의 지도·경로·미션
토픽과 metadata가 필요한 소비자용이며 현재 주행의 필수 실행은 아니다.

Frenet은 `avoid_hd_map.directory`의 drivable/lines/grid bundle과 전역경로 파일을 직접 읽는다.
LiDAR도 정본에서 만든 SHA 고정 grid bundle을 시작 시 읽는다. 이 launch가 꺼져도 HD맵 필터는
켜져 있으며, 지도 토픽 발행이 두 파일 기반 소비자의 지도를 자동 갱신하지 않는다.
