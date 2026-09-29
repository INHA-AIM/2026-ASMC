# LiDAR → 판단제어 공통 소비 계약 (L1-A 설계 기준)

> **상태:** design frozen, **운영 구현 아님**. 2026-09-27에 main `1ce434d`, 손재호
> `6435af1`, 양서준 `8b3f0b1`을 대상으로 감사했다. 이강태의 미공개 음영구간 코드는
> 포함하지 않았다. 이 문서는 기존 토픽·ROS1 메시지를 변경하거나 소비자 fallback 제거를
> 허가하지 않는다. 계측 근거는 [L1-A 보고서](../../packages/asmc_perception/lidar-l1a-validation-20260926.md)와
> [기준 manifest](../../../config/baselines/lidar_l1a_20260926.yaml)에 있다.

## 현재 연결 상태와 메시지 경계

`main`의 기본 자율주행 launch는 MORAI `/Object_topic`을 Behavior에 직접 주고,
`morai_objects_adapter.py`가 typed `/perception/objects`를 Frenet에 준다. 따라서 현재
공통 자율주행이 손재호 LiDAR를 자동으로 소비한다고 해석하면 안 된다. 서준 브랜치의
`objects_source:=lidar`는 `/last_lidar → lidar_objects_adapter → /perception/objects`로
Behavior와 Frenet의 입력을 맞추지만, **map 출력만** 수용한다. `/lidar_nogps`를
`base_link`/`map`으로 변환하는 경로는 없다.

| 단계 | 실제 입력·출력 | 좌표·시간·객체 의미 | 현재 방어/보완과 위험 |
|---|---|---|---|
| 손재호 LiDAR `6435af1` | `/velodyne_points` → `/last_lidar`, `/lidar_nogps`, `/lidar/tracked_objects_map` (`Msg/TrackedObjectArray`) | 정상 `/last_lidar`는 map, 음영 `/lidar_nogps` 헤더는 `velodyne`이나 자세 보정된 센서 원점. `velocity_mps`는 헤더 좌표의 절대속도로 문서화돼 있으나 음영의 map 근거는 무효. tracker ID는 mode 전환에서 reset된다. | `/lidar/tracked_objects_map`은 GPS gate보다 먼저 발행되어 음영에서 map 정본 아님. 빈 출력은 상태 표시가 없어 비활성/관측상 없음 구분 불가. |
| 서준 adapter `8b3f0b1` | `/last_lidar` → typed `/perception/objects` (`asmc_msgs/DetectedObjectArray`) | 정확히 `map`, nonzero stamp, finite OBB/속도·ID 중복 검사. 원본 header·OBB·`vx/vy` 복사, `yaw_rate=0` (미측정). `classification`으로 type 결정; `motion_state` 복사. | invalid frame은 전체 프레임 reject·무발행. source timeout은 경고만 내며 새 빈 배열은 만들지 않음. 이 코드는 main에 없음. |
| Behavior `main` | `/vehicle/state`, `/Object_topic` (기본) → `/behavior/command_v2` | Ego map/rear-axle 상태를 route에 투영. MORAI `unique_id`를 객체 ID로 유지하고 객체 x/y를 map처럼 씀. MORAI 객체 local km/h 속도의 크기를 m/s로 바꿔 vehicle/pedestrian/static, lead/gap·TTC·회피 대상 결정. 객체의 sensor frame/stamp 자체는 검증하지 않고 수신시각으로 freshness 판단. | 0.3 s 수신-age 초과 객체를 빈 집합으로 치환해 대상 clear. 이 빈 집합은 정상 empty 관측과 같지 않음. |
| Behavior `8b3f0b1` | `objects_source:=lidar`에서 typed `/perception/objects` → 같은 command | source ID와 `vx/vy` 보존, semantic type + `motion_state` 구분; UNKNOWN/LOW_THREAT도 footprint와 경로 corridor 확인. LOW_THREAT는 gap/TTC speed를 0으로 쓰되 원본 속도 보존. adapter가 map/header를 검사하지만 Behavior는 수신-age를 사용. | LiDAR ID 재연결, 같은 차량의 작은 fragment 억제, 대상 소실 grace, 침범 candidate 확인은 planner 보완. 추적 원인 해결 전 제거 금지. invalid/stale 시 대상 clear 정책은 별도 안전 검토 필요. |
| Frenet `main` | `/perception/objects`, `/vehicle/state`, Behavior command → `/planning/trajectory` | 객체 array는 정확히 `map`/nonzero source stamp, finite center·속도·치수, 유효 8-corner OBB, unique ID. 각 객체의 `sqrt(vx²+vy²)`가 speed. 직접 제공된 OBB가 정본이며 center로 재생성하지 않음. | 객체 freshness는 **수신-age** 0.3 s, source stamp는 nonzero 검사; 오래된 scan을 새로 수신하면 age를 충분히 잡아내지 못할 수 있음. invalid 변환 시 STOP. 차량 상태도 map·valid·nonzero stamp 필요. |
| MPC `main` | `/planning/trajectory`, 최신 `/vehicle/state` → `/ctrl_cmd` | trajectory와 ego는 `map`, nonzero source stamp·finite 값. trajectory의 `ego_at_plan`은 provenance이며 제어의 x0는 최신 `/vehicle/state`; 객체 ID/속도를 직접 구독하지 않음. | trajectory 0.25 s/vehicle 0.20 s **수신-age** watchdog과 invalid 입력에서 STOP. source stamp를 현재시각으로 위장하면 회귀가 무력화됨. 긴급정지 별도. 음영에서 state invalid면 LiDAR만 local로 내도 주행 전환이 되지 않음. |

`/vehicle/state`의 GPS/IMU bridge는 GPS status·수치·stamp 및 수신-age를 검사하고
map `valid=false`를 낼 수 있다. 반면 손재호 LiDAR의 GPS 판정은 stale/NaN/복구 점프를
동등하게 다루지 않는다. **객체만 base_link로 바꿔도** Behavior/Frenet/MPC의 차량 상태와
trajectory 계약이 map이므로 음영 주행의 전체 handoff는 완료되지 않는다.

### ROS1 wire MD5 — 섞어 실행 금지

| 타입 | main `1ce434d` / 손재호 `6435af1` | 서준 `8b3f0b1` |
|---|---|---|
| `Msg/TrackedObject` | `6c96135eb03a3ee93bba474fa835f57d` | `24023c3f86bb466d143f4cd4786f1aa9` |
| `Msg/TrackedObjectArray` | `e127b2aabd7494ad01a5a56b005bf07e` | `dda823b466dd93ef3f37e6d9cc80e1fc` |
| `asmc_msgs/DetectedObject` | `a965285ae904ae3ade687b9c54c2c515` | `2889ec910dcdb9c6374cf71a4adc9b20` |
| `asmc_msgs/DetectedObjectArray` | `14fbd9679c2d87ae7498e4269d9678da` | `e54b9e001c89ab95955fb33059db2256` |

서준 브랜치의 `motion_state` 상수·필드 추가가 중첩 배열 MD5까지 바꾼다. 같은 토픽명으로
재생하거나 서로 다른 nested `devel`을 source해서는 안 된다. 정확한 MD5 재검사는
`python3 tools/lidar/validate_l1a_baseline.py`가 pinned Git 정의에서 계산한다.

## 다음 구현이 만족할 의미 (아직 토픽/타입 지정 없음)

| 축 | 동결한 의미 | 실패/모호 상태 |
|---|---|---|
| geometry | local은 차량에 고정된 rear-axle `base_link` m/rad이며 중력 수평 frame이 아니다. 수평화 입력은 **실제 적용 행렬의 역변환 → 고정 sensor→base 장착 변환**, raw 센서 입력은 고정 장착 변환만 적용한다. map은 관측 당시 base pose를 추가 적용한다. 중심·8-corner OBB·방향·속도는 같은 frame/stamp를 공유하며 3D 코너가 정본이다. | pose invalid/stale일 때 map만 불가, local geometry는 보존. 헤더만 개명하거나 body local에 IMU 자세를 다시 적용하지 않음. |
| 속도 | 이미 절대속도인 벡터는 축만 회전하고 CAN 속도를 더하지 않는다. 회전하는 base에서 위치를 미분한 상대속도는 `v_abs_base = v_rel + v_ego + ω×p_base`로 변환한다. legacy 의미 불명확 시 unknown. | 상대 미분에 필요한 같은 시각의 twist가 없으면 unknown. map pose가 없으면 map 속도만 불가. 정적/UNKNOWN 분류를 근거로 속도를 0으로 위장하지 않음. |
| 시간 | 관측 `header.stamp`, 처리 완료 시각, ROS 수신 시각을 별도 관리. delay/age는 각 원점에서 계산한다. | stale scan에 현재 시각 stamp 부여 금지. source stamp가 영점/역행하면 품질 저하·reject로 노출. |
| GPS/출력 | 내부 track 상태와 출력 projection을 분리. `valid/invalid/stale/recovering`에서 local 고정-frame 출력과 map 고정-frame 출력의 가용성을 명시한다. | 한 토픽의 frame 전환 금지. 잘못된 map 출력 대신 무발행/명시적 invalid, 소비자 fail-safe 확인. |
| 관측 상태 | `observed_empty`, `inactive`, `sensor_missing`, `pose_invalid`는 서로 다르다. | 비활성/누락을 fresh empty 배열로 바꾸면 충돌 회피가 해제될 수 있음. 현재 wire에는 구분 필드가 없으므로 후속 합의 필요. |
| 주행 관련성 | OBB footprint ↔ HD맵 차로/주행영역/경로의 겹침을 **별도** 판정. 인접 차로 차량, 진입·횡단 후보를 보존한다. 차로 번호는 competition scope를 따른다. | 경계 침범은 복수 후보/ambiguous, 밖은 outside/unknown. 카메라 drivable의 장애물 구멍은 객체 삭제 근거가 아님. |
| 불확실성 | class·motion·tracking age/miss·관측 품질을 섞지 않는다. ID 연속성·분할/합쳐짐은 추적에서 개선한다. | 중심점 하나로 lane 단정하거나 UNKNOWN 객체를 위험 없음으로 단정하지 않음. |

LiDAR 책임은 지면·벽·천장 제거, 군집/OBB/ID/속도 안정성, 관측 품질과 frame/time의
정직한 발행까지다. Behavior/Frenet/MPC 책임은 위험/대상 선택, 차로 변경, 회피 기동 유지,
충돌 판단·정지다. 판제의 ID 재연결/fragment 억제/소실 grace 중 **원인 보완**은
LiDAR 회귀 개선이 입증된 뒤에만 제거 후보이며, 기동 안전 히스테리시스는 별개로 보존한다.

후속 구현은 새 version/type 또는 호환 adapter를 합의한 뒤 진행한다. 이 단계에서
`/last_lidar`, `/lidar_nogps`, `/perception/objects` 이름·MD5와 판제 소스는 그대로다.

## L1-B1 격리 검증 — 운영 연결 전

[코어 실행법](../../../tools/lidar/frame_time/README.md),
[검증 보고서](../../packages/asmc_perception/lidar-l1b1-validation-20260927.md),
[고정 manifest](../../../config/baselines/lidar_l1b1_20260927.yaml)를 따른다.

- 관측·수신·재생 처리 시각은 명시한 같은 clock의 **정수 ns**다. bridge 수신 stamp는
  광선 측정 시각이 아니다. CPU 소요시간은 별도 monotonic clock으로 잰다.
- pose는 처리 cutoff까지 도착한 동일 시간축 표본만 사용한다. 유효한 양쪽 표본 사이
  최대 100 ms까지만 보간하고 외삽하지 않는다. 영점·중복·역행·무효 상태를 기록한다.
- GPS 안테나 pose와 rear-axle pose를 구분해 lever arm을 한 번만 제거한다.
- 카메라와 LiDAR의 시간·자세 이력은 공유 가능한 기반이다. 카메라는 촬영 시각의 노면
  BEV 투영에, LiDAR는 명시한 시각/좌표 변환에 사용한다. 동시각 센서 간 투영은 고정
  extrinsic으로 연결할 수 있으므로 동일 자세를 양쪽에 중복 적용하지 않는다.
- 장착 변환·수평화·스캔 내 deskew·프레임 간 자차 운동보상은 별개다. 현재 XYZI에는
  점별 시각이 없고 MORAI 스캔 생성 방식도 확인되지 않아 deskew는 구현하지 않았다.

`frame_time_eval`은 JSONL만 생성한다. 통과해도 planner 입력을 대체하거나 GPS 상태 전환을
구현한 것은 아니다. 운영 연결은 차량 상태·trajectory까지 함께 후속 합의한다.

## L1-B2 GPS/출력 격리 후보 — 운영 미연결

[L1-B2 보고서](../../packages/asmc_perception/lidar-l1b2-validation-20260927.md)와
[재실행](../../../tools/lidar/gps_transition/README.md)은 raw GPS 건강 상태,
관측 시점 map pose, planner 차량 상태·trajectory를 **별도 가용성**으로 검사한다.
정차 bag의 두 음영 window는 `status=1`이어도 위도·경도 0이므로 invalid다.
GPS가 invalid/stale/recovering/jump-rejected여도 유효한 `base_link` 객체 ID·3D 코너·
원본 stamp를 GPS gate만으로 폐기하거나 현재시각으로 갱신하지 않는다. 그러나 판제의
현행 map 계약에 local 객체를 무변환으로 흘려보내지 않는다.

`observed_empty`는 detector가 **그 관측을 정상 처리했다는 증거**가 있을 때만 허용한다.
detector 미실행·센서 누락·잘못된 stamp/pose·비활성 출력은 다른 상태다. 현재 운영
ROS 메시지에는 이 상태 필드가 없으므로 추후 versioned message/호환 adapter가 필요하다.
격리 selector는 실제 tracker/판제 코드에 연결하지 않았으며, 정차 bag에는 관측시각
rear-axle pose가 없어 map OBB·음영 전환 제어를 검증하지 못했다. 점프 gate 수치는
실험 가설이고 텔레포트에서는 의도적으로 거부된다. 운영 변경과 판제 fallback 제거는
연속 주행·상태/trajectory 합동 수락 뒤 별도 결정한다.

## L1-B3 카메라 보호 매칭 격리 후보 — 운영 미연결

[YOLO/LiDAR 매칭 결과](../../packages/asmc_perception/lidar-l1b3-validation-20260928.md)는
객체 클래스 퓨전의 최대 200ms 대기와 터널 벽 제거 보호점의 **즉시 50ms
stamp 매칭**이 별개임을 확인한다. 후자는 `/YOLO/BBox`의 YOLO11m
객체 box를 사용하며 TwinLite 차선·주행가능영역을 뜻하지 않는다.
25ms 후보는 유효 빈 검출, 아직 미도착, 시간창 밖, timeout, 잘못된
stamp, 보호 OFF를 구분한다. `matched_empty`는 객체가 검출되지
않았다는 뜻일 뿐 LiDAR 장면에 장애물이 없다는 판정이 아니다.

후속 [L1-B3-G](../../packages/asmc_perception/lidar-l1b3g-validation-20260928.md)에서
같은 2,562 scan 중 1,011 valid filter 입력으로 geometry까지 재생했다.
107건의 geometry 변화 중 음영 99건은 모두 valid-empty 선택으로 broad FOV
보존이 꺼진 결과다. 추가 매칭 304건을 검출 정확도 향상으로 바꾸지 않으며,
25ms 후보는 운영 미채택이다. 기하·연속주행 수락 전까지 판제는 기존
`/last_lidar`와 `/lidar_nogps`의 frame·stale 제약을 유지해야 한다.

## L1-B4-A 기하 보호 비교 — 소비 인터페이스 변경 없음

[격리 결과](../../packages/asmc_perception/lidar-l1b4-validation-20260929.md)는
독립 브랜치의 삭제 정책 실험이며 새로운 운영 publisher가 아니다.
valid-positive, valid-empty, unsupported-only, 미수신/시간 초과,
잘못된 stamp, 보호 OFF를 구분하고, 메시지 수신과 실제 보호점 생성도 구분한다.
빈 camera 결과는 LiDAR 공간에 장애물이 없다는 근거가 아니다.

camera semantic 보호와 geometry foreground 보호는 별도 마스크다.
기하 보호점을 car/person으로 표기하거나 후단 OBB 거부를 우회하지 않는다.
벽과 분리할 증거가 부족하면 uncertain으로 남기며 낮은 점 수·카메라 미검출만으로
점 삭제를 정당화하지 않는다. 이 진단은 격리 JSONL에만 있고 기존 ROS wire에 추가하지 않았다.

합성에서는 객체 분리가 개선됐지만 실제 정차 입력에서는 벽 근처 구조물 조각이
다수의 최종 객체로 남는 위험이 확인됐다. 따라서 후보는 운영 미채택이며
판제 fragment 억제/ID 재연결/소실 유예를 제거할 근거가 아니다.
local/map 좌표·GPS 상태·관측 stamp의 설계 계약도 이 실험으로 자동 연결되지 않는다.

## L1-B4-B Capture 비교 — 운영 소비 계약 유지

[새 실제 GT 비교](../../packages/asmc_perception/lidar-l1b4b-capture-validation-20260929.md)는
105장면의 raw→voxel→cluster 점 계보를 검사했다. 기존 wall-stage 대상점 보존이 이미 100%였고
기하 후보는 labelled target 지원 객체를 늘리지 못하면서 구조물 조각 출력을 늘렸다.
따라서 `/last_lidar`, `/lidar_nogps`의 운영 producer와 frame·stamp 의미를 바꾸지 않는다.
판제의 안정화 보완 로직을 제거할 근거도 아니다.

GT와 camera missing/empty fixture는 격리 평가 전용이다. Capture의 trigger surrogate stamp,
5회 정차 warmup, native IMU 시간 오류를 실제 관측 시각·tracking·GPS 전환 수락으로 쓰지 않는다.
다음 비교는 충분한 관측점의 군집 분할/누락 후보 한 단위이며 ground/ROI/wall/OBB 또는
판제 메시지 연결을 동시에 변경하지 않는다.
