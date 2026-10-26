> PR #111까지의 이관 기록이다. 아래의 현재·다음 작업은 당시 판단이며, 현행 실행은 [B4 실행 정본](../../../src/asmc_lidar/docs/planning-runtime.md)을 따른다.

# asmc_lidar 정본화와 판제 전환 기반

**2026-10-26 후속 구현:** 안승현이 B1/B2의 기존 기능 보존 이식·공용 추적 관측·판제 직접
소비를 구현했다. [구현·실행·검증 정본](../../../src/asmc_lidar/docs/planning-runtime.md)을 따른다. 재호의 속도/터널 개선을
이 연결의 선행조건으로 두지 않는다. B3 실제 미션 인수·B4 구 패키지 삭제는 미완료다.
아래 B0 코드 대조는 `ab8db094` 당시의 근거이며 미연결 기능 목록을 현재 미구현으로 해석하지 않는다.

## 결정과 B0 기준선

**판제 소비 전환과 구 패키지 폐기를 먼저 완결할 기반을 만든다.** 광범위한 좌표·IMU
리팩토링과 3카메라 fusion 확대는 이 기준선을 확보한 뒤 진행한다. 손재호의 속도·추적·
터널 개선과 안승현의 지도·카메라 지원은 같은 계약 위에서 작은 변경으로 병행한다.

2026-10-26 문서 점검에서 main `ab8db094`(PR #108 병합)를 코드 대조 기준으로 삼았다. 이 문서는
**기능 대조·작업 순서·인수 기준**이며 새 판제 어댑터 구현, 운영 입력 전환, 구 패키지 삭제의
완료 기록이 아니다. 기존 L1 manifest와 PR #103–#108 실험의 SHA·수치는 당시 근거로 보존한다.

- 현재 운영: `lidar+sensorfusion`의 Main → `/last_lidar` + 같은 stamp의 `/lidar/motion_debug`
  → `planning_inputs`의 `validated_lidar` → `/planning/objects`.
- 새 실행: `asmc_lidar/lidar.launch` → `/migration/lidar/objects_local,map` 및
  `observations_local,map`. 판제는 아직 이 출력을 직접 소비하지 않는다.
- 목표: `asmc_lidar`의 공용 관측을 판제 입력 경계가 직접 소비한다. 새 구형 wire wrapper는
  만들지 않으며, 전환 검증 뒤 구 Main·nested workspace를 제거한다.
- 책임·전체 이력은 [팀 인계](migration-through-pr111.md), 완료 상태는
  [작업표](migration-tasks-through-pr111.md), 실행은 [README](../../../src/asmc_lidar/README.md)를 따른다.

아래 실행/기능 대조는 **PR #108 병합 직후 B0 시점**이다. 10월26일 B1/B2 구현에서
정식 namespace·공용 속도·추적 관측·판제 입력이 추가됐다. 현재 실행은
[planning-runtime](../../../src/asmc_lidar/docs/planning-runtime.md)을 따르며 아래 미연결 표기를 현재 제한과 혼동하지 않는다.

## 현재 적용 설정

실행 값의 정본은 [lidar_pipeline.yaml](../../../src/asmc_lidar/config/lidar_pipeline.yaml)이다.

| 항목 | 현재 적용과 한계 |
|---|---|
| 입력·좌표 | LiDAR·IMU·허용 Competition CAN 입력. 후륜축 `base_link` local, 유효 scan pose에서만 map |
| local 준비 | scan 시각 IMU·CAN age 각각120ms, 최대50ms 대기. GPS만 없어도 local은 계속되지만 IMU/CAN 부족이면 `INPUT_INVALID` |
| map 준비 | GPS 최대300ms 제한 예측·정상3회 복구. GPS0/무효/stale에서 이전 map 위치를 무기한 쓰지 않음. 5Hz GPS를 독립적인10Hz 위치 측정으로 바꾼 것이 아님 |
| 기본 전처리 | 센서 ROI x[-30,50]·y[-20,20]·z[-2,3]m, 거리0.5–100m, voxel0.1m, roll/pitch 수평화·지면 제거 |
| 높이 ROI | 수평화 후 명목 지면 위2.5m 초과점 제거, 명목 base 높이0.36m. 높은 물체의 상부 형상은 잘릴 수 있음 |
| 카메라 fusion | 기본 ON, **전방만**. `mission16_v1`, unique, ±50ms·200ms 대기·IoU0.25·확인/변경/유예5/3/3. YOLO 생산자는 confidence0.40·NMS0.50, LiDAR 추가 cutoff0.0 |
| 지도 | `asmc_hd_map` v7 정본에서 만든 SHA 검증 bundle. A029/A175/A406와 합류 실선·톨게이트 섬 수정 포함. 판제의 옛 road mask와 구분 |
| 실제 전달 필터 | 기본 `filter`. 도로 합집합 바깥 경계 **안쪽10cm**에서 점군 **3점 이상 AND 전체10% 이상**, OBB 면적 **10% 이상** 모두 요구. tracker·측정 형상은 보존 |
| 보호·무효 | 최신 실제 YOLO vehicle(car/bus/truck)·dynamic_obstacle(person/dog/deer/bicycle/stroller) 매칭 보호. LiDAR 속도·미션 근접만으로 보호하지 않음. 지도/pose 무효이면 지도 제외 중단 |
| 상부 객체 | 충분한 유한 support(10점 이상), 알려진 카메라 분류 여부와 하부 근거를 검사. 미분류 객체의 최저 support와 OBB 하단이 모두 base z=1.884m(지붕1.284+여유0.60)보다 높을 때 제외. GPS와 독립 |
| 터널 필터 | `experimental_tunnel_filter` 기본 OFF. ON에서 박스 튐이 관찰됐으며 벽·천장 처리 수락은 미완료 |

필터는 fusion 이후 발행 배열만 압축한다. local/map·관측은 같은 제외 ID 집합을 사용한다.
debug의 원본 track·점군은 검수용이므로 발행 제외 객체가 화면의 원본 계층에 남을 수 있다.
웹의 숨김 체크박스는 처리 설정을 변경하지 않는다. [필터 검증 이력](../../../src/asmc_lidar/docs/hd-map-filter-analysis.md#a19-10cm-경계와-실제-발행-필터--2026-10-25)을 따른다.

## 구 Main과 새 패키지의 기능 대조

아래는 소스 연결 여부 대조다. 구현됨을 전체 주행 품질 수락으로 해석하지 않는다.

| 구 Main 제공 기능 | asmc_lidar 현재 상태 | 전환 전에 할 일 |
|---|---|---|
| ROI·voxel·지면·군집·OBB | 구현. 높이 ROI·전달 필터는 PR #108 추가 | 기존 비교 근거 재사용, 추가 필터 차이는 별도 설명 |
| ID·연관·CA Kalman·track 생애주기 | 구현. 예측 track도 내부 유지 | 센서 중단·ID 재사용·재시작·GPS 복귀 회귀 |
| 현재 관측 track 발행 | 구현. `miss_count=0`만 공용 출력 | 구 Main의 `MakePlannerObjects`도 같은 관측 조건. 예측 발행 기능이 사라진 것으로 설명하지 않음 |
| 중심·8코너·방향·크기 | 구현. 공용 `center`는 **OBB 중심** | 구 wire의 `cluster_center`와 `obb_center`가 별도였음. KF/군집 중심을 쓰던 소비자는 의미 확인 |
| 원본 YOLO 클래스·그룹과 유지 정책 | 전방 구현, 3뷰 수신 API 제공 | 실제 카메라/scan matching 회귀. 3뷰 객체 매칭은 후속 |
| 판제 numeric type | 공용 출력은 그룹·원본 라벨 문자열을 제공 | 판제의 구 enum과16종/3그룹 대응을 직접 소비 경계에서 명시. unknown을 정지/무시 대상으로 바꾸지 않음 |
| 내부 속도 추정 | CA 상태 유지, MotionClassifier 진단 복사본 연결 | 부호·dt·단위·기준점·자차 보상 검증 후 공용 출력에 연결 |
| 판제용 보정 속도 벡터·speed_kph | **공용 출력 미연결**. 상대/절대 validity 모두 false | 구 Main의 최종 적용 속도와 새 KF 진단 속도를 혼동하지 않음. 측정/보정 근거와 유효 조건 확정 |
| is_dynamic 및 UNKNOWN/LOW_THREAT/PENDING/DYNAMIC | 내부·웹/trace 진단은 있음. **공용 dynamic_valid=false** | 실제 움직임은 카메라 소비 그룹과 구분. 판제에 필요한 상태·유효성의 전달 방법 합의 |
| yaw rate | 공용 필드 있으나 유효하게 생산하지 않음 | 구 `Msg/TrackedObject`에도 없던 항목. 구 기능 누락으로 계산하지 않음 |
| GPS 무효 처리 | scan-time local 유지, map은 `POSE_UNAVAILABLE` | IMU/CAN 누락과 GPS 누락 구분. 터널의 공개 절대속도·동정적 판정은 아직 미수락 |
| road/known-static/구조물 catalog/relevance/clipping | 새 HD맵·상부 전달 필터는 적용. 구 체인 전체를 이식하지 않음 | [편입표](../../../src/asmc_lidar/docs/filter-migration.md)로 필요한 기능별 차이 확인. 원본 OBB clipping은 정본에 넣지 않음 |
| 최종 /last_lidar·motion_debug 결합 | **새 경로에는 없음**, 공용 관측으로 대체할 목표 | 판제에 직접 공용 타입·상태 소비 경계 구현. 진단 JSON을 다시 정본으로 붙이지 않음 |
| debug 점군·Marker·웹 | 단계 observer, RViz 옵션·웹 구현 | 원본/발행 후 계층·stale 표시와 단일 producer 확인 |
| HSV 영상 인지·구형 UDP 객체 송신·차량 제어 | **의도적으로 미이관** | 카메라/판제 책임으로 분리. 구 UDP7510·LiDAR의 `/ctrl_cmd_0`를 되살리지 않음 |

**속도 계산 자체가 사라진 것은 아니지만, 구 판제가 사용하던 최종 속도·운동 정보가 새
공용 메시지로 전달되는 상태는 아니다.** `typed_output.cpp`는 velocity 필드를 채우지 않고
validity를 false로 둔다. 기본값0을 정지 객체의 측정 속도나 유효 static으로 소비하면 안 된다.
`pipeline.cpp`의 MotionClassifier는 진단 복사본만 갱신하며 GPS 무효 때 map 이력을 비운다.
구 판제의 네 단계 운동 상태를 공용 메시지의 `is_dynamic` bool 하나로 임의 압축하지 않는다.
실제 소비에 필요한 단계·불확실성의 표현은 B1/B2 계약에서 확정한다.

현재 CAN 경로의 XYZ 속도 norm·후진 부호, IMU 무효 초기120ms 경계는 기존 미수락 항목이다.
음영구간의 형상 검출이 동작하더라도 절대속도·동정적 판단까지 검증됐다는 뜻은 아니다.

### 코드 확인 위치

- 새 출력: [typed_output.cpp](../../../src/asmc_lidar/src/ros/typed_output.cpp), [LidarObject](../../../src/asmc_msgs/msg/LidarObject.msg).
- 새 운동 진단: [pipeline.cpp](../../../src/asmc_lidar/src/pipeline/pipeline.cpp),
  [motion_display.cpp](../../../src/asmc_lidar/src/tracking/motion_classifier.cpp), [debug_publisher.cpp](../../../src/asmc_lidar/src/ros/debug_publisher.cpp).
- 구 판제 결과: [PlannerObjectAdapter.cpp](https://github.com/INHAautonav/2026-ASMC/blob/b2e8329f1cb71f11150d5125335e01c5b8d252c5/src/lidar%2Bsensorfusion/src/Main/src/PlannerInterface/PlannerObjectAdapter.cpp),
  [PlannerObjectRosPublisher.cpp](https://github.com/INHAautonav/2026-ASMC/blob/b2e8329f1cb71f11150d5125335e01c5b8d252c5/src/lidar%2Bsensorfusion/src/Main/src/PlannerInterface/PlannerObjectRosPublisher.cpp).
- 현재 판제 연결: [objects.launch](../../../src/planning_inputs/launch/objects.launch),
  [tracked_wire_bridge.py](https://github.com/INHAautonav/2026-ASMC/blob/b2e8329f1cb71f11150d5125335e01c5b8d252c5/src/planning_inputs/scripts/tracked_wire_bridge.py),
  [objects_node.cpp](../../../src/planning_inputs/src/objects_node.cpp),
  [private DetectedObject](../../../src/planning_inputs/msg/DetectedObject.msg).

## 전환 기반 작업 단위

| 단위·상태 | 담당·구현 범위 | 완료 기준 |
|---|---|---|
| B0 · 완료 | 안승현: 현재 설정·기능 차이·이후 순서 고정 | 이 문서와 공용 계약·인계·작업표가 일치. 아래 문서/계약 검사 통과, 실행 변경 없음 |
| B1 · 구현/자동 검증 완료, 주행 인수 대기 | 안승현: 구 적용 속도·4단계 상태·필요 전달 정책 보존 이식. 손재호: 향후 속도 정확도 개선 | 같은 scan의 벡터·기준점·형상·상태 회귀와 GPS/IMU 중단·복구 검사 통과. 실제 속도 정확도/전체 코스 인수는 B3 |
| B2 · 구현/자동 검증 완료, 주행 인수 대기 | 안승현: 공용 추적 관측 직접 소비·구 Main/Msg 없는 전체 실행 연결. 판제: 실제 소비 인수 | session/generation/sequence·config·calibration·상태·속도 validity 검사. 정상 빈 결과는 전달, 무효/미표현 속도는 발행 보류. private 계약 유지 |
| B3 · 진행, 공동 인수 대기 | 공동: 고정 자료·실시간 대조 후 명시적 소비 전환 | 2026-10-26 사용자 대회용 MolitComp03·기본 조향 false의 새 LiDAR 직접 주행 확인. 기능 동등성/의도적 차이·전 미션·GPS0/복귀·카메라 중단·센서 중단·재시작·복구 인수는 남음. 정식 토픽 단일 생산자 |
| B4 · 미착수 | 안승현·재호·판제: asmc_lidar 정본화·구 패키지 폐기 | 전체 소비 참조 이전·깨끗한 빌드/설치/실행 확인 후 nested Main/Msg·전용 launch/wrapper/의존성·CI·잔류 설치 파일 정리. 현재 실행 안내는 새 패키지만 사용 |

B1/B2의 보존 이식과 연결은 [현재 실행 기준선](../../../src/asmc_lidar/docs/planning-runtime.md)에 기록했다.
새 `LidarTrackingObservation`/`LidarTrackingState`는 additive 공용 계약이며 기존 공용/구형 MD5와
과거 bag은 보존한다. 알고리즘 정확도 개선과 운영 인수는 같은 완료 선언으로 묶지 않는다.

기존/복구 wire bridge는 `Msg/TrackedObjectArray` 또는 같은 MD5의 `asmc_msgs/TrackedObjectArray`만
받는다. `LidarObservation`으로 이름만 remap하면 연결되지 않는다. 판제 private `DetectedObject`는
속도 validity bool이 없으므로 **필드 변환만으로 무효 속도 문제가 해결되지 않는다.** 센서/pose 무효,
정상 빈 결과, 미수신·stale을 각각 어떤 동작으로 소비할지 판제 담당과 확정한다.
판제 소유 입력 어댑터·private 계약 변경은 영향과 검사 결과를 명시한 PR로 검토한다.
인지 필터 이전을 이유로 TTC·추종·회피·정지·재출발·센서 장애 대응을 함께 바꾸지 않는다.

일반 HD맵/Frenet의 map 처리는 유지한다. 터널은 local 객체·local family·차량 상태의 시각/좌표를
맞춘 별도 소비 경계를 확인한다. 전체 판제를 base_link로 바꾸거나 가짜 odom을 만드는 것은
전환 선행조건이 아니다. 새로운 전역 위치 추정 없이 터널 map 객체를 계속 발행하지 않는다.

구 Main의 raw/final 출력, motion JSON, 카메라 BBox 어댑터, HSV·UDP·제어·debug·build 소비자를
각각 추적한다. `/lidar_nogps` 같은 역사적 이름은 문서 등장만으로 현재 발행 중이라고 간주하지
않고 실제 advertise/구독/실행 설정으로 확인한다. 필요한 동작을 새 경로에서 수락하기 전에
구 코드를 지우지 않으며, 불필요한 구 동작까지 전부 복제하지도 않는다.

B3 전까지 고정 구 launch를 복구 경로로 유지한다. 전환은 구 생산자를 종료한 후 새 소비를
선택하며, 양쪽을 같은 정식 토픽에 동시에 붙이지 않는다. B4 삭제 PR에는 마지막 구 소스 SHA와
검증 근거를 남기고 frozen 데이터/메시지 재현 자료를 보호한다. 이후 복구는 이전 검증 revision의
배포 절차로 관리하며, 폐기한 nested 실행을 상시 두 번째 정본으로 남기지 않는다.
현재 노드는 정식 `/perception/lidar/*` 등으로의 발행을 `allow_operational_output` guard로
차단한다. B3에서 정식 이름·소비 설정과 함께 명시적으로 검토할 배선이며, 이번 문서 작업에서
guard를 해제하거나 운영 namespace를 기본으로 바꾸지 않는다.

## 이후 개선 순서와 담당

| 순서 | 작업·담당 | 필요한 결과 |
|---|---|---|
| 기반과 병행, 운영 수락 전 확인 | **GPS 없는 터널 local 인식·벽/천장 개선 — 손재호**. 승현은 카메라·family·시간/보정 지원 | 아래 터널 회귀에서 실제 장애물 보존·구조물 혼입·local 공급 확인. 현재 터널 품질을 수락 완료로 표시하지 않음 |
| 기반 이후 | 제한된 구조/좌표·IMU 공통화 — 안승현·재호 | [C0–C4 설계](../../architecture/perception-coordinate-refactoring.md)를 필요한 단위만 적용. 판제 전환과 대규모 코드 이동을 한 비교에 섞지 않음 |
| 기반 이후 | 3카메라 YOLO matching/fusion — 손재호, 승현 연결 지원 | 뷰별 원본 픽셀/보정·시간·ID를 사용. 겹침 객체의 중복 확인·분류 충돌 해결. 느린/중단 뷰 때문에 전체 LiDAR를 대기시키지 않음 |
| 실패 근거에 따라 | 지도/필터 잔여 차이 — 안승현, 소비 확인 공동 | 위치 오차·경계·known-static/구조물 catalog의 필요한 기능만 편입. 행동 판단은 판제 유지 |
| 실패 근거에 따라 | OBB·추적·속도 안정화 — 손재호 | 혼합 군집·가림·visible face 변화·ID/속도 튐을 단계별 분리. 필요 객체 제거로 안정적으로 보이게 만들지 않음 |

### 손재호: GPS 없는 터널 local 인식

**사용자 요구:** 터널에서 GPS 없이 벽면·천장 혼입을 줄이고 실제 충돌 가능한 객체를 local로
제공한다. 카메라 주행가능영역 또는 family를 보조할 수 있지만 **바깥 흰선과 벽 사이 여유 공간에
객체가 있을 수 있으므로 선 밖 점군을 일괄 삭제하지 않는다.** 이 여유의 폭은 아직 검증되지
않았으며 임의의 고정 margin을 운영값으로 확정하지 않는다.

다음은 구현 제안이며 현재 실행에 연결된 기능이 아니다. 상세 구조물 근거 계약은
[기존 인계](migration-through-pr111.md#모드-없는-구조물-분리--손재호의-후속-구현-계약)를 재사용한다.

1. **LiDAR만의 실패를 먼저 분리한다.** 같은 raw에서 기본 높이 ROI·RANSAC·TunnelFilter
   OFF/ON·군집·OBB·추적을 차례로 비교한다. 2.5m cap 아래 천장 가장자리/설비와 벽·객체의
   혼합 군집은 지금도 남을 수 있다. 큰 OBB 전체를 지우는 대신 실제 지지점과 혼합 원인을 본다.
2. **지면/구조물의 관측 근거를 제한한다.** 천장은 지면 대비 높이·법선·하부 관측·차량
   통과 여유, 벽은 수직 평면의 유한 지지 범위·길이·돌출/깊이 단절을 대조한다. 평평한 트럭
   측면·정차 차량도 벽처럼 보일 수 있다. 벽 가까운 장애물과 낮은 돌출부를 보존하고,
   하부가 가려진 물체는 unknown으로 남긴다. 명목 높이0.36m를 매 순간 측정한 높이로 쓰지 않는다.
3. **카메라는 보조 근거로 사용한다.** 현재 원본 크기 확률은
   `/perception/camera/{front,left,right}/drivable/probability`, family는
   `/perception/lane/boundary_family`다. 수요 기반 출력·원본 크기/Geometry·상태/시각 계약은
   [카메라 설정](../../../src/asmc_camera/config/cameras_v3.yaml)과
   [runtime 계약](../../contracts/camera-runtime-v1.md)을 따른다.
   원본 크기 영상 구독 비용과 필요한 model-grid 경로를 먼저 비교한다. 렌더링한 BEV를 다시
   계산 입력으로 삼지 않는다. **현재 LiDAR는 이 semantic/family를 구독해 구조물을 제거하지 않는다.**
4. **3D 투영과 지면 투영을 구분한다.** 실제 LiDAR 점을 카메라에 연결할 때는 그 점의3D 위치와
   장착/IMU 변환을 한 번만 적용한다. 높은 천장점을 도로 위 점이라고 가정해 지면으로 눌러
   투영하지 않는다. drivable은 보이는 지면의 semantic이지 물리적 빈 공간/장애물 부재의 증명이 아니다.
5. **family는 역할·범위를 가진 근거다.** 바깥 흰선은 차로 경계이며 벽/도로 끝의 정답이 아니다.
   `observed/inferred`, support 범위·신뢰도·source age를 검사한다. 추정/외삽 곡선만으로
   hard exclusion 경계를 만들지 않는다. 선 밖·시야 밖·장애물로 난 semantic 구멍은 보수적으로 보존한다.
6. **입력 실패가 추가 삭제를 만들지 않게 한다.** session/generation·config/calibration·원본
   시각·IMU 대응을 확인한다. family/semantic STANDBY·stale·중단·시간 불일치·낮은 확신에서는
   LiDAR local로 복귀한다. 세 뷰가 모두 올 때까지 scan을 무기한 기다리지 않는다.
7. **시간 누적은 별도 조건이다.** 현재 scan의 구조물 근거부터 검증한다. 과거 벽/객체를 합치려면
   유효한 상대 ego motion 정렬이 필요하다. IMU 위치 이중적분·개발용 Ego GT·마지막 GPS로
   운영 위치를 만들어 쓰지 않는다. map을 local로 옮기는 것만으로 GPS 부재가 해소되지 않는다.

구조물은 객체 tracker에서 제외할 수 있어도 주변이 자유공간이라는 뜻은 아니다. 관측 경계나
미확정 점유를 판제가 필요로 하면 별도 계약으로 합의한다. 이 새 경계 표현 전체를 첫 전환의
필수 재설계로 확대하지 않으며, 이미 적용된 높이 ROI/상부 전달 필터와 후속 벽 억제를 구분한다.

### 터널 검증과 완료 근거

- 실제 객체 없는 벽/천장뿐 아니라 차로 안 객체·벽 근처/흰선 밖 객체·정차 차량·트럭 측면과
  상부만 보이는 물체·낮은 돌출물·가림을 포함한다. 필요한 객체의 신규 소실은 실패로 남긴다.
- 진입·내부·곡선·출구, 조향/가감속 roll/pitch, SUNNY/FOGGY·어두운 영상,
  GPS0/복귀·CAN/IMU 부족·카메라 중단/재시작을 구분한다.
- 동일 scan의 원본점 계보·단계별 제거 사유·혼합 군집·OBB·track ID와 공개 상태를 연결한다.
  웹은 원본/억제/발행 계층을 구분하고 제외 사유를 토글한다. 화면 숨김만으로 검출 개선을 주장하지 않는다.
- 객체 보존·구조물 잔존·시간/공급 공백·단계별 p50/p95/p99·CPU/RSS를 함께 기록한다.
  이미 합의한 ≥10Hz·공백≤250ms·freshness 기준은 평균 처리율과 구분한다. 새 비용/정확도 gate는
  시험 전 확정한다. 새 bag/실험은 저장 규약의 고유 run ID·입력/설정/지도/소스 SHA를 사용한다.

이 목록의 터널·3뷰·정확도 개선은 B1/B2 보존 이식으로 완료되지 않는다. 다음은 **실제
판제 소비 인수(B3)와 구 패키지 폐기(B4)**다. 재호의 터널 개선은 병행 작업으로 둔다.

## B0 문서 단위 검증 — 2026-10-26

- 문서 migration/history·현재 파일 링크/anchor·동결5개 검사 통과.
- runtime 문서 검사 통과: launch27개·직접 명령103개.
- 공용 메시지 MD5/계약·runtime 문서 unittest19개 통과.
- 변경은 Markdown과 기계 판독 계약의 구현 상태뿐이다. launch·알고리즘·운영 설정·메시지 정의는
  변경하지 않았다. 이번에는 ROS 실행·bag 수집·모델 품질·실시간 수락 시험을 수행하지 않았다.

이 결과는 B0 문서·기능 대조 완료의 근거이며 B1–B4 구현 완료나 판제 전환 수락이 아니다.
