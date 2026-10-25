# 인지·판제 좌표계와 IMU 공통화 검토

## 상태와 착수 순서

2026-10-25에 `feat/lidar-hd-map-filter-audit`의 `80b64676` 코드를 대조한 후속 설계다.
**PR #108은 main `ab8db094`에 병합됐다. 다음은 [판제 전환·구 패키지 폐기 기반 B0–B4](../../src/asmc_lidar/docs/cutover-foundation.md)이며,
아래 광범위한 리팩토링은 그 기준선을 마련한 뒤 진행한다.** 최초 코드 대조 SHA는 당시 근거로 보존한다.
이 문서 추가는 공용 라이브러리 구현, pose 생산자 통합, 판제 입력 전환이나 운영 수락을 의미하지 않는다.
현재 실행·검증 정본은 [LiDAR 인계](../handoffs/lidar-migration.md)와
[패키지 파이프라인](../../src/asmc_lidar/docs/pipeline.md)을 따른다.

## 권장 결정

- 인지의 공용 local 결과는 **후륜축 중심의 실제 `base_link`**로 제공한다.
- 센서 원본·수평화 좌표는 내부 계산에 유지한다. 모든 점을 먼저 map이나 base_link로 옮겨야 하는 것은 아니다.
- HD맵·미션 링크·전역경로·일반 Frenet은 **`map`**을 유지한다.
- map 객체와 지도 관련성 판단은 **해당 관측 시각에 유효한 pose**가 있을 때만 제공한다.
- GPS 무효가 local 검출·카메라 연결을 중단시키지 않게 한다. 다만 필요한 IMU·속도가 무효인 경우와 구분한다.
- 공통화의 우선 대상은 좌표·시각·보정·유효성의 의미다. 코드 이동만으로 정확도·지연 개선을 선언하지 않는다.

ROS [REP 105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)는
차체에 고정된 base_link, 연속적이지만 누적 오차가 생길 수 있는 odom, 전역 위치 보정으로
불연속 변화가 가능한 map을 구분한다. [REP 103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)의
차체 축은 X 전방·Y 좌측·Z 위, 카메라 optical 축은 Z 전방·X 우측·Y 아래다.
이 규약은 현재 패키지의 후륜축 원점·UTM 지도 원점·장착값을 대신 정해주지 않는다.

## 현재 경로와 소비 계약

| 단계 | 현재 구현 | 유지하거나 정리할 경계 |
|---|---|---|
| YOLO | 원본 영상 bbox·header·원본 클래스/그룹 | 2D 추론은 IMU로 영상을 회전하지 않음. 3D 연결은 별도 투영 |
| TwinLite | model-grid class/confidence → 카메라별 지면 근거 → shared family | 렌더링한 BEV raster를 다시 피팅 입력으로 사용하지 않음 |
| family | 원본 시각 IMU를 선택해 실제 base_link의 지면과 xyz 계산 | nominal 높이0.36m, M1/M0·유효성·observed/inferred 유지 |
| 비교 BEV·영상 BBox 어댑터 | family가 선택한 CameraGroundProjection 재사용 | 같은 원 JPEG·epoch·보정 ID 대응, 재조회·이중 보정 금지 |
| 새 LiDAR | ROI/voxel → 수평화 → 높이/지면 → 군집/OBB/추적 → 전방 fusion | 내부 수평화 좌표를 base_link로 잘못 표기하지 않음 |
| 새 LiDAR 출력 | local/base_link와 pose 조건부 map의 공용 배열·관측 | local/map은 같은 session/generation/scan_sequence의 두 표현 |
| HD맵·상부 전달 필터 | map OBB footprint·전체 cluster support 조회, local 하부 높이 확인 | 기본 filter·10cm·점군3점/10% AND OBB10%. 발행 배열만 제외, tracker 보존. 화면 샘플로 판정하지 않음 |
| 일반 판제 | planning_inputs → map 객체·차량 상태 → Behavior/Frenet → map 궤적 | 현재 base_link 입력을 거부하므로 단순 remap 전환 불가 |
| 음영 판제 | base_link family와 영상 투영·차선 소비 어댑터 | 일반 Frenet 입력과 별도 경계·소비 수락 |

주요 코드:
[카메라 보정 선택](../../src/asmc_camera/src/asmc_camera/runtime/ground_projection.py),
[family 실행](../../src/asmc_camera/src/asmc_camera/nodes/lane_vectorizer_node.py),
[비교 BEV](../../src/asmc_camera/src/asmc_camera/bev/comparison_projection.py),
[판제 영상 투영](../../src/jamming_lane_boundary_adapter/src/ground_projection.cpp),
[LiDAR 실행](../../src/asmc_lidar/src/ros/runtime.cpp),
[공용 출력 변환](../../src/asmc_lidar/src/ros/typed_output.cpp),
[판제 frame 검사](../../src/asmc_planner/src/Parser/PlanningInput.cpp).

운영 Main은 계속 별도다. [main.launch](../../src/lidar+sensorfusion/src/Main/main.launch)의
`enable_common_geometry`와 `lidar_level_roll` 기본값은 false다. 새 asmc_lidar의 공통 경계·roll+pitch
기본값이 운영 Main까지 자동 적용됐다고 표시하지 않는다. 실제 실행은 launch 인자와 ROS param도 확인한다.

## 세 좌표계와 변환 불변식

1. **Sensor:** 장착된 센서 원점·축. 센서 기준 거리/직육면체 ROI를 계산한다.
2. **Leveled sensor:** 실제 적용한 roll/pitch 수평화 이후의 계산 좌표. 실제 차체 축과 다르다.
3. **Base:** 차체에 고정된 후륜축 중심. 차체가 기울면 base_link도 기울며 지면은 기울어진 평면이다.

`T_BS`를 base←sensor 장착 변환, `L`을 leveled←sensor 회전, `T_MB(t)`를 map←base pose라 하면:

```text
p_B = T_BS · inverse(L) · p_L
p_M = T_MB(t_scan) · T_BS · inverse(L) · p_L
```

`L`은 근처 IMU로 다시 추정하지 않고 실제 전처리에서 사용한 행렬을 context에 보존한다.
중심·8코너·장축은 같은 변환을 적용한다. 이미 base_link로 발행한 코너를 소비자가 다시
수평화하거나 mount translation을 더하지 않는다. 속도는 위치와 다른 변환 계약을 가진다.
[변환 코어](../../src/asmc_lidar/src/core/frame_time_core.cpp)와
[scan 좌표 이력](../../src/asmc_lidar/src/geometry/scan_frame.cpp)이 현재 근거다.

## map 중심과 local 중심의 선택

### 가까운 인지와 센서 fusion

카메라와 LiDAR의 상대 장착 관계에는 GPS가 필요하지 않다. 검출·근거리 형상·센서간 투영을
전역 위치에 종속시키면 GPS 갱신·무효·복귀가 불필요한 결과 변동을 만들 수 있다.
local 기하를 정본으로 보존하고 지도 판단을 부가 결과로 두는 편이 음영구간·재시작 검증에 유리하다.

그러나 base_link는 움직이는 좌표다. 전 프레임의 위치를 그대로 다음 프레임 Kalman에 넣으면
자차 조향·이동이 객체 운동으로 섞인다. 현재 scan frame 정렬과 상대 CA 모델의 보상 의미를
보존하고, CAN 이동량을 중복 차감하지 않는다. GPS가 없는 절대속도를 추정했다고 간주하지 않는다.
새 공용 LidarObject의 relative/absolute velocity와 dynamic validity는 현재 false다.
이 의미를 보존하는 추적·속도 보완은 재호와 별도 검증한다.

연속 local 기준 odom이 추적에 유리할 수 있지만, 검증된 터널 상대 이동 입력 없이 새 odom을
선언하지 않는다. 기존에 보류한 카메라 ego motion·시간 누적을 이번 공통화의 필수조건으로 확대하지 않는다.

### HD맵 조회

객체를 map으로 옮기는 방법과 지도를 local로 옮기는 방법은 동일 시각의 동일 pose를 사용하면
같은 공간 관계다. 지도만 local로 회전해도 GPS 오차·지연이나 터널 위치 소실은 해결되지 않는다.
위치 근거가 없으면 지도 기반 제외를 중단하고 local 객체와 판단 불가 사유를 보존한다.

현재 권장은 정적 grid를 한 번 로딩하고 scan별 합성 변환을 한 번 계산해 필요한 점·OBB만 조회하는 것이다.
매 scan에 전체 raster를 local로 warp하지 않는다. 여러 소비자가 같은 주변 지도를 반복 사용해
비용이 확인될 때만 주변 영역 캐시를 대조한다. 캐시를 쓰더라도 pose 시각·지도 SHA·유효 범위가 필요하다.
현재 [HD맵 audit](../../src/asmc_lidar/src/ros/runtime.cpp)은 이 정적 grid 조회 구조다.

GPS5Hz가 map 출력5Hz를 뜻하지는 않는다. 현재 LiDAR의
[bounded pose](../../src/asmc_lidar/src/geometry/bounded_pose.cpp)는 과거 유효 GPS와 scan 자세·속도로
최대300ms의 제한 예측을 한다. 관측/예측·GPS age·복귀 상태를 남겨야 하며,
10Hz 출력이 10Hz 독립 위치 측정이나 지도 경계 정확도 보장은 아니다.
map이 무효일 때 정상 빈 결과로 바꾸거나 마지막 pose로 계속 생성하지 않는다.
현재 POSE_UNAVAILABLE 같은 무효 상태 전달과 단순 미수신도 구분한다.

### 일반 경로 계획

HD맵 링크·전역경로·Frenet s/d와 map 궤적 계약은 유지한다. 전체 판제를 base_link로 바꾸는 작업은
목표/경로/궤적/제어 어댑터까지 넓어지며 현재 개선의 선행조건이 아니다.
가까운 충돌 기하를 local로 계산하는 것은 가능하지만, 궤적과 객체가 같은 기준 시각·좌표인지 먼저 확인한다.
지도 위치 무효 시 음영 경로 선택과 정지·fallback 정책은 판제 책임이다.

## 확인한 중복과 불일치 가능성

### 위치 상태 생산자 세 경로

| 경로 | 현재 코드/설정의 특징 | 통합 전 확인 |
|---|---|---|
| bridge `/vehicle/state` | GPS 위치를 직접 사용, 수신 시간 기반 속도 필터, GPS/IMU의 오래된 header 시각 | GPS 안테나와 후륜축 기준점·yaw basis·시각 의미 |
| planning_inputs `/planning/vehicle/state` | 후륜축 lever arm, ENU→UTM grid yaw, source 시각 예측·GPS 위치 보정 gain0.35 | 판제의 현행 개선과 응답 특성을 보존 |
| 새 LiDAR pose history | 자체 IMU/CAN 이력, GPS 제한 예측, 최대 age300ms와 GPS 복귀 gate | scan 시각 pose와 판제 pose의 차이·이유 |

근거:
[bridge 추정기](../../src/asmc_bridge/src/vehicle_state/vehicle_state_adapter.cpp),
[판제 추정기](../../src/planning_inputs/src/vehicle_state_adapter.cpp),
[판제 설정](../../src/planning_inputs/config/vehicle_state.yaml),
[LiDAR pose 이력](../../src/asmc_lidar/src/geometry/pose_history.cpp).
이 표는 세 생산자가 동일하다는 뜻이 아니며, 차이를 확인했다고 현행 판제 추정기를 bridge의 옛 구현으로 되돌리지 않는다.
`frame_id=map`만 같아도 같은 pose는 아니다. 같은 시각·기준점·원점·yaw·보정 방식을 맞춰야 한다.

현재 [VehicleState](../../src/asmc_msgs/msg/VehicleState.msg)는 map의 x/y/yaw와 하나의 valid를 제공한다.
이를 그대로 local IMU 입력 대신 쓰면 GPS 무효가 local 처리를 막을 수 있다.
자세·속도의 사용 가능성과 map 위치의 사용 가능성을 분리한 조회 계약을 먼저 설계한다.
단일 pose 정본은 목표이며, 현재 해당 공용 생산자가 구현된 상태는 아니다.

### IMU 시각 선택 차이

- 카메라: SensorFrameTiming으로 원본 JPEG/IMU를 연결하고 동일 source clock·session 안에서 quaternion SLERP.
  최대 bracket100ms·수신 age350ms·3뷰 skew35ms, 근거 부족 시 세 뷰 모두 M0.
- LiDAR: 현재 scan header 이전의 최신 유효 IMU/CAN을 선택한다. 기본 age는 각각120ms다.
  원본 카메라 clock과 Velodyne device-hour clock을 동일하다고 가정하지 않는다.
- LiDAR PointCloud2는 현재 x/y/z/intensity이며 점별 획득 시각을 제공하지 않는다.
  scan 자세 보정은 점별 deskew 완료와 다르다.

기하 수학 공통화와 timestamp 정책 변경은 별도 커밋·대조로 분리한다.
카메라의 bracket 정책을 scan에 무조건 복제하거나, LiDAR의 최신 표본을 카메라에 대입하지 않는다.
입력별 시각 근거·선택 표본·정책·실패 이유를 context로 조회할 수 있어야 한다.

## IMU 공통 모듈의 경계

공통 원리는 `R_WB = R_WI · R_BIᵀ`, `n_B = R_WBᵀ · [0,0,1]`이다.
카메라는 이 법선으로 픽셀 광선과 지면의 교점을 바꾸고, LiDAR는 이 자세에 대응하는 회전으로 3D 점을 수평화한다.
기존 센서 mount를 한 번만 적용한다. nominal 높이0.36m는 동적 상하 운동 추정값이 아니다.

| 공통화 후보 | 센서/알고리즘에 남길 책임 |
|---|---|
| quaternion 정규화·SLERP·회전 검증 | JPEG 디코딩·YOLO/TwinLite 입력·TensorRT 실행 |
| IMU mount와 body 자세·지면 법선 | 픽셀→지면/BEV homography·sparse semantic 재투영 |
| source/receipt clock·bounded 이력·reset/무효 처리의 공통 기초 | 센서별 보간/표본 선택·대기 예산과 fallback 정책 |
| rigid transform 합성·역변환·방향/벡터 변환 | LiDAR point leveling·지면 제거·군집·OBB |
| 보정 식별자·불변 frame context·대응 fixture | family 폭 prior·joint fit·support와 tracker 모델 |

현재 카메라의 [ground_projection](../../src/asmc_camera/src/asmc_camera/runtime/ground_projection.py)은
이미 family solver와 분리돼 있다. [CameraGroundProjection](../../src/asmc_msgs/msg/CameraGroundProjection.msg)은
family worker의 실제 프레임별 선택을 비교 BEV와 BBox 어댑터가 재사용하게 한다.
다만 metadata 생산 주체는 여전히 family worker다. lane OFF에서도 필요한 미래 YOLO 지면 투영은
독립 소비 연결을 설계해야 하며, 지금 모든 센서의 공용 서비스가 완성됐다고 표시하지 않는다.

공용 수학을 family나 lidar 추론 패키지 한쪽에 계속 귀속시키기보다, 필요한 최소 기하·시각 코어를
가칭 `asmc_geometry` 같은 작은 라이브러리로 분리하는 것이 후보 방향이다.
기존 `asmc_lidar_frame_time`의 공통 부분을 재사용하고 C++/Python은 같은 fixture로 수치·실패 의미를 검사한다.
Python binding은 배포·호출 비용까지 비교한 뒤 결정한다. 즉시 모든 Python 연산을 C++로 바꾸지 않는다.
`asmc_msgs`는 wire 정의만 담당하며 추정기·계산·파일 I/O 코드를 넣지 않는다.
매 처리마다 ROS 서비스로 자세를 요청하는 구조나 새 대형 중계 노드는 만들지 않는다.

## 성능상 먼저 볼 항목

1. scan별 합성 행렬·프레임별 보정 선택을 불변 context로 한 번 만들고 재사용한다.
2. map 필요 시에만 파생 기하를 만들되 HD맵 audit·recorder·화면 소비도 실제 수요에 포함한다.
   현재 TypedOutput은 local/map 배열을 둘 다 조립한다. 구독 수요만 보고 map pose 계산을 끄면 audit가 깨질 수 있다.
3. 원본 점군을 local/map/debug용으로 반복 깊은 복사하지 않는다. 원본/수평화 버퍼의 소유권과 수명을 확인한다.
4. 지도 raster 재생성보다 정적 grid 조회와 필요한 점 변환을 우선한다.
5. debug/JSON/ROS 배열은 실제 수요에 맞춰 조립한다. 진단을 정본 객체 입력으로 쓰지 않는다.

작은 회전행렬 연산 절약만으로 TensorRT 추론이나 군집 지연이 크게 줄어든다고 예상하지 않는다.
현재 가드레일의 긴 OBB 문제는 군집·관측 형상과 support 판정 문제도 포함하며 좌표계 변경만으로 해결되지 않는다.
이 설계 자체에 대한 속도 개선율은 아직 미측정이다. 이전 HD맵/높이 ROI 실험 수치를 공통화 효과로 사용하지 않는다.

## PR #108 이후 작업 단위

| 순서 | 작업 | 완료 근거 | 제외 범위 |
|---|---|---|---|
| C0 | 현행 생산자·소비자·frame/time/기준점 목록과 동일 입력 fixture 고정 | 같은 시각 pose 차이와 local/map round-trip 재현 | 알고리즘·토픽 변경 |
| C1 | 기하·시각 공통 라이브러리 최소 분리 | 기존 수학·무효 처리 동등성, C++/Python fixture, 독립 빌드 | 위치 추정 gain·보간 정책 변경 |
| C2 | 자세 조회를 센서별 정책으로 연결 | 카메라 M1/M0 동등성, LiDAR leveling·카메라 투영 동등성, 시각 실패 재현 | 새 ego motion·점별 deskew |
| C3 | pose 정본과 scan 시각 조회 설계·대조 | lever arm·yaw·원점·GPS0/중단/복귀·예측 오차 비교 | 판제 단독 정책 변경 |
| C4 | 중복 계산/출력 조립 축소 | 입력→출력 p50/p95/p99·최대 공백·RSS/CPU·복사 수 비교 | 근거 없는 품질 경량화 |
| 공동 전환 | 검증된 pose/객체 소비 경계 연결 | 판제·재호와 고정 bag 및 실제 구간 수락·복구 확인 | 사용자 확인 없는 운영 전환 |

전환 기반 B0–B4 뒤 공통화를 시작할 때는 **C0만 구체화**한다. 공통화할 패키지 이름이나 새 메시지는 그 경계 검토 후 확정한다.
안승현은 카메라·HD맵 연결과 공통 기하 대조, 재호는 추적·속도·fusion 의미를 확인한다.
판제 담당은 현재 pose/궤적·Frenet 소비와 fallback 정책을 확인한다. 관련 공개 계약 변경은 공동 검토한다.

## 필수 회귀와 복구

- quaternion q/-q·mount 회전·yaw-only·roll/pitch 결합과 카메라/센서 optical 축.
- 센서→수평화→base→map round-trip, 중심/8코너/장축·yaw 대응과 입력 버퍼 불변성.
- 원본/수신 시각·clock domain·세션·중복/역행·결측·늦은 도착·처리 중 reset.
- GPS 정상→0→중단→복귀에서 local 유지와 map 무효, source age/선택 근거 보존.
- IMU·속도 무효 시 정상 빈 결과와 혼동하지 않음. 무효 속도를0/static으로 승격하지 않음.
- 같은 RGB/model-grid의 family 역할·기하·support·M1/M0, BEV와 BBox의 exact-image 정합.
- 고정 scan의 점군/OBB/track/fusion·지도 판정 동등성. 의도적 정책 차이는 별도 비교.
- 공개 메시지 MD5·기존 Main·판제 소비와 새 라이브러리 독립 빌드 유지.
- 단순 수학 분리 단계에서는 출력 동등성을 우선하고, 성능은 동일 입력에서만 비교한다.

각 단계는 이전 설정·바이너리·fixture·SHA를 보존한다. 초기에는 시험 namespace에서 대조하고
기존 Main·판제 소비는 유지한다. pose 계산이나 센서 선택의 의미가 바뀐 단계는 별도 수락한다.
기존 [카메라 IMU 설명](camera-imu-projection.md), [센서 시각](bridge-sensor-timing.md),
[LiDAR 계약](../contracts/lidar-planning-consumer-v1.md)의 현행 의미를 이 문서의 후보안으로 덮어쓰지 않는다.
