# LiDAR 이관 설계 — L1

2026-10-22 결정. [L0–L8 작업표](migration-tasks.md)의 L1 결과이며, 진행 담당은 안승현이다.
손재호의 별도 개발은 병행한다. **아래 새 실행 구조·정책은 설계이며 아직 구현되지 않았다.**
현재 판제는 기존 nested Main을 사용한다. L2부터 한 단위씩 구현하고 L7에서 소비를 전환한다.

기계 판독 기준선은 [L1 manifest](../../../config/baselines/lidar_migration_l1_v1.yaml), 현재/목표의
전체 경계는 [통합 인계](../../../docs/handoffs/lidar-migration.md), 출력 의미는
[소비 계약](../../../docs/contracts/lidar-planning-consumer-v1.md)을 따른다.

## 1. 근거와 현재 상태

### 규정 출처와 입력 승인

[공식 운영규정](https://morai.atlassian.net/wiki/external/ZDBiNmI0MGFiNTY2NDRkYWIyZTMzNWFkYzkwY2Q4NTU)과
공식 후속 공지가 우선이다. workspace의 `docs/competition/references/regulations/source/v1.2-20260930.md`는
2026-10-05 복사본이며 SHA를 manifest에 고정했다. 2026-10-22 URL은 HTTP 200이지만 동적 본문을
추출하지 못했으므로 온라인 최신 내용 전체를 재검증했다고 표시하지 않는다. 팀 레포의
[규정 요약](../../../docs/competition/README.md), [v1.2 영향](../../../docs/competition/rule-v1.2-impacts.md)과
사본의 2-4·2-8을 대조했다. 초기 PDF는 과거 근거이며 현재 판단의 정본이 아니다.

- 2-8: Competition Vehicle Status, CollisionData, GPS/IMU/Camera/LiDAR 등 명시된 UDP만 사용한다.
- 2-4: Ego Detail Status·Ego Vehicle Status·Get TrafficLight Status는 판정 전용이다.
- Competition 제공 범위는 자세·x방향 속도·각속도·제어 입력 등이다. 위치, y/z 속도,
  가속도 등 미제공 필드를 0이라도 실제 측정으로 사용하지 않는다.

| 실제 연결 | 현재 코드·관측 | 이관 결정 |
|---|---|---|
| type787 / 9797→9798 → `/Competition_topic` | 허용 채널을 의도한 bridge 경로. 실제 현장 packet·필드 수락 미완료 | 실제 원천·형식·종방향 속도/부호/단위를 확인한 뒤 사용 |
| Ego / UDP9099 → `/Ego_topic` | 개발 호환 출력 | 새 본선 LiDAR는 구독·자동 fallback하지 않음 |
| Main의 `/Competition_topic`→`/Ego_topic` remap | 실제 속도 입력은 개발 Ego | 기준선 재현 전용으로 기록, 새 실행에는 가져오지 않음 |
| local UDP9798의 `#DetailEgoInfo$` 105B | 현재 decoder는 발행하지 않고 UNSUPPORTED로 진단 | 이름·포트만으로 Competition 수신 성공 처리하지 않음 |

`EgoVehicleStatus`라는 ROS 타입을 함께 쓰거나 bridge의 shared decoder 시험이 통과한 사실은
현장 Competition 입력의 승인이 아니다. 현재 decoder는 signed speed와 velocity XYZ를 모두
해석하고 publisher는 XYZ를 채운다. Main은 XYZ norm으로 속도를 얻어 부호를 잃는다.
새 속도 어댑터는 **허용된 x방향 속도만 SI·부호 포함으로 정규화**하고, 원본 필드·단위·축·
유효성을 기록한다. 어느 packet 필드가 해당 값인지 live 검증 없이 선택하지 않는다.
미제공 y/z=0은 측정이 아니라 종방향 운동 근사의 가정이며 진단에 표시한다.

### 코드 기준선

원본 동작 기준은 main `19bc76978a499bddacf13f5aaf0bb6d713785657`, L0 문서는 `4a16bfda`다.
2026-10-22 원격 main은 같은 SHA였고 이 구간의 runtime diff는 없다. source tree·설정·메시지·
지도·보정 SHA는 manifest로 고정한다. 이번 L1은 새 runtime 성능 측정이나 full-stack 빌드가 아니다.

현재 실행 순서는 다음과 같다.

```text
센서 ROS 수신 → latest scan snapshot
 → 기존 raw state 조립·HSV 카메라 처리
 → GPS/IMU/CAN gate
 → cloud → ROI/ego crop → voxel → pitch 보정 → 지면 → 군집 → OBB
 → 연관/KF·추적 → 운동 분류 → 전방 카메라 fusion
 → 지도·known-static·structure·relevance 필터 → /last_lidar
 → exact-stamp motion_debug 결합 → planning_inputs → /planning/objects → 판제
```

Main common geometry와 roll은 기본 OFF다. opt-in 경로는 GPS와 local의 경계를 개선했지만
local 준비에도 IMU·CAN을 모두 요구하며 typed velocity/dynamic validity는 false다.
새 목표와 이미 적용된 동작을 혼동하지 않는다. 기존 benchmark 시작 전 HSV 카메라 처리가
있으므로 기존 `lidar_pipeline` 시간을 전체 callback→출력 지연으로 쓰지 않는다.

## 2. 목표 구조와 설정 소유권

하나의 LiDAR 실행부가 계산 라이브러리를 호출한다. GPU 카메라 프로세스는 독립이다.
단계별 ROS 노드·점군 직렬화를 추가하지 않는다. 다음 폴더는 해당 구현 시 생성한다.

| 책임 | 소유 기능 | 재사용·경계 |
|---|---|---|
| geometry | 장착/수평화/역변환·시각·pose·속도 의미 | 현재 frame_time/projection/bounded_pose/IMU 선택 코어 |
| preprocessing | finite/ROI·ego crop·voxel·지면·점 계보 | 원본 threshold 보존, 새 HD ROI 제외 |
| detection | 군집·관측 평균·OBB·detection_index | 평균 중심과 OBB 중심을 함께 구분 |
| tracking | 연관·ID·miss/age·KF·운동 상태 | map projection과 독립, GPS 전환만으로 reset하지 않음 |
| fusion | 카메라 수신·투영·매칭·분류 유지 | L4는 전방 parity; 3뷰 확대는 L8 |
| filtering | 측정 품질·일반 중복·지도 관련성 | 기존 필터 parity와 지도 교체·개선은 분리 |
| ros | 입력·snapshot·worker·publisher·진단/시각화 | ROS 타입은 입출력에서만 변환 |

### 내부 처리 데이터와 스케줄

- `ScanInput`: 원본 header/source ns, callback ROS/steady 수신 시각, 입력 sequence,
  frame·point field 명세와 immutable cloud. 없는 ring/point-time을 합성하지 않는다.
- `ScanContext`: producer session·generation, scan sequence, 보정/config ID, 고정 parameter snapshot,
  IMU/속도 선택 근거, 실제 leveling 행렬, map pose와 reason. 실행 중 공유 설정을 덮어쓰지 않는다.
- `DetectionBatch`: 원본점/voxel/비지면 index 연결, cluster 지지점, 평균 중심·OBB 중심·8코너·축·크기,
  ground snapshot와 detection_index. track_id는 L3 전까지 -1이고 검출 번호로 가장하지 않는다.
- `TrackBatch`: 관측과 예측 구분·age/miss·ID·추정 속도와 기준점/유효성·분류 근거. debug용 구조와 분리한다.

처리 중 scan 하나와 대기 중 최신 scan 하나를 유지한다. callback은 짧은 snapshot 교체만 하고
피팅·점군 계산 동안 입력 잠금을 잡지 않는다. source stamp는 양수·단조 증가를 요구하고 중복/
역행은 진단 후 폐기한다. bag 재시작·clock regression은 명시적인 새 generation으로 시작한다.
생산자 재시작에는 새 UUID, stateful 설정 변경·reset에는 새 generation을 사용한다.
동일 scan의 local/map에는 같은 session·generation·sequence·track ID를 사용한다.

설정 소유권은 다음으로 고정한다. 아직 새 YAML runtime loader를 추가하지 않는다.

| 설정 | 정본과 적용 규칙 |
|---|---|
| 센서 장착·원본 영상 크기 | 현재 sensor JSON → LiDAR mount/camera calibration 참조. hash·값 검증 |
| 지도 원점·yaw 기준·제품 | asmc_hd_map 계약/제품 참조. LiDAR가 별도 지도 정본을 만들지 않음 |
| ROI/voxel/ground/cluster | asmc_lidar 처리 설정. L2 최초값은 기존 설정·코드 상수 그대로 |
| tracking/fusion/filter | L3–L5에서 기존 구현별 설정 대응, 의미 차이를 기록 후 단일 소유 |
| 토픽·queue·진단/시각화 | ros 실행 설정. 일반 launch에 모든 임계값을 노출하지 않음 |

실행 전 설정을 불변 snapshot으로 만든다. manifest에는 기존 launch 기본값과 설정 SHA를
보존하며, 미래 config ID는 loader가 소비하는 정확한 설정 내용과 참조 보정의 hash로 식별한다.
구체적인 YAML key·C++ 타입 선언은 해당 단위에서 구현한다.

## 3. 입력·좌표·유효성 계약

### 입력과 시간

| 입력 | 정본 | 선택·무효 처리 |
|---|---|---|
| LiDAR | `/velodyne_points`, PointCloud2 | 실제 완성 scan, source header 보존. empty/malformed와 정상 zero-object를 구분 |
| IMU | `/imu` | scan 이전의 유효 quaternion·각속도. 최대 source age120ms·선택 대기50ms |
| GPS | `/gps` | finite·lat/lon 영점·원점·map 범위·점프 검사. 무효 barrier 보존 |
| 차량 속도 | 검증된 `/Competition_topic` | x속도 m/s·부호 포함. 최대 age120ms·선택 대기50ms. 미검증/누락은 invalid |
| 카메라 | `/perception/camera/objects/observations`와 객체 상태 | 기존 8관측/350ms API·session/generation·보정 ID. LiDAR가 모드를 요청하지 않음 |

IMU/속도는 선택 마감까지 도착한 표본만 사용하며 source>scan인 표본은 사용하지 않는다.
대기는 source 허용 age를 늘리지 않는다. stale은 source 시간으로, 수신 watchdog은 steady로
따로 검사한다. 입력 generation마다 선택 source·receipt와 누락/교체 사유를 남긴다.
GPS bounded 변환은 기존 코어처럼 **scan 수신까지 도착한** GPS만 사용한다.

source timestamp provenance는 bridge 계약을 따른다. 현재 두 bag의 LiDAR는 bridge_receive이며
callback receipt나 bag record time과 같다고 가정하지 않는다. 원본 source metadata가 없는
자료에 광선 시각을 복원했다고 표시하지 않는다. ROS clock끼리 age, steady끼리 duration을
계산하며 서로 빼지 않는다. 전송 지연을 보정한 deskew는 이번 설계의 구현 기능이 아니다.

### 좌표와 pose 정책

공개 좌표는 후륜축 실제 차체 `base_link`(x전방·y좌측·z위), map은 HD맵의 UTM grid 축이다.
IMU true ENU yaw는 기존 판제와 같은 grid heading 변환을 **한 번만** 적용한다.

```text
raw sensor p → 같은 scan의 L로 수평화한 p_level
공개 local p_base = T_base_sensor × inverse(L) × p_level
공개 map p_map = T_map_base(scan) × p_base
L = R_mount^T × Ry(pitch) × Rx(roll) × R_mount
```

원점/8코너/방향축은 같은 변환을 사용한다. yaw만으로 보정된 3D 코너를 다시 만들지 않는다.
카메라 투영은 해당 scan의 inverse(L)와 장착/광학 변환을 합성하며 원본 픽셀 보정을 쓴다.
외부 카메라/family의 지면 plane 보정을 LiDAR raw 점에 다시 적용하지 않는다.

| 정책 | 고정값·동작 |
|---|---|
| 정상·음영 leveling | 같은 roll+pitch 정책, 실제 적용 L을 scan context에 저장 |
| GPS 사이 map 위치 | 기존 BoundedPoseHistory, 최대300ms 포함·초과 차단 |
| GPS 무효·중단 | 0/NaN/범위·점프 불가 시 즉시 barrier, 침묵300ms 초과 차단 |
| GPS 복구 | source 단조 증가·허용 간격 안의 정상 표본3회 연속 |
| bounded 전제 | 유효한 scan 자세·속도 필요. 없으면 map unavailable |
| strict API | 100ms bracketing/외삽 없음, 단위시험·비교 API로 유지. 자동 fallback 없음 |
| 장착 | LiDAR [0.80,0,1.45]m, GPS [0.35,0,1.25]m는 센서셋 출처 확인 |
| 높이 | 0.36m는 명목 base-to-ground 거리, 장착 z나 map altitude origin이 아님 |

GPS 안테나 lever는 GPS 관측시점의 자세로 한 번 제거한다. map XY 원점은 현재 계약의
[302595,4124145]m, altitude origin0m를 기준선으로 고정한다. GPS altitude−rotated lever−origin과
0.36m를 임의로 같게 만들지 않는다. 현재 높이 audit은 primary 자료 한정이며 절대 높이
정확도 보장이 아니다. 지도 밖이거나 보정 출처가 다르면 map 유효성을 만들지 않는다.

### 입력 조합별 결과

| 상태 | local | map·속도 |
|---|---|---|
| scan·IMU·속도·GPS 유효 | 매 처리 scan의 정상 결과 | 유효 pose에서 동일 관측 변환 |
| GPS 무효/침묵 | 검출·ID 생애주기 유지 | map 객체 없음, POSE_UNAVAILABLE. local ID reset 없음 |
| 속도 미수신/미검증 | 유효 IMU 기반 검출 유지 | bounded map 불가, ego 보상에 의존한 운동/절대속도 invalid |
| IMU 무효/stale | 보정된 정상 결과를 만들지 않음 | INPUT_INVALID와 사유, 마지막 자세로 몰래 진행하지 않음 |
| scan 없음 | 새 관측 발행 없음 | 진단 heartbeat와 소비자 timeout, 새 stamp 빈 배열 금지 |
| scan 정상·객체 없음 | OBSERVED_EMPTY | pose가 유효할 때만 정상 빈 map, 아니면 POSE_UNAVAILABLE |

속도가 없을 때도 geometry를 속도 gate로 막지 않는다. 상대 위치 미분은 ego의 병진/회전에
영향받으므로 보상 불가 상태를 실제 물체 속도로 relabel하지 않는다. 관측·예측은 계속 구분한다.
유효 twist를 구성할 때 원본 속도/각속도 stamp와 평가시각 scan을 별도 보존한다.
`v_abs_base = dp_base/dt + v_ego_base + omega_base × p_base`는 같은 기준점·시각의 유효 입력일 때만
적용한다. 이미 절대속도인 벡터에 ego 속도를 다시 더하지 않는다. 카메라 그룹은 운동 GT가 아니다.

### 공개 타입·출력과 호환 요구

| 의미 | 현재 공용 필드 | L1 결정 |
|---|---|---|
| scan/frame·상태 | LidarObjectArray.header/status/reason/stamp_provenance | 보존, empty/invalid/pose-unavailable/침묵 구분 |
| ID·실측/예측 | track_id/detection_index/observation_state/age/miss_count | 검출 번호와 track ID 구분, predicted를 새 실측으로 세지 않음 |
| 형상 | center/obb_corners/yaw/length/width/height | center는 OBB 중심, 원본 평균은 내부에 별도 보존 |
| 분류 | classification/camera_class | group과 원본 이름 보존, unknown도 유효 geometry 가능 |
| 속도 | relative/absolute_velocity(_valid), dynamic_valid/is_dynamic | 미확정은 false. UNKNOWN/PENDING를 static으로 치환 금지 |
| 회전·점 근거 | yaw_rate_valid/yaw_rate_rad_s/source_point_count | 현재 source_point_count는 voxel 후 군집 지지점 수이며 raw 점 개수가 아님. raw 계보는 내부 보존 |
| 새 관측 식별 | 현 타입에 없음 | L6 additive 계약에 session·generation·sequence·config/calibration ID 필수 |
| 네 단계 운동 상태 | 현 타입은 bool만 있어 부족 | UNKNOWN0/LOW_THREAT1/DYNAMIC_PENDING2/DYNAMIC3을 typed로 전달 |

L6 추가 관측 계약은 geometry·관측 ID·객체별 운동 상태를 **한 원자적 메시지**로 제공한다.
객체 상태는 같은 track_id에 연결하고 누락·중복 ID/길이 불일치는 정상 수신으로 처리하지 않는다.
진단 JSON의 exact-stamp 결합으로 정본 운동 상태를 복구하는 구조는 가져오지 않는다.
기존 LidarObject/Array·TrackedObject·DetectedObject의 정의/MD5는 유지한다. 새 타입의 이름·
구현·등록과 소비 어댑터는 L6 계획에서 다루며, 미구현 타입을 현재 registry에 추가하지 않는다.

정식 후보는 `/perception/lidar/objects_local`, `/perception/lidar/objects_map`, 진단은 기존
`/perception/lidar/diagnostics`를 기준으로 한다. 객체 관측용 additive wire의 실제 topic/type
연결은 L6에서 명시한다. 시험 출력만 `/migration/lidar/...`로 remap하고 입력은 정본을 구독한다.
Main opt-in과 동시 실행 시 같은 정식 토픽/TF를 중복 생산하지 않는다. map→base TF authority를
새로 차지하지 않는다. 같은 이름의 wire type을 묵시적으로 바꾸지 않는다.

local/map은 한 검출·추적의 두 표현이다. 실제 처리한 scan마다 새 결과를 제공하며 필요
구독자에만 직렬화한다. 별도10Hz timer로 결과를 복제하지 않는다. 최소10Hz 목표와 실제
scan/처리율을 구분한다. 과거 shadow bag 자체의 공급 부족을 처리 실패로 계산하지 않는다.
진단은1Hz heartbeat에서 입력 age·drop·queue·선택 source·상태를 집계하고 점군/이미지는 수요 기반이다.
상태·진단의 최종 ROS 타입과 READY/freshness 판정은 L6 소비 어댑터에서 구체화하되 객체의
scan 상태·세션 식별·4단계 운동 상태는 JSON 없이 전달한다.

## 4. 현재 소비자·필터와 전환 경계

| 현재 경로 | 실제 역할 | 이관 위치 |
|---|---|---|
| Main `/lidar/tracked_objects_map` | 최종 관련성 필터 전 track/fusion 관측 | 기준선 원본과 새 내부 TrackBatch 비교 |
| Main `/last_lidar` | road overlap/clip·known-static·structure/relevance 이후 | L5 parity, L7 전까지 운영 유지 |
| `/lidar/motion_debug` + wire bridge | exact stamp·ID로 4상태 결합, 없으면 validated 출력 보류 | L6 atomic typed 정본으로 대체 후 소비 검증 |
| planning_inputs validated_lidar | Main의 검증 결과를 기본 소비, 사적 필터 재실행 안 함 | 현재 소비 유지, 향후 얇은 변환 경계 |
| planning_inputs main_raw | 별도 분류/필터를 실행하는 선택 경로 | 중복 소유권 목록에 기록, 기본과 구분 |
| planner Behavior/Frenet | /planning/objects의 enum·OBB·속도·age와 차량 상태 소비 | 행동·경로 정책은 유지 |
| 음영 lane/bbox 어댑터 | 현재 카메라 기반 차선/bbox 소비 | LiDAR 전체 소비가 이미 연결됐다고 표시하지 않음 |

Main과 planning_inputs의 DrivableBoundary·DrivableFootprint·KnownStaticObject·StaticStructureExclusion
cpp는 include 경로를 제외하면 같다. MotionClassifier는 Main 쪽에 map invalidation·warmup 등
차이가 있어 단순 동일 파일로 통합하지 않는다. PlanningRelevance는 Main 추가 경계다.
L5까지 각 경로의 raw velocity·계획에 적용한 velocity, keep/drop·clip 사유를 분리해서 비교한다.

인지 소유는 측정/연관 품질·속도 이상·일반 중복·지도 관련성이다. TTC·추종 대상·정지·안전
재출발·회피·센서 장애 대응, 선행차 선택에 종속된 fragment 정책은 판제 책임이다. PR #101의
DynamicObjectPolicy·차량 도로 포함/충돌 판단은 이관하지 않는다. 차량 상태 전체 추정기를
인지 필터와 함께 가져오지도 않는다.

기존 road_mask와 최신 HD 제품은 같은 제품이 아니다. 좌표 원점이 같아도 geometry SHA를
구분하고 L5 최초 이관은 기존 road_mask로 재현한다. 최신 HD 교체·점군 ROI와 외부 진입
관측 영역·인접 차로 예외는 L8 비교로 둔다.

## 5. 기준선 자료와 의도적 차이

두 raw bag은 D 정본에서 전체 bytes를 다시 hash하고 인덱스·첫 header만 읽었다. bag 재생·
ROS publish·시뮬 실행은 하지 않았다. 둘 다 `/Ego_topic`은 있지만 `/Competition_topic`은 없다.
따라서 개발 parity에는 쓰되 현장 허용 속도 입력의 통과 증거로 쓸 수 없다.

| 자료 | 새로 확인한 범위 | 한계 |
|---|---|---|
| primary 48.9265s·649 scan | SHA/bytes, GPS245·IMU2384·전방979, 첫 stamp ns·PointXYZI | 독립 객체 GT 없음, 당시 Ego 입력 |
| shadow 24.7519s·208 scan | SHA/bytes, GPS124·IMU1223·전방494, 첫 stamp ns·PointXYZI | 평균 scan 공급 자체가10Hz 미만, live 공급 수락에 사용 불가 |
| B1a·PR #72 Git evidence | fixed raw cases·합성 archive/manifest hash | 과거 실행의 품질·failure 의미 유지 |
| GPS invalid/missing variant | 이전 manifest의 hash·재생성 recipe | `/evidence/fixtures`는 과거 임시 경로, 현재 파일 존재 확인 안 됨 |

Git evidence와 D B1a full-evidence는 출처/용도를 함께 기록한다. 없는 과거 alias를 복구하거나
새 C 사본을 만들지 않는다. L2 검출 비교는 원본 bag·Git raw case로 착수 가능하다. GPS variant는
L3 준비 때 원본 SHA·동일 window·recipe로 새 run에 생성하고 과거 variant와 hash를 대조한다.
고정 YOLO는 과거 결과이며 현 E8 새 추론이 아니다. 새 출력은 storage resolve/register-run을
거쳐 D에 쓰고 이전 run은 덮어쓰지 않는다. L1의 임시 코어 시험 바이너리만 `/tmp`에 생성했다.

### 동작 보존과 의도적 수정

- 보존: ROI 범위·ego crop, voxel0.1m, ground/cluster·OBB 수치, 평균/OBB 중심 구분,
  CA/KF·연관·ID·분류 확인/변경/유예, 기존 filter 결정과 raw/계획 속도 구분.
- 의도적 수정: source ns 보존, GPS0/stale map 차단, GPS와 local 분리, 같은 roll/pitch,
  실제 base 좌표·camera 투영, 속도 출처/부호/유효성·재시작 ID·JSON 의존 해소.
- 이식 제외: 구형 UDP7510(최대20), `/ctrl_cmd_0`, HSV 인지·중복 JPEG, 미사용 temporal copy,
  오래된 Planning/Control/IMM 주석 코드. 기존 실행에서의 삭제는 L7 이후 별도 변경.
- 차이 주의: 원본 setting의 RearOverhang0.700m와 현재 규정 요약0.790m는 다르다. L2 parity는
  원본값을 보존하고 차량 치수 정정은 별도 비교한다. legacy IMU 경로는 intensity 대입이 없어
  XYZ parity와 intensity 보존 요구를 분리한다. undefined payload의 byte parity는 요구하지 않는다.

## 6. L2에 넘길 구현 목록과 검사

| 기존 구현 | L2에서 추출할 것 | 제거할 의존성 / L3로 남길 것 |
|---|---|---|
| Parser::LiDARParser_Callback / main::CopyData | PointCloud2 검증·immutable scan snapshot·generation 선택 | 전역 raw 고정 배열·센서 잠금 장기 보유, CAN/GPS gate 제거는 L3 정책 연결과 구분 |
| RoiVoxel::Roi / Voxel | range/XYZ ROI·장착 기반 ego crop·0.1m voxel | Global.hpp의 VehicleSpec/LiDAR 전체 대신 작은 config·점 view |
| ImuRansac::CorrectingPoints | 명시 입력 행렬로 점 변환·intensity 보존 | frame_boundary 전역/VehicleSpec extern. L2는 원본 pitch 행렬로 parity, L3에서 roll+pitch 연결 |
| Ransac::NormalVectorFiltering / GroundPlaneSnapshot | 평면/지지점·출력·fallback state | ROS ConfigureOnce·함수 static을 config/state 소유 객체로 분리 |
| Clustering::AdaptiveClusterTolerance / Clustering | 기존 거리 적응 연결·cluster indices | Planning/VehicleSpec extern과 debug ROS 출력 |
| Tracking::Objectinfo_for_km와 obb_util | cluster 지지점·평균·OBB·8코너·detection_index | map 변환/KF·연관·ID 발급 제외, geometry 부분만 분리 |
| LidarProcess stage counters / PipelineTrace | cloud→cluster→OBB 비교 입력/중간 산출 | 원래 함수 전체 이식·추적 queue/marker publisher 혼입 금지 |

L2 검출 비교는 동일 source ns·입력/설정·보정으로 pairing하고 raw index/점 순서가 달라도
지지점 집합을 대응한다. RANSAC은 seed/난수·처리 순서를 고정해 재현성을 먼저 확인한다.
계산이 달라지면 좌표 수정과 혼합해 면제하지 않는다. 구체적 geometry 오차 gate와 반복 횟수는
L2 계획에서 입력·정밀도를 기준으로 고정한다. L1 코어 epsilon을 전체 detector 수락값으로 복사하지 않는다.

L1에서는 기존 frame_time(35개)·geometry/bounded pose·IMU 선택 실행파일을 새로 컴파일해
검사했다. projection·8코너, 300ms 포함/초과·invalid barrier·복구3회·GPS 도착 인과성,
source ns/clock·unknown velocity가 통과했다. 기존 aggregate initializer의 receipt_clock 생략
warning은 남아 있으며 실패가 아니다. 소스 수정 없이 manifest에 범위와 결과를 기록했다.

완료 시 문서/계약·MD5·manifest SHA·현재/기준선 source tree를 대조하고 L1만 닫는다.
기존 시험은 검출 GT·현장 Competition·주행 부하·planner cutover를 승인하지 않는다.
다음은 **L2만** 구체화한다. L3–L8 구현·push·PR·병합·추가 수집은 이번 단위에 포함하지 않는다.
