# LiDAR → 판단제어 공통 소비 계약

`asmc_lidar`가 객체·속도·운동 상태의 생산 정본이고, `planning_inputs`는 공용 관측을 판제
private 형식으로 변환한다. 구 Main/Msg/wire·JSON 결합과 판제의 중복 인지 필터는 B4에서 폐기한다.
기계 판독 정본은 [LiDAR 계약](../../config/contracts/lidar_candidate_topic_contract_v1.yaml),
직접 명령·유효성·검증은 [실행 안내](../../src/asmc_lidar/docs/planning-runtime.md)를 따른다.

## 현재 소비와 병행 비교

`/perception/lidar/planning_observations_map` (`asmc_msgs/LidarPlanningObservation`)
→ `planning_objects_adapter` → `/planning/objects` (`planning_inputs/DetectedObjectArray`)
→ 기존 Behavior/Frenet/MPC다. source 선택 옵션은 제거했고 시험 입력 topic remap은 유지한다.
비교 생산자는 `/migration/lidar/*`로 격리한다. 같은 정식 토픽에 두 생산자를 연결하지 않는다.

## 좌표·기하·운동

| 항목 | 의미 |
|---|---|
| local | 후륜축 `base_link`. 음영 전용이 아니며 GPS 무효에도 기하를 제공 |
| map | scan 시각의 유효 GPS/IMU pose·지도 원점으로 변환. 무효는 `POSE_UNAVAILABLE` |
| center | OBB 중심. 별도 `tracking.cluster_center`와 같다고 가정하지 않음 |
| OBB | 변환된8코너 정본. yaw는 장축 XY 표현이며 roll/pitch의8코너를 yaw 하나로 재구성하지 않음 |
| absolute velocity | 동일 scan의 평면 KF 상대속도 + ego 병진 보상 추정값, m/s. 유효 scan pose·유한 계산 근거가 필요하며 GT 정확도 수락과 구분 |
| relative velocity/yaw rate | 현재 validity=false인 미검증 항목. 유효한0 측정으로 소비하지 않음 |
| motion state | UNKNOWN0 / LOW_THREAT1 / DYNAMIC_PENDING2 / DYNAMIC3. 카메라 그룹과 별개 |

GPS 무효는 map 운동 이력을 비우고 local의 절대속도/운동 validity도 false로 만든다. local 객체를
global Frenet 입력에 이름만 remap하지 않는다. 터널 local/family 소비·모드와 장애 대응은 판제 소유다.

## 식별·시간·상태

`observation.objects.header`가 source stamp·좌표계의 정본이다. session/generation/scan_sequence와
config/calibration ID를 함께 검사하고 private 배열에도 복사한다. local/map을 두 번의 독립 관측으로 세지 않는다. generation은
명시적 처리 reset에서 바뀌며 scan마다 증가하는 sequence와 구분한다.

판제는 queue1·source age300ms·미래 허용50ms, 종료 session·이전 generation·중복/역행 stamp/sequence·
같은 세대 설정 변경·motion 배열 정렬을 검사한다. 정상 빈 결과는 새 빈 배열로 전달한다.
무효/stale/pose 부족은 발행 보류하고 기존 timeout/센서 장애 행동을 유지한다.

## 분류와 전달 필터

원본 이름·vehicle/dynamic_obstacle/static_obstacle은 같은 공용 판제 관측의 객체에 유지한다.
private 어댑터는 classification/camera_class를 보존하며 숫자 type은 vehicle/person/unknown을 유지한다. 판제 내부 STATIC_OBSTACLE=3 정의도
있지만 아직 이 그룹에서 매핑하지 않는다. dog/deer/bicycle/stroller와 정적8종은 UNKNOWN이 되어
숫자만으로 원본 종류·그룹을 복원할 수 없으며, 충돌 기하·적용 속도·운동 상태는 유지한다.
vehicle/dynamic_obstacle도 정지해 있을 수 있다. static_obstacle도 실제 정지의 보증이 아니며
그룹으로 motion_state·속도 validity를 덮어쓰지 않는다. PEDESTRIAN은 person 전용 정책 구분으로
보존할 수 있다. [현재 소비 대조와 후속 제안](../handoffs/perception-to-planning.md#yolo-그룹과-판제-타입의-소비-대조--2026-10-28)을 따른다.

HD맵 v7·안쪽10cm·점군3점/10% AND OBB10%·높이/상부·관측 확인3/5는 생산자에 적용한다.
최신 유효 카메라 vehicle/dynamic 매칭은 보호한다. 필터는 전달 결과만 줄이고 tracker/원본 OBB는 보존한다.
판제는 같은 classifier·도로/시설물 필터·clipping을 반복하지 않는다. TTC·추종·정지·재출발·회피·
위험도와 센서 장애 행동은 판제에 유지한다. [필터 소유권](../../src/asmc_lidar/docs/filter-migration.md)을 따른다.

## 변경과 미수락 항목

공용 메시지 필드·MD5는 유지한다. 2026-10-28 후속에서 private DetectedObject에 두 문자열,
DetectedObjectArray에 session/generation/scan_sequence/config/calibration ID를 추가했다.
private MD5는 변경되므로 전체 소비자 재빌드·재시작이 필요하다. 숫자 type·운동 정책은 유지한다.
전 미션·속도 정확도·터널 벽/천장·IMU 초기 무효 경계·3뷰 fusion 수락은 [공동 작업표](../../src/asmc_lidar/docs/migration-tasks.md)에 남긴다.
과거 wire/필터와 검증 근거는 [PR #111까지의 대조](../records/lidar/planning-consumer-through-pr111.md)와
[소스 보존 manifest](../../config/baselines/lidar_canonicalization_v1.json)에 보존한다.

## PR #114 통합 소비 규칙

판제 입력은 `LidarPlanningObservation` 하나다. 유효 기하의 속도만 미확정이면
`velocity_unavailable=true`·UNKNOWN으로 기하를 보존한다. vx/vy의0은 측정된 정지가 아니다.
새 session/generation의 raw track ID는 새 private ID에 연결한다. motion·속도 품질 계산은 인지 소유다.
일반 `tracks_local/map`의 기존 `LidarTrackingObservation`은 별도 분석 계약이며 자동 대체하지 않는다.
별도 companion wrapper/source 선택은 제거했고 판제는 현재 실행 정본을 따른다.
