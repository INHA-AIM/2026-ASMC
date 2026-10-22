# LiDAR → 판단제어 공통 소비 계약

**2026-10-16 카메라 연결:** 독립 Main YOLO는 제거하고 `asmc_camera/camera.launch`만 추론을 소유한다.
[현재 실행·3뷰 인계](../handoffs/camera-to-lidar.md)를 따른다.

**카메라 분류:** Main의 `camera_class_profile` 기본은 `mission16_v1`입니다.
tracked `classification`은3그룹, `yolo_class`는원본 이름입니다. 기존 numeric planning enum은 유지되며
unknown 타입도 기하·속도를 가진 장애물입니다. [모델과 소비 계약](../operations/camera-deployment.md)을
함께 확인합니다. 좌표·시간·운동의 기존 기준은 그대로 적용합니다.


> 상태: **기존 Main 운영 + 새 `asmc_lidar` 이관 목표 계약**. 좌표·시간 코어와 카메라
> 수신·버퍼·투영은 구현됐고 Main의 공용 local/map 출력은 opt-in이다. 새 LiDAR 처리 shadow 노드는 PR #103에서 구현됐다(운영 미연결). 안승현이 [L0–L8](../../src/asmc_lidar/docs/migration-tasks.md)을 진행하며,
> 손재호 개발은 병행한다. 판제는 L7 소비 전환 전까지 기존 `lidar+sensorfusion`을 사용한다.
> [기계 판독 상태](../../config/contracts/lidar_candidate_topic_contract_v1.yaml)와 현재/목표를 함께 확인한다.

## 현재 소비와 이관 중 격리

현재 기본은 Main의 `/last_lidar`와 같은 stamp의 `/lidar/motion_debug`를
`planning_inputs`가 연결하는 `validated_lidar` 경로다. 선택 `main_raw`는 별도 필터·운동
분류를 실행한다. 기본 경로에서 양쪽 필터가 모두 중복 실행된다고 설명하지 않는다.
새 정본 전달의 JSON 진단 의존 해소는 L6이며 아직 구현하지 않았다.

정식 후보 local/map 경로는 아래 표를 유지하되, 새 실행의 동시 비교 출력만
`/migration/lidar/objects_local`, `/migration/lidar/objects_map`으로 namespace/remap한다.
센서·카메라 입력은 정본 경로를 구독한다. 새 상태·진단 타입, session/ID·소비자 캐시 초기화,
pose 선택과 시간 한계는 L1에서 확정한다. 현재 없는 타입을 구현된 계약으로 등록하지 않는다.
L7 전까지 `/last_lidar`·`/lidar_nogps`·판제 입력은 유지하고 시험 생산자를 연결하지 않는다.

## 공용 타입과 목표 연결

| 대상 | 의미 |
|---|---|
| `asmc_msgs/LidarObjectArray` | frame·관측 stamp·출력 상태와 객체 배열 |
| `asmc_msgs/LidarObject` | 지속 ID/검출 번호·관측/예측·중심/8코너·속도/동정적 유효성 |
| `asmc_msgs/CameraObjectBoxArray` | source header·영상 크기·입력 유효성·원본 픽셀 box 배열 |
| `asmc_msgs/CameraObjectBox` | box·그룹 class_name·원래 라벨 source_class_name·confidence/유효성. 원본 numeric class ID는 현 타입에 없음 |
| `/perception/lidar/objects_local` | 후륜축 `base_link`의 local 결과. 현재 Main opt-in; 새 패키지 생산자는 L6 |
| `/perception/lidar/objects_map` | 같은 관측·ID를 관측시점 pose로 변환. 현재 Main opt-in; 새 패키지는 L3/L6 |
| `/perception/camera/objects` | 3뷰 원본 픽셀 검출. 현재 Main 어댑터는 전방만 소비 |
| `/perception/camera/objects/observations` | session/generation·보정/모델/설정 ID 포함. asmc_lidar 수신·버퍼·투영 지원 제공, shadow 노드는 `camera_front`만 매칭·fusion; 3뷰 객체 매칭/fusion은 손재호의 후속 구현 |

기존 `TrackedObject*`, `DetectedObject*`의 MD5는 유지한다. 기존 `/last_lidar`,
`/lidar_nogps`, `/perception/objects`를 자동 remap하지 않는다. 구 타입으로 표현할 수
없는 예측·미확정 속도·출력 무효를 정상 빈 배열이나 유효 0속도로 채우지 않는다.
호환 어댑터는 프레임을 보류할 때 이유를 진단으로 알리고 상태형 원본은 보존한다.

새 타입도 다음 소비자 계약 검토가 필요하다. `classification`/`camera_class`가 비어
있으면 미확정이며, 지속 ID 미할당은 `track_id=-1`로 문서화해 사용한다.
카메라 입력의 원래 라벨은 `source_class_name`, LiDAR 출력에서는 `camera_class`에 보존한다. 원본 COCO 숫자 ID나 다른 정보가 필요하면 additive/versioned
계약을 검토한다. 현 필드를 임의 의미로 재사용하거나 MD5를 묵시 변경하지 않는다.

## 좌표·시간·속도

아래 strict 보간은 기존 목표 계약이다. Main opt-in의 bounded 정책과 구분하며 새 실행의
정책·시간 한계는 L1에서 확정해 이 절에 반영하고 L3에서 검증한다. 목표는 GPS/미션과
무관한 local 처리·발행과 scan 시각의 유효 pose에서만 생성하는 map 객체다. map 무효는
정상 빈 객체로 위장하지 않고 상태로 구분한다. 지금 실행 기본값을 바꾼 것은 아니다.

- `base_link`는 차량에 고정된 후륜축 기준이다. 센서 원점의 중력 수평화 좌표와 다르다.
  실제 적용한 수평화의 역변환 → sensor-to-base 장착 변환 → 유효할 때 map pose 순서다.
  중심·8코너·방향축에 같은 변환을 쓰고 yaw로 3D 코너를 다시 만들지 않는다.
- stamp는 관측의 bridge 수신 기준을 보존한다. 실제 광선 발사 시각이나 처리 완료시각으로
  바꾸지 않는다. 관측·수신·처리 timestamp는 같은 clock임을 검증하고 CPU duration은
  steady/monotonic clock으로 따로 잰다.
- map pose는 처리 시점까지 도착한 유효 양쪽 표본에서 최대 100ms 간격으로 보간한다.
  외삽하지 않는다. GPS 안테나와 후륜축 offset, map 원점·고도 기준을 검증한다.
  2D VehicleState만으로 완전한 3D pose를 만들지 않는다.
- 이미 절대속도인 벡터는 축만 회전한다. base 좌표 위치의 미분은
  `v_abs_base = v_rel + v_ego + omega × p_base`로 변환한다. 같은 시각의 twist가
  없거나 legacy 의미가 불명확하면 속도는 미확정이다. CAN 속도를 중복 가산하지 않는다.
- 객체 추적 상태는 GPS/map projection과 분리한다. GPS 정상→무효→복구가 local ID나
  검출 설정을 바꾸지 않는다. map 출력이 없더라도 local 결과는 유지한다.

## 유효성·주행 관련성

`OBSERVED_EMPTY`는 그 scan을 정상 처리하고 객체가 없을 때만 쓴다. `INPUT_INVALID`,
`POSE_UNAVAILABLE`와 구별하며 predicted 객체를 새 관측으로 표시하지 않는다.
센서가 끊겼으면 새 stamp의 빈 관측을 만들지 말고 진단과 소비자의 timeout으로 알린다.

카메라 정상 빈 결과, 미수신, stale, 미지원 class를 구분한다. empty/missing이라는
이유로 LiDAR 객체를 지우지 않는다. 카메라 drivable의 장애물 구멍도 삭제 근거가 아니다.
HD맵·카메라 문맥의 frame/시간·유효 범위가 맞지 않으면 관련성은 unknown이다.

객체 footprint와 차로/주행영역/경로의 관계는 별도 속성으로 계산한다. 인접·진입·횡단
가능 객체를 남기고 경계에 걸친 객체의 차로를 하나로 강제하지 않는다. 현재 경로 밖이라는
이유만으로 객체를 제거하지 않는다. 구조물은 충돌 경계이며 자동으로 무시 가능한 객체가 아니다.

### 구조물 경계와 객체 목록의 소비 경계

L8 후속 후보인 모드 없는 처리의 [구조물 분리 인계 계약](../handoffs/lidar-migration.md#모드-없는-구조물-분리--손재호의-후속-구현-계약)을 따른다.
벽/가드레일/천장을 개별 객체 추적에서 제외해도 관측된 충돌 경계는 별도로 보존해야 한다.
`OBSERVED_EMPTY`인 객체 배열도 주변이 자유공간이라는 뜻은 아니다. 미확정 구조물/객체는
카메라 미검출이나 경로 밖이라는 이유로 삭제하지 않는다.

현재 `LidarObject*`에는 구조물 지지 범위·표면·품질 계약이 없다. 새 구조물 표현 구현을 L2–L7 이관의 선행조건으로 두지 않는다. 구조물 경계와 미확정
점유의 표현/상태/freshness를 판제와 별도 additive/versioned 계약으로 확정하기 전에는
객체 목록에서 구조물을 빼는 변경을 운영 승격하지 않는다. MarkerArray는 검토 수단일 뿐
판제용 경계 계약을 대신하지 않는다. 이번 문서 갱신은 새 메시지나 토픽을 추가하지 않는다.

## 판제 공동 확인 사항

L5에서는 측정 품질·속도 이상·일반 중복·지도 관련성 필터를 인지 소유로 통합한다.
TTC·추종·정지/재출발·회피·센서 장애 대응은 판제에 유지한다. PR #101의 동적 장애물
행동 판단·안전 재출발도 판제 정책이다. 새 경로 확인 전 기존 필터·소비 코드를 삭제하지 않는다.

| 파트 | 연결 전에 확인할 사항 |
|---|---|
| Behavior | 실제 GT/typed 입력 선택, 분류·차로 투영·대상 선택, source stamp freshness, invalid/미수신 정책 |
| Frenet | 실제 변환 코드의 map/OBB·속도·ID 요구, 변환 보류와 STOP 처리 |
| MPC | 객체의 직접 소비보다 차량 상태와 trajectory의 frame·시간·유효성 일치 |
| 음영 어댑터 | local 객체와 차량 상태·경로의 좌표 정합, GPS 복귀 때 전환 정책 |

과거 감사에서 main `1ce434d`와 서준 `8b3f0b1`은 `DetectedObjectArray`의 MD5가 달랐다.
현재 이관 대상 SHA에서 `rosmsg md5`와 정의를 다시 비교해야 한다. 옛 결과로 최신 소비
호환을 승인하지 않는다. 판제 ID 재연결·fragment 억제·대상 소실 유예는 원인 개선과
기동 안전을 구분하고 담당자 회귀 이후에만 제거를 논의한다.

좌표·시간 코어 시험은 [패키지 README](../../src/asmc_lidar/README.md)에 있다.
실제 GPS 전환·차량/횡단 객체·카메라 동시 부하·세 판제 모듈의 운영 수락은 후속 작업이다.

## Main 공통 경계의 opt-in 후보

`enable_common_geometry:=true`에서 기존 `asmc_msgs/LidarObjectArray`로
`/perception/lidar/objects_local` (`base_link`)과 `objects_map` (`map`)을 제공한다.
[구현·시간 정책·직접 launch·수락 상태](../records/lidar/lidar-camera-frame-boundaries.md)를
확인한다. 생산 호스트는 아직 nested Main이며 골격의 전체 기능 이전은 완료되지 않았다.
관측 상태·속도/dynamic validity를 소비자가 확인해야 하며 자동 remap·판제 cutover는 없다.
strict 100ms bracketing API와 opt-in 300ms 제한 GPS 보정 API를 구분한다.
