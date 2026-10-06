# LiDAR → 판단제어 공통 소비 계약

> 상태: **LiDAR 이전 구현용 계약, 카메라 객체 생산자 opt-in 구현**. `asmc_lidar`는 좌표·시간 라이브러리
> 골격이며 검출/추적/ROS 어댑터는 손재호의 후속 구현이다. 기존 주행은 원본
> `lidar+sensorfusion`을 사용한다. [인계 계획](../../packages/asmc_lidar/README.md)과
> [타입·목표 토픽](../../../config/contracts/lidar_candidate_topic_contract_v1.yaml)을 함께 본다.

## 공용 타입과 목표 연결

| 대상 | 의미 |
|---|---|
| `asmc_msgs/LidarObjectArray` | frame·관측 stamp·출력 상태와 객체 배열 |
| `asmc_msgs/LidarObject` | 지속 ID/검출 번호·관측/예측·중심/8코너·속도/동정적 유효성 |
| `asmc_msgs/CameraObjectBoxArray` | source header·영상 크기·입력 유효성·원본 픽셀 box 배열 |
| `asmc_msgs/CameraObjectBox` | box·그룹 class_name·원래 라벨 source_class_name·confidence/유효성. 원본 numeric class ID는 현 타입에 없음 |
| `/perception/lidar/objects_local` | 후륜축 `base_link`의 local 결과. 생산자 구현 예정 |
| `/perception/lidar/objects_map` | 같은 관측·ID를 관측시점 pose로 변환. 어댑터 구현 예정 |
| `/perception/camera/objects` | P2a는 camera_front만 출력. 카메라 생산자+LiDAR 호환 어댑터 opt-in 구현 |

기존 `TrackedObject*`, `DetectedObject*`의 MD5는 유지한다. 기존 `/last_lidar`,
`/lidar_nogps`, `/perception/objects`를 자동 remap하지 않는다. 구 타입으로 표현할 수
없는 예측·미확정 속도·출력 무효를 정상 빈 배열이나 유효 0속도로 채우지 않는다.
호환 어댑터는 프레임을 보류할 때 이유를 진단으로 알리고 상태형 원본은 보존한다.

새 타입도 다음 소비자 계약 검토가 필요하다. `classification`/`camera_class`가 비어
있으면 미확정이며, 지속 ID 미할당은 `track_id=-1`로 문서화해 사용한다.
원래 라벨은 source_class_name에 보존한다. 원본 COCO 숫자 ID나 다른 정보가 필요하면 additive/versioned
계약을 검토한다. 현 필드를 임의 의미로 재사용하거나 MD5를 묵시 변경하지 않는다.

## 좌표·시간·속도

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

모드 없는 처리의 [구조물 분리 인계 계약](../../packages/asmc_lidar/README.md#모드-없는-구조물-분리--손재호의-후속-구현-계약)을 따른다.
벽/가드레일/천장을 개별 객체 추적에서 제외해도 관측된 충돌 경계는 별도로 보존해야 한다.
`OBSERVED_EMPTY`인 객체 배열도 주변이 자유공간이라는 뜻은 아니다. 미확정 구조물/객체는
카메라 미검출이나 경로 밖이라는 이유로 삭제하지 않는다.

현재 `LidarObject*`에는 구조물 지지 범위·표면·품질 계약이 없다. 구조물 경계와 미확정
점유의 표현/상태/freshness를 판제와 별도 additive/versioned 계약으로 확정하기 전에는
객체 목록에서 구조물을 빼는 변경을 운영 승격하지 않는다. MarkerArray는 검토 수단일 뿐
판제용 경계 계약을 대신하지 않는다. 이번 문서 갱신은 새 메시지나 토픽을 추가하지 않는다.

## 판제 공동 확인 사항

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

좌표·시간 코어 시험은 [패키지 README](../../../src/asmc_lidar/README.md)에 있다.
실제 GPS 전환·차량/횡단 객체·카메라 동시 부하·세 판제 모듈의 운영 수락은 후속 작업이다.
