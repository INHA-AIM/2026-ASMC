# 판제가 사용할 LiDAR 실행 기준선

## 적용 범위와 담당

2026-10-26, main `ab8db094`와 문서 기준 `a046b8a9`에서 **안승현이 구 Main의 누락 기능을
보존 이식하고 공용 관측의 판제 직접 소비를 연결**했다. 재호의 개선 완료를 기다리지 않는다.
속도 모델·군집·OBB·피팅·fusion 임계값을 새로 튜닝한 작업이 아니다.

`asmc_lidar`와 `planning_inputs`는 Main/Msg 없이 빌드·실행할 수 있다. 이후 판제는 이 입력
경계로 개발하고, 승현과 재호는 동일한 공용 계약 뒤의 인지 구현을 개선할 수 있다.
**소스상 실행 준비와 실제 판제 주행 인수는 구분한다.** 기존 운영 프로세스를 종료하거나
구 패키지를 삭제하지 않았다. 전체 판제 launch의 음영 BBox 입력도 공용 카메라 결과를 직접
소비하도록 바꾸어 숨은 Main/Msg 의존성을 제거했다. 통합 판제의 기존 기본 선택은 인수 전까지 `validated_lidar`다.

2026-10-26 현재 PC 제한 주행에서 네트워크 제어 선택과 UDP 조향 부호의 불일치를 확인했다.
제어 경계 수정 후보를 사용한 새 LiDAR 입력 시험은 약107m 직선·곡선에서 최대 횡오차9.7cm였다.
제어 UDP bridge만 true override로 재시작한 실제 적용 대조도 약107m·최대 횡오차12.4cm,
158개 조회 표본의 계획 유효성을 유지했다. 당시 객체는 정상 빈 결과였으며 장애물·미션 수락은 아니다.
별도 시험1회의 입력 freshness 정지는 원인 미확정으로 남긴다. LiDAR 필터나 TTC를 완화해
이 증상을 우회하지 않는다. [제어 설정·대조 근거·남은 검증](../../../docs/operations/planning-control.md#morai-제어-프로토콜과-정지-순서)을 따른다.

같은 날 사용자가 **대회용 MolitComp03의 기본 bridge 설정(`invert_steer=false`)으로 새 LiDAR
판제 주행이 동작함을 직접 확인**했다. 실행 조회에서도 해당 release·local UDP와
`source_policy=asmc_lidar`, `/perception/lidar/tracks_map` 소비를 확인했다. 이는 사용자 직접
연결 확인이며 전 미션·장애물 속도 정확도·LAN2LAN 수락은 아니다. 공용 조향 기본값은 false를
유지하고, 별도로 확인한26R1 조합은 true override를 사용한다. 통합 판제의 기본 source 전환과
구 패키지 삭제는 공동 인수 단계에서 진행한다.

## 가져온 기능과 의도적 차이

| 구 Main 기능 | 이번 연결 |
|---|---|
| CA Kalman 추정과 MotionClassifier | 기존 `tracking/motion_display` 계산 재사용. 동일 scan의 판제용 보정 벡터와4단계 운동 상태를 공용 출력에 연결 |
| 최종 속도 | `f32_PlannerVxKPH/VyKPH ÷ 3.6`을 scan의 실제 수평화 역변환·장착 회전·map 회전으로 한 번 변환. CAN을 다시 더하지 않음 |
| 추적 정보 | ID·age·관측 상태, OBB 중심과 별도의 `cluster_center` 유지. 후자는 구 Core/fusion 기준점이며 KF 중심과 같다고 보장하지 않음 |
| 알려진 고정 시설물 | **새 경로에서 제거**. 과거 기록 중심2.5m 근접만으로 unknown 실제 객체를 삭제할 수 있음. 현재 HD맵·점군 근거로 판단 |
| 설정된 구조물 구역 | **새 경로에서 제거**. 고정 map 사각형·map Z 조건은 semantic 근거가 아님. 현재 HD맵·base 기준 상부 근거로 판단 |
| 지면 파편·관측 확인/해제 | `filtering/observation_quality.*`로 이식. 지면 근거·최소 높이0.10m·확인3회/해제5회·기존 근접 예외를 유지 |
| road mask·OBB clipping | 현재 v7·10cm·점군3점/10% AND OBB10%·상부 필터를 유지. 구 mask와 OBB clipping을 다시 넣지 않음 |
| 기존 wire·JSON 결합 | 새 경로에서 사용하지 않음. 판제가 공용 추적 관측을 직접 소비 |

처음 보존 이식한 세 계산 중 좌표 catalog와 사각 구역 필터는 재검토 후 제거했다. 코드·설정·
불필요한 catalog 사본도 새 패키지에서 제거했으며 구 Main/복구 경로는 유지한다. 관측 품질의
기존 함수·임계값은 유지한다. 정본은 [현재 YAML](../config/lidar_pipeline.yaml), 출처와 제거 이유는
[provenance](../config/delivery/provenance.yaml), 상세는 [필터 소유권](filter-migration.md)다.

현재 최신 YOLO vehicle/dynamic 보호는 이 포팅 필터보다 우선한다. 이는 PR #108의8종
보호를 유지하기 위한 의도적 차이다. 지도 기반 제외 뒤 관측 품질을 평가하며
최종 ID 집합을 local/map·배열·두 관측 타입에 동일하게 적용한다. 원본 점군·OBB·tracker를
바꾸거나, 제거된 객체를 tracking 삭제로 연결하지 않는다.

관측 확인 코드에는 기존 근접 보존 등 인지/판제 책임이 섞여 있다. **이번에는 동작 보존을
우선해 이식했다.** 후속 리팩토링에서 해당 부분을 판제와 분리하며, 이번에 TTC·추종·회피·
정지·재출발·장애 대응을 수정한 것은 아니다. [기능별 소유권](filter-migration.md)을 따른다.

## 공용 메시지와 속도의 의미

기존 `LidarObject`, `LidarObjectArray`, `LidarObservation`과 구 wire의 MD5는 유지했다.
4단계 운동 상태와 별도 기준점은 새 공용 타입으로 원자적으로 전달한다.

- `asmc_msgs/LidarTrackingObservation`: `observation`은 기존 `LidarObservation`,
  `tracking[]`은 그 안의 객체와 같은 순서·같은 ID의 `LidarTrackingState`다.
- `LidarTrackingState`: `track_id`, `cluster_center`, `motion_state`.
  상태는 UNKNOWN0 / LOW_THREAT1 / DYNAMIC_PENDING2 / DYNAMIC3이다.
- header·출력 상태·기하·속도·session/generation/scan_sequence·설정/보정 ID는 기존 관측이
  정본이다. 구 이름 alias나 구 Msg를 다시 발행하는 wrapper/중계 노드는 추가하지 않았다.
- 동일 generation의 config/calibration 변경, 과거 generation/session·중복/역행 stamp/sequence,
  정렬되지 않은 tracking ID와 잘못된 기하를 거부한다. 소비자는 최근 종료 session64개만 보관하고
  source age350ms·미래 허용50ms도 함께 검사한다.

`absolute_velocity_mps`는 **구 MotionClassifier가 판제에 적용하던 추정값**이다. LOW_THREAT의
0속도 정책과 UNKNOWN 예열 중 수치도 보존한다. `absolute_velocity_valid`는 유효한 scan pose와
해당 track의 유한한 계산 근거를 뜻하며, 독립 GT로 정확도를 승인했다는 뜻이 아니다.
**UNKNOWN의 수치0을 정지 확인으로 해석하지 않는다.** `dynamic_valid=false`와 UNKNOWN을
유지하고, 카메라의 `dynamic_obstacle` 그룹을 실제 운동 판정으로 바꾸지 않는다.

다음은 기존 계산의 한계로 남긴다.

- KF 상대속도+CAN의 기존 근사, 회전 레버암·가시 면 변화·후진 부호·정확도 개선은 후속이다.
- `relative_velocity_valid`는 false다. 움직이는 수평화 좌표의 미분을 검증된 상대속도라고 내보내지 않는다.
- GPS 무효에서는 local 기하를 유지하되 map 기반 운동 이력을 비운다. local의 절대속도/운동
  validity도 false다. 터널 속도·운동 상태 개선을 이번 이식으로 해결했다고 표시하지 않는다.
- yaw rate는 기존 producer도 측정하지 않았다. flag는 false이고 판제의 기존0 표현을 유지한다.

음영 입력의 구 `camera_bbox_detector.launch`는 제거했다. 기존 frame bridge에서 전방·cutoff0.7·
정확한 image stamp 대응을 유지하고 공용 원본 subpixel bbox를 private frame에 전달한다.
구 Main의 정수 wire 양자화만 제거됐으며 차선 판정·IMU 투영·차선변경/제어 계산은 바꾸지 않았다.

## 실제 연결과 유효성

```text
bridge·카메라 (각각 별도 실행)
 → asmc_lidar/lidar.launch
 → /perception/lidar/tracks_map : asmc_msgs/LidarTrackingObservation
 → planning_inputs/objects.launch source_policy:=asmc_lidar
 → /planning/objects : planning_inputs/DetectedObjectArray
 → 기존 Behavior/Frenet
```

기본 `lidar.launch`와 웹 `view.launch`는 `/perception/lidar`를 사용한다. 같은 launch에
`namespace:=/migration/lidar`를 주면 병행 비교할 수 있다. 센서·카메라 입력은 바뀌지 않는다.
단일 처리 worker와 기존 ROS 노드를 유지하며, 단계마다 노드를 추가하지 않는다.
`tracks_local/map`은 구독자가 있을 때만 직렬화한다. 기존 geometry 배열/관측도 같은 계산에서 제공한다.

새 판제 입력은 다음을 보장한다.

- 공용 관측을 queue1로 직접 수신한다. 구 wire bridge·motion JSON·두 번째 classifier·road/시설물
  필터·OBB clipping을 실행하지 않는다. 그룹/원본 이름을 기존 numeric enum에 명시적으로 대응한다.
- source stamp·ID·원본 OBB·적용 속도·4단계 상태를 보존한다. 새 producer/reset 경계에서는 이전
  관측을 배제하며 이전 private 분류 이력을 재사용하지 않는다.
- 정상 `OBSERVED_EMPTY`는 새 빈 배열로 전달한다. `INPUT_INVALID`·`POSE_UNAVAILABLE`·stale·
  표현할 수 없는 속도는 빈 객체로 바꾸지 않고 발행을 보류한다. 판제의 기존 timeout/장애 정책은 유지한다.
- global 입력은 `map`만 받는다. GPS 없는 local 객체는 `/perception/lidar/tracks_local`에서
  상태/속도 validity와 함께 직접 소비할 수 있지만, global Frenet에 이름만 remap하지 않는다.
- `/planning_objects_adapter/diagnostics`에 수신/발행/거부 수와 최근 사유를1Hz로 제공한다.
  기록은 bounded이며 진단을 다시 판제 정본으로 join하지 않는다.

시설물/구역 파일을 로딩하지 않는다. `delivery_policy=observation_quality_only`로 남은 정책을
진단한다. 도로 안 잡음의 높이/지면 근거와 시간적 확인은 HD맵 안/밖 판정으로 대체되지 않는다.
기존 최소 두께0.10m보다 얇더라도 지면 위 clearance 근거가 충분하면 보존하고, 확인3회·해제5회를
유지한다. 예측만으로 재발행하지 않는다. 기존 근접 보존·확정 분류 보존도 유지한다. GPS 무효에서는
map 기준 품질 이력을 비우고 local을 보존한다. HD맵 누락/무효는 지도 제외를 중단한다.

## 실행·시험·복구

Noetic 컨테이너의 outer workspace만 빌드하면 된다. 아래 시험은 합성 입력을 사용하고
자체 ROS master를 정리한다. 운영 master·bridge·카메라·Main·GUI를 종료하지 않는다.

```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
catkin_make -j2 -l2 -DASMC_LIDAR_PIPELINE_REQUIRED=ON
source devel/setup.bash
catkin_make tests -j2 -l2
python3 src/planning_inputs/test/common_lidar_ros_test.py
```

실행은 별도 터미널에서 각각 시작한다. 두 번째 명령은 차량 제어를 포함하므로 주행 인수
상황에서 사용한다. 자동 전체 실행 회귀는 **별도 ROS master와 합성 입력**에서만 수행했다.
운영 master의 차량 제어·판제 소비는 전환하지 않았다.

```bash
roslaunch asmc_lidar lidar.launch debug:=true
roslaunch asmc_planner autonomy_no_bridges_v2.launch lidar_source_policy:=asmc_lidar
```

GUI는 필요할 때 별도로 `roslaunch asmc_lidar view.launch` 후 `http://127.0.0.1:8091/`를 연다.
새 launch를 켜기 전 기존 시험 처리·웹이 남아 있지 않은지 확인하고 한 세트만 실행한다.
웹은 포팅 정책 제외를 별도 색상·숨김 항목으로 표시한다. 표시 체크는 실제 발행 정책을 바꾸지 않는다.

병행 비교는 `lidar.launch namespace:=/migration/lidar`와 판제의
`lidar_input_topic:=/migration/lidar/tracks_map`을 함께 지정한다. Main의 opt-in 공용 출력과
새 정식 출력을 같은 토픽에 동시에 붙이지 않는다. 입력 덮어쓰기·중복 출력은 canonical 허용
상태에서도 시작을 거부한다. TF는 기본 OFF다.

복구는 새 판제·LiDAR 실행을 종료하고 구 Main 및
`autonomy_no_bridges_v2.launch lidar_source_policy:=validated_lidar`를 사용한다.
구 빌드/source 순서는 [기존 Main 안내](../../lidar+sensorfusion/RUN_SENSOR_FUSION.md)를 따른다.
소비 인수 전 구 패키지와 이 복구 경로를 삭제하지 않는다.

## 검증 범위와 다음 작업

로컬 검증은 독립 build/devel에서 Main/Msg 없이 수행한다. 기존 MD5·직렬화, 포팅한 원래
필터 회귀, typed vector 회전, 배열/상태 정렬과 tracker 불변성, 공개 launch를 사용한
실제 LiDAR→판제 입력의 GPS/IMU 중단·복구·reset·재시작을 검사한다.
합성 입력 시험은 이동 객체의 속도 정확도·전체 코스 recall·공동 자율주행 수락이 아니다.
Hosted CI는 push 후 별도로 확인한다.

2026-10-26 최초 연결 완료 근거(시설물/구조물 제거 전, 기능 `dd7a229c`, 카메라 입력 `c508b165`, 표시 `a32a3a97`):

| 검사 | 결과와 범위 |
|---|---|
| 독립 build/devel CTest | 43/43 통과. Main/Msg 없는 새 구성, 기존 필터·수신·runtime·ROS 회귀 포함 |
| 필수 LiDAR gate | 필요한 CTest 등록/실행 36/36 확인. 생략을 통과로 계산하지 않음 |
| 공용 추적 관측→private 판제 ROS | 16개 확인. 실제 공개 launch, GPS0/복구·IMU 중단/복구·reset·생산자 재시작, 동일 stamp·ID·기하·속도/상태, 단일 발행자 |
| 전체 판제 실행 경계 | 별도 master에서 11개 확인. Behavior/Frenet/MPC 입력 구독, 기존 wire/validated 복구, 시설물 제외·정상 빈 결과 |
| 음영 카메라 연결 ROS | 6개 suite·42개 검사 통과. 전방 선택·confidence·빈/무효·잘린 bbox·차체 mask·IMU 투영·차선변경·종료 |
| 마지막 typed/표시 회귀 | typed output 13개, 웹 순수 JS68개, 실제 Chrome21개 확인. 무효 운동을 정지로 표시하지 않음 |
| 문서·계약 | 현재 링크/동결5개/history, launch27개·직접 명령108개, 관련 unittest27개·Main 카메라 소유권4개 통과 |
| 기본 컨테이너 | outer workspace 전체 재빌드, 새 메시지/실행파일 확인. 운영 bridge/master와 기존 시험 LiDAR·웹은 재시작하지 않음 |

같은 날 후속 정리에서는 고정 시설물/사각 구조물 필터 두 개를 제거하고 **LiDAR/판제 입력
CTest39/39·필수 gate34/34·직접 ROS 연결16개**를 다시 통과했다. 관측 품질은28개 회귀로
검사했으며 과거 catalog/blacklist 위치에 있는 unknown도 위치만으로 삭제하지 않음을 포함한다.
구 Main 복구 분기와 판제 본체·제어 알고리즘은 이 정리에서 변경하지 않았다. CI cache/concurrency는
YAML·내부 bash 구문을 확인했으며 hosted 실행 시간 개선은 아직 측정하지 않았다.

직접 연결 시험의 scan은 합성 자료다. 실제 전 미션 주행 인수·3뷰 fusion·GPS 없는 터널 속도 및
벽/천장 품질 수락은 이 결과에 포함하지 않는다. 기존 trial이 `/migration/lidar`로 실행 중이라면
같은 프로세스의 namespace·메시지가 재빌드만으로 바뀌지 않으므로 전환 시 한 세트만 재실행한다.

다음은 **판제 담당과 이 입력으로 실제 미션을 확인 → 구/신규 차이 인수 → 구 패키지 삭제·
정본화 마무리**다. 이후 승현은 카메라/지도 연결·필요한 공통 구조를, 재호는 추적/속도·3뷰
fusion·GPS 없는 터널 벽/천장 개선을 진행한다. 재호의 개선을 이 보존 이식의 착수 조건으로
두지 않는다. [터널 보조 근거·흰선 밖 객체 보존 원칙](cutover-foundation.md#손재호-gps-없는-터널-local-인식)은 유지한다.
