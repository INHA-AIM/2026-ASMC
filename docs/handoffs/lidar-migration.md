# LiDAR 통합 이관 계획과 소비 인계

**2026-10-26 실행 기반 구현:** 재호의 개선을 기다리지 않고 안승현이 구 Main의 보정 속도·
4단계 운동 상태·시설물/구조물/관측 확인 정책을 보존 이식하고 판제의 공용 관측 직접 소비를
연결했다. [현재 실행·차이·검증·복구 정본](../../src/asmc_lidar/docs/planning-runtime.md)을 먼저 읽는다.
`lidar.launch`/`view.launch` 기본은 `/perception/lidar`, 판제는 `lidar_source_policy:=asmc_lidar`로 선택한다.
통합 판제의 기존 기본 입력·실행 중인 운영 노드와 구 패키지는 유지한다. 아래 10월25일 차이표의
속도/운동 상태 미연결·직접 소비 미구현은 이번에 보완했으며, 실제 공동 주행 인수·구 패키지 삭제는 남았다.

**역할·PR #108 기준(2026-10-25, 현재 구현은 위 링크):** PR #108은 main `ab8db094`에 병합됐다. 안승현은 **HD맵 필터·카메라 연결 지원**,
손재호는 **추적·속도·운동 상태·fusion 및 GPS 없는 터널 local 인식**을 담당한다.
좌표·시간·공용 메시지와 판제 전환은 공동 확인한다.

`asmc_lidar`에는 독립 검출·추적·전방 fusion, 공용 local/map 배열·관측, 단계별 웹/RViz와
실제 HD맵·상부 전달 필터가 구현됐다. 기본은 **v7·filter·안쪽10cm·점군3점/10% AND OBB10%**다.
지도/pose 무효는 지도 제외를 중단하고, 실제 최신 YOLO vehicle/dynamic 매칭을 보호한다.
원본 tracker는 유지하며 발행 배열만 제외한다([현재 설정](../../src/asmc_lidar/docs/cutover-foundation.md#현재-적용-설정)).

**통합 판제의 기본은 인수 전까지 구 `lidar+sensorfusion` Main이다.** 새 launch의 기본 출력은
`/perception/lidar/*`이며 비교할 때만 `namespace:=/migration/lidar`로 격리한다. Main의 opt-in 공용 출력은 이 독립 생산자와 별도 경로다.
PR #108 당시에는 보정 속도·운동이 미연결이었다. 10월26일 안승현의 보존 이식으로 공용
추적 관측과 직접 입력 선택을 구현했다. 기존 wire/JSON 경로는 기존 운영/복구에만 사용한다.

B1/B2는 현재 실행 기준선에서 구현했으며, 다음은 실제 인수·정본화 마무리다. [기능 차이·전환 기반 B0–B4](../../src/asmc_lidar/docs/cutover-foundation.md)다.
필요한 정보 생산 → 공용 관측 직접 소비 → 비교·전환 → 구 패키지 삭제를 먼저 준비한다.
광범위한 리팩토링·3뷰 fusion은 그 뒤로 두고 재호의 터널 개선은 병행한다.
[카메라 입력 API](../../src/asmc_lidar/docs/camera-input.md)와 기존 L0/L1·PR #103–#108 근거는 재사용한다.

shadow의 비교 기준선은 Tracker yaw 수정이 반영된 Main `8f93c22e`(`fix/lidar-tracking-frame-yaw`, main 미병합)이고,
그 수정은 shadow에 이미 들어 있다. 기존 Main은 동결하며 shadow 이식은 Main Tracker 수정의 선행 병합을 요구하지 않는다.
저장소 Main과의 OBB yaw·corner 차이는 의도된 것이다([의도적 차이](../../src/asmc_lidar/docs/pipeline.md#기준선과-저장소-main과의-의도적-차이)).
[좌표·GPS 경계 구현과 수락 보류](../records/lidar/lidar-camera-frame-boundaries.md)를 참고한다.
[external 6개 좌표 구현과 현재 보정 검토](../records/camera/camera-lidar-coordinate-reference-review.md)에는
행렬 수치 대조와 센서 시각·지면·pose 제공자 차이를 기록했다.

작업 상태·선행 조건·완료 근거의 정본은 [단계별 작업표](../../src/asmc_lidar/docs/migration-tasks.md)다.
L1 구조·입출력·시각·pose 정책은 [이관 설계](../../src/asmc_lidar/docs/migration-design.md),
고정 소스/설정·입력 SHA는 [L1 manifest](../../config/baselines/lidar_migration_l1_v1.yaml)를 따른다.
현재 판제는 nested `lidar+sensorfusion`의 Main과 `/last_lidar`·tracked/motion 경로를 계속 쓴다.
새 경로의 검증과 판제 인수 전에는 이 실행 경로·기본 설정·출력을 변경하지 않는다.

## 이번 PR 이후 공동 작업 순서

구 기능 보존 이식·직접 입력을 구현했으므로 [현재 실행 기준선](../../src/asmc_lidar/docs/planning-runtime.md)을 검토한 뒤 실제 판제 인수·구 패키지 폐기를 진행한다. 기존 계획 [B0–B4](../../src/asmc_lidar/docs/cutover-foundation.md#전환-기반-작업-단위)를 진행한다.
이는 단순 토픽 remap이 아니라 공용 속도/상태 전달·직접 소비·인수·구 패키지 폐기 작업이다.
**local/map·IMU·pose 공통화**는 [C0–C4 분석](../architecture/perception-coordinate-refactoring.md)에
보존하되 이 기반 이후로 둔다. GPS 없는 터널 객체·벽/천장 개선은 손재호가 맡고 승현이
카메라/family·시간·보정을 지원한다. 바깥 흰선 밖 점군을 일괄 삭제하지 않는다.

### 재호의 PR #104 인수 시작

이 절은 PR #104 당시 구조 인수 절차다. 현재 작업 시작점은 위의 B0–B4이며, 아래 SHA와 검증은 당시 기록이다. **PR #104는 main `6927fe9d`로 병합**됐다.
병합 전 검토 대상은 `refactor/lidar-structure-contracts`, 당시 실행 코드 기준은 `56b1caaa`다.
main `a63727c5`의 재호 PR #105(원본 `2648f117`)까지 통합했다. PR #103의 검출·추적·전방
fusion과 PR #105의 웹 모니터·단계 observer·진단용 운동 이력을 다시 이식할 필요가 없다.
PR 병합은 구조·계약의 공동 개발 기준선 확정이며 판제 입력 전환이나 음영구간 수락이 아니다.

| 먼저 확인할 것 | 정본 |
|---|---|
| 새 include·실행 이름·내부 필드 | [이름 이전표](../../src/asmc_lidar/docs/name-mapping.md). 옛 이름 wrapper/별칭 없음 |
| 디렉토리 책임·reset·실제 검증과 제한 | [구조·관측 계약](../../src/asmc_lidar/docs/structure-contract-refactor.md) |
| 빌드·시험·실행 | [패키지 README](../../src/asmc_lidar/README.md#빌드시험) |
| 재호·승현의 다음 작업·완료 기준 | [공동 개발 인수와 우선 작업](../../src/asmc_lidar/docs/migration-tasks.md#공동-개발-인수와-우선-작업) |

1. **PR #104 병합 후** 본인의 미커밋 작업을 먼저 보존하고 자신의 작업 브랜치에서 최신 main을
   반영한다. 공개한 브랜치의 이력을 강제로 덮어쓰지 않는다. 개인 작업 브랜치에서의 예:

   ```bash
   git fetch origin
   git merge origin/main
   ```

2. 옛 `pipeline/types.hpp`의 평면 필드 접근은 `detection`, `tracking`, `camera`로 나눠 수정한다.
   파일을 옛 위치로 복사하거나 구형 메시지를 이름만 치환하지 않는다. `MotionObject`와
   `MotionClassifier`는 `tracking/motion_display.hpp`, ROS debug 발행은 `ros/debug_publisher.hpp`에 있다.
3. 같은 Noetic 환경에서 **공용 메시지와 새 LiDAR를 함께 재빌드**한 뒤 이 문서의 실행 구분을
   확인한다. `/root/ws` 컨테이너 기준이며 nested Main devel은 새 경로 실행에 필요하지 않다.

   ```bash
   cd /root/ws
   source /opt/ros/noetic/setup.bash
   catkin_make --pkg asmc_msgs asmc_lidar -DCATKIN_DEVEL_PREFIX=/root/ws/devel \
     -DASMC_LIDAR_PIPELINE=ON -DASMC_LIDAR_PIPELINE_REQUIRED=ON -j2 -l2
   source /root/ws/devel/setup.bash
   ```

4. 기존 프로세스는 실행 중인 코드가 자동 교체되지 않는다. 인수 시험 전 기존 launch·RViz·Marker
   실행 여부를 확인하고, 본인이 관리하는 시험 프로세스만 정리한 뒤 한 세트로 실행한다.
   bridge/카메라/ROS master를 중복 시작하지 않는다. 반복 대조는 독립 master를 유지해 launch 종료로
   남은 노드의 ROS 등록이 끊기지 않도록 한다. 현재 팀 주행용 Main은 계속 유지한다.
5. 재호는 추적·속도·운동 상태·matching/fusion, 승현은 HD맵 필터·카메라 연결 지원을 각각
   작은 작업 단위로 진행한다. 공용 메시지·좌표·시각·출력 정책은 공동 확인하고 판제 전환은 L7에서 한다.

### 처리·RViz·웹 실행 구분

| 목적 | 명령과 동작 |
|---|---|
| 기본 처리 | `roslaunch asmc_lidar lidar.launch` — 처리 노드1개, camera fusion 기본 ON |
| 카메라 없이 대조 | `roslaunch asmc_lidar lidar.launch fusion:=false` — 카메라 구독·보정 조회 없음 |
| 처리와 RViz | `roslaunch asmc_lidar lidar.launch rviz:=true` — 처리1개 + Marker 변환기 + RViz |
| 웹용 단계 샘플 제공 | `roslaunch asmc_lidar lidar.launch debug:=true` — 구독 수요가 있을 때만 bounded 샘플 발행 |
| 웹 화면만 추가 | 별도 터미널 `roslaunch asmc_lidar view.launch` → `http://127.0.0.1:8091/` |

처리 명령들은 서로 대안이다. 이미 같은 namespace의 처리가 있으면 두 번째 처리를 시작하지 않는다.
`rviz` 기본 OFF, `debug` 기본 OFF, `view.launch`는 **재호 PR #105의 읽기 전용 웹 화면만** 시작한다.
`rviz:=true`는 웹을 켜지 않고, `view.launch`는 처리·카메라·RViz를 시작하지 않는다.
이미 열린 RViz를 유지할 때는 `rviz:=false`로 두고 [수동 Marker 연결](../../src/asmc_lidar/README.md#처리-결과를-rviz에서-보기)을 따른다.

다른 출력 namespace는 처리와 웹에 같은 `namespace:=...`를 준다. RViz 옵션은 해당 결과를 자동 구독한다.
센서와 카메라 입력은 기존 절대 경로를 유지한다. YOLO 추론은 `asmc_camera`만 담당하며,
`fusion:=true`는 이미 발행된 관측을 LiDAR가 사용하는 옵션이다. 현재 matching은 전방만 구현됐다.

### 정본 관측과 진단 표시의 경계

- 출력은 `/migration/lidar/objects_local`, `objects_map`, `observations_local`, `observations_map`,
  `diagnostics`, `trace`다. 배열은 `asmc_msgs/LidarObjectArray`, 원자적 관측은
  [`asmc_msgs/LidarObservation`](../../src/asmc_msgs/msg/LidarObservation.msg)이다.
- source stamp·frame은 **`objects.header`**를 따른다. 같은 scan의 local/map은
  `(producer_session_id, generation, scan_sequence)`로 연결한다. `header.seq`를 연결 키로 쓰거나
  local/map을 독립적인 두 관측으로 세지 않는다.
- 생산자 재시작은 새 session, 명시적 `~reset`은 같은 session의 generation 증가다.
  소비 캐시를 해당 경계에서 비우고 이전 결과를 배제한다. 단순 GPS 무효·카메라 상태 변화·입력 공백은
  generation 변경이 아니다. `config_id`·`calibration_id`는 실제 실행 설정/보정의 식별자다.
- 공개 local은 후륜축 `base_link`, 내부 `obb_corners_level`은 수평화 센서 좌표다.
  공개8코너를 사용할 때는 이미 변환됐으므로 RViz·웹·소비자가 IMU나 장착 변환을 다시 적용하지 않는다.
  단계 debug의 수평화 코너는 별도 입력이므로 해당 scan context로 표시 좌표에 한 번 변환한다.
- `classification`은 소비 그룹, `camera_class`는 원본 라벨이다. `vehicle`, `dynamic_obstacle`,
  `static_obstacle`과 실제 움직임은 별개다. `unknown`을 static으로 간주하지 않는다.
- 공개 상대속도 validity는 false이며 절대 적용 추정값·동정적 validity는 유효 근거에 따른다.
  4단계 상태는 공용 추적 관측으로 전달한다. 웹의 색상 자체가 유효성의 근거는 아니다.
  웹 화면 숨김은 표시 기능이다. PR #108의 기본 `filter`는 별도로 실제 발행 배열에서 HD맵·상부
  제외를 수행한다. 원본 debug track이 보인다고 판제 전달 배열에 남은 것으로 해석하지 않는다.
- 정상 빈 결과·입력 무효·map pose 무효·미수신/stale을 구분한다. 현재 local은 IMU와 속도가 필요하고,
  map은 유효 pose가 있어야 한다. GPS 0·중단 때 마지막 위치를 무기한 재사용하지 않는다.
  상세는 [카메라 입력 API](../../src/asmc_lidar/docs/camera-input.md)와
  [LiDAR 소비 계약](../contracts/lidar-planning-consumer-v1.md)을 따른다.

### 검증된 범위와 아직 남은 문제

코드 기준 `56b1caaa`의 GitHub **CI·Planning integration build 모두 통과**를 확인했다.
PR #105 통합 CTest31/31, 웹 core29개·JS61개와 문서/계약 검사는
[검증 기록](../../src/asmc_lidar/docs/structure-contract-refactor.md)에 연결했다.
3,428회 동등성은 PR #103 기준 구조 변경의 근거이며 PR #105 추가 후 최신 처리 비용/메모리 실측은 아니다.
새 실행 이름과 시각화가 동작한다는 사실을 다음 품질 문제의 해결로 확대하지 않는다.

- **음영구간:** 사용자가 TunnelFilter ON 경로에서 박스가 많이 튀는 현상을 확인했다.
  같은 입력의 OFF/ON·군집·OBB·ID·진단 운동 상태를 분리해 검토해야 하며 필터 기본 OFF를 유지한다.
- **IMU 무효 전환:** 기존 PoseHistory가 최신 무효 stamp 하나만 저장해 첫120ms에 이전 정상 표본과
  대기 scan의 순서에 따라 정상 결과가 나올 수 있다. 지속 무효 검사 통과와 초기 전환의
  즉시 무효화 보장은 다르다. 운영 정책은 이번 PR에서 바꾸지 않았고 재호의 L3 보완 대상으로 남긴다.
- **속도·fusion:** signed 속도/후진·절대속도·운동 상태의 공개 validity, 실제 카메라 bag 품질과3뷰
  matching, 공동 주행·판제 전환은 미완료다. HD맵·상부 필터는 PR #108에서 새 발행 배열에 적용됐고,
  전체 코스 객체 recall·위치 오차 수락과 구 필터 전체 대체는 별개다. 팀 주행은 계속 기존 Main을 쓴다.

## 최신 HD맵 사용 여부 — 2026-10-25

- 새 `asmc_lidar`는 HD맵 계약의 원점 `[302595.0, 4124145.0, 0.0]`을 사용한다.
  A1.1에서 정본으로부터 만든 SHA 고정 bundle을 audit에 연결했고 A1.3에서 v7로 대조했다.
  런타임 지도 토픽을 매 scan 구독하는 방식이 아니라 정본에서 만든 bundle을 한 번 로드한다.
  PR #108은 이 지도에 따른 실제 전달 제외를 구현했다. 구 Main은 이 bundle을 사용하지 않는다.
- 운영 Main의 `main.launch`는 `planning_inputs/config/validated_0781402/road_mask.png`와
  같은 이름의 YAML을 사용한다. PNG SHA는 `b27521b882486f1f46d450f455f2292044b87dae731136d0c00e5786c0605f2c`다.
- 최신 `asmc_hd_map` 주행가능영역 정본은 `kcity2025_reviewed_high_speed_adjacent_drivable_v7`이며
  `config/kcity2025_compact_drivable_map.json`과 파생 `kcity2025_compact_drivable_grid.npz`다.
  기존 mask와 `map_id`·원점이 같아도 같은 geometry가 아니다. 기존 PNG의 흰 영역은
  2,678,451픽셀이다. PNG는 흰색이 주행가능,
  OccupancyGrid는0이 주행가능이므로 값도 그대로 치환하지 않는다.
- A1의 v6 대조는 당시 근거로 보존한다. 최신 v7과 판제 v5 사본은 구분한다. 최신 v7에는
  합류 실선·톨게이트 섬 제외도 반영됐다. 실제 전달 필터와 별개로 동적 장애물 진입·로터리 접근·
  위치 오차·전체 코스 객체 보존 및 판제 소비 전환의 수락은 남아 있다.

### A1 ROI 검토 조건 — 구간 전제와 5Hz GPS

**최신 결정:** 기본 ROI는 유지하고 HD맵 점군 ROI는 보류한다. 우선 **OBB·추적·camera fusion
이후 판제 전달 판정**을 검토하며, 원본 객체·track과 OBB 형상을 변경하지 않는다.
[A1 결과·실행 안내](../../src/asmc_lidar/docs/hd-map-filter-analysis.md)에 기존 필터의 실제 호출
경로·재현 한계·갤러리·다음 구현 경계를 모았다.

2026-10-24 사용자 제공 운영 조건: **정적 장애물·동적 장애물·로터리 구간을 제외하면
HD맵 주행가능 영역 밖에서 객체가 진입하지 않는다.** 공식 규정으로 확인된 사실과 구분해
기록한다. 일반 구간의 지도 기반 점군 ROI는 검토 후보이며 적용이 확정된 정책이 아니다.
세 예외 구간은 접근·횡단·회피 공간을 별도로 검토하고, Ego가 예외 링크에 진입하기 전에도
그 공간이 센서에 보이면 필요한 관측을 보존한다.

- GPS 5Hz의 명목 갱신 간격은 200ms다. 마지막 위치를 그대로 쓰면 그 사이 이동량이
  지도 정합 오차가 될 수 있다. 예를 들어 10m/s에서 200ms 동안의 이동은 2m이며,
  이는 실제 측정된 위치 오차나 도로 횡방향 오차를 뜻하지 않는다.
- IMU 자세는 회전을 보정하지만 병진 위치를 직접 제공하지 않는다. GPS 사이의 scan 위치는
  검증된 차량 속도·방향·시간 정합을 이용한 제한 예측 또는 지연을 허용한 보간으로 평가한다.
  IMU 가속도의 단순 적분이나 기존 pose의 age 통과만으로 정확한 ROI 정합을 보장하지 않는다.
- 기존 최대 300ms pose 정책은 입력 사용 한계다. 지도 경계에서 객체를 제거해도 된다는
  정확도 인증으로 사용하지 않는다. source/수신 시각, scan age, 회전·가감속·GPS 복귀와
  지도/장착 오차를 포함해 경계 객체의 보존을 확인한다.
- 위치·지도·구간 판정이 불확실하거나 pose가 무효이면 지도 기반 제거를 적용하지 않고
  local 객체를 보존한다. 직전 GPS나 고정 지도 ROI를 음영구간에서 계속 재사용하지 않는다.
  경계 여유는 검증 결과로 정하며 점군을 잘라 OBB·추적을 악화시키는지도 확인한다.
- A1은 기존 필터 감사와 오프라인 유지/제외 후보/판단 불가 기록까지 진행한다.
  동결 trace 8개·합성 36개를 대조했고 실제 50,635객체 관측의 제외 후보는0이었다.
  위치 오차·외부 접근 영역 검증이 없어 실제 제거 승격은 보류한다. 실제 자료는 주로
  A448/A126/A128이며 세 예외 미션의 실제 주행 수락으로 세지 않는다.
  다음 단위도 판정만 연결해 비교하며 실제 제외는 OFF로 유지한다. 점군 ROI는 후속 검토다.

| 구분 | 현재 구현 | 이번 이관에서 할 일 |
|---|---|---|
| 센서 처리·검출·추적·전방 fusion | Main 운영 + PR #103 shadow 구현 | shadow를 재사용하고 L1 요구 차이·미수락만 보완 |
| `asmc_lidar` | 독립 검출·추적·전방 fusion·shadow 실행/재생 | L2 인수 → L3 속도/유효성 → L4 실제 bag → L5 필터 |
| 공용 local/map 출력 | Main opt-in + 이관 배열·원자적 관측 식별 | 세부 운동 상태 보완과 정식 소비 전환은 L6–L7 |
| 카메라 추론 | `asmc_camera` 단일 소유, Main 구형 YOLO 제거 완료 | 모델을 재이식하지 않고 관측·퓨전만 연결 |
| 남은 구형 기능 | Main의 HSV 영상 처리·UDP 객체 송신·제어 발행 등 | 새 경로에 가져오지 않음. 기존 삭제는 소비 전환 이후 |

현재 연결은 [카메라→LiDAR 인계](camera-to-lidar.md), 실제 수신 API는
[카메라 입력 API](../../src/asmc_lidar/docs/camera-input.md), 좌표·출력 의미는
[소비 계약](../contracts/lidar-planning-consumer-v1.md)을 따른다.
[좌표·GPS 경계](../records/lidar/lidar-camera-frame-boundaries.md)와
[external 좌표 검토](../records/camera/camera-lidar-coordinate-reference-review.md)는 기존 근거로 보존한다.

현재 CompetitionInfoPublisher는9088→9099→`/Ego_topic`으로 확인됐다.
[실측·정정](../records/bridge/2026-10-23-competition-channel-audit.md)에 따라 이름만으로 금지 입력을
판정하지 않는다. 속도 없어도 local 형상을 유지하는 L1 목표, signed 속도와 후진 검증은 남았다.

## 현재 경로와 이관 목표

```text
현재: bridge → nested Main → /last_lidar + tracked/motion → planning_inputs → 판제
목표: bridge → asmc_lidar 계산 라이브러리 + ROS 실행부
                 ← asmc_camera 관측 (LiDAR에서 수신·투영·fusion)
             → 공용 local/map 객체와 상태 → 판제 소비 어댑터
```

목표 내부 책임은 `geometry / preprocessing / detection / tracking / fusion / filtering / ros`다.
계산 라이브러리와 ROS 입출력을 분리하고 단계마다 ROS 노드를 추가하지 않는다.
PR #104에서 PR #103 계산을 책임별 디렉토리로 재배치했다. 이전 경로는 [대응표](../../src/asmc_lidar/docs/name-mapping.md)를 따른다. 구형 UDP `127.0.0.1:7510`, `/ctrl_cmd_0` 발행,
HSV 차선·정지선·신호등 처리는 새 LiDAR 경로에 이식하지 않는다. 카메라 보정·투영은 유지한다.

## 시험 namespace와 전환

| 대상 | 정식 후보 | 이관 동시 비교 |
|---|---|---|
| local 객체 | `/perception/lidar/objects_local` | `/migration/lidar/objects_local` |
| map 객체 | `/perception/lidar/objects_map` | `/migration/lidar/objects_map` |
| 원자적 관측 | `/perception/lidar/observations_local,map` (`asmc_msgs/LidarObservation`) | `/migration/lidar/observations_local,map` |
| 판제 추적 관측 | `/perception/lidar/tracks_local,map` (`asmc_msgs/LidarTrackingObservation`) | `/migration/lidar/tracks_local,map` |
| 진단 | 판제 정본 입력으로 사용하지 않음 | `/migration/lidar/diagnostics`, `trace` |

이번 구조 정리에서 기존 `/asmc_lidar/shadow/*`를 위 `/migration/lidar/*`로 변경했다. 여기서 예전 shadow는 음영구간이 아니라 병행 검증 실행을 뜻했다. L2–L5의 시험 실행기는 기존
판제에 연결하지 않는다. 출력 namespace/remap만 전환하며 센서·카메라 입력은 정본 경로를
계속 구독한다. Main opt-in도 같은 정식 출력 이름을 쓰므로 동시 비교 시 충돌을 검사한다.

L7에서 기존 발행자·호환 어댑터를 정지하고 새 경로를 정식 이름으로 실행한 뒤 판제 입력을
명시적으로 전환한다. 문자열을 코드에서 다시 고치지 않는다. frame·stamp·속도/무효 의미와
재시작 ID·소비자 캐시 초기화를 확인하고 같은 토픽의 생산자는 하나만 유지한다.
복구는 새 생산자·어댑터를 정지한 뒤 고정한 기존 Main과 판제 입력 설정을 함께 복원한다.

## A1.1 HD맵 판정 연결

아래 두 단락은 A1.1 착수 당시 범위다. 현재 발행 필터는 뒤의 A1.9 안내와 분석 정본을 따른다.

안승현의 A1 조사 다음 단위는 `filtering/` C++ 코어를 기존 runtime/replay에 연결하는
**판정·진단 전용** 작업이다. 원본 객체·tracker·fusion 결과와 판제 입력은 유지한다.
기본 YAML은 OFF, 내부 audit 설정은 SHA로 고정한 지도 파생 bundle을 읽어 유지/제외 후보/
판단 불가와 사유를 기록한다. 실제 객체 제외 모드는 없다. 지도 오류가 LiDAR 객체 발행을
막지 않으며 GPS 무효에서는 local을 보존한다. 현재 위치 오차·예외 공간 외부 접근 검증은
없어 runtime이 해당 근거를 true로 만들지 않는다.

새 trace 선택 필드와 최근100scan/1Hz diagnostics만 추가한다. 재호의 추적·속도·운동 상태·
fusion 보완과 판제 정책은 변경하지 않는다. 계산·공급 비교와 갤러리는
[A1/A1.1 분석 정본](../../src/asmc_lidar/docs/hd-map-filter-analysis.md#a11-실행-연결과-진단)에 모은다.
다음 단위에서 위치 오차와 미션 접근 범위를 검증한 뒤 실제 제외·소비 전환을 별도 PR로
검토한다. audit 연결을 기존 판제 필터 제거 완료로 해석하지 않는다.

후속 검수 미리보기는 OBB 겹침과 객체 전체 cluster support를 비교한다. 카메라의 최신 양성
`dynamic_obstacle`·`vehicle` 연결은 모든 구간에서 보호하며, 머리 위 후보는 최저 점·OBB 하단을 함께 본다.
보호는 person/dog/deer/bicycle/stroller와 car/bus/truck의 실제 최신 YOLO matching과 시간/epoch 유효성에 한한다.
`yolo_points_and_obb_subcell_v5`에서는 LiDAR motion/속도·미션10m 영역만으로 unknown을 보호하지 않는다.
유지 분류와 최신 양성 매칭을 분리하고, 이후 unknown·miss·세대 변경이면 보호 근거를 비운다.
일반 객체는 도로 합집합 바깥 경계10cm 안쪽에서3점 이상 AND10% 이상을 요구하고,
OBB도 같은 영역과 면적10% 이상 겹쳐야 한다. 기본 표시는 점군 AND OBB로 한쪽 미달을 다른 쪽이 보완하지 않는다.
카메라 보호·지도/pose 불명 보존을 우선하며, 현재 **새 패키지의 발행 배열에는 제외를 적용**한다.
현재 matching은 전방이며3뷰 matching 확대는 재호의 후속 책임이다. 체크박스는 화면만 바꾸고
원본 tracker·기존 Main·판제 소비 경로는 유지한다. [정책과 한계](../../src/asmc_lidar/docs/hd-map-filter-analysis.md#a19-10cm-경계와-실제-발행-필터--2026-10-25),
[디버거 사용](../../src/asmc_lidar/docs/web-monitor.md#hd맵-필터-디버깅)을 참고한다.

2026-10-25 사용자 검수로 A029·A175·A406을 추가한 v7 정본과 물리 차선5개를 반영했다.
기존102개 도로 면/ID와121개 차선 형상은 보존한다. 새 시험 프로필·bundle SHA와 보완선의
추정 근거는 [A1.3](../../src/asmc_lidar/docs/hd-map-filter-analysis.md#a13-인접-도로와-차선-검수-반영--2026-10-25)에 둔다.
웹의 도로 분할 테두리는 제거했다. 새 패키지의 전달 제외는 A1.9에서 연결했고 판제 입력 전환은 미완료다.

현재 기본은 HD맵 filter·IMU 수평 높이 ROI다. 높이 ROI는 군집 입력을 바꾸고,
HD맵·상부 전달 필터는 tracker를 보존하면서 발행 배열에서 제외한다.
현재 규칙·복구·검증 범위는 [A1.9](../../src/asmc_lidar/docs/hd-map-filter-analysis.md#a19-10cm-경계와-실제-발행-필터--2026-10-25)를 따른다.

## 판제 소비 전환 시 정리할 중복 처리

다음 PR의 기능별 원본·목표 소유권·편입/분리 조건은
[판제 필터 편입표](../../src/asmc_lidar/docs/filter-migration.md)에 따로 정리했다. 시설물·구조물·
관측 확인은 보존 이식했고 새 직접 입력에서는 중복 계산하지 않는다. 실제 주행 인수와 구 코드 삭제는 남았다.

이 절은 판제 입력 전환 PR의 정리 목록이다. A1/A1.1은 당시 audit-only였고, A1.9부터는
새 발행 배열에 실제 HD맵·상부 제외가 적용됐다. 이는 구 판제 입력 전환 완료와 다르다. 기존 Main의 `/last_lidar` 소비·복구가 남아 있는 동안
legacy 필터를 먼저 삭제하지 않는다.

| 대상 | 전환 PR에서 확인할 일 |
|---|---|
| `planning_inputs` 입력 어댑터 | `LidarTrackingObservation`의 공용 타입·session/generation·scan 식별·frame/stamp·상태를 처리. `/last_lidar` 타입에 토픽명만 바꾸어 연결하지 않음 |
| road/known-static/structure | 새 인지 경로에서 **실제로 검증·활성화한 기능만** 판제 측 중복 호출·설정에서 제거. audit-only이면 기존 필터를 대체했다고 간주하지 않음 |
| OBB clipping·재구성 | 새 경로에서는 원본 중심·8코너를 보존. 예전 Main/planning_inputs의 clipping을 자동 재적용하지 않음 |
| 운동 상태·속도 | 재호가 검증한 공개 validity를 소비. 진단 색·JSON·UNKNOWN을 측정된 static/0속도로 바꾸지 않음. private classifier의 대체·유지는 별도 검증 후 결정 |
| wire/debug 결합 | 현재 `/lidar/motion_debug`와 exact stamp/ID를 맞추는 gate를 공용 관측 계약으로 대체 검증한 뒤 정리. 단순 삭제로 준비 상태·유효성 검사를 잃지 않음 |
| 유지할 판제 책임 | TTC·추종 대상·정지·회피·재출발·센서 장애 대응, local/map 선택과 상태 확인. 인지 필터 이관을 이유로 삭제하지 않음 |

새 경로/기존 경로별 비교, 정상 빈 결과와 입력/pose 무효, GPS 중단·복귀, 생산자 재시작,
단일 발행자와 rollback을 판제 담당과 확인한다. 변경 파일·기능·영향을 먼저 공유하고
판제 소비 전환은 별도 PR로 검토한다. 기존 필터 전체를 끄는 설정 변경과 토픽 remap만으로
전환을 완료하지 않는다. A1의 분석 정본은
[필터 소유권·오프라인 검증](../../src/asmc_lidar/docs/hd-map-filter-analysis.md)이다.

## 이관 전 단계의 공통 기준

- local 처리·발행은 GPS·미션 구간에 종속시키지 않는다. 실제 후륜축 `base_link`와 내부
  수평화 좌표를 구분하고 중심·8코너·방향·속도 변환을 일관되게 적용한다.
- roll/pitch 보정은 구간별 다른 정책 대신 같은 좌표·시각 규칙으로 검증한다. 지면 근거나
  IMU 무효를 숨기지 않는다. 자세 보정은 추적기의 ego motion 보상이나 deskew를 대신하지 않는다.
- map 객체는 scan 시각의 유효 pose에서만 만든다. GPS 0·무효·stale에서 이전 위치를
  무기한 재사용하지 않는다. 새 실행은 L1에서 고정한 bounded300ms·복구3회를 사용한다. strict100ms 보간 코어는 별도 시험 API로 유지한다.
- 정상 빈 결과·센서 무효·pose 무효·미수신을 구분한다. 미확정 속도·운동 상태를 유효한
  0/static으로 치환하지 않는다. 현재 `LidarObjectArray`의 상태와 속도 validity를 재사용 검토한다.
- 측정 품질·속도 이상·일반 중복·지도 관련성은 인지 책임이다. TTC·추종·정지·재출발·회피와
  센서 장애 대응은 판제에 유지한다. 최신 main의 동적 장애물 행동 정책도 이관 대상이 아니다.
- Main/planning_inputs의 중복 필터를 단일 구현으로 정리하되 기존 임계값 보존과 정책 변경을
  분리한다. HD맵 점군 ROI·새 구조물 억제·3뷰 매칭 확대는 L8에서 우선순위를 다시 정한다.
- 기존 코드 삭제는 새 경로 검증·소비 전환 뒤 B4에서 진행한다. 이번 B0 문서 단위는
  launch·메시지·운영값을 변경하지 않는다. L0의 과거 문서 범위는 당시 기록으로 보존한다.


## 읽는 순서와 검증 상태

1. 이 README와 [현재 카메라 인계](camera-to-lidar.md)에서 실행/계약·현재 구현과 후속 과제를 확인한다.
2. [작업표](../../src/asmc_lidar/docs/migration-tasks.md)에서 L1 결과와 PR #103 차이를 확인하고 L2 인수부터 한 단위씩 계획한다. B1a 과제 대응표와 과거 증거를 함께 확인한다.
3. [PR #72 연속 회귀](../records/lidar/lidar-continuous-regression.md)에서 시간·지면·추적 검증과 미수락 항목을 확인한다.
4. [좌표 경계](../records/lidar/lidar-camera-frame-boundaries.md)와 [외부 참고 검토](../records/camera/camera-lidar-coordinate-reference-review.md)를 세부 근거로 읽는다.
5. [소스 README](../../src/asmc_lidar/README.md)에서 코어 API·빌드·시험을 확인한다.

`Main` 연결 구현은 opt-in이며 공통 경계와 roll 기본값은 모두 OFF다.
지면 제거·군집·OBB·tracker·전방 fusion은 이관 검증 노드로 이식했고 roll·pitch 수평화와 local/map 출력을 검증했지만,
운영 경로에는 연결하지 않았다. 구조물 분류는 아직 이전하지 않았다.
24점 삭제/보존과 신규 분류 차이는 [원본 이미지·점군 비교 갤러리](../../config/baselines/evidence/lidar_continuous_regression_v1/user-review/index.html)에서 확인한다.
24점은 장애물 정답이 없는 합성 지면 높이 입력이며, 실제 장애물 소실 사례로 해석하지 않는다.

## 재호: PR #72 반영 후 비교 갤러리 먼저 보기

현재 개발 PC에서는 [전체 실험 갤러리 목차](http://localhost:8768/)의 `lidar-pr72`에서
LiDAR 비교와 카메라 모델 비교를 함께 찾을 수 있다. 이 주소는 로컬 artifact 서버이며,
다른 PC에서의 재현·인계에는 아래 저장소에 포함한 갤러리와 실행 안내를 사용한다.

PR #72가 반영된 checkout에서
`config/baselines/evidence/lidar_continuous_regression_v1/user-review/index.html`을
파일 탐색기에서 브라우저로 연다. HTML·사진·GIF·상태/속도 JSON을 모두 Git에 포함했으므로
**ROS, GPU, MORAI, bag 원본이나 기존 임시 컨테이너가 없어도 볼 수 있다.** GitHub의
HTML 소스 보기 대신 로컬의 `user-review` 폴더 전체를 사용한다.

HTTP로 보고 싶으면 **저장소 루트** 터미널에서 아래 명령을 실행하고 브라우저로
`http://localhost:8767/index.html`을 연다. 종료는 해당 터미널의 `Ctrl+C`다.

```bash
python3 -m http.server 8767 --bind 127.0.0.1 \
  --directory config/baselines/evidence/lidar_continuous_regression_v1/user-review
```

검토 순서는 **연속 구간 1·2·3의 벽 조각/바닥 노이즈 → 분류 10번의 OBB와 우측 영상 →
11관측의 분류 카운터 → 속도 급변 분포**다. 24점은 사진 없는 합성 정책 시험이다.
같은 source stamp의 point→cluster→OBB→track/분류 이력을 이어서 확인한다.
[작업표](../../src/asmc_lidar/docs/migration-tasks.md)의 과거 구조물·속도 증거로 연결한다. 현재 인수·보완은 위 공동 개발 인수표를 따르며 연구 확대는 L8이다.
현재 세션의 localhost 8766 서버나 `/tmp` 경로는 향후 재현에 필요하지 않다.

L8 위치·속도 개선의 과거 참고 순서는 [과제 목록의 관측 조사·비교 순서](../../src/asmc_lidar/docs/migration-tasks.md#위치속도-관측-조사와-재호의-비교-순서--2026-10-08)를 따른다.
군집 평균·OBB 중심·필터 위치를 구분하고 시간/초기화/연결 → 구조물·군집 → 일관된
관측 위치 → 불확실성/연결 순서로 비교한다. Autoware 공개 구현과 연구 논문은
참고 후보이며 우리 센서에서 실행·성능 검증된 대안처럼 취급하지 않는다.
운영진이 답변한 후륜축 속도 GT와 현재 브릿지의 km/h 단위는
[속도 GT 인계](../../src/asmc_lidar/docs/migration-tasks.md#morai-속도-gt-기준--2026-10-06-전달-qa-반영)에서 확인한다.

**PR #72 병합 범위:** 공통 좌표·시간 코어, Main의 opt-in 연결, 회귀 도구·증거와
인계 자료다. 공통 경계/roll 기본값 OFF와 공개 속도/동정적 validity false를 유지한다.
병합은 객체 보존·속도 정확도·정상 map 추적 품질 또는 운영 기본값 승격의 승인이 아니다.

## 역할과 남긴 파일

카메라 패키지는 `asmc_camera`다(2026-10-06 `asmc_perception`에서 이름 변경). 안승현이
YOLO26l E8/사전학습 생산자를 `asmc_camera`의 정본으로 제공한다. 모델/통합 부하 검증도 카메라가 맡는다. 안승현은 HD맵 필터·카메라 연결을, 손재호는 추적·속도·운동 상태·fusion을 담당하며
소비 변경은 판제와 검토한다. HD맵은 지도와 주행영역 정본을 제공한다.
자세한 역할 분담은 [통합 로드맵](../architecture/consolidation-roadmap.md)을 따른다.

| 제공한 구성 | 사용할 때의 조건 |
|---|---|
| `asmc_lidar_frame_time` 라이브러리 | 실제 수평화 행렬·센서 extrinsic·유효 pose를 호출자가 제공 |
| IMU 선택 함수 | 도착 완료 표본만 전달, 유한값/유효 quaternion 검사 후 valid 지정 |
| v3 장착 설정 | 출처 SHA와 단위 확인. 장착 z를 노면 기준 센서 높이로 사용하지 않음 |
| 회귀시험 | 좌표·8코너·속도·관측시각·pose/IMU 선택 의미 검사. 실제 검출 성능 시험은 후속 |
| 공용 `asmc_msgs` 타입 | 기존 객체/관측 타입 MD5 유지, LidarTrackingState/Observation 추가. 같은 scan의 기하·적용 속도·4단계 상태를 원자적으로 제공. Main의 geometry opt-in 출력은 별도 경로 |
| 처리 노드·replay | `lidar_pipeline_node`·`lidar_replay`와 처리 `lidar.launch` 하나. 기본 `/perception/lidar`, 비교 namespace 선택 가능. 적용 속도/동정적 validity는 근거에 따름. TunnelFilter 기본 OFF, 실제 공동 주행/터널 성능은 미수락 |
| 웹 모니터 | PR #105를 현재 구조에 연결. `lidar.launch debug:=true`와 별도 `view.launch`, 기본 localhost:8091. 진단용 운동 표시와 실제 공용 속도/validity를 구분. HD맵·상부·이식 정책 제외가 발행 배열에 적용됨. [상세](../../src/asmc_lidar/docs/web-monitor.md) |

빌드·시험 명령과 API 위치는 [패키지 README](../../src/asmc_lidar/README.md)에 있다.
과거 연구용 지면·tracker 후보와 이전 `unified.launch`, `ground_compare_live.launch`는 남기지 않았으며
이를 안내하는 명령은 현재 버전에서 사용할 수 없다. 현재 처리는 `lidar.launch`, 읽기 전용 웹 화면은 `view.launch`로 분리한다.

## 통합 이관 순서

[L0–L8 작업표](../../src/asmc_lidar/docs/migration-tasks.md)가 유일한 실행 순서 정본이다.
L0 문서 → L1 구조·계약·기준선 → L2 검출 → L3 좌표·시간·추적 → L4 전방 fusion →
L5 인지 필터 → L6 발행·실행·시각화 → L7 대조·판제 전환 → L8 개선 우선순위로 진행한다.
각 단위는 별도 계획·구현·검증·커밋으로 닫는다. 재호의 PR #103·#105와 이번 구조를 재사용해
[공동 개발 인수와 우선 작업](../../src/asmc_lidar/docs/migration-tasks.md#공동-개발-인수와-우선-작업)부터 진행한다. 전체 경로를 다시 이식하지 않는다.

아래 모드 없는 구조물 분리·참고 구현 제안은 과거 인계의 후속 연구다. 이관 필수조건으로
모두 확장하지 않는다. 기존 객체 보존 회귀는 재사용하고 새 구조물 표현·억제는 L8에서
별도 선택한다. 기존 실패를 해결된 것으로 표시하거나 전체 승격 기준을 생략하지 않는다.

## 입력·출력과 처리 책임

[소비 계약](../contracts/lidar-planning-consumer-v1.md)과
[기계 판독 계약](../../config/contracts/lidar_candidate_topic_contract_v1.yaml)은
후속 구현의 목표다. 현재 publisher 목록이 아니다. 이관 검증 노드의 `/migration/lidar/*`는 이 계약의 토픽이 아니다.

- 현재 Main의 local 입력은 관측 시점 유효 IMU와 차량 속도(CAN)다. 각각 age 120ms, 최대 대기 50ms이며 GPS는 기다리지 않는다. CAN 없이 처리하는 LiDAR+IMU 전용 정책은 후속 설계다.
- scan stamp는 bridge 수신 시각이며 광선 측정 시각으로 바꾸어 설명하지 않는다.
  CPU 처리시간은 monotonic clock으로 별도 측정한다.
- 코어의 엄격 `PoseHistory` API는 동일 clock에서 최대 100ms 보간하며 외삽하지 않는다.
  Main의 별도 `BoundedPoseHistory`는 GPS 최대 age 300ms의 제한 보정과 정상 3회 복구를 사용한다.
  공통 경로는 ROS source ns와 steady 수신·처리 시각을 분리한다. 두 정책을 혼용하지 않는다.
  안테나와 후륜축 offset, HD맵 원점·높이, 복구 위치 점프는 ROS 어댑터에서 검증한다.
- 절대속도 벡터는 축만 회전한다. 상대 위치 미분은 자차 병진/회전을 함께 사용한다.
  twist가 없으면 unknown이며, 0속도나 static으로 위장하지 않는다.
- 객체/구조물 경계/미확정을 구분한다. 평평함·긴 OBB·낮은 점 수·카메라 미검출만으로
  삭제하지 않는다. 인접 차로·횡단 객체와 카메라 drivable의 장애물 구멍을 보존한다.
- 주행 관련성·차로 귀속은 속성으로 전달한다. 추종 대상·기동 유지·회피·정지는 판제 책임이다.

## 모드 없는 구조물 분리 — 손재호의 후속 구현 계약

2026-10-05에 작성한 구조물 설계다. **2026-10-25 우선순위:** 손재호가 GPS 없는 터널 local
인식·벽/천장 안정화를 전환 기반과 병행한다. 전체 구조물 표현 재설계를 첫 소비 전환의
필수조건으로 확대하지 않되, 터널의 필요한 객체 보존·local 공급은 실제 운영 인수에서 확인한다.
[최신 터널 작업·카메라/family 사용 원칙](../../src/asmc_lidar/docs/cutover-foundation.md#손재호-gps-없는-터널-local-인식)을 따른다.
현재 높이 ROI·상부 전달 제외와 별개로 아래 일반 구조물 근거·경계 계약은 설계이며 미구현이다. 건물/터널이라는 의미 클래스를 반드시 맞히는 것이 아니라, 독립 객체와
관측된 구조물 경계를 구분해 벽 전체에 차량용 OBB를 씌우는 문제를 줄이는 것이 목표다.

```text
LiDAR + IMU → 좌표·시간 정합 → 공통 ROI/voxel·지면 처리
  → 구조물 근거 추출 → 경계·돌출부를 고려한 군집 → 혼합 군집 검사
    ├─ 객체 후보: OBB·추적 (의미 클래스/속도는 미확정 가능)
    ├─ 구조물 경계: 실제 지지 범위의 표면·경계와 품질
    └─ 미확정: 관측점·점유 근거 보존, 억제하지 않음
```

같은 단계·설정을 일반도로→터널→복귀에 적용한다. GPS/미션 ID는 분기 조건이 아니며
GPS는 map 변환/지도 문맥의 가용성에만 관여한다. 현재 새 `asmc_lidar`도 LiDAR+IMU+차량 속도로
local을 처리한다. CAN 없는 LiDAR+IMU 경로의 추적·속도 정책은 후속 설계다.
IMU나 지면 근거가 무효이면 관련 구조물 판단도 미확정으로 내리고 실패 이유를 남긴다.

### 근거 추출과 군집 경계

| 사용할 관측 근거 | 반드시 함께 확인할 제한 |
|---|---|
| 중력 대비 법선, 평면 잔차, 연결된 지지점 | 차량/트럭 측면도 평평하다. 평면이라는 이유만으로 구조물로 확정하지 않는다. |
| 실제 지지 범위·연결성·높이 | 벽을 관측 밖까지 무한 연장하지 않는다. 가드레일은 얇고 끊긴 관측일 수 있다. |
| 표면 앞 돌출부·깊이 단절 | 벽 근처 차량/보행자/낮은 물체를 별도 후보로 보존한다. 희소 관측은 삭제 근거가 아니다. |
| 지면 대비 천장 높이·차량 통과 공간 | 검증된 차량 외형/자세와 여유 높이가 필요하다. 낮은 천장·보·돌출물은 충돌 위험으로 남긴다. |
| 반복 관측의 일관성 | 자차 운동·시간 대응이 유효한 경우만 사용한다. 정지 차량도 정적이므로 정적=구조물이 아니다. |

먼저 고정한 기존 군집 결과에 구조물 근거를 겹쳐 혼합 원인을 기록하고, 점 연결을 변경하는
단계는 별도 비교한다. 벽과 차량이 섞인 군집의 큰 OBB를 마지막에 삭제하는 방식은 금지한다.
군집 전 경계 근거와 군집 후 혼합 검사를 연결하되, 분리할 근거가 부족하면 미확정으로 보존한다.
OBB 크기 제한이나 지면 임계값을 동시에 조정해 구조물 분리 효과를 숨기지 않는다.

HD맵/카메라는 선택 문맥이다. frame·stamp·유효 범위·출처·품질을 검사한 뒤에만 보강한다.
누락/stale/잘못된 pose, 카메라 미검출 및 drivable의 장애물 구멍은 억제를 강화하는 근거가 아니다.
현재 경로 밖인 인접 차로 차량·진입/횡단 가능 객체도 남긴다.

### 결과와 진단의 설계 요구

- scan 식별자·관측 stamp·내부 수평화 행렬·출력 frame과 원본점 인덱스 계보를 보존한다.
  voxel 하나와 원본 여러 점의 대응, 삭제/보존 이유를 추적할 수 있어야 한다.
- 구조물 근거에는 지지점 인덱스, 법선/평면 또는 경계 형상, 실제 관측 범위, 잔차·지지량,
  품질/유효성·판정 이유를 둔다. 점 수만으로 보정되지 않은 확률 confidence를 만들지 않는다.
- 구조물 근거 마스크와 실제 억제 마스크는 분리한다. 객체 보호 근거와 충돌하면 억제하지
  않고 미확정 사유를 남긴다. **새 구조물 억제 OFF에서는 기존 비지면 점의 추가 삭제가 0건**이어야 한다.
- 객체 OBB와 구조물 표면/경계, 미확정 점유는 역할이 다르다. 구조물은 객체 추적에서
  제외할 수 있어도 충돌 경계에서 지우면 안 된다. 빈 객체 배열은 자유공간 보장이 아니다.
- 현재 `LidarObject*`는 구조물 경계용 메시지가 아니다. 구조물 형상을 객체 corner나
  classification 필드에 억지로 넣지 않는다. 후속 구현에서 별도 additive/versioned 계약을
  판제와 합의하며, 그 전에는 내부 결과와 검토용 점군/MarkerArray/진단으로 확인한다.
  시각화만으로 판제의 충돌 경계 소비가 구현됐다고 판단하지 않는다.

### 작은 단위의 분석·구현·승격 순서

아래는 L8에서 구조물 개선을 선택할 경우의 내부 순서이며 L0–L7의 실행 순서를 대체하지 않는다.

1. 원본 SHA·설정·입력을 고정하고 부적절한 OBB가 생기는 점→군집→객체 경로를 기록한다.
2. 구조물 근거 추출/시각화만 추가한다. 억제 OFF로 기존 검출 결과와 점 보존 parity를 확인한다.
3. 객체 보호·혼합 군집 분리를 한 단위씩 비교한다. 지면/군집/추적 변경은 서로 다른 실험으로 둔다.
4. 객체 보존과 경계 전달을 함께 검증한 뒤에만 억제 ON을 검토한다. 기본값 승격은 구현 담당과
   판제 담당자의 별도 수락 사항이며, 실패 시 억제 OFF와 고정 기준선으로 되돌린다.

| 고정 회귀 입력 | 비교할 실패와 성공 근거 |
|---|---|
| 일반도로 가드레일·건물, 터널 바닥/벽/천장 | 단일 경로 유지, 구조물 지지 범위, 혼합 군집/대형 OBB 변화 |
| 트럭 측면·정차 차량·벽 근처 보행자/장애물 | 객체점/관측 하부 보존, 추가 완전 소실, 차량을 벽으로 잘못 억제하는지 |
| 낮은 장애물·낮은 천장/보·가드레일 옆 객체 | 충돌 공간 안의 관측이 보존되는지, 경계가 무한 확장되지 않는지 |
| 희소/부분 가림·곡면/단차·인접/횡단 객체 | 미확정 보존, 과분할/합침, 현재 경로 밖 객체를 삭제하지 않는지 |
| 동일 scan/IMU에서 GPS·카메라/지도 가용성 변경 | GPS만 바꾸면 local geometry 동일, 문맥 누락이 추가 삭제를 만들지 않음 |

정량 gate는 실험 전에 고정한다. 원본점/객체 보존과 최종 객체 품질을 별도로 보고하고,
필터가 붙인 구조물 분류를 정답으로 사용하지 않는다. 정답 없는 실제 bag에서는 대표
BEV/측면·군집 대응·평면 선택·판정 이유를 검토한다. 시간 안정성은 위치 대응이 확인된
재관측에서만 평가한다. 15Hz 입력의 처리율·누락·단계별 p50/p95/p99와 설정/소스 SHA를 남긴다.
RViz는 객체 OBB/ID, 구조물 지지 범위/법선, 미확정점, 억제 전후, 상태를 각각 토글하고
stale 화면을 지운다. 이는 구현 후 수락 항목이며 현재 실행 가능한 시각화 목록이 아니다.

### 참고 구현 검수와 현실적인 첫 제안 — 2026-10-05

이번 검수는 **공식 문서/논문·선택한 주요 코드의 읽기 검토**다. 새 후보의 빌드·bag 실험·
실시간 성능 검증은 하지 않았다. 아래 우선순위는 우리 계약에 맞춘 설계 제안이지 검증된
성능 순위가 아니다. external은 개인 워크스페이스의 참고 자료이며 팀 런타임 의존성이 아니다.
이번에 clone/vendor/의존성 추가를 하지 않았고 Patchwork++·linefit 연구 코드를 복구하지 않았다.

| 우선순위·참고 | 참고할 기능 | 우리 입력에서의 제한·적용 판단 |
|---|---|---|
| **1. PCL 평면 지지점 + 연결성** | `SACSegmentation`의 평면 계수와 inlier index, `RegionGrowing`의 법선/곡률 기반 표면 분리 | 기존 PCL 처리와 연결하기 쉬운 첫 후보. 국소 지지 범위를 추출할 뿐 벽/차량 의미를 보장하지 않는다. 희소 VLP16 법선이 불안정하면 unknown으로 두고, 전역 최대 평면을 통째로 삭제하지 않는다. |
| **2. PCL Conditional Euclidean Clustering** | 거리 외에 사용자 정의 점 연결 조건을 적용, 작은/큰 군집도 별도로 회수 가능 | 근거 추출 검증 후 군집 단계만 별도 비교. 구조물 경계/깊이 단절을 연결 조건에 반영할 수 있으나 과분할을 유발할 수 있다. 점 수/크기로 빠진 군집도 계보와 미확정 결과에 보존한다. |
| **조건부 대조군: depth_clustering (IROS 2016)** | range-image에서 빔 간 각도와 거리로 연결성을 판단하는 희소 점군 군집화 | 논문에 VLP16 평가가 있으나 구조물 의미 분류기는 아니다. 비스듬한 벽의 과분할 한계가 명시돼 있다. 센서 원점 투영·빈 셀·셀 충돌·원본점 계보 검증을 먼저 통과해야 한다. |

공식 구현 근거:
[PCL 평면 분할](https://pointclouds.org/documentation/tutorials/planar_segmentation.html),
[Region growing](https://pointclouds.org/documentation/tutorials/region_growing_segmentation.html),
[Conditional Euclidean Clustering](https://pointclouds.org/documentation/tutorials/conditional_euclidean_clustering.html).
이들은 구현 부품이지 구조물 자동 삭제를 완성한 알고리즘이 아니다. 튜토리얼의 거리/점수
기본값을 그대로 사용하지 않으며, 실제 구현 때 Noetic 컨테이너의 PCL 버전·설정을 고정한다.

논문 **Fast Range Image-Based Segmentation of Sparse 3D Laser Scans for Online Operation**
(Bogoslavskyi·Stachniss, IROS 2016)의
[원문](https://www.ipb.uni-bonn.de/pdfs/bogoslavskyi16iros.pdf)과
[공식 코드](https://github.com/PRBonn/depth_clustering)를 확인했다. 로컬
`external/baselines/depth_clustering`은 `2a824e728725be635eec4cc5214350faea364712`,
`LICENCE.txt`는 MIT이며 작업트리가 깨끗했다. `angle_diff.cpp`의 연결각 계산과
`spherical_projection.cpp`의 점 인덱스 저장을 확인했다. 특히 후자는 “가까운 점”이라는
주석과 달리 `current_written_depth < dist_to_sensor`일 때 갱신해 더 먼 깊이를 선택한다.
따라서 그대로 복사하지 말고 다중점 셀의 대표 깊이/라벨 복원 회귀를 먼저 추가해야 한다.
현재 CMake는 OpenCV/Eigen/Boost 및 Qt/QGLViewer/OpenGL을 요구하므로 코어만 격리할 때
GUI 의존성을 따로 검토한다. 논문의 FPS를 우리 전체 처리시간으로 환산하지 않는다.

후순위 또는 이번 목적과 다른 자료도 구분한다.

- **TRAVEL**: 기존 `external/baselines/TRAVEL`의 `95dc2fbd66a343efd9060c45a5711b6307a950a4`
  (작업트리 깨끗함)과 [공식 레포](https://github.com/url-kaist/TRAVEL)의 README를 확인했다.
  지면/군집 코어가 분리된 참고 구현이지만 이전 미채택 결론을 뒤집거나 재실험하지 않는다.
  군집 분리 자체가 구조물 의미 판정을 대신하지 않는다.
- **M-detector**: `external/baselines/M-detector`의
  `0645dabe307a99f3b59a5a3d3007acfdcad1a39f` (작업트리 깨끗함), README/설정과
  LICENSE의 GPL v2 표기를 확인했다. [공식 레포](https://github.com/hku-mars/M-detector)는
  움직임 검출과 odometry 입력을 다룬다. 정차 차량/정적 장애물과 벽을 구별하는 문제를
  해결하지 않으므로 첫 단계로 채택하지 않는다. 시간·운동/배포 의존성 검토도 선행해야 한다.
- **RangeNet++ (IROS 2019)**: [논문](https://www.ipb.uni-bonn.de/pdfs/milioto2019iros.pdf)과
  [공식 추론 레포](https://github.com/PRBonn/rangenet_lib)를 온라인으로만 확인했다.
  학습 기반 점별 의미 분류의 참고 대상이지 현재 SOTA라고 선정한 것이 아니다. 우리 VLP16/
  MORAI에서의 라벨 대응·도메인 차이·GPU 통합 부하가 미검증이며, weights/license와
  revision도 미동결이다. 이번에는 clone/학습/추론하지 않고 후순위로 둔다.

**당시 재호에게 제안한 첫 연구 실험은 1번의 관측 전용 구조물 근거 추출이다.**
현재 다음 작업은 위 공동 개발 인수표를 따르며 아래 연구 제안의 재개 여부는 L8에서 결정한다. 기존 지면·
군집·OBB를 고정하고 동일 scan에서 유한한 평면 지지 범위와 돌출부/미확정만 겹쳐 표시한다.
억제 OFF의 점/객체 결과 동일성, 벽과 트럭 측면을 혼동한 사례, 지지 범위의 누락과 계산
비용을 먼저 기록한다. 가드레일/곡면의 근거가 부족하면 미확정으로 남긴다. 그 결과를 보고
2번의 연결 조건 변경을 별도 실험하며 둘을 한꺼번에 적용하지 않는다. 근거 분리 성능이
확인되기 전에는 새로운 자동 억제나 판제 입력 전환을 하지 않는다.

## 수락과 기존 패키지 삭제 조건

| 대상 | 확인할 증거 |
|---|---|
| 원본 기능 | 동일 입력의 지면/군집/OBB·ID/연관·생애주기·분류 유지, 의도적 차이 목록 |
| 좌표·시간 | 비영점 roll/pitch/yaw·장착 offset·8코너·속도, stamp 영점/역행, 미래 pose 제외 |
| GPS | 정상/NaN/중단/stale/복구 점프에서 local 연속성·map 상태, 한 토픽 frame 고정 |
| 구조물 | 벽/가드레일/차량 측면/낮은·인접·횡단 객체, 새 억제 OFF에서 추가 삭제 없음 |
| 소비 | 기존 MD5 보존, camera·LiDAR 타입 직렬화, 세 판제 모듈과 음영 소비의 보류/freshness 처리 |
| 부하 | 입력 Hz·처리 Hz·drop·queue·p50/p95/p99·CPU/GPU/메모리, 카메라 동시 실행과 RViz 영향 |

15Hz 설정과 실제 수신/처리율을 함께 기록한다. 약 12Hz 완성 cloud를 허용한 운영 요구와
15Hz 부하 시험 결과는 각각 표기한다. 실제 검증하지 않은 주행/이동 객체 조건은 미검증으로 남긴다.
현재 골격 검증은 검출·추적 정확도나 주행 성능 승인에 해당하지 않는다.

기존 패키지 삭제는 B4에서 모든 consumer 이전·인수·복구 확인 후 별도 PR로 한다.
참조/의존성/launch/설치 잔류물·CI까지 정리하고 `asmc_lidar`를 유일한 현행 실행 정본으로 만든다. rollback은
새 생산자/어댑터를 종료하고 고정한 원본 launch로 복원한다. 같은 토픽에 양쪽 생산자를
동시에 붙이지 않는다. 이번 PR은 운영 입력 전환이나 카메라 rename을 수행하지 않는다.

## 이번 골격의 검증 기록 — 2026-10-05

- 기존 build/devel을 사용하지 않은 Noetic 임시 경로에서 패키지·공용 메시지 빌드와
  전체 상위 catkin workspace 빌드가 통과했다.
- CTest 2개가 통과했다. 좌표·시간 실행파일 내부 35개 사례와 IMU 선택시험을 포함한다.
- 생성 메시지 직렬화/MD5 검사, 공용 계약 unit test 25개와 bridge profile test 2개가 통과했다.
- 문서 보존 검사는 과거 Git 이력 대조까지 통과했다. CI에서 발견한 신규 타입 등록 누락과
  기존 음영 어댑터 디렉터리 이동 후 남은 감사 경로를 교정했다. 실제 ROS 패키지명은
  `lane_boundary_adapter`이며 디렉터리만 `jamming_lane_boundary_adapter`다.
- 이 골격에는 runtime publisher가 없으므로 시뮬레이터 검출·주행 수락은 수행 대상이 아니다.
  손재호의 기능 이전 후 위 수락 목록으로 검증한다.

## 연구 이력 조회

사용자 요청으로 개인 연구 코드·설정·비교 그림·보고서를 현재 트리에서 삭제했다.
정리 전 전체 내용은 `020f6c2`, 원래 연구 worktree의 추가 GT 수집 변경은 `f1dfb84`에서
조회할 수 있다. Git 이력을 재작성하지 않았으며 재호 작업 브랜치와 외부 연구 checkout은
그대로다. 실제 bag·Capture 자료는 `$ASMC_DATA`에 유지한다.

```bash
git show 020f6c2:src/asmc_lidar/src/ground_live.cpp
git log --all -- src/asmc_lidar
```

복구가 필요하면 별도 worktree에서 검토하고 현재 골격에 연구 폴더를 일괄 복사하지 않는다.

2026-10-23 사용자 직접 확인: 음영구간 TunnelFilter ON 시 객체 박스가 많이 튐. 표시 연결만 확인했으며 안정성 미수락, 필터 기본 OFF 유지. 재호의 추적·속도·fusion과 안승현의 HD맵 필터 후속 검토에서 같은 입력으로 원인을 분리한다.
