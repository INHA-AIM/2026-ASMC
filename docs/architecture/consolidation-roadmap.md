# 저장소 통합·HD Map·인지·현장 배포 로드맵

> **문서 역할:** Working Proposal — 패키지 통합과 본선 배포를 위한 단계별 실행 계획
> **작성 기준:** 2026-09-20 저장소와 현장 점검 반출 자료의 read-only 감사
> **진행 상태:** 1·2단계, 3단계 P3-G local closeout, P4-A·P4-B·P4-C1~C6,
> 5단계 P5-A~P5-E 완료. 2026-10-05부터 독립 LiDAR 후보 패키지 경계를 추가한다.
> 과거 LiDAR의 perception 흡수와 모든 노드 단일 실행기 계획은 현재 결정이 아니다.
> **보호 범위:** `behavior_planner`, `frenet_planner`, `mpc_controller`,
> planning 소유 Behavior/Frenet/MPC package는 이 계획의 일방적 이동·삭제 대상이 아니다.
> read-only 음영 차선 어댑터는 현재 `src/jamming_lane_boundary_adapter/`에 둔다.

## 1. 결론

팀이 직접 유지하는 공통 ROS 코드의 **목표**는 다음 다섯 패키지다.
카메라 패키지는 2026-10-06 `asmc_perception`에서 `asmc_camera`로 이름을 바꿨다(호환 wrapper 없이 완전 이전, 옛 경로에는 이전 안내 README만 남김).
YOLO 생산자 이전은 아직 남아 있으며,
`asmc_lidar`는 좌표·시간 코어만 남긴 이전용 골격이다. 아래 목표를 운영 이전 완료로 해석하지 않는다.

| 정본 패키지 | 책임 |
|---|---|
| `asmc_msgs` | 팀 내부 typed message의 유일한 공용 패키지 |
| `asmc_bridge` | MORAI UDP 수신·제어 송신, sensor decode, 외부→내부 상태 정규화 |
| `asmc_hd_map` | HD Map build/runtime, 차선·차로·주행가능영역, 체크포인트·미션 메타데이터 |
| `asmc_camera` (구 `asmc_perception`) | 카메라 객체 검출/YOLO·차선/주행영역·신호등, BEV·카메라 후처리 |
| `asmc_lidar` | LiDAR+IMU 좌표·시간, 지면/구조물·군집/OBB·추적, 카메라 분류 fusion, 객체 local/map 어댑터 |

이는 **팀 소유 공용·인지 코드의 경계**다. 독립 수집 패키지, 판제 소유 패키지와 음영 차선 어댑터는 별도다.

- 판제 소유 패키지는 현재 이름과 경로를 유지한다.
- `MORAI-ROS_morai_msgs`는 외부 wire contract이므로 `asmc_msgs`로 복사하지 않고 핀된
  dependency로 유지한다.
- Velodyne driver처럼 upstream ROS 패키지인 코드는 우리 perception 패키지로 합치지 않는다.
  apt dependency 또는 `vendor/`로 격리한다.
- 학습·dataset bake·일회성 분석은 catkin `src/`에서 빼되, runtime perception 코드와
  재현 가능한 설정은 보존한다.

패키지 이동보다 먼저 메시지·토픽·정적 데이터 계약을 동결한다. 넓은 rename과 삭제를 먼저
하면 현재 HD Map 검수 결과, 판제 consumer, UDP 현장 설정 중 무엇이 깨졌는지 분리하기 어렵다.

### 센서별 인지와 fusion의 소유 경계 (2026-10-05 개정 결정)

안승현의 요청에 따라 **카메라 목표 이름은 `asmc_camera`, LiDAR 최종 목표는
`asmc_lidar`**로 정한다. 손재호가 기존 `lidar+sensorfusion`의 기능을 보존하면서
검증된 수정사항을 반영하고 이전을 마무리한다. 별도 `asmc_fusion`은 만들지 않는다.
이 결정은 담당자별 구현 계획이며 코드 rename·운영 승격 완료를 뜻하지 않는다.

- `asmc_camera`: YOLO 추론·2D box, 차선/주행영역, 신호등, 다중 카메라 BEV.
- `asmc_lidar`: LiDAR 검출·추적, 카메라 box와의 시각/투영 매칭·클래스 유지,
  객체 footprint의 차로/주행영역 관계 및 local/map 변환. 선택적 카메라 입력이
  없어도 LiDAR는 실행하며 YOLO를 중복 로드하거나 카메라 Python 구현을 import하지 않는다.
- `asmc_hd_map`: 정적 지도·차로·주행영역·미션 제공. GPU 추론이나 동적 객체 fusion은
  넣지 않는다. map 사용이 불가능하면 객체의 지도 관련성만 unknown이며 local 객체는 남긴다.
- 공용 wire 계약은 `asmc_msgs`, 추종 대상·회피·정지·기동 유지 정책은 판제 소유다.
  `camera_bev_fusion`은 카메라끼리의 BEV 결합이므로 카메라 패키지에 남는다.

정정 이력: 같은 날짜의 이전 권고는 `asmc_perception` 이름 유지와 별도 fusion 분리를
검토했다. 담당 경계 명확화와 손재호 인계 비용을 우선한 이번 결정으로 대체한다.
과거 논문·실험 결과와 당시 후보의 수치는 변경하지 않는다.

### 카메라 이름 변경과 손재호 LiDAR 이전 상세 계획

#### A. 현재 상태와 출발점 동결

| 항목 | 현재 | 완료 목표·담당 |
|---|---|---|
| 카메라 패키지 | `src/asmc_camera/`(2026-10-06 rename); YOLO 원본 실행기는 아직 LiDAR 쪽 | `src/asmc_camera/`; 안승현이 rename 후 YOLO 실행/계약을 별도 통합 |
| 원본 LiDAR | `src/lidar+sensorfusion/`의 nested `Main`·`Msg` | 손재호가 기존 기능과 개선 패치를 `src/asmc_lidar/`로 이전 |
| 새 LiDAR 골격 | 좌표·시간/IMU 코어·회귀시험, 공용 메시지 정의 | 손재호가 검출·추적·fusion·ROS 어댑터를 원본에서 이전 |
| 구조물 억제 | 골격에는 구현 없음 | 구현 시 OFF에서 출발, 손재호의 객체 보존 회귀 이후에만 승격 |
| 판제 연결 | 후보 토픽 격리, 기존 운영 입력 유지 | Behavior/Frenet/MPC·음영 소비 담당자의 공동 검증 후 전환 |

구현 시작 시 main·손재호 작업 브랜치·통합 브랜치의 실제 SHA, 설정·센서·모델 SHA,
topic/type/MD5, 실행 인자, 고정 bag과 실패 사례를 기록한다. 문서상의 과거
`6435af1`·`8dde9b6`을 최신 코드라고 간주하지 않는다. 담당자 브랜치는 대신 덮어쓰지 않는다.

개인 연구 알고리즘·비교 도구·보고서·후보 tracker와 ROS 실행기는 삭제 커밋으로
정리하고 Git 이력에만 보존했다. [현재 인계 문서](../packages/asmc_lidar/README.md)를
따라 원본 tracker의 연관·ID 생애주기·OBB 안정화·분류 유지·동정적 정책을 우선
보존한다. 좌표/시간 오류 때문에 달라져야 하는 결과는 의도적 차이로 별도 시험한다.
정리 전 `020f6c2`에서 필요한 개선 근거만 선별 확인하고 연구 폴더를 일괄 복구하지 않는다.

#### B. 안승현 — 카메라 rename과 YOLO 생산자 정리

1. **이름 변경만 먼저:** `package.xml`, CMake/catkin, Python 설치·import, launch의
   `pkg`/`find`, 스크립트·Docker/CI·RViz/rqt·테스트·학습 경로를 목록화한다.
   `src/asmc_perception` → `src/asmc_camera`로 옮기되 weight/라벨/기본 추론 옵션은 유지한다.
   현재 컨테이너 직접 실행 방식, node 이름과 `/perception/*` 토픽은 rename 때문에 바꾸지 않는다.
2. 새 명령은 rename 구현 후 `roslaunch asmc_camera ...`로 안내한다. 구
   `roslaunch asmc_perception ...`는 최소 forwarding launch로 한 릴리스 호환을 제공하고,
   실제 consumer에서 쓰는 Python import도 조사해 필요하면 shim을 둔다. 두 이름을
   동시에 실행해 중복 노드가 생기지 않도록 안내한다. wrapper 제거는 담당자 확인 후 별도 PR이다.
   **1~2 완료(2026-10-06):** `src/asmc_camera`로 완전히 이동했다. 담당자 결정으로 forwarding
   launch와 import shim은 두지 않고 옛 경로에는 이전 안내 README만 남겼다(저장소 밖 consumer 없음).
   `docs/packages/asmc_perception/` 문서 경로 이동은 링크 점검과 함께 후속 PR로 남긴다.
3. **YOLO 물리 이전은 별도 변경:** 현재 weight·class mapping·threshold·box/stamp 출력을
   고정해 camera 쪽 단일 생산자로 옮긴다. 새 모델 비교·FP16/TensorRT·class 확장은
   이전과 섞지 않는다. LiDAR와 음영 어댑터가 같은 출력을 소비하도록 연결한다.
4. 공통 검출은 영상 원본 stamp, camera ID, 원본 이미지 크기/픽셀 box, confidence,
   원본 class와 정규화 class 의미를 보존한다. `person/car/truck/bus`는 호환 기준이다.
   빈 검출·미수신·stale·미지원 클래스는 구분하고 소비자별 crop/threshold는 어댑터에 명시한다.
   현재 후보 CameraBox 계약으로 표현이 부족하면 별도 계약 검토하며 기존 MD5를 묵시 변경하지 않는다.
5. 문서 `docs/packages/asmc_perception/` 이동은 코드 rename 때 링크 점검과 함께 처리한다.
   날짜가 있는 실험·학습 기록은 과거 이름을 보존한다. 지금은 문서 경로를 옮기지 않는다.

#### C. 손재호 — 원본 기능 보존, 수정 적용, `asmc_lidar` 마무리

1. 원본에 검증된 수정사항을 작은 커밋으로 반영하거나 새 패키지에 원본 단계를 먼저
   옮겨 동일 입력을 비교한다. 어느 순서든 **물리 이동과 알고리즘 변경은 분리**한다.
   필요한 소스만 출처·SHA·라이선스·변경 이유와 함께 이식하며 연구 worktree 전체를 복사하지 않는다.
2. 선택 평면의 높이/법선/지원 영역을 검사하는 지면 보완은 재호가 원본에 적용하고
   고정 입력으로 대조한다. 이전 실험 구현은 Git 이력에서 필요한 부분만 참고한다.
   ROI·voxel·군집·L-shape·tracker 임계값을 동시에 바꾸지 않는다. 이전만으로 기존
   객체 하부 삭제·군집 분할·큰 OBB가 해결됐다고 판단하지 않는다.
3. 공개 local 좌표를 후륜축 `base_link`로 통일한다. 실제 적용 수평화의 역변환과
   센서 장착 변환을 각각 한 번 적용하고 중심·8코너·방향축·속도를 함께 검사한다.
   내부 센서 원점 수평화 좌표를 `base_link`나 `velodyne`으로 잘못 표기하지 않는다.
4. 관측 stamp를 보존하고 수신/처리 시각을 분리한다. bridge stamp는 실제 광선 측정시각이
   아니다. 상대속도와 절대속도 축 변환·자차 병진/회전을 구분하며 CAN 속도를 중복 가산하지 않는다.
   twist가 없으면 절대속도·동정적 상태를 unknown으로 남긴다.
5. **모드 없는 단일 검출:** 일반도로→터널→복귀에 같은 단계·설정을 사용한다.
   GPS·미션으로 검출 알고리즘을 전환하거나 tracker ID를 초기화하지 않는다.
   기존 터널 분기를 단순 삭제하지 말고 필요한 천장·벽 근거 추출을 공통 경로에서 검증한다.
   지면이 불확실하면 구조물 판단도 불확실하며 오래된 평면을 무조건 재사용하지 않는다.
6. 벽·가드레일은 충돌 경계이며 곧바로 “무시 가능”하지 않다. 객체/구조물/미확정을
   분리하고 새 구조물 자동 억제는 OFF로 출발한다. 차량 측면·벽 근처 장애물·낮은 장애물·
   인접/횡단 객체 회귀를 통과한 뒤 승격한다. 평평함·긴 OBB·카메라 미검출만으로 삭제하지 않는다.
7. LiDAR 카메라 fusion은 `asmc_lidar` 내부 모듈로 유지한다. 카메라 지연·누락은
   분류 품질/상태에 반영하되 검출을 멈추거나 ID를 재생성하지 않는다. HD맵·카메라 문맥은
   선택 입력이며 잘못된 frame·stale·카메라 영역의 장애물 구멍을 삭제 근거로 쓰지 않는다.
8. `Main`/`Msg` nested build와 source 없이 상위 catkin에서 빌드·실행되게 정리한다.
   외부 `morai_msgs`는 그대로 외부 계약으로 두고, 공용 메시지는 `asmc_msgs`를 사용한다.
   legacy `Msg`는 필요할 때만 켜는 경계 어댑터로 제한한다. 연구 backend/weight/로그가
   기본 runtime의 필수 의존성이 되지 않게 분리하되 실험 이력은 삭제하지 않는다.

구조물 처리의 첫 목표는 **근거 추출 → 경계 기반 군집/혼합 검사 → 객체·구조물·미확정
분리**다. 근거 추출은 실행하되 새 억제는 OFF로 검증하고, 구조물 경계의 판제 전달과
객체 보존 회귀를 함께 통과한 뒤 승격한다. 빈 객체 목록을 자유공간으로 해석하지 않는다.
상세 분석 항목·점 계보/진단 요구·단계별 gate는
[구조물 분리 인계 계약](../packages/asmc_lidar/README.md#모드-없는-구조물-분리--손재호의-후속-구현-계약)에 모아 둔다.
현재 골격에 이 검출 기능이 구현됐다는 뜻은 아니다.

#### D. 공용 메시지·local/map·판제 소비 계약

| 출력/책임 | 이전 시 지킬 의미 |
|---|---|
| `/perception/lidar/objects_local` | `base_link` 객체·관측/예측·지속 ID·분류/속도 유효성. GPS 없이 계속 실행 |
| `/perception/lidar/objects_map` | 동일 ID/관측을 관측시점 유효 pose로 변환. 무효 pose는 정상 빈 관측과 구분 |
| 카메라 공통 출력 | 현재 후보 `/perception/camera/objects`와 타입을 출발점으로 생산자/소비자 공동 동결. 패키지 rename으로 토픽 rename하지 않음 |
| 기존 typed/legacy 출력 | 기존 MD5 유지. 표현 가능한 유효 출력만 변환하고 보류 이유·상태를 알림 |
| 판제 판단 | 객체 관련성과 차로 속성을 받아 대상 선택·충돌 판단·기동 유지. 인지로 정책을 되먹여 객체 삭제하지 않음 |

기존 `TrackedObject*`·`DetectedObject*` MD5와 운영 `/last_lidar`, `/lidar_nogps`,
`/perception/objects`는 소비자 전환 승인 전까지 유지한다. 새 상태형 `LidarObject*`는
관측/예측·미확정 속도·입력 무효를 보존한다. 프레임 검출 번호는 지속 track ID가 아니다.
기존 타입으로 표현할 수 없으면 정상 빈 배열이나 유효한 0속도로 바꾸지 않는다.

map 어댑터는 안테나와 후륜축 offset·HD맵 원점·높이를 검증한 관측시점 pose를 사용한다.
이미 도착한 유효 pose 사이 최대 100ms 보간, 외삽 금지를 출발 계약으로 유지한다.
2D `/vehicle/state`만으로 완전한 3D pose를 만들지 않는다. GPS 중단·NaN·stale·복구 점프는
map 가용성에 반영하며 local 검출·추적 reset 조건으로 사용하지 않는다.
판제가 local을 사용할지 map을 사용할지는 명시적 어댑터로 정하고 한 토픽 frame을 전환하지 않는다.

기계 정본은 [후보 topic 계약](../../config/contracts/lidar_candidate_topic_contract_v1.yaml),
현재 골격과 후속 시험 범위는 [LiDAR 인계](../packages/asmc_lidar/README.md)에서 확인한다.
서준 고정 브랜치와 main의 메시지 차이, main Behavior GT 경로, 강태 음영 소비 연결은
아직 남은 통합 항목이다. Frenet 단독 변환 통과를 세 판제 모듈 전체 승인으로 확대하지 않는다.

#### E. 검증·인계·기존 경로 삭제 gate

| 묶음 | 반드시 남길 증거 | 통과 전 금지 |
|---|---|---|
| rename/물리 이동 | clean Noetic build, import/launch/설정 탐색, 구/신 명령 smoke, 문서 링크 검사, 동일 입력 결과 | 모델 교체·토픽 이름 변경·구 경로 삭제 |
| 지면·좌표·시간 패치 | 고정 원본 대비 마스크/점 계보 비교, 비영점 roll/pitch/yaw·장착 offset·8코너·속도, stamp 오류 회귀 | tracker 임계값 동시 튜닝 |
| tracker/fusion 이식 | 고정 객체 입력의 ID/연관/생애주기·분류 유지 비교, 의도적 차이 목록, camera empty/missing/stale 시험 | 기능 이름만으로 원본 parity 주장 |
| GPS·모드 분리 | 동일 scan/IMU에 GPS만 바꾼 local 결과 동일성, 정상→음영→복귀, pose 중단/NaN/점프 시험 | local 중단/reset, frame 자동 변경 |
| 객체·구조물 | 가드레일·벽·차량 측면·낮은/인접/횡단 객체, 억제 OFF 시 새 삭제 0건 | 미검증 자동 억제 ON |
| 운영 인계 | RViz와 실제 주행, 원본 bag/설정 SHA, 카메라 동시 실행 15Hz 입력의 처리율·drop·대기·p50/p95/p99·CPU/GPU/메모리 | 10~13Hz 또는 가속 재생만으로 15Hz 통과 선언 |
| 판제 전환 | Behavior/Frenet/MPC 및 음영 소비 담당자의 frame/시간/MD5·stale·보류 시험, rollback 명령 | 기본 planner remap·원본 package/Msg 삭제 |

정량 합격 기준은 실행 전에 고정하고 개선/악화·미검증 조건을 함께 남긴다. 성능 목표를
못 맞추면 후보와 기존 실행을 병행 가능한 상태로 인계하며 “이전 완료”와 “운영 승인”을 구분한다.
현재 지면 비교 완료가 추적 정확도·map 가용성·15Hz 통합 부하 통과를 뜻하지 않는다.

커밋/PR은 **카메라 rename → 카메라 검출 생산자 이전**, **LiDAR 원본 기능 이식 →
좌표/시간·단일 경로 패치 → 공용 메시지/어댑터 → 수락·인계**로 나눈다.
두 파트는 공통 검출 계약을 먼저 합의하면 병행할 수 있다. 새 문서 계획만으로 PR·운영 전환을
자동 수행하지 않는다. 패키지 이동 변경에는 import/launch/path 변경표를 반드시 첨부한다.

완료 후 손재호는 `asmc_lidar`에서 구조물 억제·군집·추적·속도 안정성을 계속 개선한다.
안승현은 개정 규정 HD맵 → camera rename/YOLO 계약·이전 → 동시 부하 → 모델 비교 및
BDD100K 후 기존 데이터 추가학습을 진행한다. 학습 라벨/head 호환성은 별도 감사한다.
LAN2LAN은 연구 후보 성공을 모두 기다리지 않고 검증된 기준선으로 리허설한다.

rollback은 후보 출력/어댑터를 종료하고 고정된 원본 실행을 복원하는 것이다. 양쪽이
동일 운영 토픽을 동시에 발행하지 않게 한다. 구 `lidar+sensorfusion`·`Main`·`Msg` 및
rename wrapper 삭제는 모든 소비자 이관, 패키지 경로 잔여 참조 감사와 담당자 승인 이후 별도 변경이다.

## 2. 현재 감사 결과

2026-09-25 P4-C6 기준 `src/`에는 `package.xml`이 19개 있다. 이 수에는 팀 패키지뿐
아니라 LiDAR nested workspace의 `morai_msgs`, `Msg`, 여섯 Velodyne 패키지가 포함된다.
루트 `scripts/`에는 실행·학습·분석 진입점 29개가 남아 있으며, 단일 CLI 정리는
Phase 5 범위다.

| 현재 영역 | 판단 | 처리 방향 |
|---|---|---|
| `asmc_msgs` | 공용 메시지 28개(기존 13 + HD Map 12 + perception 3), 현재 정본 | 유지·호환 검증 |
| `behavior_planner/msg` | planning 소유이며 공용 메시지와 일부 중복 | 1단계에서 수정 금지. planning 협의 후 adapter 또는 별도 이관 |
| `lidar+sensorfusion/src/Msg` | `TrackedObject*`, `Yolo_bbox*` 4개가 별도 메시지 패키지 | 소비자 이관과 함께 `asmc_msgs`로 흡수 |
| `morai_udp_bridge` + `morai_sensor_bridge` + `vehicle_state` | P3-F 이전 세 패키지로 분산 | `asmc_bridge`로 물리 통합 완료 |
| `integration_launch` | P3-F 이전 planning 공동 launch | consumer launch를 갱신하고 `asmc_bridge/launch`로 통합 완료 |
| `asmc_hd_map` + `mgeo_toolkit` | runtime·builder는 통합 완료, 외부 toolkit은 provenance 입력 | geometry parity 후 toolkit 의존 축소 |
| `camera_semantic_perception` + `drivable_bev` + `traffic_light` | P4-C6 통합·검증 완료 | `asmc_perception` runtime/offline/training으로 이관하고 기존 ROS package 제거 |
| `morai_3d_detection` | 실행·참조 0인 중단된 camera 3D 연구 | P4-C6에서 제거, 필요 시 Git history에서만 복원 |
| `lidar+sensorfusion/src/Main` | 재호의 기존 운영·비교용 nested workspace | 새 LiDAR 후보 검증·인계 전까지 rollback 기준으로 유지 |
| `data_collection` | recorder·bag profile·replay와 dataset 도구 | 독립 package로 유지하고 내부 runtime/offline 경계만 정리 |
| `src/learning` | README 1개뿐이던 빈 논리 폴더 | P4-C6에서 문서 이관 후 삭제 완료 |
| `src/learning_by_cheating` | runtime·구 3-channel 학습·teacher BEV가 혼재했던 비-catkin 폴더 | 사용 중인 7-channel teacher BEV만 `tools/research/lbc_teacher_bev` 이관, 나머지는 Git 이력으로 보존 |
| `src/lidar+sensorfusion` 내부 `build/`, `devel/` | 중첩 workspace 산출물 | source 이식 확인 후 로컬 산출물 제거·ignore 강화 |

`learning_by_cheating`은 production runtime이 아니라 privileged simulator state로 GT/teacher
BEV를 만드는 연구 도구였다. 실제 사용 중이던 renderer·scenario visualizer·collector·진단만
`tools/research/lbc_teacher_bev`로 이관했다. 7-channel collector와 호환되지 않던
3-channel PA 학습기, 미검증 제어 prototype와 빈 파일은 Git 이력으로만 보존한다.
비어 있던 `src/learning`도 P4-C6에서 제거했다.

## 3. 목표 디렉터리

```text
src/
├── asmc_msgs/                 # 팀 내부 message만
├── asmc_bridge/
│   ├── src/asmc_bridge/       # network, sensor, vehicle_state
│   ├── config/profiles/       # local, lan2lan
│   ├── launch/
│   └── test/
├── asmc_hd_map/
│   ├── src/asmc_hd_map/       # build, geometry, runtime crop, validation
│   ├── config/kcity2025/      # curation + competition metadata
│   ├── maps/kcity2025/        # 작고 versioned인 최종 bundle/manifest
│   ├── launch/ · rviz/
│   └── test/
├── asmc_camera/              # 2026-10-06 rename(구 asmc_perception)
│   ├── src/asmc_camera/      # camera objects/YOLO, lane/road, traffic_light, bev
│   ├── config/ · launch/ · rviz/
│   ├── scripts/               # ROS node entrypoint만
│   └── test/
├── asmc_lidar/                # LiDAR+IMU, 지면/구조물/군집/OBB/추적, camera fusion·좌표 어댑터
├── behavior_planner/          # 판제 소유, 현 위치 유지
├── frenet_planner/            # 판제 소유, 현 위치 유지
├── asmc_controller/            # 판제 소유, 현 위치 유지
├── jamming_lane_boundary_adapter/ # 판제 소유 음영 차선 경계 adapter, 이번 이전 대상 아님
└── MORAI-ROS_morai_msgs/      # pinned external dependency

tools/
├── perception/                # train/eval/dataset/bake/analysis
├── hd_map/                    # 대용량 원본 검사·one-shot migration
└── field/                     # 현장 profile 검사·packet audit

vendor/                        # 소스 설치가 꼭 필요한 upstream만 격리
scripts/
└── udp_bridge.sh             # 별도 foreground 실행·Hz 모니터링
# 카메라 runtime은 컨테이너 내 roslaunch로 직접 실행
```

목표 `asmc_camera` 하나에 모든 연구 파일을 넣는다는 뜻은 아니다. `src/`에는 대회 runtime만
두고 학습과 분석은 `tools/perception`에서 같은 Python library를 import한다. 이 구분이 있어야
catkin package 하나를 유지하면서도 실행 이미지가 학습 산출물로 비대해지지 않는다.

## 4. 개발 단계

| 단계 | 목표 | 주요 산출물 | 삭제 허용 |
|---|---|---|---|
| **1** | 계약·메타데이터 동결과 이동 안전망 | `asmc_msgs` v1, topic contract, 체크포인트/미션 schema, baseline manifest/tests | 없음 |
| **2** | `asmc_hd_map` 제품화 | 최종 lane/lane-area/drivable bundle, global/local publisher, checkpoint·mission RViz | `link_node`는 parity 통과 뒤 가능 |
| **3** | `asmc_bridge` 통합 | UDP·sensor·vehicle state 단일 package, dev/competition profile | 구 bridge는 field replay 통과 뒤 가능 |
| **4** | `asmc_perception` 통합 | camera/traffic-light/BEV runtime, offline tool 분리 | `learning*` 정리; LiDAR는 별도 패키지 |
| **5** | 실행·시각화 정리 | 유지 가능한 실행 진입점, RViz/rqt, runtime 수락검증과 consumer handoff | 호환 wrapper 만료 뒤 가능 |
| **6** | 현장 LAN2LAN·Docker 동결 | competition image, network profile, rehearsal report, 당일 runbook | 본선 직전 기능 삭제 금지 |

각 단계는 별도 PR로 진행한다. rename과 동작 변경을 같은 PR에 넣지 않으며, 이전 경로를
지우는 PR은 새 경로의 bag replay·launch smoke 결과를 첨부한 뒤 연다.

### 2026-10-05 이후 역할과 우선순위

LiDAR 인계는 [현재 골격과 이전 계획](../packages/asmc_lidar/README.md)을 따른다.
개인 연구 실행기와 알고리즘은 삭제했고 좌표·시간 코어·IMU 선택·공용 메시지를 남겼다.
손재호가 기존 기능을 이전하며 GPS·미션에 따른 검출 모드 전환을 제거하고 local/map
어댑터를 연결한다. 이후 구조물·군집·추적 정확도를 단계별로 개선한다.
기존 nested Main/Msg 및 운영 출력은 새 계약이 판제와 검증되기 전까지 유지한다.

사용자 측 독립 작업의 순서는 **개정 규정의 공식 근거 확인과 HD맵 갱신 → 카메라 객체
검출의 asmc_camera 이전(rename은 2026-10-06 완료) 및 LiDAR/음영 어댑터 공통 소비 계약 → 동시 실행 부하
기준선 → YOLO 선택과 BDD100K 다음 기존 데이터로 TwinLiteNet+ 학습 비교 →
LAN2LAN 리허설**이다. COCO의 person/car/truck/bus는 현재 호환 기준이며 클래스
확장은 소비자별 시험 뒤 결정한다. BDD100K 라벨을 기존 white/yellow/stopline head에
강제로 맞추지 않는다. bridge는 인지 실행기에 합치지 않고 별도 foreground Hz 로그를
유지한다. LAN2LAN은 모든 연구 개선을 기다리지 않고 검증된 기준선으로 시험 가능하다.

## 5. 1단계 상세 계획 — 계약·메타데이터 동결

### 5.1 목표

1단계는 폴더를 대량 이동하지 않는다. 네 정본 패키지가 공유할 **메시지, 토픽, 정적 map
metadata, 검증 기준**을 먼저 고정한다. 이 단계가 끝나면 이후 작업은 파일 위치가 아니라
동일한 계약을 기준으로 병렬 이식할 수 있다.

### 5.2 작업 단위

#### A. 현재 runtime baseline 고정

**완료.** 기계 판독 manifest는
[`config/baselines/runtime_baseline_v1.yaml`](../../config/baselines/runtime_baseline_v1.yaml), 사람이
읽는 결과는 [P1-A runtime baseline](../records/baselines/runtime-v1.md), 재생성 도구는
`tools/audit/build_runtime_baseline.py`다.

- 현재 catkin package 24개, launch 51개와 node 선언 56개, package dependency, executable과
  정적 topic 참조를 추출했다.
- P1-A 당시 팀/runtime message 22개를 동결했고, P1-B의 additive 타입 11개를 포함하도록 현재
  manifest를 33개로 재생성했다. 기존 13개 `asmc_msgs` MD5는 P1-A 값과 동일하다.
- HD Map stage14의 build ID, file SHA와 geometry-only SHA를 남겼다. 기준은 lane marking
  121개(white 94, yellow 27, solid 90, broken 31), stopline 7개다.
- 같은 시나리오의 대표 raw/result bag에서 topic 이름·rate·message count·type·MD5를 ROS CLI
  없이 bag v2 index로 읽고, bag 자체는 복사하지 않은 채 `$ASMC_DATA` 상대 경로·크기·SHA만
  저장했다.
- Autonav v2 센서의 현재 기준은 VLP16 10 Hz다. 손재호 LiDAR runtime을 위한 15 Hz 변경은
  3단계 승인 요구사항으로 따로 기록했으며 아직 센서 JSON을 바꾸지 않았다.
- planning 소유 경로, 센서·network config, HD Map geometry에는 변경을 만들지 않았다.

#### B. `asmc_msgs` v1 동결

**완료.** P1-B에서 기존 13개 메시지의 ROS1 MD5를 유지하고 HD Map·competition 타입
11개를 추가했다. P2-D에서 현재 차로 상태용 `LaneAssociation`을 additive policy로 더해
P4-B의 perception additive 3개를 포함한 현재 `asmc_msgs` 1.0.0은 총 28개다. HD Map 기계 판독 정본은
[`config/contracts/hd_map_topic_contract_v1.yaml`](../../config/contracts/hd_map_topic_contract_v1.yaml),
perception 정본은
[`config/contracts/perception_runtime_topic_contract_v1.yaml`](../../config/contracts/perception_runtime_topic_contract_v1.yaml)이며,
전체 소유 관계는 `asmc_msgs_contract_registry_v1.yaml`이 검사한다. 사람이 읽는 규약은
[HD Map topic contract v1](../contracts/hd-map/topic-v1.md)과
[Perception runtime topic contract v1](../contracts/perception/runtime-v1.md)이다.

| 메시지 계열 | 필수 정보 |
|---|---|
| `HdMapMeta` | schema/build/map ID, frame, source hash, 생성 시각, global/local 여부 |
| `LaneMarking` / `LaneMarkingArray` | geometry, white/yellow, solid/broken, 폭, physical/logical 역할, source provenance |
| `Lane` / `LaneArray` | lane ID·차로 번호, centerline·polygon, left/right boundary ID, adjacency, 진행 방향 |
| `LaneAssociation` | 후륜축 중심 현재 차로, 횡오차·경계 여유·전환·stale 상태 |
| `DrivableArea` / `DrivableAreaArray` | polygon ID, lane/junction 여부, source, validity |
| `CompetitionCheckpoint` / `...Array` | 순서, pose, 공식 yaw, 3 m 반경, route station, link ID |
| `MissionZone` / `MissionZoneArray` | mission type, start/end station·link, official/provisional 상태, 규칙 payload |

래스터는 새 custom message를 만들지 않고 `nav_msgs/OccupancyGrid`를 쓴다. 정적 geometry는
latched publish하고 local crop은 ego pose가 들어올 때 갱신한다.

동결한 토픽은 다음과 같다.

```text
/hd_map/meta
/hd_map/global/lane_markings
/hd_map/global/lanes
/hd_map/global/drivable_areas
/hd_map/global/drivable_grid
/hd_map/global/checkpoints
/hd_map/global/mission_zones
/hd_map/local/lanes
/hd_map/local/drivable_grid
/hd_map/local/current_lane
/hd_map/debug/markers
```

메시지에 `lane_number`를 두되 도로 전체에서 고정된 숫자로 가정하지 않는다. 같은 진행 방향의
단면별 왼쪽→오른쪽 순서와 유효 station interval을 함께 저장해 합류·분기에서 번호가 바뀌는
문제를 피한다. 차선 도색(`LaneMarking`)과 통행 가능한 차로 영역(`Lane`)도 분리한다.

`tools/audit/validate_p1b_contract.py`는 ROS 설치 없이 기존/신규 message MD5, 필수 필드,
CMake 등록, package version, topic latch/scope를 검사한다. ROS Noetic build 후에는
`src/asmc_msgs/test/test_message_serialization.py`로 generated type의 MD5와
serialize/deserialize 왕복을 검사한다.
P1-B 자체에서는 publisher와 geometry를 추가하지 않았다. 이후 P1-C metadata, P1-D publisher와
P2-D `LaneAssociation`까지 완료됐으며 wire schema의 현재값은 contract YAML을 따른다.

#### C. 체크포인트 정본화

**완료.** source 설정은
[`kcity2025_competition_metadata_source.yaml`](../../src/asmc_hd_map/config/kcity2025_competition_metadata_source.yaml),
생성 bundle은
[`kcity2025_competition_metadata.yaml`](../../src/asmc_hd_map/config/kcity2025_competition_metadata.yaml),
사람이 읽는 정본은
[K-City competition metadata v1](../contracts/competition/kcity2025-metadata-v1.md)이다. 공식
checkpoint/path, current link set, reviewed route decomposition과 stage14 map SHA를 모두 고정하며
입력이 바뀌면 `--write`도 실패한다.

공식 입력은
`$ASMC_DATA/R_KR_PG_K-city_2025/2026_molit_comp_checkpoints.json`으로 두고, runtime bundle에는
source SHA와 필요한 필드만 복제한다. organizer yaw는 degree 원문을 보존하고 quaternion은
publish 시 생성한다. START/END는 폐곡선의 station `0`과 `2184.61 m`가 같은 위치인 특수점으로
취급한다.

현재 공식 path와 2025 link centerline에 투영한 결과는 아래와 같다. 모든 checkpoint의 공식
좌표는 global path 위에 있으며 XY 투영 오차는 표시 정밀도에서 `0.000 m`다.

| CP | route s (m) | current link | link s (m) |
|---:|---:|---|---:|
| START/END | 0.00 | `A2256W000748` | 0.00 |
| 1 | 72.63 | `A2256W000748` | 72.63 |
| 2 | 186.29 | `A2256W000236` | 26.28 |
| 3 | 237.42 | `A2256W000215` | 28.96 |
| 4 | 346.74 | `A2256W000222` | 30.84 |
| 5 | 471.09 | `A2256W000205` | 0.00 |
| 6 | 580.87 | `A2256W000318` | 30.25 |
| 7 | 635.11 | `A2256W000083` | 25.24 |
| 8 | 858.50 | `A2256W000304` | 223.39 |
| 9 | 930.60 | `A2256W000866` | 15.76 |
| 10 | 1118.74 | `A2256W000846` | 134.03 |
| 11 | 1286.15 | `A2256W000411` | 167.41 |
| 12 | 1593.59 | `A2256W000445` | 75.34 |
| 13 | 1741.72 | `A2256W000153` | 148.13 |
| 14 | 1859.64 | `A2256W000446` | 78.08 |
| 15 | 1996.73 | `A2256W000126` | 118.99 |

검증기는 총 16개, 순서 중복 없음, 반경 3.0 m, path 투영 오차, link 존재, yaw 유한값을
검사한다. 디버그 marker는 반경 3 m 원/원통, heading 화살표, ID·station text를 표시한다.

#### D. 미션 구간의 확정도 분리

규정 문장, 공식 그림, 팀 map-review 해석을 같은 등급으로 저장하지 않는다.

| 구간 | 현재 경계 | 상태 |
|---|---|---|
| 출발 성공 | 전체 2184.61 m의 5%인 `s=109.23 m`, `A2256W000329` 내부 | 공식 규칙에서 계산 가능 |
| 기본 속도 제한 | `A748` 시작→`A846` 종료 및 `A451` 시작→`A751` 종료 60 kph | 공식 규칙을 link 경계에 투영 |
| 고주로 속도 예외 | `A2256W000411` 시작 `s=1118.74` → `A2256W000153` 끝 `s=1741.72` | 공식 link 경계. 별도 제한값은 규정에 없으므로 `null/exempt` |
| GPS 음영 | `A2256W000448` 다음 노드부터 `A2256W000126` 중심 구간 후보 | 사용자 map review·기존 config와 일치, 공식 수치 경계 재확인 필요 |
| 정적 장애물 | `A2256W000213` 종료 뒤부터 `A2256W000220` 시작 전까지의 `A2256W000222` 교차로 영역 | 팀 질의응답·상황 검토에 따른 비공식 경계 |
| 동적 장애물 | `A2256W000728→599→207→205` | 팀 질의응답·상황 검토에 따른 비공식 경계 |
| 회전교차로 | `A2256W000148→146→151` 주변 후보 | 공식 그림을 link에 매핑한 후보 |
| 로터리 후 고주로 이전 | `A2256W000866→054→846` | 별도 미션 없음, 전역경로 추종 및 60 kph 구간 |
| 주변 차량 인지 | `A2256W000411→420→408→445→153` 후보 | 공식 그림과 속도 예외 corridor가 일치 |
| 신호등 | `A2256W000236→…→A2256W000308` 인지 활성화 + 검수 정지선 기반 경계 5개 | 활성화 policy와 정지 경계를 분리하며 구간 밖 노드 중지 가능, 공식 수치 경계 재확인 필요 |

후보 구간은 RViz에서 공식 그림, checkpoint, link ID를 한 화면에 놓고 사람이 승인하기 전까지
`provisional`로 publish한다. 판제가 `provisional` mission을 강제 규칙으로 소비하지 않도록
메시지에 상태를 필수 필드로 둔다.

P1-C bundle에는 A236→A308 신호등 인지 활성화 policy를 포함한 authored zone 10개와
traffic-light ID가 있는 검수 정지선에서 파생한 신호등 경계 5개, 총 15개가 들어간다. 상태는
`derived_official=4`, `provisional=11`이며
모든 provisional zone은 `planning_enforcement_allowed=false`다. 신호등과 팀 link 해석 후보는
P1-D에서 표시하지만 사람 승인 전 판제 강제 규칙으로 사용할 수 없다.

2026-10-05 규정집 v1.2 반영 때 11개 후보 경계를 사람이 검수해 팀 기준값 `team_final`로 올렸다(현재
`derived_official=4`, `team_final=11`). 메시지는 동결 상태라 `STATUS_DERIVED_OFFICIAL`과 payload
`authority=team_final`로 전달한다. 판제 연동 결과에 따라 source를 고치고 `rebuild_hd_map_artifacts.py`로 다시 만든다.

#### E. 1단계 테스트와 완료 조건

- `asmc_msgs` 단독 catkin build와 기존 consumer build가 통과한다.
- 기존 message MD5와 topic은 1단계에서 바뀌지 않는다.
- 16개 checkpoint가 global path 3 m corridor 안에 있고 route order가 단조 증가한다.
- `START_END` 폐곡선 처리와 5% station 계산 단위 테스트가 있다.
- mission zone은 `official`, `derived_official`, `provisional` 중 하나가 없으면 schema validation에 실패한다.
- HD Map baseline hash/count와 대표 topic manifest가 CI artifact로 생성된다.
- planning 소유 파일 diff가 0이다.

P1-A 자체 완료 조건인 manifest/report byte-for-byte 재생성 검증은
`python3 tools/audit/build_runtime_baseline.py --check`로 통과했다. ROS Noetic 컨테이너에서는
당시 `asmc_msgs` 24종 생성, 기존 13종의 실제 `rosmsg md5`, generated-message serialization과
P2-A workspace 회귀, source 비교와 local crop 계약을 포함해 `asmc_hd_map` 전체 106개
테스트를 통과했다. 이후 additive `LaneAssociation`이 추가돼 현재 contract validator 기준은
25종/11 topic이며, 현재 ROS 비의존 `asmc_hd_map` discovery는 118개다. 날짜가 다른 두
검증 횟수를 같은 baseline 수치로 덮어쓰지 않는다.

규정집 v1.2(2026-09-30)에서 checkpoint는 15개로 줄었다(구 11 삭제, 12~15 재번호). 아래 16개 수치는
P1-C·P1-D 당시 검증 기록이며 현재 기준은 [competition metadata](../contracts/competition/kcity2025-metadata-v1.md)를 따른다.

P1-C 기준으로 checkpoint 16개의 route/link XY 투영 오차는 모두 `0.0 m`, route station은
단조 증가하며 START/END station 0과 5% station `109.230586167 m` 단위 테스트가 통과했다.
P1-D publisher를 실제 ROS master에 실행한 뒤 늦게 접속한 validator가 latched 전역 토픽과
1 Hz non-latched MarkerArray를 구독했다. marking 128개, checkpoint 16개, mission/policy/speed zone
15개, debug marker 205개와 geometry SHA가 모두 기준값과 일치했다. 속도 profile의 전 구간
coverage와 A054·A846 미션 제외도 runtime validator가 확인한다.

### 5.3 1단계 PR 분할

1. **P1-A audit (완료):** package/topic/message/sensor/bag/HD Map baseline manifest와 dependency report
2. **P1-B interface (완료):** `asmc_msgs` HD Map·competition metadata message, frozen topic contract와 MD5/serialization test
3. **P1-C metadata (완료):** checkpoint source import·projection validator·권위별 mission-zone bundle
4. **P1-D debug (완료):** SHA-gated read-only publisher, runtime topic validator와 RViz preset

P1-C/P1-D와 P2-A/P2-B 구현은 `asmc_hd_map` 한 패키지로 이동했다. 최종 차선, metadata,
compact vector/grid publisher의 토픽 계약은 유지하며 패키지 이동 전후 geometry parity를
회귀 검사한다.

## 6. 2단계 이후 구현 메모

### 6.1 `asmc_hd_map`과 주행가능영역

- **P2-A 완료:**
  [`kcity2025_drivable_area_input_audit.yaml`](../../src/asmc_hd_map/config/kcity2025_drivable_area_input_audit.yaml)에
  39개 전역경로 링크의 lane-clip 후보와 `intersection_guarded` 보호구간을 동결했다. 기존 H5
  road는 진짜 vector road mesh가 아니라 Node/Link 중심선 buffer에 실측 보정을 더한 raster임을
  명시했으며, 16개 checkpoint 중심 포함·폐곡선 topology·입력 SHA·stage14 geometry hash를
  검증했다. 이 단계에서는 geometry와 publisher를 변경하지 않았다.
- **P2-B v5 완료:** 전역경로 39개, 같은 방향 usable 차로, 명시 장애물 link, A134 로터리와
  음영구간 반대차로 A131을 선택한다. 70개 Lane polygon은 endpoint node 법선의 flat cap으로 끝나며,
  41개 Lane/74개 측면은 신뢰 가능한 reviewed paint 횡거리로 보정한다. MGeo 인접차로와
  A126/A131 사이에 vector corridor fill 18개를 추가한다. 정적·동적 교차로는 선택 link
  envelope 안에서만 legacy H5 road를 gap patch로 사용하여 단일 연결·내부 hole 0개를
  보장한다. A134는 2 m 확장 envelope와 A151 접속부에서 원본 H5 포장면을 복원하고 중앙
  섬을 제외한다. 총 DrivableArea는 102개이며 `OccupancyGrid`는 0.2 m 파생 raster다. 면적은
  14,925.40 m²(legacy 전체의 16.61%)이고 checkpoint 16개를 모두 포함한다. A179·A427과
  명시적 갓길·버스·미사용 toll lane은 제외한다.
- **P2-B v6 (2026-10-05):** 정적 장애물 교차로의 우회전 `A226`·`A111` 모서리 안쪽을 최대 1.0 m·1.5 m
  half-sine으로 넓혀 아스팔트 모서리를 채운다. 가장 바깥 남행 `A1212`는 회피에 쓰지 않으므로 넣지 않는다.
  Lane 70개, DrivableArea 102개(fill 18, H5 patch 14)이며 checkpoint 15개를 모두 포함한다.
- **P2-C 통합 완료:** stage14 차선 결과를 geometry regression 기준으로 삼아 builder,
  runtime publisher, 검증기와 RViz를 `asmc_hd_map` 한 패키지로 통합했다. 원태의 이전
  standalone 도구/H5는 `development/won_tae_link_node/`에 read-only provenance 자료로
  격리했고 top-level package는 제거했다.
- **P2-D consumer handoff 완료:** global cell에 정렬된 120 m/0.2 m local grid와 같은 범위의
  Lane subset을 ego update마다 최대 5 Hz로 발행한다. local geometry와 ID는 global `map`
  namespace를 유지하고, 실제 ROS smoke에서 global/local lane 선택·grid cell parity를 검사한다.
- 최종 검토 차선·구 맵 origin 정합 차선·원태 H5 vector와 최신/구 주행가능영역을 각각
  독립 토픽으로 보는 `hd_map_source_comparison.launch`를 개발 전용으로 유지한다. 운영
  `/hd_map/global/*`과 섞지 않는다.
- 주행가능영역 원본은 Node/Link road mesh를 사용하되, 양쪽 물리 차선이 신뢰 가능한 일반
  road segment만 차선 안쪽으로 clip한다.
- junction/교차로, boundary가 없는 구간, 합류·분기 전이부에는 `intersection_guard`를 두어
  road mesh를 유지한다. 차선이 없다는 이유로 교차로를 비주행 영역으로 잘라내지 않는다.
- vector polygon과 global/local `OccupancyGrid`를 함께 제공한다.
- 검증 항목은 route 연결성, checkpoint 3 m disk 포함, 교차로 면적 보존, self-intersection,
  lane 번호/adjacency, 기존 차선 geometry hash다.
- 원태 H5는 현재 compact 교차로 gap patch 재생성의 provenance 입력이다. 최종 제품은 이를
  런타임에 읽지 않으며, 별도 artifact 보관 정책과 재생성 parity가 정해지기 전까지
  `development/won_tae_link_node`를 삭제하지 않는다.

### 6.2 `asmc_bridge`

현재 구현·입출력·timestamp/TF와 이관 검증의 상세 인벤토리는
[MORAI bridge 개발·통합 가이드](../packages/asmc_bridge/README.md)를 따른다. 이 절은 목표
아키텍처이며 현재 `asmc_bridge` package나 환경 profile이 이미 있다는 뜻이 아니다.

- UDP network/sensor decode와 `vehicle_state` 정규화를 한 패키지에 둔다.
- 공통 runtime protocol `morai_udp`와 simulator release, transport profile을 분리한다. 예:
  `morai_udp + molitcomp03 + lan2lan`, `morai_udp + 26r1 + local`.
- 외부 `morai_msgs`는 wire adapter에서만 보이고, 내부 node는 가능한 한 `asmc_msgs`를 쓴다.
- privileged Object/GT 토픽은 개발 profile에서만 허용하며 competition launch에서 시작하지 않는다.
- 개발과 현장의 **포트와 채널은 동일하게** 만들고 IP만 profile로 바꾼다. 목표 포트는 Ctrl
  `9093/9094`(mode 2), MolitComp Competition Vehicle Status `9797/9798`, CollisionData `9091/9092`,
  v2/v3 sensor `9290~9303`이다. Ctrl `9111/9112`는 P3-A에서 archive로 격리하고 기본값을
  `9093/9094`로 전환했다. Ego Vehicle Status는 release별 PUBSUB 차이를 유지하면서 포트를
  `9088/9099`로 통일했다. P3-B 당시 Competition Vehicle Status를 `7603/7502`로 해석했으나
  P3-E live/enum 대조에서 type 258 TrafficLight status임을 확인했다. 실제 MolitComp
  Competition Vehicle Status는 type 787, `9797/9798`이며 참가팀 local에서는 선택 사항이다.
  정적 decoder·fixture와 ROS topic은 유지하고 주최 현장 수락에서만 explicit gate로 요구한다.
- 현행 본선 sensor 정본은 v3 설정 15 Hz다. MORAI UI 저장본의 `sensorPeriod`와
  `frequencyIndex`, 손재호 코드의 소비 loop 15 Hz를 확인했다. 실제 완성 scan은 약 12 Hz여도 허용하며,
  packet/parse 오류, latency/CPU와 tracking 품질을 계속 관찰한다. 없는 scan을 복제하지 않는다.
- planning consumer launch는 P3-F에서 `asmc_bridge` component launch를 include하도록 전환했다.
  이후 launch 계약 변경에는 판제 리뷰를 계속 요구한다.

### 6.3 `asmc_perception`과 `learning*` 정리

현재 package와 문서·runtime 경계의 상세 인벤토리는
[Perception 개발·통합 가이드](../packages/asmc_perception/README.md)를 따른다. P4-C6에서 camera
semantic, metric BEV/lane family, traffic light의 runtime·offline·training을
`asmc_perception`으로 물리 통합했고 구 package를 제거했다. topic과 node 이름은
호환성을 위해 유지한다.

- **완료:** camera semantic, metric BEV/lane family, traffic light를 하위 Python
  namespace와 하나의 ROS package로 이식했다.
- model weight, manifest/SHA/config의 현재 배치는
  [`asmc_camera` README](../../src/asmc_camera/README.md)를 따른다.
- **완료:** train/eval adapter는 `asmc_perception/training`, offline BEV audit은
  `asmc_perception/offline`으로 구분해 이식했다.
- **P4-D 보류:** `src/lidar+sensorfusion/`은 팀원의 미완성 nested workspace로 유지한다.
  handoff 전에 `Msg/TrackedObject*`, 중복 `morai_msgs`, vendor driver를 임의로 옮기거나
  삭제하지 않는다.
- **완료:** LBC teacher BEV는 `tools/research/lbc_teacher_bev`로 경계를 옮기고
  scenario runner·문서·진단 참조를 갱신했다. `src/learning*`는 제거했고 구 코드는
  별도 `legacy/` 복사 없이 Git 이력으로만 보존한다.

### 6.4 스크립트와 시각화

실행 명령의 현재 정본은 [실행 명령어 모음](../runtime-commands.md)이다. Phase 5는
모든 process를 하나의 스크립트로 숨기지 않는다. UDP bridge는 별도 foreground
terminal에서 log·Hz·diagnostics를 계속 볼 수 있게 `udp_bridge.sh`를 단독 진입점으로
유지한다. 통합 편의 명령은 인지 runtime과 debug UI 정도로 한정한다.

초기 안의 예시는 아래였으며, 구현 전에 필요한 부분만 채택한다.

```bash
./scripts/asmc build
./scripts/asmc sim up --profile local
./scripts/asmc bridge run --profile lan2lan
./scripts/asmc map run
./scripts/asmc perception run
./scripts/asmc debug rviz
./scripts/asmc verify field
```

Phase 5의 우선순위는 wrapper 수를 늘리는 것이 아니라 perception 출력과 시각화를
줄이는 것이다.

**P5-A 완료:** [`perception_topic_policy_v1.yaml`](../../config/contracts/perception_topic_policy_v1.yaml)과
[감사 기록](../records/baselines/perception-p5a-topic-audit-v1.md)에 24 family, 펼친 63 topic의
publisher·runtime subscriber·bag profile을 동결했다. `model_grid`와 per-view BEV 3종은 각각
lane vectorizer와 fusion의 필수 내부 transport라서 이름만 보고 끌 수 없다. runtime 동작은
바꾸지 않았으며 P5-B는 per-view coverage, source-view, frame-level pipeline audit의 opt-in
전환과 단일 BEV 비교 화면부터 진행한다.

**P5-B 완료:** 일반 `start_visualization:=true` 경로는 9칸 camera semantic
mosaic와 원본 RGB/fused semantic BEV 좌우 비교 화면만 광고한다. 개별 camera panel,
기존 fused/fused-quality와 lane 중간 영상은 `visualization_legacy_rqt:=true`, camera
calibration grid는 `visualization_calibration_grid:=true`에서만 생성한다. 두 기본 영상도
subscriber가 없으면 렌더링·JPEG 직렬화를 하지 않는다. live MORAI에서 BEV 비교 영상
`1344x1050 @ 5.0 Hz`를 확인했으며 세부 근거는
[P5-B rqt 기준 기록](../records/baselines/perception-p5b-rqt-v1.md)에 있다. 이 작업은
RViz marker 구성과 planning 소유 adapter를 바꾸지 않았다. 이어서 per-view coverage와
fused source count/view를 `publish_bev_debug_metadata` opt-in으로 전환했다. frame-level
pipeline audit도 `lane_publish_pipeline_audit` opt-in으로 전환했고 result-bag batch가 이를
명시적으로 다시 켠다. 다음 묶음은 기본 runtime diagnostics·topic health를 유지하면서
중복 compatibility output의 실제 consumer 만료 여부를 검토한다.

**P5-C runtime acceptance 완료:** 기존 topic 이름은 변경하지 않는다. 기계 정본
[`perception_runtime_acceptance_v1.yaml`](../../config/contracts/perception_runtime_acceptance_v1.yaml)은
기본 camera→BEV→lane 33개 토픽과 선택적 traffic-light 3개, rqt 2개를 node/type/유일
publisher/필수 subscriber/rate/stamp/frame 기준으로 검사한다. 판제 실행 시에는
`behavior_planner`가 typed `/perception/traffic_light/detection`과 호환
`/traffic_light/state`를 모두 구독하는지도 별도 gate로 검사한다. shared-family closeout 뒤
기본-OFF 20개 토픽이
실수로 다시 광고되면 실패한다. source·launch·config 경로 anchor도 함께 검사해 package 이동 뒤
연결이 조용히 깨지는 것을 막는다. LiDAR canonical topic은 담당 구현 handoff 전이므로 여기서
필수화하지 않는다.

**P5 runtime·shared-family closeout 완료:** live와 bag replay ROS 추론은 모두
`asmc-ros-noetic`에서 직접 `roslaunch asmc_camera perception.launch`로 실행한다.
`scripts/perception.sh`는 혼선을 막기 위해 제거했고 `asmc-perception-train`은 모델 학습·검증에만
사용하며 host ROS network에서 격리한다. 음영구간 차선은 typed
`/perception/lane/boundary_family`와 full-support marker를 기본 발행하고, 기존 개별
`boundary_observations` pipeline은 명시적 호환 옵션으로만 남긴다. adapter launch는
`base_link` RViz를 함께 열어 full support와 판제 수락 구간인 0~15 m strip을 동시에 보여준다.
이 strip은 검토용 MarkerArray이며 판제의 planning horizon이나 typed geometry를 제한하지 않는다.

**P5-D consumer handoff 완료:** 신호등 producer의 소유자는 `asmc_camera`(구 `asmc_perception`), 판제 정본 입력은
typed `/perception/traffic_light/detection`으로 고정했다. Behavior는 typed 메시지를 한 번 받은 뒤
invalid/stale 상태에서 legacy String으로 우회하지 않고 UNKNOWN으로 fail-closed한다. planning
통합 launch는 중복 publisher를 막기 위해 신호등 producer를 기본으로 시작하지 않으며,
`traffic_light:=true`는 독립 호환 시험에서만 사용한다. legacy String은 모든 소비자가 typed로
전환될 때까지 삭제하지 않는다.

음영구간 차선의 판제 정본은 `/perception/lane/boundary_family`다. 대표 result bag의 typed family
293개와 reference adapter 연결을 인수 근거로 고정했고, 과거 개별
`boundary_observations`는 기본-OFF 호환 출력으로 남겼다. adapter의 0~15 m strip Marker는 검수용일
뿐 planning 입력이나 family 유효거리 제한이 아니다. `lane_boundary_adapter` 흡수·삭제 시점은
판제 담당자가 결정한다.

**P5-E 운영 closeout 완료:** bridge는 packet/HZ를 사람이 계속 볼 수 있도록 별도 foreground
process로 유지한다. live·bag inference는 `asmc-ros-noetic`에서 직접 실행하고 학습은
`asmc-perception-train`에 격리한다. rqt 기본 화면은 camera mosaic와 BEV comparison 두 개,
lane 검수는 `base_link` RViz를 자동으로 여는 adapter launch, graph 수락은
`verify_perception.sh`로 고정했다. 상세 인수표와 증거는
[P5-D/E closeout](../records/baselines/perception-p5de-closeout-v1.md)에 있다.

**웹 콘솔 보류:** 현재 RViz/rqt와 구조화 diagnostics로 필요한 검수가 가능하므로 구현하지
않는다. 실제 원격 다중 사용자 수요가 생길 때 별도 제안과 threat model을 거쳐 연다.

1. consumer가 없는 grid·debug topic은 기본 발행을 끄고 opt-in으로 돌린다.
2. BEV debug는 중복 창 대신 원본/투영 BEV와 최종 인지·융합 BEV를 나란히
   보여주는 하나의 비교 화면으로 정리한다.
3. 의미가 같은 topic은 publisher·consumer·bag 감사 후 정본 하나를 선택하고, 즉시
   삭제가 아닌 호환 기간을 거친다.
4. HD Map·perception·topic health는 RViz/rqt와 runtime verifier에서 먼저 정리한다.
   read-only web console은 실제 운영 수요가 확인될 때까지 backlog로 둔다.

기존 30개 스크립트는 `container`, `runtime`, `data`, `train/eval`, `diagnostic`으로 분류한다.
package node는 각 package `scripts/`, offline·research job은 `tools/`에 둔다.
LBC teacher BEV는 `tools/research/lbc_teacher_bev`로 이관했고, 사용 중단된 direct rosbridge
경로만 호환 기간 뒤 삭제한다. 외부 참조가 확인된 기존 명령만 한 단계 동안 deprecation
warning을 내는 wrapper로 둔다. 팀 합의로 실행 경로를 단일화한 `scripts/perception.sh`는
wrapper를 유지하지 않고 제거했으며, 활성 문서는 ROS runtime container 직접 실행만 안내한다.

시각화는 먼저 RViz/rqt에 통합한다. raw PV, semantic result, metric BEV, lane fitting, HD Map
lane/area, checkpoint 3 m radius, active mission, topic rate/staleness를 한 launch로 확인한다.
웹 콘솔은 Phase 5 완료 조건이 아니다. 원격 다중 사용자 관찰 수요가 확인되면 별도 단계로
검토하며, 그때도 차량 제어·파라미터 변경은 포함하지 않는다.

Phase 5는 P5-A~P5-E까지 완료됐다. 다음 우선순위는 LAN2LAN 현장 리허설에 바로 들어가기보다
담당자와 함께 LiDAR/fusion
기준선을 먼저 계측하고 개선 범위를 합의하는 것이다. 정본 Working Proposal은
[LiDAR 인계 계획](../packages/asmc_lidar/README.md)이며, 지면 제거,
터널 벽·천장 억제, 거리 적응형 clustering, 추적 안정화와 HD Map 기반 고주로 차로 귀속을
우선 검토한다. 담당자가 개발 중인 nested workspace는 합의·baseline 없이 이동하거나
`asmc_camera`에 흡수하지 않는다. LAN2LAN 리허설은 이 통합의 안정 지점 뒤에 수행한다.

## 7. 현장 LAN2LAN·Docker 단계

### 7.1 반출 자료에서 확인된 사실

`$ASMC_DATA/On-site inspection`에는 다음이 있다.

- MORAI `25.S4.MolitComp03` simulator/launcher 전체본: 압축 약 2.7 GB, 해제본 약 3.2 GB
- launcher version: `v.S3.20260714.95`
- 현장 network/sensor/scenario save file
- 시뮬 PC `192.168.0.1` → 참가팀 PC `192.168.0.10`인 LAN profile

현장 센서 폴더에는 다른 팀 설정도 섞여 있다. 우리 팀은 **Autonav**이며,
`0918sensors.json`을 Autonav 설정 근거로 사용하지 않는다. 현재 본선 실행 정본은
`config/morai/sensors/ASMC_sensor_set_v3.json`, v2는 10 Hz 회귀 비교용이고 현장
`Autonav_sensor_submitted.json`은 제출 이력 확인용이다.

두 Autonav 파일의 sensor 구성·주기·UDP 포트는 다음과 같이 동일하다.

| 입력 | 시뮬 host | 팀 PC destination | 주기 |
|---|---:|---:|---:|
| Camera 1/2/3/4 | 9290/9292/9294/9296 | 9291/9293/9295/9297 | 20/20/20/10 Hz |
| VLP16 | 9298 | 9299 | v3 설정·소비 15 Hz, 실제 완성 scan 약 12 Hz 허용 |
| GPS | 9300 | 9301 | 5 Hz |
| IMU | 9302 | 9303 | 50 Hz |

semantic 차이는 자유 카메라 4번 pitch가 v2/v3 `328°`, 제출본 `340°`인 한 항목뿐이다.
현재 실행 기준은 v3 `328°`를 유지한다.

현장 `2026_molit_comp_network.json`의 Ctrl profile에는 Host `192.168.0.1:9093`, Destination
`192.168.0.10:9094`가 들어 있고, Competition Vehicle Status `9797/9798`, TrafficLight status
`7603/7502`, Collision `9091/9092` 등 non-loopback port pair가 저장돼 있다. Ctrl은 팀 PC에서 시뮬 Host로 보내는
채널이므로 JSON 필드의 나열 순서를 패킷 진행 방향으로 해석하지 않는다. PUBSUB enum과 UI
이름은 실제 simulator 화면에서 다시 대조한다.

P3-A에서 개발 Ctrl도 `9093/9094`로 맞추고 `local`과 `lan2lan` network JSON을
release별 template에서 생성하도록 했다. 두 transport는 포트·mode가 같고 IP만 다르다.
Competition Vehicle Status는 참가팀 local에서 선택 사항으로 닫았고, 주최판 live capture 승인과
실제 두 PC 검증은 아직 남았다.
정적 decoder·fixture·ROS publish 계약은 P3-B, privileged stream allowlist와 v2/v3
sensor local/lan2lan IP 파생본은 P3-C에서 적용했다.
P3-D에서 `asmc_bridge` canonical launch façade와 기본 GPS/IMU `/vehicle/state` composition을
추가했다. P3-E에서 local sensor/state, Ctrl closed loop와 bag replay parity를 확인한 뒤 P3-F에서
기존 세 node package와 `integration_launch`를 `asmc_bridge` 하나로 물리 통합했다. node/topic/
diagnostics 이름은 소비자 호환을 위해 유지한다.

### 7.2 배포 원칙

- `local`과 `lan2lan`을 명시적으로 선택하며 자동 IP 추측을 금지한다.
- 3단계에서 생성한 두 profile은 포트·채널·Ctrl mode가 같고 Host/Destination IP만 달라야 한다.
- Windows 개발·현장점검 PC에서는 WSL mirrored networking과 Docker Desktop host networking을
  조합해 Windows LAN IP와 동일 UDP port가 WSL/container까지 유지되는지 검증한다. 최종 본선
  팀 PC가 native Linux라면 같은 port 계약으로 `network_mode: host`를 다시 검증한다. 어느 쪽도
  localhost 실측만으로 LAN ingress를 보장했다고 간주하지 않는다.
- NIC 고정 IP·netmask·route는 운영 요원 확인값을 사용하고 Wi-Fi/다중 route 간섭을 제거한다.
- 필요한 UDP만 host firewall에 허용하고, bind 충돌은 시작 전에 `ss`로 검사한다.
- 이미지 tag, image digest, git commit, map bundle hash, model hash, network profile hash를 한
  deployment manifest에 기록한다.
- 인터넷 없이 이미지 load→container start→topic health→주행 시작까지 복구 리허설한다.

### 7.3 현장 검증 게이트

1. 케이블 연결 후 양방향 IP reachability와 route 확인
2. 센서별 UDP packet count/size/rate 확인
3. bridge topic rate와 staleness 확인
4. Ctrl mode 2 명령의 목적지·응답 확인
5. Competition Vehicle Status와 CollisionData를 판정 연습 프로그램과 함께 확인
6. 15분 이상 full stack 주행, CPU/GPU/RAM/packet drop 기록
7. bridge 또는 perception node 단독 재시작 후 복구 확인
8. 네트워크 단절·시뮬 reset·checkpoint respawn 절차 확인
9. VLP16 설정·손재호 소비 loop 15 Hz와 `/velodyne_points` 실제 완성 scan률을 함께 기록하고,
   packet/parse 오류와 tracking 품질을 확인한다. 약 12 Hz 실제 입력은 허용한다.

결과가 통과하면 `operations/README.md`의 competition 절과 UDP 치트시트를 현장 profile로
갱신한다. 개발 profile의 localhost 값은 별도 절에 보존한다.

## 8. 삭제·완료 게이트

| 제거 대상 | 선행 조건 |
|---|---|
| `src/learning` | **P4-C6 제거 완료:** placeholder 문서 참조를 현재 경로로 이관 |
| `src/learning_by_cheating` | **제거 완료:** 필요 기능을 `tools/research/lbc_teacher_bev`로 이관하고 scenario/debug 참조 전환 |
| `src/asmc_hd_map/development/won_tae_link_node` | H5 provenance를 외부 immutable artifact로 보관하고 compact map 재생성 parity 통과 |
| `morai_udp_bridge`, `morai_sensor_bridge`, `vehicle_state`, `integration_launch` | P3-E local live·Ctrl·bag replay 통과 후 P3-F에서 제거 완료; LAN2LAN은 별도 Phase 6 gate |
| `camera_semantic_perception`, `drivable_bev`, `traffic_light` | **P4-C6 제거 완료:** 새 launch topic/latency/output parity와 model/config SHA 통과 |
| 중복 `Msg`, 중복 `morai_msgs` | 모든 consumer include/import 교체 및 clean build |
| 루트 legacy scripts | `scripts/asmc` 대응 명령·문서·CI가 존재하고 호환 기간 종료 |

삭제는 `git rm` 전에 대상 파일 목록과 대체 경로를 PR 본문에 적는다. 대용량 simulator,
rosbag, model checkpoint와 주최 배포 원본은 repo로 옮기지 않고 `$ASMC_DATA`에 유지한다.

## 9. 다음 착수점

**P1-A~P1-D, P2-A~P2-D, P3-A~P3-G local과 P4-A~P4-B는 완료됐다.** `asmc_hd_map`과
`asmc_bridge`는 각각 단일 package/runtime 계약을 소유한다. P4-A는 camera, LiDAR,
traffic-light, BEV, lane, fusion, recorder의 package/launch/topic/type/frame/timestamp/model SHA,
bag profile과 배포 금지 oracle을
[`p4a_perception_runtime_v1`](../records/baselines/perception-p4a-runtime-v1.md)로 동결했다.

P4-B에서 canonical `asmc_perception` package와 단일 compatibility launch, v3 camera calibration,
traffic-light/LiDAR additive message·topic 계약을 추가했다. 당시에는 기존 runtime source와
legacy topic을 이동·삭제하지 않고 canonical typed producer를 미연결 상태로 명시했다.

P4-C1에서 traffic-light가 실제 추론 image header를 typed 결과에 전달하도록 병행 발행하고,
당시 GPU container wrapper와 camera/BEV/lane debug 시각화 진입점을 추가했다. 이후 P5
closeout에서 wrapper는 제거하고 live ROS 추론을 `asmc-ros-noetic` 직접 실행으로 통일했다.
P4-C2는 대표 bag에서
기존/new String·typed parity, stale fail-safe와 입력 복구를 고정했다. P4-C3는 판제
`behavior_planner`를 typed 우선·legacy fallback으로 연결하고 invalid/stale fail-safe와 live ROS
연결을 검증했다. P4-C4는 production behavior node와 지도/route를 사용하는 격리 ROS loop에서
적색 STOP·녹색 새 KEEP을 검증하고 camera/BEV/lane ROS node 구현을 `asmc_perception`으로
물리 이식했다. 이어서 guarded MORAI 시험에서 Frenet/MPC/UDP 차량 제동·재출발과 원상 복구를
통과했다. P4-C5는 runtime camera/BEV/lane support와 config 정본을 `asmc_perception`으로 옮기고
기존 import/executable과 YAML 경로를 호환 전달자·SHA mirror로 남겼다. 동일 raw bag의
canonical/compatibility 출력은 source stamp별 payload mismatch 0으로 통과했다. P4-C6은
TwinLite 학습 adapter, BEV offline audit와 traffic-light model을 역할별 canonical 경계로 옮기고
`camera_semantic_perception`, `drivable_bev`, `traffic_light`, 미사용 `morai_3d_detection`과 빈
`learning` placeholder를 제거했다. 긴 실험 기록은 `src/asmc_camera/docs/`(당시 `src/asmc_perception/docs/`)에 원문으로 보존했다.

`data_collection`은 recorder, bag profile, replay, Capture와 dataset lifecycle을 소유하는 독립
package로 유지하며 `asmc_perception` 통합·삭제 대상에 포함하지 않는다.

다음 P4-D는 LiDAR 담당자 handoff 이후의 adapter/message 통합이다. `src/lidar+sensorfusion`은
담당 팀원이 개발 중인 독립 nested workspace이므로 P4-C6에서 파일,
package, 중복 `Msg`·`morai_msgs`를 이동하거나 삭제하지 않는다. LiDAR/fusion 구현이 완료되고
담당자 handoff와 대표 bag이 준비된 뒤에만 별도 P4-D gate로 canonical topic/message adapter를
연결한다. `asmc_msgs`는 이미 공용 메시지 정본이지만 nested LiDAR `Msg`를 즉시 대체한다는 뜻은
아니다. 중복 message 전환은 모든 LiDAR consumer include/import 교체, clean build와 bag parity가
동시에 통과해야 한다. `learning*` 삭제도 blocking reference zero 이후에만 수행한다.

실제 두 PC LAN2LAN은 팀원 측 코드 준비와 perception 통합, 최소 단일 실행 진입점 이후 Phase 6
리허설에서 수행한다. P3-G의 `--require-competition-status`는 그때 주최판 type 787 수락 gate로
사용하며 local 개발 완료를 막지 않는다.
