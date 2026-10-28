# LOCAL-1: GPS 독립 local 경로·MPC 실험

상태: 구현·검증 진행. 운영 기본 전환 및 터널 통과 수락 아님.

기준선: PR126 `1125582c`; 착수 시 main `3b0bc98f`. 기존 S3 r10/r11 실패를
보존한다. 목표는 semantic/family/local OBB와 속도·IMU의 공통 시각에서
경로를 생성하고 기존 mpc_core를 별도 local 입력 경계로 사용하는 것이다.
NORMAL 및 기존 음영 생산자는 수정하지 않는다. 이강태 1028 브랜치와 독립이다.

## 고정 조건

- 목표25km/h, 감속1.5m/s²에서 출발. 곡률·관측 종료·충돌·지연이 더 낮은 상한을 결정한다.
- YOLO bbox 차로 투영과 검출 즉시5km/h 규칙은 새 local 경로의 입력이 아니다.
- VehicleSpec 폭1.892m와 SDK bounding box 폭2.1608348m를 구분한다.
  bounding box가 충돌 collider와 같은지는 미검증이다. 물리 외곽 검증 전
  더 큰 박스를 실험 경계로 사용하며 일반 판제 치수는 바꾸지 않는다.
- nominal 지면0.36m, 후륜축 기준 전3.845/후0.790m, 축거3m.
- S3 근거리 미관측 보조는 명시적 비교 옵션이며 이때만5km/h 상한을 유지한다.
- 외부 GT는 평가 전용. GPS·map pose를 local 제어에 사용하지 않는다.
- 벽 RANSAC/출구 검출 추가 없음. 벽 소실은 출구 증명이 아니다.

## 참고 소스

외부 코드는 workspace external/baselines에 고정하고 ROS 실행 의존성으로 넣지 않는다.
- `sbpl`: `4d654845aa3b92aa1b4282a342e9011bac95aeb9`; BSD-3-Clause notice: `src/include/sbpl/headers.h` (별도 root LICENSE 없음)
- `commonroad-reactive-planner`: `6f2f0d91d4b9370f8e716349833308271365e7d1`; license files: ['LICENSE']
- `autoware_universe`: `bafa06574d97e6255bc7f5d8c36683574fa07ebb`; license files: ['LICENSE']

- `2026-HMAAC`: `dc942ed8dd8090738143f8451fb2819661c84668`; GPL-3.0.
  camera_perception_func_lib.py의 largest-component moments/near-row midpoint와
  image cubic-spline 경로를 개념 대조했다. 코드를 복사하지 않았다. 우리 후보는
  서로 떨어진 노면 구멍의 양쪽을 합친 무게중심을 목표로 만들지 않는다.

SBPL/state lattice의 실행 가능한 motion primitive와 search, CommonRoad의
후보 검증 분리, Autoware Path Sampler의 후보→제약→비용 구조를 참고한다.
현재 구현·수치 성능은 해당 논문의 결과와 구분한다.

## 재현 저장

새 run: `local1-20261028-audit01`, D artifacts. 사용자 명시 승인으로 이번
LOCAL-1은50GiB 미만 저장 허용, 전역 정책 변경 없음. D→G 이동 중인
SensorData는 수정하지 않는다. 실험별 ID를 새로 등록하며 기존 run은 동결 보존한다.

## 완료 조건

좌우 단일 장애물 각3회 통과·복귀, NORMAL 회귀 없음, 계획 p95≤50ms,
MPC20Hz/명령 최대공백≤250ms. 입력·경로·마진·속도·제어 실패를 각각
기록한다. 통과/치수/제동 검증 전에는 기본 활성화하지 않는다.

## 현재 구현과 실행 경계

`src/asmc_planner/src/local`은 후륜축부터 연결되는 quintic/lattice 후보,
실제 차체 sweep, semantic 확률·coverage와 local OBB 검사, 기존 SpeedProfiler를
조합한다. `src/asmc_controller/src/local`은 새 LocalTrajectory와 세션별 상대
odometry를 기존 mpc_core에 연결한다. 기존 map Trajectory/VehicleState의 GPS
검사를 완화하지 않았다. 새 메시지와 nav_msgs 의존성만 추가하며 기존 메시지
정의·MD5는 유지한다.

- family centre/right_outer 사이를 이번 터널의 **오른쪽 본차로**로 고정했다.
  가장 가까운 차로로 매 프레임 바꾸지 않는다. 생산자의 role 자체가 모든
  도로에서 우측통행 정책이라는 뜻은 아니다.
- 연결 곡선 뒤에도 family의 위치·접선·곡률을 따른다. 이전 구현의
  일정한 Y/yaw=0 연장과, 본차로 복귀보다 강했던 이전 경로 선호를 수정했다.
- NORMAL의 1.4초 MPC horizon은 3km/h에서 약1.2m만 본다. local 전용 YAML로
  2.4초(49×0.05s), steering-change cost25, heading cost6을 적용했다.
  공용 solver와 NORMAL controller.yaml은 그대로다. 이 값은 저속 검증 결과다.
- 카메라 자차 가림으로 엄격한 sweep이 실패하면 제한된 near-field assistance로
  **긴 sweep 경로를 먼저** 검사한다. 먼 장애물을 보는데도4m 회복 arc만
  만들던 지연 회피를 줄이기 위한 구조다. 알려진 OBB·비노면은 이 후보에서
  계속 검사한다. 실패 후의 짧은 semantic soft-cost 회복은 별도로 표시한다.
- fresh semantic 기반 회복은3km/h 상한이다. 상위 목표25를 달성했다고
  표시하지 않는다. soft 회복은 충돌 없음의 증명이 아니며 실제 접촉은 실패다.

### 짧은 입력 결합 공백

planner의 source freshness는350ms를 유지한다. 입력의 일시적인 결합 대기
(`scene_stale/no_complete_scene/local_objects_skew`) 때, **이미 MPC로 풀었던
회복 경로**를 같은 상대 frame에서 source age550ms까지 짧게 이어 쓴다.
현재 속도≤5km/h, 유효 상대 상태가 있어야 하며 원본 시각과 기존 jerk 제한
속도 프로필을 유지한다. 새 관측으로 재발행하지 않는다. 이는350ms freshness
연장이 아닌 별도 degraded 경로 재사용이며 trace의 `replan_grace_crawl`로
드러난다. 자세·세션·시간 역행·잘못된 형상/계약에는 적용하지 않는다.

## 주행·실패 근거

D `artifacts/local1-20261028-analysis02`의 comparison.json/index.html이 비교 정본이다.
각 run에는 입력 manifest, 등록 당시 dirty patch·미추적 소스, runtime 바이너리·
설정 SHA, 판단 JSONL,60초 분할 bag, native actor/접촉, 복구 결과를 보관했다.

| 자료 | 결과 | 해석 |
|---|---|---|
| drive07–11 | native throttle/brake/steer0 | network reload 전. 폐루프 성공 근거에서 제외 |
| drive12 빈 구간 |34.71m, 접촉0, 주행 중 강한 정지 명령24tick | 입력 지연이 제동으로 전달되던 기준선 |
| drive13 좌측 barrier |35.03m, 접촉0, 강한 정지22tick | 단1회; 좌우 각3회 수락을 대체하지 않음 |
| drive15 우측 barrier |10.67m에서 실제 접촉 | 실패. 늦은 짧은 회복 경로·RGB/OBB 기록 보존 |
| drive16 |35.06m, 접촉0, 강한 정지0, 본차로 횡오차 p95 55.7cm | 본차로 오버슛이 남은 수정 전 비교 |
| drive18 |35.00m, 접촉0, 강한 정지0, 횡오차 p95 5.2cm | local MPC 설정 후 빈 구간 |
| drive19 |31.71m, 접촉0, 강한 정지0, 횡오차 p95 19.2cm | 넓은 unknown OBB 혼입과 soft 회복20계획 |
| drive20 |35.07m, 접촉0, 강한 정지0, 횡오차 p95 5.5cm | 같은 설정 반복 |

횡오차는 family 근단의 본차로 중심 대비 **내부 일관성**이며 독립 GT 오차가
아니다. 매번 차량에서 경로를 새로 시작하면 MPC 경로 오차는 작아도 차로
우블링은 클 수 있으므로 별도 `analyze_local_following`으로 측정했다.
18/19/20의 계획 p95는28.9–33.8ms, MPC p95는4.7–15.0ms, 실제 제어 기록
최대 공백은61.3–72.8ms였다. 첫 출발 과도응답 최대속도3.6–3.8km/h도
그대로 남겼다. 전 터널 완주·고속 제동·차량 collider 검증 수락이 아니다.

### 가감속·자세 변화와 끊김

drive18에는 family683개가 모두 `accepted`, 노면 context460개가 모두
IMU 보정이었다. roll 절댓값 최대2.68°, pitch0.39°에서 자세 보정이 무효가
되어 끊겼다는 근거는 없었다. 원본 묶음의 도착 나이 p95는200.5ms,
최대255.9ms였다. 반면 새 묶음의 가장 가까운 LiDAR가51–53ms 차이여서
±50ms 결합에서 탈락하고 이전 묶음이350ms를 넘은 사례를 재현했다.
일부는 같은 source stamp의 family가 아직 callback에 들어오지 않은 경우다.
`input_join`은 최신3개 context의 원본 시각/age·격자/family 수신 여부와
가장 가까운 scan 차이를 기록한다. 자세와 시간 지연의 상관을 원인으로
단정하지 않는다. F1/3뷰 fusion의 ±50ms 자체는 바꾸지 않았다.

### 인지 OBB 혼입

drive19는 별도 장애물 actor가 없는 접근 구간에서도 unknown OBB들이
폭2.5–3.4m에 해당하는 길쭉한 footprint로 들어왔다. 이는 local 계획의
경로를 막는 **인지 입력 사례**로 별도 보존했다. 원본 point cloud가 없는
당시 자료만으로 벽/천장/가드레일 중 무엇인지 단정하지 않는다. drive20부터
원본 LiDAR·현재 trace와 자세/context diagnostics를 함께 녹화한다. planner가
임의로 이런 OBB를 지우거나 semantic 노면으로 덮어쓰지 않는다.

## 통합 웹·직접 확인

[실행 명령](../../operations/commands.md#local-1-실험-계산과-모니터)과
`http://localhost:8088/web/index_v2.html`을 사용한다. 별도 renderer를 중복
구현하지 않고 기존 V2 캔버스와 독립 local.html이 같은 renderer를 쓴다.
자동 보기 또는 `LOCAL-1 경로·노면·fusion`에서 노면·family·후보·선택 경로·
차체 sweep·정렬 OBB를 각각 켜고 끌 수 있다. 노면은 같은 source stamp만
제공하며 없으면 사유를 표시한다. YOLO는 원본 영상 bbox·클래스/그룹·시각을
보여주며 이를 노면에 다시 투영하지 않는다. 진단 추가 추론/영상 인코딩은 없다.

실제 무효 계획이 들어오면 최근 유효 기하를 같은 frame에서 최대1초 동안
회색으로만 표시하고, 현재 무효 사유를 계속 표시한다. 캐시가 무효를 정상
계획으로 만들지 않는다. 웹은 제어 명령을 발행하지 않는다.

## 다음 한 단위

빈 본차로 저속 추종을 기준선으로 고정하고, **우측 barrier 실패를 같은 입력으로
재현해 먼 연결 후보의 회피 시작·차체 여유·복귀를 검증**한다. 좌우 각3회와
실제 접촉0을 확인하기 전에는 LOCAL-1 전체 완료로 표시하지 않는다.
입력 결합 허용 범위 확대·동적 객체 시간 보상, unknown OBB 안정화와 벽/천장
분류는 각각 별도 근거가 필요하다. 출구 자동 판정과25km/h 승격도 남아 있다.

## 시간축·비용 추가 대조

drive20의 bag 수신 기준 p95는 family165.2ms, LiDAR10.8ms,
노면 묶음185.1ms였다. 같은 source stamp에서 family 뒤 semantic 묶음이
추가로 준비되는 간격은 중앙값14.7ms/p95 62.3ms였다. 이는 전체 카메라
추론·피팅·스케줄·전송을 포함한 지연이며 kernel 계산 시간만을 뜻하지 않는다.
로컬 정렬은 약0.2ms, 계획의 대부분은 약29ms의 sweep/후보 탐색이다.
계획 수신 공백의 주된 재현 원인은 임계 시각 결합이며, 단순히 코드 줄 수가
많아졌거나 IMU 보정이 실패했기 때문이라고 보지 않는다.

진단 `lidar_minus_family_source_ms`는 선택한 published header 간 차이다.
양수면 LiDAR 기준시각이 더 뒤이고, 음수면 family가 더 뒤다. 촬영 시각
또는 서로 다른 센서 source clock의 직접 차가 아니다. MessageEvent 수신
시각을 별도로 저장하여 다음을 구분했다.

- `family_source_to_receive_ms`, `lidar_source_to_receive_ms`: 각각의 header부터
  ROS 수신까지 나이. 수신 이후 callback 대기는 별도다.
- `lidar_minus_family_receive_ms`: 양수면 LiDAR 결과가 나중에 도착,
  음수면 family가 나중에 도착.
- `*_callback_queue_ms`: 수신 후 실제 callback 시작까지. 실시간 정차 예에서
  약0.05–0.1ms로, 주된100ms대 지연은 여기서 생기지 않았다.

수신 시간 부가 기록은 객체/경로 선택을 바꾸지 않으며32개로 제한한다.
메타데이터가 없으면 진단 unavailable만 표시하고 정상 계산을 중단하지 않는다.
같은 원본 입력을 쓰는 시간 정렬 라이브러리의 추가 분리보다, 먼저 카메라
producer 대기와 로컬 input join의 시간 정책을 검토하는 것이 우선이다.
F1 matching 임계값을 정확도 검증 없이 늘려서 문제를 감추지 않는다.

웹은 V2 grid row의 `min-height:auto` 때문에 사이드바 내용만큼 캔버스가
늘어나던 문제를 수정했다. 화면 높이와 wrapping toolbar를 기준으로 −2~20m를
다시 맞추며,943×845 실제 브라우저 캡처로 차체와 전체 경로가 보이는지 확인했다.
기존 최신 관측 비교는 ‘선택 입력 아님’으로 표시하고, 로컬 패널은 실제 선택
묶음의 부호 있는 source 차와 결과 수신 순서를 표시한다.

## 이번 로컬 검사

- C++ local core 회귀, 실제 별도 ROS master의 MPC 경계 회귀 통과.
- 전역 AVOID/로터리/도로 제약/충돌 기하와 기존 입력·MPC 참조 관련14개 회귀 통과.
- V2 motion/shadow/local 표시와 source-exact raster 캐시, 문서 명령·링크 검사 통과.
- 일반 controller.yaml·planner 행동 정책 변경 없음. hosted CI는 로컬 검사와 별도다.
