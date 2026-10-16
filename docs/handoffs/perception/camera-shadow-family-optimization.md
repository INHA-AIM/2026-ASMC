# 음영구간 family 실행 최적화 — 2026-10-15

PR #85의 후속 구현이다. 시작점은 [실행 계약](../../contracts/perception/camera-runtime-v1.md),
이전 증거는 [TensorRT 변환·부하 검증](camera-twinlite-tensorrt.md)이다.
아래는 당시 독립 PyTorch 기본값과 TensorRT 후보 시험의 기록이다. 최신 기본 backend와 후속 수락 상태는
[TensorRT 기본·family 공급 최적화](camera-family-supply-optimization.md)를 따른다. 아래 과거 결과는 덮어쓰지 않는다.

## 실행 경로와 수요

전방544×960 B1·좌우480×640 B2, YOLO/TwinLite15Hz·ResNet10Hz를 유지한다.
추론과 family 지면 투영은 같은 `lane_input.yaml`을 읽는다. 입력 해상도와 BEV의
미터 단위 격자는 별개이며, 보정·거리·차선 유효성 임계값은 변경하지 않았다.

필수 경로는 model-grid → 카메라별 지면 투영 → 3뷰 근거 통합 → shared family다.
영상 BEV는 별도 소비 경로다. 합성 BEV에 구독자가 없으면 뷰별 BEV 입력을 해제하고,
뷰별 출력도 수요가 없으면 native semantic 구독을 해제한다. PyTorch 복구 프로필도
`lazy_outputs=true`로 사용하지 않는 native 복원을 생략한다. Recorder 구독은 GUI와
동일한 정상 수요이며 기존 토픽·영상 좌표는 유지한다.

수요 해제·STANDBY·세대/cutoff 전환에서 동기화 그룹을 폐기하고 새 큐로 재구독한다.
이미 연결 해제된 ROS callback도 이전 그룹 epoch로는 결과를 전달하지 못한다.
같은 generation 값으로 재시작한 경우에도 성공 batch counter가 감소하면 로컬 epoch를
갱신한다. Family tracker와 대기 작업은 이전 epoch를 재사용하지 않는다.

독립 spline은 내부 `enable_independent_boundary_pipeline`을 명시한 경우에만 실행한다.
Marker/audit 구독만으로 활성화하지 않는다. 일반 `view.launch models:=lane`에는 실제
`LaneBoundaryFamily`를 그리는 family renderer가 포함된다. 같은 결과의 observed/inferred/
invalid 상태를 구분하며 다른 피팅을 생성하지 않는다. 출력은 기존
`/perception/lane/debug/shared_family_bev/compressed`다.

## 모드와 readiness

판제가 일반 구간 STANDBY → 음영 진입 전 PREPARE → 준비 확인 후 ACTIVE → 이탈 후
STANDBY를 요청한다. 위치 판단·HD맵 정책·제어는 카메라가 수행하지 않는다.

- STANDBY: 가중치 유지, 추론/피팅 중단, 큐 정리와 이전 family 무효화.
- PREPARE: 실제 새 3뷰 입력과 family 유효성까지 확인한다.
- ACTIVE: 새 결과만 공급하며 freshness350ms를 유지한다.

Family 메시지에는 generation 필드가 없으므로 기존 필드와 MD5를 유지하면서, 추론
생산자가 이번 세대에 완료한 stamp의 제한된 목록(최대256개)에 있는 피드백만 받는다.
이전 결과·알 수 없는 source·cutoff 이전 결과로 readiness를 만들지 않는다.
상태 서비스의 새 이름과 구형 alias는 같은 상태를 사용한다.

PR #85를 빌드한 `asmc-ros-noetic` 환경에서 실행한다. 기존 생산자를 종료한 뒤 사용한다.

```bash
roslaunch asmc_camera camera.launch
rosservice call /perception/camera/lane/set_mode "mode: 1
target_hz: 15.0"
rostopic echo /perception/camera/lane/status
# 준비 상태를 확인한 뒤 음영구간에서 ACTIVE
rosservice call /perception/camera/lane/set_mode "mode: 2
target_hz: 15.0"
# 구간 이탈
rosservice call /perception/camera/lane/set_mode "mode: 0
target_hz: 15.0"
```

시각화와 rqt는 별도이며 각각 한 세트만 실행한다.

```bash
roslaunch asmc_camera view.launch models:=lane
rosrun rqt_image_view rqt_image_view --clear-config /perception/lane/debug/shared_family_bev/compressed
```

TensorRT 후보 선택은 기존 private `_runtime.launch`의 `lane_runtime_config`로 한다.
복구는 후보를 종료하고 `camera.launch`를 다시 시작한다. 일반 공개 옵션은 늘리지 않았다.

## GC 진단과 이전 실패 해석

이전533ms generation2 GC 검사, TwinLite590ms와 family644ms 공백은 미해결 증거다.
GC를 끄거나 임계값·freshness를 변경하지 않는다. Family 피드백에는 scalar만 보존한다.
기존 lifecycle trace에 피드백 잠금 대기 시간을 추가했다.

`camera_runtime_probe.py --allocation-profile`은 별도 진단 실행에서 tracemalloc,
역직렬화/피드백 wall·thread CPU 시간, GC count/statistics와 종료 시 retained allocation/
tracked type 요약을 남긴다. 이 실행은 계측 비용 때문에 수락 지연 측정에 사용하지 않는다.
일반 probe는 기존 generation2 GC와 실제 입력 callback 계측을 유지한다.

이전 FOGGY r3의 `failure-lane.jpg`는 터널 외부의 다수 황색선 장면임을 영상으로 확인했다.
정확한 route station은 기록되어 있지 않아 unknown이다. 이 장면의 family 근거 부족은
음영 ODD 내부 품질 실패와 구분한다. 원래 실패 수치와 이미지는 삭제하지 않으며,
GC/공급 공백은 ODD 구분으로 면제하지 않는다.

## 고정 RGB·pose와 검증 상태

`prepare_shadow_family_fixture.py`가 기존 20260912 shadow-static bag 6개를 고정했다.
SUNNY/FOGGY×11/13/15시의 **2,356개 3뷰 묶음(7,068개 RGB 참조)**이다.
원본 JPEG를 복제하지 않고 bag/영상 SHA, header·fallback 정책, 동기화와 보간 pose를 기록한다.
연속 영상이며 독립 장면 수나 새 GT로 설명하지 않는다. 실제 촬영 시각의 동기 정확도를
보증하지 않는다. 구간은 기존 주행 시험의 station 경계로 구분한 개발 fixture다.

| 조건 | 3뷰 묶음 | 입구 전 | 음영 경계 내부 | 출구 후 |
|---|---:|---:|---:|---:|
| SUNNY11 |485|125|282|78|
| SUNNY13 |427|119|227|81|
| SUNNY15 |370|92|201|77|
| FOGGY11 |346|93|165|88|
| FOGGY13 |358|111|194|53|
| FOGGY15 |370|108|205|57|

Manifest SHA: `4b5cf3062216972a027682325ecc856150b77e184541d2f74fb142a364ced164`.
로컬 원본은 `data/runtime/camera-shadow-family-v1/fixture-r1/`, 작은 요약은
`config/baselines/evidence/camera_shadow_family_v1/`에 보존한다.

기존 `validate_twinlite_tensorrt.py --family-fixture`는 동일 JPEG와 production vectorizer를
사용해 PyTorch/TensorRT의 validity·role·support·기하 차이를 기록한다. 전용 로컬 ROS master
(11311 이외 포트)에서만 실행하며 공개 제어를 발행하지 않는다. 이 결과로 live readiness나
GT 정확도를 대신하지 않는다. 차이는 갤러리로 검토하고 자동 승격하지 않는다.

첫 시도는 Docker Desktop/WSL 통합과 MORAI 미실행으로 보류됐으며, 재실행 후 GPU parity와
음영구간 반복 시험을 수행했다. CPU 검사와 과거1,404장 품질 통과를 새 전체 수락으로 표시하지 않는다.
최종 수락은 모델 p95≤150ms·p99≤200ms, 최대공백≤250ms, YOLO/TwinLite 및 유효 family의
완전5초창≥10Hz, ResNet≥9.5Hz, VRAM≤90%, 정상 음영 ACTIVE readiness 저하0을 요구한다.
모드 준비·전환 구간은 별도 집계하고, 15Hz 달성도 최소 공급과 별도로 표시한다.

실제 Noetic launch 검사에서 옛 외부 checkpoint 인자가 component YAML보다 우선하던
문제를 확인했다. 기본 파일을 패키지 내부로 바꾸고 component config가 있을 때는 해당
YAML이 checkpoint를 소유하도록 수정했다. 최초 실패 로그와 수정 후77개 통과를 보존한다.

## 2026-10-15 반복 측정 결과

MORAI 터널 내부 route station 약1940m에서 Ego를 정차하고 13시에 측정했다.
새 family/수요 기반 BEV 구조를 두 backend에 공통 적용했으므로 아래 T0는 기존 main 전체가
아니라 **이번 구조의 PyTorch 복구 경로**다. T2는 TensorRT+GPU 전처리+lazy 출력 후보다.
모든 모델·전체 영상 sink·실제 rqt 한 개·Main preview·기존 LiDAR를 실행했다.
공개 차량 제어는 시험 토픽으로 격리했으며 actor는 생성/삭제하지 않았다.

각 날씨·backend를 60초씩3회 교차 실행했다. 지연은 callback 수신→발행의 원시 프레임을
합쳐 계산한 분위수이며 실행별 중앙값의 평균이 아니다. CPU는 camera/debug/preview 프로세스
합계의 실행별 평균을 평균한 값(100%=1 logical CPU)이다. rqt는 별도 WSL 시스템 CPU에
포함되며 VRAM은 MORAI/Windows를 포함한 전체 GPU다. Family 지연은 별도 경로다.

| 날씨·backend | TwinLite 전방 p50/p95/p99 ms | 유효 family p50/p95/p99 ms | 카메라 CPU | family 최대 공백 |
|---|---|---|---:|---:|
| SUNNY PyTorch |78.4 /118.2 /130.5|140.6 /189.7 /218.2|432.9%|256.0ms|
| SUNNY TensorRT |44.7 /71.6 /85.2|109.6 /153.8 /177.0|400.0%|231.3ms|
| FOGGY PyTorch |84.6 /124.9 /142.9|151.0 /206.6 /233.1|444.1%|193.7ms|
| FOGGY TensorRT |46.0 /77.9 /92.5|118.4 /175.7 /210.2|413.3%|198.1ms|

모델의7개 필수 stream(객체3·차선3·신호등1)은 전체12회에서 최소 공급·p95/p99·공백·stamp
기준을 통과했다. 그러나 전체 family 수락에는 다음 실패가 남아 **기본 전환은 보류**한다.

- PyTorch SUNNY r3: 유효 family 공백256ms로250ms 기준 초과.
- TensorRT FOGGY r2: 유효 family는 계속 나왔으나 source age395/419ms로 readiness2회 저하.
  입력 prepared·family valid·generation은 정상이며 reason은 `source_stale`이었다.
  이60초 실행의 계측된 모델 프로세스에서는 ACTIVE generation2 GC가 없었다.
  당시 family 처리 p95약85ms·shared fitting p95약51ms다. 후단 지연과 source age가 확인됐지만
  최초 원인은 아직 확정하지 않는다. 터널 내부 실패이므로 ODD 밖으로 면제하지 않는다.

모든 관측값과 실패는 `live-matrix-r1/`, 원시 프레임 집계는 `live-summary-r1/`에 남긴다.
일반 probe의 family-process GC 계측은 마지막 반복과 후속 시험부터 추가됐으며 각 실행의
measurement SHA에 구분된다. 이전 실행에 family GC가 없었다고 소급 주장하지 않는다.

고정2,356개 RGB 묶음에서 family validity·role 변경은0이었다. Support 상태가 바뀐 묶음은
364개다. 최초 v1의 같은 sample index 거리 비교는 보존하고, 큰 차이6건을 v2에서 공통 전방
x의 lateral 차이로 재검토했다. 3건은 차이가 대부분 sampling 위치에서 왔지만, 터널 출구 부근
SUNNY3건에는 p95약0.22m·최대0.41m의 실제 곡선 차이가 남았다. Family GT 정확도 통과로
해석하지 않으며 gallery에 두 결과를 함께 남긴다.

별도 tracemalloc60초 진단(수락 제외)에서 family deserialize p95약2.1ms,
피드백 callback p95약8.3ms를 기록했다. 진단 중 ACTIVE에 generation2 검사는 없었다.
Heap146→156MiB에는 모델/라이브러리와 측정기 receipt가 포함되어 있으므로 누수나 특정
메시지의 GC 원인으로 확정하지 않는다. `allocation-summary-r1.json`에 보존한다.

사용자 요청으로 연속 시험은 **30분에서10분으로 변경**했다. 10분 결과를30분 수락으로
표시하지 않는다.

## BEV 대조와10분 연속 시험

BEV만 구독한 SUNNY PyTorch/TensorRT 각60초와 GUI OFF TensorRT SUNNY/FOGGY 각60초는
최소 공급·지연·공백·readiness 기준을 통과했다. Family만 구독한 초기60초 선별에서도
전방14.77Hz·p9561.4ms, 유효 family 공백219ms·readiness 저하0을 기록했다.
이 선별값을 전체 GUI 수락으로 대체하지 않는다.

SUNNY13시 전체 GUI TensorRT600초는 완주했으나 **공백 기준 실패**다.

| 출력 | 평균Hz / 최소 완전5초Hz | p50 /p95 /p99 ms | 최대공백 |
|---|---:|---|---:|
| YOLO 전방 |14.94 /14.6|30.2 /45.0 /52.0|194.9ms|
| TwinLite 전방 model-grid |14.79 /13.4|46.4 /76.2 /90.6|250.0ms 초과¹|
| TwinLite 좌 model-grid |14.79 /13.4|48.8 /79.8 /94.5|250.8ms|
| TwinLite 우 model-grid |14.79 /13.4|49.8 /80.6 /96.5|251.5ms|
| ResNet |9.96 /9.8|59.3 /91.8 /101.8|163.4ms|
| 유효 family² |14.57 /13.2|116.4 /167.7 /195.7|365.9ms|

¹전방의 원시값도250ms를 조금 넘으며 반올림으로 통과 처리하지 않는다.
²family는 입력callback→시험 구독자 수신 지연과 공백이다. 생산자 발행 공백과 같다고 단정하지 않는다.
Readiness 저하0, 중복/역행stamp0, 오류0, VRAM최대62.2%였지만 공백 실패를 면제하지 않는다.
완전5초창15Hz목표95%(14.25Hz)는 일부 창에서 미달이며 최소10Hz와 구분한다.

최대 family 수신 공백은 ACTIVE187.66초의365.89ms다. 두 family의 source 간격은59.09ms였고,
주변 전방 입력 간격≤66.60ms·model-grid 발행 간격≤107.29ms로 공급이 계속됐다.
계측된 TwinLite/family GC도 주변에 없었다. 생산·전달·측정 구독자의 비용은 완전히
분리하지 못했으므로 모델 forward·MORAI 렌더링·GC 중 하나로 원인을 확정하지 않는다.
`worst-family-gap-r1.json`에 근거를 남겼으며 다음 진단은 family 발행→수신과 측정기
자체 GC/보관 비용을 구분해야 한다.

10분 ACTIVE generation2 GC 최대는 TwinLite53.2ms, family22.4ms, YOLO56.3ms였다.
과거533ms/2.34초 정지는 이번에 재현되지 않았지만 해결을 확정하지 않는다.
이벤트 caller는 검사가 촉발된 위치이며 메모리 누수의 원인이라는 뜻이 아니다.
진단명 `input_source_age_ms`는 현재 구현에서 **마지막 완료 결과의 source age**다.
최신 입력 수신 age와 혼동하지 말고 raw input callback과 별도로 해석한다.

GUI 시험은 rqt0→1→0, GUI OFF는0 유지를 확인했고 마지막 ROS/rqt 잔류0이다. 원래 Ego pose(복원 오차5.7mm),
기어D·키보드 제어·CLOUDY13시를 복원했다. 정확한 물리 속도 재현을 의미하지 않는다.
실제 Noetic catkin build/install, 설치본 E8/pretrained/TwinLite 내부 경로와 CPU 도구7개를 확인했다.

**결정:** family 전용·수요 기반 BEV·세대 게이트와 진단 구현을 유지한다. TensorRT는 후보로
남기고 PyTorch 기본값·독립 프로세스를 유지한다. PR #85는 Draft/미병합이다.
결과는 [E356 갤러리](http://127.0.0.1:8765/camera-shadow-family-20261015-r1/index.html),
작은 JSON/CSV 증거는 `config/baselines/evidence/camera_shadow_family_v1/`, 원시는
`data/runtime/camera-shadow-family-v1/`에 있다. 후속 LiDAR 연결과 운영 승격을 완료로 표시하지 않는다.

최신 결과 커밋 `ae6dd452`의 hosted 카메라/planning/LiDAR CI는 결제 또는 지출 한도 문제로
job 시작 전에 차단됐다(steps0). 코드 시험 실패나 통과로 해석하지 않는다.
`hosted-ci-block.json`에 대상 commit·check ID·확인 시각을 기록했다. 로컬 검사로 이 차단을 대체하지 않는다.
