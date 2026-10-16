# TensorRT 기본·family 공급 최적화 — 2026-10-15–16

PR #85의 최신 시작점이다. [실행 계약](../../contracts/perception/camera-runtime-v1.md)과 함께 읽는다.
TwinLite 기본 backend는 TensorRT FP16이다. 2026-10-16 사용자 직접 주행·시각화에서 YOLO와 family 차선의 정성 확인을 마쳤다.
전체 자동 공급 수락은 별도다. 사용자는 2026-10-16 아래 순간 공백/freshness 제한을 기록한 상태로 현재 리팩토링을 main에 병합하도록 승인했다.
병합 결정은 자동 시험 전체 통과를 뜻하지 않으며, 원본 실패와 후속 개선 항목을 보존한다.
이전 [음영 family 시험](camera-shadow-family-optimization.md)과 [PyTorch/TensorRT 차이](camera-twinlite-tensorrt.md)는 당시 증거로 보존한다.

## 현재 기본 구성 — 사용자 검토용

아래는 PR #85로 반영하는 `camera.launch` 정본이다. 실제 병합 커밋과 시각은 [PR #85](https://github.com/INHAautonav/2026-ASMC/pull/85)의 병합 기록을 따른다.
구형 `perception.launch`의 작은 입력/10Hz 호환 설정과 과거 실험 YAML은 현재 기본값과 구분한다.

| 항목 | YOLO 객체 인지 | TwinLite 차선 semantic |
|---|---|---|
| 모델 | YOLO26l detection, v11 E8 | TwinLiteNet+ Medium v6, 기존 EMA |
| 기본 backend | TensorRT FP16 | TensorRT FP16 |
| 전방 B1 tensor, N×C×H×W | 1×3×544×960 | 1×3×544×960 |
| 좌우 B2 tensor, N×C×H×W | 2×3×480×640 | 2×3×480×640 |
| 목표 신규 공급 | 뷰별15Hz | 뷰별15Hz, model-grid/family 상한15Hz |
| 최대 batch 대기 | 10ms | 20ms |
| 입력 차체 mask | 전방·좌·우 ON | YOLO의 차체 mask를 적용하지 않음 |
| 기본 출력 | 원본16종+세 소비 그룹, bbox/confidence | drivable 2종+road_marking 4종, class/confidence |
| 복구/선택 | `detector:=pretrained`, 재시작 필요 | `lane_runtime_pytorch.yaml` 명시적 선택 |

전방 원본1280×720은 종횡비를 유지해960×540으로 축소하고 상하2px씩 padding한다.
좌우 원본640×480은480×640 tensor에 들어가며 과거384×640의 축소/padding을 사용하지 않는다.
표의 크기는 높이×너비다. 결과의 bbox·header·카메라 보정은 원본 영상 기준을 유지한다.
TwinLite의 입력과 model-grid 투영은 같은 [`lane_input.yaml`](../../../src/asmc_camera/config/camera/lane_input.yaml)을 읽는다.
YOLO는 별도 [E8 프로필](../../../src/asmc_camera/config/camera/yolo26l_mission16_e8_three_view.yaml)의 `front_width: 960`으로 같은 tensor shape를 구성한다.
BEV의 미터 격자·카메라 보정·피팅 거리를960에 비례해 바꾸지는 않는다.

두 모델은 **독립 프로세스·worker·모드·세대**를 사용한다. 같은 B1/B2 shape나 목표 Hz가 같은 입력 stamp·발행 위상 또는 tensor 공유를 뜻하지 않는다.
YOLO 기본 경로는 CPU JPEG decode/linear resize 후 GPU 색 순서·정규화를 수행한다.
TwinLite는 CPU JPEG decode 후 원본 uint8을 GPU에 올려 기존 area 축소·RGB·정규화를 수행한다.
Mask·보간·binding dtype이 달라 최종 tensor를 공용으로 사용하지 않는다. TwinLite engine 내부 FP16과 FP32 입력 binding도 구분한다.
공유 입력 후보는 CPU 감소가 있었으나 YOLO p95가5.10–5.44ms 악화해5ms 상한을 넘었으므로 기본에 채택하지 않았다.

YOLO 기본은 confidence0.40·원본 클래스별 NMS IoU0.50이다. E8 one2one 출력 뒤 후처리하며 전체 그룹 NMS/교차 클래스 억제는 켜지 않는다.
`source_class_name`은 원본 학습 이름, `class_name`은 다음 소비 그룹이다.

| 소비 그룹 | 원본 학습 클래스 |
|---|---|
| vehicle | car, bus, truck |
| dynamic_obstacle | person, dog, deer, bicycle, stroller |
| static_obstacle | motorcycle, bollard, box, concrete_barrier, steel_barricade, traffic_drum, plastic_barrier, traffic_cone |

SportBike의 정적 분류는 사용자 확인에 따른 대회 에셋 의미다. 그룹만으로 실제 속도나 `is_dynamic`을 결정하지 않는다.
ResNet은 camera4·목표10Hz의 별도 프로세스이며 기존 전처리/bfloat16 경로를 유지한다. 세 모델 시각화 목표는10Hz다.
YOLO E8/사전학습 `.pt`·`.engine`, TwinLite checkpoint/두 engine은 패키지 `models/`에 실제 파일로 보존한다.
학습 재개용 원본 checkpoint는 별도로 유지하며, 배포본에 pruning·INT8·재학습을 적용하지 않았다.

## 고정 실행·변경 범위

YOLO/TwinLite 전방544×960 B1·좌우480×640 B2, 목표15/15/10Hz, E8·confidence0.40·NMS0.50을 유지한다.
Mask·보정·두 semantic head·피팅 거리·근거 수·robust 반복·support·freshness350ms는 변경하지 않았다.
모델별 프로세스와 공개 메시지/서비스 MD5·결과 토픽·제어 alias도 유지한다. 수집·학습·LiDAR 구현·판제 정책은 이번 범위가 아니다.

- 실행은 `camera.launch`, 시각화는 `view.launch`로 분리했다. 시각화가 추론 생산자를 시작하지 않는다.
  기능별 `/perception/camera/{objects,lane,traffic_light}/set_mode`·`status`와 기존 alias는 같은 제어 상태/세대를 사용한다.
  기존 bbox 계약은 유지하고 observation에 session/generation/calibration/model/config 식별자를 추가했다.
  이는3뷰 관측 제공이며3뷰 LiDAR fusion 구현 완료가 아니다.
- YOLO는 direct AutoBackend와 공통 TensorRT runner를 사용해 프레임마다 큰 Ultralytics Results 객체 그래프를 만드는 경로를 줄였다.
  Head·원본 좌표 복원·NMS 의미는 유지하며 GPU stream/context·shape별 출력 버퍼를 재사용한다.
- `lane_runtime.yaml`: TensorRT·GPU 전처리·수요 기반 출력 정본. Engine/checkpoint/입력/runtime SHA 검사를 유지하며 자동 fallback하지 않는다.
  Softmax/argmax를 포함한 축약 출력으로 필요한 확률/class/confidence를 옮기고 준비 시 device/pinned host 버퍼를 할당한다.
  Road-marking model-grid를 먼저 발행하며, 원본 크기 복원/직렬화는 실제 소비자가 요청할 때만 수행한다.
  현재 축약 drivable/road-marking D2H 자체는 계속 수행한다. 무전송 경로로 설명하지 않는다.
- `lane_runtime_pytorch.yaml`: 같은 입력 크기의 명시적 PyTorch 복구. 구형 작은 입력은 기존 호환 경로다.
- Family 필수 경로는 `3뷰 model-grid → 카메라별 지면 투영 → 근거 통합 → shared family 피팅/판정 → boundary_family`다.
  영상 BEV 생성/합성은 별도 수요 기반 경로이며 구독이 없으면 입력 구독과 큰 영상 처리를 중단한다.
  독립 spline 피팅은 명시적 비교 경로에만 남기고 일반 marker/view 구독으로 다시 계산하지 않는다.
  일반 구간 STANDBY → 음영 진입 전 PREPARE → 실제 새 영상/family 준비 확인 후 ACTIVE → 이탈 후 STANDBY를 사용한다.
  구간 판정은 판제 책임이며 STANDBY에서는 피팅/유효 결과 생산을 중단하고 이전 결과를 무효화한다.
- Family는 뷰별 최대4프레임에서35ms skew를 만족하는 최신 완성 묶음을 선택한다. 계산 중1개·대기 최신1개이며 대체/탈락을 계수한다.
- 계산 context는 짧은 세대 잠금에서 확보한다. 계산·메시지 조립은 밖에서 수행하고, 상태 반영·발행 직전 세대/cutoff/STANDBY를 다시 검사한다.
- 고정 spline sampling/미분 basis·정규화 행렬은 읽기 전용 bounded cache16개를 사용한다. 프레임별 임시 배열은 한 계산 안에서 재사용한다.
- Family 정본을 먼저 발행한다. 큰 debug 생성은 구독 수요와2초 진단 주기를 따른다. 비교용 독립 spline/pipeline audit 명시 경로는 호환 경로로 남긴다.
- Family 프로세스에만 BLAS/OMP/MKL thread1을 지정한다. 다른 모델의 CPU thread 설정은 변경하지 않는다.
- ROS에서 `uint8[]` support를 bytes로 역직렬화한 경우도 family 시각화가 처리한다. Support를 바꾸지 않고 observed/inferred/invalid를 그대로 그린다.
- 객체 시각화는4개 노드의 JPEG 중계 대신 단일 렌더러에서 정확한 camera/stamp의 원본을 decode·overlay·좌/전방/우 합성하고 JPEG를 한 번 encode한다.
  같은 뷰/stamp 렌더링은 재사용하고 개별 뷰는 실제 구독할 때만 추가 encode한다. 원본 JPEG는 뷰별8개/500ms, 대기는 최신1개로 제한한다.
  화면 수요가 없으면 원본 영상 구독/decode/render/encode를 중단한다. 빈 정상 결과는 갱신하고 오류/세대 변경/중단 뷰는 이전 박스를 제거한다.
  TwinLite/BEV/ResNet 시각화도 수요 기반으로 처리하고 family 화면은 발행된 정본을 그린다.
- GC 비활성화·freshness 확대·오래된 결과 재발행은 하지 않았다. 계측 journal의 bounded/file 저장은 측정기 개선이며 모델 가속과 구분한다.

## 계측 정의

모델 input callback→model-grid publish, family input→계산→publish begin/end→시험 소비자 수신을 source stamp와 실행/세대 context로 연결한다.
Family가 사용하는 서로 다른 세 뷰 stamp도 trace에 기록한다. 입력 source stamp를 실제 촬영 시각이라고 단정하지 않는다.
시험 family 구독 queue는 readiness 소비자와 같은 최신 결과 중심1이다. 큰 결과·진단 기록은 bounded async JSONL로 저장하고 ACTIVE 종료 후 읽는다.
Journal drop/끝 기록 누락·stamp 대응 실패는 계측 실패다. 모델 지연, family 계산, 전체 family 지연, 발행 공백, 소비자 수신 공백을 별도로 보고한다.
Percentile은 각 창의 실제 frame 표본을 사용하고 반복 통합은 raw 표본을 pooling한다. 실행별 p50의 평균이 아니다.

`input_source_age_ms`는 호환을 위해 남긴 **마지막 완료 모델 결과**의 age다. 같은 값의 명확한 이름은 `model_result_source_age_ms`다.
`latest_input_source_age_ms_by_view`는 최근 수신 input header의 age, `latest_input_receipt_age_ms_by_view`는 wall clock 수신 후 경과 시간이다.
GC는 Python이 순환 참조 등을 검사·회수하는 과정이다. Gen2 검사가 Python 실행을 오래 점유할 수 있지만, GPU forward나 모든 공백의 원인이라는 뜻은 아니다.
GC 정책·freshness를 바꾸지 않고 프로세스별 GC 시간, thread CPU/wall, 잠금 대기·직렬화/수신 시간을 함께 기록한다.

## 검증 결과

SUNNY/FOGGY×11/13/15시2,356개 동일 TensorRT model-grid 묶음에서 좌표 최대 차이0m, validity/role/support/shape 변경0이다.
오프라인 처리(메시지 조립 포함)는 기존 p50/p95/p99=38.03/44.24/48.01ms, 신규29.55/35.19/38.58ms다.
실시간 수락 수치와 구분한다. 기존1,404장 semantic 검증은 전처리/모델 출력이 같아 재사용했다.
합성 fitting120회 대조에서 thread1은 출력 차이0m, p95=17.54→15.34ms였다. 실제 부하 기준으로 최종 확인한다.

전체 GUI 전후 비교는 같은 수정 렌더러를 사용한다. Frozen baseline 추론/피팅 파일은 수정하지 않고 계측 launch prefix에서 동일한 scripts entrypoint와 catkin Python 경로를 선택한다.
외부 주행·시각화 wire 처리 실패·대조용 실행 연결 실패는 원본과 별도 조건 정정 기록으로 보존한다. 정차 guard는 위치/속도를 읽고 조건이 바뀌면 시험을 중단하며, 외부 제어를 덮어쓰지 않는다.
조기 종료에서는 ROS process monitor의 정리보다 먼저 signal 시각을 기록한다. 과거 조기 종료 후 정리 시간을 ACTIVE에 포함한31.4초 꼬리 공백은 계측 결함으로 정정하며, 원본 보고서는 보존한다.

### 정규 반복12회 — 2026-10-16

MORAI 터널 내부 정차·13시, Main preview/LiDAR·세 모델·전체 시각화·실제 rqt1개다.
기존/신규 TensorRT를 SUNNY/FOGGY 각각60초×3회 교차했다. 정상 완료한 회차는 유지하고 외부 휠 제어로 중단된 회차만 새 실행으로 보완했다.
아래 분위수는 raw frame을 합친 값이며 실행별 중앙값의 평균이 아니다. 실패 회차도 포함한다.

| 날씨·family 구현 | TwinLite 전방 model-grid p50/p95/p99 ms | 전체 family p50/p95/p99 ms |
|---|---|---|
| SUNNY 기존 |55.2 /95.7 /112.5|120.7 /177.5 /204.5|
| SUNNY 신규 |49.9 /90.4 /104.1|99.2 /148.2 /198.8|
| FOGGY 기존 |45.3 /74.1 /87.0|113.9 /164.4 /189.5|
| FOGGY 신규 |44.7 /70.8 /82.2|95.2 /130.5 /152.4|

모델의 최소 공급·지연·공백·stamp 기준은12회 모두 통과했다. 전체 수락은11/12회 통과다.
신규 SUNNY r2에서 readiness false1표본이 남았다. Family는 valid이고 입력은 준비됐지만 source age392.7ms가350ms를 초과했다.
최신 입력 source age43–47ms, family receipt age84.3ms, 해당 계산 wall122.5ms/thread CPU115.5ms·대기63.6ms였다.
같은 시점에 계측된 gen2 GC는 없었다. 계산 burst와 대기는 확인했지만 workload가 증가한 원인은 추가 대조가 필요하다.
Freshness를 늘리거나 이 표본을 면제하지 않는다.

### 최종 FOGGY13시·전체 GUI10분

| 항목 | 결과 |
|---|---|
| YOLO 전방 |14.94Hz, p50/p95/p99=29.8/43.8/48.8ms |
| TwinLite 전방 model-grid |14.85Hz, p50/p95/p99=43.7/69.0/79.2ms |
| 유효 family |8,899개·14.83Hz, 최저 완전5초 창14.2Hz |
| 전체 family 지연 |p50/p95/p99=91.8/123.4/142.6ms |
| Family 계산 |p50/p95/p99=40.0/54.9/70.2ms |
| Family 발행 시작→수신 |p50/p95/p99=0.87/1.49/2.97ms |
| 최대 공백 |family 발행261.7ms·수신261.5ms —250ms 기준 실패 |
| Readiness |정상 ACTIVE 저하0 |
| 자원 |전체 VRAM peak57.1%, camera/debug/Main CPU 평균384.8%(100%=1코어), RSS peak3,782.8MiB |

YOLO/TwinLite 최소10Hz·ResNet 최소9.5Hz, 모델 p95/p99·공백, stamp 순서·오류·journal 무손실은 통과했다.
15Hz의95% 목표는 별도다. TwinLite/family는 일부5초 창14.2Hz여서 전체 창의15Hz 목표 달성으로 표시하지 않는다.
실제 family 시각화6,000개에 source stamp가 있었고 bytes 처리/렌더러 오류는0이다. 수거 정책은 변경하지 않았다.

최대 공백은 ACTIVE308.67–308.93초다. Source 간격205.4ms, 해당 model-grid 공백 약234ms, 카메라 입력 callback의 전체 최대 공백은116ms 이하였다.
직전 family 계산38.2ms·다음63.6ms이며 주변의 계측된 gen2 GC는 없다. 따라서 소비자 전달이나 MORAI 렌더링·GC만을 원인으로 단정하지 않는다.
10분 gen2 GC 최대는 family18.0ms, TwinLite/시험 소비자는 관측0이다. 이전533ms/2.34초 정지를 완전히 해결했다는 근거로 확대하지 않는다.

**자동 시험 판정: TensorRT 기본값을 유지하고 전체 자동 수락은 보류한다.** 남은 항목은 SUNNY source-age 저하와10분 family262ms 공백이다.
후속은 모델→model-grid 공급 순간 지연과 family CPU burst를 같은 입력·단계별 시간으로 확인하는 작업이다.
시험 종료 당시 PR은 Draft/미병합이었다. 아래 사용자 정성 주행 확인과 병합 승인은 위 실패를 통과로 바꾸지 않는다.

CPU437개·실제 ROS 모드/관측/렌더링22개, Noetic 빌드·명시적 overlay 설치 및 설치본 E8/pretrained/TRT/PyTorch 복구 경로를 확인했다.
첫 설치가 inherited PWD 때문에 `/root/ws`를 선택한 기록을 보존하고 `catkin_make -C /data/runtime/camera-family-supply-v2 install`로 실제 overlay를 재검증했다. 소스·마운트는 바꾸지 않았다.
rqt는 회차마다0→1→0, 최종 ROS/rqt/master 잔류0이다. 마지막 사용자 Ego/P기어/키보드·CLOUDY13시를 복원했고 위치 오차는4.1mm였다.
공개 ROS 차량 제어와 actor 생성/삭제는0이다.

작은 증거는 [`camera_family_supply_v2`](../../../config/baselines/evidence/camera_family_supply_v2/), 원시는 `data/runtime/camera-family-supply-v2/`에 있다.
[E360 갤러리](http://127.0.0.1:8765/camera-family-supply-20261015-r1/index.html)는 정규13회 비교와 이전 실패/중단을 구분하고 [실험 목록](http://127.0.0.1:8765/index.html)에 연결한다.
Hosted CI는 최신 push의 check 상태를 별도 기록하며, billing 차단을 로컬 검사 통과로 대체하지 않는다.

## 사용자 직접 주행 확인 — 2026-10-16

출처는 사용자의 실제 주행·시각화 관찰이다. YOLO와 family 차선이 잘 나오며 현재 누락은 운영을 막을 정도가 아니라는 사용자 판단을 기록한다.
별도 GT 정량 평가나 SUNNY/FOGGY×11/13/15시·거리별 전체 수락으로 확대하지 않는다. 이번 관찰의 정확한 날씨/시각/거리와 표본 수는 미기록이다.

| 항목 | 관찰 | 다음 개선 요청 시 확인할 점 |
|---|---|---|
| YOLO·family | 직접 실행/시각화에서 정상 사용 가능 | 현재 설정을 기준선으로 보존 |
| bus·truck | 가끔 검출이 약하거나 누락 | 에셋 하위 종류·앞/뒤/측면·거리·날씨·터널별 실패 기록 |
| bicycle·dog | Ego 차로 위, 차량 바로 앞에 놓였을 때 인식률이 떨어지는 경향 | 측방 배치 자료와 정면/근거리/차로 내 자료의 분포를 비교 |
| dog 하위 종류 | 사용자 표기 `dog 2`, `dog 10`이 `dog1`보다 약함 | 실제 API 에셋 ID를 확인한 뒤 하위 종류별 GT/검출을 비교 |

전역경로 주변 배치 수집에 따른 정면/근거리 관측 부족은 **사용자가 제시한 근거 있는 가설**이다. 원인으로 확정하거나 현재 클래스별 recall 수치로 환산하지 않는다.
`dog 2`/`dog 10`/`dog1`은 사용자 표기를 보존한 것이며 검증된 gRPC API 이름으로 기록하지 않는다. 학습 클래스는 모두 `dog`이고 하위 유형별 평가를 별도로 유지한다.

**결정:** 현재 E8·confidence0.40·NMS0.50과 실행 설정을 유지한다. 추가 수집/학습은 지금 시작하지 않고 판제 측 개선 요청이 있을 때 진행한다.
요청 시 먼저 기존 자료의 하위 유형·정면/후면·차로 내·근거리·음영 조건을 집계하고 실패가 확인된 조건만 보강한다.
보강 후에는 bus/truck/bicycle/dog 개선뿐 아니라 기존16종의 회귀·중복/배경 FP와3뷰 공급도 함께 확인한다.

## 병합 결정과 남은 개발 항목 — 2026-10-16

사용자 직접 주행 확인 후 현재 리팩토링을 적용해 PR #85를 병합하라는 승인을 받았다.
현재 설정을 팀 개발 기준으로 반영하며, SUNNY readiness 저하1표본과10분 family 최대 공백261.7ms는 알려진 제한으로 남긴다.
Freshness350ms/공백250ms 기준이나 실패 증거는 변경하지 않는다. 전체 자동 수락·공동 자율주행 수락·3뷰 LiDAR fusion 완료를 의미하지 않는다.

최신 main의 PR #86 시나리오 검증/replay 변경을 먼저 통합한다. 기존 카메라 공개/호환 진입점과 계약을 검사하고 사용자 실행 프로세스·컨테이너 마운트는 변경하지 않는다.
병합 전 hosted CI3개는 모두 billing/spending 제한으로 steps0 상태였다. 로컬 검사와 hosted 미실행을 구분하고 CI 보호 규칙을 해제하지 않는다.
main `5c53ac06` 통합 후 문서 migration·공용/인지/토픽/실행 계약 검사와 회귀60개를 통과했다.
회귀는 카메라 facade17·planning launch/filter4·시나리오 도구 camera runtime4·공용 계약35개다.
실행 코드/모델은 추가 변경하지 않았으므로 기존 CPU437개·실제 ROS22개·빌드/출력 동등성·실시간 실패 증거를 재사용한다.

다음 작업의 첫 대상은 LiDAR 인계 문서다. 카메라별 관측 버퍼·보정·좌표·source stamp·session/generation·원본 이름/세 그룹·정상 빈 결과/무효 결과의 소비 규칙과 담당 범위를 명시한다.
현재 실행 fusion은 기존 Main의 전방 bbox unique·±50ms·200ms 대기이며, `asmc_lidar`의3뷰 fusion 구현은 후속 별도 작업이다.
그다음 디스크/데이터 경로를 확정하고 전체 docs를 현행화한다. 추가 수집/학습은 판제 측 개선 요청 이후에 진행한다.
공급 순간 지연과 family CPU burst는 별도 코드 개선 항목으로 유지하며, 가중치 추가학습으로 해결됐다고 가정하지 않는다.

## 직접 실행·복구

현재 컨테이너 마운트는 유지한다. PR #85 시험 overlay를 쓰려면 `docker exec -it asmc-ros-noetic bash` 후 **각 터미널에서** 다음을 source한다.

```bash
source /opt/ros/noetic/setup.bash
source /data/runtime/camera-family-supply-v2/devel/setup.bash
```

다른 bridge/추론 생산자가 없는 상태에서 bridge 한 세트와 카메라 한 세트를 실행한다. 기존 bridge가 있으면 추가 실행하지 않는다.

```bash
roslaunch asmc_bridge bridge.launch transport_profile:=local stream_profile:=competition
```

별도 터미널:

```bash
roslaunch asmc_camera camera.launch
```

모드 요청은 별도 터미널에서 한다. 일반 구간은 lane STANDBY를 유지한다.

```bash
rosservice call /perception/camera/objects/set_mode '{mode: 1, target_hz: 0.0}'
rostopic echo -n 1 /perception/camera/objects/status
# 같은 generation의 prepared=true 확인 후
rosservice call /perception/camera/objects/set_mode '{mode: 2, target_hz: 0.0}'
rosservice call /perception/camera/traffic_light/set_mode '{mode: 1, target_hz: 0.0}'
rostopic echo -n 1 /perception/camera/traffic_light/status
# 같은 generation의 prepared=true 확인 후
rosservice call /perception/camera/traffic_light/set_mode '{mode: 2, target_hz: 0.0}'
# 음영 진입 전
rosservice call /perception/camera/lane/set_mode '{mode: 1, target_hz: 0.0}'
rostopic echo /perception/camera/lane/status
# 같은 generation의 prepared=true 및 pipeline_ready=true 확인 후 Ctrl-C, ACTIVE
rosservice call /perception/camera/lane/set_mode '{mode: 2, target_hz: 0.0}'
# 음영 이탈
rosservice call /perception/camera/lane/set_mode '{mode: 0, target_hz: 0.0}'
```

시각화와 rqt는 각각 한 세트만 시작한다. 한 rqt의 topic 선택으로 객체/차선/신호등을 바꿔본다.

```bash
roslaunch asmc_camera view.launch models:=all
```

별도 터미널:

```bash
rosrun rqt_image_view rqt_image_view --clear-config /perception/camera/objects/debug/mosaic/compressed
```

차선 family topic은 `/perception/lane/debug/shared_family_bev/compressed`다.
터널 입구·내부·출구의 역할/위치·observed/inferred·끊김과 STANDBY→PREPARE 재개를 확인한다.
YOLO의 터널/근거리 누락은 모델 품질 항목으로 따로 기록한다.

명시적 PyTorch 복구는 **기존 camera.launch를 종료한 뒤** 내부 설정을 선택한다. 다른 카메라 생산자와 동시에 실행하지 않는다.

```bash
roslaunch asmc_camera _runtime.launch lane_runtime_config:=$(rospack find asmc_camera)/config/camera/lane_runtime_pytorch.yaml
```

## 다음 순서

1. 사용자 승인에 따라 현재 리팩토링을 PR #85로 main에 반영한다. 자동 공급의 알려진 제한은 별도 후속 개선/재검증 항목으로 유지한다.
2. LiDAR 인계:3뷰 관측·보정·시각·세대·원본 이름/그룹 계약과 담당 작업을 갱신한다. 현재 실행 fusion은 기존 Main이며3뷰 fusion 완료로 표시하지 않는다.
3. 큰 자료의 복사/해시/링크/컨테이너 경로 검증 후 E드라이브 중심 저장 경로를 확정하고 디스크를 정리한다.
4. 그 경로를 기준으로 전체 docs를 현행화하고 중복을 정리한다.
5. 추가학습은 판제 측 개선 요청 이후에 진행한다. 객체 누락/혼동이면 YOLO E8 추가학습, semantic 차선 누락이면 TwinLite 추가학습, semantic 정상인데 family가 나쁘면 피팅/보정/실행을 개선한다. 두 모델을 동시에 재학습하지 않는다.
