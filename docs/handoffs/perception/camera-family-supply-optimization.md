# TensorRT 기본·family 공급 최적화 — 2026-10-15–16

PR #85의 최신 시작점이다. [실행 계약](../../contracts/perception/camera-runtime-v1.md)과 함께 읽는다.
TwinLite 기본 backend는 TensorRT FP16이다. 전체 공급 수락과 사용자 주행 확인은 별도이며 PR은 Draft로 유지한다.
이전 [음영 family 시험](camera-shadow-family-optimization.md)과 [PyTorch/TensorRT 차이](camera-twinlite-tensorrt.md)는 당시 증거로 보존한다.

## 고정 실행·변경 범위

YOLO/TwinLite 전방544×960 B1·좌우480×640 B2, 목표15/15/10Hz, E8·confidence0.40·NMS0.50을 유지한다.
Mask·보정·두 semantic head·피팅 거리·근거 수·robust 반복·support·freshness350ms는 변경하지 않았다.
모델별 프로세스와 공개 메시지/서비스 MD5·결과 토픽·제어 alias도 유지한다. 수집·학습·LiDAR 구현·판제 정책은 이번 범위가 아니다.

- `lane_runtime.yaml`: TensorRT·GPU 전처리·수요 기반 출력 정본. Engine/checkpoint/입력/runtime SHA 검사를 유지하며 자동 fallback하지 않는다.
- `lane_runtime_pytorch.yaml`: 같은 입력 크기의 명시적 PyTorch 복구. 구형 작은 입력은 기존 호환 경로다.
- Family는 뷰별 최대4프레임에서35ms skew를 만족하는 최신 완성 묶음을 선택한다. 계산 중1개·대기 최신1개이며 대체/탈락을 계수한다.
- 계산 context는 짧은 세대 잠금에서 확보한다. 계산·메시지 조립은 밖에서 수행하고, 상태 반영·발행 직전 세대/cutoff/STANDBY를 다시 검사한다.
- 고정 spline sampling/미분 basis·정규화 행렬은 읽기 전용 bounded cache16개를 사용한다. 프레임별 임시 배열은 한 계산 안에서 재사용한다.
- Family 정본을 먼저 발행한다. 큰 debug 생성은 구독 수요와2초 진단 주기를 따른다. 비교용 독립 spline/pipeline audit 명시 경로는 호환 경로로 남긴다.
- Family 프로세스에만 BLAS/OMP/MKL thread1을 지정한다. 다른 모델의 CPU thread 설정은 변경하지 않는다.
- ROS에서 `uint8[]` support를 bytes로 역직렬화한 경우도 family 시각화가 처리한다. Support를 바꾸지 않고 observed/inferred/invalid를 그대로 그린다.

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

**결정: TensorRT 기본값을 유지하고 전체 자동 수락은 보류한다.** 남은 차단은 SUNNY source-age 저하와10분 family262ms 공백이다.
후속은 모델→model-grid 공급 순간 지연과 family CPU burst를 같은 입력·단계별 시간으로 확인하는 작업이다. 사용자 주행 확인도 아직 미완료이며 PR #85는 Draft/미병합이다.

CPU437개·실제 ROS 모드/관측/렌더링22개, Noetic 빌드·명시적 overlay 설치 및 설치본 E8/pretrained/TRT/PyTorch 복구 경로를 확인했다.
첫 설치가 inherited PWD 때문에 `/root/ws`를 선택한 기록을 보존하고 `catkin_make -C /data/runtime/camera-family-supply-v2 install`로 실제 overlay를 재검증했다. 소스·마운트는 바꾸지 않았다.
rqt는 회차마다0→1→0, 최종 ROS/rqt/master 잔류0이다. 마지막 사용자 Ego/P기어/키보드·CLOUDY13시를 복원했고 위치 오차는4.1mm였다.
공개 ROS 차량 제어와 actor 생성/삭제는0이다.

작은 증거는 [`camera_family_supply_v2`](../../../config/baselines/evidence/camera_family_supply_v2/), 원시는 `data/runtime/camera-family-supply-v2/`에 있다.
[E360 갤러리](http://127.0.0.1:8765/camera-family-supply-20261015-r1/index.html)는 정규13회 비교와 이전 실패/중단을 구분하고 [실험 목록](http://127.0.0.1:8765/index.html)에 연결한다.
Hosted CI는 최신 push의 check 상태를 별도 기록하며, billing 차단을 로컬 검사 통과로 대체하지 않는다.

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

1. 자동 검증과 사용자 직접 주행 확인 후 최신 main/회귀를 확인해 PR #85를 정리한다. 이번 작업에서 병합하지 않는다.
2. LiDAR 인계:3뷰 관측·보정·시각·세대·원본 이름/그룹 계약과 담당 작업을 갱신한다. 현재 실행 fusion은 기존 Main이며3뷰 fusion 완료로 표시하지 않는다.
3. 큰 자료의 복사/해시/링크/컨테이너 경로 검증 후 E드라이브 중심 저장 경로를 확정하고 디스크를 정리한다.
4. 그 경로를 기준으로 전체 docs를 현행화하고 중복을 정리한다.
5. 객체 누락/혼동이면 YOLO E8 추가학습, semantic 차선 누락이면 TwinLite 추가학습, semantic 정상인데 family가 나쁘면 피팅/보정/실행을 개선한다. 두 모델을 동시에 재학습하지 않는다.
