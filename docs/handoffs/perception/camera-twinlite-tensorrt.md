# TwinLiteNet+ TensorRT 검증 — 2026-10-15

시작점은 [카메라 실행 계약](../../contracts/perception/camera-runtime-v1.md)이며
이전 [B1/B2·공유 입력 증거](camera-shared960-validation.md)를 보존한다.
PR #85에서 진행하고 병합하지 않는다. 현재 기본은 독립 PyTorch이며 TensorRT는 후보다.
모델·차선 알고리즘·보정·freshness350ms·ROS MD5를 변경하지 않는다.

## 고정 모델과 실행

- Medium v6 `best_mean.pt`의 실제 선택된 EMA. 원본 SHA
  `a26311b3af4e92880f632509eff3c81defc1b95135f5b463c7a08e5b6e23d24b`.
- Upstream `90f1b8695ae311d5123b05f8534b2e11e42499d2`.
- 전방1×3×544×960, 측방1–2×3×480×640. 측방 B1은 입력 중단 시 실행용이며,
  이를 3뷰 family 유효성 기준을 완화하는 근거로 사용하지 않는다.
- 출력은 drivable background/drivable 및 marking background/white_lane/yellow_lane/stopline.
  해상도에 따라 두 head가 나뉜다는 의미가 아니다.
- TensorRT10.13.0.35·RTX4060Ti8GB·FP16. ONNX opset17, TF32 비활성화,
  attention/AdaptiveAvgPool/decoder 근사 없음. 별도 plugin 없음.
- YOLO E8·confidence0.40·NMS0.50, YOLO/TwinLite15Hz·ResNet10Hz를 유지한다.

공식 [TwinLiteNet+ 논문](https://arxiv.org/html/2403.16958v3#S4.SS3)의 TensorRT 배포는
변환 가능성의 근거다. 현재 MORAI 4종 head의 품질이나 본 PC 지연을 보증하지 않는다.

## 구현 경계

`runtime/tensorrt.py`는 shape/dtype/engine SHA 검사, 모델별 context/stream과 재사용 출력
버퍼를 담당한다. YOLO의 기존 head/NMS/좌표 복원은 detector에 유지하고 TwinLite의
head 해석은 `twinlite_tensorrt.py`가 담당한다. 자동 PyTorch fallback은 없다.

T1은 기존 전처리와 TensorRT logits 출력을 사용한다. T2는 GPU uint8 업로드·기존 area
보간, graph 내 softmax/max와 pinned D2H 버퍼를 사용한다. 입력 영상은 수정하지 않는다.
전송 결과는 host staging buffer와 별도로 소유해 다음 호출이 이전 결과를 변경하지 않는다.
원본 크기 복원은 배치 시작 시 구독 상태를 고정해 필요한 출력만 수행하며, model-grid
class/confidence는 기존 차선 후단에 우선 공급한다. class 최근접/확률 선형 복원과
argmax→resize 순서를 유지한다. YOLO와 normalized tensor를 무조건 공유하지 않는다.

CPU preprocess/inference/postprocess와 CUDA event의 input copy/engine/reduction/D2H를
구분한다. `inference_ms`는 모델 순수 forward가 아니다. Engine이 reduction을 포함하는지
기록한다. 최신 B1/B2별 GPU 시간은 diagnostic의 `latest_backend_stage_ms`로 제공한다.
갤러리의 CUDA stage 표는5Hz 진단 snapshot을 집계한 값이다. 입력 copy는 정규화된
tensor를 engine 입력 버퍼에 복사하는 구간이며, 원본 uint8 H2D·resize 전체 시간과
동일하지 않다. CPU 전처리 시간에는 GPU 연산 enqueue 시간이 포함되므로 GPU 실행
시간으로 해석하지 않는다. 입력 수신→발행 분위수는 별도의 원시 프레임 계측이다.

계측 prefix와 선택적 lifecycle trace는 4,096개 이하의 비동기 journal을 사용한다.
기록 손실 또는 journal 종료 누락은 시험 실패다. GC를 끄거나 freshness를 늘리지 않는다.
전체 run을 producer 메모리에 보관하지 않는 변경은 T0/T1/T2에 동일하게 적용한다.

## 오프라인 품질

고정 v6 validation 468시점·1,404장(세 뷰), SUNNY/FOGGY×11/13/15시의 6개 run을
사용했다. 같은 run의 인접 영상이므로 468개의 독립 장면으로 설명하지 않는다.
평가 manifest SHA는 `4add24d993fabacb4a99f9d36caaee7c7d4ffc7c391e33d4c5d087a4d64092d4`다.

| 구성 | 영상별 확률 차이 p99의 최댓값 | 최소 class 일치율 | run×view GT IoU 최대 하락 |
|---|---:|---:|---:|
| T1 FP16 logits |0.013933|99.9863%|0.1364%p|
| T2 FP16 reduced + GPU 입력 |0.013875|99.9867%|0.1530%p|
| 허용 기준 |≤0.02|≥99.5%|≤0.5%p|

두 구성 모두 통과했다. GT 없는 종류는 IoU 수락 근거에서 제외했다.
ONNX는 PyTorch FP32와 frontB1/sideB1/B2를 독립 CPU runtime으로 비교했다.
FP16 engine 생성과 품질 검증 중 MORAI·실시간 추론은 종료했다.
최종 CPU/ROS 회귀419개, catkin build/install, 문서·P1-B·perception 계약 검사 통과.
`ASMC_MODELS_ROOT=/nonexistent`에서도 install/share 아래 YOLO 두 프로필과 TwinLite
checkpoint/engine SHA를 검증했다. 검증 커밋별 CI는 증거의 verification.json에, 최신 상태는 PR #85 checks에 기록한다.

YOLO 공통 runner는 고정 val205의 E8/사전학습 두 모델, 총 410개 입력으로 이전
실행 경로와 대조했다. 박스 IoU≥0.98·confidence 차이≤0.02를 통과했다.
첫 시도에서 옛 D 드라이브 경로를 찾지 못한 실패는 보존하고, 이미지 SHA를 확인한
기존 val205 mirror로 재실행했다. 평가 영상을 새로 선정한 것이 아니다.

## 저장 영상 선별

MORAI·GUI를 끄고 세 모델을 함께 실행한 60초 대조다. 입력 공급과 판제 부하는
실제 MORAI 시험과 다르므로 이 표만으로 운영 수락을 판단하지 않는다.

| 구성 | TwinLite 전방 Hz | p50 / p95 / p99 ms | 카메라 CPU 합계 | 전체 GPU VRAM |
|---|---:|---:|---:|---:|
| T0 PyTorch |15.0|43.79 / 63.52 / 85.21|251.59%|2,233MiB|
| T1 TRT logits |15.0|36.58 / 41.06 / 42.80|245.90%|2,110MiB|
| T2 TRT reduced·GPU 입력 |15.0|35.87 / 39.42 / 40.58|233.09%|2,856MiB|

CPU는 여러 프로세스·코어의 합계라 100%를 넘는다. T2의 지연은 낮지만 GPU 입력
캐시로 VRAM 사용은 늘었다. 구독하지 않는 원본 크기 복원·직렬화는 생략하며,
현재 backend의 축약된 drivable/marking D2H는 그대로 수행한다.

T3 공유 입력은 CPU 198.51%로 T2 대비 약14.8% 감소했지만 YOLO p95 악화가
뷰별5.10–5.44ms로 합의한5ms를 넘었다. 선별 1회 결과로 기본 채택하지 않고
독립 프로세스를 유지한다. 전체 조합 반복으로 확대하지 않는다.

T2 CPU 입력 대조에서는 최대479ms 공백이 있었다. 시험 중 호스트 파일 탐색도
있었으므로 CPU 전처리만의 인과 효과로 단정하지 않는다. 실패 실행을 보존한다.

## MORAI·전체 시각화 반복

SUNNY/FOGGY 각각 T0/T2를 3회씩 교차 실행했다. 아래 p50/p95/p99는 각 날씨의
3회 원본 프레임을 합쳐 계산한 분위수이며, 실행별 중앙값의 평균이 아니다.
Main preview와 기존 LiDAR 부하를 포함하고 차량 제어 출력은 시험 경로로 격리했다.
기존 센서 bridge와 사용자 프로세스는 유지했으며 매 회차 rqt 1개와 종료 후0개를 확인했다.

| 날씨·구성 | TwinLite 전방 p50 / p95 / p99 ms | YOLO 전방 p95 ms | ResNet p95 ms |
|---|---:|---:|---:|
| SUNNY T0 |94.56 / 135.57 / 149.80|51.03|90.17|
| SUNNY T2 |54.27 / 93.15 / 108.96|49.06|91.68|
| FOGGY T0 |92.82 / 133.65 / 144.24|49.67|91.44|
| FOGGY T2 |52.83 / 92.40 / 109.70|47.39|90.30|

모델 지연 개선과 전체 카메라 수락은 구분한다. T2 SUNNY r1에서 무효 family 2개와
유효 family 최대공백252.95ms, r2에서 source age355.4ms의 readiness 저하가 있었다.
FOGGY r3에서는 ACTIVE 측정 시작 후 약6.1초 동안 무효 family89개가 발생했다
(`too_few_observed_roles`75개, `insufficient_local_multi_role_support`14개).
그 실행의 유효 family 최대공백은3,354.57ms였다. 시작 부근이라는 이유로 제외하지 않는다.
T0 SUNNY r2/r3에도 `source_stale` readiness 저하가 있었다.

현재 확인된 것은 모델 공급과 후단 유효성의 실패 지점이다. FP16 수치 변화,
날씨 전환, family 시간 처리 중 어느 것이 원인인지 아직 분리되지 않았다.
같은 RGB·pose의 후단 재생으로 원인을 추가 분리해야 하며 기본 전환은 보류한다.

## 30분 연속 시험과 최종 판정

SUNNY·전체 GUI·Main preview·기존 LiDAR 부하에서1,800초를 완료했다.

| 출력 | 평균 Hz | 최저 완전5초 Hz | p50 / p95 / p99 ms | 최대공백 ms |
|---|---:|---:|---:|---:|
| YOLO 전방 |14.91|14.4|30.21 / 45.65 / 52.34|193.15|
| TwinLite 전방 model-grid |14.82|13.4|52.13 / 94.38 / 112.02|590.50|
| ResNet |9.96|9.8|58.59 / 92.31 / 102.35|174.08|
| 유효 lane family |14.43|12.6|133.01 / 193.76 / 220.52|643.70|

Family 지연은 입력 callback→별도 subscriber 수신이며 모델 발행 지연과 다르다.
TwinLite 측방 p95는98.75/99.48ms, 최대공백590.16/590.08ms였다.
전체 VRAM 최고4,264/8,188MiB(52.08%), 카메라/preview CPU합계 평균481.22%,
RSS최고3,639.68MiB다. OOM·추론/디코딩 오류·중복/역행 stamp·기록 손실은0이었다.
객체 합성 인코딩과 발행17,627회가 일치했고 중간/개별 추가 JPEG 인코딩0회,
뷰 디코딩51,037회로 기존 합성 캐시 계약을 통과했다.

그러나 수락은 실패다. ACTIVE204.22초의 lane 프로세스 generation2 GC가532.98ms
걸렸고, 같은 구간 lane 입력 callback 공백543–550ms와 출력공백590ms가 연결된다.
GC 호출 위치는 `LaneBoundaryFamily.deserialize`, 수거 객체는0개였다. 같은 카메라의
YOLO callback은 그4초 구간에서 최대105–106ms 간격으로 계속 수신됐다.
따라서 MORAI 렌더링 문제라고 단정하지 않는다. Lane readiness false2개 표본은
`model_not_prepared`였고, 연속 시험에서 발행된 family25,979개 자체는 모두 valid였다.
readiness 표본2개를 독립 장애2건으로 해석하지 않는다.

**기본은 독립 PyTorch를 유지하고 TensorRT는 명시적 실험 후보로 제공한다.**
추론 속도·분할 품질은 개선/유지됐지만250ms 공백과 readiness0 기준을 충족하지 못했다.
공유 입력·TensorRT 기본 승격·PR Ready·병합은 하지 않는다. 다음 검증은 lane 프로세스의
객체 수명/GC 스캔 비용과 family 역직렬화·유효성 경계에 집중한다. GC 비활성화,
freshness 완화 또는 실패 구간 제거로 통과시키지 않는다.

전체 원본은 `data/runtime/camera-twinlite-trt-v1/`, 측정 당시 source는 그 아래
`measured-source/`에 보존한다. 측정 후 변경은 선택적 lifecycle trace의 기존 종료
표식 호환, 구형 기준선의 새 모듈 부재에 독립적인 측정기 import와 보고·검증 코드다. 추론 수학·입력·모델 설정은 바꾸지 않았다.
추적 종료는 예전 `trace_end/monotonic` 계약을 유지하며 분석기 회귀를 추가했다.
실험 계측 journal의 종료는 기존 `journal_end/at`로 유지한다.

요약: [기계 판정](../../../config/baselines/evidence/camera_twinlite_tensorrt_v1/summary.json),
[검사·CI 상태](../../../config/baselines/evidence/camera_twinlite_tensorrt_v1/verification.json),
[E352 갤러리](http://127.0.0.1:8765/camera-twinlite-trt-20261015-r1/index.html).
날씨는 원래 CLOUDY(type1)로 복원했고, 시험 후 기존 bridge·TF·rosout4개 등록만 남았으며 시험 rqt는0개다.

## 실행·복구

일반 사용의 `camera.launch`·`view.launch` 옵션은 그대로다. 현재 canonical 설정은
PyTorch이며, 검증 담당자는 private launch와 YAML로 후보를 선택한다.

```bash
roslaunch asmc_camera _runtime.launch mode:=active lane_runtime_config:=$(rospack find asmc_camera)/config/camera/lane_runtime_tensorrt.yaml
```

기존 실행을 종료한 뒤 위 명령을 사용한다. 동시에 두 생산자를 켜지 않는다.
복구는 후보 실행을 종료하고 일반 `roslaunch asmc_camera camera.launch mode:=active`다.
시각화는 별도 `roslaunch asmc_camera view.launch`이며 rqt는 한 개만 실행한다.

```bash
rosrun rqt_image_view rqt_image_view --clear-config /perception/camera/objects/debug/mosaic/compressed
```

변환 도구는 `tools/perception/prepare_twinlite_tensorrt.py`다. 출력 디렉터리는 새 경로여야
하며 `--output-mode logits/reduced`를 명시한다. 생성 manifest에는 source/ONNX/engine
SHA·EMA 선택·head·환경·실제 shape·실패 이유를 남긴다. 공유 설치를 바꾸지 않고
onnxruntime1.19.2를 시험 전용 PYTHONPATH에 설치했다.

아래 명령은 변환 의존성과 고정 upstream이 준비된 환경에서 MORAI·추론을 종료한 뒤
실행한다. `PYTHONPATH`에는 카메라 Python 모듈과 격리 설치한 onnxruntime이 있어야 한다.
`--output`은 기존 산출물 경로를 재사용하지 않는다.

```bash
python3 tools/perception/prepare_twinlite_tensorrt.py \
  --checkpoint src/asmc_camera/models/lane_segmentation/twinlite-medium-v6.pt \
  --output /data/models/twinlite-trt-new-build \
  --output-mode reduced --precision fp16
```

TensorRT 추론에는 ONNX export 도구나 upstream 모델을 다시 실행하지 않는다.
PyTorch 복구와 재변환에는 기존 고정 TwinLiteNet+ source 의존성이 필요하다.
배포 파일 복사만으로 다른 GPU·TensorRT 버전의 호환성까지 보장하지 않는다.
