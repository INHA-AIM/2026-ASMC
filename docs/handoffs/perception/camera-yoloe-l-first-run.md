# YOLOE-26l 첫 실행: 27종/42종과 native 3뷰

이번 작업은 추가학습 전 문제 확인이다. 운영 YOLO11m 기본값, 메시지/MD5,
LiDAR 어댑터의 네 클래스/confidence 0.7, 50ms 매칭·200ms 대기는 유지한다.
작업 폴더는 `asmc-camera-objects`, 브랜치는 `feat/camera-yoloe-l-first-run`이다.
PR #74의 s 모델 결과를 보존하고 별도 후속 변경으로 관리한다.

구조물 혼동의 클래스별 검토와 새 17종 후보는
[프롬프트 정리 문서](camera-yoloe-l-prompt-cleanup.md)를 따른다.
이는 새 vocabulary 실제 재추론 전의 후보이며 아래 27/42종 결과를 대체하지 않는다.

## 실행 계약

프롬프트 순서는 [고정 실험 설정](../../../config/experiments/camera_yoloe_l_first_v1.yaml)의
`prompt_profiles`를 따른다. `base27`과 `rich42`는 각각 checkpoint에 맞는 embedding과
engine을 만든다. 42종 결과를 필터링한 것을 27종 추론으로 보고하지 않는다.
`animal`·`scooter`를 포함한 원본 라벨과 네 소비 그룹은 별도 실험 그룹 파일에 고정했다.
구조물·신호등·표지판·노면 요소·fire를 관측해도 자동 회피 대상으로 지정하지 않는다.

| 뷰 | 원본 입력 | 텐서 (높이 × 너비) |
|---|---|---|
| front | 1280×720, 20Hz | 736×1280 |
| left/right | 640×480, 각 20Hz | 각각 480×640 |

`2plus1`은 전방 1 + 측방 2, `sequential`은 각 뷰 1,
`batch3`은 측방 resize를 그대로 유지한 채 736×1280까지 padding한다.
보넷 semantic contour mask는 전방에만 적용한다. 결과 박스와 instance mask는
padding을 제거하고 원본 크기로 복원한다. 박스·mask의 순서와 대응은 함께 유지한다.
JPEG decode는 프레임당 한 번, 최신 프레임 보관은 뷰당 하나, batch 대기는 최대 10ms다.
입력 중단 뷰를 기다려 다른 뷰를 정지시키지 않는다. 결과 복제/재발행으로 Hz를 높이지 않는다.

## 실제 checkpoint head와 export

고정 환경은 Ultralytics 8.4.150, Torch 2.1.2+cu118이다.
공식 `yoloe-26l-seg.pt`의 현재 head는 one-to-many다.
이 버전의 `predict(nms=False)`와 `export(nms=False)`는 end2end head를 선택한다.
checkpoint에 one-to-one 분기가 없으므로 그 요청을 그대로 사용하지 않는다.

이번 실험은 `predict(nms=True)`로 원래 one-to-many + 클래스별 NMS를 사용한다.
ONNX는 `export(nms=None)`로 raw one-to-many를 내보내고, 동일한 runtime NMS를 적용한다.
IoU 0.7, max_det 300, agnostic_nms false를 고정한다. 유사 이름 간 별도 그룹 NMS는 없다.
이 설정은 [공식 exporter 구현](https://github.com/ultralytics/ultralytics/blob/v8.4.150/ultralytics/engine/exporter.py)을
확인한 결과다. head가 바뀐 첫 export 시도도 별도로 보존한다.

TensorRT builder는 FP16 내부 연산을 허용하고 I/O dtype을 별도로 기록한다.
입력이 FP32이면 이를 FP16 I/O라고 부르지 않는다. workspace는 1GiB,
dynamic profile은 min `(1,3,480,640)`, opt `(2,3,480,640)`, max `(3,3,736,1280)`다.
Ultralytics 기본 dynamic export의 더 큰 최대 영상 크기를 그대로 사용하지 않는다.
다른 GPU/라이브러리 버전에는 engine 재빌드와 재검증이 필요하다.

## 도구와 증거

- `prepare_yoloe_l.py`: prompts → ONNX → engine. stage별 SHA/버전/shape 기록.
- `check_yoloe_l_engine.py`: 동일 영상의 저장된 PyTorch 결과와 TensorRT boxes/masks 비교.
- `evaluate_camera_seg_models.py --batched`: native 뷰 packing으로 선정 GT를 실제 추론.
- `run_yoloe_l_trials.py`: 명시적 고정 JSON protocol을 새 격리 master에서 직렬 실행.
- `yoloe_l_measurements.py`: 기존 18Hz 운영 gate와 구분한 10Hz 첫 가능성 gate.
- `build_yoloe_l_gallery.py`: 실제 base27/rich42 동일 영상, 미검출·중복·근거리·구조물 검토.
- `fingerprint_yoloe_l.py`: 코드·bag·모델·prompt·engine·GT 실제 SHA와 고정 환경.
- `summarize_yoloe_l.py`: 모든 실행을 보존하고 조건이 맞는 반복만 별도 집계.
- `analyze_yoloe_l_diagnostics.py`: trace ON의 입력/주기 제한/발행과 LiDAR 처리·반복 분류 대응.

원본 증거는 상위 workspace `artifacts/camera-yoloe-l-first-v1`, 모델/engine은
`data/models/camera_yoloe_l_v1`에 둔다. 전체 checkpoint·engine을 팀 Git에 vendor하지 않는다.
GT는 기존 `gt-v2`의 선정 split만 사용한다. 학습/heldout은 평가하지 않는다.
GT의 미확인 세부 종류를 prompt 이름으로 역라벨링하지 않는다.
Capture에는 검증된 source timestamp가 없으므로 actor 최초 검출·공백은 frame 단위로 기록한다.
GT 없는 신규 이름은 갤러리 관측이며 recall을 계산하지 않는다.

각 뷰의 완전한 5초 창 ≥10Hz, source age p95≤150ms/p99≤200ms,
최대 공백≤250ms와 stamp 중복/역행 0을 검사한다. 10–19Hz는 20Hz 목표 미달로 표시한다.
전체 VRAM≤85%는 자원 여유 목표로 별도 기록한다. 부하/품질 실패를 버리지 않는다.
모든 모델은 메모리에 상주하며 다른 모델 STANDBY의 비용도 포함한다.
운영 fusion은 전방 bbox뿐이다. 3뷰 instance mask fusion을 구현한 것으로 안내하지 않는다.

## 2026-10-09 첫 실행 결과

[기준선](../../../config/baselines/perception_camera_yoloe_l_first_v1.yaml)과
[수치 증거](../../../config/baselines/evidence/camera_yoloe_l_first_v1/README.md)에
성공·실패·조건 이탈을 함께 남겼다. 측정은 RTX 4060 Ti 8GB, MORAI 종료,
network-none 컨테이너, Ultralytics 8.4.150·Torch 2.1.2+cu118·TensorRT 10.13.0.35였다.

36회 실행 중 정상 조건에 맞는 성능 실행은 33회다. 첫 base27 반복에 갤러리 렌더링이
ACTIVE 첫 6.1608초 겹쳤다. 원본을 보존하고 성능 근거에서 제외했으며,
별도 r4를 포함한 r2/r3/r4 세 회를 정상 대조로 사용했다. 나머지 2회는 trace ON 진단이다.
모든 실행이 최소 가능성 gate를 통과했지만 엄격한 뷰별 20Hz는 달성하지 못했다.

### 배치와 통합 부하

confidence 0.7의 rich42 단독 ACTIVE에서 세 배치를 각각 3회 비교했다.
아래 p95는 반복·뷰 중 가장 큰 source age p95이고 전체 지연에 mask 복원 비용도 포함한다.

| 방식 | 최악 뷰 p95 | ACTIVE 전체 GPU peak VRAM |
|---|---:|---:|
| 전방 1 + 측방 2 | 40.1ms | 3343MiB |
| 순차 batch 1 | 53.2ms | 3336MiB |
| padding batch 3 | 71.2ms | 3385MiB |

최소 10Hz를 모두 통과한 후보 중 p95가 가장 작은 **2plus1**을 선정했다.
선정 confidence 0.3에서 아래 구성은 각각 3회 측정했다. Hz 범위는 반복별 평균 범위이며
수락은 별도로 완전한 5초 창에 적용했다. TwinLite와 ResNet은 나머지 구성에서도 로드돼 있다.

| 구성 | 전방 YOLO | 좌측 YOLO | 우측 YOLO | 최악 YOLO p95 |
|---|---:|---:|---:|---:|
| YOLO ACTIVE | 19.75–19.84 | 18.59–18.70 | 19.84–19.90 | 44.7ms |
| TwinLite + YOLO | 19.80–19.85 | 18.85–18.95 | 19.89–19.90 | 56.7ms |
| ResNet + YOLO | 19.79–19.80 | 18.65–18.80 | 19.80–19.85 | 43.5ms |
| 세 모델, debug OFF | 19.79–19.85 | 18.70–18.84 | 19.79–19.84 | 61.4ms |
| 세 모델, debug ON | 16.64–16.89 | 16.55–16.69 | 16.59–16.79 | 117.3ms |

TwinLite 뷰별/최종 차선과 ResNet은 기존 첫 1초·5초 창·공백 기준을 통과했다.
차선 유효 비율과 공급률은 별도 지표이며 이 도로 자료를 음영 외 차선 소비 승인으로 사용하지 않는다.
정상 실행의 최대 source age p95/p99와 최대 공백은 기준 안에 있었다.
전체 GPU peak는 3528/8188MiB(43.1%)였다. 데스크톱 메모리도 포함하며
노드별 GPU VRAM은 제공되지 않아 측정 불가로 기록했다. 이 수치를 prompt별 모델 메모리 차이로 해석하지 않는다.

trace ON C1의 ACTIVE 20초에는 뷰당 400개 입력이 왔다.
begin은 front397/left372/right399, published는 396/371/398개,
rate_wait는 3/7/1회였다. 마지막 진단의 좌측 대체 카운터는 22개로,
STANDBY/PREPARE도 포함한다. 최신 버퍼 대체와 주기 admission의 영향을 확인했다.
측정 창 끝 진행 중 처리도 별도로 남겼다. 단순히 GPU 모델이 너무 커서 20Hz를 못 낸다고 결론내리지 않는다.

### Prompt 품질과 TensorRT 대응

선정 GT 2252프레임에서 animal GT는 없었다. Vehicle/Obstacle 세부 종류가
미확인인 항목은 그룹 정확도만 평가했다. heldout 평가·추가학습은 하지 않았다.

| 실제 추론 | confidence 0.7 macro F1 | 탐색 최고 macro F1 | 선정 confidence |
|---|---:|---:|---:|
| rich42 PyTorch | 0.3014 | 0.3713 | 0.30 |
| base27 PyTorch | 0.1880 | 0.2560 | 0.25 |
| rich42 TensorRT | 0.3022 | 0.3718 | 0.30 |
| base27 TensorRT | 0.1888 | 0.2563 | 0.25 |

rich42가 이 자료에서는 개선됐으나 실제 객체 recall은 낮고 구조물 관측·유사 label 중복도 남았다.
가까운 wooden box confidence 0.64 사례는 0.7에서 빠지고 0.3에서 검출됐다.
이것은 그 관측 사례이며 모든 나무 상자·미지원 에셋 검출을 보장하지 않는다.
rich42 기본 실험은 유지하되 confidence 0.3은 실험용 명시 인자로만 적용한다.
운영 YOLO11m과 호환 어댑터의 0.7 기본값은 유지한다.

91개 동일 영상의 PyTorch/TensorRT 대조에서 rich42는 437/442개 중 432개,
base27은 272/273개 중 268개가 동일 원본 label·IoU≥0.5로 대응됐다.
대응 mask 평균 IoU는 각각 0.9954/0.9963이지만 최대 좌표 차이는 28.17/18.40px다.
미대응 5/10개와 4/5개도 보존했으며 FP16을 정확히 동등하다고 수락하지 않는다.
precision/NMS 경계와 최악 대응 사례의 추가 검토가 필요하다.

### LiDAR/fusion 범위

| bag | 처리한 LiDAR | 매칭 쌍 | drop | interval/tail p95 중 최대 |
|---|---|---|---|---|
| 주 bag, raw649 | 519 / 519 / 515 | 517 / 512 / 512 | 1 / 6 / 2 | 31.8 / 33.2 / 30.3ms |
| 음영 bag, raw208 | 188 / 190 / 185 | 187 / 189 / 184 | 0 / 0 / 0 | 24.8 / 25.5 / 24.6ms |

각각 종료 pending LiDAR1, YOLO 주1/음영2가 남았다. 종료 집계·단일 발행자·중복 소비·50ms 조건을 확인했다.
OFF 로그의 p95는 interval/tail별 최대이며 전체 pooled p95로 표기하지 않는다.
trace ON 주 bag은 raw649 = 처리452 + pose 실패1 + generation 생략196으로 연결됐다.
처리 시간 p50 83.9ms/p95 150.3ms, 매칭447/drop4/pending1이었다.
trace OFF보다 처리 수가 줄어든 관측 비용을 명시하고 이 실행으로 정상 결과를 대체하지 않는다.

반복끼리 같은 LiDAR stamp와 상호 유일한 1mm 이내 3D 중심을 대응했다.
주 r1:r2 분류 차이0, r1:r3 차이1(unknown→car), 음영은 두 비교 모두0이었다.
미대응 관측과 stamp 집합 차이도 함께 남겼다. 기존 승인된 분류 차이 2건을 새 차이에 적용하지 않는다.
숫자 track ID 변경을 추적 품질로 해석하지 않는다. 예전 YOLO11m C1과 지금의
3뷰·세 모델 C6는 부하·모델·해상도가 달라 매칭 수 동등성을 승인하지 않았다.
현재 연결은 전방의 네 클래스 bbox만 전달하며 42종·3뷰 mask fusion은 미구현이다.

### 이번 작업의 결론과 다음 우선순위

**l/전방1280·3뷰 TensorRT는 최소 10Hz 실행 후보로 유지할 수 있다.**
전방 해상도나 모델을 먼저 낮춰야 할 근거는 이번 성능 자료에서는 없었다.
20Hz 미달에는 latest buffer/주기 제어/시각화 비용이 있으므로 이를 먼저 분리한다.
검출 품질과 TensorRT의 일부 대응 차이, fusion 분류 차이는 수락 보류다.

다음은 (1) 검수 가능한 Capture GT와 teacher 초안으로 상자·근거리·가림 누락 보강,
(2) 유사 prompt와 구조물 오검출을 실제 vocabulary 재추론으로 정리,
(3) 20Hz admission과 debug 비용 개선, (4) 필요할 때 l/960 대 m/1280을 비교하는 순서다.
이번 PR은 첫 실행 도구·증거로 Draft이며 운영 승격·추가학습·4070 SUPER 보장을 포함하지 않는다.

## 직접 실행

TensorRT와 모델 파일이 준비된 실험 runtime 환경에서 ROS와 빌드 환경을 읽는다.
공유 컨테이너에 TensorRT를 자동 설치하거나 기존 bridge/master를 교체하지 않는다.
컨테이너 진입과 bridge는 사용자가 직접 실행한다. 실시간 master에서는 sim time을 false로 사용한다.

TensorRT를 준비한 별도 컨테이너 **`asmc-camera-yoloe-l-live`**는 sleep 상태다.
공유 `asmc-ros-noetic`의 설치 패키지는 변경하지 않았다. 각 추론/GUI 터미널에서 다음을 직접 읽는다.
아래 `/tmp` build는 컨테이너 이미지에 포함돼 있으며 임시 호스트 실행 script가 아니다.

```bash
docker exec -it asmc-camera-yoloe-l-live bash
source /opt/ros/noetic/setup.bash
source /tmp/seg-devel/setup.bash
source /tmp/seg-nested-devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=localhost
```

사용자가 기존 master/bridge를 직접 실행한 뒤, 중복 camera_object_detector launch를 종료하고 실행한다.
실시간 master의 `/use_sim_time`이 true로 남았을 때는 false로 바꾼다.
선정 confidence로 보는 명령은 다음과 같다.

```bash
roslaunch asmc_camera camera_objects_yoloe_l_experiment.launch confidence:=0.3
```

기준 프롬프트 비교는 첫 launch를 종료하고 실행한다.

```bash
roslaunch asmc_camera camera_objects_yoloe_l_experiment.launch prompt_profile:=base27
```

두 prompt의 0.7 고정 대조는 각각 `confidence:=0.7`을 사용한다.
launch의 confidence 기본은 0.7이고, rich42 선정 기준 0.3을 자동 운영 기본값으로 바꾸지 않았다.

필요할 때 `batch_mode:=sequential`, `batch_mode:=batch3`, `debug:=false`,
`runtime_backend:=pytorch`, `initial_mode:=0`를 지정할 수 있다.
기본 직접 launch는 ACTIVE, 자동 시험은 PREPARE 완료 후 ACTIVE다.

```bash
rqt_image_view --clear-config /perception/camera/objects/debug/instances/compressed
```

```bash
rqt_image_view --clear-config /perception/camera/objects/debug/left/instances/compressed
```

```bash
rqt_image_view --clear-config /perception/camera/objects/debug/right/instances/compressed
```

전방 bbox/보넷 overlay는 `/perception/camera/objects/debug/image/compressed`다.
source header·모드 세대는 기존 typed box 경로에 보존한다.
목표 Hz는 launch `target_hz` 또는 외부 `set_inference_mode`로 제어한다.

브라우저 비교 갤러리 원본은 상위 workspace의
`artifacts/camera-yoloe-l-first-v1/comparison-gallery-trt-030/index.html`이다.
`.7`/`.3` PyTorch 대조도 `comparison-gallery`/`comparison-gallery-030`에 보존했다.
같은 관측을 나란히 표시하며 GT 파랑, 결과/instance mask 초록, 전방 보넷 제외 영역 빨강이다.
원본 RGB·mask 파일은 Git에 업로드하지 않고 해당 로컬 증거와 SHA로 연결한다.

이 결과 이후 prompt 정리/추가학습 또는 l/960·m/1280·10Hz/20Hz 비교의 우선순위를 정한다.
이번 변경은 운영 기본값 승격, 학습, 판제의 구간/회피 정책 변경을 포함하지 않는다.
