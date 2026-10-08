# YOLOE-26l 첫 실행: 27종/42종과 native 3뷰

이번 작업은 추가학습 전 문제 확인이다. 운영 YOLO11m 기본값, 메시지/MD5,
LiDAR 어댑터의 네 클래스/confidence 0.7, 50ms 매칭·200ms 대기는 유지한다.
작업 폴더는 `asmc-camera-objects`, 브랜치는 `feat/camera-yoloe-l-first-run`이다.
PR #74의 s 모델 결과를 보존하고 별도 후속 변경으로 관리한다.

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

## 직접 실행

TensorRT와 모델 파일이 준비된 실험 runtime 환경에서 ROS와 빌드 환경을 읽는다.
공유 컨테이너에 TensorRT를 자동 설치하거나 기존 bridge/master를 교체하지 않는다.
컨테이너 진입과 bridge는 사용자가 직접 실행한다. 실시간 master에서는 sim time을 false로 사용한다.

```bash
roslaunch asmc_camera camera_objects_yoloe_l_experiment.launch
```

기준 프롬프트 비교는 첫 launch를 종료하고 실행한다.

```bash
roslaunch asmc_camera camera_objects_yoloe_l_experiment.launch prompt_profile:=base27
```

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

이 결과 이후 prompt 정리/추가학습 또는 l/960·m/1280·10Hz/20Hz 비교의 우선순위를 정한다.
이번 변경은 운영 기본값 승격, 학습, 판제의 구간/회피 정책 변경을 포함하지 않는다.
