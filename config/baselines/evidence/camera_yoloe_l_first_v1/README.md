# YOLOE-26l native 3뷰 첫 실행 증거

시작점은 [결과와 직접 실행 안내](../../../../docs/handoffs/perception/camera-yoloe-l-first-run.md)다.
이번 범위는 사전학습 l 모델의 42/27종 prompt·TensorRT·batch 실행 문제 확인이다.
학습·운영 승격·대회 전체 스택 승인·3뷰 mask fusion은 포함하지 않는다.

## 읽는 순서

1. [baseline](../../perception_camera_yoloe_l_first_v1.yaml): 선정과 수락 보류.
2. [summary.json](summary.json): 36회 전부와 정상 조건의 33회 집계. 20Hz와 10Hz를 구분.
3. [condition-deviation.json](condition-deviation.json): 첫 base27 r1의 CPU 갤러리 렌더링 중첩.
   결과를 지우지 않고 성능 근거에서 제외했다. 정상 r2/r3/r4는 전부 제출했다.
4. `quality-*-summary.json`: 선정 GT 2252개에서 실제 각 prompt·runtime 추론 결과.
   `animal` GT 부재, 미확인 subtype, heldout 미사용을 함께 읽는다.
5. [rich42-engine-check.json](rich42-engine-check.json), [base27-engine-check.json](base27-engine-check.json):
   실제 동일 영상의 PyTorch/FP16 box·label·mask 대응과 모든 미대응 결과.
6. [diagnostic-analysis.json](diagnostic-analysis.json): trace ON admission·LiDAR 처리·반복 분류 차이.
   정상 trace OFF 결과의 대체 근거가 아니다.
7. [manifest.json](manifest.json), `*.provenance.json`, [raw-index.json](raw-index.json):
   실제 코드 byte hash·bag·GT·weight·text encoder·prompt·ONNX·engine·버전·원본 기록 SHA.
8. [legacy-parity.json](legacy-parity.json): 기존 YOLO11m mask OFF 3514개 완전 일치.

## 원본과 비교 갤러리

`numeric-raw.tar.gz`는 모든 반복의 protocol/index/report/resources/observations,
네 quality 실행의 predictions.jsonl, lifecycle trace와 검증 로그를 담는다.
전체 수치에서 유리한 반복만 선택하거나 미대응 관측을 성공으로 간주하지 않았다.
archive SHA는 baseline에 고정했다. predictions의 mask 경로는 아래 로컬 증거를 가리킨다.

전체 원본은 상위 workspace `artifacts/camera-yoloe-l-first-v1`에 남아 있다.
상세 fusion point trace와 Capture RGB·instance mask 파일은 Git archive에 포함하지 않았다.
RGB가 포함된 브라우저 갤러리는 다음 HTML을 연다.

- `comparison-gallery/index.html`: 실제 PyTorch 0.7 대조.
- `comparison-gallery-030/index.html`: 실제 PyTorch 0.3 대조.
- `comparison-gallery-trt-030/index.html`: 실제 TensorRT 0.3 대조, 119개 동일 프레임 쌍.

GT는 파랑, 결과·instance mask는 초록, 전방 보넷 제외 영역은 빨강이다.
근거리 차량/사람·미검출 장애물·유사 이름 중복·구조물 관측·시간 균등 표본을 함께 담았다.
미확인 Obstacle subtype을 wooden box GT로 바꾸지 않았으며 GT 없는 신규 종류의 recall은 없다.
Capture source timestamp가 확인되지 않아 최초 검출·공백·label 전환은 frame 단위다.
원본 actor 기록은 `comparison-gallery-trt-030-continuity.json`을 함께 본다.

## 재현 환경과 순서

고정 이미지는 `asmc/camera-yoloe-l:ultralytics8.4.150-trt10.13.0.35`이고 digest는 manifest에 있다.
측정은 `asmc-camera-yoloe-l-study`, network none, GPU all, ROS master 11318에서 직렬로 수행했다.
`asmc-ros-noetic`과 공유 master/bridge/GUI를 변경하지 않았다.
모델·prompt·engine은 `data/models/camera_yoloe_l_v1`에 있다. 다른 GPU/버전은 재빌드한다.

공식 checkpoint 출처:
<https://github.com/ultralytics/assets/releases/download/v8.4.0/yoloe-26l-seg.pt>.
checkpoint SHA `a612d2d505f24e14d87ec82d688b823b6cb600646664f16125ce6c84ce360da9`.
text encoder `mobileclip2_b.ts` SHA `35d7f213e4d75f38514e4656ad3cb91158bd33e3805d8ac349f23b186f66982f`.
이전 89종 cache를 재사용하지 않고 l checkpoint에서 base27/rich42를 각각 생성했다.

export는 `prepare_yoloe_l.py --directory /prepared --config
/root/ws/config/experiments/camera_yoloe_l_first_v1.yaml --profile rich42 --stage prompts`,
`--stage onnx`, `--stage engine` 순서로 진행했다. base27도 같은 순서로 실행했다.
prompt는 export 전에 고정한다. ONNX는 nms=None, 실행은 nms=True로 원래 one-to-many head를 유지했다.
head가 달라졌던 첫 export는 로컬 `previous-end2end-export`와 로그에 보존했다.
내부 FP16 허용과 I/O FP32를 구분하며 그룹별 중복 제거는 추가하지 않았다.

`write_yoloe_l_configs.py`로 SHA를 고정하고 quality 평가기 `--batched`,
`check_yoloe_l_engine.py`로 출력을 확인했다. 결과 폴더는 덮어쓰지 않는다.
그다음 `run_yoloe_l_trials.py --jobs ... --output ...`로 저장된 protocol을 실행했다.
rich42 세 batch×3회 → base27 .7 및 rich42 .3 구성별 3회 → 주·음영 bag×3회 →
조건 확인 r4 → trace ON 진단 2회 순서다. GUI/recorder는 OFF, debug ON 3회는 별도 구성이다.
Main의 기존 fusion debug는 운영 코드 변경 없이 유지했으므로 debug 계산이 완전히 없다고 주장하지 않는다.

마지막으로 `analyze_yoloe_l_diagnostics.py --results ...`, `summarize_yoloe_l.py --results ...`,
`fingerprint_yoloe_l.py`로 분석과 SHA를 고정했다. 실행 코드 기준은 0a90b34이고
진단 wrapper 추가는 protocol과 실행 source hash에 기록했다.
후속 분석 코드 추가를 측정 당시 runtime 변경으로 해석하지 않는다.

## 보류 사항

- 엄격한 20Hz는 미달이며 최소 10Hz만 통과했다. debug ON은 약 16.6–16.9Hz까지 줄었다.
- animal GT가 없으며 작은 물체·원거리·가림에서 미검출이 남았다. 전체 대회 에셋 지원은 미확인이다.
- FP16의 일부 box/label/mask 대응 차이를 완전히 동등하다고 수락하지 않는다.
- 주 bag의 동일 관측에서 unknown→car 반복 차이 1건을 확인했다. 이전 2건 승인과 구분한다.
- fusion은 기존 네 클래스·conf.7·전방 bbox만 사용한다. 새로운 클래스/mask fusion은 미구현이다.
- 이전 YOLO11m C1과 구성 차이가 있어 matched 절대 수 동등성은 승인하지 않았다.
- camera/msgs/lidar 295시험 중 1건은 기존 facade 실패다. 검출58·측정/분석5·nested 대상296은 통과했다.
  nested 전체 vendor 시험의 외부 pcap download 중단도 로그에 보존했으며 전체 vendor 시험 통과로 표시하지 않는다.

PR은 Draft다. 결과는 추가학습·prompt 정리·20Hz scheduling/debug 개선의 다음 판단에 사용한다.
