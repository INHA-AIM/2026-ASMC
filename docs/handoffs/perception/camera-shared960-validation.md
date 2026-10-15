# B1/B2 입력 통일·배포 모델·공유 입력 검증 — 2026-10-15

현재 작업은 PR #85다. 기본 실행 구조는 모델별 독립 프로세스이며, 공유 디코딩/GPU 후보는
선별 기준을 충족하지 못해 기본으로 승격하지 않는다. TwinLite 좌우480 입력은 품질 개선이
확인된 PR 내 후보다. 기존 main의 수락을 새 입력의 수락으로 설명하지 않는다.

시작점은 [실행 계약](../../contracts/perception/camera-runtime-v1.md),
구조 경계는 [공유 입력 설계](camera-shared-input-design.md)다.
이전 [전방960 기록](camera-front960-validation.md)과 실패 원본을 보존한다.

## 배포 파일

두 canonical YOLO YAML은 package-local engine을 참조한다. 실제 파일4개를 일반 Git에
포함하며 원본 `/data/models`와 기존 release는 보존한다. 원본/배포 SHA와 크기는
[배포 목록 v2](../../../config/models/camera_object_models_v2.json)에 있다.

| 파일 | 배포 bytes | 처리 |
|---|---:|---|
| E8 checkpoint |53,073,858|원본316,458,898bytes에서 EMA와 출처만 보존|
| E8 engine |56,610,526|원본 바이트 유지|
| 사전학습 checkpoint |53,211,173|COCO80 원본 바이트 유지, 실행은7종 선택|
| 사전학습 engine |57,496,998|원본 바이트 유지|

모든 EMA parameter/buffer가 재로딩 후 같고, CPU64×96 forward의11개 tensor도 원본과
완전히 같다(max absolute difference0). 가중치 갱신은 하지 않았다. 배포 checkpoint는
optimizer/scaler/중복 training_model/scheduler_state가 없으므로 재개용으로 사용하지 않는다.
`torch.save`는 file object에 써서 임시 파일명에 따라 ZIP 내부 경로가 바뀌지 않게 한다.

별도 검증 workspace에서 catkin build/install을 수행했다. `ASMC_MODELS_ROOT=/nonexistent`
조건에서도 install/share/asmc_camera/models의 실제 engine이 해석된다. 지원 환경은
Linux x86_64·RTX4060Ti8GB·TensorRT10.13.0.35·asmc-ros-noetic이다. 다른 환경에서는
제공한 checkpoint로 engine을 재생성·검증해야 하며 자동 PyTorch fallback은 하지 않는다.
TwinLite checkpoint/upstream 소스와 ResNet의 기존 설치 의존성은 별도다.

## 입력과 실행

| 배치 | YOLO | TwinLite 후보 |
|---|---|---|
| B1 전방 |1×3×544×960|1×3×544×960|
| B2 좌·우 |2×3×480×640|2×3×480×640|

네트워크나 checkpoint는 바꾸지 않았다. 기존 측방384 입력은 lane_input_side384.yaml에
보존한다. 좌우 tensor 픽셀은25%, 세 뷰 합계는약12% 늘어난다. 기존 측방 유효 영상은
512×384였으므로 새640×480에서 실제 장면 픽셀은56.25% 늘고 좌우 padding은 사라진다.
추론과 model-grid 투영이 같은 입력 설정을 사용하고 센서 보정·원본 좌표·header는 유지한다.

독립/공유 후보 모두 같은 engine을 직접 실행한다. 기존 Ultralytics head·NMS·박스 clipping과
원본 좌표 복원을 유지하면서 매 frame의 Results/Boxes 객체 생성을 제거했다.
TensorRT execute_async_v3와 모델별 CUDA stream을 사용한다. 큰 device-wide synchronize를
추가하지 않는다. engine 내부 FP16과 binding dtype은 구분하며 실제 I/O dtype을 검사한다.

공유 후보는 한 JPEG 구독·읽기 전용 decoded cache와 뷰당2개 GPU 원본 슬롯을 사용한다.
키는 topic·camera·입력 epoch·stamp·sequence다. lease와 완료 event를 함께 확인하고
모델별 mask·보간·정규화·dtype·layout이 다르면 normalized tensor를 공유하지 않는다.
YOLO mask는 원본 크기에서 적용하며 TwinLite 입력에는 적용하지 않는다.
mask/좌표 상수 cache도 제한하고 stream 간 event로 준비 완료를 연결한다.
ResNet camera4는 공유 후보에서도 독립 프로세스로 유지한다.

## 저장 영상 품질

val205에서 E8/사전학습을 각각 비교한410건 모두 기존 Ultralytics TensorRT predictor와
직접 실행 경로의 대응 IoU≥0.98·confidence 차이≤0.02 기준을 통과했다. 이는 같은 engine의
실행/후처리 비교이며 새로운 PyTorch 대 TensorRT 품질 수치로 설명하지 않는다.

SUNNY/FOGGY×11/13/15시의6개 기존 validation run에서18개 시점·54장을 고정했다.
같은 RGB에서 GPU 전처리/CPU 전처리의 YOLO108건 모두 기준을 통과하고 TwinLite54장도
같은 출력을 냈다. 연속 관측을54개의 독립 장면으로 세지 않는다.

| Native GT 지표 | 기존 측방384 | 측방480 |
|---|---:|---:|
| 좌측 drivable IoU |97.80%|97.97%|
| 좌측 흰 차선 IoU |45.88%|63.66%|
| 좌측 황색 차선 IoU |85.40%|91.98%|
| 좌측 stopline IoU |66.69%|72.65%|
| 우측 drivable IoU |98.11%|98.00%|
| 우측 흰 차선 IoU |83.62%|85.76%|
| 우측 stopline IoU |31.72%|37.42%|

전방 입력은 같아 지표도 같다. 우측 황색 차선은 이 부분집합에 GT가 없어 평가 불가다.
이 선별 결과를 전체 대회 조건의 차선 수락으로 확대하지 않는다.

## 공유 효과와 선택

같은 JPEG·같은 frame을 두 모델이 소비하는 직렬30회 A/B/A 진단에서 디코딩180→90회,
대형 영상 H2D120→90회, 전송511,488,000→138,240,000bytes(-72.97%)를 확인했다.
작은 mask/좌표 상수의 최초 업로드는 영상 전송 집계에서 제외한다. CPU staging copy와
모델별 tensor/배치 생성은 남아 있으며 완전한 zero-copy로 설명하지 않는다.
두 독립 worker가 다른 frame을 소비하는 live에서는 실제 cache hit 수를 별도로 기록한다.

MORAI·세 모델·Main preview·기존 LiDAR 부하·전체 GUI를 포함해 선별했다. 차량 제어는
시험 토픽으로 격리하고 기존 Ego/actor는 변경하지 않았다. 첫 공유 GPU 구성은 TwinLite
약7Hz로 실패했다. stream 분리 후11–12Hz로 회복했지만 지연 기준과 다른 모델 악화 기준을
충족하지 못했다. 공유 디코딩만 사용한 후보도 실패해 독립 구성을 유지한다.

| 60초 조건 | 카메라 CPU% | TwinLite 전방 Hz | TwinLite p95 ms | YOLO p95 ms |
|---|---:|---:|---:|---:|
| 독립·SUNNY·r1 |417.6|12.75|142.9|48.4|
| GPU 공유·SUNNY |355.0|11.18|166.0|69.3|
| 디코딩 공유·SUNNY |349.5|10.98|162.6|53.2|
| 독립·FOGGY·r1 |419.1|13.38|133.5|47.6|
| GPU 공유·FOGGY |348.1|11.82|155.2|63.4|
| 디코딩 공유·FOGGY |별도 상세표|10.50|169.2|63.1|

CPU100%=1코어이며 camera/debug/Main preview 프로세스 합이다. 전체 GPU에는 MORAI와
Windows 부하가 포함된다. serial 진단에서 이득이 있어도 live 수락을 대체하지 않는다.
R1은 SUNNY/FOGGY 각각3회 측정했다. 전방 TwinLite12.6–13.4Hz·p95≤142.9ms였지만
FOGGY r3에서 YOLO 공백452.5ms가 있어 단기 전체 수락도 완료로 표시하지 않는다.
이전 direct-v2 SUNNY 대조에는 GC944ms와 YOLO callback 공백의 일치가 확인됐다.
모든 실패를 보존하며 GC 활성 상태/threshold와 freshness350ms는 변경하지 않았다.

## 30분 연속 결과와 남은 차단

30분 측정은 완주했고 자원 샘플3599개로 coverage를 통과했다. CPU평균406.6%,
RSSpeak3389.8MiB, 전체 VRAMpeak60.10%다. rqt는 매 실행 전0·중1·후0이며
날씨는SUNNY로 복원됐다. 시험 노드는 제거되고 기존 bridge만 남았다.

| 출력 | 평균 Hz | 최저5초 Hz | p50/p95/p99 ms | 최대 공백 ms |
|---|---:|---:|---|---:|
| YOLO 전방 |14.879|14.0|33.04 /50.52 /59.13|393.49|
| YOLO 좌측 |14.827|14.2|33.16 /50.89 /59.33|394.31|
| YOLO 우측 |14.850|14.2|33.58 /51.47 /60.16|394.48|
| TwinLite 전방 |12.385|8.0|102.46 /144.10 /159.22|365.68|
| TwinLite 좌측 |12.385|8.0|104.46 /146.37 /161.52|365.56|
| TwinLite 우측 |12.385|8.0|105.18 /147.02 /162.92|365.58|
| ResNet |9.928|4.4|57.59 /93.14 /104.20|3078.99|

위백분위는 원시 프레임의 callback→발행 지연이다. 평균/백분위는 좋아도 최대 공백과
완전한5초 창 최소 공급을 통과하지 못했다. 중복·역행stamp는0이다.
차선 readiness false258개 상태 표본의 원인은 source_stale21 /model_not_prepared5 /
invalid_family231 /receipt_stale1이다. 이는258개의 독립 장애 사건을 뜻하지 않는다.
유효 family평균12.036Hz·최저5초0Hz·최대 공백22.664초다. family무효 메시지의 이유는
too_few_observed_roles596개, insufficient_local_multi_role_support26개다. segmentation IoU
개선만으로 차선 family 수락을 대체하지 않으며 후단 기준을 느슨하게 바꾸지 않았다.

GC journal은 YOLO에서ACTIVE71.30초에280.42ms, ResNet에서1017.22초에2997.79ms를
기록했다. ResNet의 입력 callback 공백2998.85ms와 시간적으로 일치한다. 다른 카메라
프로세스의약396ms 입력 공백도 별도 기록됐다. 이를 MORAI 렌더링 문제로 단정하지 않는다.
계측 journal의 장기 RSS 증가와 실제 runtime heap도 후속 대조에서 분리해야 한다.
현재 GC 정책·threshold·freshness는 변경하지 않았으며 원인 미확인 공백을 면제하지 않는다.

전체15회·실패·각 뷰 p50/p95/p99·GT비교는
[E351](http://127.0.0.1:8765/camera-shared960-20261015-r1/index.html)에 있다.
원본은 workspace artifacts/camera-shared960-20261015-r1,
compact 증거는 config/baselines/evidence/camera_shared960_v1에 보존한다.

관련 CPU/ROS387개, 전체 catkin build/install, 고정 계약·문서 검사를 통과했다.
호스트의 계측 시험1건은 rosbag 부재로 실패했고 Noetic 환경에서10건 모두 통과했다.
개발 중 AIM미마운트·이전150장 manifest 선택·engine I/O dtype·보간 차이 실패도 보존한다.
최종 nativeGT/val205 검증에는 수정한205장과 실제 binding dtype·OpenCV 정수 보간을 썼다.

**공유 구성은 탈락, 독립 worker 유지, 좌우480은 검증 후보로 보존, PR Draft 유지**다.
운영 승격·병합 전에 GC/입력 공백과 차선 family 실패를 해결하고 동일 기준을 재검증해야 한다.
