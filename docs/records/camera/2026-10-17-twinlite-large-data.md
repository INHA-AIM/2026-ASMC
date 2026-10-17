# 2026-10-17 TwinLite Large Capture 데이터 준비

사용자 요청에 따라 MORAI 종료 후 CPU 전수검사와 GPU 입력 검사를 수행했다.
**실제 학습·optimizer update·새 가중치 저장·운영 모델 변경은 하지 않았다.**
도구·실행 방법은 [패키지 데이터 안내](../../../src/asmc_camera/docs/training-data.md),
고정 설정은 [v7 preparation recipe](../../../config/experiments/twinlite_large_capture_v7.json)를 따른다.

## 범위와 정책

기존 TwinLite 수집과 이후 YOLO 자동 스폰 수집의 원본 RGB/Semantic 쌍을 함께 검사한다.
원본은 `E:\morai_dataset\SensorData`, target·manifest는 `E:\asmc_data\twinlite`에 둔다.
원본을 변경하지 않고 정확한 RGB 중복은 한 번만 사용한다. 동일 RGB와 상충하는 GT는 보류한다.
RGB만 있는 Sensor Record와 렌더링·팔레트 오류를 정상 배경으로 채택하지 않는다.

기존 actor-negative 정책을 유지한다. 주행가능노면 binary와
road_marking 4-class(background/white/yellow/stopline)를 만들고, 객체 픽셀은 유효한 non-drivable
negative로 처리한다. 주차칸 선은 원본 Semantic 분류대로 포함한다. BBox·Instance 파일은
segmentation 채택의 필수 조건이 아니며 YOLO의 극소 bbox ignore 기준도 적용하지 않는다.

AIM의 합의한 static_010101/dynamic_obstacle/tunnel_static_obstacle 187개 lap을 확인했지만
RGB 54,423장에 Semantic GT가 없어 이번 지도학습에는 사용하지 않는다.

## 해상도와 GPU 검사

| 뷰 | 입력 H×W | 확인 batch | AMP | 예약 메모리 최대 |
|---|---|---:|---|---:|
| 전방 |544×960|4|ON|3,756MiB|
| 좌우 |480×640|4|ON|2,136MiB|

운영 입력과 맞췄다. 전방은 원본1280×720을960×540으로 축소하고 위아래2px을 padding한다.
좌우는 원본640×480을 유지한다. 이전640 전방보다 얇은 선의 픽셀을 더 보존하기 위한 선택이며
학습 후 품질 우위나 최적 해상도를 입증한 실험은 아니다.

공식 Large BDD 가중치 SHA는
`e503cbbb05177ef6e59fe19285b60314715109f95bf2a494185e3145dd53a4a6`이다.
`out_ll` 14개 tensor만 새4-class 출력부로 초기화하고 나머지394개 tensor는 정확히 전이했다.
두 해상도에서 실제 Capture 입력을 사용한 일회성 forward/backward의 loss·gradient가 유한했다.
검사 모델의 BN 상태는 폐기했고 optimizer와 학습 checkpoint는 만들지 않았다.
메모리는 모델 검사 프로세스의 PyTorch 예약값이며 전체 GPU/실시간 성능 수락 수치가 아니다.

## 분할·조건 균형

최종 `twinlite_morai_v7_large_capture_r2`는 **84,350장**이다. 전방·좌우의 개별 영상 합계이며
이 중 후기 YOLO route 수집본53,569장을 포함한다.

| 조건 | train | val | test |
|---|---:|---:|---:|
| SUNNY11시 |6,534|204|1,578|
| SUNNY13시 |6,663|237|1,923*|
| SUNNY15시 |11,784|213|1,659|
| FOGGY11시 |6,439|183|1,182|
| FOGGY13시 |6,486|135|1,875|
| FOGGY15시 |2,565|432|2,241|
| 조건 일부 unknown |27,919|0|0|
| 17시 추가 평가 |0|0|4,098|
| **전체** |**68,390**|**1,404**|**14,556**|

`*` SUNNY13 test는 아래에 설명한 수집자 보고 추정이다. 촬영 묶음을 유지한 비율은
train81.08%·val1.66%·test17.26%다. FOGGY15가 적으므로 최대2회 노출해5,130장 상당으로
보완하며, 다른 다섯 조건의 노출은 각각11,784장 상당이다. Unknown27,919장은1회 사용한다.
따라서 한 epoch의 예정 노출은91,969회이고 고유 train68,390장은 모두 포함한다.

| 양성이 등장하는 이미지 | train | val | test |
|---|---:|---:|---:|
| drivable |68,329|1,404|14,556|
| white_lane |68,327|1,404|14,551|
| yellow_lane |60,452|1,251|13,259|
| stopline |35,750|814|8,239|

후기 YOLO 수집 layout의 여섯 조건과 세 뷰는 모두 같은 train group에 둔다.
9월 sample-b 여섯 run은 val, 이전 comp sample 촬영 묶음은 test로 묶는다.
기존 버전·split을 덮어쓰지 않고 새 Large용 r2를 만든다.

모든 유효 train 이미지를 매 epoch 사용한다. 부족한 SUNNY/FOGGY×11/13/15시만
이미지당 최대 두 번 노출하고, unknown·17시는 한 번 사용한다.
동일 크기끼리 batch를 만들며 순서는 seed20261017+epoch로 재현한다.
Val/test는 반복하지 않고 여섯 조건을 같은 비중으로 평균한 head IoU를 함께 산출한다.
새 데이터의 best checkpoint 기준은 `six_condition_macro_mean_head_iou`다.

과거 `sim_unknown` run의 sunny13은 보존 문서의 수집자 보고에 따른 추정이다.
RPC로 확인한 값으로 격상하지 않는다. 다른 확인되지 않은 조건은 unknown을 유지한다.
같은 K-City 맵과 과거 Medium 개발에 사용한 평가 자료이므로 비공개 독립 시험으로 설명하지 않는다.

## 검증과 산출물

데이터 로더·기존 손실·task 전이·mixed shape·조건 반복 상한·경로 이탈·padding gradient·
조건 macro·4-class 독립 평가 검사18개와 촬영 묶음/날씨 provenance 검사3개를 통과했다.
실제 최종 수량·클래스 픽셀·조건별 분포는 r2의 `dataset.json`과 `verification.json`에 고정한다.
84,350개 target PNG를 모두 다시 읽어 SHA·shape·클래스 픽셀 수를 검증했다.
일시적인 메모리 읽기 오류6장은 재검사로 모두 복구했으며 미해결0이다.

원본 후보94,542장 중 제외10,192장은 GT 누락6,357장, 비정상 RGB3,722장,
Semantic 팔레트 오류87장, 기존 명시적 제외12장, 정확한 중복6장,
같은 RGB/상충 GT4개 그룹의8장이다. 원본과 실패 기록은 보존한다.

원본/학습 GT 갤러리는
[E365 · 로컬 준비 결과](http://127.0.0.1:8000/camera-seg-large-data-20261017-r2/index.html)에서
split·여섯 조건·세 뷰별 사례를 확인한다. `gpu-capacity.json`, `dataset-summary.json`과
E드라이브의 `preflight/large-capture-v7-r2/preflight.json`이 입력 검사의 근거다.
완성된 train/val 각각의 두 해상도에 batch4 AMP no-grad 검사를 통과했다.
기본 파일 존재 확인은 NTFS에서 오래 걸려 검사 프로세스만 중단했고, 전수검수의 SHA와
실제 파일 재읽기를 확인한 뒤 `--skip-file-check`로 중복 존재 확인을 생략했다.
검사 config의 epochs/LR 기본 인자는 본학습 확정값이 아니며 `training_performed=false`다.

다음 작업은 사용자 승인 후 Large 추가학습 설정·중간 val 평가·최종 test 평가를 확정하는 것이다.
현재 운영 Medium v6 TensorRT와 family 실행 설정은 유지한다.
