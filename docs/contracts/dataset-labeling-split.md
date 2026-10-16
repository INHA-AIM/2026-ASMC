# 카메라 학습 데이터·라벨·분할 계약

bbox 객체와 segmentation 차선의 입력/정답 정책을 구분합니다.
현재 실행 입력은 [카메라 runtime](camera-runtime-v1.md), 배포 가중치는 [모델 안내](../operations/camera-deployment.md),
원본·버전·현재 수량은 [데이터 준비](../operations/data-preparation.md)가 소유합니다.

## 공통 원본·계보

RGB를 입력으로 사용하고 Semantic/Instance/BBox/actor 장부는 개발 GT로만 사용합니다.
원본은 변경하지 않습니다. source label/category ID·원본 박스·실제 종류·하위 유형·actor/component·검수 근거·SHA를 보존합니다.
Teacher 이름/score를 실제 에셋명이나 확정 GT로 덮어쓰지 않습니다. Teacher 미검출은 negative가 아닙니다.
BBox 파일 미생성/누락은 정상 빈 정답과 구분합니다. GT 스트림·Capture는 운영 알고리즘 입력이 아닙니다.

train/selection/heldout은 이미지 SHA뿐 아니라 run·연속장면·증강·actor 배치 계보로 분리합니다.
같은 시점의 세 카메라와 같은 배치의 날씨/회전 반복은 다른 split에 나누지 않습니다.
기존 Capture12/3/3과 후속 v9–v11 재분할은 서로 다른 고정 버전이며 새 버전의 실제 manifest를 우선합니다.
이미 학습·개발 평가에 사용한 자료를 새로운 독립 시험으로 설명하지 않습니다.
분할을 바꾸면 이전 checkpoint의 노출 이력·재시작 필요성까지 기록합니다.

## bbox 16종·양성·ignore

ID는 [라벨 설정](../../config/datasets/camera_object_labels_v2.json)의 `pilot_target_classes`16종 순서입니다.
자료가 없다고 목록에서 빼거나 consumer3그룹으로 학습 라벨을 바꾸지 않습니다.
명칭·구 라벨·소스별 혼합 예외는 [에셋/라벨](../competition/obstacle-assets-and-labels.md)을 따릅니다.

- 완전 가림 객체는 positive에 넣지 않고 native GT/보류 근거를 보존합니다.
- 부분 가림은 실제 보이는 instance 픽셀의 extent box를 사용합니다. 점유 비율이 낮다는 이유만으로 얇은 객체를 삭제하지 않습니다.
- v11 최소 정책은 짧은변 **4px 미만 또는 면적64px² 미만**입니다. 원본 또는 실제960 letterbox tensor 중 하나라도 미달하면 ignore입니다.
- Capture의 보이는 instance 픽셀4개 미만도 ignore입니다. 이 기준은 가시거리60m의 직접 필터가 아닙니다.
- 정상 양성과 ignore가 겹치면 정상 양성 학습을 우선합니다. ignore-only 이미지는 빈 negative로 사용하지 않습니다.
- 유모차 안 아기만 person에서 제외하고 확인된 성인·유모차는 유지합니다. [component 정책](../../config/datasets/capture_annotation_policy_v2.json)을 사용합니다.

YOLO `.txt`에는 양성을, sidecar에는 ignore·완전성·안전 배경을 저장합니다.
`complete`는 충분한 대상 주석이 확인된 이미지, `partial`은 미주석 가능 영역이 남은 이미지,
`reviewed_negative`는 대상 부재를 확인한 이미지입니다. background와 unknown·ignore는 다릅니다.

complete/reviewed_negative의 배경 손실도 ignore 영역을 제외합니다.
partial은 확인된 양성과 허용된 안전 배경/클래스별 영역만 학습하고 나머지 미주석 배경을 0정답으로 강제하지 않습니다.
v11의 spatial-background-v2는 full feature cell에 대한 semantic 안전 클래스와 주석 클래스의 bbox 내부 근거를 사용합니다.
semantic 안전 클래스는 asphalt·차선/횡단보도/정지선·sidewalk·sky·building·road_edge·표지/신호이며
이는 **분류 배경 근거**입니다. 구조물이 주행 가능하거나 충돌 위험이 없다는 뜻이 아닙니다.
정확한 policy·SHA는 [v11 snapshot](../../config/docs/dataset_status_20261016.json)에 있습니다.

sidecar를 무시하는 일반 학습기로 실행하지 않습니다. 현행 Git의 파일럿 학습기는
로컬 v11 본학습용 loss 확장을 대신하지 않습니다. 전용 코드·버전·실제 backward 검증과 재개 상태를 확보한 뒤 학습합니다.

## bbox 입력·균형

v11 준비의 `imgsz=960, rect=False, multi_scale=0`은 학습 설정이며 배포의 B1/B2와 다릅니다.
전방1280×720은960×540을960정사각에 padding하고 측방640×480은960×720을 padding합니다.
뷰가 확인된 영상은 검증된 Ego mask를 적용하고 가시 픽셀/박스를 확인합니다. unknown뷰에 mask를 추측 적용하지 않습니다.
모든 유효 train 이미지를 epoch마다 포함하고 희소 이미지의 추가 노출은 최대2회입니다.
빈도는 박스 수가 아닌 유효 양성 등장 이미지로 계산하며 연속 관측을 독립 장면으로 세지 않습니다.
기본 증강은 HSV·수평 반전, Mosaic/MixUp/CopyPaste/임의 회전·이동·축소는 끕니다.

## segmentation v6

TwinLite Medium v6는 `drivable`2종과 `road_marking`4종(background/white/yellow/stopline) head입니다.
두 head는 두 해상도라는 뜻이 아닙니다. v6 학습 cache384×640과 현재 runtime 전방544×960/측방480×640은 별개입니다.
기존 cache의 dimension을 현재 runtime에 맞춘 이름으로 바꾸거나 재생성됐다고 표시하지 않습니다.

공식 Semantic 팔레트→native target→paired RGB/mask resize/padding·valid→버전 manifest를 사용합니다.
RGB는 선형 보간, class mask는 최근접이며 ignore/valid를 같은 Geometry로 변환합니다.
Blue0픽셀 검사·white/yellow/stopline 클래스·actor-negative(B) 선택,18run의train26,001/val1,404와
4capture 제외의 전체 근거는 [v6 정책·실험 보존본](../records/camera/segmentation-dataset-policy-v6.md)에 있습니다.
이는 YOLO bbox 배경 정책과 같은 loss가 아닙니다.

## 평가·검증

정상 GT부터 class/group별 일대일 IoU≥0.5 대응 후 미대응 검출의 ignore 여부를 판단합니다.
ignore 검출은 TP가 아니고 동일 객체 추가 박스는 중복 FP입니다.
complete에서만 precision/F1을 계산하고 partial은 recall/사례로 구분합니다.
현재 개발 val205와 heldout은 과거 노출·소표본/소스 편중 한계를 기록합니다.

실제 YOLO 라벨 재읽기·고정ID·좌표/객체 수·원본SHA·mask/resize/flip 정합·계보 누출을 검사합니다.
학습 loss는 complete 동등성, partial 미주석 배경 제외, ignore의 모든 활성 head gradient 제외와 정상양성 우선을 확인합니다.
새 데이터셋을 만들면 수량·클래스×소스×뷰×크기×조건·negative/partial·독립 계보와 미해결 항목을 제출합니다.
