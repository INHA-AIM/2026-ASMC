# 카메라 학습 도구

ROS live/bag 추론은 `asmc-ros-noetic`, 학습·validation은 격리 학습 환경에서 실행합니다.
runtime package를 import했다고 학습이 시작되지는 않습니다. 현재 배포는 YOLO v11 E8와
TwinLite Medium v6이며 이 문서의 과거 v1/v4 결과를 현재 모델로 적용하지 않습니다.

| 경로 | 현재 역할 |
|---|---|
| `src/asmc_camera/training/twinlite/` | TwinLite upstream adapter·checkpoint·학습/평가 |
| 저장소 `scripts/train_twinlite.py`, `evaluate_twinlite.py` | 명시 실행용 TwinLite 도구 |
| `tools/perception/train_camera_bbox_pilot.py` | 기존 bbox 파일럿 경로 |
| 로컬 v8–v11 본학습 확장 | ignore/partial/spatial-background 전용; 현재 파일럿 도구로 대체하지 않음 |

학습 정답·원본·split은 [데이터 계약](../../../docs/contracts/dataset-labeling-split.md),
현재 로컬 버전/수량·도구 공유 한계는 [데이터 준비](../../../docs/operations/data-preparation.md)를 봅니다.
추론 해상도544×960/480×640과 기존 학습 cache384×640·bbox정사각960은 별개입니다.
동일한 checkpoint/state·upstream/설치 버전·입력 변환을 고정하고 비교합니다.

배포 `.pt`는 optimizer/scaler/scheduler를 제거한 추론본입니다. 같은 run 재개는 원본 checkpoint,
새 데이터 추가학습은 확인한 weight 초기화와 새 optimizer 일정을 사용하며 혼동하지 않습니다.
전용 trainer가 필요한 bbox sidecar를 일반 yolo train으로 무시하지 않습니다.
추가학습 전 GT·평가 누출·배경·ignore 정합을 확인하고 개발 selection으로 checkpoint를 고릅니다.

과거 구체적 명령·수치·v6 이전의 모델 선택 과정은
[학습 기록](../../../docs/records/camera/twinlite-training-through-v6.md)에 보존합니다.
이 문서 PR에서 학습을 실행하거나 학습 코드를 이식하지 않습니다.
