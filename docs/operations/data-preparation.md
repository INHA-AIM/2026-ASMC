# 카메라 데이터 준비

원본을 보존하고 종류 확인·검수 기록을 거쳐 별도 학습 버전으로 export합니다.
현재 배포 모델은 **v11의 완전한8epoch(E8)**이며 현재 운영 모델과 다음 데이터 준비 버전을 혼동하지 않습니다.
Git에는 도구·규칙·요약·배포 모델을, 원본 이미지/ZIP·대형 manifest·학습 재개 checkpoint는 로컬에 둡니다.

## 저장·버전

```text
$ASMC_DATA/
├── capture_runs/<run_id>/             # 수집·actor·실행 장부
├── datasets/<run_id>/                 # Capture 원본 RGB·Semantic·Instance·BBox
├── rosbags/                          # raw/result/rejected bag
├── roboflow/
│   ├── raw/<workspace>/<project>/<version>/
│   ├── pins/                         #22개 소스 버전 고정
│   ├── curated-*/                    # 원본 의미·검수·채택/보류 계보
│   └── visual-review-*/
├── dataset_versions/<version>/       # data.yaml·manifest·summary·verification·labels
└── models/                           # 원본/실험 모델; 패키지 배포본과 구분
```

AIM 원본은 `/mnt/d/asmc_aim/dataset/{static_010101,dynamic_obstacle,tunnel_static_obstacle}` 범위입니다.
다운로드가 완료되지 않은 원본은 사용하지 않습니다. E드라이브 이전은 아직 팀 공통 경로로 전환한 상태가 아닙니다.
이전할 때 복사·SHA·원본 링크·컨테이너 mount를 확인하고 기존 원본을 정리합니다.
YOLO export 이미지는 원본 symlink일 수 있으므로 dataset 폴더만 복사하면 원본까지 이동하지 않습니다.

설정 v2는 라벨 의미 버전, 데이터 v11은 통합 버전, r4는 해당 단계의 revision입니다.
원본·부모 curated·reviews·policy·export SHA를 연결하고 이전 버전을 덮어쓰지 않습니다.
같은 배치·증강·연속 장면은 lineage로 묶으며 촬영 이미지 수를 독립 장면 수로 설명하지 않습니다.

## 현재 확인한 로컬 v11 준비본

`camera_objects_bbox_v11-preflight-r4`의 로컬 요약을2026-10-16 읽었습니다.
[수량·클래스·정책 SHA snapshot](../../config/docs/dataset_status_20261016.json)은 새 전수검사나 학습 실행 결과가 아닙니다.

| split | 이미지 | 양성 박스 | ignore |
|---|---:|---:|---:|
| train |41,345|154,537|234,208|
| selection |205|359|800|
| heldout |116|445|721|

이전 v7 채택본23,264장과 v8/v9/v10의 중간 수량은 [확보·Teacher·채택 이력](../records/camera/data-preparation-through-v7.md)에 보존합니다.
배포 E8의 가중치 출처·SHA는 [모델 manifest](../../config/models/camera_object_models_v2.json)가 정본입니다.
preparation metadata의 `training_started=false`는 생성 시점 값이며 현재 학습이 안됐다는 상태 조회가 아닙니다.
selection/heldout은 과거 개발 평가 노출 이력이 있어 새로운 비공개 시험으로 설명하지 않습니다.

## 라벨·완전성

[에셋/학습/소비 대응](../competition/obstacle-assets-and-labels.md)과
[데이터 계약](../contracts/dataset-labeling-split.md)을 함께 사용합니다.
직접 대응은 원본 박스를 유지하고 이름만 변환합니다. 혼합·복합은 개별/연속구간 근거로 해결합니다.
Capture의 정확한 native box도 세부 종류가 자동으로 확인된다는 뜻은 아닙니다.
actor ID/세대·instance component·RGB와 연결하고 BBox 파일 누락을 빈 정답으로 만들지 않습니다.

v11은 source/실제960 학습 입력의 짧은변4px·면적64px² 최소 양성 정책과 가시instance4픽셀 기준을 사용합니다.
극소/가림/unknown은 positive·ignore·완전성/안전 배경을 분리하며 정확한 배경을 근거 없이 확대하지 않습니다.
표시용 차량24/12px·기타16/8px, 배포 bbox cutoff, 학습 최소 기준은 다른 값입니다.
카메라 RGB/semantic의 하늘 구름만으로 날씨 기록을 sunny→cloudy로 바꾸지 않습니다.

## 도구와 재현 가능 범위

현재 main에는 Roboflow 감사/직접 채택·Teacher queue·Capture 변환·파일럿 도구가 있습니다.
[도구 목록](../../tools/README.md), [수집 package](../../src/data_collection/README.md),
[학습 경계](../../src/asmc_camera/training/README.md)를 확인합니다.
v8–v11의 ignore/partial/spatial-background 전용 본학습 코드는 이번 문서 PR에 가져오지 않습니다.
로컬 원본·설치·전용 학습기 revision이 필요하며 일반 `yolo train`이 supervision sidecar를 처리한다고 안내하지 않습니다.
배포 `.pt`는 optimizer/scheduler가 제거돼 학습 재개 checkpoint와 다릅니다.

## 다음 준비

TwinLite Large의 전체 Capture segmentation 준비는
[패키지 데이터 안내](../../src/asmc_camera/docs/training-data.md)를 따른다.
새 target/manifest는 `E:\asmc_data\twinlite`에 두고 기존 bbox 버전과 구분한다.
이후 YOLO 수집본도 같은 RGB/Semantic 검사를 통과하면 활용하며 bbox 최소 크기·ignore
규칙을 semantic 라벨에 적용하지 않는다. 데이터 준비와 학습 실행은 별도 단계이며,
2026-10-17 시작한 [Large 학습 실행](../../src/asmc_camera/docs/training-run.md)의 현황은 E366을 따른다.

현재는 추가수집·재학습을 시작하지 않습니다. 판제 요청이 오면 bus/truck·정면 bicycle/dog·Dog2/Dog10·터널/근거리
실패를 저장하고 클래스×하위종류×소스×조건·독립 계보를 다시 집계합니다.
확인된 부족 조건만 수집하고 라벨/배경 규칙을 train·selection·heldout에 동일 적용합니다.
평가 계보를 train에 옮긴 뒤 기존 체크포인트를 독립 평가했다고 주장하지 않습니다.
주석·loss·모델/설정 변경을 분리하고 새 버전·전용 학습기/재개 검증 후 학습합니다.
