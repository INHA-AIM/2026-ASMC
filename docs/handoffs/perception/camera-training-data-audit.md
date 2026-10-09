# 카메라 장애물 학습 데이터 확보·감사·라벨 통일

> 기준일: 2026-10-09. 사용자 지정 Roboflow 22개 프로젝트와 기존 Capture GT를 대상으로 한다.
> 운영 모델·ROS 인터페이스·confidence·fusion·컨테이너 마운트는 변경하지 않는다.

## 작업 경계

`feat/camera-training-data-audit` / `asmc-camera-data-audit`에서 별도로 작업한다.
YOLOE 실행 검증은 PR #77 / `asmc-camera-objects`에 남긴다. 이번에는 학습·추가 수집을 하지 않는다.
이 자료는 객체 검출용이며 [TwinLite Capture 계약](../../contracts/perception/dataset-labeling-split.md)의
기존 학습 분할을 변경하지 않는다. 객체 검출 GT-v2의 18개 run은 12 train / 3 selection /
3 heldout, 30,090 view-frame 분할을 유지한다.

## 다운로드와 원본 보존

[22개 소스 목록](../../../config/datasets/roboflow_sources_v1.json)의 최신 완료 공개 버전을
처음 조회할 때 pin한다. 중단 후 다시 실행해도 번호를 바꾸지 않는다. 원본 ZIP·COCO 주석·
polygon/RLE·클래스 ID·기존 split·전처리·증강·라이선스·SHA를 `$ASMC_DATA/roboflow`에 보존한다.
이미지·ZIP·대형 manifest·인증 정보는 Git에 넣지 않는다.

공식 `roboflow` SDK 1.7.0과 CPython 3.12를 전용 venv에서 사용한다.
[공식 SDK](https://github.com/roboflow/roboflow-python)와
[다운로드 안내](https://docs.roboflow.com/datasets/create-and-upload/download-a-dataset)를 기준으로
프로젝트·버전·원본 주석을 조회한다.
[고정 requirements](../../../tools/perception/requirements-camera-data-audit.txt)를 기록했으며 ROS 환경은 수정하지 않는다.
API 키는 로컬 로그인 또는 `ROBOFLOW_API_KEY`에서 읽는다. 서명 URL·키·SDK 예외 본문은 기록하지 않는다.
공개 프로젝트에도 생성된 버전이 없을 수 있으며, 이를 삭제나 접근 거부로 해석하지 않는다.
이 경우 공식 search export의 준비 상태를 먼저 확인한다. 준비되지 않으면 공식 이미지 조회 API로
공개 Dataset에 편입된 원본과 native 주석을 받아 **버전 없는 원본 스냅샷**을 별도 보존한다.
미편입 업로드는 학습 자료로 포함하지 않는다. 검색 API의 250개 제한과 페이지 경계 중복을
검사하고, 공개 Dataset 이미지 수와 고유 ID 수가 일치하며 재조회에서도 같은 구성인지 확인한다.
새 버전·라벨·학습 작업을 만들거나 공유 설정을 변경하지 않는다. 스냅샷을 버전 1로 표시하지 않는다.

```bash
python tools/perception/download_roboflow_sources.py \
  --sources config/datasets/roboflow_sources_v1.json --root "$ASMC_DATA/roboflow"
python tools/perception/snapshot_roboflow_projects.py --root "$ASMC_DATA/roboflow" --method native
```

일반 다운로드는 동시 2개·최대 3회 시도다. 스냅샷 도구는 이 작업이 끝난 후 실행한다.
`download-report.json`은 번호 있는 버전 결과, `snapshot-report.json`은 추가 원본 확보 결과다.
22개 원본 확보와 22개 공개 버전 확보를 별도로 판정한다.

## 라벨과 완전성

[라벨 정책](../../../config/datasets/camera_object_labels_v1.json)은 원본 문자열과 정규 이름을 분리한다.
학습 후보는 person / dog / deer / bicycle / car / motorcycle / bus / truck /
electric_scooter / bollard / cargo_box / wooden_box / steel_barricade /
traffic_cone / traffic_barrel / stroller다. 네 소비 그룹은 평가용 metadata이며 운영 출력을 바꾸지 않는다.
실제로 채택된 정답이 있는 클래스만 학습 export에 포함한다.

숫자 클래스, Obstacle, Vehicle, animal, scooter, bike, bicycle man은 자동 확정하지 않는다.
다른 데이터셋의 숫자 ID를 COCO ID로 해석하지 않는다. 사슴의 dog 검출을 허용한 것은
사슴의 정답도 dog로 바꾸라는 승인이 아니다. 사람+자전거 박스를 둘로 복제하지 않는다.
신호등·표지판·구조물·노면 요소는 원본을 보존하고 학습 대상에서 제외하지만,
같은 영상에 있는 대상 객체의 누락 주석을 무시하지 않는다.
번호 없는 native 스냅샷의 COCO category ID는 로컬 변환용 ID다. 원래 문자열·native JSON·
image ID·파일 이름을 함께 보존하며 이를 COCO 80종 ID로 해석하지 않는다.

파일 디코딩·크기·SHA·bbox/polygon·참조 ID를 전수 검사한다. 누락된 주석 파일은 negative가 아니다.
Capture는 instance의 가시 영역 bbox와 semantic 종류 확인을 재사용하며 기존 palette 미확인 run,
미확정 subtype, 미대응 actor pixel 격리를 유지한다. RGB가 GT 생성 당시 SHA와 다르면 격리한다.
압축 RLE를 디코딩하지 못했다면 mask 검증 완료로 표시하지 않는다.

**기존 Capture `annotation_complete`만으로 16종 학습에 자동 채택하지 않는다.** 기존 GT의
검증 범위 밖에 있던 가까운 배럴·볼라드가 보이는데 주석되지 않은 반례를 확인했다.
자동 채택에는 이번 16종 전체를 포함하는 `verified_target_classes` 근거가 필요하며,
현재 범위 근거가 없는 프레임은 개별 영상 검수를 통과한 경우에만 사용한다. 과거의
자동 채택 수치는 새 범위 검증 전 중간 결과이며 최종 학습 자료가 아니다.

개별 영상에서 `pe drum`이 도로에 놓인 금속 배럴로 확인되면 `traffic_barrel`로 대응했다.
모든 drum 문자열을 자동 변경하지 않는다. `dog`로 잘못 주석된 사슴은 영상의 뿔·체형을
확인해 `deer`로 수정했다. 낮은 팔레트형 CargoBox와 X형 보강재가 있는 WoodBox를 구분한다.
콘크리트·플라스틱 방벽이 이번 16종에서 제외됐다고 충돌할 수 없는 공간이 되는 것은 아니다.
미지원 물체는 원본에 남기고 후속 학습 범위로 따로 검토한다.

## 검수·중복·분할

외부 자료는 최초에는 검수 대기다. 소스×클래스별 seed 20261009로 최대 50장을 고정한다.
표본 전부에서 대상의 완전 주석과 가시 영역 bbox를 확인한 명확한 라벨 묶음만 승격한다.
실패 표본이나 라벨 수정 표본이 있는 묶음은 일괄 승인하지 않는다. 혼합 라벨은 개별 검수·수정하거나 격리한다.
개별 검수는 이미지 SHA에 연결하며 다른 이미지에 전용하지 않는다.

동일 파일·동일 픽셀의 원본 기록은 모두 남기고 파생 데이터에서만 대표를 선택한다.
중복 주석이 충돌하면 박스 수로 우열을 결정하지 않는다. pHash는 유사 후보이며 삭제 근거가 아니다.
split 사이 유사 후보는 검수 전 export하지 않는다. 탐색이 불완전하면 누출 0으로 판정하지 않는다.
stretch 전처리로 종횡비가 달라진 경우도 pHash 후보 검사에 포함한다. 종횡비 차이만으로
후보를 버리지 않으며 `aspect_ratio_changed`로 별도 표시한다.
외부 자료는 장면 계보가 불명확하므로 프로젝트 전체를 한 그룹으로 80/10/10에 결정적으로 배정한다.
동일 이미지로 연결된 프로젝트를 먼저 합친 뒤 seed 20261009 순서와 최대 나머지 배분으로
그룹 수를 나눈다. 그룹 크기 때문에 이미지 비율은 정확히 일치하지 않을 수 있다.
Capture 평가 이미지와 겹치는 외부 프로젝트는 평가 쪽으로 제한하고 train에서 제외한다.
Capture 고정 split끼리 충돌하면 재배정하지 않고 격리한다. 이미 본 heldout은 새로운 비공개 test가 아니다.

```bash
python tools/perception/camera_data_audit.py inventory \
  --root "$ASMC_DATA/roboflow" \
  --labels config/datasets/camera_object_labels_v1.json \
  --capture-manifest /path/to/gt-v2/frames.jsonl \
  --capture-datasets "$ASMC_DATA/datasets" \
  --output "$ASMC_DATA/roboflow/inventory-v1"
python tools/perception/camera_data_audit.py curate \
  --inventory "$ASMC_DATA/roboflow/inventory-v1/inventory.jsonl" \
  --labels config/datasets/camera_object_labels_v1.json \
  --output "$ASMC_DATA/roboflow/curated-v1" \
  --gallery /path/to/artifacts/camera-training-data-audit-v1/review \
  --export "$ASMC_DATA/dataset_versions/camera_objects_bbox_v1"
python tools/perception/camera_data_audit.py verify-export \
  --dataset "$ASMC_DATA/dataset_versions/camera_objects_bbox_v1"
```

출력은 덮어쓰지 않는다. [통합 목차](http://localhost:8768/)의 데이터 감사 페이지에서
클래스·프로젝트를 검색하고 정규 라벨·가시 영역 좌표·검수 근거를 입력할 수 있다.
브라우저에서 검수 JSON을 저장하고 `curate --reviews <file>`로 새 버전을 만든다.
클릭만으로 학습 파일을 수정하지 않는다. 혼합 라벨을 추측해서 소스 전체를 승인하는 기능은 없다.
수정된 bbox의 원래 값과 검수자·근거는 `original_xyxy` / `review_record`에 남긴다.
Capture와 외부 inventory를 따로 생성한 경우 `--inventory`에 두 파일을 함께 전달할 수 있다.
CPU 작업은 2개씩 실행하며 GPU·MORAI·bridge·ROS master는 조작하지 않는다.

검수한 원본은 다시 생성하지 않는다. 번호 없는 스냅샷의 완료 archive SHA를 재실행 시
확인하고 재사용한다. native image ID를 inventory ID에 사용해 검색 순서 변화와 분리한다.
검수 거부는 오류 확정과 확인 부족을 근거 문장으로 구분하며 둘 다 학습에서는 격리한다.
날씨·거리·터널별 정확한 수량은 메타데이터 근거가 없으면 unknown으로 남긴다.

## 완료 판정과 다음 작업

baseline에서 소스별 원본 확보, 채택·격리 수, 실제 split과 클래스 커버리지를 확인한다.
날씨·터널·뷰·거리를 파일명만으로 확정하지 않고 근거가 없으면 unknown으로 표시한다.
원본 확보와 학습 준비 완료는 별도다. CargoBox/WoodBox가 없는 데이터셋은 추가 장애물 학습 준비 완료가 아니다.
미해결 라벨·split 충돌·검수 전 후보를 학습에 넣지 않는다.

파생 YOLO 자료는 원본 RGB에 대한 bbox 정답이다. 차체는 학습 대상이 아니며 raw RGB를
몰래 마스킹하지 않는다. 후속 학습에서 검증된 뷰별 Ego mask를 적용할지 결정하고,
학습·추론 전처리를 일치시켜야 한다. 진짜 polygon·Capture instance mask는 manifest의
원본 참조로 보존하며 bbox export가 segmentation 학습 자료라는 뜻은 아니다.

다음은 고정 데이터의 YOLO26l detect / YOLOE 후보 짧은 bbox 비교 학습이다.
기존 7종과 CargoBox/WoodBox의 검출·오검출·시간적 공백·통합 지연으로 본학습 모델 하나를 선정한다.
이번 작업에서 학습·운영 모델 교체를 수행했다고 설명하지 않는다.

## 실행 위치와 증거 갱신

데이터 도구는 호스트의 `asmc-camera-data-audit`에서 전용 venv로 실행한다.
현재 데이터 루트는 상위 workspace의 `data/roboflow`다. ROS 컨테이너에는 SDK를 설치하지 않는다.
`asmc-camera-objects`의 PR #77 작업과 마운트는 보존한다.

```bash
source ../data/roboflow/.venv/bin/activate
export ASMC_DATA="$(cd ../data && pwd)"
python tools/perception/report_camera_data_audit.py \
  --root "$ASMC_DATA/roboflow" \
  --curated "$ASMC_DATA/roboflow/curated-v1" \
  --dataset "$ASMC_DATA/dataset_versions/camera_objects_bbox_v1" \
  --gallery ../artifacts/camera-training-data-audit-v1 \
  --baseline config/baselines/perception_camera_training_data_audit_v1.yaml
python tools/perception/build_experiment_gallery_hub.py --artifacts ../artifacts
```

baseline은 소스별 원본/채택/격리 수, 주석 종류, SHA, 클래스 대응, 채택 클래스의 split별 객체 수와
검수 근거를 묶는다. `classes_in_all_splits`에 없는 클래스는 공정한 비교 학습 평가를 준비했다고
표시하지 않는다. `ATTRIBUTION.md`에 외부 출처와 라이선스를 함께 남긴다.
`reviewed_subset_ready`는 채택된 작은 부분집합의 구조·분할 검사이며,
`bbox_pilot_ready`는 기존 7종과 CargoBox/WoodBox의 train/selection/heldout 확보 여부다.
두 판정을 섞지 않는다. 후자가 false이면 필요한 클래스·분할의 개별 검수를 먼저 늘린다.

## 2026-10-10 결과와 아직 남은 감사

[고정 baseline](../../../config/baselines/perception_camera_training_data_audit_v1.yaml)에 소스별 표와 SHA를 기록했다.
22개 원본 확보는 완료했다. 17개는 최신 공개 버전, 5개는 발행 버전이 없는 native 스냅샷이다.
공개 버전 22개가 존재한다고 표시하지 않는다. 보존 archive 22개의 SHA를 재검증했고 총 2,911,061,824 bytes다.
외부 이미지 46,326개(증강·중복 포함)와 Capture 30,090 view-frame, 총 76,416개를 구조 검사했다.

실제 개별 검수는 286장이다. 생성한 소스×클래스 표본은 132묶음·고유 4,799장이며,
그중 검수한 표본은 236장이다. 표본 생성과 검수 완료를 혼동하지 않는다.
확인 부족·미검수·누락·혼합 주석·구조 오류·중복 충돌·cross-split 유사 후보는 학습에서 격리했다.
**전체 수동 라벨 감사는 미완료이며 검증된 부분집합만 제공한다.**

최종 경로는 `$ASMC_DATA/roboflow/curated-v2`와
`$ASMC_DATA/dataset_versions/camera_objects_bbox_v2`다. v1 및 그 이전 중간 결과는 보존하지만
`SUPERSEDED.txt`로 후속 학습 사용을 막았다. 재실행은 새 출력 버전으로 만들고 기존 디렉터리를 덮어쓰지 않는다.

| 항목 | 최종 결과 |
|---|---|
| 채택 이미지 / 객체 | 119 / 213 |
| train / selection / heldout 이미지 | 16 / 1 / 102 |
| 미확정 라벨 / 채택 자료 구조 오류 / 확인된 pixel split 누출 | 0 / 0 / 0 |
| 채택 정답 | person 12, dog 18, deer 10, car 3, bus 2, truck 1, cargo_box 62, wooden_box 52, traffic_barrel 53 |
| 전체 격리 | 76,297 이미지 기록 |
| pHash 유사 후보 | 251,062쌍; 확정 중복 수가 아니며 원본 보존 |
| 학습·운영 모델 변경 | 없음 |

종횡비를 바꾼 증강본 검사에서 2장을 추가 격리했다. 동일 픽셀 복제본의 0.5px 이하 반올림 차이는
원본 RGB 대조와 개별 수정 기록으로 통일했으며, 단순히 박스가 더 많은 주석을 선택하지 않았다.
Capture의 기존 `annotation_complete`가 가까운 배럴·볼라드까지 포함하지 않는 반례도 보존했다.
번호 없는 스냅샷의 조회·다운로드는 여러 도구 수정 단계에 걸쳐 수행했다. 원본 archive·native JSON SHA를
수집 증거로 삼고, baseline의 코드 commit/SHA는 최종 감사·export 구현을 고정한다.

**비교 학습 준비는 보류다.** 검수된 자료가 heldout에 편중됐고 모든 split에 정답이 있는 클래스는
현재 CargoBox뿐이다. WoodBox는 train 2개·heldout 50개이고 selection 정답이 없다.
기존 7종의 일부도 train 또는 평가 정답이 없다. seed·Capture split을 바꾸거나 heldout을 train으로
이동시켜 수량을 맞추지 않는다. 없는 클래스의 정답을 빈 파일로 만들지도 않는다.

다음 검수는 미확정 자료를 전부 사용하기보다 train/selection에 배정된 소스의
CargoBox·WoodBox와 기존 7종을 우선 늘린다. 혼합 `object/obstacle`, 숫자 이름,
사람+자전거 복합 박스와 주석 누락은 개별 수정하거나 계속 격리한다.
새 검수 JSON으로 v3을 만들고 `bbox_pilot_ready`를 다시 검사한 뒤 두 모델의 짧은 bbox 학습으로 넘어간다.

```bash
python tools/perception/camera_data_audit.py verify-export \
  --dataset "$ASMC_DATA/dataset_versions/camera_objects_bbox_v2"
```

검증: CPU 시험 37개, 문서 migration, perception contract/topic policy 검사 통과.
실제 생성 YOLO 파일을 다시 읽어 이미지 SHA·클래스 ID·좌표·객체 수·파일 구성·split을 확인했다.
검수 HTML의 JavaScript 구문과 로컬 HTTP 경로도 확인한다.
GitHub CI는 `recent account payments have failed or your spending limit needs to be increased`로
job 시작 전 차단됐다. 코드 시험 실패나 기존 facade 실패와 섞지 않는다.
PR #77의 HEAD와 runtime worktree·컨테이너 마운트는 변경하지 않았다.
