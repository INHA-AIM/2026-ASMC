# 카메라 장애물 학습 데이터 확보·감사·라벨 통일

> 기준일: 2026-10-09. 사용자 지정 Roboflow 22개 프로젝트와 기존 Capture GT를 대상으로 한다.
> 운영 모델·ROS 인터페이스·confidence·fusion·컨테이너 마운트는 변경하지 않는다.

## 현재 기준 — 2026-10-10 v2 16종

이 절과 [v2 라벨 정책](../../../config/datasets/camera_object_labels_v2.json),
[v2 소스별 대응표](../../../config/datasets/camera_object_source_labels_v2.json)가 현재 합의다.
아래 v1/2026-10-09 결과·검수·export는 당시의 증거로 보존하며 현재 분류 계약과 구분한다.
**이번 PR #78에서 X teacher 검증·v3 준비를 진행한다. PR #77의 운영 실행은 수정하지 않는다.**
최신 수치·갤러리·남은 조건은 [v3 실행 결과](#v3x-teacher-실행-결과--2026-10-10)를 먼저 읽는다.

MORAI 에셋명 / 원본 데이터 라벨 / 추론 prompt / 학습 클래스 / 소비 그룹을 별도로 보존한다.
MORAI 사용자 표기는 실제 gRPC API 이름이 아니며, 미확인 API 이름은 null로 둔다.

| MORAI 사용자 표기 | 학습 클래스 | 소비 그룹 |
|---|---|---|
| deer | deer | animal |
| bicycle | bicycle | vehicle |
| scooter + sportsbike | motorcycle | vehicle |
| bollard | bollard | obstacle |
| cargobox + woodbox | box | obstacle |
| concrete_barricade + concrete-barrier | concrete_barrier | obstacle |
| steel_barricade | steel_barricade | obstacle |
| PE drum + redbarrel + trafficbarrel + yellowbarrel | traffic_drum | obstacle |
| pe firewall | plastic_barrier | obstacle |
| color cone + rubber cone | traffic_cone | obstacle |

학습 후보 16종은 `person, dog, deer, bicycle, car, motorcycle, bus, truck,
bollard, box, concrete_barrier, steel_barricade, traffic_drum, plastic_barrier,
traffic_cone, stroller`다. 입식 전동 킥보드는 제외한다. 자전거와 steel barricade는 독립
학습 클래스로 유지한다. `box`로 합쳐도 CargoBox/WoodBox의 하위 유형은 평가에 남긴다.

실행 도구의 `canonical`은 검수한 세부 의미다. `training_map`을 거쳐 `training_class`와
`consumer_group`을 생성한다. 예: `wooden_box → box → obstacle`,
`pe_drum/traffic_barrel → traffic_drum → obstacle`. `original_label`,
`original_category_id`, `original_xyxy`, mask와 검수 근거는 유지한다.
`morai_asset_name`은 독립 확인 없이는 null이며 teacher가 반환한 이름과 혼동하지 않는다.

**소스별 확정 대응:** Katri 여섯 소스의 0=car, 1=bus, 2/3=person,
5=PE_Drum, 7=bollard; 4는 탑승자 복합 박스, 6은 dog/deer 혼합이다.
jjj의 8=CargoBox→box, 9=box/concrete_barrier/steel_barricade/plastic_barrier 혼합이다.
selfcar는 **0=PE_Drum→traffic_drum, 1/2=bollard**로 Katri와 다르다.
Enzo의 barrigate/pe barrigate는 각각 concrete_barricade/pe_firewall 의미에서 새 학습
클래스로 연결한다. tooth-decay의 **Fence=steel_barricade**, Concrete_1/2=concrete_barrier,
PE=traffic_drum이다. Park의 box는 WoodBox/PE_Firewall 혼합이므로 개별 분류한다.
그 밖의 animal, object/obstacle, vehicle, static/dynamic, 복합 탑승자는 이름만 바꾸지 않는다.

### 검수·export의 범위 관리

v1 검수의 종류·좌표 수정은 읽을 수 있다. 다만 새 방벽류가 추가된 v2의 완전성을 자동
승인하지 않는다. 새 검수 JSON의 `review_scope: camera_objects_v2_16`이 같은 영상에서
새 대상까지 확인했음을 나타낸다. 과거 기록은 덮어쓰지 않는다.
사람이 새 박스를 추가할 때 `additions`에 새 id/canonical/xyxy를 기록하고
`human_confirmed: true`와 검수자를 남긴다. teacher 제안은 자동 승인되지 않는다.

### YOLOE-26x 검증 경로

[고정 실험 설정](../../../config/experiments/camera_yoloe_x_teacher_v1.json)에 base18,
rich21, bollard/plastic 표현의 단독·결합 대조를 기록했다. X checkpoint 전용 embedding,
PyTorch FP16 batch1, 전방1280/측방640, 3뷰 Ego mask를 사용한다.
1280 OOM은 failure.json에 보존하고 960을 명시적으로 별도 실행한다. TensorRT는 생성하지 않는다.

[Capture GT 준비기](../../../tools/perception/prepare_capture_teacher_gt.py)는
run·원본 종류·actor ID·연속 frame 구간으로 독립 RGB 검수 결과를 연결한다.
같은 숫자 actor ID의 Obstacle과 Vehicle이 공존할 수 있어 run+ID만으로 합치지 않는다.
Teacher 출력은 GT로 사용하지 않는다. actor 종류 확인만으로 영상의 누락 주석까지
검증됐다고 판단하지 않으며, 완전성 미확인 영상은 precision·teacher 승인에서 제외한다.

[평가·제안 도구](../../../tools/perception/camera_teacher_audit.py)는 selection에서
0.40 대조와 0.10–0.70 탐색을 수행한다. 클래스별 GT/예측 각각 20개 이상,
precision ≥95%인 후보 중 recall 최대(동률이면 높은 confidence)를 선택하고 heldout에
고정한다. heldout에서도 같은 지원 수·precision 조건을 확인해야 teacher 클래스로 승인한다.
세부 의미·16 학습 클래스·4 소비 그룹을 구분하고 사슴→dog는 animal 위치 검출로만 인정한다.

승인된 클래스만 원본 혼합 박스와 일대일 IoU ≥0.5로 연결한다. 제안 파일은
`decision: pending`, `requires_human_confirmation: true`를 남긴다.
사람+자전거 복합 박스를 한 클래스의 박스로 확정하거나 두 클래스로 복제하지 않는다.
Teacher 미검출은 빈 정답이 아니다. 모든 이름 변경·추가 박스는 사람이 최종 확인한다.

진행 순서는 명확한 자료의 v2 범위 검수 → animal/6 → 숫자9 → box/object/obstacle이다.
운영 모델·ROS 계약·차량 제어는 유지하고, 추가 수집·학습은 이번 범위 밖이다.

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

숫자 클래스는 아래 2026-10-10 사용자 제공 대응표가 있는 **해당 프로젝트의 원본 이름**에만
적용한다. Obstacle, Vehicle, animal, scooter, bike, bicycle man의 혼합·복합 의미는
여전히 개별 확인한다. 다른 데이터셋의 숫자 ID를 COCO ID로 해석하지 않는다. 사슴의 dog 검출을 허용한 것은
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

## 2026-10-10 후속: 원본 클래스 의미 확인과 활용 우선순위

> 이 절은 v1 대응표 당시의 이력이다. 아래의 cargo_box/wooden_box/traffic_barrel과
> 방벽 `requires_class_decision`은 현재 학습 명칭이 아니다. 현재 v2에서는 box/traffic_drum과
> concrete_barrier/plastic_barrier로 연결하며, 상단 계약과 최신 v3 결과를 적용한다.

**위 v2의 76,297개 격리는 데이터 품질 불합격 수가 아니다.** 이 중 76,291개에
`visual_review_required`가 있었다. 숫자 이름의 의미 미확인, 미검수, 주석 범위 미확인도
같은 격리 집계에 들어갔다. 사유는 서로 중복되므로 합산하지 않는다. 이름을 맞추기 어렵다는
이유로 MORAI 자료 자체를 버리지 않는다. 119장은 이전의 엄격한 문턱을 통과한 작은 부분집합이며,
사용 가능한 전체 자료량으로 해석해서는 안 된다.

사용자가 22개 자료가 MORAI임과 클래스 의미를 직접 확인했다.
[소스별 대응표](../../../config/datasets/camera_object_source_labels_v1.json)에 출처를 사용자 제공 정보로
기록했다. 앞서 RGB로 발견한 반례는 notes에 함께 남긴다. 대응 순서는 소스 규칙 → 전역 규칙이다.
숫자는 COCO category ID가 아닌 **원본 라벨 문자열**로 해석하며, native 변환 과정에서 새로 부여한
category ID를 의미 ID로 사용하지 않는다. 원본 라벨·ID·박스·polygon은 수정하지 않는다.

동일한 외부 inventory 46,326개에 대한
[의미 재분류 집계](../../../config/baselines/perception_camera_source_label_resolution_v1.json)는 다음과 같다.
이는 중복·증강을 포함한 기록 수이고, 영상 완전 주석·split 검수 통과 수가 아니다.

| 항목 | 기록 수 |
|---|---:|
| 외부 이미지 | 46,326 |
| 라벨 의미가 명확해진 대상 주석 | 140,077 |
| 이 중 기존 ambiguous에서 대상 이름으로 새로 대응된 주석 | 101,314 |
| 신호·표지 등 제외 주석 | 11,254 |
| 혼합·복합·학습 범위 결정 등 추가 판단이 필요한 주석 | 57,835 |
| 기록된 라벨에 ambiguous가 없는 이미지 | 28,284 |

마지막 행에는 제외 클래스만 있는 이미지도 포함될 수 있다. 실제 대상 누락 여부를 확인하기 전
negative 또는 학습 승인으로 사용하지 않는다. 기존 v2 export와 baseline 판정은 덮어쓰지 않았다.

### 소스별 적용과 남은 작업

Katri 공통 숫자 규칙은 아래 여섯 프로젝트에만 적용한다:
`cone-z2vga/katri-q4vkc`, `katri-2-txrqf`, `katri_kids-2-x5xnl`, `katri_kids-zblud`,
`merge-axyff`, `ssibal-sjav5`.
0→car, 1→bus, 2/3→person, 5→traffic_barrel(PE_Drum), 7→bollard다.
4는 사람+자전거 복합 박스, 6은 dog/deer 혼합으로 남긴다.

| 소스 | 활용과 남은 확인 |
|---|---|
| aiv5challenge/yeonsoo-sarang | car/person 유지. 현재 환경과 유사한 기본 클래스 회귀 자료로 우선 활용 |
| battery-detection/cone-traffic-tirxp | cone→traffic_cone. 신호등 제외, 실제 polygon 보존 |
| bicycle-man/morai-bicycle | bicycle man은 탑승자 복합 박스. person/bicycle 각 박스를 그리거나 별도 rider 클래스 합의 필요 |
| 위 Katri 여섯 소스 | 명확한 숫자 라벨을 먼저 통일. animal·탑승자 박스만 분리 검수. 서로 복제된 자료와 증강 계보 확인 |
| jjj-nx9di/katri_new_data-xkbev | 공통 0–7에 8→cargo_box 추가. 9는 concrete/steel barricade·wooden_box·PE_Firewall을 박스별 재분류 |
| dh-mzmhr/animal-tohen | 기존 animal 박스를 dog/deer로 구분. 좌표를 다시 그릴 필요가 없는 박스는 종류만 수정 |
| enzo-mdikr/my-first-project-eeree | cargo box/wood box/PE_Drum/bollard/이륜차 등을 우선 활용, speed 제외. 기존 RGB 검수에서 dog로 주석된 deer가 있어 해당 묶음은 보정 필요 |
| hl-fma/hlfma_2 | person 등 명확한 이름 활용. 실제 metadata의 object에는 WoodBox·PE_Drum·steel barricade 등이 섞여 있어 종류 재분류 |
| hl-fma/hlfma_3 | car/truck/bus/person 활용, others 2개 원본 주석은 확인 필요 |
| mino-qjf5p/vi2re | car/person 및 검수한 obstacle 박스 활용, 신호·표지 제외. obstacle 전체를 한 종류로 바꾸지 않음 |
| parkminseo/final-ftuy2 | human→person, car/bus 유지, red/green/yellow 제외. box는 WoodBox/PE_Firewall 분리 |
| s-workspace-p9mvp/morai_obstacle | person 회귀·replay용. 풍부한 사람 자료가 있으므로 대량 반복보다 장면 다양성 우선 |
| selfcar/morai_test-t3i2r | **0→traffic_barrel, 1/2→bollard**. bollard01/04 정보는 원본 이름으로 보존. Katri 숫자 규칙과 혼용 금지 |
| seonhyung-kim/morai-perception | pedestrian(사용자 표기 pedstrain)은 사람 포함 복합 유형, vehicle·animal·obstacle도 세부 종류 검수. traffic_light 제외 |
| tooth-decay-rnacu/morai_ai_mission-fhqkh | **Fence→steel_barricade**, PE→traffic_barrel, sign/TL 제외. Concrete_1/2는 concrete barricade 후보. Obstacle_Yellow는 혼합 종류 검수 |
| yolo-m5lvd/detection-bzetd | Bus/Car/Truck/Person 활용. bike 외형과 Obstacle 혼합 유형 확인. Traffic/StreetLight 제외 |
| yuna-z15io/obstcle-mi0kq | static/dynamic은 물체 종류가 아니므로 기존 박스에 종류 재라벨링 필요. 초기 파일럿 우선순위는 낮음 |

Concrete barricade와 PE_Firewall은 실제 대회 물리 장애물 후보다. 앞서 zero-shot prompt에서
구조물 오검출 때문에 뺐다는 이유로 학습에서도 영구 제외하지 않는다. 현재 16종 정책에는 없으므로
`requires_class_decision` 또는 혼합 후보로 보존한다. asset 외형·박스 기준을 검토하고
`concrete_barricade`·`pe_firewall` 같은 세부 클래스를 추가할지 다음 학습 ontology에서 확정한다.
이 상태를 검출할 필요 없는 배경으로 자동 변환하지 않는다.

### 기존 데이터 활용과 필요한 수집

1. **기존 자료를 먼저 활용한다.** Enzo·jjj의 CargoBox/WoodBox, selfcar·Katri의 PE_Drum/bollard,
   battery의 cone을 우선 검수한다. aiv5·hlfma_3·Katri의 기본 사람·차량은 회귀와 균형 유지용으로
   함께 사용한다. project 전체를 버리는 대신 명확한 라벨이 있는 이미지부터 완성한다.
2. **혼합 클래스는 박스 좌표를 재사용해 종류를 수정한다.** dog/deer, class 9, box,
   object/obstacle 순으로 teacher 초안과 RGB 검수를 결합한다. 사람이 자전거를 탄 복합 박스를
   COCO bicycle 박스로 바로 치환하거나 같은 박스를 두 클래스로 복제하지 않는다.
3. **누락된 대상은 해당 이미지에서 보완한다.** cone 전용 이미지에서도 학습 대상 person/car가
   보이는데 라벨이 없으면 일반 YOLO의 완전 지도학습에 그대로 넣지 않는다. 신호 주석 삭제는
   가능하지만 다른 대상의 누락을 해결하지는 않는다. 16종이 모두 매 이미지에 존재해야 한다는
   조건은 없다. 영상에 실제로 보이는 선택 대상이 빠지지 않았는지를 검사한다.
4. **추가 수집은 부족 조건에 한정한다.** 터널의 낮은 밝기·FOGGY, 원거리·부분 가림,
   현재 좌우 뷰와 Ego mask 경계, CargoBox/WoodBox·PE_Firewall 등 부족 종류를 우선한다.
   기존 자료로 충분한 밝은 장면 car/person/cone를 다시 대량 수집하지 않는다.
   semantic/instance GT를 함께 확보하되 semantic 종류만으로 객체별 instance mask가 생기지는 않는다.

사용자 확인으로 라벨 의미가 풀려도 영상 완전성·구조·중복·분할 검사는 유지한다. 다만 위 우선
묶음의 표본 검수를 끝내 명확한 라벨 묶음을 승격하는 방식으로 진행하며, 전체 76,416장을
하나씩 수작업으로 끝낼 때까지 기다리는 방식은 피한다. 실제 오류가 발견된 묶음은 개별 보완한다.
고정 Capture split과 이미 평가한 heldout은 그대로 둔다. 희소 클래스의 분할 부족은 같은 장면을
train/평가로 복제해 채우지 않고 다른 소스·장면의 정답을 확보한다.

### YOLO와 YOLOE 학습 연결

자료 의미 통일은 두 모델에 공통이다. 원본 실제 종류 → 정규 학습 클래스 →
person/animal/vehicle/obstacle 소비 그룹을 분리하고 정적·동적 여부를 클래스와 섞지 않는다.
**같은 bbox train/selection/heldout으로 YOLO26l detect와 YOLOE detection 파일럿을 비교**한다.
YOLOE도 오픈 vocabulary라는 이유로 dog/deer 혼합이나 누락된 박스를 자동 정답으로 만들지는 못한다.
YOLOE teacher는 새로운 장애물의 이름 초안을 만드는 보조로 사용하며, 틀린 prompt 출력은 검수한다.

Ultralytics의 [공식 YOLOE 학습 안내](https://docs.ultralytics.com/models/yoloe/#train-usage)는
동일 scale의 detection YAML에 seg checkpoint 가중치를 가져오고 `YOLOEPETrainer`를 사용하는
bbox 학습 경로를 제공한다. 단순히 seg 예측에서 masks를 버리는 것과 구분한다.
실제 파일럿 전 고정 설치 버전에서 변환·학습·export를 작은 입력으로 확인한다.
`YOLOEPESegTrainer`로 seg를 학습할 때에는 실제 instance 주석이 있는 자료를 사용하며,
bbox 사각형을 가짜 mask 정답으로 만들지 않는다. PE fine-tuning의 class-name 고정 방식과
학습 후 prompt 변경 가능성도 export 전에 검증한다.

현 배포 모델 YOLO26l detect는 유지한다. 부족한 CargoBox/WoodBox 등의 개선,
기존 7종 회귀, 터널 오검출·공백과 4060 Ti 통합 지연을 보고 본학습 모델 하나를 선택한다.
두 모델용 데이터를 처음부터 따로 수집하지 않는다.

소스 의미만 다시 집계하려면 아래 명령을 사용한다. 이 명령은 학습 승인·export를 수행하지 않는다.
새 `curate` 실행은 저장 inventory의 원본 이름을 소스 규칙으로 다시 해석한 뒤 기존 검수 기록을
적용한다. 기존 export와 원본은 그대로 두고 v3 등 새 경로를 지정한다.

```bash
python tools/perception/camera_data_audit.py source-label-summary \
  --inventory "$ASMC_DATA/roboflow/versioned-inventory-v1/inventory.jsonl" \
    "$ASMC_DATA/roboflow/snapshot-inventory-v2/inventory.jsonl" \
  --labels config/datasets/camera_object_labels_v1.json \
  --output "$ASMC_DATA/roboflow/source-label-resolution-v1.json"
```

후속 검증: CPU 시험 44개, 문서 migration·로컬 링크·perception contract/topic policy 통과.
숫자 의미의 소스 간 혼용, Fence 예외, 혼합 박스 자동 분할, 원본 좌표 변경과
이름 통일만으로 학습 승인되는 경우를 검사한다. 실제 v3 검수·학습은 아직 수행하지 않았다.

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

## v3·X teacher 실행 결과 — 2026-10-10

[고정 수치·SHA](../../../config/baselines/evidence/camera_yoloe_x_teacher_v1/summary.json),
[전체 소스 대응 집계](../../../config/baselines/evidence/camera_yoloe_x_teacher_v1/source-label-resolution-v2.json),
[검증 기록과 코드 snapshot](../../../config/baselines/evidence/camera_yoloe_x_teacher_v1/validation.json),
[v3 baseline](../../../config/baselines/perception_camera_training_data_v3.yaml)을 함께 읽는다.

- 원본 Roboflow 46,326장·209,166개 주석에서 의미가 풀린 후보 142,103개,
  제외 11,254개, 혼합·미확정 55,809개다. v1 inventory보다 103,340개가 추가로 해석됐다.
  이는 증강·중복을 포함한 이름 대응 수이며 학습 승인 수가 아니다.
- Enzo 100개 독립 RGB 검토와 기존 Capture 일부의 새 범위 재검수로 **v3 95장·250객체**를
  생성했다. box 96(CargoBox 46/WoodBox 50), concrete_barrier 98, traffic_drum 50,
  person 3, car 2, truck 1이다. train 3 / selection 1 / heldout 91이다.
  기존 보수적 장면 그룹과 고정 Capture split을 옮겨 분포를 맞추지 않았다.
- export의 클래스 ID·원본 SHA·실제 YOLO 좌표·객체 수·파일 구성 재검증을 통과했다.
  채택 자료 미해결 라벨·구조 오류·확인된 정확 픽셀 split 누출은 각각 0이다.
  **부분집합 생성은 통과했으나 16종 파일럿 학습은 준비되지 않았다.** 클래스와
  train/selection/heldout별 정답이 부족하다. 과거 v2 119장을 새 범위로 무조건 승격하지 않았다.
- 첫 v3 이관에서 v1의 콘크리트 방벽 제외 override 98개가 남아 새 대상이 누락됐다.
  재검증에서 발견해 사용자 제공 소스 의미와 RGB 검토로 바로잡았다.
  [실패 근거](../../../config/baselines/evidence/camera_yoloe_x_teacher_v1/migration-failure-r1.json)와
  `curated-v3-rejected-r1` / `camera_objects_bbox_v3-rejected-r1`을 보존했다. 실패 버전은 학습에 쓰지 않는다.
- X checkpoint SHA는 `d08d390a08f98195f7c87807839fe4ff93a5491645fef1bc3bf0700efafdd639`다.
  공식 checkpoint·encoder에서 5개 prompt embedding을 각각 생성하고 같은 317프레임을 실제 추론했다.
  PyTorch FP16 batch1·전방1280/측방640·3뷰 mask, network-none, MORAI 종료 상태다.
  TensorRT export·OOM·silent fallback은 없었다.
- 기존 Capture selection/heldout 5,667프레임 중 20개 간격과 독립 actor 참고 영상으로
  고정한 **317프레임의 초기 진단**이다. 완전한 16종 주석을 확인한 정식 GT가 이 표본에는
  없어서 precision·전체 데이터 recall·confidence 선정·teacher 승인은 하지 않았다.
  33개 actor 종류 참고는 Codex의 독립 RGB 검토이며 사람의 최종 확인과 구분한다.
  과거부터 공개·평가된 heldout을 새 blind test로 부르지 않는다.
- 부분 GT의 confidence 0.40·IoU ≥0.5 대응에서 base18/rich21 모두 box 2/52,
  cone 87/114다. box는 0.25에서 4/52, 0.10에서도 11/52다. 이 수는 확인한 주석만의
  위치 대응 진단이며 GT 없는 배경의 precision을 만들지 않는다.
  상자 주석은 저장 영상에서 small(<32² pixels) 36개, medium 6개, large 10개다.
  작은 객체가 많은 이 표본을 전체 장애물·근거리 상자 성능으로 일반화하지 않는다.
  `doe`·`wooden crate`·`traffic drum` 추가는 이 표본의 통합 클래스 대응을 늘리지 못했다.
  plastic barrier는 base18/rich21 1/86, bollard21 2/86, plastic21/combined21 0/86이다.
  이 차이를 근거로 최종 vocabulary를 확정하지 않았다.
- 승인 teacher 클래스와 teacher가 적용한 Roboflow 라벨은 **0**이다.
  X teacher로 모든 혼합 라벨을 자동 분리할 수 있다는 가정은 지지되지 않았다.
  정답·지원 수를 갖춘 클래스만 다시 정식 selection → 고정 heldout 검증으로 연결한다.
  기존 원본 박스가 명확하면 이름 통일과 영상 검수를 먼저 활용하는 편이 낫다.

브라우저 시작점은 `http://localhost:8768/`다. 날짜·실험 번호가 있는
`camera-training-data-audit-v3/`와 `camera-yoloe-x-teacher-v1/`을 목차에서 선택한다.
teacher 비교 갤러리는 원본 GT·예측 bbox를 표시하고 대응 instance mask는 NPZ에 보존했다.
`camera-yoloe-x-teacher-v1/capture-gt-review/`는 teacher 예측 없이 독립 Capture 종류 후보와
원본 박스만 보여 준다. `actor_review.human_confirmation_pending`이 있는 후보는 미확정이다.
새 검수 페이지는 세부 의미·통합 학습 클래스·원본 category ID를 함께 보여 준다.
사람 확인 체크와 새 박스 JSON 입력을 제공하며 다운로드한 기록만 다음 export에 반영한다.

새 버전 생성 명령은 다음과 같다. 출력 경로가 이미 있으면 덮어쓰지 않는다.

```bash
python tools/perception/camera_data_audit.py curate \
  --inventory "$ASMC_DATA/roboflow/versioned-inventory-v1/inventory.jsonl" \
    "$ASMC_DATA/roboflow/snapshot-inventory-v2/inventory.jsonl" \
    "$ASMC_DATA/roboflow/capture-inventory-v1/inventory.jsonl" \
  --labels config/datasets/camera_object_labels_v2.json \
  --reviews "$ASMC_DATA/roboflow/visual-review-v3-corrected/reviews.json" \
  --output "$ASMC_DATA/roboflow/curated-v3" \
  --export "$ASMC_DATA/dataset_versions/camera_objects_bbox_v3"
python tools/perception/camera_data_audit.py verify-export \
  --dataset "$ASMC_DATA/dataset_versions/camera_objects_bbox_v3"
```

[GT 준비기](../../../tools/perception/prepare_capture_teacher_gt.py)의 `--actor-reviews`는
독립 RGB 근거·SHA·원본 종류·actor·frame 범위를, `--image-reviews`는 v2 범위의 영상 완전성
확인 기록을 받는다. `camera_teacher_audit.py infer`는 config/profile/checkpoint/embedding을
검사한다. 필요하면 `--image-prefix <호스트 데이터 경로> --image-root /data`로 읽기 경로만 변환한다.
`suggest`는 검증 report와 checkpoint·prompt·config를 맞추고 기존 혼합 박스에 한해 제안한다.

격리 GPU 환경의 재현 순서는 아래와 같다. checkpoint와 encoder는 실행 전에 준비하고
고정 SHA를 확인한다. `/audit`는 이 데이터 감사 worktree의 read-only mount,
`/prepared`는 X 전용 모델 디렉터리, `/results`는 새 실험 결과 디렉터리다.
공유 ROS 컨테이너에는 설치하거나 덮어쓰지 않는다.

```bash
export PYTHONPATH=/opt/seg-deps:/audit/src/asmc_camera/src:$PYTHONPATH
python3 /audit/tools/perception/prepare_yoloe_l.py \
  --directory /prepared --scale x --stage prompts --profile rich21 \
  --config /audit/config/experiments/camera_yoloe_x_teacher_v1.json
python3 /audit/tools/perception/camera_teacher_audit.py infer \
  --gt /data/roboflow/capture-teacher-gt-v1/diagnostic-sample.jsonl \
  --weights /prepared/yoloe-26x-seg.pt --embeddings /prepared/rich21.npz \
  --profile rich21 --front-width 1280 \
  --hood-config /audit/src/asmc_camera/config/camera/yolo_hood_mask_v1.yaml \
  --labels /audit/config/datasets/camera_object_labels_v2.json \
  --config /audit/config/experiments/camera_yoloe_x_teacher_v1.json \
  --image-prefix "$ASMC_DATA" --image-root /data \
  --output /results/rich21-repeat-1280
```

여기서 `--image-prefix`는 manifest에 기록된 **호스트의 절대 데이터 경로**다.
다섯 profile은 각각 새 embedding과 출력 경로로 실행한다. 이미 저장한 결과의 재집계에는
`camera_teacher_audit.py analyze`와 `report_camera_teacher_audit.py`를 사용한다.
전자는 checkpoint·embedding·prompt·config가 섞인 결과를 거부하고 승인 보고서에 SHA를 연결한다.
후자는 `--root`, `--gt`, `--labels`, `--config`, `--environment`, 선택적인 `--evidence`로
원본 예측에서 비교 표·갤러리·요약을 재생성하며 추론하지 않는다.
`camera_teacher_audit.py review --gt ... --labels ... --config ... --output <새 경로>`는
독립 GT 검토 화면을 재생성한다. 브라우저는 서버 파일을 쓰지 않고 확인 기록을 다운로드한다.

검증은 데이터 CPU 50개·teacher CPU 10개·기존 측정/목차 6개·카메라 검출 회귀 75개가
통과했다. 메시지가 없는 격리 환경의 첫 import 실패는 `asmc_msgs`·`asmc_camera`를
private `/tmp` workspace에서 빌드해 해결했다. 문서 migration·계약·topic policy,
v3 export 재검증·검수 HTML JavaScript 구문·갤러리 링크와 HTTP도 별도로 확인한다.
Hosted CI의 계정 결제/사용 한도 차단은 로컬 시험 결과와 구분한다.

남은 과제는 train/selection의 명확한 소스 검수 확대, Capture의 우선 객체 종류·완전성 사람 확인,
animal/6 → 9 → box/object/obstacle 순의 개별 재분류다. source 라벨 의미가 확인됐다는 이유로
새 박스·teacher 예측까지 사람이 검수했다고 기록하지 않는다. 필요한 터널·원거리·가림 자료는
이 커버리지를 본 뒤 수집하며, 이번에는 학습이나 수집을 시작하지 않았다.
