# 카메라 데이터 준비

원본을 보존하고 종류 확인·검수 기록을 거쳐 별도 학습 버전으로 export합니다.
현재 배포 모델은 **v11의 완전한8epoch(E8)**이며 현재 운영 모델과 다음 데이터 준비 버전을 혼동하지 않습니다.
Git에는 도구·규칙·요약·배포 모델을, 원본 이미지/ZIP·대형 manifest·학습 재개 checkpoint는 로컬에 둡니다.

## 저장·버전

### 데이터 위치와 로컬 저장 정책

저장 위치는 **팀 PC별 선택**이며 특정 D/E 드라이브는 팀 공통 실행 조건이 아닙니다.
별도 설정이 없으면 아래처럼 레포와 같은 위치에 `data/`를 두면 됩니다.

```text
workspace/
├── 2026-ASMC/       # 팀 Git
├── data/            # 기본 ASMC_DATA; 이미지·GT·bag·학습 원본
└── external/        # 선택한 upstream 구현
```

호스트에서는 `ASMC_STORAGE_DATA`(명시한 물리 쓰기 루트), `ASMC_DATA`, 선택적 로컬 정책,
이전 `AIM_PROJECT/data`, checkout 옆 `data/` 순으로 경로를 정합니다. 컨테이너는
`ASMC_DATA=/data`와 실제 `/data` 마운트를 사용하며 호스트의 env.sh를 안에서 source하지 않습니다.
Docker 시작 도구는 기본 `/data`를 ROS에서 rw, 학습에서 ro로 마운트합니다.

```bash
# 레포 루트, 기본 sibling data 사용: config는 폴더/컨테이너를 만들지 않음
./scripts/docker_ros_up.sh config show
./scripts/docker_perception_up.sh config
# 사용자 지정 위치도 가능; 실제 Linux/WSL 경로를 지정
ASMC_DATA=/absolute/path/to/team-data ./scripts/docker_ros_up.sh config show
```

두 Docker 시작 도구는 저장소 상위 `.storage/env.sh`가 있을 때만 읽습니다. 다른 위치는
`ASMC_STORAGE_ENV=/absolute/path/to/env.sh`로 명시합니다. 이 파일은 신뢰할 수 있는 로컬 shell
설정이며 Git에 넣지 않습니다. 호출자가 이미 지정한 `ASMC_DATA`·`TWINLITE_SOURCE`가 우선합니다.
추가 마운트는 `ASMC_ROS_STORAGE_COMPOSE`/`ASMC_TRAIN_STORAGE_COMPOSE`로 로컬 overlay를
선택하며, 설정한 파일이 없으면 기본값으로 조용히 되돌리지 않고 실패합니다.

외장/NAS 마운트 유실을 검사하려는 PC만 상위 `.storage/policy.json`을 둡니다. 필요한 항목은
`storage_root`, `expected_mount`의 `point/type/source`, 선택적인 `minimum_free_bytes`입니다.
Linux/WSL의 `/proc/self/mountinfo`와 실제 볼륨을 대조합니다. 최소 여유 기본 50GiB도 이 정책이
있을 때만 적용하며 PC별로 조정할 수 있습니다. `ASMC_STORAGE_WORKSPACE`는 이 정책이 있는
workspace를 명시할 때 사용합니다. `ASMC_DATA_LOCATIONS`를 쓰는 컨테이너는 그 JSON의
`storage_contract`를 읽습니다. 개인 `tools/storage/manage.py` 설치는 팀 공통 요구사항이 아닙니다.

정책 없는 PC의 학습 출력은 기존 `repo/artifacts`를 유지합니다. 관리형 학습 환경은 쓰기 가능한
`/learning/training_runs`, 다음 `/artifacts`를 선택합니다. 최종 출력 경로를 쓰기 전에 검증하며
run을 재사용하지 않습니다. `docker_training_job.sh status/logs`는 기존 repo/artifacts의 job도
찾습니다. 서로 다른 위치에 같은 job이 있으면 `ASMC_TRAIN_JOB_ROOT`를 명시해야 합니다.

데이터 이관은 이 PR을 pull했다고 자동 실행되지 않습니다. 개인 파일 이동·중간 epoch 격리·
전체 파일 목록은 Git 밖 로컬 관리 기록에 둡니다. raw/GT/라벨/split·원본 체크포인트와
수집 당시 manifest는 보존하고, 같은 파일의 이전/이후 SHA와 컨테이너 내부 읽기/쓰기·
symlink 해석을 확인한 뒤 기존 경로를 전환합니다. 다른 볼륨으로 향하는 symlink는 부모 폴더
하나만 마운트해도 자동으로 따라가지 않으므로 실제 대상 마운트도 필요합니다.
운영 모델·토픽·메시지·센서 설정은 이 저장 경로 변경으로 바꾸지 않습니다.

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

AIM 원본은 제공받은 자료 중 `{static_010101,dynamic_obstacle,tunnel_static_obstacle}` 범위입니다.
물리 위치는 PC별 설정(`ASMC_AIM_ROOT` 등)으로 구분하고, 과거 manifest의 드라이브 문자열을
새 팀 기본 경로로 해석하지 않습니다. 다운로드가 완료되지 않은 원본은 사용하지 않습니다.
이전 시 SHA·원본 링크·컨테이너 mount를 검증하고 사용 권한도 별도로 확인합니다.
YOLO export 이미지는 원본 symlink일 수 있으므로 dataset 폴더만 복사하면 원본까지 이동하지 않습니다.

### Capture 원본 위치와 학습 전 경로 검사

`ASMC_DATA`는 **수집 장부·정리본·학습 버전의 루트**입니다. Windows 시뮬레이터가 쓰는
native `SensorData`와는 다르며 한쪽을 바꿔도 다른 쪽을 자동 이동하거나 연결하지 않습니다.
[data_locations.json](../../src/data_collection/config/data_locations.json)은 설치 예시 기본값만
제공합니다. 개인 archive·AIM 경로는 빈 값이며 D/E 드라이브나 원본 추가 마운트를 요구하지 않습니다.

새 Capture는 unique custom name을 유지하고 Windows 시뮬레이터의 conventional
`C:\MoraiLauncher_Win\MoraiLauncher_Win_Data\SaveFile\SensorData`를 명시합니다.
설치 위치가 다르면 기존 `morai_file_dir:=...` 인자, `ASMC_CAPTURE_DIR_WINDOWS`, 또는
`ASMC_DATA_LOCATIONS`로 가리킨 **Git 밖 PC별 JSON**의 `capture_directory_windows`를 지정합니다.
이 경로는 ROS 호스트의 데이터 경로나 Linux 마운트가 아닙니다. 실제 파일 QA를 먼저 수행합니다.

원본 조회는 CLI `--sensor-root`가 우선이며 `ASMC_SENSOR_ROOT`도 명시 root로 취급합니다.
둘 다 없으면 설정의 `capture_directory_wsl`과 선택적인 `capture_archive_wsl` 중 해당 run이
실제로 있는 **한 루트만** 선택합니다. archive는 `ASMC_CAPTURE_ARCHIVE`로도 지정할 수 있습니다.
다른 루트에 같은 run의 일부라도 있으면 `--sensor-root`를 요구하며 파일을 섞지 않습니다.
같은 실체를 가리키는 symlink 별칭은 중복 root로 세지 않습니다. 없는 명시 root를 만들지 않습니다.
원본 `source_files`·과거 SHA·split은 현재 경로에 맞춰 일괄 치환하지 않습니다.

```bash
# 저장소 루트, 실제 PC의 Linux/WSL 원본 root를 명시; dry-run은 복사하지 않음
python3 scripts/sync_capture_mode_data.py --run-id <run-id> \
  --sensor-root /absolute/path/to/SensorData --dry-run
# 학습용 이미지/라벨/target 참조를 읽기 전용 검사
python3 scripts/verify_data_locations.py \
  --dataset "$ASMC_DATA/dataset_versions/<version>"
```

`ASMC_DATA_LOCATIONS` JSON은 `schema_version: 1`과 필요한 native 경로를 담습니다.
`*_wsl`의 상대경로는 JSON 위치를 기준으로 해석하며 CLI 상대경로는 호출 위치를 기준으로 합니다.
출력 보호가 필요한 PC만 같은 JSON에 `storage_contract`를 추가합니다. archive/AIM이 없어도
일반 수집·ROS·학습 시작은 동작합니다. `ASMC_AIM_ROOT`는 입력 위치 표시일 뿐 자동 마운트 스위치가 아닙니다.
컨테이너에서 native 자료를 읽으려면 **명시한 로컬 Compose overlay**로 읽기 전용 mount와
해당 컨테이너 경로 설정을 함께 전달합니다. 호스트 경로 문자열만으로 컨테이너에 자료가 생기지 않습니다.
기존 manifest의 절대경로 별칭도 필요할 때만 로컬 overlay에 두며 팀 공통 Compose에 넣지 않습니다.

[2026-10-17 경로 검사 snapshot](../../config/baselines/evidence/data_paths_20261017/summary.json)은
PR #94에서 수행한 당시 호스트·읽기 전용 컨테이너의 검증 근거로 보존합니다.
그 안의 D/E·사용자 절대경로와 source SHA는 **과거 실행 기록**이며 현재 기본값이나 새 전수 검증이 아닙니다.
당시 native Capture RPC·학습은 실행하지 않았습니다. Large/BDD100K 조사도
[당시 학습 준비 기록](../records/camera/2026-10-17-twinlite-large-training-preparation.md)에 보존하고,
현재 실행 상태는 패키지의 학습 안내를 따릅니다.

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
새 target/manifest는 선택한 데이터 루트의 새 버전에 두고 기존 bbox 버전과 구분한다.
이후 YOLO 수집본도 같은 RGB/Semantic 검사를 통과하면 활용하며 bbox 최소 크기·ignore
규칙을 semantic 라벨에 적용하지 않는다. 데이터 준비와 학습 실행은 별도 단계이며,
2026-10-17 시작한 [Large 학습 실행](../../src/asmc_camera/docs/training-run.md)의 현황은 E366을 따른다.

현재는 추가수집·재학습을 시작하지 않습니다. 판제 요청이 오면 bus/truck·정면 bicycle/dog·Dog2/Dog10·터널/근거리
실패를 저장하고 클래스×하위종류×소스×조건·독립 계보를 다시 집계합니다.
확인된 부족 조건만 수집하고 라벨/배경 규칙을 train·selection·heldout에 동일 적용합니다.
평가 계보를 train에 옮긴 뒤 기존 체크포인트를 독립 평가했다고 주장하지 않습니다.
주석·loss·모델/설정 변경을 분리하고 새 버전·전용 학습기/재개 검증 후 학습합니다.
