# data_collection

Capture 파일·raw/result bag·보조 ROS snapshot과 데이터 계보를 관리합니다.
일반 카메라/판제 launch나 빌드로 스폰·주행·녹화가 시작되지 않습니다.

| 목적 | 실행/문서 | 권한·한계 |
|---|---|---|
| 학습 RGB/GT 파일 | [Capture 절차](docs/capture-workflow.md) | Capture1Hz·한 요청, 원본 보존/파일 QA |
| 자동 Capture 주행 | capture_route_collection.launch, [파일럿 기록](../../docs/records/camera/camera-mpc-object-collection-pilot.md) | 개발 Ego·gRPC/MPC/PP 제어, 운영 주행과 동시 실행 금지 |
| 실제 비동기 지연·재현 | bag_recorder/bag_replay.launch, [bag 운영](../../docs/operations/rosbag-evaluation.md) | raw 불변·result별 모델/설정/부모 SHA |
| 일반 토픽 snapshot | sync_collector.launch | latest_timer 보조 수집; 외부 동기 tick/paired GT 보장 아님 |
| TwinLite 학습 파생물 | [native/cache/dataset 절차](docs/segmentation-dataset.md) | 기존384×640 cache·v6 split 이력, runtime 해상도와 별개 |

## Capture 시작과 복사

MORAI의 검증된 v3 센서셋·Capture Mode에서 먼저 정차3회 RGB·semantic·instance·BBox 완성을 확인합니다.
GT streaming bridge 확장은 필요하지 않습니다. manual trigger는 차량 제어를 하지 않으며 자동 route collector와 구분합니다.

```bash
roslaunch data_collection capture_mode_collector.launch capture_hz:=1.0 max_captures:=100
# 보조 토픽 snapshot만 필요한 경우
roslaunch data_collection sync_collector.launch profile:=dry_run
```

자동주행 수집은 승인된 배치·장부로만 실행하며 제어 생산자를 하나만 둡니다.
PP 대체는 Pure Pursuit + P 속도 제어이고 PID라고 안내하지 않습니다.
이 문서 갱신에서는 수집·스폰·주행을 실행하지 않습니다.

수집 종료 후 WSL host에서 복사합니다. run ID는 실제 수집 ID로 바꿉니다.

```bash
python3 scripts/sync_capture_mode_data.py --run-id <run_id> --profile bbox3 --dry-run
python3 scripts/sync_capture_mode_data.py --run-id <run_id> --profile bbox3 --jobs 4
```

bbox3_partial은 BBox 누락을 명시하며 누락 뷰를 빈 정답으로 만들지 않습니다.
원본은 `$ASMC_DATA/datasets/<run_id>`, 실행 장부는 `capture_runs/<run_id>`, 학습 export는
`dataset_versions/<version>`입니다. Windows 저장root·native-ID·완전성은 Capture 절차를 봅니다.

[data_locations.json](config/data_locations.json)은 conventional MORAI 설치 기본값만 담습니다.
native 원본은 `--sensor-root`/`ASMC_SENSOR_ROOT` 또는 PC별 `ASMC_DATA_LOCATIONS`로 지정하며,
archive·AIM은 선택 입력입니다. 같은 run이 여러 실체 root에 있으면 자동 혼합하지 않습니다.
Windows `morai_file_dir`는 시뮬레이터 저장 경로이고 `ASMC_DATA`는 ROS 장부·정리본 경로입니다.
개인 외장 드라이브·기존 manifest 별칭·추가 마운트는 공용 기본값이 아닙니다.

## Raw/result bag

```bash
roslaunch data_collection bag_recorder.launch profile:=full_raw scenario:=manual_route controller:=wheel
# live bridge 종료·별도 재현 환경에서 paused 시작
roslaunch data_collection bag_replay.launch bag_path:=/data/rosbags/raw/<run_id> required_profile:=camera_raw
```

replay clock → 추론/recorder → 재생 시작 순서를 [bag 운영](../../docs/operations/rosbag-evaluation.md)대로 지킵니다.
Capture의 simulator 정지와 Real Time UDP 성능은 같은 실험이 아닙니다.
Sensor Record는 현재 장착 센서의 파일 저장이며 GT를 자동 생성한다고 보장하지 않습니다.
GT·gRPC는 개발용이고 competition profile의 Object/Semantic/Instance socket은 기본 생성되지 않습니다.

[인자 전체](../../docs/operations/launch-reference.md) · [데이터 준비](../../docs/operations/data-preparation.md) ·
[데이터 계약](../../docs/contracts/dataset-labeling-split.md)

## 저장 위치와 선택적 보호

팀 기본값은 **호스트의 저장소와 나란한 `data/`**, 컨테이너의 `/data`입니다.
다른 곳에 두면 호스트에서 `ASMC_DATA`를 지정합니다. 특정 드라이브·외부 데이터·개인 관리
도구는 필수가 아닙니다. 기존 수집 launch의 명시적 출력 인자도 유지합니다.

저장소 상위 `.storage/policy.json` 또는 `ASMC_DATA_LOCATIONS`의 `storage_contract`가
있는 PC만 관리형 저장소 검사를 적용합니다. 해당 볼륨 미연결·다른 위치로의 출력·읽기 전용·
설정한 최소 여유 공간 미달이면 새 쓰기를 거부하며 기존 노드를 종료하지 않습니다.
정책 없는 PC에는 이 볼륨/여유 공간 제한을 강제하지 않습니다. 선택적
`ASMC_STORAGE_DATA`는 물리 출력 루트이고 `ASMC_DATA`는 지도와 데이터의 병합 뷰일 수 있습니다.

새 수집은 고유 run ID를 사용합니다. `DatasetWriter`도 기존 run의 meta/manifest를 덮어쓰지 않고
ID 재사용을 거부합니다. `ASMC_STORAGE_UNMANAGED_OUTPUT=1`은 임시 단위시험 전용입니다.
[공통 경로·로컬 정책·이관 주의](../../docs/operations/data-preparation.md#데이터-위치와-로컬-저장-정책)를 확인합니다.
