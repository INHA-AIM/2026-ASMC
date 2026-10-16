> 기존384×640 학습 cache의 재현 절차입니다. 현재 runtime 전방544×960/측방480×640을 의미하지 않습니다.

# Segmentation target·cache·dataset

동기화가 끝난 run의 Silver PNG는 수정하지 않고, versioned derived 경로에
원본 해상도의 `lane/drivable/stopline` canonical mask를 먼저 생성한다. road-marking
v3 정책은 여기에 `background=0, white=1, yellow=2, stopline=3` categorical mask도
추가한다. 이후
TwinLite cache 단계가 이를 `384×640`으로 letterbox하고 `valid` mask와 복원
geometry를 만든다. 두 단계 모두 head 아래에서 `front/left/right` 폴더를 분리한다.

```bash
# 1) 모델 독립 native target
python3 scripts/bake_perception_masks.py \
  --data-root "$AIM_PROJECT/data" \
  --run-id kcity_drive_001 --stage native

# 2) 완료된 native target에서 TwinLite cache
python3 scripts/bake_perception_masks.py \
  --data-root "$AIM_PROJECT/data" \
  --run-id kcity_drive_001 --stage twinlite

# run 전체에 고르게 분포한 8-frame 육안 검사용 preview
python3 scripts/bake_perception_masks.py \
  --data-root "$AIM_PROJECT/data" \
  --run-id kcity_drive_001 --stage all --sample-frames 8 \
  --native-output-name perception_targets_native_preview_v1 \
  --output-name twinlite_384x640_preview_v2 --write-previews
```

기본 출력은 `derived/perception_targets_native_v1/`과
`derived/twinlite_384x640_v2/`이다. 완료 전에는 최종 디렉터리가 노출되지 않으며,
기존 파생물을 교체하려면 `--overwrite`를 명시해야 한다. `{0,1}` canonical mask는
일반 이미지 뷰어에서 거의 검게 보이므로 육안 검사는 `qa_overlays/<view>/`를 사용한다.

객체 포함 v2 데이터는 먼저 정차 근접 중복을 비파괴 방식으로 선별한 뒤 frozen 정책을
명시해 bake한다. 기본 정책은 pose 변화 `0.01 m` 미만이 3 frame 이상 이어질 때 그룹으로
보고 시작/끝, actor·stopline 변화 frame, 매 5번째 frame을 남긴다. 원본 `frames/`와
동기화 manifest는 삭제하거나 수정하지 않는다.

```bash
# --run-id는 대상 신규 run마다 반복
python3 scripts/curate_perception_frames.py \
  --data-root "$AIM_PROJECT/data" \
  --run-id <run_id> \
  --output-name curation_v2

python3 scripts/bake_perception_masks.py \
  --data-root "$AIM_PROJECT/data" \
  --run-id <run_id> \
  --stage all \
  --policy config/perception_target_policy_v2.json \
  --curation-name curation_v2 \
  --native-output-name perception_targets_native_v2 \
  --output-name twinlite_384x640_v3
```

v2 native에는 `lane`, `drivable`, `stopline`, `drivable_ignore`,
`actor_occupancy_gt`가 저장된다. v3 cache의 drivable valid는 geometric valid에서
actor ignore를 뺀 값이다. 기존 actor 없는 run은 `--curation-name` 없이 같은 v2 정책과
출력 이름으로 bake해 모든 frame을 유지한다.

2026-09-06 road-marking 준비본은 기존 산출물을 덮지 않고 다음 이름으로 생성한다.

```bash
python3 scripts/bake_perception_masks.py \
  --data-root "$AIM_PROJECT/data" \
  --run-id <run_id> \
  --stage all \
  --policy config/perception_target_policy_v3.json \
  --curation-name curation_v2 \
  --native-output-name perception_targets_native_v3 \
  --output-name twinlite_384x640_marking_v1
```

이 정책은 binary `lane`을 white/yellow presence 호환 target으로 유지하면서 별도
`road_marking` 4-class target과 all-view stopline valid를 저장한다. Blue는 MORAI 공식
팔레트 검사에는 남겨 두지만 대회 target에서는 제외하며, 한 픽셀이라도 발견되면 bake를
중단한다. `curation_v2`가 없는 기존 baseline run에는 해당 옵션만 빼고 실행한다.

Actor 픽셀에 non-drivable loss를 주는 B 파생본은 원본과 A를 덮지 않고
`config/perception_target_policy_v3_actor_negative.json`,
`perception_targets_native_v3_actor_negative`,
`twinlite_384x640_marking_actor_negative_v1` 이름으로 생성한다.

## Run-based dataset

각 run과 `front/left/right` 폴더는 물리적으로 합치지 않는다. 완료된 모델 캐시를
검사한 뒤, 여러 run을 논리적으로 묶는 버전 manifest만
`$ASMC_DATA/dataset_versions/<dataset_name>/`에 게시한다. manifest의 모든 파일 경로는
`$ASMC_DATA` 기준 상대경로이며, RGB는 복사하지 않고 원본 Intensity PNG를 참조한다.

```bash
python3 scripts/build_perception_dataset.py \
  --data-root "$AIM_PROJECT/data" \
  --dataset-name twinlite_morai_v1 \
  --train-run kcity_drive_001 \
  --train-run kcity_drive_002 \
  --train-run kcity_auto_sunny_003 \
  --val-run kcity_drive_003
```

기본 활성 head는 하위 호환을 위해 `lane,drivable`이다. 4-class 학습 버전은
`--training-head drivable --training-head road_marking`을 명시한다. `stopline`과 binary
`lane` target도 비교·확장을 위해 manifest에 보존할 수 있다. 생성기는 run이 두 split에 중복되지 않는지, 모든
RGB/mask가 존재하는지, mask 크기가 `384×640`인지, binary 값이 `{0,1}`인지,
road-marking 값이 `{0,1,2,3}`인지 전수 검사한다. 기존 dataset version을 교체할 때만
`--overwrite`를 명시한다.

조건 전환 등 명확한 결함 frame은 원본/cache를 삭제하지 않고 capture frame 전체를
manifest에서 제외한다. 같은 frame의 front/left/right가 함께 빠지며 dataset metadata에
제외 목록과 sample 수가 남는다.

```bash
python3 scripts/build_perception_dataset.py ... \
  --exclude-frame <run_id>:0 \
  --exclude-frame <run_id>:1
```

2-head A 이력 버전은 `twinlite_morai_v4_comp8_train_val_sample_b`이다.
기존 12개 run 전체를 train 26,001 samples로, 이후 별도 한 바퀴에서 조건별로 나눈
`comp_sample_b` 6개 run을 val 1,404 samples로 구성한다. 첫 sunny 11am의 실제 날씨 전환
4 capture(12 samples)는 명시적으로 제외했다. test는 아직 비워 둔다.
정확한 run 목록과 manifest hash는
`$AIM_PROJECT/data/dataset_versions/twinlite_morai_v4_comp8_train_val_sample_b/dataset.json`을
기준으로 한다. 이전 v2/v3 dataset은 학습 전 기준 이력으로 보존한다.

Road-marking/A-B 준비 버전은
`twinlite_morai_v5_marking_prep_actor_ignore`이다. v4와 같은 train 26,001/val 1,404
sample 및 4-frame 제외를 유지하고, 활성 head도 아직 `lane,drivable`이다. 차이는
`road_marking` 4-class와 all-view `stopline`을 future target으로 연결하고 Blue 0-pixel
정책을 적용했다는 점이다. 이 버전을 actor-ignore(A) 기준으로 삼고, B는 split과 모델
초기값을 고정한 채 drivable actor 픽셀의 valid 정책만 바꾼
`twinlite_morai_v5_marking_actor_negative`다. 통제 평가에서 B가 actor leak을 크게 줄이고
일반 drivable/lane IoU가 회귀하지 않아 v5 시점의 잠정 기준으로 채택했다. 현재 runtime은 v6 road-marking TensorRT이며 이 cache를 현재 입력 해상도와 혼동하지 않는다.

다음 4-class 학습 manifest는 `twinlite_morai_v6_road_marking_actor_negative`다. B와
같은 26,001/1,404 split, 4-frame 제외와 cache를 사용하고 활성 head만
`drivable,road_marking`으로 바꾼다. dataset metadata의 `target_encodings`와 각 sample의
encoding은 `background=0, white_lane=1, yellow_lane=2, stopline=3`으로 고정된다.

기존 체크포인트를 학습 없이 새 validation에 평가하는 명령은 다음과 같다.

```bash
python3 scripts/evaluate_twinlite.py \
  --data-root /data \
  --dataset-version twinlite_morai_v4_comp8_train_val_sample_b \
  --split val \
  --checkpoint artifacts/perception_eval/runs/\
twinlite_medium_morai_v1_uniform_b12_e10_20260902/best_mean.pt \
  --output-dir artifacts/perception_eval/baselines/\
twinlite_medium_morai_v1_on_v4_sample_b_val_20260904
```

이 명령은 `eval()`과 `torch.no_grad()`만 사용하고 optimizer를 만들지 않으며,
`metrics.json`에 `training_performed=false`, 체크포인트 SHA-256과 실제 사용한 EMA/model
state를 기록한다.

## Capture Hz 참고

- 기본값은 **1 Hz**다. 현재 4 Camera + LiDAR + GPS + IMU 1세트 23파일의 실측 쓰기
  완료가 약 0.50~0.57초였기 때문에 안전 여유를 둔 값이다.
- 2 Hz 이상은 `1 → 2 → 5 → 10 Hz`, 각 100회 순서로 파일 누락·부분 파일·SIM FPS를
  검사한 뒤 사용한다. 2 Hz를 넘기면 노드도 미검증 경고를 출력한다.
- 스케줄이 밀려도 catch-up capture를 연속 호출하지 않는다.
- gRPC 실패가 기본 3회 연속 발생하면 자동 중단한다.

실제 저장 없이 manifest·motion gate만 확인하려면 `dry_run:=true`를 사용한다.


현재 label/split 규칙은 [중앙 데이터 계약](../../../docs/contracts/dataset-labeling-split.md)을 따릅니다.
