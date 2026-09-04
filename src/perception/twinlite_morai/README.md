# TwinLiteNet+ MORAI training adapter

> **역할:** Active Module — MORAI manifest, TwinLiteNet+ loader, 2-head loss와 학습/eval 연결
> **현재 후보:** Medium, lane+drivable, v4 18 epoch EMA
> **데이터 정본:** [perception-dataset-labeling-split.md](../../../docs/perception-dataset-labeling-split.md)
> **평가 기록:** [perception-v4-training-evaluation.md](../../../docs/perception-v4-training-evaluation.md)
> **다음 단계:** [인지 융합 로드맵 §7](../../../docs/perception-map-fusion-roadmap.md#7-카메라-live-개발과-평가-계획)

`$ASMC_DATA/dataset_versions/<version>/<split>.jsonl`을 읽어 run/view provenance를
유지한 채 세 카메라 표본을 하나의 공유 가중치 모델에 공급한다.

## 현재 상태 요약

| 항목 | 값 |
|---|---|
| dataset | `twinlite_morai_v4_comp8_train_val_sample_b` |
| train/val | 26,001 / 1,404 samples |
| 입력 | `384x640`, view별 aspect-ratio 유지 letterbox |
| heads | `drivable`, `lane` |
| 모델 | TwinLiteNet+ Medium, pinned upstream |
| 배포 후보 | `twinlite_medium_morai_v4_comp8_init_ema_b12_e18_20260904/best_mean.pt` |
| validation | drivable `0.9842`, lane `0.8930`, mean `0.9386` |
| 다음 작업 | checkpoint를 동결한 model-independent ROS live 추론·시각화 |

이 README는 모듈 사용법을 다룬다. 데이터 정책의 근거와 run 목록은 데이터 계약에,
실험 수치와 한계는 평가 기록에만 추가한다.

## 데이터 계약

한 JSONL 행은 한 카메라 프레임의 학습 계약이다.

- `image`: `$ASMC_DATA` 기준 원본 Intensity PNG 상대경로
- `targets.lane`, `targets.drivable`: 미리 letterbox한 `{0,1}` 정답
- `valid_masks.lane`: 원본 영상 영역 1, 패딩 영역 0
- `valid_masks.drivable`: 원본 영상 영역에서 actor ignore를 뺀 영역
- `auxiliary_targets`: `drivable_ignore`와 원본 해상도 `actor_occupancy_gt` 경로
- `geometry`: RGB에 적용할 resize/padding과 추론 결과 복원 정보
- `run_id`, `view`, `frame_id`: split 누수 검사와 조건별 평가용 provenance

RGB는 로딩 시 alpha를 제거하고 BGR→RGB 변환 후 `geometry`와 동일하게 letterbox한다.
반환 tensor는 다음과 같다.

```text
image              float32 [3,384,640], range [0,1]
targets.drivable   int64   [384,640], class id {0,1}
targets.lane       int64   [384,640], class id {0,1}
valid_masks.lane       bool    [384,640]
valid_masks.drivable   bool    [384,640]
```

## 모델·loss 연결

공식 TwinLiteNet+의 `(drivable_logits, lane_logits)` 반환 순서와, 같은 이름의 dict 출력을
모두 지원한다. 각 logit은 `[B,2,384,640]`이 기본이며 단일-channel binary logit도
허용한다.

```python
from torch.utils.data import DataLoader
from twinlite_morai import (
    MaskedTwinLiteLoss,
    MoraiTwinLiteDataset,
    move_batch_to_device,
    twinlite_training_step,
)

dataset = MoraiTwinLiteDataset("/data", split="train")
loader = DataLoader(dataset, batch_size=16, shuffle=True, num_workers=8)
criterion = MaskedTwinLiteLoss()

for batch in loader:
    batch = move_batch_to_device(batch, device)
    optimizer.zero_grad(set_to_none=True)
    outputs, losses = twinlite_training_step(model, batch, criterion)
    losses["loss"].backward()
    optimizer.step()
```

Loss는 head별 focal + Tversky의 합이다. 기본 alpha/gamma는 TwinLiteNet+ 설정을
유지하되, 원본의 front 전용 `[:, :, 12:-12]` crop 대신 head별 `valid_masks`를 적용한다.
따라서 front의 상하 padding과 left/right의 좌우 padding은 두 head에서 제외되고,
차량·보행자·obstacle 픽셀은 drivable head에서만 loss와 metric에서 제외된다. v1의 공통
`valid_mask` manifest도 하위 호환으로 지원한다.

## 실제 데이터 smoke

학습 컨테이너에서 데이터 두 장을 읽고 임시 2-head network로 forward/backward한다.

```bash
cd /root/ws
python3 scripts/smoke_twinlite_data_loss.py \
  --data-root /data --dataset-version twinlite_morai_v1 \
  --split train --batch-size 2
```

이 스크립트의 network는 배관 검사용이며 실제 TwinLiteNet+ 모델이 아니다.

## 공식 external 모델 연결과 tiny-overfit

공식 모델 소스는 팀 레포에 복사하지 않고 로컬 워크스페이스의
`external/baselines/TwinLiteNetPlus`에 유지한다. 학습 컨테이너에서는 이를
`/opt/baselines/TwinLiteNetPlus`로 읽기 전용 마운트한다. Loader는 실행 전에 upstream
commit이 `90f1b8695ae311d5123b05f8534b2e11e42499d2`인지 확인하고 다르면 중단한다.

```bash
./scripts/docker_perception_up.sh up

docker exec asmc-perception-train bash -lc '
  cd /root/ws
  python3 scripts/train_twinlite_tiny.py \
    --data-root /data --dataset-version twinlite_morai_v1 \
    --config medium --max-samples 12 --epochs 30 --batch-size 3 \
    --output-dir artifacts/perception_eval/runs/twinlite_medium_tiny_30
'
```

결과의 `checkpoint.pt`, `metrics.json`, prediction panel은 Git에서 제외되는
`artifacts/perception_eval/runs/`에 저장한다. Tiny-overfit checkpoint는 배관 검증용이며
validation 성능이나 배포 모델로 사용하지 않는다.

## 초기 v1 전체 데이터 학습 기록

RTX 4060 Ti 8GB에서는 `384x640` Medium 학습에 batch 12를 사용한다. Batch 24는
메모리 약 7.5GB를 점유하면서 처리량이 크게 저하됐으므로 사용하지 않는다. 유효 영상
영역은 거리에 따른 별도 가중치 없이 모든 픽셀을 동일하게 취급하고, letterbox padding만
`valid_mask=0`으로 loss에서 제외한다.

```bash
docker exec asmc-perception-train bash -lc '
  cd /root/ws
  python3 scripts/train_twinlite.py \
    --data-root /data --dataset-version twinlite_morai_v1 \
    --config medium --epochs 10 --batch-size 12 --num-workers 6 \
    --log-interval 50 \
    --run-name twinlite_medium_morai_v1_uniform_b12_e10_20260902
'
```

산출물은 `artifacts/perception_eval/runs/<run-name>/`에 둔다. `latest.pt`는 재개용,
`best_mean.pt`는 두 head 평균 IoU 기준 배포 후보, `best_drivable.pt`와
`best_lane.pt`는 head별 분석용이다. 각 checkpoint에는 raw model, EMA, optimizer,
scheduler, AMP scaler, upstream commit과 validation 결과가 함께 저장된다.

## v2/v3/v4 고정 평가와 재학습

`evaluate_twinlite.py`는 optimizer 없이 `eval()`/`no_grad()`로만 실행하며 checkpoint
SHA-256, raw/EMA 선택, split/run/view/weather별 foreground IoU를 기록한다.

```bash
docker exec asmc-perception-train bash -lc '
  cd /root/ws
  python3 scripts/evaluate_twinlite.py \
    --data-root /data --dataset-version twinlite_morai_v2_curated \
    --split val --split test \
    --checkpoint artifacts/perception_eval/runs/twinlite_medium_morai_v1_uniform_b12_e10_20260902/best_mean.pt \
    --output-dir artifacts/perception_eval/baselines/twinlite_medium_morai_v1_on_v2_curated_20260904
'
```

2026-09-04 기준 결과는 val 평균 head IoU `0.7542`, test `0.4499`다. 이 값은 v2
추가 학습 전 5pm OOD 기준선이다. 이후 5pm을 v3 train으로 이동했으므로 재학습 후 공정한
test로 재사용하지 않는다.

실제 재학습 버전은 `twinlite_morai_v4_comp8_train_val_sample_b`이다. 기존 12개 run
전체 train 26,001 samples와 이후 별도 수집한 sample-b 6개 run val 1,404 samples로
구성한다. 첫 sunny 11am의 날씨 전환 4 capture는 val manifest에서 명시적으로 제외했다.
기존 checkpoint의 EMA **모델 가중치만** 가져오고 optimizer/scheduler/EMA/best는 새로
시작하도록 `--init-checkpoint`와 `--init-weights`를 사용한다. `--resume`은 같은 run의
중단 재개 전용이며 `--init-checkpoint`와 함께 사용할 수 없다.

본학습 전 무학습 preflight:

```bash
docker exec asmc-perception-train bash -lc '
  cd /root/ws
  python3 scripts/train_twinlite.py \
    --data-root /data \
    --dataset-version twinlite_morai_v4_comp8_train_val_sample_b \
    --config medium --epochs 10 --batch-size 12 --num-workers 0 \
    --lr 2e-4 \
    --init-checkpoint artifacts/perception_eval/runs/twinlite_medium_morai_v1_uniform_b12_e10_20260902/best_mean.pt \
    --init-weights ema \
    --output-root artifacts/perception_eval/preflight \
    --run-name twinlite_medium_morai_v4_clean_init_ema_20260904 \
    --preflight-only
'
```

`--preflight-only`는 train/val에서 한 batch씩 no-grad forward만 수행하고
`training_performed=false`를 기록한 뒤 종료한다. v4 preflight는 train/val 각각 12 sample,
두 출력 `[12,2,384,640]`과 finite loss를 확인했다. 실제 2-head 재학습에서는 같은 명령에서
`--preflight-only`를 제거하고 output root를 `artifacts/perception_eval/runs`로 바꾼다.

기존 EMA의 v4 무학습 기준은 val 전체 평균 `0.8210`, front `0.7503`, foggy `0.7582`,
foggy 3pm `0.7167`이다. 재학습 checkpoint는 전체 평균뿐 아니라 이 세 취약 구간이
기준보다 회귀하지 않는지도 함께 확인한다.

v4 2-head 본학습 명령:

```bash
docker exec asmc-perception-train bash -lc '
  cd /root/ws
  python3 scripts/train_twinlite.py \
    --data-root /data \
    --dataset-version twinlite_morai_v4_comp8_train_val_sample_b \
    --config medium --epochs 10 --batch-size 12 --num-workers 6 \
    --lr 2e-4 --log-interval 50 \
    --init-checkpoint artifacts/perception_eval/runs/twinlite_medium_morai_v1_uniform_b12_e10_20260902/best_mean.pt \
    --init-weights ema \
    --output-root artifacts/perception_eval/runs \
    --run-name twinlite_medium_morai_v4_comp8_init_ema_b12_e10_20260904
'
```

학습 도중 중단되면 같은 run의 `latest.pt`를 `--resume`으로 재개한다.
`--init-checkpoint`는 새 학습 시작에만 사용하고 `--resume`과 함께 지정하지 않는다.

## 현재 18 epoch 배포 후보

10 epoch의 종료 LR을 이어받지 않고 동일 초기 EMA, seed, v4 manifest에서 18 epoch
polynomial schedule을 새로 수행했다. 학습과 frozen validation이 모두 정상 종료됐으며
현재 live 후보는 다음 파일이다.

```text
artifacts/perception_eval/runs/
  twinlite_medium_morai_v4_comp8_init_ema_b12_e18_20260904/
    best_mean.pt
```

고정 평가 결과:

```text
artifacts/perception_eval/analysis/
  twinlite_medium_morai_v4_e18_on_v4_val_20260905/
    metrics.json
    overlays_selected/
```

`best_mean.pt`에서는 `ema_state_dict`를 우선 사용한다. 현재 validation은 매 epoch model
selection에 사용했으므로 독립 test가 아니며, 단순 추가 resume보다 live 조건별 검증과
새 test run을 우선한다.

## live 패키지와의 경계

이 모듈은 TwinLiteNet+ 전용 model/checkpoint/output adapter를 제공한다. 다음에 만들
`camera_semantic_perception` ROS 패키지는 영상 구독, view 선택, latest-only buffer,
성능계측과 표준 probability topic을 담당한다. 모델을 교체할 때 ROS node를 다시 쓰지 않고
backend adapter만 추가하는 구조를 사용한다.
