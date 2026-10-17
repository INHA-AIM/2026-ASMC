# TwinLite Large Capture 학습 데이터

현재 단계는 **데이터와 GPU 입력 준비**다. Large 추가학습과 운영 모델 교체는 시작하지 않았다.
준비 설정은 [v7 recipe](../../../config/experiments/twinlite_large_capture_v7.json),
집계와 검사 기록은 [2026-10-17 준비 결과](../../../docs/records/camera/2026-10-17-twinlite-large-data.md)를 따른다.

## 원본과 산출물

로컬 원본 Capture는 `E:\morai_dataset\SensorData`, 준비 산출물은
`E:\asmc_data\twinlite`에 둔다. 원본을 수정하거나 RGB를 시스템 디스크에 다시 복사하지 않는다.

```text
E:\asmc_data\twinlite\
├── targets/<run>/<view>/<sample>.png
└── dataset_versions/
    ├── twinlite_morai_v7_large_capture_r1/   # 원본 쌍 전수검사, 이전 split 보존
    └── twinlite_morai_v7_large_capture_r2/   # 새 Large용 train/val/test
        ├── dataset.json
        ├── train.jsonl
        ├── val.jsonl
        ├── test.jsonl
        ├── condition-provenance.json
        ├── verification.json
        └── _SUCCESS
```

원본 RGB/Semantic SHA, RGB 픽셀 SHA, target SHA, run/frame/view, 배치 계보,
날씨·시간과 근거를 manifest에 남긴다. PNG 하나에 drivable bit0,
road_marking bits1–2, validity bit3을 저장하며 로더에서 각각 복원한다.
원본 참조는 `capture_raw`·`silver`라는 명시적인 root alias로 연결한다.
따라서 준비 폴더만 복사해도 원본 RGB가 함께 이동하는 구조는 아니다.

이전 TwinLite 수집과 이후 YOLO 자동 스폰 수집을 모두 검사한다. RGB와 Semantic이 같은
Capture 이름으로 짝지어지고 렌더링·팔레트·크기가 유효하면 bbox/actor 검수 여부와 별개로
segmentation에 사용할 수 있다. RGB만 있거나 GT가 없으면 negative로 만들지 않는다.
AIM의 합의한 세 폴더는 현재 Semantic이 없어 지도학습에 포함하지 않는다.

## 입력과 라벨

| 뷰 | 원본 W×H | 입력 W×H | batch 구성 |
|---|---|---|---|
| 전방 | 1280×720 | 960×544 | 같은 크기끼리 |
| 좌/우 | 640×480 | 640×480 | 좌우 혼합 가능 |

전방은 960×540 영상 위아래에 2px씩 padding한다. 좌우는 원본 크기를 유지한다.
RGB는 운영 전처리와 같은 AREA 축소·RGB/255·padding114를 쓰고 정답은 최근접으로 변환한다.
Padding만 invalid이며 실제 영상의 객체 픽셀은 유효한 non-drivable negative다.
YOLO bbox의 최소 크기·ignore 규칙은 semantic 픽셀 라벨에 적용하지 않는다.

타깃은 기존 drivable binary와 road_marking 4-class(background/white/yellow/stopline)다.
주차칸 선은 원본 Semantic의 흰/황 차선 분류대로 포함한다. Ego 차로의 경계 역할은
후단 family에서 판단하며, 이미지 라벨에서 다른 선을 임의로 지우지 않는다.
BDD Large의 이진 lane 마지막 출력부 `out_ll`만 4-class로 다시 초기화하고,
나머지 사전학습 tensor는 정확하게 전이한다. 이전 Medium MORAI 가중치로 초기화하지 않는다.

## 분할과 조건 균형

후기 자동 스폰의 같은 layout·여섯 날씨/시간·세 뷰는 하나의 split에 묶어 train에 사용한다.
9월 sample-b의 여섯 run은 val, 그 이전 comp sample 촬영 묶음은 test로 분리한다.
큰 촬영 묶음을 지키므로 전체 수량을 임의의 80/10/10에 정확히 맞추지 않는다.
기존 데이터 버전과 당시 split은 덮어쓰지 않는다.

모든 유효 train 이미지를 매 epoch 사용하고, SUNNY/FOGGY×11/13/15시의 부족한 조건만
이미지당 최대 두 번 노출한다. Unknown·17시 자료는 한 번 사용하고 조건을 추정해 채우지 않는다.
`sim_unknown`의 기존 sunny13 추정은 과거 수집자 보고로 별도 표시하며 RPC 확인으로 격상하지 않는다.
Val/test는 반복하지 않고 pooled 지표와 여섯 조건을 같은 비중으로 평균한 지표를 함께 기록한다.
새 데이터의 체크포인트 선택은 `six_condition_macro_mean_head_iou`를 사용한다.

같은 K-City 맵을 사용하고 과거 Medium 개발에 쓰인 자료가 있으므로 공간적으로 새로운
비공개 시험이나 대회 전체 수락으로 설명하지 않는다. 새 BDD Large 학습의 train과 평가
이미지·촬영 묶음은 분리하며 정확한 RGB 중복과 서로 충돌하는 정답은 제외한다.

## 준비 재현과 무학습 검사

OpenCV·PyTorch가 있는 기존 이미지와 고정 TwinLiteNet+ source를 사용한다.
원본은 read-only, 새 target·학습 출력은 E 드라이브로 mount한다. 두 버전 이름이 이미
있으면 도구는 덮어쓰지 않으며 새 revision을 지정해야 한다.

```bash
python3 tools/perception/prepare_twinlite_captures.py \
  --silver-root /sources --capture-root /capture_raw --aim-root /aim_raw \
  --output-root /prepared \
  --gallery /artifacts/camera-seg-large-data-20261017-r1

python3 tools/perception/finalize_twinlite_captures.py \
  --data-root /prepared --silver-root /sources --capture-root /capture_raw \
  --code-root /work \
  --parent-gallery /artifacts/camera-seg-large-data-20261017-r1 \
  --gallery /artifacts/camera-seg-large-data-20261017-r2

export ASMC_TWINLITE_SOURCE_ROOTS='{"capture_raw":"/capture_raw","silver":"/sources"}'
python3 scripts/train_twinlite.py \
  --data-root /prepared --dataset-version twinlite_morai_v7_large_capture_r2 \
  --twinlite-root /opt/baselines/TwinLiteNetPlus --config large \
  --init-checkpoint src/asmc_camera/models/lane_segmentation/twinlite-large-bdd100k.pt \
  --batch-size 4 --num-workers 0 --seed 20261017 \
  --output-root /prepared/preflight --run-name large-capture-v7-r2 \
  --skip-file-check --preflight-only
```

마지막 명령은 양쪽 해상도의 no-grad forward와 loss만 검사하고 종료한다.
`--skip-file-check`는 원본 전수검사·최종 target 재읽기·SHA 검증 후 중복된 존재 확인을
생략하는 옵션이다. 처음 시도한 기본 존재 확인은 NTFS에서 오래 걸려 중단 기록을 남겼다.
실제 추가학습은 별도 승인 후 일정·LR·평가 주기와 최종 test 사용 시점을 확정한다.

추가학습한 4-class checkpoint의 별도 val/test 평가는 `scripts/evaluate_twinlite.py`를
사용한다. 이 도구도 같은 shape sampler와 조건 macro 집계를 사용하며 val/test를 반복하지 않는다.
이진 BDD 원본을 4-class 정답에 바로 비교하는 실행은 클래스 계약 오류로 차단한다.
