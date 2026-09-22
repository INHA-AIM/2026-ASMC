# MORAI 인지 데이터 라벨링·분할 계약

> **문서 역할:** Canonical Data Contract — Capture 원본, mask, ignore, manifest와 split
> **문서 허브:** [README.md](../../README.md)
> 기준일: 2026-09-08<br>
> 대상 모델: TwinLiteNet+ Medium (drivable + 4-class road-marking)<br>
> 상태: actor-negative(B) 잠정 채택, v6 본학습·validation·live 시각화 완료
>
> **빠른 읽기:** 현재 data contract는 §1~8과 §10의 최종 결정이다. §9의 날짜별 학습 결과는
> 결정 근거를 보존한 rollout 기록이며 새 결과를 덮어쓸 위치가 아니다. 신규 성능 결과는
> `docs/records/perception/`에 추가한다.

## 1. 현재 데이터와 사용 범위

동기화 정본은 Git 레포 밖 `$ASMC_DATA/datasets/<run_id>/`에 둔다. Windows
`SensorData`는 라벨 bake와 표본 검증이 끝날 때까지 백업 원본으로 유지한다.

| Run | frame | TwinLite 3-view sample | 초기 용도 |
|---|---:|---:|---|
| `kcity_drive_001` | 958 | 2,874 | train |
| `kcity_drive_002` | 681 | 2,043 | train |
| `kcity_drive_003` | 306 | 918 | validation |
| `kcity_auto_sunny_003` | 2,770 | 8,310 | train |
| 합계 | **4,715** | **14,145** | sunny baseline |

2026-09-03~04에 예제 시나리오를 변형해 sunny/foggy, NPC, 동적·정적 장애물,
제밍 구간을 포함한 8개 run을 추가했다. 정차 중 거의 같은 장면은 원본에서 지우지 않고
`curation_v2` 선택 manifest로만 제외한다. 최종 `twinlite_morai_v2_curated` 구성은 다음과 같다.

| Split | run | frame | 3-view sample |
|---|---:|---:|---:|
| train | 8 | 6,692 | 20,076 |
| val | 2 (sunny/foggy 3pm) | 999 | 2,997 |
| test | 2 (sunny/foggy 5pm) | 976 | 2,928 |
| 합계 | **12** | **8,667** | **26,001** |

위 v2 split은 5pm의 학습 전 OOD 성능을 측정한 이력으로 보존한다. 5pm도 강건화 학습에
사용한다는 수집 목적에 맞춰, 실제 2-head 재학습용 split은 덮어쓰지 않고
`twinlite_morai_v3_comp8_train_val_drive003`으로 새로 게시했다.

| Split | 구성 | frame | 3-view sample |
|---|---|---:|---:|
| train | 신규 comp 8개 전체 + 기존 `drive_001/002`, `auto_sunny_003` | 8,361 | 25,083 |
| val | 기존 `kcity_drive_003` | 306 | 918 |
| test | 미배정 | 0 | 0 |

신규 8개 run의 원본 4,843 frame 중 3,952개를 선택하고 정차 근접 중복 891개를
제외했다. 선택은 capture frame 단위이므로 같은 시점의 front/left/right가 서로 다른
split 또는 선택 상태로 갈라지지 않는다. 기존 4개 run에는 정차 중복 조건에 해당하는
구간이 없어 4,715 frame을 모두 유지했다.

2026-09-04에 예제 시나리오 한 바퀴를 날씨·규정 시간별 6개 연속 구간으로 나눈
`comp_sample_b` 472 frame을 추가 수집했다. 이 batch는 기존 12개 run과 촬영 시점이
겹치지 않으므로 최종 2-head 재학습용 독립 validation으로 고정했다.

| Split | 구성 | frame | 3-view sample |
|---|---|---:|---:|
| train | 기존 12개 run 전체 | 8,667 | 26,001 |
| val | 신규 `comp_sample_b` 6개 run | 468 | 1,404 |
| test | 미배정 | 0 | 0 |
| 합계 | 18 run | **9,135** | **27,405** |

첫 sunny 11am 구간은 MORAI 날씨 전환 직후 0~3 frame이 실제 안개 화면이었다. 원본과
파생 cache는 보존하고 dataset builder의 명시적 frame exclusion으로 세 view 12 sample만
val에서 제외했다. 나머지 468 frame은 파일 누락 0, Semantic 미등록 색 0이며, train/val
`run_id` 및 sample 중복도 0이다. 현재 학습 버전은
`twinlite_morai_v4_comp8_train_val_sample_b`를 기준으로 한다.

같은 split과 제외 목록을 그대로 복제한 road-marking 준비 버전
`twinlite_morai_v5_marking_prep_actor_ignore`도 발행했다. 활성 학습 head는 비교 가능성을
위해 여전히 `lane,drivable`이고, `road_marking`과 all-view `stopline`은 future target이다.
27,405 sample의 이미지 경로, mask shape, binary/categorical 값 범위를 전수 검사했다.
이 버전은 향후 장애물 정책 A/B의 actor-ignore(A) 기준으로 사용한다.

통제 A/B 결과 actor-negative(B)는 도로 문맥 actor leak을 `14.26% → 3.20%`로 낮췄고,
일반 도로 false-hole은 `1.304% → 1.353%`로 `0.049%p` 증가했다. 같은 공통 GT의
drivable/lane IoU는 각각 `0.98364/0.89332`로 A보다 소폭 높아 회귀가 없었다. 따라서
B dataset `twinlite_morai_v5_marking_actor_negative`와 checkpoint
`twinlite_medium_morai_v5_actor_negative_init_ema_b12_e18_20260906/best_mean.pt`를
현재 live 및 다음 학습의 잠정 기준으로 채택한다. 독립 test 전까지 최종 확정으로 보지는
않으며 원본 A checkpoint와 비교 산출물도 유지한다.

4-class 학습용 `twinlite_morai_v6_road_marking_actor_negative`는 B와 같은 12개 train
run 26,001 sample, 6개 val run 1,404 sample, 4-frame 제외 목록과 cache를 그대로 쓴다.
활성 head만 `drivable,road_marking`으로 바꿨고 27,405 sample의 파일·shape·binary 및
categorical 값 범위를 전수 검사했다. `road_marking` encoding은
`background=0, white_lane=1, yellow_lane=2, stopline=3`이다.

`20260903_211826_kcity_comp_sample_sunny_sim_unknown_wheel`은 수집자 확인에 따라 실제
MORAI 환경 시각을 **sunny 1pm 추정**으로 해석한다. 다만 확정 timestamp가 없는 원본
lineage를 사후에 단정하지 않기 위해 물리 `run_id`와 manifest는 바꾸지 않고, 조건별 분석과
후속 수집 대장에서만 이 override를 적용한다.

`kcity_auto_sunny_001`, `002`는 `$ASMC_DATA/rejected_runs/`에 거부 기록만
남기며 학습·평가 manifest에서 제외한다. 앞으로 잘못된 run은 센서 파일 삭제 전에
`capture_runs/<run_id>` 메타데이터를 `rejected_runs/<run_id>`로 이동하고
`rejection.json`을 추가한다.

## 2. 원본 채널의 역할

| Capture 채널 | 역할 | TwinLite 사용 |
|---|---|---|
| `Intensity` | 실제 배포 시 입력과 같은 RGB | 입력 |
| `Semantic` | MORAI 26.R1 클래스별 무손실 RGBA | 라벨 원본 |
| `Instance` | 객체별 instance ID | 보존, 현재 TwinLite 미사용 |
| `Depth` | 카메라 깊이 GT | 보존, 현재 TwinLite 미사용 |

카메라는 `front`, `left`, `right`를 공유 가중치 TwinLite 학습 표본으로 사용한다.
`traffic_light`(`CAMERA_4`)는 차선·drivable 학습에서 제외하고 별도 신호등 모델용으로
보존한다. 객체 검출은 현재 LiDAR-primary이므로 TwinLite에 detection head를 추가하지 않는다.

GT 수집은 RGB 카메라와 같은 `CAMERA_1~4`에서 Capture Mode로 Intensity/Semantic/
Instance/Depth를 한 trigger에 저장한 결과를 사용한다. 별도 Semantic 카메라를 추가하는
`ASMC_sensor_set_GT.json` 방식은 카메라 간 시계열·외부 파라미터 정합 부담이 있으므로
현재 파이프라인에서는 사용하지 않는다.

Simulation Time의 Sensor Record는 현재 장착된 센서만 저장한다. 2026-09-14 v2 실측에서도
`CAMERA_1~4`, `LIDAR_5`, `GPS_6`, `IMU_7`만 갱신됐으며 Semantic 카메라 디렉터리는
갱신되지 않았다. 따라서 v2 Sensor Record 결과를 RGB–Semantic 학습 pair로 간주하지 않는다.
향후 5~10 Hz supervised sequence가 실제로 필요할 때만 RGB·Semantic front/left/right와
필요한 상태 센서로 구성한 최소 sync-training 센서셋을 별도 version으로 만들고, frame key,
timestamp, 해상도, extrinsic, 누락률을 검증한 새 manifest로 기존 Capture dataset과 분리한다.

## 3. Semantic → 정답 마스크

### 3.1 공식 26.R1 RGB 팔레트

| 클래스 | RGB | Lane | Drivable |
|---|---|---:|---:|
| Asphalt | `(127,127,127)` | 0 | 1 |
| White Lane | `(255,255,255)` | 1 | 1 |
| Yellow Lane | `(255,255,0)` | 1 | 1 |
| Blue Lane | `(0,178,255)` | v3: 0 | v3: 0 |
| Crosswalk | `(76,255,76)` | 0 | 1 |
| Stop Line | `(255,0,0)` | 0 | 1 |
| Road Sign | `(204,127,51)` | 0 | 1 |
| Road Edge | `(178,178,178)` | 0 | 0 |
| Sidewalk 및 기타 | 공식 팔레트 값 | 0 | 0 |

기존 binary v1/v2 Lane은 White/Yellow/Blue를 foreground로 했으나, 전체 accepted-run
감사에서 Blue가 0 pixel임을 확인한 v3부터 대회 학습 target은 White/Yellow만 사용한다.
Crosswalk, Stop Line, Road Edge는 binary lane에 넣지 않는다. 공식 팔레트의 Blue 항목은
향후 시뮬·맵 변화 감시를 위해 제거하지 않고, v3 bake 시 Blue가 한 픽셀이라도 나타나면
실패시키는 감사 조건으로 남긴다.

Drivable은 단순히 Asphalt만 선택하지 않고 표면에 덧그려진 Lane, Crosswalk,
Stop Line, Road Sign을 합친다. 이들을 제외하면 같은 도로 위에서 페인트 부분만
background 구멍이 되기 때문이다. 보도와 Road Edge는 제외한다.

현재 v2 정책 `kcity_static_surface_actor_ignore_v2`에서는 차량 계열, 보행자,
`obstacle` 픽셀을 drivable loss에서 **ignore**한다. 마스크 값을 단순 background로
두는 것과 달리, `drivable_valid = geometric_valid AND NOT drivable_ignore`로 만들어
해당 픽셀의 gradient와 IoU 집계 자체를 제외한다. 정적 도로 표면을 배우는 head가
우연히 배치된 객체를 비주행 표면으로 암기하지 않게 하기 위함이다.

같은 actor 픽셀은 별도 `actor_occupancy_gt`로 원본 해상도에 보존해 LiDAR 점유 및
카메라-LiDAR 융합 평가에 쓴다. `standing_object`는 터널 구조물·가드레일·기둥 등
상시 도로변 구조물이므로 transient actor ignore가 아닌 명시적 background다.

Binary canonical PNG 마스크 값은 `{0,1}`의 `uint8`로 고정한다. v3
`road_marking`만 `background=0`, `white_lane=1`, `yellow_lane=2`, `stopline=3`의
categorical `uint8`이다. 여기서 background는 이 head에서 세 표식 중 어느 것도 아닌
유효 영상 픽셀이며 drivable과 같은 뜻이 아니다. QA용 overlay만 사람이 보기 좋은 색으로
시각화한다. 공식 팔레트에 없는 픽셀, 깨진 파일, 기하 불일치는 억지로 background로
바꾸지 않고 해당 frame을 reject한다.

### 3.2 v2 전체 팔레트 감사

신규 8개 run의 14,529개 3-view sample, 총 7,438,848,000 pixel을 전수 검사했다.
공식 팔레트 밖 색상은 0 pixel이었다. Actor 양성 sample은 9,384개, actor 픽셀은
28,072,796개였으며 그 픽셀의 Instance nonzero 비율은 100%였다.

| 주요 클래스 | 픽셀 비율 |
|---|---:|
| Asphalt | 27.07% |
| White Lane | 1.12% |
| Yellow Lane | 0.59% |
| Crosswalk | 0.44% |
| Stop Line | 0.12% |
| Road Sign | 0.15% |

기존 100-frame 감사 수치는 초기 sunny baseline 참고치로만 유지한다. 실제 v2 변환은
전수 감사에서 검증한 공식 팔레트와 `config/perception_target_policy_v2.json`을 사용한다.

### 3.3 전체 accepted-run road-marking 감사와 v3 정책

2026-09-06에 18개 accepted run의 curation 전 원본 10,030 frame, 세 카메라
30,090 sample, 15,406,080,000 pixel을 다시 전수 검사했다. 미등록 RGB와 Blue는 모두
0 pixel이었다. 집계 파일은
`$ASMC_DATA/capture_batches/20260906_all_accepted_runs_road_marking_audit.json`이며
SHA-256은 `2ac8fcce949070efb9723787434a0849ad608869b6aa9ee3d1bbe7f4a9959948`이다.

| View | White pixels / 양성 frame | Yellow pixels / 양성 frame | Stopline pixels / 양성 frame |
|---|---:|---:|---:|
| Front | 82,640,724 / 10,030 | 36,820,245 / 9,883 | 8,205,761 / 7,245 |
| Left | 25,481,188 / 10,028 | 54,264,914 / 8,806 | 2,868,453 / 5,680 |
| Right | 70,218,077 / 10,025 | 1,308,464 / 9,183 | 1,849,170 / 5,730 |
| 합계 | **178,339,989 / 30,083** | **92,393,623 / 27,872** | **12,923,384 / 18,655** |

Stopline이 좌·우 view에도 각각 5천 frame 이상 존재하므로 향후 3-camera BEV를 위해
all-view 정답을 보존한다. 좌/우 Yellow 픽셀 수가 크게 비대칭이므로 학습·평가 때 전체
평균만 보지 않고 view별 class IoU/recall을 필수로 보고한다. 동결 정책은
`config/perception_target_policy_v3.json`이다.

### 3.4 drivable 장애물 정책 A/B 결과와 채택 상태

- A: v5와 같은 actor-ignore. Actor 픽셀의 drivable valid를 0으로 둔다.
- B: 같은 actor 픽셀의 drivable target을 0, valid를 1로 두어 non-drivable loss를 준다.
- 두 실험은 split, 384×640 geometry, augmentation, seed, 초기 checkpoint, epoch와
  lane target을 동일하게 유지한다. Road-marking classifier 변경을 이 실험에 섞지 않는다.
- 비교값은 일반 drivable IoU뿐 아니라 actor 영역 carve-out rate, actor 없는 도로의
  false-hole rate, view/weather별 IoU, lane 성능 회귀와 실시간 지연을 포함한다.

B가 actor 구멍을 분명히 만들면서 actor 없는 도로의 false-hole이나 lane/drivable 전체
성능을 허용 범위 이상 떨어뜨리지 않는 조건을 충족해 **잠정 채택**했다. 공통 1,404-sample
평가와 선택 overlay는
`artifacts/perception_eval/analysis/twinlite_actor_policy_ab_on_v5_val_20260907/`에
보존한다. 어느 쪽이든 최종 safe-area는 카메라 단독 판단값이 아니라 LiDAR occupancy와
결합한다.

## 4. 기하 전처리 계약

목표 입력은 TwinLiteNet+ Medium의 `H×W=384×640`이다. RGB, lane mask,
drivable mask에 동일한 scale과 padding을 적용한다.

| View | 원본 | resize | padding |
|---|---|---|---|
| front | 720×1280 | 360×640 | top/bottom 12 px |
| left/right | 480×640 | 384×512 | left/right 64 px |

- RGB downscale: `INTER_AREA`; 그 외 RGB resize: `INTER_LINEAR`
- mask resize: 항상 `INTER_NEAREST`
- letterbox padding: `valid_mask=0`; 실제 영상 영역은 `valid_mask=1`
- RGBA Intensity는 alpha를 제거하고 RGB 3채널만 모델에 입력
- `geometry.jsonl`에 `original_hw`, `scale`, `resized_hw`, `pad_ltrb`,
  `valid_roi`를 기록해 출력 결과를 원본 좌표로 복원

Upstream `BDD100K.py`의 front 전용 `640×360` resize/crop은 side view에 맞지 않으므로
그대로 사용하지 않는다. MORAI manifest 기반 dataset adapter를 별도로 작성한다.

### 4.1 배포 센서 해상도 결정 시점

현재 수집 원본은 규정 상한인 Front `1280×720`, Left/Right `640×480`이며 이를
1차 학습의 기준으로 사용한다. 최종 배포 센서 해상도는 마스크 생성 전에 임의로 낮추지
않고, 1차 모델을 세 카메라 실시간 스트림에 연결해 FPS, end-to-end latency, UDP drop,
CPU/GPU 사용량을 측정한 뒤 확정한다. 상한 설정이 안정적이면 그대로 유지한다.

부하 때문에 낮춰야 하면 위치·각도·FOV와 원본 종횡비를 유지하는 별도 저부하 sensor
profile을 만들고, 해당 profile의 live RGB validation run으로 성능을 다시 확인한다.
해상도 결정은 초기 mask bake와 baseline 학습을 막지 않지만 최종 학습·배포 검증 전에는
동결해야 한다.

## 5. 파생 데이터와 버전

원본 RGB를 다시 복사하지 않고 기존 `frames/intensity`를 참조한다.
Semantic 해석과 모델 입력 변환은 논리적·물리적으로 두 단계로 분리한다.

1. **Canonical target**: 원본 해상도에서 palette를 binary 또는 categorical head mask로 변환한다.
2. **Model cache**: canonical mask를 특정 모델 입력 규격으로 letterbox하고 valid mask와
   복원 geometry를 만든다.

두 단계는 한 상위 명령으로 연속 실행할 수 있지만 각각 독립 manifest와 완료 marker를
가진다. 모델 해상도나 backbone이 바뀌면 canonical target은 재사용하고 model cache만
다시 생성한다.

```text
$ASMC_DATA/datasets/<run_id>/
  frames/                                  # 동기화 정본, 수정 금지
  derived/curation_v2/                     # 원본 보존형 frame 선택 결정
    manifest.jsonl
    summary.json
  derived/perception_targets_native_v2/    # 모델 독립 canonical target
    lane_masks/{front,left,right}/
    drivable_masks/{front,left,right}/
    stopline_masks/{front,left,right}/
    drivable_ignore_masks/{front,left,right}/
    actor_occupancy_gt_masks/{front,left,right}/
    manifest.jsonl
    qa_summary.json
  derived/twinlite_384x640_v3/              # 재생성 가능한 모델별 cache
    lane_masks/{front,left,right}/
    drivable_masks/{front,left,right}/
    stopline_masks/{front,left,right}/
    drivable_ignore_masks/{front,left,right}/
    valid_masks/{front,left,right}/          # geometric valid
    drivable_valid_masks/{front,left,right}/
    geometry.jsonl
    qa_summary.json
  derived/perception_targets_native_v3/     # v2 binary + 4-class road marking
    road_marking_masks/{front,left,right}/  # 0=background, 1=white, 2=yellow, 3=stopline
  derived/twinlite_384x640_marking_v1/       # v3의 384×640 nearest cache
    road_marking_masks/{front,left,right}/

$ASMC_DATA/dataset_versions/twinlite_morai_v4_comp8_train_val_sample_b/
  dataset.json
  train.jsonl
  val.jsonl
  test.jsonl
```

라벨 정책, 해상도, split 중 하나라도 바꾸면 기존 결과를 덮어쓰지 않고 버전을 올린다.

## 6. Split 정책

연속 1 Hz 프레임은 서로 매우 비슷하므로 frame 단위 무작위 split을 금지한다.
`run_id` 전체를 한 split에만 배정한다.

초기 sunny baseline은 아래와 같았으며 v2에서는 독립 검증·테스트 run을 확보했으므로
기존 4개 run을 모두 train으로 재배정했다.

```text
train = kcity_drive_001 + kcity_drive_002 + kcity_auto_sunny_003
val   = kcity_drive_003
test  = 비워 둠 — 이후 별도 재주행으로 수집
```

v2 split은 5pm 학습 전 OOD 기준, v3 split은 `drive_003` 연속성 기준을 남기기 위한
이력이다. 실제 재학습용 v4에서는 기존 12개 run을 모두 train으로 사용하고, 이후 별도로
주행한 `comp_sample_b` 6개 run만 val로 사용한다. 같은 한 바퀴를 여섯 구간으로 나눴으므로
시간·날씨별 경로 구간이 서로 다르다는 교란은 남아 있다. 따라서 checkpoint 선택과 실제
대회 조건 회귀 탐지에는 사용하되 순수한 날씨 인과 비교로 해석하지 않는다. 최종 test는
재학습 이후 규정 시간 11am/1pm/3pm에서 다시 수집해 한 번만 평가한다.

향후 날씨 데이터는 단순히 sunny frame과 섞고 random split하지 않는다.

1. 각 날씨(clear/rain/fog/night)에서 독립 run 수집
2. 같은 연속 주행은 한 split에만 배치
3. train/val에는 필요한 날씨를 모두 넣되 경로 또는 seed를 분리
4. robustness test에는 완전히 보류한 run 사용

## 7. 라벨·split QA 게이트

라벨 bake 완료 조건:

- RGB/Semantic native shape 일치
- 입력 frame마다 lane/drivable/valid mask 존재
- binary mask 고유 값이 `{0,1}`, road-marking mask가 `{0,1,2,3}`의 부분집합
- v3 전체 입력에서 Blue Lane 0 pixel; 새 Blue 발견 시 fail-fast
- RGB와 세 mask의 scale/pad 동일
- 비공식 Semantic 색상 0개 또는 reject 명단 존재
- view/run별 lane·drivable pixel 비율 report
- 100개 이상 RGB+mask overlay gallery 육안 검토
- train/val/test 사이 `run_id` 및 `(run_id, frame_id, view)` 중복 0
- 각 split manifest가 존재하는 파일만 참조

학습 게이트:

1. front 32~100 sample tiny-overfit
2. lane/drivable head가 뒤바뀌지 않았는지 overlay 확인
3. 세 view를 공유 가중치로 학습하되 view별 IoU 별도 보고
4. TwinLiteNet+ Medium을 먼저 평가
5. 같은 split·전처리로 원조 YOLOP `SEG_ONLY`를 폴백 비교

## 8. 정지선·3-view 확장 결정

- 정지선은 `Stop Line=(255,0,0)`에서 독립 binary mask와 4-class road-marking mask를
  함께 생성한다. binary lane에는 합치지 않고 drivable에는 도로 표면의 구멍을 막기 위해
  계속 포함한다.
- 1순위 확장안은 TwinLite lane decoder의 최종 classifier를
  `background/white/yellow/stopline` 4-class로 바꾸는 것이다. binary lane은 기존 2-head
  checkpoint 비교와 presence 파생을 위한 호환 target으로 보존한다.
- 공유 encoder와 drivable/lane 학습에는 front/left/right를 모두 독립 표본으로 사용한다.
  정지선 정답과 향후 loss도 all-view를 사용하되, 실제 정지 거리 평가는 front와 BEV에서
  각각 보고하고 view별 class IoU를 따로 계산한다.
- 배포 출력은 정지선 mask 자체에 더해 `stamp`, `distance_m`, `confidence`, `valid/age`를
  포함한다. 신호 결합·감속·정지는 판단/제어 모듈의 책임으로 둔다.

대회 이후에는 원본 3-view와 calibration을 보존한 채 카메라별 segmentation logit을
ego-BEV로 투영·융합하고, 이후 learned view transform과 temporal/vector map으로
확장한다. 이를 위해 새 run에는 sensor-set 사본·hash, 카메라 K/D/T, sim timestamp,
weather/route/controller를 자기완결 메타데이터로 남긴다. 현재 1 Hz Capture 데이터는
단일 프레임 인지에는 사용한다. temporal BEV/E2E 연구용 5~10 Hz sequence는 RGB만 필요한지
RGB–Semantic supervised pair가 필요한지 먼저 구분하고, 후자의 경우에만 최소 sync-training
센서셋을 사용한다. Simulation Time sync 결과는 ideal-input 분석, Real Time UDP bag 결과는
실제 운용 분석으로 서로 대체하지 않고 함께 보존한다.

## 9. 다음 구현 순서

완료된 단계:

1. Semantic palette mapper와 native `lane/drivable/stopline` canonical mask baker
2. canonical mask → paired letterbox/`valid`/geometry model-cache writer
3. 정차 근접 중복을 원본 비파괴 방식으로 거르는 `curation_v2`
4. actor-ignore/occupancy를 포함한 native v2와 TwinLite `384×640` v3 cache
5. v4 run-based split manifest와 27,405 sample 전체 mask 경로·shape·이진값 검사
6. 기존 2-head 체크포인트를 고정한 새 val/test 기준 추론
7. 전체 18개 run의 White/Yellow/Blue/Stopline 감사와 Blue 0-pixel gate
8. Blue 제외 4-class road-marking 및 all-view stopline native/cache 생성
9. 동일 split/checkpoint actor-ignore(A) 대 actor-negative(B) 통제 평가 및 B 잠정 채택
10. B 기반 v6 `drivable+road_marking` manifest 전수 검사와 2→4 출력 preflight
11. sunny/foggy×3-view×white/yellow/stopline 균형 18장 v6 tiny-overfit 통과
12. v6 전체 18 epoch 본학습 완료: best mean `0.93949`, road-marking mIoU `0.89555`
13. checkpoint task 자동 감지 live backend와 `road_marking/class_id` 3-view 시각화 연결

다음 단계:

1. MORAI live에서 4-class 색·경계와 front/3-view 지연을 조건별로 재측정
2. 특히 희소한 right-view yellow와 stopline false positive/recall을 육안·수치 확인
3. 문제가 확인될 때만 class weight 또는 sampling을 통제 비교
4. 규정 시간대의 새 독립 test를 수집해 B binary와 4-class checkpoint를 평가
5. camera/map/LiDAR 공통 BEV pilot과 차로 구조화 진행

### 9.1 2026-09-07 v6 4-class tiny-overfit 결과

`config/twinlite_v6_tiny_overfit_samples.json`은 sunny/foggy와 front/left/right의 각
조합에서 white/yellow/stopline 픽셀이 많은 표본을 하나씩 골라 18장으로 구성했다. Medium
모델을 v5 B EMA에서 초기화하고 batch 3, LR `5e-4`, AMP, class weight 없음으로 80 epoch
학습했다. 총 137.4초가 걸렸고 overfit contract와 overlay 육안 검사를 모두 통과했다.

| 지표 | 학습 전 | 80 epoch |
|---|---:|---:|
| masked eval loss | 1.55734 | 0.82984 |
| drivable IoU | 0.98441 | 0.99029 |
| road-marking foreground mIoU | 0.03041 | 0.79188 |
| white IoU | - | 0.76539 |
| yellow IoU | - | 0.72264 |
| stopline IoU | - | 0.88762 |

이는 4-class 인코딩, 2→4 출력 교체, loss, metric과 checkpoint 배관 검증 결과다. 희소
정지선을 의도적으로 농축한 동일 18장 평가이므로 일반화 지표나 class weight 근거로 쓰지
않는다. 전체 train에서 road-marking 유효 픽셀 분포는 background `97.8838%`, white
`1.3265%`, yellow `0.7012%`, stopline `0.0886%`다. Multiclass Tversky가 class별
겹침을 이미 균등하게 다루므로 첫 본학습에는 공격적인 역빈도 가중치를 적용하지 않고,
validation의 class별 recall/false positive를 보고 후속 실험을 결정한다.

2026-09-04~05 v4 10/18 epoch 학습 결과와 overlay·수렴 분석은
[perception-v4-training-evaluation.md](../../records/perception/twinlitenet-v4-evaluation.md)를 따른다.

### 9.2 2026-09-02 Medium binary tiny-overfit 결과

공식 external checkout commit `90f1b8695ae311d5123b05f8534b2e11e42499d2`의
`medium` 모델을 RTX 4060 Ti에서 실제 `twinlite_morai_v1` train 12 sample로 학습했다.
Batch 3, AdamW LR `5e-4`, AMP, augmentation 없음, 30 epoch이며 총 43.4초가 걸렸다.

| 지표 | 학습 전 | 30 epoch |
|---|---:|---:|
| masked eval loss | 1.1534 | 0.6720 |
| drivable foreground IoU | 0.2975 | 0.9561 |
| lane foreground IoU | 0.0221 | 0.6035 |

두 head와 CUDA backward가 정상 동작한다는 G1 배관 검증 결과다. 같은 12 sample을 평가한
overfit 수치이므로 일반화 성능으로 해석하지 않는다. Overlay에서는 세 view의 drivable
형태가 정답을 따르며 lane은 근거리 굵은 표식부터 학습됐다. Side valid ROI 경계의 가는
lane 오검출은 본학습에서 별도 확인한다.

### 9.3 2026-09-02 Medium 전체 데이터 10 epoch 결과

`twinlite_morai_v1`의 train 13,227 sample, validation 918 sample을 모두 사용했다.
입력은 `384x640`, batch 12, AdamW LR `5e-4` + polynomial decay, AMP와 EMA를
사용했으며 augmentation은 적용하지 않았다. 유효 영상 영역은 거리별 추가 가중치 없이
동일한 공간 가중치를 적용하고 padding만 제외했다. RTX 4060 Ti에서 10 epoch 총
7,377.3초, peak CUDA memory 약 2.50GB가 걸렸다.

| 지표 | epoch 1 | best |
|---|---:|---:|
| drivable foreground IoU | 0.9460 | 0.9778 (epoch 9) |
| lane foreground IoU | 0.7726 | 0.8597 (epoch 10) |
| 두 head 평균 IoU | 0.8593 | 0.9187 (epoch 10) |

Epoch 10 카메라별 IoU는 front `DA 0.9692 / lane 0.7592`, left
`0.9810 / 0.8906`, right `0.9830 / 0.8912`다. Front lane이 side보다 낮으므로
독립 test run과 더 먼 얇은 차선·교차로 구간에서 우선 검증한다. 배포 후보는 EMA가 포함된
`artifacts/perception_eval/runs/twinlite_medium_morai_v1_uniform_b12_e10_20260902/best_mean.pt`다.

### 9.4 2026-09-04 v2 무학습 기준 추론

기존 `twinlite_medium_morai_v1_uniform_b12_e10_20260902/best_mean.pt`의 epoch 10
EMA 가중치를 고정하고 `twinlite_morai_v2_curated`의 val/test만 추론했다. 체크포인트
SHA-256은 `a88794304c149160be239e9dafe0f54d00808ab9358224e3c4c933c1ccc0d94d`이며
가중치 업데이트는 없었다.

| Split | Lane IoU | Drivable IoU | 평균 head IoU | 처리량 |
|---|---:|---:|---:|---:|
| val (3pm) | 0.7131 | 0.7953 | 0.7542 | 51.3 sample/s |
| test (5pm) | 0.3302 | 0.5696 | 0.4499 | 52.6 sample/s |

test 하락은 새 장애물·NPC가 포함된 5pm run에 대한 기존 모델의 도메인 차이를 드러내는
학습 전 기준점이다. 특히 lane이 가장 크게 약화됐다. 이후 5pm run을 v3 train으로
재배정했으므로 이 결과는 학습 전 이력으로만 보존하고 재학습 후 공정한 test로 재사용하지 않는다.
대표 overlay에서는 5pm 곡선의 차선·도로 누락, 낮은 태양 고도와 안개 장면의 하늘
drivable 오검출이 확인됐다. sunny/foggy 대표 12장과 낮은 IoU 4장을 결과 디렉터리에
함께 보존했다.
상세 결과는
`artifacts/perception_eval/baselines/twinlite_medium_morai_v1_on_v2_curated_20260904/metrics.json`에
run/view/weather별로 저장했다.

### 9.5 2026-09-04 v3 재학습 직전 기준

`twinlite_morai_v3_comp8_train_val_drive003`은 train 25,083 samples, val 918 samples,
test 0 samples다. 모든 신규 comp 8개 run과 5pm을 train에 포함하고 기존
`kcity_drive_003`을 val로 유지했다. 전체 마스크를 다시 전수 검사했고 split sample
중복은 0개다.

기존 epoch 10 EMA를 고정해 val에 추론한 연속성 기준은 Lane IoU `0.8597`, Drivable
IoU `0.9777`, 평균 `0.9187`이다. 이 checkpoint 자체가 과거 같은 val에서 선택됐으므로
독립 일반화 수치로 해석하지 않는다. 결과는
`artifacts/perception_eval/baselines/twinlite_medium_morai_v1_on_v3_comp8_val_drive003_20260904/`
에 checkpoint/dataset/manifest hash 및 overlay와 함께 저장했다.

재학습 초기화는 기존 EMA 모델 가중치만 가져오고 optimizer, scheduler, 새 EMA와 best
점수를 초기화한다. `train_twinlite.py --preflight-only`로 train/val 각 12 samples를
forward한 결과 두 출력 모두 `[12,2,384,640]`, loss finite, `training_performed=false`를
확인했다. 기록은
`artifacts/perception_eval/preflight/twinlite_medium_morai_v3_init_ema_20260904/`에 둔다.

### 9.6 2026-09-04 v4 독립 validation 기준

최종 재학습 버전 `twinlite_morai_v4_comp8_train_val_sample_b`는 train 26,001 sample,
val 1,404 sample, test 0 sample이다. 기존 12개 run은 모두 train에 두고, 이후 별도로
주행한 sunny/foggy × 11am/1pm/3pm 여섯 구간을 val에만 배정했다. sunny 11am의 실제
날씨 전환 frame 0~3은 명시적으로 제외했으며 원본과 cache는 보존한다.

기존 epoch 10 EMA를 고정해 얻은 재학습 전 기준은 다음과 같다.

| 범위 | Lane IoU | Drivable IoU | 평균 head IoU |
|---|---:|---:|---:|
| val 전체 | 0.7926 | 0.8494 | 0.8210 |
| sunny | 0.8469 | 0.9443 | 0.8956 |
| foggy | 0.7442 | 0.7722 | 0.7582 |
| front view | 0.7216 | 0.7789 | 0.7503 |
| foggy 3pm 구간 | 0.6952 | 0.7383 | 0.7167 |

결과는 `artifacts/perception_eval/baselines/`의
`twinlite_medium_morai_v1_on_v4_sample_b_val_20260904/`에 저장했다. v4 manifest로
다시 수행한 warm-start preflight도 train/val 각각 12 sample에서 두 출력
`[12,2,384,640]`, finite loss, `training_performed=false`를 확인했다. 기록은
`artifacts/perception_eval/preflight/twinlite_medium_morai_v4_clean_init_ema_20260904/`에
있다.

재학습 checkpoint 선택 시 전체 평균이 `0.8210`보다 높은지만 보지 않고 front `0.7503`,
foggy `0.7582`, foggy 3pm `0.7167`의 회귀 여부를 함께 본다. 이 val은 모델 선택에 쓰므로
최종 성능 보고용 test로 재사용하지 않는다.

## 10. 객체 포함 데이터 v2 결정 및 적용

아래 전환 게이트는 2026-09-04에 모두 완료했다.

1. 차량 계열, 보행자, `obstacle`, `standing_object`, ego hood가 실제 Capture Mode
   Semantic에서 어떤 색과 형태로 나오는지 소량 pilot run으로 감사한다.
2. 차량 계열·보행자·명시적 obstacle을 drivable loss에서만 제외한다.
3. 공통 `valid_mask`와 별도로 `drivable_valid_mask`를 만들고 mask baker, cache,
   manifest, dataset adapter, loss, IoU를 함께 버전업한다.
4. 객체가 없는 기존 run과 객체가 있는 신규 run을 함께 보존하고, 기존 v1 모델의
   무학습 기준 성능을 먼저 측정한다.
5. 정지선은 독립 head 대신 all-view 4-class road-marking의 한 class로 학습하고,
   MGeo/전역경로 정지점을 배포 1차 기준으로 둔다.

원본 Intensity/Semantic/Instance/Depth는 정책과 무관하게 보존하므로, 수집을 다시 하지
않고도 background/ignore 정책을 바꿔 파생 데이터셋을 재생성할 수 있다.

### 10.1 2026-09-04 객체 포함 batch 감사 및 v2 정책 동결

주최 측 예시 시나리오의 맑음/안개·시간대 변형 8개 run을 동기화했다. 성공 요청
4,851개 중 sunny 11am의 Instance/Depth 일부가 누락된 8개 capture는 원본을 보존하고
Silver dataset에서 명시적으로 제외했다. 최종 정본은 4,843 capture, 학습 카메라 3대
기준 14,529 sample이며 87,174개 파일의 완전성을 확인했다. foggy 1pm의 gRPC timeout
sequence 521은 애초 manifest에서 실패이므로 성공 정본에 포함되지 않는다.

세 학습 view의 Semantic 14,529장, 7,438,848,000 pixel을 공식 팔레트로 전수 검사한
결과 미등록 RGB는 0 pixel이었다. 차량 계열·보행자·`obstacle`은 9,384 sample,
28,072,796 pixel에서 검출됐고 이 픽셀은 Instance에서 모두 nonzero였다. `obstacle`은
스폰한 콘·위험물/화염 소품으로, `standing_object`는 터널 골조·가드레일·지주 등
상시 고정 구조물로 확인됐다. 정지선은 front 4,843장 중 3,352장에 존재했다.

이에 `kcity_static_surface_actor_ignore_v2` 정책을 동결한다.

- Drivable ignore: 차량 계열, 보행자, `obstacle`.
- Drivable background: `standing_object`, sidewalk/building/road edge 등 고정 비도로와
  고정된 ego hood/body.
- Lane/stopline: visible-only. actor 뒤를 복원하지 않으며 별도 actor ignore를 적용하지
  않는다.
- Stopline: drivable positive를 유지하면서 독립 target으로 저장한다. 초기 binary
  stopline head 구상은 front-only였지만, 현재 4-class road-marking loss는 3-camera BEV
  확장을 위해 all-view에 적용한다.
- 같은 actor class로 `actor_occupancy_gt`를 별도 보존하되 TwinLite 학습 target으로 쓰지
  않는다.

정책 원본은 `config/perception_target_policy_v2.json`, 전수 통계는 workspace의
`data/capture_batches/20260903_20260904_kcity_comp_sample_semantic_audit.json`, 대표 RGB/
Semantic overlay는 같은 이름의 `_qa` 디렉터리에 둔다. v2 native/cache와 dataset
manifest에는 정책 ID·정책 SHA-256·curation manifest SHA-256을 기록했고, adapter/loss/
IoU는 head별 valid 정의를 사용한다. 각 run의 `_SUCCESS`가 완료 여부의 기준이다.
