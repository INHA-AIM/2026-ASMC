# 카메라 객체 검출 인수 가이드 — 손재호

**현재 계약 — 2026-10-14:** 기본 YOLO26l v11 E8·16종·3카메라15Hz와 세 소비 그룹은
[현재 배포 계약](camera-mission16-deployment.md)을 따릅니다. 아래 YOLO11m/P2a/구 그룹/0.7 cutoff는
당시 인계 기록입니다. 이번 배포의 confidence는0.40, Main 추가 cutoff는 기본0.0이며 원본 라벨은 유지합니다.


> **2026-10-09 통합 작업:** [3뷰 COCO7 실행·모델 선정](camera-three-view-coco-runtime.md)을 현재 시작점으로 사용합니다. 아래 기존 실험과 실패 증거는 보존합니다.

> **문서 상태:** Active Guide / Handoff
> **Handoff 허브:** [README.md](../README.md)
> **Perception 경계:** [Perception 개발·통합 가이드](../../packages/asmc_perception/README.md)
> **기준일:** 2026-10-06 (P2a 보완; 과거 학습 수치는 당시 기록)
> **대상:** LiDAR·센서융합 담당 손재호, 카메라 담당 안승현
> **범위:** 기존 detector 즉시 사용, 경량 후보 선택, NAS Capture 데이터 fine-tune 준비
> **현재 구현:** [P2a 생산자·어댑터](camera-objects-p2a.md). 학습·모델 선택·동시 부하는 후속 단계
> **2026-10-08 카메라 다음 단계:** [YOLO11m COCO 클래스 확대](camera-coco-class-expansion.md)는
> 기존 네 클래스 기본값과 LiDAR 호환 정책을 유지하는 별도 80종 관측 실험이다.

> **2026-10-07 후속 연구:** [카메라와 LiDAR 장애물 융합 분석](camera-lidar-fusion-study.md)에
> YOLO26 detect/seg, 공개 데이터·가중치, 터널 객체 분리와 full-stack 부하 비교를 정리했다.
> 아래 YOLO11m 긴급 실행 경로는 유지한다. 연구 비교에서는 YOLO26-seg를 MORAI fine-tune
> 이전에도 시험할 수 있으며, 아래 O0~O3의 순서를 필수 선행 조건으로 강제하지 않는다.
> 새 모델·mask는 아직 운영 채택이나 기존 메시지 변경이 아니다.

## 1. 먼저 내릴 결정

객체 분류가 급하면 **저장소의 기존 COCO YOLO11m을 그대로 사용**한다. 현재 ROS launch와
class filter가 이 weight에 맞아 있어 가장 빨리 LiDAR fusion을 진행할 수 있다.

1. `YOLO11m`으로 먼저 기능과 MORAI 오검출 양상을 확인한다.
2. 지연·VRAM이 부담일 때 동일 조건에서 공식 `YOLO11s`를 비교한다.
3. 그다음 후보가 `YOLO26s`다. 현재 Ultralytics·export·ROS 환경 호환을 먼저 확인해야 하므로
   긴급 경로의 첫 선택은 아니다.
4. COCO 모델의 MORAI 보행자 recall이 부족할 때 NAS 데이터를 사용해 3-class fine-tune한다.
5. 위 경량화 뒤에도 full-stack 지연 기준을 실패할 때만 unified model이나 TwinLiteNet+
   detection head를 검토한다.

카메라 공통 그룹은 **`person / vehicle / obstacle`**로 고정한다.

- `person`: person, bicycle, pedestrian
- `vehicle`: car, truck, bus, motorcycle, train과 MORAI 차량 이름
- `obstacle`: 그 밖의 라벨과 MORAI Obstacle; traffic light와 stop sign은 제외

원래 라벨은 `source_class_name`에 별도로 남긴다. P2a의 실제 추론 허용 라벨은 기존 네 개
`bus/truck/person/car`로 유지한다. 그룹을 추가해도 기존 LiDAR fusion의 분류 정책은 바꾸지 않는다.
카메라 미검출은 LiDAR 객체 삭제 근거가 아니다. NAS 3클래스 학습·소비자 매핑은 후속 단계에서 검증한다.

카메라 담당의 현재 1순위는 판단 파트에 음영 구간 차선·주행가능영역 정보를 넘기는 일이다.
따라서 객체 작업은 이 문서를 기준으로 LiDAR·fusion 파트가 독립 착수하고, 신규 학습이나
adapter 수정이 실제로 필요해진 시점에 카메라 파트와 합류한다.

## 2. 저장소에서 확인된 현재 상태

| 항목 | 확인 결과 | 의미 |
|---|---|---|
| 기본 weight | `src/lidar+sensorfusion/src/Main/YOLOWeight/yolo11m.pt` | 공식 COCO 80-class YOLO11m |
| SHA-256 | `d5ffc1a674953a08e11a8d21e022781b1b23a19b730afc309290bd9fb5305b95` | 전달·복사 뒤 동일 weight 확인용 |
| 현재 launch | [`main.launch`](../../../src/lidar+sensorfusion/src/Main/main.launch) | 위 YOLO11m을 지정 |
| 현재 입력/출력 | 전방 `/camera/image/compressed` → `/YOLO/BBox` | 전방 detector 한 개만 실행 |
| 현재 허용 이름 | `bus`, `truck`, `person`, `car` | COCO weight와 바로 호환 |
| 기본 confidence | `0.7` | 편의 기본값일 뿐, 보행자 recall 검증값은 아님 |
| 다른 로컬 weight | `yolo11m(1).pt` | 파일명과 달리 4-class custom YOLO11s |
| custom s SHA-256 | `6cf89c17d9a1122b9b6f3f6abe7574bf3f80cc4653e549d1bfd102ec8e421bf9` | 파일 식별용 |

사용자는 두 기존 weight가 COCO로 학습됐음을 확인했다(2026-10-06). 이번 단계에서 둘 다 보존한다.

두 weight는 2026-08-17 commit `e41cceefb7b079be74f51b0b74036989bee53502`에서 sensor-fusion
workspace와 함께 들어왔다. commit과 실행 문서에는 **왜 YOLO11m을 선택했는지, 어떤 MORAI
비교를 했는지 기록이 없다.** COCO weight와 Ultralytics 학습·export 도구를 바로 쓸 수 있고
정확도 여유가 있는 m 크기라 골랐을 가능성은 있지만 추정일 뿐이다. YOLO11이 MORAI에
특화됐다는 근거는 없다.

`yolo11m(1).pt`는 내부 구조와 class name이 `bus/car/person/truck`인 YOLO11s다. checkpoint에
남은 전체 4-class 지표는 `mAP50=0.7979`, `mAP50-95=0.5083`이지만, 학습 데이터·run split·
class별 pedestrian recall·독립 test lineage가 저장소에 없다. 이름만 보고 운영 weight로
바꾸지 말고 **출처를 복구할 때까지 진단 후보**로만 사용한다.

현재 실행·topic 확인 절차는
[`RUN_SENSOR_FUSION.md`](../../../src/lidar+sensorfusion/RUN_SENSOR_FUSION.md)를 따른다.

### 새 3-class weight는 아직 drop-in 교체가 아니다

현재 [`YOLODetection.py`](../../../src/lidar+sensorfusion/src/Main/src/Camera/YOLO/YOLODetection.py)는
`bus/truck/person/car`만 통과시킨다. downstream adapter도 `person/pedestrian`은 보행자로,
`car/truck/bus`는 차량으로 해석하지만 문자열 `vehicle`은 아직 일반 장애물로 처리한다.

따라서 새 checkpoint의 class name을 `person/vehicle/obstacle`로 정의하면 weight 경로만 바꿔서는
안 된다. 학습 결과를 runtime에 넣는 단계에서 filter와 canonical mapping을 함께 고쳐야 한다.
이는 후속 통합 작업이며 이번 문서 작업에서는 수정하지 않는다. 긴급 사용에는 기존 COCO
YOLO11m이 가장 안전한 이유다.

## 3. 어떤 모델을 먼저 볼 것인가

공식 640px COCO detection 표의 값이다. T4 TensorRT 수치는 모델 간 참고값이며 현재 MORAI PC,
ROS image decode, NMS와 fusion까지 포함한 end-to-end 지연이 아니다.

| 후보 | COCO mAP50–95 | Params | FLOPs | T4 TRT10 | 이 팀에서의 위치 |
|---|---:|---:|---:|---:|---|
| YOLO11n | 39.5 | 2.6M | 6.5B | 1.5 ms | 11s도 느릴 때 정확도 하한 탐색 |
| **YOLO11s** | **47.0** | **9.4M** | **21.6B** | **2.5 ms** | 첫 경량 대안 |
| **YOLO11m** | **51.5** | **20.1M** | **68.1B** | **4.7 ms** | 현재 즉시 사용 기준선 |
| YOLO26n | 40.9 | 2.4M | 5.5B | 1.7 ms | 26s 이후의 추가 경량 후보 |
| **YOLO26s** | **48.6** | **9.5M** | **20.9B** | **2.5 ms** | 호환성 확인 뒤 두 번째 대안 |

YOLO11m은 11s보다 공식 COCO mAP가 `+4.5`지만 parameter는 약 `2.14배`, FLOPs는 약
`3.15배`, 공식 T4 지연은 약 `1.88배`다. 그래서 m으로 정확도 기준선을 잡고 s가 보행자
recall을 유지하는지 확인하는 순서가 합리적이다.

YOLO26 공식 속도에는 fused·선택적 NMS-free 실행 조건이 포함된다. 현재 YOLO11 ROS 경로와
숫자만 직접 비교하지 않는다. `11s`와 `26s`는 같은 MORAI 이미지, 입력 크기, precision,
backend, confidence/NMS 조건에서 다시 재야 한다.

공식 출처:

- [Ultralytics YOLO11](https://docs.ultralytics.com/models/yolo11/)
- [Ultralytics YOLO26](https://docs.ultralytics.com/models/yolo26/)
- [COCO detection dataset](https://docs.ultralytics.com/datasets/detect/coco/)

## 4. class 계약

| 공통 그룹 | 원래 라벨 예 | NAS MORAI BBox class | P2a 사용 |
|---|---|---|---|
| `person` | person, bicycle, pedestrian | Pedestrian | person만 허용 |
| `vehicle` | car, truck, bus, motorcycle, train | Vehicle, Sedan, SUV, Truck, Bus, Van, Wagon, MPV | car/truck/bus만 허용 |
| `obstacle` | 그 밖의 검출 라벨 | Obstacle | 현재 허용 네 라벨에 없음 |
| 제외 | traffic light, stop sign | 별도 신호등 경로 | 공통 객체에서 제외 |

기존 COCO 80-class weight를 재학습 없이 쓸 때는 detector head를 바꾸지 않는다. 결과에서
`person`과 `car/bus/truck`만 사용하는 방식이다. 내부 80-class 연산은 그대로이므로 출력
class filter 자체가 추론량을 크게 줄이지는 않는다.

반대로 MORAI fine-tune에서 head를 1개에서 2~3개 class로 늘리는 비용은 backbone·neck·box
branch에 비해 매우 작다. **class 수를 하나 늘리는 비용**과 TwinLiteNet+에 **다중 scale
feature, box/class head, assignment, box loss, NMS를 전부 추가하는 비용**은 다른 문제다.

## 5. NAS export를 객체 학습에 쓰는 방법

### 5.1 위치와 필요한 묶음

업로드 원본은 `E:\asmc_capture_export_20260911`이고, 2026-09-11 사용자 확인 기준으로 팀 NAS
업로드가 완료됐다. NAS의 실제 share/mount 경로는 저장소에 기록돼 있지 않으므로 폴더명
`asmc_capture_export_20260911`로 찾고, 보이지 않으면 업로더에게 mount 경로만 확인한다.

| 목적 | 필요한 파일 | 대략적 크기 |
|---|---|---:|
| 기존 COCO YOLO 즉시 사용 | 없음 | 0 |
| 2D detector 최소 fine-tune | 루트 메타파일 + `meta.tar.xz` + `cam_rgb_front/` | 약 9.3 GiB |
| semantic/instance로 box 감사 | 위 묶음 + `cam_gt/` | 약 10.2 GiB |
| 카메라–LiDAR offline association | 위 묶음 + `lidar/` | 약 11.5 GiB |

루트 메타파일은 `README.md`, `MANIFEST.json`, `SHA256SUMS`다. `cam_extra.tar`의 73프레임은
짝이 불완전한 보충분이 섞여 있으므로 최초 detector dataset에는 넣지 않는다. 꼭 쓸 때만
`EXTRA_NOTES.json`의 `safe_to_overlay=true`인 프레임을 개별 검토한다.

일부 묶음만 받았을 때는 export 루트에서 받은 파일만 checksum 검증한다.

```bash
sha256sum --ignore-missing --strict -c SHA256SUMS
```

공간이 충분한 새 디렉터리에 metadata와 전방 RGB를 같은 위치로 푼다.

```bash
mkdir -p asmc_capture_20260911
tar -xJf meta.tar.xz -C asmc_capture_20260911
for archive in cam_rgb_front/*.tar; do
  tar -xf "$archive" -C asmc_capture_20260911
done
```

압축을 풀면 주요 파일은 다음처럼 조인된다.

```text
<run_id>/frames/intensity/front/<NNNNNN>.png
bbox/<run_id>/front/<NNNNNN>_BBox2D.txt
```

`run_id + 6자리 frame 번호`가 조인 키다. 번호가 중간에 비어 있으므로 `0..N-1` 연속
인덱스로 만들지 않는다. RGB 파일 목록을 기준으로 순회한다.

### 5.2 BBox2D를 YOLO label로 바꾸는 계약

`BBox2D` 한 줄은 다음 10필드다.

```text
class u1 v1 u2 v2 unique_id (r, g, b, a)
```

전방 RGB는 1280×720이다. 변환기는 다음을 지켜야 한다.

1. 좌표를 영상 경계에 clip하고 `x2 > x1`, `y2 > y1`을 검사한다.
2. 위 class 표에 따라 `Pedestrian → person`, 차량 8종 → `vehicle`로 병합한다.
3. `Obstacle`은 독립 `obstacle`로 매핑하고 `vehicle`로 합치지 않는다. 품질·범위는 별도 slice로 평가한다.
4. YOLO 형식은 `class_id x_center/W y_center/H width/W height/H`로 저장한다.
5. BBox 파일이 없더라도 RGB가 manifest에 정상 등록된 frame이면 유효한 negative sample로
   보고 빈 label을 만든다. archive 누락을 negative로 오인하지 않는다.
6. `source_class`, `unique_id`, 원래 `xyxy`, `run_id`, frame 번호는 별도 manifest에 보존한다.
7. split은 frame이 아니라 `run_id` 단위로 나누고 날씨·시간대가 한쪽에만 몰리지 않게 한다.

현재 export를 직접 감사한 전방 annotation 수는 다음과 같다.

| 대상 | annotation 수 | 비고 |
|---|---:|---|
| `Pedestrian` | 3,255 | `person` 양성 |
| 차량 8종 합 | 7,913 | `vehicle` 양성; 현재 export에는 `Van` 행이 없음 |
| `Obstacle` | 4,222 | 후속 obstacle 학습 후보 |
| 전방 RGB | 10,030 frames | 18개 primary run |

이 수치는 frame 수나 고유 객체 수가 아니라 **frame별 box 행 수**다. BBox 파일은 물체가 있는
frame에만 존재하므로 파일이 없는 정상 RGB를 누락 데이터로 버리지 않는다.

vanilla Ultralytics YOLO label은 ignore-region을 직접 표현하지 않는다. 첫 fine-tune에서는
`Obstacle`의 독립 그룹 학습을 평가하고, 해당 물체가 person/vehicle false positive를 만드는지
별도 slice로 측정한다. 문제가 실제로 확인될 때만 ignore-mask를 지원하는 loader를 후속
작업으로 만든다. 범위가 불명확한 `Obstacle`을 차량에 합쳐 문제를 숨기지 않는다.

### 5.3 이 데이터로 하면 안 되는 것

- Capture Mode 저장 중 시뮬레이터가 멈추므로 인접 frame을 실제 시간 연속 궤적으로 쓰지 않는다.
- detection 단일-frame 학습에는 쓸 수 있지만 tracking·odometry 학습은 `rosbags/`를 쓴다.
- 8-bit 상대 depth를 미터 depth로 해석하지 않는다.
- 같은 run의 인접 frame을 train과 val/test에 나눠 넣지 않는다.
- 전방 detector 첫 실험에 left/right/traffic-light RGB를 섞지 않는다.

## 6. 학습·비교 순서

2026-10-07 사용자 확인: AIM 데이터는 Windows D드라이브에 있다. 로컬에서
`D:\asmc_aim\dataset` (`/mnt/d/asmc_aim/dataset`) 디렉터리의 존재를 확인했다.
전체 파일 구성·이미지 GT 유무·RGB와 annotation 정합은 아직 감사하지 않았다.
아래 O0~O3는 이전 계획의 기록이며, 최신 실행 순서는 `camera-runtime-plan.md`를 따른다.
후속 학습은 YOLO26s/m COCO 사전학습 모델 평가 → NAS MORAI BBox2D 추가 학습 →
AIM 데이터 감사 및 검수 라벨 보강 순서로 계획한다. AIM에 이미지 GT가 없으면 자동
예측을 임시 라벨로 사용하고 누락·오검출을 수동 수정한다. 미라벨 프레임을 정상 음성
샘플로 간주하지 않으며, 독립 평가 세션은 학습·규칙 조정에서 분리한다.

### O0 — 학습 없이 바로 사용

- 현 `yolo11m.pt`와 현 launch를 사용한다.
- COCO의 `person/car/bus/truck`만 현재 fusion에 전달한다.
- 먼저 sunny/foggy, 근거리/원거리, 가림, 작은 보행자 false negative를 모은다.
- 기본 `conf=0.7`을 고정된 정답처럼 취급하지 말고 holdout에서 threshold sweep한다.

이 단계가 LiDAR 분류와 판단 연결을 진행하기 위한 긴급 경로다.

### O1 — 공식 YOLO11s와 무학습 비교

- 공식 COCO YOLO11s를 같은 입력·threshold sweep·backend로 비교한다.
- pedestrian recall이 유지되면서 지연과 VRAM이 줄 때만 m을 대체한다.
- 출처 불명 custom s는 비교표에 별도 행으로 두고 공식 s와 섞어 부르지 않는다.

### O2 — MORAI 3-class fine-tune

- 초기 weight는 O1에서 채택된 COCO `YOLO11s` 또는 정확도 여유가 필요하면 `YOLO11m`이다.
- NAS BBox2D로 `person/vehicle/obstacle` head를 fine-tune한다.
- vehicle 추가가 같은 holdout의 pedestrian recall을 낮추면 pedestrian-only 후보를 우선한다.
- 학습 command, epoch, batch와 augmentation은 GPU와 dataset manifest가 확정된 뒤 실험 기록에
  남긴다. 이 문서는 임의 hyperparameter를 정본으로 지정하지 않는다.

### O3 — YOLO26s/n 또는 공개 driving data 전이

- O0~O2가 정확도나 runtime을 만족하지 못할 때만 YOLO26s/n을 같은 조건으로 비교한다.
- COCO → BDD100K native detection → MORAI 3-class 전이는 단순 COCO → MORAI보다 이득이
  필요한 경우에만 진행한다.
- BDD의 rider/bicycle/motorcycle/train을 임의로 vehicle에 합치지 않고 native head에서
  사전학습한 feature만 MORAI 단계로 전이한다.

## 7. 평가표에 반드시 남길 값

같은 **미사용 run holdout**에서 아래를 모델별 한 행으로 남긴다.

| 범주 | 기록할 값 |
|---|---|
| 정확도 | person/vehicle/obstacle별 recall, precision, AP50, AP50–95 |
| 안전 slice | 작은·가린·원거리 pedestrian FN, sunny/foggy, 시간대별 FN/FP |
| threshold | confidence와 NMS IoU sweep, 채택 operating point |
| 실행 | preprocess, inference, decode/NMS 각각의 p50/p95 |
| 자원 | GPU utilization, peak VRAM, 출력 Hz, queue age/drop/stale |
| 계보 | weight SHA-256, upstream version, dataset manifest, run split, git commit |

우선순위는 `pedestrian recall → false positive 허용 수준 → latency/VRAM → vehicle AP`다.
COCO 전체 mAP나 평균값 하나만으로 승격하지 않는다.

## 8. 실제 카메라 배치와 통합 중단 조건

```text
front RGB ─┬─ TwinLiteNet+ lane/drivable ───────────┐
           └─ YOLO person/vehicle/obstacle ─→ LiDAR class│
left RGB  ─── TwinLiteNet+ shared weight ───────────┼→ 3-view BEV
right RGB ─── TwinLiteNet+ shared weight ───────────┘
TL RGB    ─── 현재 신호등 모델 ─────────────────────→ signal state
LiDAR     ─── OBB/track/unknown obstacle ───────────→ fusion/planning
```

- P2a 객체 detector는 전방만, 기존 20Hz 입력을 처리하며 단독 출력 18Hz 이상을 검증한다.
  YOLO 10Hz와 LiDAR 처리 10Hz는 후속 성능 단계에서 기존 최대 주기와 비교한다.
- 차선·주행가능영역은 전·좌·우에 같은 TwinLite weight를 적용한다.
- 신호등 모델은 성능이 확보됐으므로 전용 카메라에서 그대로 유지한다.
- 세 weight를 쓴다는 사실만으로 병목이라고 결론내리지 않는다. 실제 중복은 전방 RGB를
  TwinLite와 YOLO가 각각 encode하는 부분이고, 좌우와 신호등 카메라 forward는 통합해도 남는다.

3-view BEV + 전방 detector + 신호등 + LiDAR를 함께 켠 full-stack에서 기존 BEV gate인
`fused 9Hz 이상`, `sync 폐기율 5% 미만`, `camera-to-fused p95 130ms 이하`를 지키면 별도
detector 구성을 유지하고 통합 모델 연구를 중단한다.

게이트를 실패하면 다음 순서로 줄인다.

1. YOLO11m → 공식 YOLO11s/n
2. detector 실행 주기, FP16/TensorRT와 queue 정책
3. YOLO26s/n 또는 공개 unified model 비교
4. 마지막으로 TwinLiteNet+ custom detection head 개발

TwinLite custom head는 class channel 두 개를 붙이는 일이 아니다. multi-scale neck,
box/class head, assignment, loss, NMS, loader/evaluator와 multi-task loss 균형이 모두 필요하다.
full-stack 병목이 확인되기 전에는 이 비용을 지지 않는다.

## 9. 손재호에게 넘길 최소 체크리스트

- [ ] 현 YOLO11m SHA-256과 launch weight 경로 확인
- [ ] `RUN_SENSOR_FUSION.md`대로 `/YOLO/BBox`와 fusion debug 출력 확인
- [ ] sunny/foggy MORAI 표본에서 person/vehicle FN·FP와 현재 `conf=0.7` 기록
- [ ] 지연이 문제일 때만 공식 YOLO11s를 동일 조건으로 비교
- [ ] fine-tune이 필요하면 NAS에서 root metadata + `meta.tar.xz` + `cam_rgb_front/`만 먼저 수령
- [ ] checksum 확인 뒤 run-level train/val/test manifest 작성
- [ ] 3-class weight를 runtime에 넣기 전에 class filter/downstream mapping 변경을 별도 요청
- [ ] full-stack gate를 통과하면 unified/TwinLite detection-head 연구를 시작하지 않음

관련 장기 전략과 BDD100K 라벨 매핑은 카메라 파트의 조사 기록을 팀 레포에 복사해 중복
관리하지 않고, 필요 시 카메라 담당에게 요청한다. 이 문서는 객체 담당이 지금 결정하고 실행할
수 있는 범위만 정본으로 삼는다.
