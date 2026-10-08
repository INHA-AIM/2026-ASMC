# YOLO26·YOLOE seg 모델 선정

작업 브랜치는 `feat/camera-seg-model-selection`, 작업 폴더는 `asmc-camera-objects`다.
PR #73의 `62981e6` 위에서 진행하며 #73 병합 후 최신 main으로 옮긴다. 기존 검증
worktree와 shared 컨테이너 마운트는 유지한다. 학습·운영 모델/기본값 승격은 이번 범위에
없다. 결과와 실패는 `config/baselines/perception_camera_seg_selection_v1.yaml`로 모은다.

## 공식 소스와 실제 실행 구분

상위 workspace `external/baselines`의 원본 clone은 아래 revision으로 고정했다.

| 공식 참고 | 전체 SHA | LICENSE | 읽은 코드 |
|---|---|---|---|
| [THU-MIG/yoloe](https://github.com/THU-MIG/yoloe/tree/40cd606cabdbe2b566d6f14a6b162c89206e9a1b) | `40cd606cabdbe2b566d6f14a6b162c89206e9a1b` | AGPL-3.0 | text/visual/prompt-free prediction, `train_pe.py`, `train_pe_all.py`, `train_seg.py`, `export.py` |
| [AILab-CVC/YOLO-World](https://github.com/AILab-CVC/YOLO-World/tree/4f70adbaacf5685bd9ec5bea85f1f91057f6fc0b) | `4f70adbaacf5685bd9ec5bea85f1f91057f6fc0b` | GPL-3.0 | v2 pretrain/finetune config, `docs/reparameterize.md`, `docs/updates.md` |

YOLO-World v2는 동일 공식 저장소의 구성이다. v2.1 자료·성능과 혼용하지 않는다.
World는 이번 정량 후보가 아니다. 원본 두 repository의 설치·학습·export 실행은 하지
않았다. YOLOE-26의 실제 실행은 Ultralytics 8.4.150/torch 2.1.2+cu118에서 확인한다.
clone을 vendor/submodule로 추가하거나 shared Python 설치를 덮어쓰지 않는다.

공식 weight와 text encoder는 [Ultralytics assets v8.4.0](https://github.com/ultralytics/assets/releases/tag/v8.4.0)
에서 확보했다. 호스트 `data/models/camera_seg_v1`에는 weight·encoder·89개 prompt 임베딩과
출처 JSON을 둔다. checkpoint와 prompt SHA는 결과 manifest에 고정한다.
text tokenizer만 새 network-none 컨테이너의 `/opt/seg-deps`에 설치했다. 측정 중 다운로드나
prompt 변경이 없으며 GPU 추론 노드는 준비된 embedding을 읽는다.

## GT 준비와 정확도 해석

`prepare_camera_seg_gt.py`는 RGB·semantic·instance RGBA와 BBox2D actor ID를 연결한다.
원본 세 채널 SHA, BBox source/원본 xyxy, actor ID, visible instance bbox, mask를 보존한다.
객체 이름은 semantic에 확인되는 car/truck/bus/person까지만 확정한다. `Obstacle`·일부
`Vehicle`은 coarse GT로 남기며 나무 상자·동물이라고 임의 명명하지 않는다.

18개 실행을 seed 260108로 12/3/3 분할했다. 같은 실행의 모든 뷰가 같은 split이다.
전방·좌·우 총 30,090프레임 중 최종 사용 가능 15,259, 격리 14,831프레임이다.
초기 변환 v1의 29,404프레임 중 kcity 초기 4실행의 14,145프레임을 추가 격리했다.
이 실행은 다른 actor 색상(예: 255,0,0)을 사용해 현재 palette로 빈 GT를 확정할 수 없다.
v1 원본도 보존한다. train 10,533 / selection 2,252 / heldout 2,474프레임이 사용 가능하며,
12/3/3 실행 분할은 바꾸지 않았다. 선정용 row SHA가 v1/v2에서 동일함을 확인했다.
선정용 완전 연결은 2,252프레임이다. heldout은 inference·threshold 결정에 사용하지 않는다.
동물 GT는 없어 animal 정확도는 평가 불가다. BBox 누락만으로 빈 정답을 만들지 않으며,
actor semantic pixels가 있는데 BBox/mask 연결이 없으면 격리한다. BBox가 없어도 확인한
actor semantic pixels가 0인 영상은 actor 관측이 없는 프레임으로 명시적으로 다룬다.
이 검사는 물리 세계의 모든 장애물이 라벨링됐다는 증명이 아니다.

같은 영상과 semantic 보넷 윤곽 mask로 YOLO11m/26s-seg/E26s-seg를 각각 실행한다.
측방에 전방 mask를 복사하지 않는다. 원본 result 순서·float 좌표·confidence·instance
대응을 저장한다. 평가 bbox는 가시 mask의 bbox이며 원본 BBox2D도 비교용으로 남긴다.
작고 멀거나 가린 객체를 평가에서 삭제하지 않는다. GT가 없는 인프라 라벨 네 종류는
검출 결과에 보존하고 actor 정확도에는 넣지 않는다. 나머지 새로운 라벨의 정확한 종류와
오검출은 gallery 검토가 필요하다. 알고리즘 출력으로 정답을 만들지 않는다.

동일 그룹 일대일 IoU≥0.5에서 0.7 고정 대조와 0.10–0.70/0.05 sweep을 제출한다.
GT가 존재하는 그룹만 macro F1을 계산하며 동률에서는 높은 confidence를 선택한다.
mask IoU는 대응된 instance의 가시 mask IoU이며 미검출 객체를 지운 전체 정확도가 아니다.
세부 이름 정확도는 세부 GT가 확인된 객체만 계산한다. actor/run/view의 관측이 이어지는
구간의 matched→missed는 `analyze_camera_seg_accuracy.py`에서 연속 frame ID로 다시
확인한다. 원본 평가 요약의 초기 continuity 카운터는 관측 공백을 건널 수 있어
선정용 시간 안정성 지표로 사용하지 않는다. actor가 보이지 않는 구간을 임의 보간하지 않는다.
근거리·가림은 실제 거리 GT가 없어 large/small bbox·영상 경계·instance fill을 별도로
제출하고 gallery에서 검토한다. 이를 미터 단위 거리나 확인된 가림 정답으로 확대하지 않는다.

## 그룹·결과·좌표 계약

실험 그룹 파일 `object_classes_seg_experiment.yaml`만 네 그룹을 사용한다.
person→person, COCO 동물+deer→animal, bicycle/이륜차/car/truck/bus/train→vehicle,
나머지→obstacle이다. source_class_name은 원래 이름이다. 기존 세 그룹 설정의
bicycle→person과 제외 정책은 바꾸지 않는다. 그룹만으로 회피 대상·동적 여부를 결정하지 않는다.

기존 CameraObjectBoxArray·MD5·모드 서비스는 그대로다. 운영 토픽에는 box만 발행하며
instance mask는 평가 파일·debug compressed overlay로 제공한다.
3뷰는 실험 전용이며 기존 전방 runtime acceptance의 frame_id 요구를 3뷰로 변경하지 않는다. LiDAR 호환 어댑터는
camera_front, bus/truck/person/car, confidence≥0.7 정책을 유지한다. 전체 클래스를
관측해도 `/YOLO/BBox`로 새로운 클래스를 자동 전달하지 않는다.

실험에서 3뷰는 backend 하나와 뷰별 LatestFrameBuffer·독립 20Hz admission을 사용한다.
뷰 순서를 매 batch 회전해 공정하게 처리하고 원본 frame_id와 header를 유지한다.
예열·PREPARE/ACTIVE 세대와 오래된 결과 배제는 기존 제어를 사용한다. 동일 결과를
재발행해 처리율을 높이지 않는다. 운영 기본 YOLO launch는 전방 YOLO11m 그대로다.

## 고정 성능·fusion 프로토콜

`camera_seg_selection_v1.yaml`은 4060 Ti 8GB, MORAI 종료, network-none, master 11317,
세 모델 GPU 상주 조건을 고정한다. 정상 camera 비교 36회는 두 후보×전방/3뷰×
{객체 단독, TwinLite+객체, ResNet+객체}×3회다. 회차 순서는 순환한다. 단독은 다른 모델이
로드된 STANDBY 상주 성능이다. 해당 PREPARE 세대 준비 완료 후 ACTIVE를 요청한다.
GUI/debug/trace OFF에서 뷰별 ≥18Hz와 중복·역행 stamp 0을 검사하고, 동시 모델의 기존
10Hz 공급 기준도 따로 판정한다. 실패를 삭제하거나 목표 주기를 낮추지 않는다.

debug ON은 후보별 전방 단독 3회씩 별도 비교한다. 주·음영 bag은 YOLO11m 기준선과
두 후보를 각 2회 실행해 box fusion을 비교한다. Main의 기존 기본 경로, 50ms/200ms,
IoU/분류 정책을 유지한다. mask가 운영 fusion에 전달된다는 의미는 아니다. 분류·매칭
차이는 모델 교체의 관측 결과이며 예전 분류 차이 2건 승인을 재사용하지 않는다.

학습 후보는 전방 단독 처리율 수락을 먼저 확인하고 macro F1→mask IoU→p95 지연으로
선택한다. 3뷰나 동시 구성 미달은 명시해 사용할 수 있는 실행 조합을 한정한다.
두 후보 전방 단독 모두 미달이면 선정 보류다. 모델선정은 현재 미지원 장애물에 대한
추가학습 출발점 선정이며 대회 운영 승인과 다르다.

## 완료 결과와 수락 범위

4060 Ti 8GB에서 고정 54회 실행을 모두 완료했다. 개별 실행 판정은 50회 통과/4회
실패다. 전방·3뷰의 YOLO ≥18Hz와 중복·역행 source stamp 0은 모두 통과했지만,
TwinLite 동시 실행 3회와 ResNet 동시 실행 1회에서 기존 공급 기준이 미달했다.
실패 실행도 원본 archive에 포함하며 C4/C5 전체를 운영 승인으로 표시하지 않는다.

| 후보 | 선정 confidence | macro F1 | 대응 instance mask IoU | 전방 최소 Hz | 3뷰 뷰별 최소 Hz | 전방/3뷰 최대 p95 wall 지연 |
|---|---:|---:|---:|---:|---:|---:|
| YOLO26s-seg | 0.15 | 0.20194 | 0.82317 | 19.69 | 18.70 | 29.82 / 82.11ms |
| YOLOE-26s-seg | 0.10 | 0.21577 | 0.74383 | 19.60 | 18.34 | 30.28 / 85.57ms |

정해 둔 macro F1 우선 순위로 **YOLOE-26s-seg를 추가학습 후보**로 선정했다.
선정용 GT는 person 348, vehicle 2,710, obstacle 2,306개이며 animal은 없다.
YOLO11m의 최적 macro F1은 0.21678이므로 현재 점수만으로 기존 모델보다 전반적으로
우수하다고 주장하지 않는다. YOLOE의 obstacle recall은 0.12663이지만 precision은
0.04974로 오검출이 많다. 보완학습이 필요한 출발점이며 운영 모델은 YOLO11m이다.
고정 성능/fusion 측정은 confidence 0.7에서 수행했다. 선정 confidence 0.10의 처리율과
fusion은 아직 검증하지 않았으며 설정 기본값을 0.10으로 변경하지 않는다.

| bag / 후보 | YOLO11m matched 2회 | 후보 matched 2회 | 안정된 공통 관측의 분류 차이 | 판정 |
|---|---|---|---:|---|
| 주 / YOLO26 | 538, 535 | 536, 534 | 1 | 미수락 |
| 주 / YOLOE | 538, 535 | 532, 531 | 6 | 미수락 |
| 음영 / YOLO26 | 191, 190 | 191, 193 | 0 | 해당 비교 통과 |
| 음영 / YOLOE | 191, 190 | 192, 194 | 0 | 해당 비교 통과 |

offset p95는 모두 50ms 이내였고 단일 `/YOLO/BBox` 발행자를 확인했다. 주 bag의
매칭 감소와 신규 분류 차이를 이전 승인 예외로 면제하지 않는다. 이번 실험 완료와
새 모델의 운영 fusion 수락은 구분한다. mask OFF의 기존 경로 3,514프레임 비교는
모두 정확히 일치했고 경계 예외·오류는 0이다.

raw LiDAR와 mask 지원점 비교는 144개 고정 표본에서 YOLO26 2,387→1,925점,
YOLOE 2,127→1,749점이었다. raw 센서 장착 변환→base_link→전방 optical 투영이며
수평화 역변환을 적용하지 않는다. 최근접 raw scan ±50ms의 오프라인 비교이고
깊이 가림·도착 정책·자차 운동 보상·3D 객체 GT는 없다. 점 감소만으로 벽 제거 또는
실제 객체 보존 효과를 수락하지 않는다. 운영 fusion은 여전히 box 경로다.

로컬 전체 빌드, generated ROS 3뷰/모드/mask/header/실제 어댑터 시험과 계약 검사는
통과했다. outer catkin 288개 중 기존 facade 실패 1개는 `autonomy_no_bridges_v2.launch`의
신호등 기본 true/기대 false이며 신규 실패와 구분한다. nested Main 296개는 모두 통과했다.

### 갤러리·원본 증거와 재현

- [비교 갤러리](../../../config/baselines/evidence/camera_seg_selection_v1/gallery/index.html): 선정 split만 사용한 170프레임·49개 라벨, YOLO11m/26/E의 GT·box·mask 비교.
- [LiDAR box/mask 지원점 갤러리](../../../config/baselines/evidence/camera_seg_selection_v1/mask-support-gallery/index.html): 빨강은 box 지원점, 노랑은 mask 지원점.
- [판정 baseline](../../../config/baselines/perception_camera_seg_selection_v1.yaml)과 [54회 요약](../../../config/baselines/evidence/camera_seg_selection_v1/matrix-summary.json): 모든 성공·실패와 공통 객체 비교.
- [GT v2 요약](../../../config/baselines/evidence/camera_seg_selection_v1/gt-summary.json), [선정 row 동일성](../../../config/baselines/evidence/camera_seg_selection_v1/gt-selection-identity.json), [자료 inventory](../../../config/baselines/evidence/camera_seg_selection_v1/data-inventory-summary.json).
- [원본 manifest](../../../config/baselines/evidence/camera_seg_selection_v1/raw-manifest.json): 아래 archive와 로컬 원본의 SHA. weight/RGB 원본은 넣지 않는다.

브라우저에서 저장소 checkout의 `gallery/index.html`을 직접 열 수 있다. GitHub 파일 화면은
HTML 앱을 실행하지 않으므로 HTML과 상대 위치의 이미지를 함께 내려받아야 한다.
`selection-raw.tar.gz`는 GT v2·초기 v1 manifest·전체 mask·예측·원본 lineage를 포함한다.
`runtime-raw.tar.gz`는 54회 관측·리소스·log와 초기 smoke/실패를 포함한다.
`supplemental-raw.tar.gz`는 정확 비교 전체 프레임 결과·초기 자료 감사·공식 출처를 포함한다.
GT v2가 최종 정답 manifest이며 v1은 실패 이력이다. 초기 평가의 연결 GT SHA는 v1이지만
선정 row 동일성을 별도로 확인했으므로 선정 결과는 변하지 않았다. lineage의 ROS 요청
시각과 원본 simulator 시각을 센서 exposure 시각으로 간주하지 않는다.

모델 파일·prompt·설정 SHA 및 runtime `89bc3d9`는 archive manifest에 고정했다.
재현은 고정 이미지 `sha256:1ecd9f50491565cad32e32980bb5df5b48487f470440cb66c9ec3b8dc5c11400`,
새 outer/nested 빌드, `/opt/seg-deps` offline tokenizer, data read-only, network-none,
ROS master 11317에서 수행한다. shared master/bridge에서는 자동 측정기를 실행하지 않는다.
다음은 ROS 환경과 모델 자산을 준비한 전용 컨테이너 내부 명령이다.

```bash
python3 tools/perception/run_camera_seg_matrix.py \
  --revision 89bc3d9 --output /results/matrix-reproduction
python3 tools/perception/analyze_camera_seg_matrix.py /results/matrix-reproduction \
  --accuracy /results/accuracy
```

새 결과 폴더를 사용하며 `--limit` smoke 결과를 전체 선정 증거로 사용할 수 없다.
GT 변환/격리·threshold 재평가·gallery·지원점 도구는 `tools/perception/*seg*.py`에 모았다.

## 직접 launch와 GUI

컨테이너·bridge·ROS 환경 진입은 사용자가 직접 한다. 다른 객체 launch를 종료하고:

```bash
roslaunch asmc_camera camera_objects_seg_experiment.launch model:=yolo26
```

```bash
roslaunch asmc_camera camera_objects_seg_experiment.launch model:=yoloe views:=all
```

```bash
rqt_image_view --clear-config /perception/camera/objects/debug/instances/compressed
```

측방 창은 `/perception/camera/objects/debug/left/instances/compressed`,
`/perception/camera/objects/debug/right/instances/compressed`를 선택한다.
`initial_mode:=0`은 STANDBY, `debug:=false`는 mask overlay OFF다. 기본 ACTIVE·20Hz이며
TwinLite·ResNet을 자동 실행하지 않는다. 실시간에서는 master의 `/use_sim_time=false`를
확인하고 bag 측정은 별도 master의 true/clock 환경에서 수행한다.

## 다음 학습·AIM·BEV 인계

학습은 다음 PR에서 선정한 한 모델에만 수행한다. Capture의 검수된 actor mask와
학습 split을 먼저 사용하고 검수된 AIM 라벨이 확보되면 Capture 일부와 함께 보강한다.
다음 학습 전에는 coarse `Obstacle`/`Vehicle`의 실제 종류를 검수하고 클래스표를 고정한다.
격리된 초기 4실행의 palette/BBox 연결을 확인하기 전에는 학습에 넣지 않는다.
동물 GT가 없어 동물 성능은 보장하지 않으며 최종 heldout은 학습·threshold 선정에 사용하지 않는다.
AIM D드라이브 RGB 417,435개는 semantic/instance/2D BBox GT가 확인되지 않은 상태다.
확인되지 않은 3D geometry를 투영해 자동 정답으로 사용하지 않는다. 이번에는 자료와
부족 사례를 목록화하며 AIM의 camera4 rear와 Capture 신호등 camera4를 혼용하지 않는다.

TwinLite BEV는 현재 base_link 고정 지면 평면 투영이며 영상 시점 IMU 보정은 없다.
판제의 독립 시험에서는 영상 시점 도로 지면 평면으로 camera-to-ground 투영을 갱신하는
방법을 비교한다. 중력 기준 roll/pitch와 도로 상대 기울기는 구분하며 경사로·bank를
잘못 평탄화하지 않는다. 이미 만들어진 BEV를 단순 회전하는 방식으로 정확한 투영을
주장하지 않는다. 지연 보상용 짧은 자차 운동 보상도 별도 시험이다.
주행가능영역 mask는 객체 부재·충돌 없는 공간의 근거가 아니며 장애물·미관측과 결합한다.
차선 피팅 소비는 음영 ODD, 구간·주행 모드·결과 소비는 판제 책임으로 유지한다.

## 7종 미션 출력 비교와 학습 설계

사용자가 승인한 COCO 교집합은 `person, bicycle, car, motorcycle, bus, truck, dog`다.
person은 person, dog는 animal, 나머지 5종은 vehicle에 대응한다. 이 7종에는 일반
obstacle에 해당하는 클래스가 없다. deer·유모차·킥보드·상자·콘·드럼통·바리케이드·나무
등은 외형과 실제 보유 GT를 검수해 신규 세부 학습 라벨로 추가해야 한다. train/bench의
기존 오검출과 생활용품을 출력에서 제외하며 신호등/정지표지판은 별도 관측이다.
이 목록은 개별 이름이 미확인인 102개 F2 에셋 전체를 검수했다는 의미가 아니다.
[승인된 정책](../../../config/experiments/camera_mission_classes_v1.yaml)에 원래 COCO ID도 보존한다.

`classes:=mission`은 새 7종 출력 설정과 네 그룹 설정을 선택한다. 기본 `classes:=all`은
기존 실험이다. YOLOE의 checkpoint-bound 89-name embedding cache는 그대로이며
**7종 출력 허용 필터와 7개 prompt로 재구성한 모델은 다르다.** 다음 실제 비교에서
축소 prompt를 만들면 별도 SHA와 결과로 기록한다. class 수 축소만으로 backbone 연산
절감을 주장하지 않는다. 기존 model/target Hz/hood mask/source 좌표·메시지·어댑터는 유지한다.

```bash
roslaunch asmc_camera camera_objects_seg_experiment.launch model:=yolo26 classes:=mission
```

```bash
roslaunch asmc_camera camera_objects_seg_experiment.launch model:=yoloe classes:=mission
```

다른 객체 launch를 종료한 뒤 하나씩 실행한다. `views:=all`은 동일한 7종 출력의 3뷰
선택이며 GUI와 bridge는 별도다. instance overlay 토픽은 위 직접 launch 절과 같다.

### 저장 결과 재평가 — 새 추론 측정과 구분

[요약 baseline](../../../config/baselines/perception_camera_mission_classes_v1.yaml)과
[전체 threshold 결과](../../../config/baselines/evidence/camera_mission_classes_v1/summary.json)는
기존 선정용 2,252프레임의 저장된 box/mask를 필터링한 결과다. 새 GPU 실행·시간적 안정성·
7-prompt 모델·m 모델을 측정한 결과가 아니다. 전체 클래스 재계산이 기존 원본과 정확히
같음을 확인했고, 미지원 obstacle GT 2,306개도 FN으로 남겼다. 잘 검출되는 GT만 남겨
점수를 높이지 않는다. original bbox 순서·mask index·coords·confidence는 유지한다.

| 모델 | 0.7 원본→7종 macro F1 | 7종 sweep 최대 macro F1 / conf | 0.7 person TP | 0.7 vehicle TP / FP |
|---|---:|---:|---:|---:|
| YOLO11m 참고 | 0.13274→0.13251 | 0.22304 / 0.10 | 57 | 168 / 0 |
| YOLO26s-seg | 0.11432→0.11416 | 0.20197 / 0.15 | 51 | 123 / 0 |
| YOLOE-26s-seg | 0.11661→0.10716 | 0.19550 / 0.10 | 49 | 106 / 1 |

animal GT는 없어 평가 불가다. 7종은 obstacle TP도 0이다. YOLOE의 원본 추가 클래스
검출을 제외했으므로 이전 full-class 선정 순위와 목적이 달라진다. 이전 학습 후보 결정을
지우지 않고 **미션 범위에서 재검토**한다. 현재는 YOLO26s-seg를 학습 출발점으로 우선
검토하며 첫 검출 시점·공통 actor의 검출 공백·실제 런타임 검증 후 확정한다. 기존
threshold의 큰 FN과 선별되지 않은 세부 GT의 한계는 그대로다.

재현은 ROS/SciPy 환경의 CPU 명령이며 GPU나 shared ROS master를 사용하지 않는다.

```bash
python3 tools/perception/rescore_camera_mission_classes.py \
  --gt /results/gt-v2 --accuracy /results/accuracy \
  --policy config/experiments/camera_mission_classes_v1.yaml \
  --output /results/mission-classes-reproduction
```

### 모델 비교와 학습 순서

| 경로 | 이 시스템에서의 역할 | 확정 전 확인 |
|---|---|---|
| YOLO26s-seg 추가학습 | 우선 후보. Capture instance GT와 mask 기반 fusion 연구를 함께 활용 | 실제 상자 등 신규 종류 GT, first-detection/gap, 선택 threshold의 동시 부하 |
| YOLO26s bbox 추가학습 | mask의 fusion 보완 이득이 부족할 때 단순한 대안 | mask를 사용하지 않는다는 이유로 bbox 검출 품질 우위를 가정하지 않음 |
| YOLO26m-seg 추가학습 | s의 작은 객체·조기 검출 한계가 확인될 때 승격 후보 | 전방/3뷰·동시 모델의 18Hz 및 latency/VRAM |
| YOLO26m bbox 추가학습 | 큰 backbone을 쓰면서 mask branch 비용을 피하는 대안 | bbox m도 s-seg보다 가볍다고 가정하지 않음 |
| YOLOE-26s/m prompt + 추가학습 | 라벨이 부족한 신규 종류의 초안 생성·미학습 범주 관측 후보 | prompt 미검출/오검출, 실제 대회 클래스 PE fine-tuning, runtime 비용 |

여섯 경로를 모두 학습하지 않는다. 먼저 같은 7종에서 기존 s 결과와 일반 YOLO26의
s-bbox/m-bbox/m-seg를 짧은 대표 구간으로 추론 비교한다. 필요가 입증된 경우만 E-m을
추가한다. 대회 핵심 지표는 객체 최초 검출·누락/오검출·최대 공백·fusion 객체 보존이며
macro F1은 보조로 함께 본다. 학습은 선정한 한 모델만 시작하고 m 승격은 실패 근거가
있을 때 비교한다. mask의 실제 fusion 이득이 없으면 bbox 경로를 사용할 수 있다.
일반 YOLO26도 custom class로 fine-tune할 수 있어 상자 추가학습에 YOLOE가 필수는 아니다.
[일반 seg 학습](https://docs.ultralytics.com/tasks/segment/#train),
[YOLOE PE 학습](https://docs.ultralytics.com/models/yoloe/#train-usage)을 참고하되 실제 고정
8.4.150의 trainer/head 이전과 출력은 학습 실행 전에 확인한다.

### 추가 수집 없이 보유 데이터 활용

1. **Capture GT를 먼저 고정한다.** 사용할 actor mask/box를 직접 연결하고 coarse
   Obstacle/Vehicle의 세부 종류를 검수한다. 가시 mask와 bbox 기준을 통일하고 원본
   actor ID·세부 라벨·학습 ID 대응을 보존한다. 초기 4run palette 미확인 자료는 격리한다.
   split 12/3/3을 유지하며 학습·selection·heldout의 역할을 바꾸지 않는다.
2. **일반/AIM RGB는 학습용 세션만 먼저 선별한다.** D드라이브 417,435장을 일괄 라벨링하기
   전에 세션·카메라·조도·음영·객체별 표본을 고르고 근접 중복을 줄인다. 검증 세션은
   teacher threshold 조정이나 student 학습에 섞지 않는다. camera4 rear는 Capture TLC와
   혼용하지 않는다. 배경·기하를 확인한 원본을 보존한다.
3. **YOLO26x로 기존 7종 라벨 초안을 만든다.** bbox 학습이면 x detect의 box를 사용하고,
   seg 학습이면 x-seg의 box+instance mask를 우선 사용한다. x detect+SAM은 별도 대안이다.
   큰 teacher는 오프라인이므로 실시간 모델과 같은 latency 제한을 적용하지 않지만
   4060 Ti 8GB의 해상도/메모리는 소규모 시험으로 확인한다. model·버전·SHA·conf·
   해상도·원본 결과·검수 이력도 기록한다.
4. **자동 라벨은 GT와 분리해 검수한다.** teacher가 놓친 대상·누락된 mask·여러 사람이
   하나로 합쳐진 결과·신규 상자/콘 등을 확인한다. 선택한 학습 클래스의 보이는 대상을
   모두 주석 처리하지 못한 영상은 완전 라벨 영상으로 넣지 않는다. 무검출을 빈 GT로
   확정하지 않고 validation/heldout 정답은 독립 검수한다. [공식 자동 주석 도구](https://docs.ultralytics.com/reference/data/annotator/)
   역시 detector box를 사용하므로 미검출 객체를 자동 복구하는 보장이 아니다.
5. **미지원 종류는 Capture에서 먼저 학습한 teacher로 보강한다.** COCO x/x-seg는 deer·
   wooden crate·cone 등의 새 라벨을 반환할 수 없다. Capture로 선정 모델을 학습한 뒤
   그 모델 또는 검수한 YOLOE prompt 초안을 사용해 기존 일반/AIM 영상의 신규 종류를
   보강한다. segmentation 경계만 만드는 SAM은 종류 정답을 대신하지 않는다.
6. **검수한 자료를 혼합해 한 모델을 추가학습한다.** Capture GT와 검수된 실제 영상의
   출처/검수 수준을 유지하며 기존 car/person과 신규 종류를 함께 평가한다. 독립 test의
   에셋별 누락·거리/가림/음영·시간 연속성과 실제 GPU/fusion을 확인한 뒤 운영 적용한다.

YOLO26x 자동 라벨링과 실제 fine-tuning은 이번 클래스 선택 작업에서 실행하지 않았다.
자료 수집 없이 시작할 수 있지만 보유 영상에 없는 에셋의 인식 능력을 학습 완료로
주장하지 않는다. 선택한 7종과 후속 학습의 신규 클래스 목록은 서로 다른 정책 버전이다.
