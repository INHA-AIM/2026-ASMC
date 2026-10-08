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
