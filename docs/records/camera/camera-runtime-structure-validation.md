# 카메라 실행 구조·계약·후처리 검증 — 2026-10-14–15

현재 기본값과 후속 검증은 [15Hz·confidence 후속 기록](camera-rate-confidence-followup.md)을 따른다.
아래 E347의 결정·수치는 기본 NMS0.70/TwinLite10Hz였던 변경 전 기록으로 보존한다.

실행 정본은 [카메라 실행·3뷰 관측 계약](../../contracts/camera-runtime-v1.md)이다.
이번 변경은 카메라 생산자와 공용 메시지에 한정한다. 기존 E8/사전학습 가중치와
원본16종→세 그룹·3뷰 mask를 유지한다. 학습·수집·판제 정책·asmc_lidar 연결은 변경하지 않는다.

## 결정

- **R1 독립 프로세스 유지.** R2의 공유 JPEG decode는 작동하지만 CPU와 지연의 채택 기준을 충족하지 못했다.
- **YOLO15 / TwinLite10 / ResNet10Hz 유지.** TwinLite15의 SUNNY 두 번째 반복에서 한 완전한5초 창이14.2Hz로 목표95%인14.25Hz에 미달했다. 최소9.5Hz와 목표15Hz를 구분한다.
- **공개 기본은 confidence0.40·동일 클래스 NMS0.70 유지.** E8의 검증된 대조는 `nms_iou:=0.50`이다. 교차 클래스 억제는 작은 그룹 점수 이득과 이름 점수 감소 때문에 제외했다. 전체 그룹 NMS를 사용하지 않는다.
- **전체 실시간 공급 수락은 보류.** 최종 기본 구성의 SUNNY-r3·FOGGY-r1에서 입력/header와 출력의 동시 공백이 발생했다. 모델 callback→발행 지연 통과만으로 이 실패를 면제하지 않는다.

## 같은 영상·실행 구조 비교

동일 bag의 네 카메라 JPEG를 독립 master에서 재생했다. 같은 이미지가 충분히 처리되도록
입력만 네 tick 동안 유지했으며 생산자가 결과를 재발행하지는 않는다. ROS seq는 publisher 순번이므로
원본 이미지 대응은 실제 재생 stamp→이미지 인덱스로 확인했다.

| 항목 | R0 기존 main | R1 독립 프로세스 정리 | R2 공유 입력·독립 worker |
|---|---:|---:|---:|
| 전방 YOLO p50 / p95 / p99 ms | 42.53 / 53.96 / 56.67 | 42.21 / 53.97 / 56.70 | 81.43 / 115.16 / 125.71 |
| 전방 차선 p95 ms | 54.03 | 54.61 | 158.30 |
| 신호등 p95 ms | 53.96 | 54.94 | 82.05 |
| 카메라 CPU 평균, 100%=1core | 226.39% | 230.25% | 245.59% |
| 합산 RSS peak MiB | 3120.87 | 3125.54 | 1970.15 |
| 전체 GPU VRAM peak MiB | 4955 | 4962 | 4652 |

R2는 R1 대비 CPU가6.7% 증가하고 전방 YOLO p95가113% 증가했다. 메모리 감소만으로 채택하지 않는다.
R2의 최종 확인에서는 decode1161회·cache hit513회·busy/error0, 각 뷰 최대2슬롯이었다.
동일 영상에서 **bbox180/180·차선180/180·신호등60/60**이 R0와 R1/R2 모두 일치했다.
신호등 confidence 차이도0이었다. 입력 세대 경계 보완 후 같은 비교를 다시 통과했다.

공유 CUDA stream/context 및 device-wide synchronize에 의한 간섭은 근거 있는 가설이다.
이 단계에서 원인을 단정하거나 미검증 stream/H2D 변경을 기본에 넣지 않았다.

## 실시간 반복과 지연

MORAI SUNNY/FOGGY에서 기준선과 R1을 각3회·60초 교차 측정했다. 모델별 ACTIVE 조합,
TwinLite15Hz 6회, legacy Main 부하, 실제 rqt 및 시각화 경로를 별도로 측정했다.
모델은 모두 로드한 상태에서 활성 조합만 바꿨고 공개 차량 제어를 발행하지 않았다.
Main 부하 시험의 기존 제어 stub은 `/b0/discarded_ctrl_cmd*`로 격리했다. 이는 asmc_lidar 이관 수락이 아니다.

최종 기본 구성의 YOLO raw-frame 통합 지연:

| 날씨 | 뷰 | p50 ms | p95 ms | p99 ms |
|---|---|---:|---:|---:|
| SUNNY | 전방 | 35.74 | 53.33 | 62.61 |
| SUNNY | 좌측 | 35.81 | 53.40 | 64.38 |
| SUNNY | 우측 | 35.07 | 52.63 | 62.34 |
| FOGGY | 전방 | 37.70 | 55.50 | 66.13 |
| FOGGY | 좌측 | 38.43 | 56.86 | 67.70 |
| FOGGY | 우측 | 39.07 | 57.03 | 68.39 |

p50/p95/p99는 모델 또는 shared broker의 실제 callback 진입→ROS publish 반환까지의
monotonic wall 분포다. 센서 촬영→발행 지연으로 설명하지 않는다. 반복별 중앙값을 평균내지 않고
원시 프레임을 합쳤다. 처리율은 새 stamp만 세고 완전한5초 창을 사용했다.
예측 단계의 `inference_ms`는 predict 전처리·GPU·후처리를 포함한 wall 값이며 순수 GPU kernel 시간이 아니다.
side batch의 같은 시간값을 두 뷰에서 합산하지 않는다.

지연은 기준 안이지만 최종 두 회차에 입력 공백이 있었다. SUNNY-r3의 camera4 입력 최대0.764초,
좌우 입력 약0.718초, FOGGY-r1의 세 뷰 입력 약0.87초였다. 서로 다른 모델 프로세스에서
동일 source header와 callback 공백을 확인했다. MORAI 렌더링 또는 bridge 중 어느 원인인지는
단정하지 않는다. 원래 실패와 센서-only/YOLO-only 대조는 모두 보존한다.

같은 날씨에서 센서·bridge만 120초씩 실행한 대조의 최대 입력 공백은 SUNNY
143.4ms·FOGGY166.6ms였다. YOLO만 ACTIVE, 나머지 모델은 로드된 STANDBY인
60초 대조에서도 최대 공백은155–186ms로 긴 중단이 재현되지 않았다.
세 모델 동시 실행과의 관계를 추가로 확인해야 하며, 이 대조가 원래 두 실패를 면제하지는 않는다.

GPU 수치는 MORAI와 배경 프로그램을 포함한 전체 VRAM이다. CPU·system RAM은 Linux guest 범위이며
Windows 프로그램의 CPU/RAM을 모두 계측한 것으로 설명하지 않는다. 합산 RSS에는 공유 페이지가 포함될 수 있다.
문서·브라우저 등 사용자 작업은 종료하지 않았다.

시각화 비용 대조는 TwinLite15Hz에서 별도로 수행했다. GUI OFF 여섯 회차의 카메라 CPU는
293.37–296.18%, 합산 RSS peak는3115–3134MiB였다. 실제 rqt와 현재 overlay·합성 경로를
활성화한 한 회차는 CPU433.56%·RSS3892.55MiB였다. 배경 부하가 다른 단일 대조이므로
정밀한 인과 효과로 해석하지 않지만 시각화 비용이 작다고 가정하지 않는다.
일반 camera.launch는 시각화 노드를 시작하지 않으며 raw debug 영상 발행도 끈다.

## 후처리

모든 조합은 E8 one2one 출력·같은 GT/ignore/mask를 사용했다. confidence13개×동일 클래스 IoU4개×
교차 클래스 옵션2개, 총104개를 selection에서 비교했다. 최종 bbox clipping은 IoU를 바꿀 수 있으므로
원본 영상 좌표와 실제 packed tensor 좌표를 함께 저장해 실행과 같은 단계에서 NMS를 계산했다.

| 자료·설정 | 3그룹 macro F1 | 16종 macro F1 | 중복 FP | 이름 FP | 배경·위치 FP |
|---|---:|---:|---:|---:|---:|
| selection205 · .40/.70 | 70.08% | 72.97% | 20 | 22 | 81 |
| selection205 · .40/.50, 교차 억제OFF | 72.33% | 74.69% | 1 | 22 | 77 |
| heldout116 · .40/.70 | 63.05% | 66.25% | 17 | 38 | 108 |
| heldout116 · .40/.50 | 64.68% | 67.56% | 1 | 36 | 106 |

고정 대조에서는 selection/heldout 모두 정상 GT 대응 집합의 손실·추가가0이었다.
완전 GT만 precision/F1을 제공하고 partial은 recall로 분리한다. 평가 범위와 소표본·소스 편중은
JSON에 보존했다. selection205는 반복 사용한 개발 평가본이며 heldout116도 새로운 미공개 시험으로
설명하지 않는다. 이전 PyTorch 동적 입력의 unknown 뷰와 실제 runtime packing이 달라 과거 점수와
직접 우열을 비교하지 않는다. 뷰 미확인 영상에는 Ego mask를 추정하지 않았다.

```bash
# 기본을 유지한 채 E8 대조를 직접 선택
roslaunch asmc_camera camera.launch mode:=active models:=objects nms_iou:=0.50
```

## 검증·재현

기존 bbox·모드 메시지와 MD5를 보존했다. 새 observation의 session/generation/calibration/model/config ID와
alias의 동일 상태를 검사했다. 입력 프레임에 수신 세대를 붙이고 begin 시 같은 잠금 안에서 검사해
stamp 역행 시 이전 pending frame이 새 세대로 다시 채택되는 경계를 막았다. 신호등도 입력 callback에서
세대를 관측해 오래된 staged frame을 차단한다.

- 카메라 CPU 회귀313개, 별도 runtime49개, metric4개, 계약35개 및 generated ROS 경계 시험.
- 최종 입력 세대 경계 코드로 독립 ROS master에서 모드·alias·관측 metadata·입력 중단11항목을 재확인했다.
- 실제 catkin build, launch 해석, 읽기 전용 cache·mask 격리·2슬롯 상한·고정 class ID.
- PREPARE/ACTIVE/STANDBY, 정상 빈 결과·decode 오류·입력 중단·frozen clock·오래된 세대·fatal CUDA 모의 시험.
- Hosted CI는 PR의 실제 결과를 별도로 기록한다. 로컬 통과로 대체하지 않는다.

PR #85의 첫 hosted CI는 새 메시지를 공용 등록표에 빠뜨린 오류로 계약 검사에서 실패했다.
등록표와 고정 메시지 수·MD5 회귀 시험을 보완했고 로컬35개 계약 시험을 통과했다.
수정 후 hosted CI 결과는 PR의 최신 checks와 로컬 `hosted-camera-ci*.log`에서 확인한다.
두 번째 CI의 RF 호환 시험을 계기로 실제 launch의 backend 선택도 확인했다. 기본값을
`param`으로 지정하면 명시적 RF YAML을 덮는 문제가 있어 `rosparam` 기본값으로 수정했다.
기본 YOLO와 명시적 RF 선택을 실제 launch 해석으로 확인했다. E8 기본 구성의 처리 설정은 같다.

원본 로그·raw-frame 표·이미지 갤러리는 workspace `artifacts/camera-runtime-structure-20261014-r1/`와
`data/runtime/camera-structure-v1/results/`에 있다. E347을 기존 ASMC 실험 목록에서 찾을 수 있다.
Git에는 요약 CSV/JSON·설정·도구만 보존하고 이미지와 큰 원시 기록은 올리지 않는다.
실험 도구는 `compare_camera_runtime.py`, `run_camera_structure_matrix.py`,
`cache_camera_runtime_predictions.py`, `compare_camera_postprocessing.py`,
`summarize_camera_structure.py`, `build_camera_runtime_gallery.py`다. 일반 사용자는 shell wrapper 없이
`camera.launch`·`view.launch`와 별도 rqt 명령을 사용한다.

현재 로컬 실행에서는 기존 마운트를 바꾸지 않고 아래 overlay를 source한다.

```bash
# asmc-ros-noetic 터미널
source /data/runtime/camera-structure-v1/devel/setup.bash
roslaunch asmc_camera camera.launch mode:=active models:=objects
# 별도 터미널에서 같은 overlay를 source한 뒤
roslaunch asmc_camera view.launch models:=objects
rosrun rqt_image_view rqt_image_view --clear-config /perception/camera/objects/debug/mosaic/compressed
```

팀원은 이 PR의 workspace를 빌드·source하고 같은 launch를 사용한다. 운영 모델 교체는 없다.
기존 작업은 로컬 `work/camera-training-followup`의 `f4db7c50`에 보존했다. 이번 브랜치는 main
`fcfe8c587f09f27a3f6aa25e500f97299dd71e15`에서 시작했다. 보존 커밋 전체는 PR에 포함하지 않았으며
이번 평가에 필요한 supervision-aware 집계와 기존 갤러리 표시 보존 부분만 가져왔다.

다음 작업은 상류 입력 공백 확인과 asmc_lidar의 카메라별 보정·버퍼·투영·관측 통합 연결이다.
3뷰/mask fusion 또는 판제 주행 정책은 이번 완료 항목에 넣지 않는다.
