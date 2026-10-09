# YOLO11m COCO 클래스 확대 실험

> **2026-10-09 통합 작업:** [3뷰 COCO7 실행·모델 선정](camera-three-view-coco-runtime.md)을 현재 시작점으로 사용합니다. 아래 기존 실험과 실패 증거는 보존합니다.

PR #72를 `a8c4f3b`에 병합한 뒤 별도 카메라 브랜치에서 비교한다. **운영 기본값은
네 클래스이며 전체 COCO 관측은 명시적인 실험 프로필**이다. 실행별 성공·실패와
원본 결과는 [실험 baseline](../../../config/baselines/perception_camera_coco_classes_v1.yaml)에 연결한다.

후속 허용 클래스 선정과 `person / animal / vehicle / obstacle` 후보, MORAI 출제 범위와
학습 라벨은 [객체 에셋·분류 설계](../competition/obstacle-assets-and-labels.md)에 모았다.
기존 세 그룹과 80종 실험은 유지한다. 후속 [seg 모델 선정](camera-seg-model-selection.md)은 별도 실험에서 animal 그룹을 추가하며 신규 학습은 수행하지 않는다.

## 직접 실행과 시각화

컨테이너·bridge·ROS master와 source 환경은 별도로 준비한다. 실시간 bridge와 같은
master를 사용하고 bag 이후의 master라면 `/use_sim_time`을 false로 돌린다.

```bash
roslaunch asmc_camera camera_objects_coco_experiment.launch classes:=all
```

```bash
rqt_image_view --clear-config /perception/camera/objects/debug/image/compressed
```

실험 launch는 **YOLO와 객체 overlay만 ACTIVE**로 실행한다. TwinLite·BEV·차선과
ResNet은 시작하지 않는다. rqt는 자동 실행하지 않는다. 먼저 실행한 launch를 `Ctrl+C`로
종료하고 `classes:=baseline`으로 바꾸면 기존 네 클래스를 같은 mask로 비교한다.

`debug:=false`는 overlay와 debug raw 발행을 끈다. `initial_mode:=0`은 로드·dummy 예열 후
STANDBY이며 기존 서비스로 PREPARE 준비 완료를 확인한 뒤 ACTIVE를 요청할 수 있다.
`target_hz:=20.0`이 기본이고 직접 실험에서는 launch 인자로 바꿀 수 있다. 이번 고정
비교에서는 목표 주기를 변경하지 않는다.

## 관측과 소비 경계

| 항목 | 기존 기본값 / baseline | all 실험 |
|---|---|---|
| 허용 원본 라벨 | bus, truck, person, car | COCO 80종 모두 |
| 그룹 설정 | 기존 object_classes.yaml | 별도 object_classes_coco_all.yaml, excluded 없음 |
| 신호등·정지표지판 | 제외 | 원본 라벨 보존, coarse obstacle 그룹 |
| 출력 타입·토픽 | CameraObjectBoxArray / 기존 objects·debug 토픽 | 동일 |
| LiDAR 호환 어댑터 | 네 클래스, confidence 0.7 | 동일, 새 클래스를 자동 전달하지 않음 |

기존 잠정 매핑인 person·bicycle→person, car·truck·bus·motorcycle·train→vehicle,
나머지→obstacle을 유지한다. `source_class_name`은 원래 라벨이다. **obstacle은 거친
관측 그룹이며 회피 대상 확정이나 자유공간/주행 승인이 아니다.** 신호등 검출은 camera4
ResNet의 색상 상태 판정을 대신하지 않는다. 구간 활성화·결과 소비는 판제 책임이다.

COCO는 정해진 80개 범주다. 콘·대회 나무 상자 등을 검출할 수 있다고 범위 확장만으로
보장하지 않는다. 전체영상 추론 결과를 후처리하는 crop/새 hood 필터를 섞지 않고 기존
semantic 윤곽 mask를 유지한다. 클래스 선택은 연산량 절감 실험도 아니다.

클래스 설정은 `src/asmc_camera/config/camera/yolo11m_coco_all.yaml`, 그룹은 같은 디렉터리의
`object_classes_coco_all.yaml`이다. 기존 YAML을 덮어쓰지 않는다. 개별 객체 launch는
`class_groups_config`, 상위 perception launch는 `objects_class_groups_config`로 그룹 파일을
선택한다. 빈 allowed_names의 기존 의미를 전체 클래스로 바꾸지 않았다.

## 고정 비교와 도구

`config/experiments/camera_coco_classes_v1.yaml`은 동일 weight SHA·confidence 0.7·IoU 0.7·
max_det 300·precision/입력 크기·전방 영상·mask를 고정한다. checkpoint names와 80종 목록이
일치하고 두 프로필이 클래스 선택 외에 같은지도 확인한다.

- 기존 `compare_camera_objects.py`로 mask OFF의 3,514프레임 정확 비교를 먼저 수행한다.
- `compare_camera_coco.py`는 같은 디코딩·masked 이미지로 두 backend를 번갈아 실행한다.
  카메라/주/음영 bag의 모든 결과와 기존 네 클래스 상대 순서·좌표·confidence 차이를 보존한다.
  추가 클래스와 max_det 선택의 영향을 이유 없이 동등성 예외로 처리하지 않는다.
- GT287과 가까운 차량 GT39를 고정해 일대일 IoU ≥0.5의 기존 person/vehicle 평가를 유지한다.
  새 클래스의 GT가 없으면 정량 정확도로 계산하지 않는다. 이미 승인된 mask 손실 1건과
  클래스 확대의 신규 손실은 별개다.
- 새 라벨별 최초·최고 confidence·시간 균등 3표본에서 최대 5프레임의 브라우저 갤러리를
  제공한다. 원본 영상 위 왼쪽 baseline/오른쪽 all 박스를 보여주며 판독 전에는 pending이다.
- `run_camera_coco_matrix.py`는 B0 관측/자원/종료 집계를 재사용하고 YOLO만 로드한다.
  camera/debug 각각 두 프로필×3회, fusion 두 bag×두 프로필×2회, 총 20회다.
  실패를 지우거나 유리한 회차만 선택하지 않는다.

고정 bag 측정은 MORAI·다른 GPU 추론이 없는 전용 network-none 환경과 master 11316에서
직렬 실행한다. shared 11311 bridge/master는 유지한다. 바깥 workspace를 source한 다음
nested Main/Msg 환경을 source한다. 자동 실행에는 GUI·차량 제어·UDP 드라이버를 포함하지
않고 Main의 legacy 제어 stub은 시험 토픽으로 격리한다.

카메라 44–86초에서 STANDBY 2초→해당 PREPARE 세대 준비 확인→ACTIVE 20초를 측정한다.
성능 기준은 새 관측 ≥18Hz·중복/역행 stamp 0이다. debug ON은 별도 비용 측정이며 GUI는 없다.
주·음영 fusion은 기존 LiDAR 기본 경로·50ms/200ms·네 클래스 어댑터를 유지한다.
PR #72의 opt-in 공통 경계/roll을 함께 켜지 않는다.

## 수락과 다음 작업

### 동일 입력 검증 — 2026-10-08

mask OFF 기존 경로는 3,514프레임 모두 정확히 일치했고 경계 예외도 없었다.
mask ON baseline/all의 카메라 3,514·주 979·음영 494프레임에서 기존 네 클래스의
좌표·confidence·상대 순서 차이는 0건이었다. 이는 저장된 같은 프레임의 비교이며,
실시간 처리율이나 fusion 도착 순서의 동등성을 대신하지 않는다.

GT287의 baseline/all 대응은 각각 195개, 가까운 차량 GT39는 각각 18개로,
이번 클래스 확대에 따른 신규 누락은 0개였다. GT287에는 person 386개·vehicle 540개,
GT39에는 vehicle 24개가 있으므로 **신규 누락 0은 기존 모델의 완전한 recall을 뜻하지 않는다.**
기존 mask 손실 예외를 확대 시험의 누락에 적용하지 않았다.

카메라 bag에서 추가된 박스는 traffic light 294·train 27·bench 4개다. 반복 프레임의
박스 개수이며 고유 객체 수나 정확도 점수가 아니다. 주 bag은 truck 2·car 330개,
음영 bag은 두 프로필 모두 허용 confidence에서 검출 박스가 없었다. COCO 80종으로
넓혀도 해당 음영 장애물을 해결하지 못했다.

[브라우저 비교 갤러리](../../../config/baselines/evidence/camera_coco_classes_v1/offline/gallery/index.html)는
새 라벨별 고정 표본 11프레임을 포함한다. 파일 탐색기에서 HTML을 열거나 저장소 루트에서
다음 명령을 실행한 뒤 `http://localhost:8768/index.html`로 확인한다.

```bash
python3 -m http.server 8768 --bind 127.0.0.1 \
  --directory config/baselines/evidence/camera_coco_classes_v1/offline/gallery
```

영상 검토에서 `camera-01645`의 건물/컨테이너 외벽이 `train [vehicle]`,
`camera-02056`의 가드레일이 `bench [obstacle]`로 검출된 오검출을 확인했다.
전체 새 클래스의 독립 GT 평가는 아니며 나머지 표본은 사용자 검토
대상이다. 원본 JSONL·차이 목록도 같은 evidence에 보존했다.

### 1배속 반복과 fusion 결과

RTX 4060 Ti 8GB, MORAI 종료·network-none·master 11316에서 YOLO만 로드했다.
GUI는 없고 debug ON은 별도 반복이다. 모든 20회와 원본 로그·관측·자원은
[matrix summary](../../../config/baselines/evidence/camera_coco_classes_v1/matrix/summary.json)와
[원본 archive](../../../config/baselines/evidence/camera_coco_classes_v1/matrix-raw.tar.gz)에 보존했다.

| 프로필 | debug | 3회 Hz | wall 지연 p95(ms) | GPU 사용률 p95(%) | 전체 VRAM 최대(MiB) |
|---|---|---|---|---|---|
| baseline | OFF | 19.84, 19.79, 19.90 | 25.8, 25.8, 25.9 | 40.0, 40.0, 40.0 | 2586.0 |
| baseline | ON | 19.89, 19.90, 19.90 | 23.3, 23.7, 23.9 | 41.0, 38.0, 42.0 | 2614.0 |
| all | OFF | 19.89, 19.90, 19.90 | 24.9, 26.2, 24.3 | 40.1, 40.0, 42.0 | 2658.0 |
| all | ON | 19.89, 19.89, 19.84 | 24.3, 24.2, 25.9 | 41.0, 39.0, 41.0 | 2597.0 |

전체 VRAM은 Windows/desktop까지 포함한 값이다. GPU 프로세스별 VRAM은 지원되지
않아 측정 불가로 남겼다. 노드별 CPU/RSS·source age p50/p95/p99·오류·프레임 대체는
각 report의 resources/diagnostics에서 확인한다. debug 비용의 반복 변동을 보존하며
서로 다른 반복의 작은 지연 차이를 성능 개선으로 단정하지 않는다.

| bag | baseline 매칭 | all 매칭 | offset p95 최대(ms) | 안정된 공통 객체 관측 | 신규 분류 차이 | 미대응 공통 관측 |
|---|---|---|---|---|---|---|
| primary | [646, 647] | [646, 646] | 34.5 | 7703 | 0 | 1908 |
| shadow | [207, 207] | [207, 207] | 24.8 | 1889 | 0 | 1163 |

주 bag의 기준선 반복 자체에서 분류 차이 1관측이 있었지만, 확대의 안정된 공통
관측 회귀는 0건이었다. 미대응 관측은 성공으로 계산하지 않았다. 모든 실행의 처리된
LiDAR stamp 집합은 bag별로 같았고 single `/YOLO/BBox` publisher도 확인했다.
주 bag은 baseline 최저 646쌍 대비 저하가 없지만 최고 647쌍과 비교하면 1쌍 적다.
이를 추적 개선·객체 recall 개선으로 해석하지 않으며, trace OFF 기록만으로 정확한
매칭 선택/timeout 원인을 확정하지 않는다. 기존 승인 분류 예외는 적용하지 않았다.

바깥 회귀 537개 중 알려진 facade 1개 실패, jamming 94개·Main 296개는 통과했다.
추가 범위의 Velodyne vendor 18개 실패는 필수 PCAP 미존재와 driver 종료/hz timeout
상태로 보존했다. vendor를 통과로 처리하거나 운영 코드를 수정하지 않았다. 초기 Msg
source 누락 및 실행기 fingerprint 경로 오류도 [실패 로그](../../../config/baselines/evidence/camera_coco_classes_v1/attempts/README.md)에 유지한다.

이번 PR은 기존 경로 회귀와 전체 비교·실패 원인 제출로 완료한다. 미달 구성은 운영
미수락으로 기록하며 80종을 기본값으로 승격하지 않는다. 기존 facade 실패와 이전 승인
분류 차이를 신규 손실에 자동 적용하지 않는다.

다음은 YOLO26 detect → seg의 boxes만 비교 → 같은 seg 결과의 masks 활용이다.
카메라 수·주기·동시 부하, AIM/MORAI 데이터 감사·학습, FP16·batch·공통 처리·TensorRT와
4070 SUPER 인수는 독립 검증한다. 재호의 구조물·군집·OBB·속도 인계는
[LiDAR 시작 문서](../../packages/asmc_lidar/README.md)를 따른다.
