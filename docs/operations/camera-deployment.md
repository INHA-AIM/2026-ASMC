# 카메라 객체 배포·소비

> 현재 실행·새 제어 경로는 [카메라 실행·3뷰 관측 계약](../contracts/camera-runtime-v1.md)이 정본입니다.
> 아래 기존 profile/서비스 설명과 과거 검증 결과는 호환·재현 기록으로 보존합니다.

현재 개발 기본 프로필은 **YOLO26l collect v5 E3, 15종, 전방960·좌우640, TensorRT FP16,
3뷰15Hz·2+1 batch, Ego mask ON, confidence0.40·동일 클래스 NMS0.50**이다.
사용자가 선택한 추가학습 3에포크 EMA를 사용한다. traffic_cone을 제외했고 stroller의
모델 ID는 14다. [교체·연결 검증 기록](../records/camera/2026-10-28-mission15-e3-deployment.md)을
따르며, 아래 과거 E8의 주행 성능 기록을 E3의 주행 검증으로 해석하지 않는다.

## 모델 선택·파일 배포

| 공개 `detector` 선택 | 가중치 | 지원 원본 이름 | 비고 |
|---|---|---|---|
| `e3` — 기본 (`collect_e3`) | collect v5의 완전한3epoch `epoch2.pt` EMA | traffic_cone을 제외한15종 | RTX5070에서 새로 만든 engine |
| `e8` (`finetuned_e8`) | v11의 완전한8epoch `epoch7.pt` | 이전16종 | 이전 모델 대조용; engine은 RTX4060Ti용 |
| `pretrained` (`pretrained_coco7`) | 공식 YOLO26l detection | person, dog, bicycle, car, motorcycle, bus, truck | checkpoint는 COCO80, runtime은7종만 출력; 신규 장애물을 지원한다고 표시하지 않음 |

선택은 실행 시 고정한다. 변경하려면 기존 detector를 종료한 뒤 재시작한다. 동시에 두 모델을
같은 출력 토픽에 띄우지 않는다. PREPARE 완료와 새로운 ACTIVE 세대 결과를 기다린다.
STANDBY는 모델을 해제하는 동작이 아니다. 살아 있는 worker의 경로 파라미터만 바꾸어
가중치가 변경됐다고 간주하지 않는다. 각 실행의 실제 profile·engine SHA는 diagnostics에 나온다.

사용자 요청으로 E3와 기존 모델의 checkpoint·engine 실제 배포 파일을 카메라 패키지에 포함한다.
[배포 파일·SHA 목록 v3](../../config/models/camera_object_models_v3.json)가 현재 배포의 정본이다.
E8은 원본 EMA를 그대로 유지한53.07MB 배포본이며 학습 재개 상태는 포함하지 않는다.
각 모델 YAML은 `$(find asmc_camera)/models/object_detection/`의 engine을 읽는다.
기존 [모델 Release](https://github.com/INHAautonav/2026-ASMC/releases/tag/camera-yolo26l-v11-e8-v1)와
[원본 목록 v1](../../config/models/camera_object_models_v1.json)은 과거 배포·학습 재개용으로 보존한다.
E3 engine은 RTX5070·TensorRT10.13.0.35 CUDA12에서 생성했다. 기존 E8/사전학습 engine은
RTX4060Ti용이다. 다른 GPU/런타임에 같은 engine을
무조건 적용하지 않으며, 제공한 checkpoint와 `prepare_coco7_models.py --head-profile`로 다시 생성한다.
E8은 one2one+외부 same-class NMS이고, 사전학습 복구 engine은 검증된 one2many 경로다.
checkpoint·가중치 라이선스는 Ultralytics AGPL-3.0이며 저장소 코드 라이선스와 구분한다.

지원 GPU·컨테이너에서 패키지를 pull·빌드하면 YOLO를 별도 다운로드하지 않아도 된다.
아래 명령은 **과거 release 재현·원본 checkpoint 확보용**이며 현재 실행의 필수 단계가 아니다.
RTX50 런타임 의존성은 `docker/ros-noetic/requirements-yolo-runtime-rtx50.txt`에 고정한다.

```bash
gh release download camera-yolo26l-v11-e8-v1 --repo INHAautonav/2026-ASMC --dir /tmp/asmc-camera-models-v1
python3 tools/perception/install_camera_model_release.py \
  --release-dir /tmp/asmc-camera-models-v1 --models-root "${ASMC_DATA:?ASMC_DATA를 지정하세요}/models"
```

`ASMC_DATA`는 원본 데이터를 보존할 호스트 루트다. 과거 프로필은 `ASMC_MODELS_ROOT`를
사용할 수 있지만 현재 canonical 프로필은 package-local 경로를 사용한다. 설치기는
기존 모델을 덮어쓰지 않는다. 원본 E8 checkpoint에만 재개 상태가 보존돼 있다.

## 직접 실행

현재 정본은 `camera.launch`·`view.launch`다. 최신 checkout을 빌드/source한 `asmc-ros-noetic`에서 실행한다.
카메라가 YOLO 단일 생산자이고 LiDAR는 공용 3뷰 관측을 소비한다. 루트 workspace 빌드와 연결은 [LiDAR 인계](../handoffs/camera-to-lidar.md)를 따른다.

```bash
roslaunch asmc_camera camera.launch mode:=active models:=objects
```

사전학습 모델은 기존 detector를 종료한 뒤 선택한다.

```bash
roslaunch asmc_camera camera.launch mode:=active models:=objects detector:=pretrained
```

시각화와 GUI는 각각 별도 한 세트다.

```bash
roslaunch asmc_camera view.launch models:=objects
rosrun rqt_image_view rqt_image_view --clear-config /perception/camera/objects/debug/mosaic/compressed
```

기본 `camera.launch`는 세 모델 상시 ACTIVE·발행이다. `models:=objects`도 객체 ACTIVE로 시작한다.
명시적인 `mode:=standby` 복구에는 `/perception/camera/objects/set_mode`의 mode1 PREPARE →
같은 세대 prepared=true 확인 → mode2 ACTIVE를 사용한다. [판제 소비 범위](../handoffs/perception-to-planning.md)는
생산자 발행 제한과 구별하며, Frenet 객체 입력을 구간별로 끊지 않는다.
TwinLite 기본은 Large v7 E8·15Hz 목표·TensorRT·전방544×960/좌우480×640이다.
[Large 배포 검증](../records/camera/2026-10-18-twinlite-large-deployment.md)의 동일 평가·정차 부하 결과와
전방 황색차선 회귀와 SUNNY freshness 실패를 확인한다. 사용자가 이 제한을 확인한 뒤 Large E8 기본 전환을 승인했다. Medium은 명시적 복구 설정으로 보존한다. 음영 lane 모드와 readiness는 [실행 계약](../contracts/camera-runtime-v1.md)을 따른다.
신호등 생산자는 카메라 담당의 `asmc_camera` 하나다. 판제 launch는 별도 ResNet을 시작하지 않는다.
중단·재개·주기는 카메라의 기존 `/perception/camera/traffic_light/set_mode`로 제어한다.

생산자 cutoff는 공개 `confidence:=0.40`으로 지정한다. 변경하려면 재시작한다.
소비자는 `box.confidence`로 추가 필터링할 수 있다. LiDAR 설정의 `fusion.camera_objects_conf_thres` 기본0.0은 생산자가 승인한 박스를 통과시킨다.
그 값을 낮춰도 생산자가 버린 박스를 되찾을 수 없다.

## Camera → LiDAR → 판제 계약

| `class_name` — 소비 그룹 | `source_class_name` — 원본 이름 |
|---|---|
| vehicle | car, bus, truck |
| dynamic_obstacle | person, dog, deer, bicycle, stroller |
| static_obstacle | motorcycle, bollard, box, concrete_barrier, steel_barricade, traffic_drum, plastic_barrier |

SportBike/Scooter는 대회 확인에 따라 static이다. 자전거·유모차는 이동하는 사람 에셋에
붙을 수 있어 dynamic이다. 유모차 안의 아기는 별도 person 정답으로 만들지 않는다.
그룹은 이동 가능성과 소비 목적이다. 실측 이동 상태·속도·충돌 위험을 그룹만으로 확정하지 않는다.
`source_class_name`은 모델의15종 이름(이전 프로필은16종/7종)이며 정확한 MORAI 에셋명을 알아냈다는 뜻이 아니다.
CargoBox/WoodBox는 box, Scooter/SportBike는 motorcycle, PE_Firewall은 plastic_barrier다.

- `/perception/camera/objects`: `asmc_msgs/CameraObjectBoxArray`, 원본 영상 bbox 좌표·크기·source header,
  view(`camera_front/left/right`), 원본 이름·그룹·confidence 보존. 메시지·서비스 MD5는 유지한다.
- `valid=true`·빈 boxes는 정상 빈 관측이다. `valid=false`, 오래된 stamp, 중단 입력은 새 빈 관측과 구분한다.
  카메라 bbox만으로 map 위치·거리·속도를 만들지 않는다.
- LiDAR는 공용 `CameraObjectObservation`의 원본 이름·그룹·confidence·식별자를 직접 수신한다.
  구 `/YOLO/BBox` 어댑터와 nested 메시지 workspace는 폐기했다.
- LiDAR 기본은 `fusion.camera_class_profile: mission15_no_cone_v1`, `fusion.views: all`이다.
  전방·좌측·우측을 각 보정과 시각으로 연결하고 scan마다 근거를 통합한다.
  unique·±50ms·200ms 제한과 원본 이미지 stamp를 유지하며 같은 근거를 중복 확인 횟수로 세지 않는다.
- `/perception/lidar/tracks_local/map`의 공용 관측은 `classification`에3그룹,
  `camera_class`에원본 이름을 보존한다. track의 실측 속도·기하·운동 판정은 별도다.
- 기존 `planning_inputs/DetectedObject` 숫자 enum은 vehicle/pedestrian/unknown이다.
  vehicle은vehicle, dynamic_obstacle+원본person은pedestrian, 나머지 장애물은unknown으로
  기하·속도를 보존한다. **unknown을 객체 없음 또는 정적으로 해석하면 안 된다.**
  최신 판제 입력은 `classification`·`camera_class` 문자열도 보존한다.
- 미등록 라벨은unknown/진단으로 남긴다. 클래스 이름만 보고 static으로 자동 배정하지 않는다.

과거 Main/wire 분류 정책은 삭제 전 고정 revision에서 재현한다. 현재 모델 대조는 카메라 프로필을 선택하고 LiDAR 공용 관측 계약을 따른다.
이는 위 `pretrained_coco7` 선택(동일3그룹 계약 유지)과 다르다.
이전 E8의 cone까지 전달하려면 LiDAR 설정 파일의 `fusion.camera_class_profile`을 `mission16_v1`로 함께 맞춘다.
시각화도 `view.launch detector:=e8`로 맞춘다. 기존 engine의 다른 GPU 호환성은 별도로 확인해야 한다.

## 과거 E8 검증 기록과 남은 과제

[고정 비교 근거](../../config/baselines/evidence/camera_mission16_deployment_v1/comparison.json)에서
E8/E22의16종 macroF1은74.21/71.19%,3그룹은74.96/70.38%다. 개발val205를 반복 사용했고
버스·트럭의 완전GT는14/12개로 적다. E8은 버스 F166.67%, 트럭58.33%다.
사전학습은 각각78.26/47.06%여서 버스 회귀를 숨기지 않는다.

[bag 근거](../../config/baselines/evidence/camera_mission16_deployment_v1/bag-summary.json):
E8/E22 비교8회, E8 TensorRT4회. TensorRT는 약14.8–15Hz, 입력수신→출력
p50 33–36ms·p95 41–45ms, 최대공백173.3ms, 전체VRAM peak1966/8188MiB,
중복·역행stamp0이다. 다른 두 모델은 로드된STANDBY였다.
PyTorch 비교의 일부250ms 초과 공백과 FP16 parity 차이도 보존했다.
독립 연속GT 없는 bag의 전체recall은 만들지 않았다. 이 표는 당시 전방1280/측방640·NMS0.70 조건의 결과이며
현재960/NMS0.50 전체 수락으로 바꾸지 않는다. 후속 전체 GUI·10분 시험과 남은 family 순간 공백은
[최신 family 공급 기록](../records/camera/camera-family-supply-optimization.md)을 따른다.

현재 다음 작업은 [대회 완료 로드맵](../architecture/consolidation-roadmap.md)을 따른다.
[10/14 수집·학습 제안](../records/camera/camera-deployment-roadmap-20261014.md)은 당시 후보 기록이다.
