# 카메라 객체 배포·소비 계약 — 2026-10-15

> 현재 실행·새 제어 경로는 [카메라 실행·3뷰 관측 계약](../../contracts/perception/camera-runtime-v1.md)이 정본입니다.
> 아래 기존 profile/서비스 설명과 과거 검증 결과는 호환·재현 기록으로 보존합니다.

현재 PR의 개발 프로필은 **YOLO26l v11 E8, 16종, 전방960·좌우640, TensorRT FP16,
3뷰15Hz·2+1 batch, Ego mask ON, confidence0.40·동일 클래스 NMS0.50**이다.
이전 승인된1280·NMS0.70의 검증 기록과 새 입력의 수락은 구분한다.
사용자가 실시간 육안 시험 후 개발 기본 적용을 승인했다. SUNNY/FOGGY·11/13/15시 전체 조건,
터널·범퍼 앞 근거리의 안정적 인지는 아직 충족하지 못했다. 이 배포를 전 구간 성능 수락으로
해석하지 않는다. 학습은 종료했고 E22 대신 E8을 사용한다.

## 모델 선택·파일 배포

| `objects_model_profile` | 가중치 | 지원 원본 이름 | 비고 |
|---|---|---|---|
| `finetuned_e8` — 기본 | v11의 완전한8epoch `epoch7.pt` | 합의한16종 | 신규 장애물 지원, 버스 회귀·터널/근거리 누락 남음 |
| `pretrained_coco7` | 공식 YOLO26l detection | person, dog, bicycle, car, motorcycle, bus, truck | checkpoint는 COCO80, runtime은7종만 출력; 신규 장애물을 지원한다고 표시하지 않음 |

선택은 실행 시 고정한다. 변경하려면 기존 detector를 종료한 뒤 재시작한다. 동시에 두 모델을
같은 출력 토픽에 띄우지 않는다. PREPARE 완료와 새로운 ACTIVE 세대 결과를 기다린다.
STANDBY는 모델을 해제하는 동작이 아니다. 살아 있는 worker의 경로 파라미터만 바꾸어
가중치가 변경됐다고 간주하지 않는다. 각 실행의 실제 profile·engine SHA는 diagnostics에 나온다.

사용자 요청으로 checkpoint2개·engine2개의 실제 배포 파일을 카메라 패키지에 포함한다.
[배포 파일·SHA 목록 v2](../../../config/models/camera_object_models_v2.json)가 패키지 배포의 정본이다.
E8은 원본 EMA를 그대로 유지한53.07MB 배포본이며 학습 재개 상태는 포함하지 않는다.
두 canonical YAML은 `$(find asmc_camera)/models/object_detection/`의 engine을 읽는다.
기존 [모델 Release](https://github.com/INHAautonav/2026-ASMC/releases/tag/camera-yolo26l-v11-e8-v1)와
[원본 목록 v1](../../../config/models/camera_object_models_v1.json)은 과거 배포·학습 재개용으로 보존한다.
TensorRT engine은 RTX4060Ti·TensorRT10.13.0.35에서 검증했다. 다른 GPU/런타임에 같은 engine을
무조건 적용하지 않으며, 제공한 checkpoint와 `prepare_coco7_models.py --head-profile`로 다시 생성한다.
E8은 one2one+외부 same-class NMS이고, 사전학습 복구 engine은 검증된 one2many 경로다.
checkpoint·가중치 라이선스는 Ultralytics AGPL-3.0이며 저장소 코드 라이선스와 구분한다.

지원 GPU·컨테이너에서 패키지를 pull·빌드하면 YOLO를 별도 다운로드하지 않아도 된다.
아래 명령은 **과거 release 재현·원본 checkpoint 확보용**이며 현재 실행의 필수 단계가 아니다.
공유 컨테이너의 pip 패키지는 변경하지 않는다.

```bash
gh release download camera-yolo26l-v11-e8-v1 --repo INHAautonav/2026-ASMC --dir /tmp/asmc-camera-models-v1
python3 tools/perception/install_camera_model_release.py \
  --release-dir /tmp/asmc-camera-models-v1 --models-root "${ASMC_DATA:?ASMC_DATA를 지정하세요}/models"
```

`ASMC_DATA`는 원본 데이터를 보존할 호스트 루트다. 과거 프로필은 `ASMC_MODELS_ROOT`를
사용할 수 있지만 현재 두 canonical 프로필은 package-local 경로를 사용한다. 설치기는
기존 모델을 덮어쓰지 않는다. 원본 E8 checkpoint에만 재개 상태가 보존돼 있다.

## 직접 실행

팀원의 최신 main을 빌드하고 source한 `asmc-ros-noetic`에서 실행한다. 이 launch는 차량 제어를
시작하지 않는다. 기존 카메라 producer를 종료하고 bridge·카메라 센서 입력을 유지한다.

```bash
source /root/ws/devel/setup.bash
roslaunch asmc_camera perception.launch profile:=debug \
  start_camera_lane:=false start_traffic_light:=false
```

사전학습 모델 대조는 종료·재실행 후 아래 인자를 추가한다.

```bash
roslaunch asmc_camera perception.launch profile:=debug \
  objects_model_profile:=pretrained_coco7 \
  start_camera_lane:=false start_traffic_light:=false
```

별도 터미널:

```bash
rosrun rqt_image_view rqt_image_view --clear-config /perception/camera/objects/debug/mosaic/compressed
```

판제 관리 실행은 `profile:=planning`으로 STANDBY에서 시작한다. 기존
`/camera_object_detector/set_inference_mode` (`asmc_camera/SetInferenceMode`)에
mode1 PREPARE → `prepared=true` 확인 → mode2 ACTIVE를 요청한다. 구간 정책은 판제 소유다.
`idle`은 미실행, `debug`는 ACTIVE다. [모드 인터페이스](camera-inference-control.md)를 따른다.
TwinLite 목표는 이번에10Hz를 유지하고,15Hz 통합 최적화는 후속 검증 과제다.

신호등 생산자는 한 곳만 소유한다. 현재 `asmc_planner/autonomy_no_bridges_v2.launch`는
신호등을 기본으로 켠다. 이 launch를 쓰면 카메라 launch는 `start_traffic_light:=false`로 두거나,
카메라가 신호등을 맡는 경우 판제 launch에 `traffic_light:=false`를 명시한다. 판제 신호 규칙을
끄는 인자가 아니다. 이전 facade 시험의 외부 생산자 기본 가정은 현재 코드에 맞춰 수정했다.

생산자 cutoff는 `objects_conf_thres:=0.40`으로 지정한다. 더 낮추려면 재시작한다.
소비자는 `box.confidence`로 더 엄격히 거를 수 있다. 전방 BBox 어댑터의
`/camera_objects_yolo_bbox_adapter/conf_thres`는 추가 cutoff이며 기본0.0이다.
그 값을 낮춰도 생산자가 이미 버린 박스를 되찾을 수 없다.

## Camera → LiDAR → 판제 계약

| `class_name` — 소비 그룹 | `source_class_name` — 원본 이름 |
|---|---|
| vehicle | car, bus, truck |
| dynamic_obstacle | person, dog, deer, bicycle, stroller |
| static_obstacle | motorcycle, bollard, box, concrete_barrier, steel_barricade, traffic_drum, plastic_barrier, traffic_cone |

SportBike/Scooter는 대회 확인에 따라 static이다. 자전거·유모차는 이동하는 사람 에셋에
붙을 수 있어 dynamic이다. 유모차 안의 아기는 별도 person 정답으로 만들지 않는다.
그룹은 이동 가능성과 소비 목적이다. 실측 이동 상태·속도·충돌 위험을 그룹만으로 확정하지 않는다.
`source_class_name`은 모델의16종/7종 이름이며 정확한 MORAI 에셋명을 알아냈다는 뜻이 아니다.
CargoBox/WoodBox는 box, Scooter/SportBike는 motorcycle, PE_Firewall은 plastic_barrier다.

- `/perception/camera/objects`: `asmc_msgs/CameraObjectBoxArray`, 원본 영상 bbox 좌표·크기·source header,
  view(`camera_front/left/right`), 원본 이름·그룹·confidence 보존. 메시지·서비스 MD5는 유지한다.
- `valid=true`·빈 boxes는 정상 빈 관측이다. `valid=false`, 오래된 stamp, 중단 입력은 새 빈 관측과 구분한다.
  카메라 bbox만으로 map 위치·거리·속도를 만들지 않는다.
- Main의 기본 `camera_class_profile:=mission16_v1`, `start_yolo:=false`다. 전방만 기존
  `/YOLO/BBox`로 변환하며 producer는 하나다. 이 구형 wire에는 원본 이름·bbox·header만 있고
  confidence/그룹 필드는 없다. confidence는 변환 전 필터링하고 그룹은 같은 고정 표로 재구성한다.
- fusion은 **전방 bbox·unique·±50ms·200ms 대기**다. 좌우 bbox는 독립 출력이며 아직3뷰 fusion이 아니다.
  분류 확인은 새로운 카메라 관측만 세고, 이미지 source stamp를 유지한다.
- `/lidar/tracked_objects_map`와 validated tracked companion은 `classification`에3그룹,
  `yolo_class`에원본 이름을 보존한다. track의 실측 속도·기하·운동 판정은 별도다.
- 기존 `planning_inputs/DetectedObject` 숫자 enum은 vehicle/pedestrian/unknown이다.
  vehicle은vehicle, dynamic_obstacle+원본person은pedestrian, 나머지 장애물은unknown으로
  기하·속도를 보존한다. **unknown을 객체 없음 또는 정적으로 해석하면 안 된다.**
  판제가3그룹별 정책을 구현할 때는 문자열을 보존한 tracked companion을 함께 사용한다.
- 미등록 라벨은unknown/진단으로 남긴다. 클래스 이름만 보고 static으로 자동 배정하지 않는다.

완전 legacy 복구는 `camera_coco7.launch`와 Main의 `camera_class_profile:=legacy`를 명시한다.
이는 위 `pretrained_coco7` 선택(동일3그룹 계약 유지)과 다르다.

## 검증 범위와 남은 과제

[고정 비교 근거](../../../config/baselines/evidence/camera_mission16_deployment_v1/comparison.json)에서
E8/E22의16종 macroF1은74.21/71.19%,3그룹은74.96/70.38%다. 개발val205를 반복 사용했고
버스·트럭의 완전GT는14/12개로 적다. E8은 버스 F166.67%, 트럭58.33%다.
사전학습은 각각78.26/47.06%여서 버스 회귀를 숨기지 않는다.

[bag 근거](../../../config/baselines/evidence/camera_mission16_deployment_v1/bag-summary.json):
E8/E22 비교8회, E8 TensorRT4회. TensorRT는 약14.8–15Hz, 입력수신→출력
p50 33–36ms·p95 41–45ms, 최대공백173.3ms, 전체VRAM peak1966/8188MiB,
중복·역행stamp0이다. 다른 두 모델은 로드된STANDBY였다.
PyTorch 비교의 일부250ms 초과 공백과 FP16 parity 차이도 보존했다.
독립 연속GT 없는 bag의 전체recall은 만들지 않았다. MORAI+전체ACTIVE·30분 시험은 남아 있다.

다음 작업의 순서·수집 조건·라벨 정책은 [후속 개발 계획](camera-deployment-roadmap-20261014.md)을 따른다.
