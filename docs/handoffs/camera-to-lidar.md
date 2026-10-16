# 카메라 YOLO → 기존 Main → 손재호 LiDAR 이전 — 2026-10-16

시작점은 [LiDAR 인계](lidar-migration.md)다. 카메라 실행은 [카메라 계약](../contracts/camera-runtime-v1.md),
관측 metadata는 [기계 정본](../../config/contracts/camera_object_observation_v1.yaml)을 따른다.
현재 판제는 nested `lidar+sensorfusion`의 Main 출력을 소비한다. `asmc_lidar` 전체 이전은 완료되지 않았다.

## 현재 연결과 소유권

```text
asmc_camera/camera.launch — 3뷰 YOLO, 독립 실행
  → /perception/camera/objects — 원본 픽셀 bbox·이름·그룹·confidence
  → Main의 전방 전용 CPU 어댑터
  → /YOLO/BBox — 기존 Msg wire
  → Main의 전방 fusion·LiDAR 추적
  → /last_lidar + 기존 tracked/motion 출력
  → planning_inputs → 판제
```

- **안승현/카메라:** 모델·전처리·bbox·세 그룹·confidence·3뷰 관측과 실행 성능.
- **이번 Main 정리:** 독립 YOLO 노드/래퍼/전용 가중치/설치 경로를 제거한다. Main은 카메라 모델을 시작하거나 로드하지 않는다.
- **손재호:** 검출·지면/구조물·군집·OBB·추적·카메라 매칭/fusion·ROS 출력의 `asmc_lidar` 이전.
- **판제:** 결과 소비·행동 정책·음영 lane 활성화. 이번 변경은 판제 입력 토픽·MD5·정책을 바꾸지 않는다.

음영 어댑터의 `camera_bbox_detector.launch`도 별도 추론 없이 전방 CPU 변환만 제공한다. 전용 출력 토픽과 소비 cutoff0.7은 그대로이며 Main 어댑터의 기본0.0과 구분한다.
수집/시나리오 launch의 폐기 `start_yolo` 인자도 제거했다.

Main은 이미 외부 카메라가 기본이었다. 이번 정리는 `start_yolo`와 `yolo_device/yolo_require_cuda/yolo_perf_log_interval_sec` 복구 옵션을 없앤다.
이 인자를 넘기는 기존 호출에서는 제거해야 한다. 복구 모델은 카메라의 `detector:=pretrained`를 사용한다.
`camera_class_profile`은 fusion의 분류 정책이며 독립 YOLO 실행 옵션과 다르므로 유지한다.

## 카메라 기본 설정과 소비 정보

| 항목 | 현재 기본 |
|---|---|
| 객체 모델 | YOLO26l detection v11 E8, TensorRT FP16 |
| 복구/대조 | 공식 YOLO26l COCO80 checkpoint, runtime 7종 필터; `detector:=pretrained` |
| 전방 tensor | B1·1×3×544×960, 원본1280×720 → 960×540+상하2px padding |
| 좌우 tensor | B2·2×3×480×640, 원본640×480 |
| 주기·후처리 | YOLO15Hz·TwinLite15Hz·ResNet10Hz 목표, confidence0.40·동일 클래스 NMS0.50 |
| 차체 mask | YOLO 전방/좌/우 ON; 원본 영상 bbox·header·보정 유지 |
| 모델/프로세스 | 패키지 내부 배포 파일, 독립 프로세스; 환경/SHA 불일치에 자동 fallback 없음 |
| TwinLite/family | Medium v6 TensorRT, 같은 B1/B2 크기, 음영구간에서만 주행 소비 |

Tensor 크기로 카메라 intrinsics를 바꾸지 않는다. 검출은 이미 원본 픽셀 좌표로 복원돼 있다.
목표 Hz가 같아도 두 모델의 source stamp나 실행 위상이 같다는 뜻은 아니다.

| 소비 그룹 | 원본 학습 클래스 |
|---|---|
| vehicle | car, bus, truck |
| dynamic_obstacle | person, dog, deer, bicycle, stroller |
| static_obstacle | motorcycle, bollard, box, concrete_barrier, steel_barricade, traffic_drum, plastic_barrier, traffic_cone |

`source_class_name`은 모델 이름이고 MORAI 에셋 ID가 아니다. CargoBox/WoodBox는 box, Scooter/SportBike는 motorcycle, PE_Firewall은 plastic_barrier다.
그룹은 대회 에셋의 이동 가능성과 소비 목적이다. 실제 이동·속도·`is_dynamic`은 추적 결과로 판단한다.

### 현재 전방 호환 경계

`/perception/camera/objects`의 세 뷰 중 `camera_front`만 변환한다. `/YOLO/BBox` 발행자는 어댑터 하나다.
구형 wire에는 bbox·원본 이름·header만 있고 confidence·그룹·session/generation 필드가 없다.
Confidence는 변환 전에 거르고 그룹은 Main의 mission16 표로 재구성한다. 추가 cutoff 기본0.0으로 생산자의0.40 결과를 통과시킨다.
`/camera_objects_yolo_bbox_adapter/conf_thres`로 더 엄격히 거를 수 있지만 생산자가 버린 박스는 복구하지 못한다.

정상 빈 결과는 새 source header의 빈 배열로 전달한다. 무효/측방/0 stamp 입력은 전달하지 않는다.
모드·세대와350ms silence 무효화는 Main의 기존 상태 구독/시간 게이트가 담당한다. 구형 wire 자체가 전체 관측 metadata를 전달한다고 설명하지 않는다.
전방 fusion은 **unique·±50ms·최대200ms 대기**와 기존 확인/변경/누락 유예를 유지한다.
카메라 결과가 없거나 비어 있어도 LiDAR 객체를 삭제하지 않는다. 3뷰 출력은3뷰 fusion 완료가 아니다.

Tracked 결과의 `classification`은 그룹, `yolo_class`는 원본 이름이다.
판제의 기존 숫자 enum은 vehicle/pedestrian/unknown이고, 원본 person 외의 dynamic/static 장애물은 unknown으로 표현될 수 있다.
**unknown은 객체 없음이나 정지 상태가 아니다.** 기하·속도는 보존되며 세 그룹별 정책에는 문자열을 보존한 tracked 정보를 확인해야 한다.

## 빌드·실행·복구

최신 checkout이 컨테이너에서 보이는 경로를 사용한다. 기존 `/root/ws` 마운트를 이 문서 때문에 바꾸지 않는다.
바깥 workspace를 먼저 빌드/source하고 nested Main을 빌드한다.

```bash
cd /path/to/2026-ASMC
source /opt/ros/noetic/setup.bash
./scripts/build_ws.sh -j2
source devel/setup.bash
cd src/lidar+sensorfusion
catkin_make -j2 -l2 -DCATKIN_WHITELIST_PACKAGES='Main;Msg;morai_msgs'
```

모든 터미널은 같은 ROS master를 사용한다. Bridge는 기존 한 세트만 유지한다.
카메라 터미널은 바깥 workspace를 source한다.

```bash
source /opt/ros/noetic/setup.bash
source /path/to/2026-ASMC/devel/setup.bash
roslaunch asmc_camera camera.launch models:=objects
```

별도 터미널에서 준비를 요청한다. 서비스 accepted만으로 준비 완료를 판단하지 않는다.

```bash
rosservice call /perception/camera/objects/set_mode '{mode: 1, target_hz: 0.0}'
rostopic echo /perception/camera/objects/status
# 같은 generation의 prepared=true 확인 후 Ctrl-C
rosservice call /perception/camera/objects/set_mode '{mode: 2, target_hz: 0.0}'
```

Main 터미널은 바깥→nested 순으로 source한다.

```bash
source /opt/ros/noetic/setup.bash
source /path/to/2026-ASMC/devel/setup.bash
source /path/to/2026-ASMC/src/lidar+sensorfusion/devel/setup.bash
roslaunch Main main.launch rviz:=false
```

카메라 미실행/STANDBY에서도 LiDAR 처리는 유지하지만 새 카메라 분류는 공급되지 않는다.
LiDAR-only 비교는 `enable_sensor_fusion:=false`를 명시한다. Bridge와 Main의 Velodyne UDP 수신자를 중복 실행하지 않는다.

객체 시각화는 별도 한 세트만 실행한다.

```bash
roslaunch asmc_camera view.launch models:=objects
rosrun rqt_image_view rqt_image_view --clear-config /perception/camera/objects/debug/mosaic/compressed
```

복구는 기존 카메라 launch를 종료한 뒤 `camera.launch models:=objects detector:=pretrained`로 재시작하고 PREPARE/ACTIVE를 다시 확인한다.
메시지·세 그룹 계약은 유지하지만 사전학습7종은 신규 장애물 지원 모델이 아니다.
신호등을 판제 launch가 이미 실행한다면 카메라 `models:=all`을 추가로 실행하지 않는다. 모델별 생산자는 하나만 둔다.

### 이전 빌드의 잔류 실행파일

CMake에서 설치 항목을 제거해도 과거 devel/install 파일은 자동 삭제되지 않는다.
해당 workspace의 실행을 끝낸 뒤 아래 **두 실행파일의 devel/install 사본만** 실제 잔류 여부를 확인해 정리하고 다시 빌드한다.

```text
src/lidar+sensorfusion/devel/lib/Main/YOLODetection.py
src/lidar+sensorfusion/devel/lib/Main/yolo_detection_node
src/lidar+sensorfusion/install/lib/Main/YOLODetection.py
src/lidar+sensorfusion/install/lib/Main/yolo_detection_node
```

다른 사람의 workspace·노드·rqt를 일괄 삭제/종료하지 않는다. 새 설치본에서는 Main의 두 구형 실행파일이 존재하지 않아야 한다.
과거 P2a 비교의 소스·가중치는 변경 전 커밋 `ae8dcbc0eb80f06f8c8db54742543aad369ef3cc`에 있다.
재현이 필요하면 그 커밋의 별도 checkout·일치하는 빌드/source 환경·독립 ROS master를 사용한다. 현재 운영에 복구하지 않는다.

## 손재호의 다음 구현 경계

`asmc_lidar/config/camera_objects.yaml`은 참조 설정과 계약 시험뿐이다. 구독·버퍼·투영·fusion 노드는 아직 없다.
골격에는 카메라 모델·TensorRT·Main/Msg 의존성을 추가하지 않는다.

1. **관측 수신·버퍼:** `/perception/camera/objects/observations`의 `CameraObjectObservation`을 뷰별로 받는다.
   `detections.header`·원본 크기와 session/generation·calibration/model/config ID를 보존한다.
   세션/세대 변경·stamp 회귀·STANDBY·입력 중단에서 이전 관측을 제거하며 중복 관측을 새 확인 횟수로 세지 않는다.
2. **보정·투영:** frame과 해당 카메라 보정·원본 영상 크기를 연결한다. source stamp는 bridge source/fallback 정책을 따른다.
   수평화/장착/관측시점 pose 변환을 중복 적용하지 않는다. bbox만으로 거리나 map 위치를 생성하지 않는다.
3. **매칭·통합:** 3뷰의 서로 다른 관측 시각·가림·중복 객체를 처리한다. 현재 전방50ms/200ms 정책을3뷰 구현 완료로 복사하지 않는다.
   큐 크기·새 매칭 정책·중복 근거 통합은 구현 시 별도 비교로 확정한다.
4. **출력·이전:** 원본 이름/그룹과 실측 운동 상태를 분리하고, 기존 판제 계약을 공동 확인한 뒤 `asmc_lidar`로 소비를 전환한다.

정상 빈 결과는 `detections.valid=true`와 빈 boxes다. 무효 결과는 `valid=false`이며 미수신/stale과 구분한다.
새 관측 계약은 이 차이를 전달하지만 현재 구형 wire의 손실을 자동 해결한 것으로 표시하지 않는다.

## 검증 상태와 남은 개선

PR #85의 현재 카메라 실행은 사용자 직접 주행 확인 후 main에 반영됐다. 전체 자동 수락과는 구분한다.
SUNNY source-age 저하와10분 family 최대261.7ms 공백은 [최신 family 기록](../records/camera/camera-family-supply-optimization.md)에 남아 있다.
카메라/family 모델·피팅은 이번에 바꾸지 않으므로 기존 품질 검증을 재사용한다.

사용자 관찰의 bus/truck 간헐적 누락, 차로 앞 bicycle/dog, `dog 2`·`dog 10` 약세도 같은 기록을 따른다.
추가학습은 판제 측 개선 요청 이후에 진행한다. 시간 공급/좌표 문제가 학습으로 해결된다고 가정하지 않는다.
이번 연결·빌드 검증 결과는 [작은 증거](../../config/baselines/evidence/camera_lidar_handoff_v2)에 보존하며 공동 주행 또는3뷰 fusion 수락으로 확대하지 않는다.

### 이번 연결 검증 결과

| 검사 | 결과 |
|---|---|
| 바깥 카메라/메시지/골격·nested Main | 분리된 clean build/install 통과 |
| 기존 Main 회귀 | 320개·오류/실패0 |
| 골격 | CTest4개 suite 통과, 새 Python 교차 계약4항목 포함 |
| 카메라/계측 | runtime80·detection89·계측36 통과 |
| 실제 ROS | 어댑터 wire·모드/세대·COCO7 어댑터·단일 발행 경로 통과 |
| 음영 소비 경로 | 기존 lane/clipped/lane-change/Ego-mask ROS fixture4개 통과; 소비 cutoff0.7 유지 |
| 메시지·설치 | 기존 Msg MD5 동일; 새 설치본에 구형 detector/relay 없음, 어댑터·카메라 engine·골격 설정 있음 |
| Main 생산 코드 | 변경 전 커밋의 C++/header75개와 SHA 동일 |

고정 primary/shadow bag의 전방 RGB1,473장을 현재 E8·960·mask·confidence0.40/NMS0.50으로 재추론했다.
이를 동일한 관측/가용 시각 입력으로 변경 전후 Main의 실제 처리 함수를 호출하는 검증 전용 replay에 넣었다.

| Bag | RGB | LiDAR frame | 유효 local 입력 frame | 객체 관측 | 전후 전체 필드 차이 |
|---|---:|---:|---:|---:|---:|
| primary |979|649|648|21,028|0|
| shadow |494|208|208|13,515|0|

객체 관측 수는 반복 등장하는 track의 frame별 합계이며 고유 객체나 GT 수가 아니다.
원본 이름·그룹·ID·운동/상태·기하·지면 등을 모두 비교하고 수치 허용 오차는1e-6이다.
RGB record 시각을 논리적 예측 가용 시각으로 사용했으므로 **실제15Hz 스케줄링·추론 지연·현장 공급 수락은 아니다.**
실제 ROS 전송과 모드 경계는 별도 fixture에서 검사했다. 카메라 결과는 학습 정답으로 저장하지 않았다.

첫 bag 대조에서는 참조 harness 복사 시 보존된 mtime 때문에 이전 object가 재사용돼 분류 차이가 발생했다.
생산 코드는 바꾸지 않고 두 버전이 같은 검증용3그룹 초기화를 사용하도록 실제 재컴파일한 뒤 위 결과를 확인했다.
최초 실패·실행파일 SHA·정정 근거는 [조건 정정](../../config/baselines/evidence/camera_lidar_handoff_v2/condition-corrections.json)에 보존한다.
이 검증에서 production main을 우회하는 replay의 카메라 분류 초기화 누락도 보완했다. 과거 분류 대조에는 `_camera_class_profile:=legacy`를 명시한다.

재현 도구는 `tools/perception/prepare_camera_handoff_replay.py`, Main의 `lidar_continuous_replay`,
`tools/perception/compare_camera_handoff_replay.py`다. 참조와 현재 checkout에는 같은 최신 replay harness를 적용/재빌드하고 생산 소스는 보존한다.
Raw bag·예측·전체 로그는 `data/runtime/camera-lidar-handoff-v2/`에 있고 Git에는 요약·SHA·시험/정정 근거만 올린다.
시험 ROS/rqt 잔류0을 확인했고 사용자 bridge·컨테이너 마운트·MORAI 날씨/actor는 유지했다.
