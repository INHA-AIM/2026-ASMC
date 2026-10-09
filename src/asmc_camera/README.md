# `asmc_camera` runtime

> **상태:** P4-C6 — camera/BEV/lane/traffic runtime·offline·training 패키지 통합 완료

분산된 camera semantic, metric BEV/lane과 traffic-light를 하나의 인지 실행 경계로 조합한다.
camera semantic, BEV/fusion/lane과 신호등 ROS node 구현, 실행 support library·config, 모델,
TwinLite 학습 adapter와 offline BEV audit은 이 패키지가 소유한다. P4-C6에서 기존
`camera_semantic_perception`, `drivable_bev`, `traffic_light` ROS 패키지와 `src/perception` 그룹을
제거했지만 topic과 node 이름은 바꾸지 않았다. `data_collection`과 팀원 개발 중인 LiDAR/fusion은 독립
패키지·workspace이며 이 launch가 소유하거나 시작하지 않는다.

2026-10-05 이전 계획은 [통합 로드맵](../../docs/architecture/consolidation-roadmap.md)을 따른다.
**2026-10-06 패키지 이름을 `asmc_perception`에서 `asmc_camera`로 바꿨다.** node 이름, `/perception/*`
당시에는 토픽, launch 인자와 기본값을 유지했다. 이후 모델 실행 상태와 프로필을 추가했다. 구 패키지는 호환 wrapper 없이 완전히 옮겼으므로
`roslaunch asmc_perception ...`와 `import asmc_perception`은 동작하지 않는다
([이전 안내](../asmc_perception/README.md)). 전방 YOLO11m 생산자는 opt-in으로 추가했다([P2a 안내](../../docs/handoffs/perception/camera-objects-p2a.md)).
`camera_bev_fusion`은 다중 카메라 처리로 카메라 쪽에 남고, LiDAR 객체에 카메라 분류를
결합하는 fusion은 손재호가 이전을 마무리할 `asmc_lidar` 내부에 둔다. 별도 fusion 패키지는
만들지 않는다.

## 실행

### 선정한 3뷰 개발 모델

[실험 갤러리 통합 목차](http://localhost:8768/)에서 카메라·LiDAR 비교를 함께 확인한다.
8개 M/L·detect/seg·960/1280 비교와 최종 선정 근거는
[3뷰 인계 문서](../../docs/handoffs/perception/camera-three-view-coco-runtime.md)를 따른다.

YOLO26l detect·전방1280·좌우640×480·TensorRT FP16·2+1 batch·뷰별15Hz·
confidence0.40을 개발 기준으로 선정했다. 0.50 대비 검수 정답26개 회복·FP25개 추가를
확인했고 전체 오검출 이미지는 인계 문서의 confidence 갤러리에 보존한다. `asmc-ros-noetic`의 ROS 환경에서 직접 실행한다.

```bash
roslaunch asmc_camera camera_coco7.launch profile:=debug
```

```bash
rqt_image_view --clear-config /perception/camera/objects/debug/mosaic/compressed
```

좌·전방·우 YOLO 결과는 `/perception/camera/objects/debug/mosaic/compressed` 한 화면에
나란히 표시한다. 각 칸은 원본 비율을 유지하며 source stamp·age와 차체 mask를 표시한다.
뷰 간에는 최신 결과를 독립적으로 갱신하므로 하나의 동기 관측으로 소비하지 않는다.
입력이0.5초 이상 오래되면 해당 칸의 박스를 지우고 STALE로 표시한다.
합성 영상 header는 합성 시각이며 원본 관측 stamp는 각 칸에 따로 남는다.
화면은 최대10Hz, YOLO 추론 목표는 뷰별15Hz다. rqt가 없으면 합성용 구독·디코딩을
하지 않고, 기존 개별 뷰 토픽은 그대로 사용할 수 있다. YAML·모델·confidence는 변경하지 않는다.

YOLO만 확인하려면 같은 launch에 `start_camera_lane:=false start_traffic_light:=false`를
추가한다. 기본 debug는 TwinLite·YOLO·ResNet 모두 ACTIVE이며 LiDAR·판제·차량 제어는
시작하지 않는다. 차선 피팅 결과의 주행 소비는 음영 ODD로 한정한다.

좌우 debug 토픽은 `/perception/camera/objects/debug/left/image/compressed`,
`/perception/camera/objects/debug/right/image/compressed`다. planning은 세 모델 STANDBY,
debug는 ACTIVE이며 GUI는 별도로 연다. 모델·뷰별 목표 `target_hz_by_view`·생산자 최소 confidence는
`config/camera/yolo26l_detect_coco7_three_view.yaml`에서 수정한다. 시작 컷만 바꾸려면
`objects_conf_thres:=0.55`를 launch에 추가하고 재시작한다.

판제는 메시지 confidence로 더 엄격한 소비 컷을 적용할 수 있다. LiDAR 호환 어댑터는
`rosparam set /camera_objects_yolo_bbox_adapter/conf_thres 0.65`로 실행 중 추가 컷을
변경하며, `0.0`으로 생산자가 보낸 후보를 모두 다시 통과시킨다. 생산자에서 버린 후보는
복원하지 못한다. 검출 tradeoff·mask 신규 실패·판제 deadline·운영 승격 보류는
[3뷰 모델 선정·실행 안내](../../docs/handoffs/perception/camera-three-view-coco-runtime.md)를 따른다.
아래 기존 `perception.launch` 기본 모델은 별도 운영 수락 전까지 유지한다.

### 기존 실행 프로필

YOLO26s-seg·YOLOE-26s-seg의 별도 관측 실험은
[seg 모델 선정 안내](../../docs/handoffs/perception/camera-seg-model-selection.md)를 따른다.
전방/3뷰 mask overlay를 제공하지만 학습·운영 기본 모델·LiDAR 소비 정책은 바꾸지 않는다.

기본 `perception.launch`는 모델을 실행하지 않는 idle 프로필이다. bridge·컨테이너와
GUI는 별도로 준비하고, ROS 환경을 읽은 터미널에서 다음 중 하나를 직접 실행한다.

```bash
# 판제용: 세 모델 로드·예열 후 STANDBY, 모델별 서비스로 활성화
roslaunch asmc_camera perception.launch profile:=planning
```

```bash
# 디버그용: 모든 모델 ACTIVE + debug 영상 발행
roslaunch asmc_camera perception.launch profile:=debug
```

planning/debug는 TwinLite 10Hz 배치·전방 YOLO 20Hz·camera4 ResNet 10Hz를 목표로 한다.
기본 `start_*=false`(idle)는 모델 노드 자체가 없고, planning의 STANDBY는 GPU 모델을
유지하며 서비스 요청을 받는다. 구간·결과 소비·주행 모드는 판제가 결정한다.
기존 차선 반복 추론은 `profile:=legacy`로 명시적으로 재현한다. 목표 Hz와 초기 모드,
모델별 제외·서비스·시각화는 [직접 실행 안내](../../docs/handoffs/perception/camera-inference-control.md)를 따른다.

bag 기록·재생은 별도 `data_collection` 진입점을 사용한다.

```bash
roslaunch data_collection bag_recorder.launch profile:=full_raw
```

기존 LiDAR는 nested workspace의 `Main`, `Msg`와 Velodyne overlay를 사용한다.
새 `asmc_lidar` 골격은 이 overlay 없이 라이브러리만 빌드하며, 어느 쪽도
`asmc_camera` launch가 자동으로 시작하지 않는다.

### 보넷 mask 기본값과 비교

```bash
roslaunch asmc_camera perception.launch profile:=debug
```

YOLO 입력 복사본에만 정규화 다각형을 적용하고 원본 좌표를 유지한다. overlay에 영역과
활성 상태를 표시한다. 기존 보행자 누락 1건의 사용자 수락 예외를 기록하고 기본값을 **true**로 선정했다.
일반 debug 명령은 semantic GT 보넷 윤곽 mask를 사용한다. 전체영상 비교는 `objects_hood_mask:=false`를
명시한다. 영역 파일·진단·수락 결과는 [mask 안내](../../docs/handoffs/perception/camera-inference-control.md#보넷-제외-semantic-gt-윤곽-mask-기본-활성화)를 따른다.
mask는 YOLO 합성곱 연산량을 줄이는 기능이 아니다.

### 전방 YOLO11m 객체 검출 (opt-in)

bridge·컨테이너·GUI를 준비한 뒤 카메라 터미널에서 직접 실행한다.

```bash
roslaunch asmc_camera perception.launch profile:=debug \
  start_camera_lane:=false start_traffic_light:=false
```

별도 LiDAR 터미널에서 outer와 nested 환경을 순서대로 source하고 실행한다.

```bash
source /root/ws/devel/setup.bash
source /root/ws/src/lidar+sensorfusion/devel/setup.bash
roslaunch Main main.launch start_yolo:=false rviz:=false
```

차선과 함께 화면을 확인하려면 `perception.launch profile:=debug`를 사용한다.
YOLO 전용 화면은 `/perception/camera/objects/debug/image/compressed`이며 LiDAR/GPS가 필요 없다.
`/perception/camera/objects`는 전방 source header·원본 픽셀 좌표와
`person/vehicle/obstacle` 그룹·원래 라벨을 보존한다. 현재 허용 라벨과 confidence는
기존 `bus/truck/person/car`, 0.7이다. 처리 실패는 invalid 배열로 구분한다.
LiDAR의 호환 어댑터가 기존 `/YOLO/BBox`로 변환하며 기본 `start_yolo:=true` 경로는 보존한다.
객체만 검사할 때는 `python3 scripts/verify_perception_runtime.py --camera-objects --no-camera-lane`을 쓴다.
모델·MD5·CPU/GPU·진단·롤백·증거는 [P2a 안내](../../docs/handoffs/perception/camera-objects-p2a.md)를 따른다.

### YOLO11m COCO 80종 관측 실험

기본 네 클래스는 유지한다. 전방 YOLO와 객체 overlay만 ACTIVE로 확인하려면 다음을
실행한다. 신호등·정지표지판도 원본 라벨로 제공하지만 coarse obstacle 그룹이 판제의
회피 대상 확정을 의미하지는 않는다. ResNet 신호 상태 판정과도 구분한다.

```bash
roslaunch asmc_camera camera_objects_coco_experiment.launch classes:=all
```

```bash
rqt_image_view --clear-config /perception/camera/objects/debug/image/compressed
```

이전 launch를 종료하고 `classes:=baseline`으로 바꾸면 네 클래스·같은 보넷 mask를
비교한다. 이 launch는 TwinLite/BEV/차선·ResNet·LiDAR·rqt를 자동 실행하지 않는다.
설정 파일·소비 경계·고정 비교와 검증 상태는 [클래스 확대 안내](../../docs/handoffs/perception/camera-coco-class-expansion.md)를 따른다.

### `camera_bev`와 `camera_bev_fusion`


`camera_bev` 노드는 front/left/right 카메라의 semantic probability·road-marking을
카메라별 calibration으로 `base_link` 평면에 각각 투영한다. 출력은
`/perception/bev/debug/{view}/*`이며, 현재는 후단 fusion의 내부 입력이자 개발
비교 계층이다.

`camera_bev_fusion`은 세 투영 결과의 source timestamp를 맞추고, calibration 품질·
가시 영역 가중치로 겹침을 해결해 하나의 `/perception/bev/camera/*`를 만든다.
`coverage`, `source_count`도 이 단계에서 만들며 lane vectorizer가 소비할 운영 정본은
이 fused output이다. 두 노드는 중복 알고리즘이 아니라 **카메라별 투영 → 다중
카메라 융합**의 순차 단계다.

P5-A의 producer·subscriber·bag 감사 정본은
[`perception_topic_policy_v1.yaml`](../../config/contracts/perception_topic_policy_v1.yaml)이다.
`debug/{view}` semantic 3종과 `model_grid` 2종은 후단 계산 입력이므로 일반 debug image처럼
끄지 않는다. per-view coverage, fused source count/view와 frame-level lane pipeline audit은
P5-B에서 명시적 평가 옵션으로 전환했다.

P5-C는 topic을 다시 바꾸지 않고 현재 연결을 실행 중 검사한다. perception과 필요한 선택
process를 먼저 실행한 뒤 저장소 루트 host에서 다음을 사용한다.

```bash
./scripts/verify_perception.sh
./scripts/verify_perception.sh --traffic-light --visualization
./scripts/verify_perception.sh --traffic-light --require-planning
```

첫 명령은 camera→BEV→lane의 node/type/producer/subscriber/rate/stamp/frame을 검사한다.
두 번째는 신호등과 canonical rqt 두 topic, 세 번째는 판제의 typed/legacy 신호등 구독까지
확인한다. `lane_boundary_adapter`를 함께 띄운 검수에는 `--require-lane-adapter`를 별도로 쓴다.
정확한 옵션과 실패 해석은
[`perception-p5c-runtime-acceptance-v1.md`](../../docs/records/baselines/perception-p5c-runtime-acceptance-v1.md)를
따른다.

## 시각화

debug image publisher를 함께 켠다.

```bash
roslaunch asmc_camera perception.launch profile:=debug
```

화면은 GUI 환경이 있는 `asmc-ros-noetic` 컨테이너에서 연다. 아래 명령은 먼저
`docker exec -it asmc-ros-noetic bash`로 들어간 각각의 터미널에서 실행한다.

```bash
rqt_image_view --clear-config /perception/camera/debug/mosaic/compressed
rqt_image_view --clear-config /perception/bev/debug/comparison/compressed
```

첫 topic은 좌·전방·우의 원본·주행가능·road-marking을 모은 9칸 영상이다. 두 번째는
calibration으로 투영·융합한 원본 RGB BEV와 fused semantic BEV를 같은 범위로 나란히
보여준다. semantic 쪽은 0.5 기준 이진 주행가능 영역과 고정 색상의 차선 class만
표시하며 확률·confidence 밝기를 인코딩하지 않는다. 둘 다 subscriber가 있을 때만
렌더링하고 JPEG를 발행한다.
`start_visualization`은 idle/planning에서 false, debug에서 true다. rqt 창은 별도로 연다.

신호등은 `start_traffic_light:=true`를 함께 주면 `camera_traffic_light_visualizer`가 추가로 뜬다.

```bash
roslaunch asmc_camera perception.launch profile:=debug
rqt_image_view --clear-config /perception/traffic_light/debug/image/compressed
```

이 영상은 typed `/perception/traffic_light/detection`을 같은 stamp의 `/cam_extra` 원본 위에
그린다. 테두리는 판정 색(빨강·노랑·초록, 무효·stale·미검출은 회색)이고, 왼쪽 위에 label과
confidence, VALID/STALE/INVALID, 추론 시간, 상위 3개 후보가 표시된다. 같은 stamp의 원본이
캐시에 없으면 최신 프레임 위에 그리고 그 사실을 문구로 남긴다. 다른 debug 영상처럼
subscriber가 있을 때만 렌더링한다.

이전 camera별 panel, fused/fused-quality, lane evidence/fitted/boundary/reprojection 영상이
필요한 한시적 비교에서는 `visualization_legacy_rqt:=true`를 추가한다. camera별 지면
calibration grid는 별도 `visualization_calibration_grid:=true`로만 켠다. RViz marker는
이 rqt 간소화의 변경 대상이 아니다.

per-view projected coverage와 fused source count/view도 기본 off다. 평가 bag이나
overlap 분석에서 필요하면 `publish_bev_debug_metadata:=true`를 지정한다. legacy rqt를
켜면 이 metadata도 호환 기본값으로 함께 켜진다.

큰 frame별 `/perception/lane/pipeline_audit`도 일반 주행에서는 광고·누적하지 않는다.
rejection 원인 분석이나 result bag replay에서는 `lane_publish_pipeline_audit:=true`를
지정한다. `run_shadow_lane_bag_batch.py`는 기존 평가 계약을 위해 이 값을 자동으로 켠다.

`start_traffic_light`은 producer 노드 생성 옵션이며 구간 판단 옵션이 아니다.
planning은 STANDBY로 외부 요청을 기다리고 debug는 ACTIVE로 계속
추론하며, HD맵의 `TRAFFIC_LIGHT_REQUIRED` 구간(A236~A308)에 따른 process/compute 활성화는
판제 supervisor와 함께 연결할 후속 항목이다.

## P4 closeout MORAI live 확인

이 절은 package 통합 PR 전 사람이 MORAI를 주행하며 확인하는 최소 절차다.
MORAI UI에서 실행 release에 맞는 `*_local.json`과
`ASMC_sensor_set_v3_local.json`을 불러온 뒤 Play/Connect한다.

```bash
# Terminal A — asmc-ros-noetic container, local UDP bridge
docker exec -it asmc-ros-noetic bash
cd /root/ws
./scripts/udp_bridge.sh

# Terminal B — asmc-ros-noetic container, canonical perception
docker exec -it asmc-ros-noetic bash
cd /root/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch asmc_camera perception.launch profile:=debug
```

별도 터미널에서 ROS 토픽을 확인한다. `rostopic hz`는 계속 대기하므로 항목별로
`Ctrl+C`로 나오거나 터미널을 나눠 실행한다.

```bash
# Terminal C — asmc-ros-noetic container
docker exec -it asmc-ros-noetic bash
cd /root/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# MORAI v3 camera input: front/left/right 약 20 Hz, extra 약 10 Hz
rostopic hz /image_jpeg/compressed
rostopic hz /cam_left/image_jpeg/compressed
rostopic hz /cam_right/image_jpeg/compressed
rostopic hz /cam_extra/image_jpeg/compressed

# canonical runtime output: GPU 상태에 따라 입력보다 느릴 수 있지만 계속 갱신되어야 함
rostopic hz /perception/camera/front/drivable/probability
rostopic hz /perception/bev/camera/drivable_probability
rostopic hz /perception/lane/boundary_family
rostopic hz /perception/traffic_light/detection

# 계약·상태 sample
rostopic echo -n 1 /perception/lane/boundary_family/header
rostopic echo -n 1 /perception/lane/boundary_family/operating_domain
rostopic echo -n 1 /perception/lane/boundary_family/algorithm_id
rostopic echo -n 1 /perception/lane/boundary_family/deployment_allowed
rostopic echo -n 1 /perception/lane/diagnostics
rostopic echo -n 1 /perception/bev/diagnostics
rostopic echo -n 1 /perception/traffic_light/detection
```

`/perception/lane/boundary_family`는 음영구간 판제 소비 정본이며 무옵션
`profile:=planning`/`profile:=debug`에서 차선 파이프라인을 켜면 발행된다. legacy 개별 spline·tracker와
`/perception/lane/boundary_observations`는 기본 비활성화다. 기존 호환·진단용으로
필요할 때만 `lane_enable_independent_boundary_pipeline:=true`를 사용한다.

음영구간 adapter는 제어와 연결하지 않고 별도로 켠다.

```bash
# Terminal D — asmc-ros-noetic container
cd /root/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch lane_boundary_adapter shadow_boundary_family_candidate.launch

# Terminal E — adapter output
rostopic hz /planning/debug/lane_boundary_family/strips
rostopic echo -n 1 /planning/diagnostics/lane_boundary_family_adapter
```

adapter launch는 `base_link` 전용 RViz를 자동으로 열고
`/perception/lane/debug/shared_family/markers`와
`/planning/debug/lane_boundary_family/strips`를 기본 표시한다. headless replay나 성능
측정에서는 `rviz:=false`를 지정한다.

rqt 영상이 필요할 때만 각각 별도 `asmc-ros-noetic` 터미널에서 연다.

```bash
rqt_image_view --clear-config /perception/camera/debug/mosaic/compressed
rqt_image_view --clear-config /perception/bev/debug/comparison/compressed
```

자동 RViz의 음영 ODD 밖에서
`valid=false` 또는 정지·disconnect 후 stale/marker clear는 정상 fail-safe다.
음영구간은 A448 종료 후부터 A126 종료까지이며 회피용 반대 차로는 A131이다.
이 구간에서는
`frame_id=base_link`, `operating_domain=shadow_two_way_v1`, 동결 algorithm ID,
`deployment_allowed=false`를 확인한다. observed는 청록, structure-inferred는 주황,
invalid는 연결하지 않은 실제 공백으로 보여야 한다.

신호등은 A236~A308에서 실제 색과 typed
`/perception/traffic_light/detection`, 호환 `/traffic_light/state`의 전환을 같이 확인한다.
이 live 검사에서는 `/ctrl_cmd`를 발행하지 않으므로 수동 주행 중에 사용해도 된다.

## P4-C1 경계

- camera lane은 v3 센서셋과 SHA로 고정한 `cameras_v3.yaml`을 사용한다.
- v2와 v3의 네 camera pose, intrinsics, period와 UDP 설정이 동일한지는 unit test가 검사한다.
- 신호등은 기존 `/traffic_light/state`, `/traffic_light/detail`과 typed
  `/perception/traffic_light/detection`을 병행 발행한다.
- typed 결과는 실제 추론에 사용한 `camera_extra` image header를 그대로 보존한다.
- LiDAR는 기존 `/lidar/tracked_objects_map`, `/last_lidar`를 유지한다.
- 신규 `asmc_msgs/TrafficLightDetection`, `asmc_msgs/TrackedObjectArray`는 additive 계약이다.
- LiDAR typed producer는 아직 없으므로 해당 canonical topic을 발행한다고 가정하면 안 된다.
- traffic-light 체크포인트는 `models/traffic_light/best.pt`가 정본이다.
- Capture/gRPC GT, simulator object/traffic truth는 deployment 입력으로 사용하지 않는다.

## P4-C2 parity 회귀

대표 raw bag의 `camera_extra` 프레임을 기존 producer와 canonical producer에 한 장씩 같은
순서로 넣어 legacy String, typed result, source header, stale 전환과 복구를 비교한 동결 시험이다.
호환 producer 패키지는 P4-C6에서 제거했으므로 이 A/B harness를 현재 HEAD에서 다시 실행하지
않는다. 결과와 입력·체크포인트 SHA는 baseline validator로 확인한다.

```bash
python3 tools/audit/validate_traffic_light_parity.py --check-artifacts
```

기준 결과는 `config/baselines/perception_traffic_light_parity_v1.yaml`이다. 이 통과는 producer
병행 발행의 근거이며 판제 consumer cutover 완료를 뜻하지는 않는다.

## P4-C3 판제 consumer

`behavior_planner`는 이제 `/perception/traffic_light/detection`을 권위 입력으로 구독한다. 기존
`/traffic_light/state` 구독도 유지하지만 typed message를 한 번도 받지 못한 구 실행을 위한
fallback일 뿐이다. typed message를 받은 뒤에는 다음 규칙을 적용한다.

- `valid=false` 또는 `stale=true`: `UNKNOWN`, legacy String으로 우회하지 않음
- typed 수신이 local stale timeout을 넘김: `UNKNOWN`, legacy String으로 우회하지 않음
- typed producer를 한 번도 보지 못함: fresh legacy String 사용

기존 판제 통합 launch가 시작하는 executable은
`traffic_light/live_infer.py`에서 `asmc_camera/traffic_light_node.py`로 바뀌었다. topic 이름은
삭제하거나 remap하지 않았으며 producer 경로 변경은 두 launch 파일의 migration 주석과
`behavior_planner/README.md`에 기록돼 있다. unit/build, live ROS 연결과 MORAI 차량
정지·재출발까지 통과했다. 다만 모든 consumer/launch의 legacy 참조가 없어지기 전에는 기존
topic을 제거하지 않는다.

## P4-C4 신호 수락시험

production `behavior_node`, 현재 MGeo와 전역경로를 쓰되 Frenet/MPC/UDP 출력은 시작하지 않는
격리된 ROS decision-loop 시험을 제공한다.

```bash
# asmc-ros-noetic 컨테이너
roslaunch asmc_camera traffic_light_closed_loop_acceptance.launch
cat /tmp/asmc_traffic_light_closed_loop_acceptance.json
```

고정 pose는 A728 정지선 8.95 m 전방이다. 시험은 같은 typed stream에서
`GREEN → KEEP`, `RED → STOP(map 목표점 포함)`, `GREEN → 새 KEEP command_id`를 검사한다.
기준은 `config/baselines/perception_traffic_light_closed_loop_v1.yaml`이다. 이 시험은 판제 판단
loop의 수락 근거이며 실제 simulator의 제동거리·MPC·UDP actuator까지 통과했다는 뜻은 아니다.

실제 `behavior → Frenet → MPC → /ctrl_cmd → UDP → MORAI` 차량 시험은 다음처럼 별도 opt-in한다.
기본 launch는 제어가 잠겨 있으며 아래 두 flag가 모두 있어야 Ego를 텔레포트하고 움직인다.

```bash
# controlled simulator only
roslaunch asmc_camera traffic_light_vehicle_acceptance_stack.launch \
  run_acceptance:=true allow_vehicle_control:=true \
  repo_root:=/root/ws grpc_host:=host.docker.internal
cat /tmp/asmc_traffic_light_vehicle_acceptance.json
```

MPC 출력은 `/acceptance/ctrl_cmd`에 격리되고 시험 node가 원래 Ego 상태를 저장한 뒤에만
`/ctrl_cmd`로 전달한다. 종료 시 full brake, gate close, pose·speed·gear·control mode 복원을
수행한다. 26.R1 local 실측에서 적색은 물리 정지선 5.21 m 전에 `0.0 m/s`로 멈췄고, 녹색에서
1.33 m 재진행·2.27 m/s를 확인했다. 근거는
`config/baselines/perception_traffic_light_vehicle_closed_loop_v1.yaml`이다. 이 결과는 synthetic
typed 신호 이후 제어 loop 수락이며 camera 분류 정확도 시험은 아니다.

## P4-C4/C5 물리 이식 경계

다음 실제 node 구현은 `src/asmc_camera/src/asmc_camera/nodes/`로 이동했다.

- camera semantic inference와 visualizer
- per-view BEV, 3-view fusion과 BEV visualizer
- lane vectorizer와 lane visualizer

canonical launch는 모두 `pkg="asmc_camera"`로 실행한다. P4-C5에서 camera backend,
runtime BEV/lane 수학과 config 정본도 각각
`asmc_camera.camera`, `asmc_camera.bev`, `asmc_camera/config`으로 옮겼다. 기존 Python namespace는 canonical
symbol을 다시 export하던 호환 package와 config mirror는 P4-C6 검증 후 제거했다.

재학습 코드는 `asmc_camera.training.twinlite`, offline replay·pseudo-GT·audit 코드는
`asmc_camera.offline.bev`와 `tools/bev/`에 분리했다. 이 namespace는 차량 runtime launch가
import하거나 시작하지 않는다. 긴 실험 이력은 `docs/`에 보존한다.

live 3-camera→BEV→lane 출력은 `config/baselines/perception_runtime_move_smoke_v1.yaml`에
고정한다. 입력 bag과 recorder의 소유권은 계속 `data_collection`에 있다.

P4-C5 representative raw bag을 0.5배속으로 두 진입점에 각각 재생한 결과 front class-ID 79,
fused BEV 38, lane observation 43개의 공통 source stamp에서 payload mismatch가 모두 0이었다.
ROS connection-local `header.seq`만 비교에서 제외하고 frame·stamp·config SHA와 전체 payload는
그대로 비교했다. 당시 도구는 호환 package 제거와 함께 retired 처리했고, 측정값·input SHA와
canonical config/model SHA는 `config/baselines/perception_support_config_parity_v1.yaml`에 보존한다.

기계 판독 상태는 `config/runtime.yaml`, 파트 간 topic·message 정본은
[`perception_runtime_topic_contract_v1.yaml`](../../config/contracts/perception_runtime_topic_contract_v1.yaml),
전체 이관 gate는 [Perception 개발·통합 가이드](../../docs/packages/asmc_perception/README.md)를 따른다.

## 검증

```bash
python3 -m unittest src/asmc_camera/test/test_facade_contract.py
python3 -m unittest src/asmc_camera/test/test_traffic_light_contract.py
python3 tools/audit/validate_perception_contract.py
python3 tools/audit/validate_traffic_light_parity.py --check-artifacts
roslaunch --nodes asmc_camera perception.launch
roslaunch asmc_camera traffic_light_closed_loop_acceptance.launch
# controlled simulator only; explicit opt-in required
roslaunch asmc_camera traffic_light_vehicle_acceptance_stack.launch \
  run_acceptance:=true allow_vehicle_control:=true
```

P4-C6에서 camera/BEV/traffic 호환 package를 제거하고 offline·training 자산까지 이 패키지로
이관했다. legacy topic은 외부 소비자 호환 계약이므로 package 삭제와 무관하게 계속 병행한다.
`data_collection`은 독립 package로 유지한다.
`src/lidar+sensorfusion`은 담당 팀원이 개발 중인 독립 workspace이므로
그 파일과 내부 `Msg`·`morai_msgs`는 건드리지 않는다. LiDAR 완성·handoff 뒤에만 별도 P4-D로
canonical adapter와 message 전환을 진행한다.

## 모델별 외부 실행 제어

현재 차선 피팅과 `LaneBoundaryFamily`는 음영구간 전용이다. 음영구간 밖의 구현·검증은
아직 완료되지 않았으므로 그 결과를 주행에 사용하면 안 된다. debug 프로필의 전체 ACTIVE와
결과의 `valid`/`pipeline_ready`는 구간 밖 주행 사용을 허용하지 않는다. 구간 적용·활성화·
결과 소비 조건은 판제가 결정하며 카메라 생산자에 GPS/구간 판단을 넣지 않는다.

`managed:=true`에서 TwinLite·YOLO·ResNet18을 독립적으로 STANDBY/PREPARE/ACTIVE로
요청할 수 있다. 직접 launch·서비스·GUI 명령, 판제 역할과 현재 수락 제한은
[모델별 실행 상태 안내](../../docs/handoffs/perception/camera-inference-control.md)를 따른다.

## LiDAR와의 관측 좌표 경계

카메라 객체는 원본 영상의 픽셀 좌표·header를 유지한다. LiDAR 관측별 수평화
역변환·장착 변환·local/map 출력과 원본 박스의 fusion은 Main의
[공통 경계 어댑터](../../docs/handoffs/perception/lidar-camera-frame-boundaries.md)가 담당한다.
구간 활성화·결과 소비·차선 주행 모드는 판제가 결정한다.

## YOLOE-l 3뷰 첫 실행 실험

별도 [42종/27종·TensorRT batch 실험 안내](../../docs/handoffs/perception/camera-yoloe-l-first-run.md)를
따른다. `camera_objects_yoloe_l_experiment.launch`는 준비된 실험 runtime에서만 실행하며,
기존 운영 YOLO11m 기본값을 변경하지 않는다. 원본 이름과 네 소비 그룹을 보존하고,
공식 l checkpoint의 one-to-many head·동일 NMS를 PyTorch/TensorRT에 사용한다.
