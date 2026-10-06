# 카메라 객체 생산자 P2a 사용·검증

> 기준: main `af253c3575c1e704e70511c2a2033ec15e33299a`, 전방 YOLO11m 동작 보존.
> opt-in 구현이며 LiDAR 전체 이전이나 대회 full-stack 성능 승인을 뜻하지 않는다.
> 최신 통합 기준은 컨트롤러 PR #67을 포함한 main `c5d0b78`이다. 검출·LiDAR·HD맵 실행 코드는
> 기존 측정 기준과 동일하며 최신 main에서 전체 빌드와 카메라·메시지 회귀를 다시 확인했다.

`/image_jpeg/compressed` → `asmc_camera` → `/perception/camera/objects` →
`Main` 호환 어댑터 → `/YOLO/BBox` → 기존 fusion으로 연결한다.
LiDAR 처리 주기, 50ms 관측 stamp 매칭과 200ms 도착 대기는 유지한다.
기본 `Main` 실행은 기존 YOLO를 사용하고 새 생산자는 기본 비활성화다.

## 빌드와 직접 실행

컨테이너·bridge·ROS master는 사용자가 준비한다. 모든 터미널은 동일한 ROS master를 사용한다.
실시간 실행은 `asmc-ros-noetic`에서 `ROS_MASTER_URI=http://127.0.0.1:11311`,
`ROS_HOSTNAME=localhost`로 통일한다. 컨테이너의 `/root/ws`는 `asmc-camera-objects`
worktree여야 한다. 예전 HD맵 worktree나 rename 검증 worktree에는 P2a launch가 없다.
컨테이너를 재생성했으면 기존 터미널에서 나와 새로 진입한다. bridge 또는 `roscore`가
실행된 뒤 `rosparam`을 사용한다. `source`는 다른 workspace의 파일을 새 코드로 바꾸지 않는다.
바깥 메시지를 먼저 생성하고 source한 뒤 nested workspace를 빌드한다.

```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
./scripts/build_ws.sh -j4
source devel/setup.bash
cd src/lidar+sensorfusion
catkin_make -j4 -DCATKIN_WHITELIST_PACKAGES='Main;Msg;morai_msgs'
```

카메라 터미널:

```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=localhost
rosparam set /use_sim_time false
roslaunch asmc_camera camera_objects.launch
```

LiDAR 터미널:

```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source src/lidar+sensorfusion/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=localhost
roslaunch Main main.launch start_yolo:=false rviz:=false
```

차선과 객체를 함께 켜려면 카메라 터미널의 launch를 다음으로 선택한다.

```bash
roslaunch asmc_camera perception.launch start_camera_objects:=true
```

객체만 켜는 상위 launch는 `start_camera_lane:=false start_camera_objects:=true`다.
`camera_objects_config`로 설정을 지정할 수 있다. RTX50 이미지에서는 기존
`ASMC_YOLO_PYTHON` 환경 변수를 launch-prefix로 사용한다. 노드가 실패해도 차선 launch는 계속된다.
GUI와 bridge는 이 launch가 열지 않는다.

같은 ROS 환경을 읽은 GUI 터미널에서 기존 fusion overlay를 연다.

```bash
rqt_image_view --clear-config /fusion/debug_image
```

초록은 YOLO, 빨강은 LiDAR 투영, 청록은 fusion 매칭이다. 이 화면에는 Main이 필요하다.
카메라 객체 launch만 실행하면 객체 메시지와 진단만 발행하고 별도 overlay는 만들지 않는다.
차선·객체·신호등을 함께 확인할 때는 기존 객체 단독 launch를 종료하고 다음을 실행한다.

```bash
roslaunch asmc_camera perception.launch start_camera_objects:=true start_visualization:=true start_traffic_light:=true traffic_light_wait_first:=0
```

영상 publisher가 등록된 뒤 각 GUI 터미널에서 한 창씩 직접 실행한다.

```bash
rqt_image_view --clear-config /perception/camera/debug/mosaic/compressed
rqt_image_view --clear-config /perception/bev/debug/comparison/compressed
rqt_image_view --clear-config /perception/traffic_light/debug/image/compressed
```

`start_visualization`은 debug publisher만 켠다. 세 모델 동시 화면 확인과 객체 단독 18Hz
수락 측정은 별도 실행이다. 모든 터미널에서 같은 master를 사용하고, 구·신 생산자를 동시에
켜지 않는다.

객체 단독 수락 검사와 운영 상태:

```bash
python3 scripts/verify_perception_runtime.py --camera-objects --no-camera-lane
rostopic hz /perception/camera/objects
rostopic echo -n 1 /perception/camera/objects/diagnostics
rostopic info /YOLO/BBox
```

`/YOLO/BBox` 발행자는 한 개여야 한다. 새 경로와 기본 `Main start_yolo:=true`를 동시에 켜면
중복 경고가 나온다. 롤백은 새 카메라 생산자를 종료한 뒤 Main을 `start_yolo:=true`로 다시 실행한다.

## 메시지와 모델

`class_name`은 `person/vehicle/obstacle`, `source_class_name`은 원래 라벨이다.
현재 실행의 허용 이름은 `bus/truck/person/car`, confidence는 0.7이다.
좌표는 원본 영상 픽셀의 소수 xyxy이며 호환 어댑터에서 기존과 같이 `int()`로 자른다.
source header, 순서, 원래 라벨을 보존하고 정상 빈 검출도 전달한다.
디코딩·추론 실패는 `valid=false` 빈 배열이며 어댑터가 전달하지 않는다.
유효한 전방 header와 0이 아닌 source stamp가 필요하다.

YOLO11m SHA는 `d5ffc1a674953a08e11a8d21e022781b1b23a19b730afc309290bd9fb5305b95`다.
두 원본 weight는 보존한다. CUDA 실제 연산·모델 device·예열을 확인한 뒤 입력을 구독한다.
CPU는 `device: cpu`, `require_cuda: false`를 설정 파일에 명시해야 한다.
생성 메시지가 오래됐으면 생산자·어댑터가 시작에 실패하므로 두 workspace를 위 순서로 재빌드한다.

진단은 처리율, 단계별 시간, source-to-output p50/p95, 최신 source stamp, 대체 프레임과 오류 수,
입력 중단, 중복 생산자, 모델 SHA·device·버전을 보고한다. 18Hz 조건은 객체 단독·20Hz 입력의
수락 기준이며 3뷰 TwinLite·신호등 동시 실행이나 4070 SUPER 성능을 보증하지 않는다.

## 재현 도구와 증거

현재 **구현 완료, 수락 보류**다. 3,514프레임 정확 비교, 구·신 1배속 A/B,
객체 단독 runtime 검사, 음영 fusion과 장애 주입은 통과했다. 주 fusion bag의
절대 매칭 수는 초기 구 468/468 대비 신 464/463, 추가 진단 구 465/469 대비 신 466/462로
조건을 만족하지 못했다. 같은 source stamp·3D 중심으로 대응한 실제 객체의 분류 전환도
신 경로에서 1~2프레임 달랐다. 실행 사이 track ID 재배정이 많으므로 같은 숫자 ID만으로
객체 동등성을 단정하지 않는다. 실패 실행도 [증거](../../../config/baselines/perception_camera_objects_parity_v1.yaml)에 모두 보존한다.
기존 C++ fusion·50ms/200ms 정책을 변경하거나 이 결과를 수락 완료로 표시하지 않는다.
이 문제를 해결하기 전 다음 단계 승격과 최종 병합은 보류한다.

검증은 GPU 전달·`--network none` 전용 컨테이너에서 직렬로 실행한다. 바깥과 nested 환경을
위 순서로 source한다. 다른 실시간 GPU 추론은 중지된 상태여야 한다.
bag 재생에는 별도 master에서 `/use_sim_time=true`와 `--clock`을 사용한다.

```bash
python3 tools/perception/compare_camera_objects.py /data/rosbags/hd_map_validation/20260921_174512_kcity2025_manual_route_hdmap/20260921_174512_kcity2025_manual_route_hdmap_0.bag --output /tmp/p2-offline.json
python3 tools/perception/replay_camera_objects.py /data/rosbags/hd_map_validation/20260921_174512_kcity2025_manual_route_hdmap/20260921_174512_kcity2025_manual_route_hdmap_0.bag --output /tmp/p2-ab
python3 tools/perception/replay_camera_objects.py /data/rosbags/raw/lidar_manual_20260929_130623/raw_0.bag --fusion --output /tmp/p2-fusion-primary
python3 tools/perception/analyze_fusion_identity.py /tmp/p2-fusion-primary
python3 tools/perception/check_camera_object_faults.py --output /tmp/p2-faults.json
```

정확 비교는 원본 `SpeedSignDetection` 메서드를 그대로 실행하고 ROS 발행/스케줄만 대체한다.
새 결과는 실제 생성 메시지로 직렬화·역직렬화한 뒤 어댑터를 통과한다.
confidence `0.7±0.005`에서 포함·제외 차이만 예외로 나열하며 좌표·순서 변화는 허용하지 않는다.
재생은 구·신 두 번씩 실행하고 처리율·커버리지·중복/역행 stamp·단일 발행자·지연을 기록한다.
fusion은 두 구 실행의 변동폭, 5초 매칭 로그와 공통 LiDAR stamp·track 분류를 비교한다.
마지막 5초 미만 구간은 기존 로그가 집계하지 않으므로 보고 매칭 수와 전체 처리 프레임을 구분한다.

09-21 bag에는 Ego가 없어 fusion 주 검증에 쓰지 않는다. 음영 보완은 LiDAR가 들어 있는
`20260912_003803_shadow_static_avoid` bag이다. LiDAR가 없는 09-13 bag은 fusion에서 제외한다.
증거 정본은 [parity YAML](../../../config/baselines/perception_camera_objects_parity_v1.yaml)이다.

## 후속 단계

[전체 카메라 계획](camera-runtime-plan.md)의 구간별 준비·활성화, GPS 복귀, LiDAR 처리 10Hz +
YOLO 10Hz 비교, 측방 YOLO, jamming 공통 생산자 연결, YOLO26, 공유 디코딩과 모델별 TensorRT는
각 단계에서 독립 검증한다. 이번 이전에서 주기·precision·추론 헤드·fusion 정책은 바꾸지 않는다.
