# 카메라 객체 생산자 P2a 사용·검증

**과거 P2a 기록:** 아래 YOLO11m·구형 Main 실행법은 당시 검증 재현용이다. 현재 실행은 [LiDAR 카메라 인계](../../handoffs/camera-to-lidar.md)를 따른다. 독립 Main YOLO는 현재 checkout에서 제거됐으며 과거 소스/가중치는 커밋 ae8dcbc0에서 확인한다.


> 기준: main `af253c3575c1e704e70511c2a2033ec15e33299a`, 전방 YOLO11m 동작 보존.
> opt-in 구현이며 LiDAR 전체 이전이나 대회 full-stack 성능 승인을 뜻하지 않는다.
> 최신 통합 기준은 컨트롤러 PR #67을 포함한 main `c5d0b78`이다. 검출·LiDAR·HD맵 실행 코드는
> 기존 측정 기준과 동일하며 최신 main에서 전체 빌드와 카메라·메시지 회귀를 다시 확인했다.

`/image_jpeg/compressed` → `asmc_camera` → `/perception/camera/objects` →
`Main` 호환 어댑터 → `/YOLO/BBox` → 기존 fusion으로 연결한다.
LiDAR 처리 주기, 50ms 관측 stamp 매칭과 200ms 도착 대기는 유지한다.
기본 `Main` 실행은 기존 YOLO를 사용하고 새 생산자는 기본 비활성화다.

## 최종 수락 결정 — 2026-10-07

사용자가 승인한 병합 계획에 따라 **전방 YOLO11m opt-in 이전을 수락**한다.
엄격한 자동 비교 결과와 과거 실패는 수정하지 않는다. 최신 주 bag의 안정 관측
3,206개 중 stamp `1790654820956000000`의 `unknown/car` 차이 한 건을 명시적으로
허용한다. 원인은 하드웨어로 확정하지 않았다. 별도 trace의 51ms 사례를 이 관측의
직접 원인 증명으로 사용하지 않는다.

수락 근거는 3,514프레임 정확 검출 동등성, 연속 상태 전달 재현, 최신 구/신 반복
재생의 매칭·offset, 객체 단독 처리율과 장애·회귀 검사다. 이전 음영 `177 < 181`
실패와 모든 실패 실행도 보존한다. 이번 결정은 모든 실행의 엄격한 기준 통과나
전체 대회 주행 배포 승인에 해당하지 않는다. 아래 보류/Draft 설명은 당시 검증 이력이다.

Claude 미커밋 10개는 현재 구현에 흡수됐다. 경로·SHA·흡수 커밋은
`config/baselines/perception_claude_work_absorption_v1.json`에 기록한다.

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
roslaunch asmc_camera perception.launch profile:=debug start_camera_lane:=false start_traffic_light:=false
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
roslaunch asmc_camera perception.launch profile:=debug start_traffic_light:=false
```

객체만 켜는 상위 launch는 `profile:=debug start_camera_lane:=false start_traffic_light:=false`다.
`camera_objects_config`로 설정을 지정할 수 있다. RTX50 이미지에서는 기존
`ASMC_YOLO_PYTHON` 환경 변수를 launch-prefix로 사용한다. 노드가 실패해도 차선 launch는 계속된다.
GUI와 bridge는 이 launch가 열지 않는다.

같은 ROS 환경을 읽은 GUI 터미널에서 기존 fusion overlay를 연다.

```bash
rqt_image_view --clear-config /fusion/debug_image
```

초록은 YOLO, 빨강은 LiDAR 투영, 청록은 fusion 매칭이다. 이 화면에는 Main이 필요하다.
현재 debug 프로필은 LiDAR/GPS 없이 YOLO 전용 overlay도 제공한다.

```bash
rqt_image_view --clear-config /perception/camera/objects/debug/image/compressed
```

개별 `camera_objects.launch`는 기본 STANDBY이므로 일반 확인은 상위 debug 프로필을 사용한다.
차선·객체·신호등을 함께 확인할 때는 기존 객체 단독 launch를 종료하고 다음을 실행한다.

```bash
roslaunch asmc_camera perception.launch profile:=debug
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

현재 **구현 완료, 수락 보류**다. 초기 검증의 3,514프레임 정확 비교, 구·신 1배속 A/B,
객체 단독 runtime 검사, 음영 fusion과 장애 주입은 통과했다. 주 fusion bag의
절대 매칭 수는 초기 구 468/468 대비 신 464/463, 추가 진단 구 465/469 대비 신 466/462로
조건을 만족하지 못했다. 같은 source stamp·3D 중심으로 대응한 실제 객체의 분류 전환도
신 경로에서 1~2프레임 달랐다. 실행 사이 track ID 재배정이 많으므로 같은 숫자 ID만으로
객체 동등성을 단정하지 않는다. 실패 실행도 [증거](../../../config/baselines/perception_camera_objects_parity_v1.yaml)에 모두 보존한다.
기존 C++ fusion·50ms/200ms 정책을 변경하거나 이 결과를 수락 완료로 표시하지 않는다.
이 문제를 해결하기 전 다음 단계 승격과 최종 병합은 보류한다.

후속 진단에서는 기본 비활성인 `Main fusion_trace_path`로 카메라·YOLO·LiDAR 수신,
처리 generation 누락, 선택된 stamp·대기·제거 사유, 분류 후보·확인·miss 카운터를
버퍼링한 JSONL로 기록한다. 정상 종료는 `Fusion sync tail`로 마지막 부분 구간의 수와
offset을 출력하고 `Fusion sync total`로 전체 매칭·누락·남은 큐를 보고한다. 종료 뒤에는
rosconsole이 닫혀 있으므로 두 종료 기록은 stdout으로 직접 flush한다. 검증기는 기존
5초 로그와 tail의 합이 전체 누적 수와 정확히 일치하는지도 요구한다. 마지막 부분 구간을
제외한 합계는 실행마다 집계 범위가 달라져 구·신 비교에 사용할 수 없다.
새 경로의 전체 매칭 수 ≥ 구 경로 두 실행의 최솟값, 모든 구간 offset p95 ≤ 50ms 조건을 유지한다.
이 관측 기능은 매칭·분류 정책을 바꾸지 않는다.
같은 관측을 55ms/12ms 지연으로 전달하는 CPU 재현 시험은 실제 matcher·class policy의
결과 동등성을 확인하지만, 실제 bag의 서로 다른 처리 프레임 집합을 수락한 증거는 아니다.
후속 환경·검증 상태는 [별도 기록](../../../config/baselines/perception_camera_objects_followup_v1.yaml)에 있다.

2026-10-07 사용자가 재연결한 컨테이너에서 직접 실시간 확인을 완료했다. 후속 재생에서
종료 뒤 rosconsole 기록 소실과 마지막 부분 구간 집계 누락을 확인해 수정했다. 추적 비활성
주 bag 전체 매칭은 구 `503/496`, 신 `503/500`, 신 offset p95는 `31.6/29.0ms`로
매칭 조건을 만족했다. 분류는 상호 유일한 동일 관측 2,954개 중 한 프레임이 달랐다.
음영 bag은 구 `181/181`, 신 `177/187`로 첫 신 실행의 절대 매칭 수가 부족했다.
분류 차이와 신 경로 timeout은 없었지만 처리한 LiDAR 프레임 집합이 달라졌으므로 수락은 보류한다.

추적 실행의 동일 source 프레임 936개에서 원래 라벨·개수·순서·정수 좌표·source stamp·frame_id 차이는
없었다. 분류 차이 한 건은 같은 LiDAR·YOLO 쌍의 `class_before`와 `class_after`가 모두
동일했으나 구 경로의 현재 LiDAR 출력에 51ms 이전 분류 상태가 남고 신 경로에는 새
상태가 반영된 사례다. 다른 차이에서는 같은 관측에 도달하기 전 확인·miss 카운터나
track 이력이 달랐다. Main의 latest-frame 선택으로 구 경로 자체도 LiDAR 649개 중
138/142 generation을 건너뛰었다. 매칭 대기 추가나 분류 임계값 변경으로 해결됐다고
간주하지 않으며, 숫자 ID 재배정과 실제 분류 차이를 구분한 보고서를 함께 보존한다.

다음 수정의 설계에서는 geometry의 source stamp와 비동기 분류 상태가 어느 관측을 의미하는지,
부하로 선택된 LiDAR 집합이 달라도 무엇을 동등하다고 볼지 먼저 확정해야 한다. 동일한
관측 집합의 결정적 재현과 실제 재생의 처리 부하·미처리 프레임 평가를 구분한다.
현재의 엄격한 runtime 분류 gate는 계속 유지한다. 발행 대기·처리율 제한·LiDAR 큐 정책·
분류 임계값·수락 범위의 변경은 별도 계획이며 이 PR에서 임의로 적용하지 않는다.

검증은 GPU 전달·`--network none` 전용 컨테이너에서 직렬로 실행한다. 바깥과 nested 환경을
위 순서로 source한다. 다른 실시간 GPU 추론은 중지된 상태여야 한다.
bag 재생에는 별도 master에서 `/use_sim_time=true`와 `--clock`을 사용한다.

```bash
python3 tools/perception/compare_camera_objects.py /data/rosbags/hd_map_validation/20260921_174512_kcity2025_manual_route_hdmap/20260921_174512_kcity2025_manual_route_hdmap_0.bag --output /tmp/p2-offline.json
python3 tools/perception/replay_camera_objects.py /data/rosbags/hd_map_validation/20260921_174512_kcity2025_manual_route_hdmap/20260921_174512_kcity2025_manual_route_hdmap_0.bag --output /tmp/p2-ab
python3 tools/perception/replay_camera_objects.py /data/rosbags/raw/lidar_manual_20260929_130623/raw_0.bag --fusion --output /tmp/p2-fusion-primary
python3 tools/perception/analyze_fusion_identity.py /tmp/p2-fusion-primary
python3 tools/perception/replay_camera_objects.py /data/rosbags/raw/lidar_manual_20260929_130623/raw_0.bag --fusion --fusion-trace --output /tmp/p2-fusion-trace
python3 tools/perception/analyze_fusion_trace.py /tmp/p2-fusion-trace
rosrun Main fusion_class_trace_replay /tmp/p2-fusion-trace/old_1.fusion.jsonl 5 3 3
python3 tools/perception/check_camera_object_faults.py --output /tmp/p2-faults.json
```

`fusion_class_trace_replay`는 테스트 빌드에 포함된 CPU 도구다. 기록한 관측·miss를 실제
C++ class policy에 다시 넣어 각 객체의 분류·원래 라벨·후보·확인·miss 상태와 완전한
집계를 검사한다. 인자 `5 3 3`은 Main의 최초 확인·전환 확인·miss grace 값이다.
이 도구는 기록한 `class_before`를 입력으로 사용하며 기하 대응이나 실행 사이 tracking
이력을 다시 계산하지 않는다. 따라서 통과 결과로 실제 재생의 분류 차이를 수락하지 않는다.

### 연속 상태 진단

추가 진단은 `class_track_birth`와 명시적인 LiDAR 지면 노이즈 복구
`class_tracking_reset`, fusion 직전 `class_lidar_ready`, 발행에 넘기는
`class_publish_input`을 연결한다. 객체 ID·내부 중심 `f32_CoreX/Y/Z`, 원본 LiDAR stamp,
입력 generation, 분류·후보·확인·miss·lock 상태를 기록한다. 발행 입력에는 마지막으로
역사 분류 상태를 전달한 LiDAR·YOLO stamp와 적용한 현재 LiDAR stamp, 이번 처리 프레임에서
전달했는지가 포함된다. 상태 전달은 label이 그대로인 경우도 포함하며 새로운 관측을 뜻하지 않는다.
발행 입력 기록은 planner publisher의 필터 적용 전 상태다. 실제 발행 분류 비교는 기존
`TrackedObjectArray` 기록과 동일 stamp·객체 대응으로 별도 확인한다.

```bash
rosrun Main fusion_continuous_trace_replay /tmp/p2-fusion-trace/old_1.fusion.jsonl 5 3 3
```

새 도구는 최초 생성 상태와 명시적인 LiDAR 복구만 입력으로 받는다. 이후 분류는 이전
재현 결과에서 이어가며, `class_before`와 발행 입력은 정답 비교용이다. 실제 matcher,
분류 정책과 운영 코드가 함께 사용하는 분류 복사·이력 반영 함수를 사용한다. YOLO history의
admission·제거·고유 소비, 역사 객체에서 현재 객체로 전달한 상태와 발행 시점의 provenance까지
검사한다. 기록된 기하 관측이 없는 쌍, 빠진 객체 생성·발행·종료 기록은 재현 실패로 처리한다.
과거 형식의 trace에 새 입력을 추측해 넣지 않는다. 이 도구는 기하 대응 이후의 연속 상태를
검사하며 3D 투영·IoU 재계산이나 서로 다른 runtime 처리 프레임의 동등성을 증명하지 않는다.

합성 시험은 동일 관측을 다른 지연으로 전달하면 역사 분류 갱신은 같아도 현재 프레임에
발행되는 분류는 달라질 수 있음을 재현한다. LiDAR 관측 선택이 달라지면 확인 횟수도 달라진다.
이 결과를 실제 재생의 엄격한 분류·절대 매칭 수 gate에 대한 예외로 사용하지 않는다.

`roslaunch` 화면 전달에서 stdout/stderr가 섞여 5초 집계가 잘린 진단 실행도 보존한다.
새 검증기는 `/rosout`의 완전한 `/Main` `Fusion sync 5s` 메시지를 저장하고 종료 stdout의
tail·total과 정확히 대조한다. 빠지거나 중복된 interval은 여전히 실패다. 이 수집 변경은
검증 도구만 사용하며 운영 메시지·토픽·시간 매칭·분류 정책은 바꾸지 않는다.

2026-10-07 추가 trace 8개는 연속 재현이 모두 통과했다. 역사 분류 상태 112,923개와
각 프레임의 추적 carry·현재 객체 전달·발행 입력을 검사했다. 주 bag의 문제 stamp
`1790654794688000000`에서 구 경로의 발행 입력은 역사 LiDAR
`1790654794637000000`/YOLO `1790654794654529333`의 `unknown`·확인 4회 상태였고,
신 경로는 현재 LiDAR/YOLO `1790654794677862167`의 확정 `car` 상태였다.
두 경로 모두 마지막 계산을 현재 객체로 정확히 전달했지만 현재 관측의 완료 시점이 달랐다.
이는 상태 복사 누락의 증거가 아니라 발행에 실리는 관측 시점 차이이며 기존 분류 gate에서는 실패다.

음영 trace의 구 처리·매칭·pending은 `183/182/1`, `186/185/1`, 신은
`186/186/0`, `184/184/0`이었다. 모든 실행의 drop은 0이다. 처리한 관측의 수와 원본
stamp 집합이 달랐고, 매칭 수 차이는 처리 공급량·남은 큐 차이로 정확히 분해된다.
이 trace 실행의 통과로 과거 `177 < 181` 실패를 지우지 않는다. 과거 실패의 관측 출력은
178프레임, 매칭 177, pending 1로 일치하지만 그 실행에는 원본 처리 trace가 없으므로
누락된 generation의 정확한 목록이나 개별 처리 지연의 원인을 소급해 증명하지 않는다.

추적 비활성 최종 수락 측정에서 주 bag 전체 매칭은 구 `503/504`, 신 `510/512`이고
신 offset p95는 `34.4/34.0ms`였다. 하지만 안정된 동일 기하 관측 3,206개 중 한 건
(`1790654820956000000`)에서 신 두 번째 실행의 분류가 달라 수락은 계속 보류다.
이 추적 비활성 실행의 개별 상태 흐름을 trace 실행의 다른 stamp 설명으로 대체하지 않는다.
음영 bag은 구 `187/185`, 신 `186/188`, 신 p95 `24.8/24.8ms`로 통과했다.
객체 단독 10초 구간 두 실행은 약 `20.21/20.10Hz`, 중복·역행 stamp 0이며 runtime 검사와
실제 GPU 장애 7개가 통과했다. 이 짧은 검사와 이전 3,514프레임 전체 A/B 증거는 구분한다.
카메라 backend·생산자·어댑터·메시지·weight는 이번 추가 작업에서 변경하지 않았으므로
이전 정확 비교와 전체 객체 단독 A/B는 그대로 보존했다.

전체 바깥/nested 빌드, ROS-only 새 workspace의 생성 메시지 빌드·연속 재현,
Main 267개, 메시지 1개, jamming 94개, 계약·진단 48개와 validator 4개가 통과했다.
카메라 223개 중 기존 facade의 `traffic_light` 기본값 실패 한 건은 같은 실패로 남았고
새 실패는 없었다. trace·비활성 수락·음성 대조·실패한 관측 기록은 기존 archive를 바꾸지 않고
`continuous_reports.tar.gz`에 추가 보존한다. PR #68은 Draft이며 시간/분류 정책을 변경한
후속 수정이 필요하면 별도 계획으로 확정한다.



정확 비교는 원본 `SpeedSignDetection` 메서드를 그대로 실행하고 ROS 발행/스케줄만 대체한다.
새 결과는 실제 생성 메시지로 직렬화·역직렬화한 뒤 어댑터를 통과한다.
confidence `0.7±0.005`에서 포함·제외 차이만 예외로 나열하며 좌표·순서 변화는 허용하지 않는다.
재생은 구·신 두 번씩 실행하고 처리율·커버리지·중복/역행 stamp·단일 발행자·지연을 기록한다.
fusion은 두 구 실행의 변동폭, 5초 매칭 로그와 공통 LiDAR stamp·track 분류를 비교한다.
기존 실패 기록의 5초 로그 합계와 전체 처리 프레임은 구분한다. 수정 뒤에는 5초 로그·마지막
tail·전체 누적 수를 함께 보존하고 누락된 집계는 실패로 처리한다. 기존 track gate와 상호 유일한
동일 관측의 구 경로 안정 분류 비교도 유지한다. 대응되지 않은 관측이나 모호한 중심은 성공으로 간주하지 않는다.
원인 추적 실행과 최종 추적 비활성 성능 실행을 구분하고, 사용자 실시간 확인 완료 전에는
자동 GPU bag 재생을 시작하지 않는다.

09-21 bag에는 Ego가 없어 fusion 주 검증에 쓰지 않는다. 음영 보완은 LiDAR가 들어 있는
`20260912_003803_shadow_static_avoid` bag이다. LiDAR가 없는 09-13 bag은 fusion에서 제외한다.
증거 정본은 [parity YAML](../../../config/baselines/perception_camera_objects_parity_v1.yaml)이다.

## 후속 단계

[전체 카메라 계획](camera-runtime-plan.md)의 구간별 준비·활성화, GPS 복귀, LiDAR 처리 10Hz +
YOLO 10Hz 비교, 측방 YOLO, jamming 공통 생산자 연결, YOLO26, 공유 디코딩과 모델별 TensorRT는
각 단계에서 독립 검증한다. 이번 이전에서 주기·precision·추론 헤드·fusion 정책은 바꾸지 않는다.
