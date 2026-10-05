# BBOX가 내 차로 / 옆 차로에 있는지 판정

거리 추정 없이 기존 전방 YOLO BBOX와 camera boundary family를 사용한다.
`src/camera_bbox_lane_node.cpp`에 인자 기본값·주석·main·판정 함수를 둔다.
이 노드는 관측/표시 전용이다. 자동 차선변경과 `/ctrl_cmd` 발행은 하지 않는다.
기존 LiDAR, YOLO, 카메라 차선 모듈의 소스는 수정하지 않는다.

## 처리

1. 기존 YOLO가 원본 영상 stamp를 보존한 BBOX를 발행한다.
2. `camera_bbox_frame_bridge.py`가 **같은 stamp의 원본 영상**과 BBOX를 묶는다.
   영상 없음, frame_id 불일치, 객체 수 불일치는 invalid다.
3. C++ 노드는 기존 `BuildBoundaryFamilyStripContext`로 차선을 검증한다.
   BBOX와 가장 가까운 차선 stamp를 사용하되 source age/시각 차이/계약 hash를 검사한다.
4. `cameras_v2.yaml`의 지면 보정으로 유효 차선 띠만 원본 영상에 투영한다.
   차선 단절 구간이나 관측 끝을 이어 붙이거나 연장하지 않는다.
5. BBOX 하단 위 8px~아래 8px의 접촉 검사 영역을 실제 차로 폴리곤과 비교한다.
   두 차로가 동시에 관측되는 row를 최소 3개 확보하고, 각 row의 겹침 폭을 합산한다.
   차로별 비율은 겹침 면적 / (BBOX 폭 × 유효 row 수)이며 기본 임계값은 각각 10%다.
   두 차로 모두 임계값 이상이면 BOTH, 한쪽만 넘으면 OWN/ADJACENT다.
   소량의 겹침만 있으면 UNKNOWN이고, 관측된 영역과 겹침이 전혀 없으면 OUTSIDE다.
   화면 좌우로 내 차로를 고정하지 않는다.
6. 내 차로는 기본적으로 전방 4m의 차선 띠 중 y=0이 포함된 띠를 서로 다른 차선
   frame 3개에서 확인한다. 차선을 걸친 상태는 UNKNOWN이며 `lane_index=0/1`로 고정 가능하다.

평평한 직선 노면, 물체 하단이 실제 접점에 가까운 경우를 전제로 한다.
화면 하단에서 잘린 물체와 중간 차선 gap은 UNKNOWN이다.
두 차로가 모두 12m 이상 관측되고 BBOX 하단이 관측 끝보다 위쪽, 지면 소실선보다
5px 이상 아래쪽이면 `DEFERRED_FAR`로 분리한다. 해당 물체의 차로와 거리는 추정하지 않는다.
기본 `far_boundary_margin_px=0`으로 관측 범위에 진입하기 직전의 정지 띠를 피한다.
이 조건을 만족하지 않는 차선 관측 부족은 UNKNOWN이다.
다른 물체에 가려진 하단, 부정확한 BBOX, 차체 pitch 변화는 BBOX만으로 항상 감지할 수 없다.
따라서 판정 결과를 확정된 물리 점유나 충돌 회피 가능 여부로 해석하지 않는다.

## 토픽과 값

| 토픽 | 역할 |
|---|---|
| `/planning/camera_bbox_lane/yolo` | 기존 `Msg/Yolo_bbox_arr`, 별도 detector 출력 |
| `/planning/camera_bbox_lane/frame` | 정확한 원본 영상+BBOX 내부 연결 |
| `/planning/camera_bbox_lane/result` | `lane_boundary_adapter/CameraBBoxLaneResult` |
| `/planning/camera_bbox_lane/image` | RViz용 원본 영상+차선+BBOX+라벨 |
| `/planning/camera_bbox_lane/diagnostics` | 입력 상태 / 판정 상태 |

`assignments`는 입력 `boxes`와 같은 순서다. 객체 추적 ID는 아니다.

- `0 UNKNOWN`: 판정 불가
- `1 OWN`: 내 차로
- `2 ADJACENT`: 옆 차로
- `3 BOTH`: 양쪽에 걸침
- `4 OUTSIDE`: 관측된 두 차로 바깥
- `5 DEFERRED_FAR`: 관측 범위보다 먼 BBOX, 차로 미확정 (영상 라벨 FAR_UNRESOLVED)

`own_state`, `adjacent_state`는 `0 UNOBSERVED`, `1 NO_DETECTION`, `2 DETECTED`다.
**NO_DETECTION은 검출 없음이며 빈 차로가 보장된다는 뜻이 아니다.**
UNKNOWN BBOX가 있으면 `valid=false`이며 확인된 DETECTED는 유지한다.
DEFERRED_FAR만 미확정이면 `valid=true`, `reason=far_boxes_deferred`로 전달하지만
미검출 차로는 NO_DETECTION 대신 UNOBSERVED다. `valid`만으로 빈 차로라고 해석하면 안 된다.
새 메시지 상수가 추가되어 재빌드 후 classifier와 controller를 함께 재시작해야 한다.
입력 끊김/중복 stamp 지속은 UNKNOWN으로 바뀌며 마지막 관측을 새 시각으로 갱신하지 않는다.
RViz에도 timeout/invalid 문구를 표시한다.

기존 YOLO는 bus/truck/person/car만 통과시킨다. 콘·박스·바리케이드 검출은 이 설정으로
보장하지 않는다. 차량 BBOX를 구분하는 기능과 임의 물체를 검출하는 기능은 별도다.

## 준비와 실행

RTX 5080 환경에서 기존 ROS 컨테이너의 cu118 대신 `asmc-perception-train`의 cu128 torch를
사용한다. YOLO 추가 의존성은 `/opt/camera-bbox-deps`에만 설치한다.
기존 차선 추론용 torch/NumPy/OpenCV는 바꾸지 않는다. 현재 컨테이너 설치는 완료했다.
컨테이너를 새로 만들었을 때만 다음을 실행한다.

```bash
docker exec -it asmc-perception-train bash -lc 'python3 -m pip install --no-deps --target /opt/camera-bbox-deps -r /root/ws/src/jamming_lane_boundary_adapter/requirements-camera-bbox.txt'
```

루트 패키지 빌드 (현재 완료):

```bash
docker exec -it asmc-ros-noetic bash -lc 'source /opt/ros/noetic/setup.bash && source /root/ws/devel/setup.bash && cd /root/ws && catkin_make --pkg lane_boundary_adapter -j2'
```

브리지와 기존 카메라 차선 인식은 켜둔다. 자동 주행을 시험하지 않을 때는 기존
camera follower/avoidance 노드도 종료한다. 아래 명령은 기존 주행 노드를 중지시키지 않는다.
LiDAR Main은 필요 없다. 중첩 workspace를 source하는 이유는 기존 YOLO와 Msg를 읽기 위해서다.

터미널 1: GPU YOLO만 실행. 기존 fusion의 `/YOLO/BBox`와 분리된 토픽을 사용한다.

```bash
docker exec -it asmc-perception-train bash -lc '
source /opt/ros/noetic/setup.bash &&
source /root/ws/devel/setup.bash &&
source /root/ws/src/lidar+sensorfusion/devel/setup.bash &&
roslaunch lane_boundary_adapter camera_bbox_detector.launch
'
```

터미널 2: 차로 판정과 RViz.

```bash
docker exec -it -e DISPLAY="$DISPLAY" asmc-ros-noetic bash -lc '
source /opt/ros/noetic/setup.bash &&
source /root/ws/devel/setup.bash &&
source /root/ws/src/lidar+sensorfusion/devel/setup.bash &&
roslaunch lane_boundary_adapter camera_bbox_lane.launch use_rviz:=true
'
```

RViz `BBOX Lane Classification`에서 MY_LANE/ADJACENT/BOTH/OUTSIDE/UNKNOWN 라벨을 본다.
MY_LANE·BOTH는 빨간색, ADJACENT는 파란색, 나머지는 노란색이다.
차선은 내 차로 녹색, 옆 차로 파란색으로 표시한다.

`boundary_margin_px=3`은 경계 걸침 판정 여유다. `max_sensor_skew=0.20`,
`source_timeout=0.5`는 차선/BBOX 시각 허용 범위다. 변경은 launch 인자로 지정할 수 있다.

## 검증

별도 ROS master에서 실제 `Msg/Yolo_bbox_arr` → 영상 동기화 → C++ 판정 연결을 검사한다.

```bash
docker exec -it asmc-ros-noetic bash -lc 'source /opt/ros/noetic/setup.bash && source /root/ws/devel/setup.bash && source /root/ws/src/lidar+sensorfusion/devel/setup.bash && rostest lane_boundary_adapter camera_bbox_lane.test'
```

테스트: 내/옆/양쪽/바깥 차로, 자차 차로 전환, 차선 단절, 관측 범위 밖, 잘린 BBOX,
NaN, 중복/오래된 입력, 차선·영상 미수신, 보정 해상도 불일치, 객체 수 불일치.
`/ctrl_cmd` publisher가 없음을 검사한다. 실제 동굴 검출 정확도나 회피 주행을 검증한 것은 아니다.

검증 결과: ROS 빌드 및 차로 판정 통합 테스트 7개 통과. RTX 5080의 독립 ROS master에서
YOLO 단독 launch로 합성 영상 12 frame의 GPU 추론과 원본 stamp 보존을 확인했다.

## 차선변경 주행

이 관측 결과로 1회 차선변경하는 별도 노드는 [CAMERA_BBOX_LANE_CHANGE.md](CAMERA_BBOX_LANE_CHANGE.md)를 참고한다.
주행용 launch에서는 기준 차로 topic을 연결해 중앙선 통과 중에도 같은 index를 사용한다.


## YOLO 보닛 제외

현재 전방 1280×720 프레임에서 자차 보닛이 y≈578부터 보이며 YOLO가 이를 car로 검출했다.
`camera_bbox_crop.py`가 주행 전용 YOLO 입력의 아래 21%를 제외해 1280×568로 전달한다.
resize/상단 이동 없이 원본 좌표와 header를 유지하므로 기존 지면 투영에 그대로 대응한다.
차선 추론 입력과 LiDAR 공용 YOLO 구현은 변경하지 않는다.

- `hood_crop_bottom_ratio=0.21`: 아래쪽 제외 비율. 0이면 비활성화, 허용 범위 [0, 0.5).
- 이 값을 바꿀 때 **detector launch와 lane change/classifier launch에 동일하게 지정**한다.
- `/planning/camera_bbox_lane/detector_input/compressed`: YOLO에 실제 들어가는 절단 영상.
- 기존 RViz BBOX 영상은 원본 크기를 유지하며 아래 제외 영역을 회색과 `YOLO EXCLUDED`로 표시한다.
- 절단선에 닿거나 아래에 있는 BBOX는 `bottom_contact_clipped_by_hood_crop`로 UNKNOWN 처리한다.
  잘린 물체의 BBOX 하단을 실제 도로 접점으로 해석하지 않는다.

실제 저장 프레임 비교: 원본 보닛 car confidence 0.737 → 절단 후 해당 검출 없음.
독립 ROS 검증: 1280×568 입력, 원본 시각/좌표 유지, 일반 BBOX 판정,
절단선/원본 보닛 BBOX UNKNOWN 처리 통과. 로그: `/tmp/hood-crop-check-eoyj323k` (asmc-ros-noetic).


## 접촉 겹침 판정과 표시

기본 인자(관측/주행 launch 모두 동일 이름):

- `contact_band_height_px=8`: BBOX 하단 위쪽 접촉 검사 높이(px).
- `contact_row_tolerance_px=8`: 아래쪽 접점 오차 허용(px). 최대 16.
- `min_contact_overlap_ratio=0.10`: 각 차로에 속한다고 판단할 최소 겹침 비율.
- `min_contact_support_rows=3`: 두 차로가 함께 관측되어야 하는 최소 row 수.

BBOX 하단 row의 관측이 없으면 두 차로 관측 끝 부근에서만 작은 픽셀 오차를 허용한다.
중간 INVALID gap은 메우지 않으며, 실제 투영된 폴리곤 밖으로 경계를 연장하지 않는다.
하단 접점이 보닛 제외선 또는 영상 끝에 잘렸으면 비율을 근거로 차로를 단정하지 않는다.

RViz의 `/planning/camera_bbox_lane/image`에서 초록 영역=내 차로, 자홍 영역=옆 차로,
노란 사각형=BBOX 하단 검사 영역, 그 안의 초록/자홍 색=실제 겹친 부분을 표시한다.
`OWN nn% / ADJ nn%`는 차로별 접촉 겹침 비율이다. 거리 또는 충돌 확률이 아니다.
잘린 접점은 `CONTACT CLIPPED`, 나머지 판정 불가는 `CONTACT UNRESOLVED`로 표시한다.
검증: 차로 판정 10개 + 차선 변경 통합 15개 통과.
로그(asmc-ros-noetic): `/tmp/bbox-contact-regression-ehqvp_pi`.


## 잘린 기존 BBOX 연속성

`CameraBBoxLaneResult.clipped_association`는 boxes와 같은 길이의 bool 배열이다.
정상 접점으로 확인된 기존 물체와 유일하게 연결되고, 현재 보이는 하단 영역도 같은 기하 차로를
지지하는 경우만 제한적으로 OWN/ADJ를 유지한다. 세부 조건과 제어 속도 제한은
CAMERA_BBOX_LANE_CHANGE.md의 보닛 절단 연속 추적 절을 참고한다.
RViz에는 `TRACKED CLIPPED (LIMITED)`로 표시한다. 처음부터 잘렸거나 추적 조건을 만족하지
못한 박스는 기존처럼 CONTACT CLIPPED/UNKNOWN이다.
목표 차로가 바뀌면 영상 영역 색도 목표=초록, 다른 차로=자홍으로 전환한다.
