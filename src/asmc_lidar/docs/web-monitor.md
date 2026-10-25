# asmc_lidar 웹 모니터 — 처리 결과를 브라우저에서 보기 (읽기 전용)

`roslaunch asmc_lidar view.launch`는 이미 실행 중인 LiDAR의 결과를 구독하는 **읽기 전용 웹 모니터만** 띄운다.
처리는 별도 `roslaunch asmc_lidar lidar.launch debug:=true`에서 시작한다.
목적은 "내 asmc_lidar가 실제로 어떤 점을 남겼고 어떤 객체를 추적하는지"를 확인하는 것이다.
모니터는 ROS를 구독만 하고, 발행·서비스 호출·파라미터 변경·shell 실행을 하지 않는다.
판제는 계속 기존 `Main`(`/last_lidar`)을 쓴다. 이 launch는 Main·planner·카메라 추론·UDP bridge·Velodyne
driver를 시작하지 않으며, 기존 운영 토픽을 바꾸지 않는다. TunnelFilter는 기본 OFF다.

## 가장 짧은 실행

### 간소화 화면 / 3D / 베이크 지도

- `예측 트랙(pred) 표시`는 기본 OFF다. miss>0 객체의 박스·라벨·목록·선택만 숨기며
  추적기의 예측 계산이나 토픽은 바꾸지 않는다.

- 기본 숨김은 점군·높이 후보 ON, OBB 비교 후보 OFF다. `제외 후보 모두 숨기기 (화면만)`로 전체를 바꾼다. 점군 근거/OBB 근거/상부 높이 후보를
  개별로 켜고 끌 수 있으며 같은 객체가 여러 사유에 해당할 수 있다. OBB·라벨과 **정확히 소속이 확인된 표시 점**을 함께 숨긴다.
  **목록·원본 ROS 객체·tracker는 유지**한다. 목록에서 선택한 후보는 다시 그려 근거를 확인한다.
  보류·지도/pose 무효 객체는 숨기지 않는다. 중심점 하나의 도로 포함 여부로 삭제하지 않는다.
- `! ID 변경 의심`은 화면에서 관측한 교체 후보이며 동일 물체의 확정 식별이 아니다.
  유효 map pose/영역 안 객체에 대해 최근 0.8초의 map 중심(거리 2m 이하),
  OBB 크기비(1.8 이하), class 호환성을 비교한다. 기존 ID가 관측 중이거나 후보가
  0.5m 이내 점수 차로 모호하면 표시하지 않는다. 의심 배지는 3 scan초 동안 유지한다.
  GPS 무효, 세션 변경, scan 시각 역행, 1초 초과 공백은 이력을 초기화한다.
  5Hz 화면 관측 기반이므로 모든 ID 전환을 잡지 못하며 실제 새 객체를 혼동할 수도 있다.

- 좌표계는 Local(base_link) / Map만 선택한다. 점군은 Raw / ROI / RANSAC 이후 중 하나,
  최종 추적 OBB와 ID·class는 별도 ON/OFF다. 측면·전처리 상세는 필요할 때 펼치고,
  객체 목록과 선택 객체의 필터 근거는 기본으로 표시한다.
- `보기 → 화면 → 3D · 회전 보기`: 점군 XYZ와 OBB 8 corner를 직교 투영한다.
  좌클릭 드래그는 2D·3D 모두 회전, 가운데 버튼 드래그 또는 Shift+좌클릭 드래그는 이동,
  휠은 확대다. 차량 중심 버튼은 이동·회전도 초기화한다. 실시간 추론·필터링 설정은 바꾸지 않는다.
- 3D 자차는 RViz와 같은 길이·폭·축 위치와 형상 확인 높이1.644m를 사용한다.
  명목 지면 z=-0.36m, 지붕 z=1.284m는 `base_link` 기준이다.
- 기본 OBB 색은 **필터 미리보기**다: 초록 유지, 분홍 점군 후보, 보라 OBB 후보,
  주황 상부 후보, 노랑 보류. 소속 점도 같은 색이다. 객체에 연결되지 않은 점은 회청색이다.
  `제외점 다시 표시`의 거리/XY/센서Z/자차/상부 높이 항목은 체크하면 제거점을 다시 그린다.
  옆의 값은 해당 scan의 실제 적용 설정이며 웹 체크는 필터를 재실행하지 않는다.
  `운동 진단 (재호)`를 선택하면 기존 DYNAMIC/PENDING 빨강, LOW_THREAT 초록,
  UNKNOWN/invalid 노랑으로 바뀐다. 카메라 그룹도 별도로 선택할 수 있다.
  소비 그룹을 실제 운동 판정으로 해석하지 않는다.
- `motion_display`는 기존 Main `MotionClassifier`의 중앙값·급변 후보·변위 확인·상태 전이
  함수 본문을 유지한 transport-free 이식이다. 임계값은 현재 Main 기본값(x10/y4km/h,
  변위 x8/y3km/h)이며 별도 튜닝하지 않았다. map OBB bounds/벡터는 저장된 scan Context로
  계산하고 모든 처리 scan마다 갱신한다. GPS 무효 시 이력을 지워 UNKNOWN으로 복구한다.
  이는 주행 성능 수락이나 typed dynamic/velocity validity 승격이 아니다. 판제는 미연결이다.
- `주행가능영역·실제 차선`: **`asmc_hd_map`의 최신 v7 정본**
  `config/kcity2025_compact_drivable_map.json`을 읽기 전용으로 표시한다.
  판제의 별도 v5 사본으로 자동 대체하지 않는다. 원점은 판제 `config/behavior/global_info.json`과 pipeline YAML의 map.origin을
  대조한다. 불일치·파일 누락이면 숨기고 사유를 표시한다. 판제 노드를 켤 필요는 없다.
- `/api/drivable`은 서버 시작 때 검증한 도로 면·실제 paint segment를 제공한다. 원본/운영 지도를 수정하거나
  이 영역으로 LiDAR 객체를 제거하지 않는다. 지도 부재가 LiDAR 처리를 막지 않는다.

2026-10-25부터 도로 면을 한 번에 채워 링크별 polygon 테두리와 내부 겹침 띠를 표시하지 않는다.
별도 링크 중심선·차로 안내 구분선은 추가하지 않는다. 같은 product가 SHA로 고정한 실제 차선·정지선만
덧그리며 RViz와 같은 metric 점선 분할 함수를 사용한다. 선과 도로의 SHA가 맞지 않으면 선을 숨기고
사유를 표시한다. 이는 표시 변경이며 local/map 객체 좌표나 tracker를 수정하지 않는다.
- scan의 map pose가 무효하면 Local 보기에서도 지도는 숨긴다. 3D 지도는 고도 모델이
  아닌 자차 높이의 참고 평면이다. 과거 GPS 위치로 지도를 고정해 계속 표시하지 않는다.
- 지도 파일이 바뀌면 모니터를 재시작한다. Python/launch 변경 후에는 launch 재시작이 필요하다.

```bash
# outer workspace 하나만 source한다 (nested Main devel 불필요)
source /opt/ros/noetic/setup.bash && source devel/setup.bash
roslaunch asmc_lidar view.launch open_browser:=true
```

접속 URL: `http://127.0.0.1:8091/` (기본 포트 8091, 기존 planner dashboard 8088과 다름).
서버는 기본으로 loopback(127.0.0.1)에만 바인딩한다.

| 상황 | 명령 |
|---|---|
| 기존 처리 결과에 화면만 추가 | `roslaunch asmc_lidar view.launch open_browser:=true` |
| 다른 namespace 결과 보기 | `roslaunch asmc_lidar view.launch namespace:=/migration/lidar_tunnel` |
| 카메라 없이 처리 시작(별도 터미널) | `roslaunch asmc_lidar lidar.launch fusion:=false debug:=true` |
| 포트 변경 | `roslaunch asmc_lidar view.launch dashboard_port:=8092` |
| 다른 PC에서 접속(명시적 선택) | `roslaunch asmc_lidar view.launch bind_address:=0.0.0.0` |

화면에 점군이 나오려면 처리 노드를 `lidar.launch debug:=true`로 시작해야 한다.
기본 실행은 debug OFF이고, 구독자와 rate limit이 충족된 scan만 샘플링한다.
`view.launch`는 debug를 켜는 서비스나 파라미터 변경을 수행하지 않는다.

## 인자

| 진입점 | 인자 | 기본값 | 의미 |
|---|---|---|---|
| `view.launch` | `namespace` | `/migration/lidar` | 구독할 결과 namespace |
| `view.launch` | `open_browser` | `false` | 준비 후 브라우저 한 번 열기 |
| `view.launch` | `dashboard_port` | `8091` | HTTP 포트 |
| `view.launch` | `bind_address` | `127.0.0.1` | `0.0.0.0`은 명시적 외부 공개 |
| `view.launch` | `pipeline_config` | 패키지 `config/lidar_pipeline.yaml` | 지도 원점 대조용(생산자와 같은 파일 사용) |
| `view.launch` | `max_clients` | `8` | 동시 HTTP 연결 상한 |
| `view.launch` | `http_header_timeout_s` / `http_request_timeout_s` | `5.0` / `20.0` | 헤더/전체 요청 제한 |
| `lidar.launch` | `debug` | `false` | 단계별 샘플 발행 활성화 |
| `lidar.launch` | `debug_rate_hz` | `5.0` | 샘플 scan 주기 상한 |
| `lidar.launch` | `debug_max_points` | `20000` | 단계당 전송 점 상한 |

구형 `monitor.launch`와 `start_pipeline`은 제거했다. 호환 wrapper나 별칭은 없다.

## 기존 실행 구성과의 관계

### A. 기존 주행을 유지하면서 관측 추가

터미널 1~6(`udp_bridge.sh`, `asmc_camera camera.launch`, `Main main.launch`, `lane_boundary_adapter`,
`asmc_planner autonomy_no_bridges_v2.launch`, `asmc_planner dashboard.launch`)은 그대로 둔다.
처리가 없을 때만 새 터미널에서 `roslaunch asmc_lidar lidar.launch debug:=true`를 시작한다.
이미 같은 namespace의 처리가 있으면 두 번째로 실행하지 않는다. 화면은 별도 터미널에서:

```bash
cd <저장소>            # 컨테이너의 /root/ws
source /opt/ros/noetic/setup.bash && source devel/setup.bash
roslaunch asmc_lidar view.launch open_browser:=true
```

`view.launch`가 켜는 프로세스는 `asmc_lidar_monitor_node.py` 하나다.
bridge·카메라·Main은 이미 떠 있는 것을 구독한다(같은 토픽을 두 번 발행하지 않는다).
**주의:** Main과 새 처리가 함께 돌면 LiDAR 처리가 두 번 수행된다(독립 pipeline 2개, 추가 연산 비용은 해당 구성에서 측정해야 함).
planner dashboard(8088, 판제 상태)와 이 LiDAR monitor(8091, 필터 단계·추적)는 서로 다른 화면이다.

### B. LiDAR 독립 시험 (Main·planner 불필요)

입력은 bridge(`./scripts/udp_bridge.sh`) 또는 bag(`rosbag play --clock in.bag` + `rosparam set use_sim_time true`)이다.
카메라 class를 보려면 기존 `asmc_camera camera.launch` 한 세트만 추가한다. 관측 시험이므로 Main과 planner는 켜지 않는다.

```bash
# 터미널 1: roscore 또는 bridge/bag
# 터미널 2 (선택): roslaunch asmc_camera camera.launch mode:=active models:=objects
# 터미널 3:
source /opt/ros/noetic/setup.bash && source devel/setup.bash
roslaunch asmc_lidar lidar.launch fusion:=false debug:=true
# 별도 터미널:
roslaunch asmc_lidar view.launch open_browser:=true
```

bag를 반복 재생(`-l`)하면 두 번째 바퀴부터 stamp가 과거로 돌아가 처리 노드가 scan을
`duplicate_or_regressing_source_stamp`로 거부한다(화면은 stale). 처음부터 다시 보려면 노드를 다시 시작한다.

## 환경별 브라우저 접속

| 환경 | `open_browser:=true` 동작 | 수동 접속 |
|---|---|---|
| 일반 Ubuntu(데스크톱) | `xdg-open` → python `webbrowser` | `http://127.0.0.1:8091/` |
| WSL Ubuntu + Windows 브라우저 | `wslview` → `powershell.exe Start-Process` → `cmd.exe start` | Windows 브라우저에서 `http://127.0.0.1:8091/` (WSL2 localhost 전달; 127.0.0.1 바인딩으로 확인됨) |
| Docker(`docker/ros-noetic`, `network_mode: host`) | 컨테이너 안에서는 열지 않고 URL만 출력 | 호스트 브라우저에서 `http://127.0.0.1:8091/` (host 네트워크는 loopback을 공유) |
| Docker 포트 매핑 없는 bridge 네트워크 | 열지 않음 | 기존 컨테이너를 재생성하지 않는다. 새 컨테이너를 만들 때만 `-p 8091:8091`과 `bind_address:=0.0.0.0`을 함께 쓴다 |

브라우저 열기에 실패해도 LiDAR 노드와 서버는 계속 동작하며 로그에 정확한 URL을 출력한다.
재연결 때 브라우저를 다시 열지 않는다. 포트가 사용 중이면 모니터 노드만 종료되고(`required=false`)
처리 노드는 계속 실행된다. 이때 `dashboard_port:=8092`처럼 다른 포트로 모니터만 다시 띄운다
(`view.launch`는 항상 화면만 실행한다).

## 화면

- **큰 화면**: BEV 또는 3D 점군·OBB·자차. 좌클릭 드래그 회전, 가운데 드래그 이동,
  휠 확대/축소, `차량 중심` 복귀. 별도 측면 화면은 표시하지 않는다.
- **단계 선택**: Raw / ROI / RANSAC 이후(지면 제외) 중 하나. 추적 OBB·ID·class는 별도 ON/OFF다.
  pred 표시는 기본 OFF, 점군·높이 후보 숨김은 기본 ON이다. 실제 점 수와 화면 샘플 수는 구분한다.
- **좌표계**: Local(base_link) / Map. 그 scan의 수평화·장착·map pose로 변환하며,
  map pose 무효 시 Local은 유지하고 지도 오버레이와 Map 선택은 비활성화한다.
- **상태**: 서버·pipeline·map·카메라 상태는 상단, Hz·처리시간 등은 접힌 실행 수치에 있다.
- **객체 상세**(오른쪽): track ID·age·miss, classification·camera_class, frame·stamp, OBB center(표시 frame과
  leveled), length/width/height, yaw, source point count, distance. 속도·동정적 validity가 false이므로
  **"미검증/미제공"**으로 표시한다. KF 내부값(`abs_v`, `velocity`)은 아래 "진단용 미검증 값"에만 보인다.
- 오른쪽 상태 표: 단계별 실제 점 수, RANSAC 비지면/제거 지면, cluster 수, detections/tracks(관측/예측),
  roll/pitch/yaw/속도, 지면 평면 계수, map pose, 처리·capture ms, 누락 단계(waiting/missing), 모니터 수신 통계.

OBB·판정 메타데이터는 점군 네 단계의 도착을 기다리지 않고 먼저 표시한다. 점군은
**같은 scan stamp**의 자료가 준비되면 추가한다. 다른 scan의 점군을 현재 박스에 붙이지 않는다.
미완성 묶음은 기존0.6초 한도 내에서 정리하며, producer session/generation 전환 때 이전
묶음을 비운다. reset 경계 scan의 식별 불가능한 점군은 섞는 대신 생략한다.
브라우저는100ms마다 최신 결과를 조회하고, 한 번에 요청 하나와 그리기 하나만 대기한다.
숨긴 측면 화면은 점을 그리지 않는다. 점군 전송 기본5Hz와 실제 LiDAR 처리율은 별개다.

## HD맵 필터 디버깅

상단은 원본/유지/제외 후보/보류 수, 지도 일치 여부와 그리기 시간을 보여준다.
목록의 `점/OBB %`는 **전체 cluster support의 도로 점 비율 / 기존 OBB 겹침 비율**이다.
실제 점은 voxel·지면 제거·군집 이후의 해당 객체 점이며, 화면 샘플이나 원본 UDP 전체 점이 아니다.
객체를 선택하면 support 수, 도로/경계/미지 셀 수, 최저 점·OBB 하단·지붕+여유,
카메라 보호 여부, 원래 OBB 판정과 보류 사유를 함께 본다.

`제외 후보 모두 숨기기`는 독립 점군/OBB/높이 근거의 후보를 합쳐 **추적 OBB·typed local 비교 OBB·
해당 track에 연결된 근거 점**에 같은 기준으로 적용한다. 선택한 객체도 숨김을 유지하고 목록/상세만
열린다. 목록의 흐린 행과 상세의 `화면 표시`에서 상태를 확인한다. 상단 후보 수는 종합 preview 판정,
`현재 관측/표시 대상/숨김`은 독립 사유 스위치까지 적용한 수로, OBB 비교 후보 때문에 서로 다를 수 있다.
표시 대상 수는 레이어를 실제로 켰는지와 별개인 필터 통과 수다.

이는 도로 밖 모든 객체를 숨기는 기능이 아니다. YOLO 동적·차량 그룹 보호, 지도/위치/점 근거 부족,
지도 SHA 불일치와 예측 track 표시를 구분한다. 근거가 없는 객체를 중심점 위치만 보고 제외하지 않는다.
Raw/ROI/voxel/leveled 전체와 **추적 전 detection OBB**는 원본 단계 비교이므로 후보 숨김을 적용하지 않는다.
이 레이어가 켜져 있으면 화면에 별도 안내한다. 같은 위치의 원본 점·추적 전 박스를 확인하려면 사용하고,
최종 후보 숨김을 검수할 때는 끈다. tracker 이전 detection에는 검증된 최종 track/audit 대응이 없으므로
가까운 중심이나 같은 index만으로 최종 판정을 추측하지 않는다.

카메라로 연결된 `dynamic_obstacle`·`vehicle`은 구간에 관계없이 지도 밖 제외 후보에서 보호한다.
이는 현재 **전방 matching**이 확보한 유효하고 최신인 양성 근거에 적용한다. 좌우 카메라 bbox만으로
임의3D 객체를 만들거나 미구현3뷰 matching이 동작한다고 표시하지 않는다.
현재 정책은 `yolo_vehicle_dynamic_interior_v3`다. person/dog/deer/bicycle/stroller와 car/bus/truck의 **최신 실제 매칭 원본과 그룹**을
확인하고 같은 consumer epoch·시간 한계500ms·후속 미검출0을 요구한다. 과거 확정 분류를 보존하는
`str_YoloClass`나 LiDAR 속도/운동 진단을 양성 관측 대신 쓰지 않는다. 이후 unknown·미검출·세대 변경은
보호 근거를 비운다. `YOLO 동적 보호`·`YOLO 차량 보호`와 `도로 근거 유지`를 화면에서 구분한다.
도로 근거는 합집합 바깥 경계를20cm 줄인 내부에서3점 이상 AND 전체10% 이상이다. 격자0.2m의
1셀 직사각 침식으로 시작 시 한 번 계산하며 내부 차로 경계는 침식하지 않는다. 실제 HD맵은 변경하지 않는다.
각 checkbox 옆 기준과 상세의 원본/안쪽 도로 점 수·비율로 비교한다. 확정 클래스 unknown이라도
최신 유효한 YOLO 양성 연결이 있으면 보호할 수 있다. 이 경우 상세의 원본 매칭 분류를 확인한다.
미션 주변10m에 있다는 이유로 unknown을 전부 보호하던 규칙은 미리보기에서 제거했다.
미션 범위는 OBB 비교용 metadata에만 남는다. `구형 미션 보호`가 보이면 이전 실행 바이너리이므로
LiDAR 처리 노드를 재시작해야 하며 브라우저 새로고침만으로 C++ 판정은 바뀌지 않는다.
높이 후보는 알려진 카메라 객체를 보호하고 최저 점과 OBB 하단 모두 지붕+0.60m를 넘을 때만
표시한다. 하부 가림·위치 오차·예외 공간의 미검증은 최종 판정에서 계속 보류한다.

기본 설정은 패키지 HD맵 audit와 높이 ROI다. 개인 실험 프로필 없이 다음으로 실행한다.

```bash
roslaunch asmc_lidar lidar.launch debug:=true
# 별도 터미널
roslaunch asmc_lidar view.launch
```

브라우저는 `http://localhost:8091/`다. 같은 namespace의 기존 두 실행을 종료한 뒤 시작한다.
다른 프로필을 대조할 때만 아래와 같이 생산자와 모니터에 **동일 파일**을 전달한다.
완료 run은 덮어쓰지 않는다.

```bash
# 호스트 저장 규약으로 새 run을 등록한 뒤, 컨테이너 /root/ws
python3 src/asmc_lidar/tools/filter_audit/prepare_runtime_profile.py \
  --template src/asmc_lidar/config/lidar_pipeline.yaml \
  --manifest /artifacts/<검증한-번들-run>/manifest.json \
  --output /artifacts/<새-run>/runtime.yaml
roslaunch asmc_lidar lidar.launch debug:=true pipeline_config:=/artifacts/<새-run>/runtime.yaml
# 별도 터미널
roslaunch asmc_lidar view.launch pipeline_config:=/artifacts/<새-run>/runtime.yaml
```

현재 PC에서 검증한 설정·성능·제한은 [필터 분석](hd-map-filter-analysis.md)에 연결한다.
화면에서 숨겨도 판제 전달 제외는 활성화되지 않는다.

OBB는 BEV에서 **회전된 footprint**(메시지 corner 0–3을 선택 frame으로 변환·투영한 다각형)로 그린다. 상자 안의
선은 중심에서 corner 1–2 변의 중점까지로, OBB 축(corner 0→1 변, yaw 방향)과 평행하다.
3D는 8개 corner의 12개 변을 그린다. 화면 표시를 위해 ROS 객체의 yaw·corner를 바꾸지 않는다.

### 표시 장면의 신선도와 서버 재시작

- 서버는 실행마다 새 **세션 ID**를 만들고 frame 번호를 1부터 다시 센다. 브라우저는 `since=<frame>&session=<세션>`으로
  요청하며, 세션이 다르면(또는 cursor가 서버 번호보다 크면) 서버는 최신 장면을 즉시 보낸다. 브라우저는 세션이 바뀌면
  이전 장면·점군·객체를 버리고 새 장면부터 그린다(페이지 새로고침 불필요, 이전 번호를 따라잡을 때까지 기다리지 않음).
- **표시 장면 경과** = 서버가 그 장면을 보낼 때의 나이(모니터의 monotonic 시계, 장면 첫 조각 수신 시점부터) + 브라우저가
  받은 뒤 지난 시간(`performance.now`). 상태값(`/api/status`, 서버의 scan 수신 경과)과 분리해서 판단하므로 상태만
  갱신되고 장면이 오지 않으면 STALE이 된다. 새로 받은 장면이라도 이미 오래된 것이면 fresh가 아니다. ROS `/clock`을 쓰지 않아
  bag/시뮬레이터 시계가 멈춰도 경과·STALE이 계속 갱신된다.
- 요청은 3 s 안에 응답이 없으면 취소하고 다음 주기(0.2 s)에 다시 시도한다. 연결 실패·응답 정지(timeout)·서버 혼잡(503)을
  구분해 표시한다.

### 상태 구분

| 표시 | 뜻 |
|---|---|
| 서버: 연결 실패 / 응답 정지 (timeout) / 서버 혼잡 (503) | 모니터 HTTP 서버 접속 불가 / 3 s 안에 응답 없음(요청 취소 후 재시도) / 연결 상한 초과. 마지막 화면은 과거 데이터(붉은 배너) |
| 서버 재시작 감지 (세션 변경) | 이전 장면과 객체를 폐기했고 새 서버의 첫 장면을 기다리는 중 |
| pipeline: ROS 연결됨, LiDAR 입력 없음 | 처리 diagnostics가 `input_silence`(scan이 1 s 이상 처리되지 않음) |
| pipeline: LiDAR 입력 stale | 마지막 debug scan이 1 s 이상 지남 |
| pipeline: IMU/CAN 무효 → INPUT_INVALID | 로컬 pose 없음/stale/무효; 객체 0개, 점군은 표시하지 않음 |
| typed local: OBSERVED_EMPTY | 정상 처리, 객체 없음 |
| map: 사용 불가 — POSE_UNAVAILABLE | GPS 없음/stale/무효; local 화면은 계속 동작 |
| camera: LiDAR 단독 / fusion 비활성 / 비활성(사유) | 카메라 보정 파일 없음 / `enable_camera_fusion:=false` / 카메라 missing·stale·invalid |
| 화면 정지 (Pause) | 사용자가 표시를 멈춘 것(보라 배너, BEV 라벨 `PAUSED`, 표시 장면 경과 계속 증가). 서버·pipeline은 계속 동작하고 상태값은 계속 갱신된다. 재개하면 다음 요청에서 최신 장면을 받는다 |
| STALE 배너 + 흐린 OBB | **표시 중인 장면**이 1 s보다 오래됨. 배너는 서버가 debug scan을 받고 있는데 장면이 안 오는지, 입력 자체가 멈췄는지 구분한다. 표시 객체는 현재 장애물이 아니다 |

## 모니터용 토픽 (처리 노드 `~debug_enabled:=true`일 때만)

| 토픽 | 타입 | 내용 |
|---|---|---|
| `<prefix>/raw`, `/roi`, `/voxel` | `sensor_msgs/PointCloud2` | sensor frame(velodyne) x y z intensity |
| `<prefix>/leveled` | `sensor_msgs/PointCloud2` | 수평화 frame + `label`(uint8: 0 군집 입력, 1 RANSAC 제거 지면, 2 tunnel 제거) + `cluster`(int32, −1 없음) |
| `<prefix>/scan` | `std_msgs/String` | `asmc_lidar_debug_scan_v1` JSON: frame 이름, 수평화 행렬, `base_from_sensor`, `base_from_leveled`, map pose, counts, timings, 샘플 정보(actual/sent/stride), 지면 평면, detection·track OBB(수평화 frame), typed local/map 배열, fusion 상태, 차량 치수 |

- `debug_rate_hz`로 제한된 scan만 내보내며 구독자가 없으면 아무것도 직렬화하지 않는다.
- 캡처는 pipeline 스레드에서 bounded stride 복사만 하고(실측 1~3 ms) 직렬화·발행은 별도 스레드가 한다.
  최신 snapshot 하나만 보관한다(덮어쓴 수는 diagnostics `debug_overwritten`).
- 알고리즘 입력·출력은 바꾸지 않는다(`test/pipeline/test_stage_observer.cpp`: observer 유무와 무관하게 digest·track 동일).
- 제거된 지면은 RANSAC이 실제로 남긴 index로 표시한다(raw/voxel 좌표 차집합이 아니다).
- 공용 `asmc_msgs` 계약은 바꾸지 않았다. `debug/*` 이름도 운영 이름 guard를 거친다.

## 성능과 제한

- 기본 5 Hz, 단계당 최대 20000점. 실제 bag(raw 18k, voxel 9.7k)에서 frame 한 장 약 1 MB, loopback fetch 2~4 ms.
- 브라우저가 느리면 폴링 간격만 늘어난다(서버는 최신 장면을 캐시하고 pipeline은 기다리지 않는다).
- HTTP 연결은 `max_clients`개까지만 handler thread를 가진다. 초과 연결은 bounded queue(32)의 rejecter thread 하나가
  non-blocking 503으로 닫고, queue가 차면 즉시 닫는다. 헤더를 끝내지 않는 연결은 `http_header_timeout_s`, 큰 응답을 읽지 않거나
  천천히 읽는 연결은 `http_request_timeout_s`에서 reaper thread가 끊고 slot을 돌려준다. handler는 store/state lock을 짧게 복사할
  때만 잡으므로 느린 client가 ROS callback을 기다리게 하지 않는다. 연결 통계는 `/api/status`의 `http`에 있다.
  장면 보관은 최대 4개, 무제한 history·trail·journal은 없다.
- 2D Canvas만 사용하며 외부 라이브러리·CDN·인터넷이 필요 없다.
- 속도 정확도·동정적 판정·카메라 fusion 성능은 이 화면의 검증 범위가 아니다.

## 시험

```bash
python3 src/asmc_lidar/test/monitor/test_monitor_core.py      # 장면 묶기·frame 인코딩·HTTP 경로·세션·연결 상한/timeout·브라우저 열기 (ROS 불필요)
node src/asmc_lidar/test/monitor/test_monitor_view.js         # 2D/3D·지도·색상·표시 필터·ID 의심·신선도 (node >= 10, CI 포함)
python3 src/asmc_lidar/test/monitor/test_monitor_browser.py   # 실제 UI 선택·숨김·typed/소속 점·보호/보류 (Chrome/Chromium 필요)
(cd build/asmc_lidar && ctest -R "stage_observer|monitor" --output-on-failure)
rostest asmc_lidar monitor_node.test                          # 실제 처리 노드 + 모니터 + 합성 입력
```

`monitor_node.test`는 합성 점군(지면 + 상자)으로 상태 전이(정상·INPUT_INVALID·입력 중단·복구), 장면/stamp/frame
정합, 고정 경로만 제공, 모니터가 아무것도 발행하지 않음을 확인한다. 합성 입력은 인터페이스 시험이며 정확도 시험이 아니다.

브라우저 검사는 외부 Python/JS 패키지 없이 합성 장면을 임시 loopback 서버로 제공한다.
별도 임시 Chrome profile을 사용하고 종료하므로 사용자의 브라우저·ROS·rqt를 건드리지 않는다.
실행 파일을 못 찾으면 `--chrome /path/to/chrome`을 지정한다. 기존 Node CI와 달리 Chrome 실행은 별도 로컬 검사다.
