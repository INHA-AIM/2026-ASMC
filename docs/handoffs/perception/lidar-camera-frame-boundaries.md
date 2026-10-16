# LiDAR·카메라 좌표와 GPS 경계 — opt-in 구현

> 2026-10-16 현재 실행은 [카메라→Main/LiDAR 인계](../../packages/asmc_lidar/camera-handoff.md)를 따른다. 아래 날짜별 실험 명령은 당시 checkout 재현용이며, 현재 Main에서는 start_yolo 옵션이 제거됐다. 원본 수치·설정·실패 증거는 보존한다.

`feat/lidar-camera-frame-boundaries`는 B1a PR #71 병합 main `534ddc5`를 기준으로
한다. 사용자의 승인으로 #71을 병합했고, 실패 원본과 고정 재호 브랜치 `8dde9b6`은
유지한다. 새 경로와 roll 보정은 각각 수락 전이며 기본 OFF다.

external 6개 구현의 좌표·관측 시각 처리와 실제 Python/C++의 54조건 수치 대조는
[좌표 계약 참고 구현 검토](camera-lidar-coordinate-reference-review.md)에 기록한다.
합성 행렬 일치와 실제 시간 정합·지면/높이·객체 보존 수락은 구분한다.

## 운영 경계와 구현 위치

`asmc_lidar`는 ROS 없는 좌표·시간 코어를 제공한다. 현재 검출·군집·Kalman·
Hungarian·fusion은 nested `Main`에서 한 번 실행하며, 새 `Geometry/` 모듈이
그 코어를 연결한다. 전체 패키지 이전이나 터널 필터 채택이 완료된 것은 아니다.
카메라는 영상 모델과 원본 픽셀 박스를 제공하고, 판제가 구간 활성화와 결과 소비를 결정한다.

| 좌표/출력 | 의미 |
|---|---|
| 원본/ROI 점 | LiDAR 장착 위치의 센서 좌표 `velodyne` |
| 지면·군집·내부 추적 | 실제 scan 행렬로 수평화한 센서 좌표 |
| `/perception/lidar/objects_local` | 후륜축 중심 `base_link`, 현재 실측 OBB |
| `/perception/lidar/objects_map` | 동일 관측·ID를 가용 pose로 변환한 `map` OBB |
| `/perception/lidar/diagnostics` | 입력 침묵·무효, map 보류, 관측 세대·출처·출력 비용 |

typed 출력은 구독자가 있을 때 객체 변환·직렬화·발행한다. 구독자가 없어도 local
처리와 추적은 지속한다. map 출력 때문에 검출·추적을 다시 실행하지 않는다.
local만 구독하면 typed map geometry도 계산하지 않는다. legacy 출력·map 기반
motion 분류 비용은 별도이며, 전체 비용은 아직 부하 측정 전이다.

일반 경로의 후보 수평화는 `Ry(pitch) × Rx(roll)`이며, 비영 장착 회전에서는
`R_mountᵀ × R_vehicle_level × R_mount`로 합성한다. pitch-only 후보는 별도로
유지한다. 실제 적용 행렬·source stamp·입력 세대·보정 SHA는 fusion history와 함께
보관한다. 과거 관측의 카메라 투영은 해당 관측의 행렬을 사용한다.

수평화 점 → 수평화 역변환 → 센서 장착 변환 → base_link → camera/map 순서다.
카메라 투영은 행렬을 합성해 점군 전체의 중간 복사를 만들지 않는다. raw 점은
수평화 역변환을 적용하지 않는다. 기존 IoU·분류 정책·50ms 매칭·200ms 대기·
YOLO 고유 소비는 유지한다.

opt-in의 수평화 debug 점군·tracking marker는 `lidar_leveled`로 표시하고, 실제
관측 행렬의 `base_link → lidar_leveled` TF를 제공한다. raw/ROI의 `velodyne`
장착 TF는 기존 bridge 소유다. Main은 bridge의 `map → base_link` TF를 덮어쓰지
않으며, 객체 map 변환에 그 TF를 사용하지 않는다. `/lidar_nogps`는 변경하지 않는다.

## Local 추적과 GPS 정책

local 입력은 scan 이전의 유효 IMU·차량 속도다. freshness 120ms와 최대 50ms
callback 대기를 유지하며 GPS를 기다리지 않는다. 새 무효 IMU/속도 샘플이 있으면
이전 정상 샘플로 대체하지 않는다. 상대 CA 예측·크기 비용·Hungarian·ID 정책은
유지하되 연관과 중복 트랙 정리는 항상 local 기준이다. 이전 상태를 현재 수평화
좌표로 재표현하고, CAN 이동량을 상대 CA 예측에 다시 빼지 않는다.

GPS 무효·침묵은 local tracker/class history를 초기화하지 않는다. map 기반
motion 이력만 무효화하고 복구 시 다시 준비한다. 현재 2D Kalman 도함수의 3D
속도·각속도 의미가 입증되지 않았으므로 typed relative/absolute velocity와
dynamic validity는 모두 false다. 이를 유효한 0속도·정적 판정으로 소비하면 안 된다.

기존 `PoseHistory`는 100ms 이내 bracketing 보간만 제공한다. 별도
`BoundedPoseHistory` API는 다음 정책을 사용한다.

- 공통 Main은 원본 ROS source ns, callback entry의 ROS receipt ns와 steady receipt ns,
  worker의 steady processing ns를 구분한다. GPS는 실제 수신이 scan receipt 이전이고
  source stamp도 scan 이전인 표본만 사용하며 age는 최대 300ms다.
  처리 시각까지 늦게 도착한 GPS로 지난 관측의 map 가용성을 바꾸지 않는다.
  source ROS 시각과 monotonic 시각을 서로 빼지 않는다. 기존 opt-out 경로는 ms를 유지한다.
  엄격 API는 동일 clock만 허용하고 별도 bounded API에만 ROS/steady 조합을 제공한다.
  구현 회귀와 재생 수락 결과는 [연속 회귀](lidar-continuous-regression.md)에 구분한다.
- GPS 관측 당시 IMU로 안테나 offset을 제거한 뒤, 유효 scan IMU·속도로 짧은
  위치 보정을 한다. GPS 관측과 scan의 자세를 혼동하지 않는다.
- 무효 status·0 좌표·비유한 값·잘못된 map offset은 map 가용성을 해제한다.
  전달 중단은 마지막 정상 표본의 age가 300ms를 넘으면 보류한다.
- 정상 GPS 연속 3회로 복구한다. 단절 전후 GPS 위치 차분으로 속도를 계산하거나
  음영 내부에서 map 위치를 지속 추정하지 않는다.
- map 원점·센서 JSON SHA·LiDAR/카메라 장착·광학 축·영상 크기·FOV를 시작 시 검사한다.

주 bag의 읽기 전용 높이 감사에서는 관측 이전 Ego/IMU가 있는 GPS 238표본의
`GPS altitude - 회전한 안테나 lever z`와 Ego z 차이가 최대 0.047803m였다.
map 높이 원점 0m와 MORAI altitude를 사용하는 근거이며, 다른 장비·datum이나
HD맵의 2D z=0 표현까지 완전한 3D 높이 보장으로 확대하지 않는다.

정상 빈 관측은 `OBSERVED_EMPTY`, 필요한 입력 무효는 `INPUT_INVALID`,
local 관측에 대한 map pose 불가는 `POSE_UNAVAILABLE`로 구분한다. 침묵이나
초기 무관측에 새 stamp의 가짜 배열을 만들지 않는다. 예측 중심과 오래된 코너를
섞은 ghost는 typed 출력에 넣지 않는다.

legacy map/UDP는 유효 pose를 요구한다. motion 상태의 무효를 표현하지 못하는
legacy 형식은 준비 전 출력도 보류하고 진단한다. typed map geometry는 이 경우에도
유효 pose가 있으면 제공할 수 있다. 보류 시 이전 map marker를 삭제하며 opt-in
marker lifetime은 300ms다. 기존 소비자가 이 차이를 검토하기 전 기본 승격하지 않는다.

## 직접 실행

바깥 workspace를 빌드·source한 뒤 nested workspace를 빌드해야 한다. 새 셸에서:

```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
catkin_make -j2
source devel/setup.bash
cd src/lidar+sensorfusion
catkin_make -j2
source devel/setup.bash
```

컨테이너 진입·bridge·ROS master는 사용자가 별도로 준비한다. 실시간에서는
`ROS_MASTER_URI=http://127.0.0.1:11311`, `ROS_HOSTNAME=localhost`,
`use_sim_time=false`를 사용한다. bag 재생은 전용 격리 환경만 사용한다.

```bash
roslaunch asmc_camera camera_objects.launch initial_mode:=2
```

별도 터미널에서 바깥·nested 환경을 모두 source한 뒤 pitch-only 후보:

위 `initial_mode:=2`는 객체 모델을 ACTIVE로 시작한다. 개별 객체 launch도 기본은
STANDBY이므로 생략하면 입력을 받아도 반복 추론·운영 결과를 내지 않는다.

```bash
roslaunch Main main.launch start_yolo:=false rviz:=false \
  enable_common_geometry:=true lidar_level_roll:=false
```

roll·pitch 후보는 `lidar_level_roll:=true`로 바꾼다. 기존 경로 복원은
`enable_common_geometry:=false`이며, 객체 생산자는 한 개만 실행한다.
시각화는 별도 터미널에서 직접 연다.

```bash
rqt_image_view --clear-config /fusion/debug_image
```

RViz local 점군은 Fixed Frame을 `base_link`로 두고 `/lidar/debug_clustered`,
`/lidar_tracking_markers`를 선택한다. map pose 보류는 아래 상태로 확인한다.

```bash
rostopic echo /perception/lidar/objects_map/status
rostopic echo /perception/lidar/diagnostics
```

## 측정과 남은 수락

현재 CPU·ROS 결과와 미완료 항목은
[baseline](../../../config/baselines/perception_lidar_camera_frame_boundaries_v1.yaml)에 기록한다.
Main 전체 회귀는 통과했고 바깥 회귀에는 기존 facade 실패 1건이 있다. 전체 패키지
시험 target 빌드에는 변경하지 않은 `behavior_planner/fsm_command_binding_test`의
`undefined reference to main`도 남아 있어 이를 신규 LiDAR 실패와 구분한다.

고정 프로토콜은 [실험 설정](../../../config/experiments/lidar_camera_frame_boundaries_v1.yaml)에
있다. `run_frame_boundary_matrix.py`는 기존 B1a fixture와 고정 YOLO 입력을 재사용해
state 54회, subscriber cost 24회를 직렬 실행한다. 별도 diagnostic 6회는 기존
`fusion_trace_path`를 사용한다. 같은 port의 master 재사용과 MORAI 실행·확인 실패를
거부한다. 상태 비교의 debug cloud 관측 비용과 cost 실행의 무구독 조건은 분리한다.

원본 성공·실패는 모두 보존한다. 분석기는 GPS 조건별 local stamp/geometry/ID와
map gate, fusion 종료 집계를 보고하며 자동 운영 승인을 내리지 않는다. realtime
스케줄링 차이를 결정론적 전체 파이프라인 동등성으로 표시하지 않는다.

종료 집계 smoke에서는 GPS 전달 중단 조건의 208 scan 모두 local 출력되고 map
124개는 `POSE_UNAVAILABLE`로 보고됐다. stamp 중복/역행은 0이었다. 이는 한 번의
smoke이며 반복 프로토콜 수락을 대체하지 않는다. timer의 마지막 부분 집계 누락과
ROS 종료 후 logging 실패 시도는 보존했고 직접 stdout 최종 집계를 확인했다.

### 2026-10-08 고정 재생 결과

Simulator 종료를 감시하며 state 54회와 subscriber cost 24회, 총 78회를 완료했다.
실행 완료는 운영 수락을 뜻하지 않는다. 원본 전체 로그·관측·자원 기록과 분석은
[증거 목록](../../../config/baselines/evidence/lidar_camera_frame_boundaries_v1/README.md)에 있다.

공통 경로의 주/음영 typed local은 매 실행 각각 649/208개 source stamp를 보고했다.
주 bag에는 입력 무효 1–2개가 포함됐으며 음영은 모든 208개가 처리됐다.
중복/역행 stamp는 0이고, GPS 무효·전달 중단에서도 local 공급은 이어졌다.
map observer gate 위반은 0이지만 observer와 Main callback의 수신 경계는 다르므로
시간 계약 수락으로 확대하지 않는다. 정상 GPS에서도 일부 `POSE_UNAVAILABLE`이
있다. source-ms cutoff 및 replay clock 지연 문제와 함께 원본 기록을 유지한다.

24개의 정상/무효·중단 GPS 대조 모두 strict local payload 일치는 실패했다.
공통 stamp 집합은 같지만 주 bag 542–602프레임, 음영 179–197프레임에서 형상·ID 등을
포함한 payload가 달랐다. 첫 primary/pitch 반복의 상호 유일 중심 대응(1mm)에서는
대조별 8,252관측의 분류가 같았지만, 미대응 관측 1,300여 개가 남는다.
정상 GPS 두 반복 사이에도 형상·ID 차이와 분류 차이 1건이 있었다.
이는 원인을 GPS 또는 추적 결함으로 단정할 증거도, 스케줄링 차이로 면제할 근거도
아니다. 동일 raw 관측·IMU/CAN 선택을 고정하는 전체 CPU 재현이 필요하다.

구독 없음/local/map/동시 구독을 각 3회 비교했을 때 주 bag Main CPU의 실행별
p50 중앙값은 34.7–34.9%(한 CPU core=100%), RSS 실행별 p95 최대는 221.6MiB 이하였다.
음영 CPU는 20.5–24.4%, RSS는 246.4MiB 이하였다. 주 bag local 단독/동시 출력 p95
wall 지연은 각각 44.5–45.6/44.5–45.3ms, 음영은 47.2–48.1/47.0–49.7ms였다.
이 표본에서는 검출을 두 번 하는 수준의 추가 부하는 관측되지 않았지만,
비용 0 또는 전체 카메라/GPU 스택 성능 보장은 아니다. 고정 YOLO, 진단 observer
포함, GUI 없음이라는 측정 범위를 유지한다.

아직 남은 수락은 실제 callback receipt·원본 ns의 시간 계약 적용, 방향을 검증한
자차 속도, 동일 scan의 전체 파이프라인 CPU 재현, pitch-only/roll·pitch 객체
보존·지면 비교, 정상 map 추적 품질, 실제 GPU 추론·fusion 부하와 사용자 직접
launch 확인이다. 새 경계와 roll 기본값은 OFF이며 PR #72는 Draft다.
기준 미달·독립 GT 부족은 보류로 남기고 기존 분류 차이 승인을 새 실패에 적용하지 않는다.

재호 후속 과제는 [B1a 개선 과제](../../packages/asmc_lidar/b1a-improvement-tasks.md)의
단일 경로 터널 필터 이식·잔여 벽/군집 혼합·객체 보호/높이·충돌 경계·전체 패키지
이전이다. jamming 어댑터·21% crop·카메라 모델/클래스/mask·HD맵 pin은 유지한다.
차선 피팅 소비는 음영 ODD로 한정하며 판제가 구간과 주행 모드를 결정한다.
