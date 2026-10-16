# 카메라 음영 family → 판제 인계

> **문서 상태:** Active Handoff Guide
> **Handoff 허브:** [README.md](README.md)
>
> **동결 ID:** `shadow_camera_perception_v3_candidate_20260915`
>
> **적용 ODD:** `shadow_two_way_v1`
>
> **상세 실험:** [BEV 개발 이력](../records/camera/bev-development-history.md)
> **실행 계약:** [asmc_msgs](../../src/asmc_msgs/msg/LaneBoundaryFamily.msg),
> [jamming_lane_boundary_adapter](../../src/jamming_lane_boundary_adapter/README.md)
> **Planning 경계:** [Planning/Control 개발 가이드](../architecture/planning-control.md)

이 문서는 음영 왕복 2차로에서 카메라가 제공하는 차선 family와 주행가능영역을 판제가
어떻게 해석해야 하는지 설명한다. 인지 쪽에서는 ROS 출력까지만 제공한다.
현재 세 모델은 상시 ACTIVE이며 [구간별 소비 안내](perception-to-planning.md)를 따른다.
공식 팀 음영은 A126이며 A448은 접근 구간이다. A126 진입 전에 새 유효 family를 확인한다.
어느 위치에서 정보를 사용할지, 어느 차로가 home/opposing인지, 장애물을 피해 어느 경로를 선택할지와
closed-loop 합격 여부는 판제 파트가 소유한다. 카메라 생산자는 Behavior/Frenet/MPC나 `/ctrl_cmd`를 발행하지 않는다.
현재 판제 통합 launch는 별도의 재밍 어댑터를 enable_control=true로 포함하므로 전체 스택이 read-only라는 뜻은 아니다.

## 1. 데이터 흐름과 소유권

```text
MORAI RGB ×3 --UDP--> bridge --ROS--> TwinLiteNet+ v6
                                        ├─ metric shared boundary family
                                        └─ camera drivable BEV + coverage
                                                     │
                                                     ▼
                                    planning-owned adapter / policy
                                    ├─ route·MGeo로 차로 의미 결정
                                    ├─ LiDAR object/occupancy와 결합
                                    ├─ 회피 corridor·복귀 목표 생성
                                    └─ Frenet trajectory → MPC
```

| 책임 | 인지 | 판제·퓨전 |
|---|---:|---:|
| 카메라 source timestamp와 좌표 보존 | O | 검증 |
| white/yellow 경계 family와 support 상태 생성 | O | 소비 |
| camera drivable score와 coverage 생성 | O | 소비 |
| 음영구간 진입·이탈 판정과 기능 on/off | X | O |
| home/opposing/현재·목표 차로 의미 부여 | X | O |
| LiDAR occupancy 차감과 안전 여유 적용 | X | O |
| 회피 방향·복귀 시점·trajectory 생성 | X | O |
| stale/fallback과 closed-loop 합격 기준 | 진단 제공 | O |

카메라 모듈이 켜져 있는 것과 판제가 그 결과에 제어 권한을 주는 것은 별개다. 판제는
기본 disabled feature flag와 ODD gate를 별도로 두는 것이 권장되지만, 그 flag와 연결 코드는
이 인수 작업의 범위가 아니다.

`lane_boundary_adapter`의 family monitor는 read-only reference다. 같은 패키지의 BBox 차로/재밍 제어 launch는 별도 경로다.
최종 판제는 가능하면 그 node를 별도 프로세스로 영구 유지하지 않고,
`BuildBoundaryFamilyStripContext()`·freshness/invalid gate·관련 테스트를
planning-owned lane/corridor module에 흡수한다. 흡수 전에는 아래 8절 인수 체크리스트와
adapter README의 bag/closed-loop parity를 통과해야 한다. standalone package 삭제는
대체 consumer merge 후 별도 PR에서만 수행한다.

## 2. family 정본과 선택적 raster

| 토픽 | 타입 | 의미 | 필수 해석 |
|---|---|---|---|
| `/perception/lane/boundary_family` | `asmc_msgs/LaneBoundaryFamily` | 세 경계의 공동 metric 곡선과 sample별 근거 | freshness, `valid`, ODD, algorithm, support 검사 |
| `/perception/bev/camera/drivable_probability` | `sensor_msgs/Image` | 카메라 기반 주행가능 score `0~255` | coverage와 함께 사용 |
| `/perception/bev/camera/coverage` | `sensor_msgs/Image` | 한 카메라 이상이 기하적으로 관측한 cell은 255 | 0은 non-drivable이 아니라 unknown |

기본 필수 경로는 boundary_family다. drivable raster는 소비자/recorder/시각화 수요가 있을 때만
생성되며 무구독 상태에서 raster가 없다고 family 실패로 판정하지 않는다. raster를 사용하는 소비자는
probability와 coverage 둘 다 구독해야 한다. 보조 토픽은 `/perception/bev/diagnostics`, `/perception/lane/diagnostics`다.
`/perception/bev/camera/source_count`는 기본 off이며 overlap 검수에서
`publish_bev_debug_metadata:=true`로 켠다. road-marking raster와 Marker는 분석·시각화용이며
판제의 최소 입력 계약이 아니다.

특히 `/planning/debug/lane_boundary_family/strips`는 reference adapter가 `x[0,15]m`에서
수락한 interval만 그리는 MarkerArray다. 이를 planning 데이터로 구독하거나 짧은 길이를
`boundary_family`의 유효거리로 해석하지 않는다. 정본 typed family는 전체 polyline과 sample별
support를 보존한다. 판제가 더 긴 horizon을 선택할 때도 `INVALID`를 연결하거나 추정으로
승격하지 말고, freshness·폭·support gate와 별도 far-range audit를 적용한다.

family와 해당 raster 출력은 `header.frame_id=base_link`이고 x-forward, y-left다. timestamp는 callback
수신 시각이 아니라 세 카메라 입력 묶음의 최신 source stamp다. BEV fusion은 세 view의
stamp 차이가 35ms 이내일 때 최대 10Hz로 발행한다. 한 view가 빠지면 오래된 raster를 다시
발행하지 않으므로 consumer가 마지막 메시지를 무기한 유지해서는 안 된다. 초기 freshness
상한은 lane과 raster 모두 0.35초를 제안하며, 최종 수치는 판제 full-stack 실측으로 정한다.

## 3. 차선 family 계약

동결된 producer 식별자는 다음과 같다.

```text
operating_domain = shadow_two_way_v1
algorithm_id     = twinlite_v6_shadow_boundary_family_candidate_v3
representation   = shared_parametric_bspline_normal_offsets_v2
expected count   = 3
config SHA-256   = 691b998247c960bba06d84513b217709a55f8744530978e32d9028289e217070
deployment_allowed = false
```

`deployment_allowed=false`는 메시지를 버리라는 뜻이 아니라, 현재 결과가 판제 연결과
closed-loop 승인을 대신하지 않는다는 뜻이다. read-only 검토에서는 이 값을 요구한다.
실제 제어 승격 시 판제 파트가 별도 opt-in과 검증 기록을 남겨야 하며, 인지 쪽 flag를 조용히
바꾸는 방식으로 권한을 열지 않는다.

세 member의 역할은 `LEFT_OUTER_BOUNDARY`, `ROAD_CENTRE_BOUNDARY`,
`RIGHT_OUTER_BOUNDARY`다. 이는 음영 도로에서의 기하 순서이며 다음 의미를 포함하지 않는다.

- home lane 또는 opposing lane
- 절대 차로 번호
- 차선 변경 방향이나 회피 권고
- yellow 경계의 어느 쪽이 현재 진행 방향인지에 대한 route 판정

판제는 전역경로/MGeo와 차량 위치를 함께 사용해 두 인접 strip 중 현재·목표 strip을 정해야
한다. 단순히 yellow가 영상 왼쪽/오른쪽에 있다는 이유만으로 home/opposing을 고정하면
중앙선 straddle이나 반대 차로 회피 중 오판할 수 있다.

### 3.1 Sample 상태

`polyline[i]`와 `support_state[i]`는 정확히 대응한다.

- `OBSERVED`: 해당 역할의 현재 frame 카메라 evidence가 직접 지지한다. 정상 후보로 사용한다.
- `STRUCTURE_INFERRED`: 같은 frame에서 받아들인 평행 family의 normal-offset 구조가 보완한
  구간이다. 이전 frame, IMU, MGeo 또는 temporal prediction이 아니다. 사용 가능하지만
  낮은 권한으로 다룬다.
- `INVALID`: 계산된 곡선 좌표가 남아 있어도 인지 근거가 없는 구간이다. 기본 제어에서는
  사용하지 않고, 해당 위치에서 strip을 분할한다.

aggregate `confidence`와 residual은 calibration된 충돌 확률이 아니다. 현재 C++ reference
adapter는 inferred segment에 `0.60`을 곱하고, producer가 이미 전체 role을 강등한 경우에는
중복으로 곱하지 않는다. 판제는 이 값을 cost 또는 후보 순위에 사용할 수 있지만 안전 확률로
곧바로 해석하면 안 된다.

### 3.2 권장 변환

판제는 새 spline을 다시 fitting하기보다
`lane_boundary_adapter::BuildBoundaryFamilyStripContext()`를 재사용할 수 있다. 이 core는
다음을 보장한다.

1. `frame_id`, ODD, algorithm, representation과 배열 형태 검사
2. producer `valid_x`와 consumer horizon의 교집합 적용
3. 인접 두 경계의 네 endpoint가 모두 non-invalid일 때만 strip interval 생성
4. inferred endpoint가 하나라도 있으면 degraded segment로 분리
5. invalid endpoint가 하나라도 있으면 hard split
6. 폭, lateral order, 교차와 급격한 폭 변화 검사

현재 read-only adapter의 `x[0,15]m`는 audit 범위이지 perception의 영구 최대거리 계약이
아니다. producer는 설정된 전체 polyline과 sample 상태를 보낸다. 판제는 매 frame 연속된
usable interval과 실제 최대 horizon을 계산하고, 모델 재학습으로 범위가 늘면 새 audit 뒤
horizon만 갱신할 수 있어야 한다.

`valid=false`인 새 메시지는 “현재 frame fit reject”다. 토픽이 없거나 stale인 상태와
구분하되 두 경우 모두 이전 곡선을 무기한 유지하지 않는다. 현재 인지에는 Ego-motion 기반
temporal curve 전파가 없으므로, 이전 승인 경계를 얼마나 유지할지는 판제 fallback 정책이다.

## 4. Drivable BEV 계약

현재 computational grid는 다음과 같다.

```text
frame:      base_link
x:          [-5, 20] m
y:          [-8, 8] m
resolution: 0.10 m/cell
shape:      height 250 × width 160
row:        (20 - x) / 0.10
column:     (8 - y) / 0.10
```

현재 raster는 `sensor_msgs/Image`이므로 메시지 자체에는 metric 원점과 해상도가 들어 있지
않다. 이는 음영구간 v3의 **동결된 임시 전송 envelope**이며, 판제 consumer는
[`bev_grid.yaml`](../../src/asmc_camera/config/bev/bev_grid.yaml)과 동결 기록의 SHA-256
`8bad13875a2b7d9eec4a35687433446b1a57112257ec34520cf5c1d6e47e0ad3`을 함께 pin해야 한다.
다른 grid를 같은 토픽 이름으로 조용히 해석해서는 안 된다. 공통 BEV custom message와
metadata topic은 MGeo/LiDAR 계약을 함께 합의할 때 결정하며, 이번 v3 인수에서 선행 확정하지
않는다.

한 frame을 받아들일 때는 다음 envelope를 모두 확인한다.

- drivable과 coverage의 `header.stamp`, `frame_id`, `height`, `width`가 서로 같음
- `frame_id=base_link`, `encoding=mono8`, `height=250`, `width=160`
- `step>=160`이고 buffer 길이가 `step×height` 이상임
- freshness와 BEV diagnostics가 판제의 full-stack 기준을 만족함

둘 중 하나가 없거나 stamp/shape가 다르면 두 영상을 섞지 말고 그 frame 전체를 unknown으로
처리한다. Image row padding 가능성이 있으므로 `width`만큼 연속이라고 가정하지 말고 `step`을
따라야 한다.

`drivable_probability`는 세 카메라 score의 기하 품질 가중 평균이다. `0`은 두 의미가 가능하다.

- `coverage=255`, score=0: 카메라가 관측했으며 non-drivable 쪽으로 판단
- `coverage=0`, score=0: 카메라가 관측하지 못한 unknown

따라서 coverage 없이 score만 threshold하면 unknown을 장애물 또는 도로 밖으로 잘못
확정한다. opt-in `source_count`는 같은 cell을 본 카메라 수 `0~3`이며 overlap seam
디버그와 선택적 신뢰도 조정에 사용할 수 있다.

이 모델은 actor-negative 정책으로 학습되어 차량·보행자·장애물 위치가 낮은 drivable
score 또는 hole로 보일 수 있다. 그러나 object detector나 occupancy sensor가 아니므로 모든
장애물을 놓치지 않는다는 보장은 없다. 반대로 그림자·가림·노면 표식 때문에 낮아질 수도
있다. 판제에서는 drivable을 **soft traversability evidence**로 쓰고, LiDAR object/occupancy를
충돌 veto로 우선 적용하는 구성이 안전하다. threshold와 morphology는 아직 perception
계약으로 동결하지 않았으며 판제 목적과 closed-loop 결과에 맞춰 설정한다.

## 5. 장애물 회피·차로 주행에 쓰는 순서

권장 결합 순서는 다음과 같다.

1. route/MGeo로 현재 음영 ODD와 진행 방향을 판정한다.
2. fresh하고 계약이 맞는 boundary family에서 observed/degraded strip segment를 만든다.
3. route 문맥과 y=0 vehicle projection으로 현재 후보 strip을 찾되, 중앙선 위에서는 두
   후보가 겹칠 수 있음을 허용한다.
4. camera drivable score를 coverage가 있는 cell에만 soft cost로 적용한다.
5. LiDAR object/occupancy와 속도·안전 여유를 planning/fusion에서 적용한다.
6. Frenet 후보가 boundary를 넘는 위치, obstacle clearance와 복귀 가능 구간을 함께 평가한다.
7. family가 invalid/stale이면 카메라 geometry의 권한을 내리고 MGeo/이전 승인 상태/정지 같은
   판제 fallback으로 전환한다.

인지가 safe corridor를 직접 만들어 주지 않는 이유는 회피 대상의 속도, 차량 footprint,
제동거리, route intent와 허용 중앙선 침범을 판제가 함께 알아야 하기 때문이다. 카메라
drivable과 lane family는 근거를 제공하고 최종 corridor 생성은 판제·퓨전 파트가 소유한다.


## 6. 현재 실행·모드·품질

TwinLite Medium v6 TensorRT FP16, 전방544×960 B1·좌우480×640 B2, 추론 목표15Hz입니다.
family는 model-grid→지면투영→최신3뷰→shared fitting 경로이고 영상 BEV 합성과 독립입니다.
원본 RGB/보정·미터 grid·support 의미는 유지합니다. BEV raster의10Hz 상한과 family/모델15Hz 목표를 섞지 않습니다.
일반 구간은 HD맵이며 lane STANDBY, 음영 진입 전에 아래 절차를 사용합니다.

```bash
roslaunch asmc_camera camera.launch models:=objects,lane
# 별도 터미널
rosservice call /perception/camera/lane/set_mode '{mode: 1, target_hz: 0.0}'
rostopic echo /perception/camera/lane/status
# 같은 generation의 prepared/pipeline_ready 확인 후
rosservice call /perception/camera/lane/set_mode '{mode: 2, target_hz: 0.0}'
# 음영 이탈 후
rosservice call /perception/camera/lane/set_mode '{mode: 0, target_hz: 0.0}'
```

이 예제의 objects도 STANDBY에서 시작하므로 필요하면 objects PREPARE/ACTIVE를 별도로 요청합니다.
신호등 producer는 카메라/판제 중 하나만 실행합니다. 위 models 조합에는 신호등이 없습니다.
일반 시각화는 view.launch, read-only strip 검수는 별도 monitor입니다.

```bash
roslaunch asmc_camera view.launch models:=lane
rosrun rqt_image_view rqt_image_view --clear-config /perception/lane/debug/shared_family_bev/compressed
# marker/strip 계약을 검토할 때만
roslaunch lane_boundary_adapter shadow_boundary_family_candidate.launch rviz:=false
```

현재 `camera_bbox_lane_change.launch`는 기본 enable_control=false이고 판제 통합 launch가 true로 덮어씁니다.
카메라 모드와 최종 NORMAL/JAMMING 선택은 동일 기능이 아닙니다. 구간 활성화·정지/재개·fallback은 판제에서 확인합니다.

## 7. 수락 상태와 인수 확인

#85의 TensorRT semantic 품질·리팩토링은 검증했고 사용자 직접 주행에서 family가 잘 표시됨을 확인했습니다.
자동 시험의 family 최대공백261.7ms(250ms 기준 초과)와 SUNNY freshness 사건은
[최신 공급 기록](../records/camera/camera-family-supply-optimization.md)에 남아 있습니다.
freshness350ms를 늘리거나 예전 결과 재발행으로 해결하지 않습니다.
현재 기본 backend 선택·사용자 승인과 전체 자동 수락을 구분합니다.

판제는 ODD·algorithm/config·source/receive age·valid/support를 검사하고 INVALID interval을 연결하지 않습니다.
PREPARE 새 세대와 정상 ACTIVE 창을 구분하며 재시작/입력 중단 시 과거 family를 사용하지 않습니다.
회피 corridor·현재/반대 차로 의미·LiDAR veto·복귀·명령 단절 제동은 판제와 함께 검증합니다.
일반 구간의 lane OFF/STANDBY도 다른 모델 공급을 방해하지 않아야 합니다.

과거 v3의4bag/1,270frame 수락과 당시 실행 명령은 [동결 인수 기록](../records/camera/shadow-handoff-v3.md)에 보존합니다.
이를 현재960·전체 GUI·실제 판제 연결 수락으로 대체하지 않습니다.
직접 실행과 현재 객체 인계는 [실행 명령](../operations/commands.md), [카메라→LiDAR](camera-to-lidar.md)를 따릅니다.
