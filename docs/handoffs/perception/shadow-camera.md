# 음영구간 카메라 인지 v3 판제 인수 가이드

> **문서 상태:** Active Handoff Guide
> **Handoff 허브:** [README.md](../README.md)
>
> **동결 ID:** `shadow_camera_perception_v3_candidate_20260915`
>
> **적용 ODD:** `shadow_two_way_v1`
>
> **상세 실험:** [BEV 개발 이력](../../../src/asmc_camera/docs/bev-development-history.md)
> **실행 계약:** [asmc_msgs](../../../src/asmc_msgs/msg/LaneBoundaryFamily.msg),
> [jamming_lane_boundary_adapter](../../../src/jamming_lane_boundary_adapter/README.md)
> **Planning 경계:** [Planning/Control 개발 가이드](../../packages/planning-control/README.md)

이 문서는 음영 왕복 2차로에서 카메라가 제공하는 차선 family와 주행가능영역을 판제가
어떻게 해석해야 하는지 설명한다. 인지 쪽에서는 ROS 출력까지만 제공한다. 어느 위치에서
기능을 켜고 끌지, 어느 차로가 home/opposing인지, 장애물을 피해 어느 경로를 선택할지와
closed-loop 합격 여부는 판제 파트가 소유한다. 현재 perception 코드는 Behavior, Frenet,
MPC 또는 `/ctrl_cmd`에 연결하지 않는다.

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

`lane_boundary_adapter`는 이 계약을 검증하기 위한 임시 read-only reference package다.
최종 판제는 가능하면 그 node를 별도 프로세스로 영구 유지하지 않고,
`BuildBoundaryFamilyStripContext()`·freshness/invalid gate·관련 테스트를
planning-owned lane/corridor module에 흡수한다. 흡수 전에는 아래 8절 인수 체크리스트와
adapter README의 bag/closed-loop parity를 통과해야 한다. standalone package 삭제는
대체 consumer merge 후 별도 PR에서만 수행한다.

## 2. 판제가 읽을 최소 토픽

| 토픽 | 타입 | 의미 | 필수 해석 |
|---|---|---|---|
| `/perception/lane/boundary_family` | `asmc_msgs/LaneBoundaryFamily` | 세 경계의 공동 metric 곡선과 sample별 근거 | freshness, `valid`, ODD, algorithm, support 검사 |
| `/perception/bev/camera/drivable_probability` | `sensor_msgs/Image` | 카메라 기반 주행가능 score `0~255` | coverage와 함께 사용 |
| `/perception/bev/camera/coverage` | `sensor_msgs/Image` | 한 카메라 이상이 기하적으로 관측한 cell은 255 | 0은 non-drivable이 아니라 unknown |

보조 토픽은 `/perception/bev/diagnostics`, `/perception/lane/diagnostics`다.
`/perception/bev/camera/source_count`는 기본 off이며 overlap 검수에서
`publish_bev_debug_metadata:=true`로 켠다. road-marking raster와 Marker는 분석·시각화용이며
판제의 최소 입력 계약이 아니다.

특히 `/planning/debug/lane_boundary_family/strips`는 reference adapter가 `x[0,15]m`에서
수락한 interval만 그리는 MarkerArray다. 이를 planning 데이터로 구독하거나 짧은 길이를
`boundary_family`의 유효거리로 해석하지 않는다. 정본 typed family는 전체 polyline과 sample별
support를 보존한다. 판제가 더 긴 horizon을 선택할 때도 `INVALID`를 연결하거나 추정으로
승격하지 말고, freshness·폭·support gate와 별도 far-range audit를 적용한다.

모든 최소 출력은 `header.frame_id=base_link`이고 x-forward, y-left다. timestamp는 callback
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
[`bev_grid.yaml`](../../../src/asmc_camera/config/bev/bev_grid.yaml)과 동결 기록의 SHA-256
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

## 6. 실행과 기록

인지 출력은 bridge·GUI와 같은 `asmc-ros-noetic` 컨테이너의 별도
terminal에서 실행한다. bag replay ROS 추론도 같은 컨테이너를 사용한다.
`asmc-perception-train`은 모델 학습과 학습 중 validation 전용이다.

```bash
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash
roslaunch asmc_camera perception.launch profile:=legacy \
  camera_checkpoint:=/root/ws/artifacts/perception_eval/runs/twinlite_medium_morai_v6_road_marking_init_b_b12_e18_20260908/best_mean.pt \
  lane_publish_pipeline_audit:=true
```

`boundary_family` 보강 fitting과 정본 발행은 운영 기본값이다. 성능 측정에서 marker
생성 비용만 빼려면 `lane_enable_shared_family_debug:=false`를 추가하고
`lane_enable_boundary_family_candidate` 기본값은 유지한다. 기존
`boundary_observations`는 호환·진단용 비정본이며 운영 기본에서는 발행하지
않는다. 기존 A/B 재현에서만
`lane_enable_independent_boundary_pipeline:=true`로 개별 spline·tracker와 함께 켠다.

UDP bridge와 read-only adapter는 `asmc-ros-noetic`에서 실행한다. adapter는 선택 사항이며
차량 제어를 발행하지 않는다. 아래 기본 명령은 `base_link` RViz와 최종 strip/perception
support marker를 자동으로 연다.

```bash
source /root/ws/devel/setup.bash
roslaunch lane_boundary_adapter shadow_boundary_family_candidate.launch
```

bag replay나 처리시간 측정에서는 RViz 부하를 제외한다.

```bash
roslaunch lane_boundary_adapter shadow_boundary_family_candidate.launch rviz:=false
```

성능 측정에는 raster/Marker를 기록하지 않는 `shadow_runtime_profile_result`를 쓴다. 판제
인수 replay에는 실제 최소 출력을 필수로 기록하는
`shadow_camera_perception_handoff_result`를 쓴다.

```bash
rosrun asmc_camera run_shadow_lane_bag_batch.py \
  --selection /root/ws/artifacts/perception_eval/analysis/shadow_maneuver_20260913/selection.yaml \
  --checkpoint /root/ws/artifacts/perception_eval/runs/twinlite_medium_morai_v6_road_marking_init_b_b12_e18_20260908/best_mean.pt \
  --model-id <unique-v3-handoff-model-id> \
  --enable-boundary-family-candidate \
  --result-profile shadow_camera_perception_handoff_result \
  --external-pipeline
```

위 runner와 inference pipeline은 `asmc-ros-noetic`의 별도 terminal에서 같은 ROS
master를 사용해 실행한다. `--external-pipeline`은 runner가 모델을 중복으로
띄우지 않고 recorder/replay만 소유하게 한다.

raw bag은 변경하지 않고 결과를
`/data/rosbags/results/<raw_run_id>/<model_id>/`에 둔다. planning 알고리즘 A/B도 같은 raw
또는 같은 handoff result를 재생해 입력을 고정해야 한다.

## 7. v3 동결 성능과 최적화 판단

대표 straddle, lateral sweep, Sunny/Foggy static avoid-return 4개 raw bag을 GUI 없이 격리
재생했다. 결과 bag 4개는 모두 `status=complete`이고 총 1,270 frame에서 다음을 확인했다.

| 항목 | 결과 |
|---|---:|
| family accepted | 1,270 / 1,270 |
| 최장 reject | 0 frame |
| runtime error | 0 |
| run별 family 출력 | 10.324 / 10.323 / 9.381 / 10.406 Hz |
| 전체 lane body p50 / p95 | 76.37 / 96.96 ms |
| shared fit p50 / p95 | 31.66 / 40.88 ms |
| sample observed / inferred / invalid | 67.22 / 11.50 / 21.29% |
| valid span p50 / p95 | 20.07 / 21.75 m |
| inlier residual p95의 frame 분포 p95 | 0.293 m |

네 bag에서는 v3의 distributed fallback이 0회였고 모두 strict path였다. 따라서 이 결과는
기존 정상 동작 비회귀와 처리량을 증명하지만 fallback의 실제 발생률·정확도를 과장하지
않는다. GUI, 두 visualizer, RViz와 map overlay를 함께 띄운 개발 snapshot에서는 lane
처리율이 약 7.5Hz, p95가 약 105ms까지 내려갔다. 이는 시각화 포함 stress 참고치이며
headless 판제 입력 기준선은 위 result-topic rate다.

새 handoff profile도 straddle bag 한 개로 smoke 검증했다. `boundary_family` 401개
(`10.29Hz`), drivable/coverage 각각 386개(`9.90Hz`)와 세 diagnostics가 기록됐고 모든 오류는
0이었다. 10.9MB LZ4 result bag의 SHA-256은
`a404a662b1d2da9b197f34f96e38f261c4b5391258aeae3b63192a9882bd99dd`다. 이는 recorder와
스키마 검증이며 판제 closed-loop 검증은 아니다.

결론은 **v3 geometry·threshold·상태 계약을 먼저 동결하고 판제에 넘긴다**이다. 9Hz gate는
통과했지만 여유가 크지는 않으므로 full-stack에서 9Hz를 깨면 다음 순서로 최적화한다.

1. 판제가 `boundary_family`를 실제 후보로 선택한 뒤에만 legacy independent spline,
   temporal tracker와 legacy typed publish를 끄는 opt-in profile 검토
2. 중복 projection audit/extract 순회 결합
3. shared fit의 basis 구성, robust iteration, residual lookup과 solve를 세분 profile

각 최적화는 같은 4개 bag에서 algorithm/config ID, support state, invalid hard split과 geometry
분포가 유지되는지 확인해야 한다. threshold 완화나 ROI 축소로 처리율만 올리는 것은 허용하지
않는다. 정본 수치·bag SHA-256은
[`freeze.yaml`](../../../artifacts/perception_eval/analysis/twinlite_v6_shadow_boundary_family_candidate_v3_profile_20260915/freeze.yaml)에
있다.

## 8. 판제 인수 체크리스트

- [ ] 입력 feature flag 기본값이 off이고 음영 ODD 판단이 판제 소유다.
- [ ] algorithm/representation/ODD와 source freshness를 검사한다.
- [ ] `valid=false`, missing과 stale을 구분해 기록한다.
- [ ] inferred를 observed보다 낮은 권한으로 쓰고 invalid interval은 잇지 않는다.
- [ ] yellow/white 역할을 home/opposing/차로 번호로 직접 치환하지 않는다.
- [ ] drivable score는 coverage와 함께 읽고 unknown을 non-drivable로 오인하지 않는다.
- [ ] drivable/coverage의 stamp·shape·encoding을 함께 검사하고 동결 grid hash를 pin한다.
- [ ] LiDAR occupancy/object가 camera score보다 충돌 veto 우선권을 가진다.
- [ ] 장애물 회피 후 route 문맥으로 원래 진행 차로 복귀를 결정한다.
- [ ] 같은 raw/result bag으로 feature off/on과 fallback을 비교한다.
- [ ] 실제 planning 부하에서 출력률·stale·closed-loop trajectory를 검증한다.
- [ ] 대회판 simulator에서 별도 회귀 검증 전 production 승격으로 표시하지 않는다.
- [ ] adapter core·config·test 흡수 후 동일 bag에서 strip/degraded/gap·diagnostics parity를 확인한다.
- [ ] standalone package 삭제 전 외부 launch·code·문서 참조 0, rollback 경로와 판제 owner 리뷰를 확인한다.
