# drivable_bev

> 상태: 3-camera fused metric BEV + metric lane-vector pilot + GPS 음영구간
> boundary selector. 12-run provisional gate와 typed provisional handoff 구현을 완료했으며,
> 전 구간 topology/판제 정본 계약은 아직 합의 전이다.

TwinLiteNet+의 원본 영상 크기 drivable probability와 4-class road-marking 결과를
카메라별로 `base_link` 지면 격자에 투영한 뒤 하나의 BEV로 융합한다. MGeo, GPS, LiDAR
없이 동작한다. 카메라별 출력은 calibration/debug용으로 유지하고 정식 camera-BEV 출력은
fused 토픽을 사용한다.

## 좌표와 기본 격자

```text
base_link: 후륜축 중앙
x: 전방 +, y: 좌측 +, z: 위 +
정적 노면: base_link에서 z=-0.35 m (z=0은 후륜축 중심 높이)
BEV 화면: 위=전방, 왼쪽=차량 좌측

x [-5, 20] m, y [-8, 8] m, 0.10 m/cell
출력 250 rows x 160 columns, ego=(col 80,row 200)
```

위 값은 `bev_grid.yaml`을 사용하는 정식 pilot 기본값이다. 개발 통합 launch는 원거리와
반대차로 방향의 열화를 숨기지 않고 보기 위해 별도 `bev_grid_dev.yaml`을 사용한다.

```text
development exploration grid: x [-5, 30] m, y [-12, 12] m, 0.10 m/cell
output: 350 rows x 240 columns, ego=(col 120,row 300)
```

개발 격자의 전체 범위는 정확도 검증 결과가 아니며 판제 ODD도 아니다. 현재 음영구간의
`0~10`, `10~15`, `15~20m` 구분은 거리별 열화를 비교하기 위한 **audit bin**이다. 이를
고정 control/preview 인터페이스로 해석하지 않는다. production launch와 설정은 개발 격자
추가로 변경되지 않았다.

OpenCV `(column,row)`과 metric 좌표의 변환은 `grid.py`만 소유한다.

```text
row = (x_max - x) / resolution
col = (y_max - y) / resolution
```

이 값은 명시적 ground plane을 적용한 뒤 v6 validation 전체 468개 complete 3-view frame
group에 계획된 threshold를 다시 적용한 결과다. `30/25/20 m × 12/10/8 m` 후보 중 모든
sunny/foggy·경계 구간 기준을 통과한 후보가 없어 계획에 명시한 fallback을 유지했다.
현재 fallback 범위의 drivable IoU는 sunny/foggy `0.9899/0.9814`, 통합 white/yellow IoU는
`0.6536/0.6215`, recall은 `0.6843/0.6514`다. z=0 평가보다 크게 개선됐지만 엄격한
recall `0.75` 및 외곽 구간 기준은 아직 넘지 못했다. 즉 이 범위는 실용적인 출력
상한이지 차선 전체가 강건하다는 보증이 아니다. 차선은 confidence와 거리 제한을 함께
사용해야 한다. audit은
`artifacts/perception_eval/analysis/twinlite_v6_bev_range_ground_v2_20260911/metrics.json`에 있다.
audit에는 각 후보의 5 m 종방향·4 m 횡방향 bin과 sunny/foggy·카메라별 GT 유효 픽셀 수도
함께 기록한다.

최초 `z=0` 범위 audit의 GT도 `384×640` 학습 cache에서 native 크기로 복원한 값이었다.
따라서 범위 선택 당시 Cloudy live 장면은 정확도 계산에 들어가지 않았지만, native GT에서
학습 크기로 줄어들 때의 얇은 차선 손실까지는 분리하지 못했다. 아래 native loss-budget
audit과 ground-plane 재평가로 이 한계를 보완했다. 기존 2026-09-10 `z=0` artifact는
삭제하지 않되 `superseded_due_to_ground_plane_contract`인 역사적 결과로만 보존한다.
현재 범위는 lane fitting 재평가 전까지 계속 임시 상한이다.

카메라 coverage 밖은 0이며 원거리 차선을 외삽하지 않는다. 원래 800×400 격자와 비교하면
cell 수는 87.5% 줄었다. 새 ground plane에서 세 카메라의 기하학적 union coverage는
87.9%, 2-view 이상 overlap은 43.8%다.

## 입력과 출력

각 view의 세 입력은 동일 source timestamp여야 한다.

```text
/perception/camera/{view}/drivable/probability
/perception/camera/{view}/road_marking/class_id
/perception/camera/{view}/road_marking/confidence
```

출력은 모두 `sensor_msgs/Image`, `mono8`, `frame_id=base_link`다.

```text
/perception/bev/debug/{view}/drivable_probability
/perception/bev/debug/{view}/road_marking/class_id
/perception/bev/debug/{view}/road_marking/confidence
/perception/bev/debug/{view}/coverage
/perception/bev/debug/diagnostics                 # per-view projector
```

세 view의 stamp 범위가 35 ms 이하인 묶음만 융합한다. 한 view가 빠지거나 35 ms를 넘으면
해당 frame을 발행하지 않는다. Drivable은 정적 기하 품질맵 기반 가중 평균, road marking은
`품질 × class confidence`가 가장 큰 view의 class를 선택한다. class ID는 평균하지 않는다.
BEV 통합 launch는 view 누락을 줄이기 위해 최대 20 ms 동안 batch를 coalesce하며, 독립
카메라 추론 launch의 기본 4 ms는 바꾸지 않는다.

정식 fused 출력은 모두 `base_link`, `mono8`이다.

```text
/perception/bev/camera/drivable_probability
/perception/bev/camera/road_marking/class_id
/perception/bev/camera/road_marking/confidence
/perception/bev/camera/coverage
/perception/bev/camera/source_count
/perception/bev/diagnostics                       # fused 정식 진단
```

차선 벡터 pilot은 native 확대나 lane raster를 거치지 않는 다음 model-grid 토픽을 사용한다.
추론 노드는 구독자가 있을 때만 이 배열을 복사·발행하며, 3-view batch 기준 최대 10 Hz다.

```text
/perception/camera/{view}/road_marking/model_grid/class_id   # 384x640 mono8
/perception/camera/{view}/road_marking/model_grid/confidence # max softmax, 0~255
```

기존 native-resolution 토픽은 하위 호환을 위해 그대로 유지한다.

음영 왕복 2차로 ODD에서 선별한 경계는 다음 typed provisional 토픽으로도 발행한다.

```text
/perception/lane/boundary_observations
    asmc_msgs/LaneBoundaryObservationArray
```

배열은 `base_link`에서 관측된 경계를 하나의 공통 `reference_x_m`에서 왼쪽(+y)부터
오른쪽(-y) 순서로 전달한다. 기본 기준은 5m지만 일부 경계가 그 지점까지 닿지 않으면 모든
경계가 공유하는 종방향 구간 안에서 5m에 가장 가까운 지점을 사용한다. 따라서 consumer는
고정 5m를 가정하지 않고 같은 frame의 모든 경계가 동일한 reference x를 갖는지 확인해야
한다. 각 원소에는 short-term `tracking_id`, white/yellow 색상, 관측 카메라 bitmask, metric
polyline, 유효 x 범위, semantic/geometric score와 bridge 비율이 들어간다.
`order_valid=false`이면 공통 support가 없어 순서를 확정하지 못한 것이므로 `lateral_order`와
`reference_y_m`을 차로 의미로 사용하면 안 된다. score는 calibrated probability가 아니다.

이 계약에는 `home_lane`, `opposing_lane`, 절대 차로 번호가 없다. 현재 producer가
`operating_domain=shadow_two_way_v1`, `expected_boundary_count=3`을 내보내는 것은 첫 ODD의
완성도 기준일 뿐 메시지 자체는 N개 경계를 표현한다. fresh valid empty와 stale/missing,
`valid=false`를 구분해야 하며, 판제는 부분 관측을 정상 상태로 처리해야 한다. 기존
`/perception/lane/boundaries` MarkerArray는 시각화·과거 bag 호환용으로 유지한다.

기본 시각화 `/perception/bev/debug/fused/compressed`는 하나의 fused BEV에 drivable=녹색,
white=흰색, yellow=노란색, stopline=빨간색과 metric grid를 표시한다. 상단 범례는
`x[min,max]m`, `y[min,max]m`, 해상도를 표시하며 영상 폭에 맞춰 자동 축소한다. 기존 3×3
mosaic는 `publish_per_view_mosaic:=true`일 때만 발행한다. JPEG debug는 구독자가 있을 때만
생성한다.

`/perception/bev/debug/fused_quality/compressed`는 drivable 채색을 제거한 개발용 품질
화면이다. 배경 밝기·색조는 한 cell을 관측한 카메라 수 `1/2/3`, white/yellow/stopline
밝기는 선택 class confidence를 나타낸다. coverage는 정확도 보증이 아니므로 이 화면도
판제 입력으로 사용하지 않는다. 원본 mono8 `/perception/bev/camera/road_marking/confidence`,
`/perception/bev/camera/source_count`도 rqt에서 직접 선택할 수 있다.

## Calibration

`config/cameras_v2.yaml`은 `ASMC_sensor_set_v2.json`의 최소 calibration snapshot이다.
기본 launch는 sensor-set SHA-256과 센서 ID, 해상도, FOV, pose를 검사하고 다르면
시작을 중단한다.

MORAI의 `base_link` 원점은 노면이 아니라 후륜축 중앙이므로 같은 snapshot에 노면을
`z = dz_dx*x + dz_dy*y + z_at_origin`으로 반드시 명시한다. 2026-09-11 두 구간 LiDAR
실측을 바탕으로 정적 기본값은 `z=-0.35 m`이며 모든 카메라 homography, coverage/GSD,
model-grid projector, camera reprojection과 offline evaluator가 같은 불변 `GroundPlane`을
사용한다. 항목이 없거나 유한하지 않으면 시작을 실패시키며 `z=0`으로 fallback하지 않는다.
카메라 mount z를 `1.20→1.55 m`로 변조하지 않는다.

Intrinsic은 native 해상도와 horizontal FOV로 계산한다. 센서셋의
`focalLengthpixel=320`은 front 1280×720/FOV 90도 등과 일치하지 않아 사용하지 않는다.
Pitch 양수=하향, yaw 양수=좌향인 MORAI mount frame을 OpenCV optical frame으로 바꾼다.
RGB grid overlay로 이 부호와 실제 노면 정합을 반드시 확인한다.

## 실행

ROS와 GPU 컨테이너를 올리고 UDP bridge를 실행한 뒤, GPU 추론까지 함께 시작한다.

```bash
# GPU perception 컨테이너
source /root/ws/devel/setup.bash
BEV_CHECKPOINT=/root/ws/artifacts/perception_eval/runs/twinlite_medium_morai_v6_road_marking_init_b_b12_e18_20260908/best_mean.pt
roslaunch drivable_bev three_view_bev_pilot.launch \
  checkpoint:="$BEV_CHECKPOINT"
```

이미 3-view 추론을 실행 중이면 중복 모델을 띄우지 않는다.

```bash
roslaunch drivable_bev three_view_bev_pilot.launch start_inference:=false
```

차선 evidence, spline과 모든 개발용 화면을 함께 점검할 때는 개발 launch 하나를 사용한다.
각 debug publisher는 항상 advertise되므로 rqt 목록에 나타나지만, JPEG decode/render/encode와
대용량 PointCloud 직렬화는 실제 구독자가 있을 때만 수행한다.
이 launch는 기본적으로 `bev_grid_dev.yaml`과 `visualization_dev.yaml`을 선택하므로
정식 pilot보다 넓은 `[-5,30]×[-12,12] m`가 표시된다.
검증된 shared lane-family도 이 개발 launch에서만 명시적으로 켜지며, 일반
`lane_vector_pilot.launch`의 기본값은 꺼짐이다.

```bash
# GPU perception 컨테이너
source /root/ws/devel/setup.bash
roslaunch drivable_bev three_view_perception_dev.launch \
  checkpoint:=/root/ws/artifacts/perception_eval/runs/twinlite_medium_morai_v6_road_marking_init_b_b12_e18_20260908/best_mean.pt
```

ROS GUI 컨테이너에서는 rqt를 토픽 인자 없이 열어 dropdown으로 화면을 바꿔 본다.

```bash
asmc-ros-gui rqt-image
```

helper는 임시 XDG 설정을 자동 생성해 `/root/.config/ros.org/rqt_gui.ini`에 남은 예전
모니터 좌표와 선택 토픽을 불러오지 않는다. CLI에서 compressed 영상을 바로 열 때도 base
topic으로 정규화하고 transport를 자동 선택한다.

```bash
asmc-ros-gui rqt-image /perception/lane/debug/shared_family_bev
```

우선 확인할 화면은 다음과 같다.

```text
/perception/camera/debug/mosaic/compressed
/perception/bev/debug/fused/compressed
/perception/bev/debug/fused_quality/compressed
/perception/bev/debug/mosaic/compressed
/perception/bev/debug/{front,left,right}/camera_grid_overlay/compressed
/perception/lane/debug/evidence_bev/compressed
/perception/lane/debug/fitted_bev/compressed
/perception/lane/debug/shared_family_bev/compressed
/perception/lane/debug/camera_reprojection_mosaic/compressed
```

판제 handoff와 result bag을 확인할 때는 typed 토픽을 직접 점검한다.

```bash
rostopic echo /perception/lane/boundary_observations -n1
rostopic hz /perception/lane/boundary_observations
```

2026-09-12 기존 Foggy 11시 raw bag 한 개를 새 `shadow_boundary_result` profile로 재생한
smoke에서는 typed boundary 240 frame이 평균 `10.195 Hz`로 기록됐고 recorder/diagnostics/error
기술 gate를 모두 통과했다. 이 중 5 frame은 세 경계가 같은 reference x를 통과하지 않아
`order_valid=false`였다. 따라서 consumer가 partial/invalid/order-invalid를 실제 분기로
처리해야 한다는 계약도 재현 데이터로 확인했다. 이 smoke는 geometry 정확도 재평가가 아니라
메시지·녹화·감사 배관 검증이다.

운용 처리량을 잴 때는 시각화 노드 자체도 제외할 수 있다.

```bash
roslaunch drivable_bev three_view_bev_pilot.launch start_visualizer:=false
```

개발 중 기본 viewer는 위 `rqt_image_view`다. GUI 전달 문제를 진단하거나 단일 fused
토픽만 고정해서 볼 때는 다음 `image_view`를 fallback으로 사용한다.

```bash
rosrun image_view image_view \
  image:=/perception/bev/debug/fused \
  _image_transport:=compressed \
  _autosize:=true
```

기존 3×3 화면이 필요하면 launch에 `publish_per_view_mosaic:=true`를 넘기고
`/perception/bev/debug/mosaic/compressed`를 구독한다.

정식 pilot에서 확장 격자를 일시적으로 재현해야 할 때만 다음처럼 명시적으로 설정한다.
기본값은 계속 작은 provisional grid다.

```bash
roslaunch drivable_bev three_view_bev_pilot.launch \
  bev_grid_config:=$(rospack find drivable_bev)/config/bev_grid_dev.yaml \
  visualization_config:=$(rospack find drivable_bev)/config/visualization_dev.yaml
```

카메라별 지면 격자도 확인한다.

```bash
rqt_image_view /perception/bev/debug/front/camera_grid_overlay \
  _image_transport:=compressed
```

배관만 확인할 때 sensor-set 검증을 끌 수 있지만 실제 평가에서는 사용하지 않는다.

```bash
roslaunch drivable_bev camera_bev.launch \
  views="front, left, right" verify_sensor_set:=false
```

## 검증

```bash
PYTHONPATH=src/perception/drivable_bev/src \
python3 -m unittest discover -s src/perception/drivable_bev/test -v

rostopic hz /perception/bev/debug/front/drivable_probability
rostopic hz /perception/bev/camera/drivable_probability
rostopic echo /perception/bev/diagnostics -n1
```

첫 fused live gate에서는 직선 차선 폭, 곡선 좌우 방향, 정지선 수직성, actor-negative hole,
seam과 중복 차선을 확인한다. 목표는 fused 출력 9 Hz 이상, timestamp sync 거부율 5% 미만,
fusion p95 10 ms 이하, camera-to-fused p95 130 ms 이하다.

### 2026-09-10 26.R1.H3 정지 장면 pilot

- v2 RGB는 세 view 모두 약 20 Hz로 들어왔고 BEV 출력은 source-time throttle 후 view별
  약 10 Hz였다.
- BEV projection+publish p95는 front `5.23 ms`, left `4.96 ms`, right `4.74 ms`였고
  세 view 모두 projection error는 0건이었다.
- RTX 4060 Ti에서 8개 진단 구간의 추론-only 처리량 중앙값은 약 `49.5 view-fps`,
  projector와 lazy visualizer를 함께 띄우고 debug 구독을 닫은 값은 약
  `47.9 view-fps`였다. 약 3.4% 차이는 짧은 live run의 변동을 포함한 참고값이다.
- RGB ground-grid는 노면 소실점과 맞았고, left/right coverage는 각각 차량 좌·우에
  배치되어 축 반전은 보이지 않았다.
- mosaic나 camera-grid를 실제 구독하면 JPEG 인코딩 부하가 추가된다. 처리량 측정에서는
  debug 구독을 닫거나 `start_visualizer:=false`를 사용한다.

위 수치는 정지 교차로 장면의 per-view plumbing/calibration gate다. 이후 validation audit로
격자를 `[-5,20]×[-8,8] m`로 축소했으며 fused live 수치는 별도 검증 기록으로 추가한다.

### 2026-09-10 fused live/replay gate

- 최종 live 설정은 `batch_wait_ms=20`, view timestamp 허용 범위 35 ms, fused 최대 10 Hz다.
- 충분히 워밍업한 26.R1.H3 live 누적 표본에서 fused `9.66 Hz`, sync 폐기율 `3.33%`,
  fusion p95 `3.90 ms`, camera-to-fused p95 `95.35 ms`였다.
- categorical softmax는 backend 안에서 class ID와 max confidence로 축약한다. 이를 통해
  confidence 전체 4채널을 native 해상도로 복원하던 초기 구현의 약 34 view-fps를 제거하고
  최종 추론 처리량을 약 `50 view-fps`로 회복했다.
- `camera_raw` run
  `20260910_044312_kcity2025_bev_fusion_pilot_cloudy_1pm_morai_auto`는 242.437초,
  RGB 세 view 각각 약 20.0 Hz, header regression 0으로 audit을 통과했다.
- 같은 raw를 1.0× replay한 result
  `twinlite_v6_fused_bev_smoke_r1`에는 camera output, per-view BEV와 fused BEV가 분리 저장됐다.
  replay의 마지막 구조화 진단은 fused `9.84 Hz`, sync 폐기율 `3.66%`, fusion p95
  `5.42 ms`, camera-to-fused p95 `111.54 ms`, 추론 `50.1 view-fps`, decode/inference error
  0건이었다. 이 replay는 최종값 직전의 12 ms coalescing이며 최종 20 ms 설정은 live에서
  더 낮은 누락률로 확인했다.

자동 범위 audit의 20 m 이내 raw overlap marking Chamfer p95는 목표 `0.35 m`를 넘었다.
서로 다른 물리 차선이나 한 view에서 누락된 선까지 nearest distance에 포함한 값이므로
extrinsic 오차만 뜻하지는 않지만, 목표 통과로 기록하지 않는다. lane fitting 단계에서
class·방향·길이로 대응되는 선끼리 매칭한 뒤 seam 오차를 다시 측정해야 한다. 현재 fused
raster는 calibration/debug pilot이며 바로 조향 기준선으로 사용하지 않는다.

### 2026-09-10 native GT BEV loss budget — superseded

> 이 절의 수치는 당시 모든 투영이 노면을 `base_link z=0`으로 잘못 가정한 역사적
> 기준선이다. artifact는 비교 재현을 위해 보존하지만 현재 성능값으로 사용하지 않는다.
> 대체 정본은 바로 다음 2026-09-11 절이다.

`scripts/analyze_camera_bev_loss_budget.py`로 v6 validation 1,404 sample, 468개 complete
3-view frame을 전수 평가했다. 정확도 입력은 Sunny/Foggy Capture뿐이며 Cloudy raw bag은
사용하지 않았다. native target policy, dataset/checkpoint/calibration/BEV hash와 전체 결과는
`artifacts/perception_eval/analysis/twinlite_v6_bev_loss_budget_20260910/`에 있다.

| Fused 경로 | Drivable IoU | White IoU / F1@0.2m | Yellow IoU / F1@0.2m | Stopline IoU / frame recall |
|---|---:|---:|---:|---:|
| native GT reference | 1.0000 | 1.0000 / 1.0000 | 1.0000 / 1.0000 | 1.0000 / 1.0000 |
| cache→native→hard BEV | 0.9853 | 0.4573 / 0.9504 | 0.6337 / 0.9820 | 0.7051 / 0.9934 |
| cache model-grid→hard BEV | 0.9910 | 0.5556 / 0.9742 | 0.7026 / 0.9889 | 0.7452 / 0.9934 |
| prediction→native→hard BEV | 0.9744 | 0.4142 / 0.7962 | 0.5607 / 0.8920 | 0.6574 / 0.9474 |
| prediction model-grid→hard BEV | 0.9769 | 0.4351 / 0.8400 | 0.6031 / 0.9130 | 0.6504 / 0.9474 |
| prediction model-grid→soft BEV | 0.9769 | 0.4034 / 0.7859 | 0.5903 / 0.8818 | 0.6649 / 0.9408 |

학습 cache 왕복의 낮은 exact IoU와 높은 tolerance F1을 함께 보면 native 축소는 선의
두께·위치를 주로 바꾸며 0.2 m 안에서 선 자체는 대부분 보존한다. native 확대를 생략한
direct hard 투영은 현재 prediction보다 white/yellow F1과 Chamfer를 모두 개선했다.
반면 단순 4-class 확률 평균은 Sunny/Foggy white F1이 각각 `-0.0101/-0.0105`, yellow가
`-0.0161/-0.0063` 변했고, white/yellow Chamfer p95 평균도 `0.135 m` 악화되어 정한
Soft-BEV gate를 통과하지 못했다. 겹침 영역의 background 확률이 얇은 foreground를
희석한 것으로 보이므로 현재 형태를 runtime에 이식하지 않는다.

당시 hard prediction의 거리별 F1@0.2m은 0~10 m에서 white `0.894~0.901`, yellow
`0.976~0.977`이나 15~20 m에서 각각 `0.486/0.669`로 낮다. 센서 해상도·주기는 이 결과의
원인 분리가 끝날 때까지 v2 값을 유지한다. Homography와 0.10 m raster는 모든 경로가
공유하는 표현 바닥이며 별도의 절대 지면 GT 오차로 과장해 해석하지 않는다.

재현 명령:

```bash
python3 scripts/analyze_camera_bev_loss_budget.py \
  --checkpoint artifacts/perception_eval/runs/twinlite_medium_morai_v6_road_marking_init_b_b12_e18_20260908/best_mean.pt \
  --output-dir artifacts/perception_eval/analysis/twinlite_v6_bev_loss_budget_<date>
```

### 2026-09-11 ground-plane 보정 및 재평가

좁은 구간과 GPS 음영구간에서 LiDAR 노면을 독립 측정했다. base-link 원점 노면 높이는
각각 `-0.354 m`, 20-frame 음영구간 중앙값 `-0.331 m`였고 둘 다 정적 기본값
`-0.35±0.02 m` 안이다. 보정된 RGB 역투영 차로폭은 좁은 구간 약 `3.00 m`, 음영구간
중앙값 `3.61 m`이며, 음영구간의 LiDAR 경계 중앙값 차이는 `3.52 m`였다. 같은 물리선의
front-left/front-right 횡잔차 p95는 6~15 m 공통 관측 구간에서 각각 `0.028/0.026 m`,
LiDAR 3D 도색점의 front RGB 재투영 p95는 `0~0.43 px`였다. 측정 원자료와 제한사항은
`artifacts/perception_eval/analysis/twinlite_v6_ground_plane_audit_20260911/metadata.json`에
있다.

동일 v6 EMA, 동일 Sunny/Foggy validation 468개 그룹으로 loss budget을 다시 계산했다.

| Fused 경로 | Drivable IoU | White IoU / F1@0.2m | Yellow IoU / F1@0.2m | Stopline IoU / frame recall |
|---|---:|---:|---:|---:|
| native GT reference | 1.0000 | 1.0000 / 1.0000 | 1.0000 / 1.0000 | 1.0000 / 1.0000 |
| cache→native→hard BEV | 0.9898 | 0.5388 / 0.9742 | 0.7017 / 0.9909 | 0.8008 / 1.0000 |
| cache model-grid→hard BEV | 0.9940 | 0.6350 / 0.9869 | 0.7662 / 0.9945 | 0.8225 / 1.0000 |
| prediction→native→hard BEV | 0.9827 | 0.5306 / 0.8924 | 0.6589 / 0.9530 | 0.7873 / 0.9848 |
| prediction model-grid→hard BEV | 0.9844 | 0.5438 / 0.9183 | 0.6954 / 0.9636 | 0.7787 / 0.9773 |
| prediction model-grid→soft BEV | 0.9844 | 0.5376 / 0.8907 | 0.7125 / 0.9525 | 0.7992 / 0.9848 |

잘못된 `z=0` 기준에 비해 prediction→native→hard의 white/yellow IoU는
`+0.1164/+0.0982`, F1@0.2m은 `+0.0962/+0.0610` 개선됐다. 이는 모델이 새로 좋아진 것이
아니라, 예측과 native GT를 올바른 물리 지면에 비교하게 된 효과다. 새 기준에서도
model-grid hard가 native hard보다 F1이 높다. 단순 soft 융합은 hard 대비 평균 F1이
`-0.0011`이고 Chamfer도 개선하지 못해 여전히 runtime에 채택하지 않는다.

prediction native-hard의 거리별 F1@0.2m은 Sunny/Foggy에서 0~15 m white
`0.965~0.927 / 0.928~0.906`, yellow `0.999~0.962 / 0.985~0.960`이다. 15~20 m는 white
`0.729/0.723`, yellow `0.893/0.854`로 낮아진다. 범위 audit은 전체 marking recall이
Sunny/Foggy `0.684/0.651`이고 외곽 구간이 더 낮아 모든 후보가 다시 실패했다. 따라서
`[-5,20]×[-8,8] m`는 계속 provisional이며 lane-vector 재평가 전에는 바꾸지 않는다.

BEV-only live에서는 fused `9.13 Hz`, fusion p95 `4.36 ms`, camera-to-fused p95
`98.23 ms`였다. projection p95는 front/left/right `3.72/3.54/2.78 ms`, 오류 0으로 기존
기준보다 회귀하지 않았다. 보정된 lane-vector는 30그룹 smoke에서 생성·artifact 저장까지
통과했지만 offline vectorization p95 `62.6 ms`이므로 파라미터 성능 채택으로 해석하지
않는다. 새 artifact는 각각
`twinlite_v6_bev_loss_budget_ground_v2_20260911/`,
`twinlite_v6_bev_range_ground_v2_20260911/`,
`twinlite_v6_lane_vector_ground_smoke_20260911/`이다.

### 2026-09-10 metric lane-vector pilot

`model-grid pixel → letterbox 제거 → native ray → 명시적 ground-plane 교점` lookup으로 white/yellow
evidence를 연속 `base_link (x,y)` 점으로 만든다. 이를 0.25 m 종방향 bin에서 횡방향
군집화하고, 같은 class의 node를 거리·방향 gate로 연결한 뒤 robust cubic B-spline으로
표현한다. B-spline은 control point를 갖는 연속 곡선이므로 vector 표현에 해당한다.
ROS Noetic 컨테이너에 SciPy를 새로 설치하지 않고 동일 basis와 robust weighted least
squares를 NumPy로 구현해 배포 의존성을 늘리지 않았다. 이 구현은 단위 테스트에서
결정론·outlier 제거·gap 표기를 검증한다.
다만 visible-only GT와의 정량 평가는 관측 evidence로 지지되는 spline sample만 사용하며,
점선 사이를 연결한 부분은 `bridged_length_ratio`와 60-frame 수동 검수 대상으로 분리한다.

중요하게도 현재 Semantic의 `white/yellow`는 **색상 표식**이지 lane-boundary instance가
아니다. 주차칸 테두리와 안전지대 사선처럼 같은 색의 비차로 경계도 포함될 수 있다.
현재 종방향 bin·최소 span은 짧은 횡단선을 일부 억제하지만, 진행 방향과 평행한 주차선까지
확실히 구별하지 못한다. 따라서 모든 spline을 `lane candidate`로만 부르고, 수동 검수 CSV에
`non_lane_marking_false_positive`, `parking_or_safety_zone_context`를 별도로 기록한다.
이 오검출률이 2% 이하라는 gate까지 통과하지 않으면 BEV 범위나 판제 lane boundary로
승격하지 않는다. 후속 판별은 방향·연속성·평행 경계 구조와 drivable 양측 문맥을 먼저
평가하고, 부족하면 MGeo 또는 lane-boundary 전용 라벨을 사용한다.

디버그 출력은 아직 planning 계약이 아니다.

```text
/perception/lane/debug/evidence_points             # PointCloud2 x,y,z,class/conf/quality/source
/perception/lane/debug/markers                     # sampled spline MarkerArray
/perception/lane/boundaries                        # shadow ODD selected/stable MarkerArray
/perception/lane/debug/evidence_bev/compressed
/perception/lane/debug/fitted_bev/compressed
/perception/lane/debug/boundaries_bev/compressed
/perception/lane/debug/shared_family/markers
/perception/lane/debug/shared_family/diagnostics
/perception/lane/debug/shared_family_bev/compressed
/perception/lane/debug/camera_reprojection_mosaic/compressed
/perception/lane/diagnostics
```

`/perception/lane/debug/shared_family/markers`에는 차선 support와 함께 실제 차량 제원의
디버그 footprint도 표시한다. 반투명 파란 상자는 차체 `x[-0.790,3.845]m`,
`y[-0.946,0.946]m`, 빨간 점은 후륜축 중앙인 `base_link`, 주황 가로선은 후륜축
`x=0`, 녹색 가로선은 전륜축 `x=3.0m`다. 이 표식은 좌표 기준일 뿐 lane fitting이나
판제 후보 계산에는 사용하지 않는다.

`/perception/lane/boundaries`는 왕복 2차로 GPS 음영구간에 한정한 provisional 출력이다.
색상 spline 전체를 먼저 만들지 않고 track 단계에서 종방향 support·heading·confidence를
검사해 노란 중앙 경계와 그 양쪽의 흰 outer boundary를 최대 3개로 줄인 뒤 cubic fitting한다.
각 Marker의 `text` JSON에는 stable local ID, 왼쪽→오른쪽 순서, white/yellow, metric valid-x,
confidence/score, source mask, 관측 길이와 bridge 비율이 들어간다. `home_lane` 또는
`opposing_lane` 의미는 붙이지 않으며 판제가 노란선과 ego 위치를 이용해 판단한다.
노란선이 순간적으로 보이지 않으면 분리된 흰 후보를 최대 2개까지만 `unresolved`로
발행하고 보이지 않는 경계를 새로 만들어내지 않는다.

시간축 tracker는 같은 class 곡선의 공통 x 구간 횡오차로 ID를 대응하고 짧은 누락 후
재검출을 허용한다. timestamp 역행 또는 0.75초 초과 단절에서는 상태를 초기화하므로 bag
경계나 시뮬레이터 reset을 가로질러 ID가 잘못 이어지지 않는다.

Sunny/Foggy validation 468개 complete 3-view frame을 native GT로 평가했다. 현재 유지한
설정은 confidence `0.50`, x bin `0.25 m`, lateral gap `0.30 m`, track gap `2.0 m`,
association residual `0.45 m`, 최소 관측 span `2.0 m`, spline smoothing `0.05`다.

| 방식/날씨 | White F1@0.2 / Chamfer p95 | Yellow F1@0.2 / Chamfer p95 |
|---|---:|---:|
| direct sunny | 0.809 / 0.945 m | 0.913 / 0.286 m |
| direct foggy | 0.785 / 1.835 m | 0.865 / 0.738 m |
| fitted sunny | 0.733 / 2.811 m | 0.898 / 1.154 m |
| fitted foggy | 0.733 / 3.054 m | 0.847 / 0.900 m |

현재 association+spline은 중복 spline이 468 frame 중 77 frame에서 발생했고 median lane
수 6, bridged ratio p95 0.87이었다. offline vectorization p95 `83.0 ms`, live 초기
측정은 약 `6 Hz`, processing p95 `61.7 ms`, camera-to-vector p95 `137.6 ms`로 목표도
통과하지 못했다. 따라서 spline을 판제 입력으로 채택하지 않고, 정확도 기준선은 direct
metric evidence로 유지한다.

장시간 debug launch 누적 snapshot에서도 input pair/queue 오류는 0이었지만 output
`6.45 Hz`, processing p95 `85.7 ms`, camera-to-vector p95 `174.5 ms`였다. 이 값은 rqt
구독과 시뮬레이터 중단·재연결 구간을 포함하므로 steady-state benchmark와 분리해야 하지만,
현재 구현이 실시간 gate를 통과했다는 근거로 사용할 수는 없다.

confidence `0.60`은 정확도가 악화됐다. x bin `0.40 m`와 최소 span `4 m`는 중복 선을
줄였지만 recall/F1도 낮췄다. 같은 evidence의 shape-preserving piecewise cubic도 유의미한
개선을 주지 않아 채택하지 않았다. 수치를 보기 좋게 만들기 위해 ROI를 줄이거나 선을
과도하게 smoothing하지 않는다.

최종 468-frame 재평가 artifact는
`artifacts/perception_eval/analysis/twinlite_v6_lane_vector_candidate_gate_20260910/`에
있다. 이 artifact의 p95는 `83.0 ms`이며, 60개 overlay는 전체 validation 순서에 균등
분포한다. `manual_review.csv`의 false bridge와 비차로 표식 항목은 사람이 검수하기 전까지
의도적으로 비어 있고 `manual_review_complete=false`다.

재현 가능한 산출물은 다음 스크립트로 만들며 기존 경로를 덮어쓰지 않는다.

```bash
python3 scripts/analyze_camera_lane_vectors.py \
  --checkpoint artifacts/perception_eval/runs/twinlite_medium_morai_v6_road_marking_init_b_b12_e18_20260908/best_mean.pt \
  --output-dir artifacts/perception_eval/analysis/twinlite_v6_lane_vector_<date>

# vector gate와 60-frame 수동 검수가 통과한 뒤에만 실행
python3 scripts/analyze_camera_lane_range.py \
  --vector-audit artifacts/perception_eval/analysis/twinlite_v6_lane_vector_<date> \
  --output-dir artifacts/perception_eval/analysis/twinlite_v6_lane_range_<date>
```

### GPS 음영구간 12-run replay matrix

평가 입력은 `shadow_clear/shadow_static_avoid_v1 × Sunny/Foggy × 11/13/15시`의 12개
wheel raw bag이다. raw 디렉터리명에 `morai_auto`가 남은 세 run은 수집 당시 controller
인자만 잘못 입력된 것이므로 원본 lineage를 바꾸지 않고 selection manifest의
`effective_controller: wheel`로 정정한다. 장애물 없는 6개는 후보·tracker 설정 선정에,
정적 장애물 6개는 회피·복귀 강건성 검증에 사용한다.

headless batch는 paused replay → inference/BEV/boundary → result recorder → replay start
순서를 run마다 지키며 raw를 수정하지 않는다. 기존 complete result는 덮어쓰지 않는다.
각 run을 닫을 때 camera/BEV/lane diagnostics, boundary MarkerArray, lane output/error count를
기술 품질 gate로 확인한다. 경계가 0개인 프레임은 인지 성능 결과이지 recorder 실패가
아니므로 별도로 집계한다.

```bash
source /root/ws/devel/setup.bash
rosrun drivable_bev run_shadow_lane_bag_batch.py \
  --selection /data/rosbags/audits/shadow_dataset_20260912/selection.yaml \
  --checkpoint /root/ws/artifacts/perception_eval/runs/twinlite_medium_morai_v6_road_marking_init_b_b12_e18_20260908/best_mean.pt \
  --model-id twinlite_v6_shadow_boundary_v3 \
  --rosbag-root /data/rosbags
```

현재 기본 Docker 구성은 dataset 보호를 위해 `asmc-perception-train:/data`가 read-only이고
`asmc-ros-noetic:/data`만 writable이다. 따라서 실제 팀 구성에서는 GPU 컨테이너에
headless pipeline을 한 번 실행하고, ROS 컨테이너에서 `--external-pipeline`으로 replay와
result recorder를 관리한다.

```bash
# terminal 1: asmc-perception-train
roslaunch drivable_bev three_view_lane_pipeline.launch \
  checkpoint:=/root/ws/artifacts/perception_eval/runs/twinlite_medium_morai_v6_road_marking_init_b_b12_e18_20260908/best_mean.pt

# terminal 2: asmc-ros-noetic
rosrun drivable_bev run_shadow_lane_bag_batch.py \
  --selection /data/rosbags/audits/shadow_dataset_20260912/selection.yaml \
  --checkpoint /root/ws/artifacts/perception_eval/runs/twinlite_medium_morai_v6_road_marking_init_b_b12_e18_20260908/best_mean.pt \
  --model-id twinlite_v6_shadow_boundary_v3 \
  --rosbag-root /data/rosbags \
  --external-pipeline
```

external pipeline의 진단 카운터는 여러 run에 걸쳐 누적될 수 있다. run별 실제 기록 수는
result metadata의 `topic_statistics`를 사용하고, 경계 가용성·ID 평가는 각 result bag
안의 MarkerArray만 사용한다.

명령과 조합만 검증할 때는 `--dry-run`, 중단 뒤 complete result를 건너뛸 때는 `--resume`,
한 run smoke에는 `--only-run-id <run_id>`를 사용한다. 결과는
`results/<raw_run_id>/<model_id>/`, 전체 요약과 프로세스 로그는 selection manifest 옆에
저장한다. replay 전에는 동일 raw topic을 발행하는 live UDP bridge를 반드시 종료한다.

12개 결과가 모두 생성되면 다음 감사기로 프레임별 경계 가용성과 시간축 일관성을
집계한다.

```bash
rosrun drivable_bev audit_shadow_lane_results.py \
  --selection /data/rosbags/audits/shadow_dataset_20260912/selection.yaml \
  --model-id twinlite_v6_shadow_boundary_v3 \
  --rosbag-root /data/rosbags
```

산출물은 selection manifest 옆의
`twinlite_v6_shadow_boundary_v3_boundary_audit.yaml`이다. run/날씨/시나리오별 any-boundary,
yellow-centre, 3-boundary corridor 완성률과 unresolved 비율, stable-ID 교체·재획득,
중복 role 및 lateral-order 오류를 기록한다. `static_minus_clear`는 장애물 회피 조건의
강건성 저하를 바로 볼 수 있게 정적 장애물 결과에서 clear 결과를 뺀 값이다. 이 감사는
가용성·시간 일관성 평가이며 metric GT 정확도나 home/opposing 의미 판정은 아니다.

### 2026-09-12 음영구간 replay 결과

12개 raw run은 모두 기술 품질 gate를 통과했다. 반복 replay 초기에 외부 pipeline이 이전
bag의 source-time deadline을 유지해 다음 bag을 영구 throttle하는 문제가 발견됐다.
카메라 model-grid와 per-view/fused BEV가 100 ms 이상 timestamp 역행을 새 replay/simulator
epoch로 처리하도록 수정했다. 최초 `twinlite_v6_shadow_boundary_v1` 결과는 실패 감사용으로
보존하고 정본으로 사용하지 않는다.

10 Hz model-grid 상한의 v2와 15 Hz 상한의 v3를 같은 12 run에서 비교했다. 15 Hz는 모델
추론 주기가 아니라 model-grid publish cap이며 fused BEV는 계속 독립적으로 최대 10 Hz다.
v3는 boundary 실제 rate를 clear `9.56~10.22 Hz`, static `10.08~10.43 Hz`로 올렸고,
fused BEV도 전체 `9.55~10.07 Hz`를 유지했다. lane vectorization p95는 v2
`43.80~45.95 ms`, v3 `43.26~46.21 ms`로 유의미한 회귀가 없었다. 따라서 headless
lane pipeline 기본은 15 Hz cap으로 정한다.

| 조건 | frames | yellow centre | left outer | right outer | 3-boundary complete | unresolved |
|---|---:|---:|---:|---:|---:|---:|
| clear 6 runs | 1,572 | 98.03% | 94.91% | 85.62% | 82.95% | 1.97% |
| static avoid 6 runs | 1,201 | 96.25% | 88.43% | 77.52% | 71.27% | 3.75% |

모든 frame에서 경계가 하나 이상 나왔고 resolved role 중복과 좌우 순서 오류는 0건이었다.
run 경계를 제외한 stable-ID 교체는 clear 10건, static 13건이었다. 정적 장애물에서 yellow
가용성 하락은 `1.77%p`로 작지만 세 경계 완성률은 `11.68%p` 하락했고, 주로 right outer
boundary가 빠졌다. 따라서 이 출력은 판제가 interface와 회피·복귀 로직을 개발하기 위한
**provisional 관측**으로는 사용 가능하지만, 세 경계를 항상 가정하거나 단독 steering
기준으로 사용하는 것은 금지한다. 각 경계의 role/color/polyline/confidence와 누락 상태를
그대로 소비해야 한다.

replay의 bag receipt clock과 원본 camera header가 다른 clock domain인 run에서는
camera-to-fused/end-to-end age를 계산하지 않는다. 기존 v3 metadata의 표본 없는 `0.0 ms`는
실제 zero latency가 아니며 코드에서는 `unavailable`로 명확히 표시하도록 정정했다.
연산시간은 replay에서도 유효하지만 실제 end-to-end latency는 live UDP에서 다시 측정한다.

정본 산출물:

```text
/data/rosbags/audits/shadow_dataset_20260912/selection.yaml
/data/rosbags/audits/shadow_dataset_20260912/twinlite_v6_shadow_boundary_v3_batch_summary.yaml
/data/rosbags/audits/shadow_dataset_20260912/twinlite_v6_shadow_boundary_v3_boundary_audit.yaml
/data/rosbags/results/<raw_run_id>/twinlite_v6_shadow_boundary_v3/
```

### 2026-09-13 차로 이탈·회피 maneuver replay

중앙선 straddle, 좌우 lateral sweep, 정적 장애물 회피·복귀 Sunny/Foggy의 raw bag 4개를
추가로 기록했다. 모두 `camera_raw`, v2 UDP, receive timestamp이며 카메라 약 19.86~19.91Hz,
Ego 약 48.9~49.2Hz로 정상이다. 이 set은 날씨·시간 조합 정확도 matrix가 아니라 차량이
차로 중앙을 벗어날 때의 시간 연속성과 경계 가용성 A/B용이다.

기존 producer는 모든 경계가 고정 `reference_x=5m`를 지나야만 순서를 유효하게 했다.
차량이 중앙선을 밟거나 가로질러 side-camera 곡선의 전방 support가 짧아진 frame에서도
경계 자체는 존재했지만, 이 조건 때문에 lateral order가 길게 무효화됐다. 공통 support
안에서 5m에 가장 가까운 지점을 고르는 계약으로 바꾼 뒤 네 run의 `order_invalid`는 모두
0이 됐다.

| 조건 | 기존 3-boundary complete | 기존 order-invalid | 공통 기준점+tracker complete | 최종 stable ID 수 |
|---|---:|---:|---:|---:|
| 중앙선 straddle Sunny 13시 | 41.75% | 212/400 | 94.50% | 4 |
| lateral sweep Sunny 13시 | 29.87% | 174/375 | 73.87% | 8 |
| 정적 회피·복귀 Foggy 11시 | 77.89% | 17/199 | 83.76% | 4 |
| 정적 회피·복귀 Sunny 13시 | 67.38% | 32/282 | 76.82% | 4 |

tracker는 최대 8 frame의 재획득 창과 관측 lateral velocity를 사용한다. 누락 frame에
가상 perception boundary를 발행하지 않으며, 다시 관측된 같은 물리선의 local ID만 더
안정적으로 연결한다. 특히 straddle의 stable ID switch는 2→1, lateral sweep은 공통
기준점-only 결과의 9→5, Sunny 회피·복귀는 7→1로 줄었다.

yellow confidence를 `0.50→0.45`로 낮춘 실험은 가용성을 늘리지 못하고 일부 지표만 소폭
악화시켜 채택하지 않았다. 최종 threshold는 0.50이다. 네 run 1,261 frame에서 실제
white-yellow pair는 1,119 frame이었다. yellow가 잠시 보이지 않지만 두 white outer가
남은 131 frame은 perception 출력에 중앙선을 합성하지 않고, 별도 planning adapter가
왕복 2차로 폭·공통 길이·폭 변화 gate를 통과할 때만 `degraded_inferred_center`로 표시한다.
실제 adapter replay 수신 1,258 frame 중 정상 관측 strip 1,090(86.65%), degraded 131
(10.41%), unusable 37(2.94%)로 총 usable은 97.06%였다. 이 수치는 차선 정확도나 안전한
closed-loop 주행 합격률이 아니라 인터페이스 연속성 기준선이다.

로컬 A/B 산출물은
`artifacts/perception_eval/analysis/shadow_maneuver_20260913/`, result bag은
`/data/rosbags/results/<raw_run_id>/twinlite_v6_shadow_maneuver_robust_v1_20260913/`에 둔다.
raw bag은 수정하지 않았다.

### 2026-09-13 음영구간 진단·map pseudo-GT gate

joint lane-family를 구현하기 전 첫 gate를 완료했다. `/perception/lane/pipeline_audit`는 매
lane output stamp마다 model pixel→metric evidence→node→track→prefit→spline→boundary→
temporal 결과와 단계별 rejection reason을 `DiagnosticArray`로 발행한다. 누적 counter만
저장하지 않고 frame payload도 함께 기록하므로 result bag마다 처음부터 다시 집계할 수 있다.
`shadow_boundary_result`와 `camera_semantic_result` 프로필에는 이 토픽을 optional로 추가해
이전 bag 호환을 유지했다. 이전 12-run/4-maneuver result에는 이 토픽이 없으므로 새 baseline
audit는 `pipeline_summary.available=false`를 명시하며, 탈락 원인 분포는 다음 replay부터
얻는다.

Marker는 producer와 C++ consumer 모두 매 frame `DELETEALL`을 발행하지 않는다. 현재
`(namespace,id)`는 `ADD`, 사라진 ID만 `DELETE`해 화면 깜박임과 canonical boundary
dropout을 분리했다. 메시지 freshness/invalid 처리는 그대로 유지한다. 라이브 smoke에서
typed boundary와 pipeline audit가 각각 66 message/7.45초로 기록됐고 boundary 출력은 약
8.36Hz였다. 장시간 smoke 중 한 frame에서 발견된 smoothing spline의 evidence 유효범위와
공개 polyline 범위 불일치는 두 범위의 교집합만 공개하도록 고쳤다. 수정 후 약 50초 live
run은 pipeline error 없이 complete 3-boundary를 발행했고 Ctrl-C도 닫힌 ROS topic 예외 없이
종료됐다.

음영구간 정본 설정은 `config/shadow_zone_kcity2025.yaml`이다. sample scenario의
`shadedAreaList.UNIQUEID=12`와 link `A2256W000126/A2256W000131`을 명시하고, MORAI scenario
yaw를 map 장축으로 바꾸는 `-90°` 규약을 설정에 기록했다. 생성기는 반대 방향 link를 같은
arc-length 방향으로 정렬하고 중간 yellow와 법선 offset white 두 개를 만든다.

```bash
rosrun drivable_bev build_shadow_lane_gt.py \
  --config /root/ws/src/perception/drivable_bev/config/shadow_zone_kcity2025.yaml \
  --data-root /data/R_KR_PG_K-city_2025 \
  --output-dir /root/ws/artifacts/perception_eval/analysis/<new_map_gt_run>
```

최초 산출물은 0.5m 간격 231 point, yellow 길이 115.71m, link 중심선 간격
p05/median/p95 `3.490/3.511/3.544m`, nominal lane width 3.5m다. 최초 lateral-only
보정은 road center `-0.18m`, 양쪽 폭 `3.47m`를 제안했지만, 가까운 곳은 맞고 거리에
비례해 세 경계가 같은 방향으로 밀리는 원거리 잔차가 남았다. 이 형태는 폭이나 평행이동
오류가 아니라 map→base heading 편차다.

2026-09-12 Sunny/Foggy 음영 12-run의 Intensity LiDAR 도색점 8,857개로 heading 후보마다
세 boundary offset을 다시 추정하는 joint search를 수행했다. 최적 offset은
`ego_heading_offset=-1.25°`였고, 이때 권장 road center offset은 약 `+0.005m`,
positive/negative 폭은 `3.484/3.521m`였다. 설정에는 각각 `0.00m`, `3.48m`, `3.52m`로
반올림했다. `-1.25°`는 이 음영 map pseudo-GT의 map→base 변환에만 적용하며 차량 heading,
카메라 extrinsic 또는 운용 좌표계를 전역 변경하지 않는다. 10/20/25m에서 각도만으로 생기는
횡오차는 각각 약 `0.22/0.44/0.55m`여서 관찰된 원거리 패턴과 일치한다.

동일 48-frame 최종 audit에서 pseudo-boundary 0.2m 이내 고반사점 비율은 최초
`18.749%`에서 `37.854%`로 증가했고 0.6~1.2m 대조구간은 `0.000%`를 유지했다. 세 curve의
front 투영은 48/48 frame에서 유효했다. LiDAR 3D→RGB 직접 재투영 p95도 최대 `0.43px`이므로
camera intrinsic/extrinsic이나 ground plane을 화면에 맞춰 변조할 근거는 없다. 이 결과도
paint survey가 아니므로 상태는 `map_pseudo_gt`, `deployment_allowed=false`다.

```bash
rosrun drivable_bev calibrate_shadow_lane_gt.py \
  --raw-root /data/rosbags/raw \
  --run-pattern '20260912_*shadow*' \
  --pseudo-gt /root/ws/artifacts/perception_eval/analysis/shadow_lane_map_pseudo_gt_20260913/shadow_lane_pseudo_gt.json \
  --ego-heading-offset-deg -1.25 \
  --output /root/ws/artifacts/perception_eval/analysis/<new_lidar_calibration>/calibration_heading_m125.json
```

```bash
rosrun drivable_bev audit_shadow_lane_map_alignment.py \
  --raw-run-root /data/rosbags/raw/20260912_001454_kcity2025_shadow_clear_sunny_1pm_wheel \
  --pseudo-gt /root/ws/artifacts/perception_eval/analysis/shadow_lane_map_pseudo_gt_pose_v4_20260914/shadow_lane_pseudo_gt.json \
  --calibration /root/ws/src/perception/drivable_bev/config/cameras_v2.yaml \
  --output-dir /root/ws/artifacts/perception_eval/analysis/<new_alignment_audit> \
  --sample-count 48
```

Sunny 13시 clear bag의 shadow polygon 내부 camera/LiDAR 각 48 frame에서 자동 gate를
통과했다. 세 curve는 front image 48/48에 투영됐고, LiDAR ground point 중 pseudo-boundary
0.2m 이내의 intensity≥40 비율은 최종 보정 후 `37.854%`, 0.6~1.2m 대조 구간은
`0.000%`였다. 거리는 0.5m sample 점이 아니라 각 polyline segment까지의
최단거리로 계산했다. contact sheet도 함께 남겼다. 이는 map-camera-LiDAR 좌표 정합의
강한 근거지만 사람이 이미지 전체를 검수하기 전까지 정식 GT로 승격하지 않는다.

재현 산출물:

```text
artifacts/perception_eval/analysis/shadow_lane_map_pseudo_gt_20260913/
artifacts/perception_eval/analysis/shadow_lane_map_alignment_20260913/
artifacts/perception_eval/analysis/shadow_lane_lidar_calibration_20260913/
artifacts/perception_eval/analysis/shadow_lane_map_pseudo_gt_lidar_v2_20260913/
artifacts/perception_eval/analysis/shadow_lane_map_alignment_lidar_v2_20260913/
artifacts/perception_eval/analysis/shadow_lane_heading_calibration_20260914/
artifacts/perception_eval/analysis/shadow_lane_map_pseudo_gt_pose_v4_20260914/
artifacts/perception_eval/analysis/shadow_lane_map_alignment_pose_v4_20260914/
artifacts/perception_eval/analysis/shadow_diagnostic_gt_baseline_20260913/
```

`*_lidar_v2_20260913`은 lateral-only 중간 결과, `*_pose_v4_20260914`가 joint heading을
포함한 현재 검수 기본값이다. `pose_v3`는 탐색 중 생성된 예비 결과이므로 사용하지 않는다.

마지막 baseline 디렉터리는 기존 12-run v3와 4-maneuver independent spline 결과를 새
audit schema로 고정한다. 기술 gate는 16/16 run이 통과했다. shared lane-family A/B에서는
같은 selection과 raw bag을 사용하고 이 baseline을 덮어쓰지 않는다.

#### 주행 중 map pseudo-GT 육안 검수

추론 마스크를 거치지 않고 전·좌·우 원본 RGB를 `base_link` metric BEV로 직접 합친 뒤,
동시각에 가장 가까운 `/Ego_topic` pose로 map pseudo-GT 세 선만 변환해 겹쳐 볼 수 있다.
이 검수 전용 grid는 `x[-5,25]m`, `y[-8,8]m`, `0.05m/cell`이며 기존 semantic BEV 설정이나
판제 계약을 바꾸지 않는다.

```bash
roslaunch drivable_bev shadow_map_rgb_bev.launch
```

현재 컨테이너에서 rqt로 볼 때는 transport suffix를 빼고 base topic을 선택한다.

```bash
bash /root/ws/docker/ros-noetic/asmc-ros-gui rqt-image \
  /perception/bev/debug/shadow_map_rgb
```

`rqt_image_view`의 dropdown에서 다음 두 토픽을 번갈아 선택한다.

- `/perception/bev/debug/shadow_rgb/compressed`: GT가 없는 3카메라 raw RGB BEV
- `/perception/bev/debug/shadow_map_rgb/compressed`: 같은 RGB BEV + yellow/map center와
  magenta/map outer pseudo-boundary

두 토픽은 항상 advertise되지만 실제 decode·warp·JPEG encode는 구독자가 있을 때만 한다.
카메라 세 장은 35ms gate로 결합하고 map overlay는 camera median stamp에서 40ms 이내인
가장 가까운 Ego pose만 사용한다. 시간차와 drop은
`/perception/bev/debug/shadow_map_rgb/diagnostics`에 기록한다. 이 화면은 수동 검수 도구이며
`deployment_allowed=false`인 pseudo-GT를 정식 정답이나 판제 입력으로 승격하지 않는다.

range evaluator는 정확히 같은 `range_inputs.npz`와 checkpoint/dataset/calibration/config
hash를 소비한다. 현재 vector gate와 수동 false-bridge 검수가 끝나지 않았으므로
`x[-5,20]m, y[-8,8]m`는 계속 `provisional`이며, 새 범위를 validated로 승격하지 않았다.
최종 range audit은 Sunny/Foggy 각각의 drivable, white/yellow F1·recall, 근/원거리
Chamfer와 GT support를 모두 검사한다. 어떤 후보도 최소 `x[0,15]×[-6,6]m`를 통과하지
못하면 좁은 ROI를 성공으로 선택하지 않고 원인을 model/calibration/association으로 남긴다.

설계 참고는 segmentation evidence의 곡선화를 다룬
[Differentiable Least-Squares Fitting](https://openaccess.thecvf.com/content_ICCVW_2019/html/CVRSUAD/Van_Gansbeke_End-to-end_Lane_Detection_through_Differentiable_Least-Squares_Fitting_ICCVW_2019_paper.html)과
차량 좌표계 후처리 예시인
[Apollo dense lane postprocessor](https://apollo.baidu.com/docs/apollo/9.x/denseline__lane__postprocessor_8cc_source.html)를
사용했다. external의 CLRNet·CLRerNet·UFLDv2는 현재 runtime 의존성이 아니라, 후처리로
모델 오차를 해결하지 못할 때 같은 데이터로 비교할 모델 후보일 뿐이다.

## 다음 단계

대회 우선범위는 전 구간 일반화가 아니라 GPS 음영 왕복 2차로의 장애물 회피·복귀다.
TwinLiteNet+을 다른 lane network로 교체하지 않고 현재 model-grid evidence와 B-spline 배관을
보강한다. 다음 순서를 정본으로 사용한다.

1. **완료 — 진단·GT 정본화:** evidence filter, node association, track split,
   spline rejection, boundary selection, temporal reacquisition과 planning gate의 탈락 사유를
   프레임별 counter로 분리한다. canonical message dropout과 매 frame `DELETEALL`에 따른
   Marker 깜박임도 구분한다.
2. **대표 gate 완료 — instrumented replay:** clear/회피/횡이동 대표 4개 raw를 새 진단
   토픽과 함께 재생해 독립 spline의 주 탈락 지점이 boundary residual gate임을 확인했다.
   shared 후보가 대표 A/B를 통과한 뒤 12-run+4 maneuver 전체로 확장한다.
3. **완료 — shared-family offline A/B:** 세 독립 spline 대신 하나의 공통 cubic B-spline과
   법선 방향 좌·우 폭을 robust weighted least squares로 추정한다. `3.48/3.52m`는 hard
   고정값이 아니라 soft prior이며, 두 경계 이상이 관측될 때만 누락 역할을
   `structure_inferred`로 만든다. 대표 4개 bag의 정량 gate를 통과했고, 공개 판제 출력은
   바꾸지 않은 채 아래 5단계에서 debug runtime sidecar로만 이식했다.
4. **완료 — support·허위 연결 대표 gate:** fitted role에서 0.35m 이내인 실제 metric
   evidence만 `observed`로 인정하고, 0.5m 미만의 짧은 support island를 제거한다. 세 역할 중
   같은 station에서 두 역할 이상이 지지할 때만 family를 유효하게 하며, 그 support의 내부
   공백만 최대 0.75m까지 연결한다. 한 역할만 남은 앞·뒤 꼬리는 `invalid`로 잘라낸다.
   고잔차 75 frame 전수 계측과 생성된 54개 고잔차 overlay 대표 검수에서 명백한 횡단
   false bridge는 발견되지 않았다.
5. **완료 — 전체 matrix와 debug runtime:** 보수화된 후보를 12-run+4 maneuver 전체로
   확대해 조건·조향별 비회귀를 확인하고, 상태별 Marker·rqt 영상·diagnostics sidecar를
   개발 launch에 연결했다.
6. **다음 — 직접 주행·최적화:** 상태 전환, 깜박임과 false bridge를 직접 검수하고 shared
   fitting 비용을 줄인다. 그 뒤 Ego/IMU 기반 ego-motion 보상과 0.2~0.3초 temporal
   prediction을 붙여 runtime에 이식한다. 관측/구조추정/시간예측/invalid 상태를 구분하고
   home/opposing 의미는 붙이지 않는다.
7. 음영구간 closed-loop에서 장애물 회피·반대편 진입·복귀를 판제와 공동 검증한 뒤 거리별
   유효 x 범위와 BEV display crop을 재평가한다.
8. 원거리 yellow evidence 자체가 부족하다는 결과가 남을 때만 음영 데이터 fine-tuning이나
   lane 전용 모델을 별도 비교한다. 일반 구간 MGeo matching과 센서 Hz/해상도 최적화는 그
   이후다.

외부 연구 snapshot은 `external/baselines/{LaneDetection_End2End,LaneAF,RVLD,
pytorch-auto-drive,StructLane}`에 있다. LaneCPP와 BSNet은 공개 구현이 확인되지 않아 논문만
참고한다. runtime에서 외부 repository를 import하지 않으며, 라이선스가 없거나 비상업
조건인 소스를 팀 코드에 복사하지 않는다. 구체적인 수식·GT·gate는
[`perception-bev-lane-interface-plan.md`](../../../docs/packages/asmc_perception/bev-lane-interface-plan.md#p4--음영구간-joint-lane-family-보강)를
따른다.

센서 해상도·10/15/20 Hz 최적화는 camera-to-vector 기준선이 완성된 뒤 정확도·지연 저하
없이 부하를 줄이는 마지막 단계로 미룬다.

현재 runtime은 `src/asmc_hd_map/development/won_tae_link_node`, LBC, 외부 `morai-roach`/연구 repository를 import하지 않는다.
기존 camera native/model-grid 토픽과 typed boundary 공개 계약은 하위 호환한다.

### 2026-09-14 representative pipeline-audit replay

instrumented replay의 첫 gate로 clear Sunny/Foggy 각 1개와 lateral sweep Sunny,
static avoid-return Foggy 각 1개를 별도 model ID
`twinlite_v6_shadow_pipeline_audit_v1_20260914`로 기록했다. 네 result는 모두 recorder 기술
gate를 통과했고 `/perception/lane/boundary_observations`와
`/perception/lane/pipeline_audit`가 약 `9.57~10.25Hz`로 일대일 기록됐다.

| 대표 조건 | frames | yellow | 3-boundary complete | 최장 incomplete |
|---|---:|---:|---:|---:|
| clear Foggy 13시 | 373 | 100.00% | 88.47% | 6 frames |
| clear Sunny 13시 | 304 | 100.00% | 82.57% | 14 frames |
| lateral sweep Sunny 13시 | 370 | 77.30% | 74.05% | 13 frames |
| avoid-return Foggy 11시 | 200 | 95.00% | 83.50% | 5 frames |

집계기가 association의 direct `rejected_*` counter를 빠뜨리던 문제를 바로잡고, raw event
합계와 해당 reason이 발생한 frame 수/rate 및 outcome별 incidence를 함께 남긴다. 대표
1,247 frame에서 partial 또는 yellow 없는 frame은 모두
`boundary_selection.spline_residual_above_limit`와 동시에 발생했다. confidence 탈락은
소수였고 `association.lateral_residual`은 complete frame에도 항상 존재해 outage의 단독
원인이 아니다. 따라서 threshold를 낮추는 실험보다 metric evidence를 공동 곡선·법선
offset으로 설명해 개별 spline residual을 안정화하는 shared lane-family를 다음 A/B 대상으로
선정한다.

재현 산출물은 다음에 있으며 raw bag과 기존 v3 result는 수정하지 않았다.

```text
/data/rosbags/audits/shadow_pipeline_audit_20260914/
/data/rosbags/results/<four-representative-run-id>/twinlite_v6_shadow_pipeline_audit_v1_20260914/
```

대표 4-run A/B와 support·false-bridge 대표 gate에 이어 전체 12+4 replay까지 완료했다.
run별 stratified paired 비교를 포함한 전체 정량 gate는 통과했다. shared 결과는 다음
runtime 직접 검수 단계로 올리되 public boundary 계약은 기존 independent 경로를 유지한다.

### 2026-09-14 shared lane-family offline A/B

`shared_lane_family.py`는 LaneCPP의 연속 곡선·법선 방향 물리 prior와 Differentiable LSQ의
confidence-weighted fitting 원칙을 팀 코드로 독립 구현한다. 외부 연구 repository 코드를
복사하거나 runtime dependency로 사용하지 않는다. 한 개의 공통 parametric cubic B-spline과
좌·우 normal offset을 번갈아 robust fitting하며, 폭 `3.48/3.52m`는 관측으로 조정되는 soft
prior다. 한 선만으로 세 경계를 생성하지 않으며, 두 white 또는 yellow+한 white처럼 최소 두
역할이 있을 때만 빠진 역할을 `structure_inferred`로 표시한다.

평가는 기존 independent와 동일한 metric evidence·track을 입력으로 사용했다. 대표 4개 raw
bag 1,255 frame 중 공지된 음영 polygon 내부 508 frame을 map-supported ODD로 별도 집계했다.

| 지표 | independent | shared family | 변화 |
|---|---:|---:|---:|
| ODD 내 3경계 complete | 78.74% | 100.00% | +21.26%p |
| ODD 내 yellow 역할 가용 | 85.63% | 100.00% | +14.37%p |
| map distance p95 | 0.3761m | 0.3775m | +0.0013m |
| 0~10m map p95 | 0.1844m | 0.1629m | -0.0216m |
| 10~15m map p95 | 0.2954m | 0.2602m | -0.0352m |
| 15~20m map p95 | 0.3997m | 0.3897m | -0.0100m |
| left 경계 reference-y jitter p95 | 0.0303m | 0.0213m | -0.0090m |

대표 정량 gate는 통과했다. 반면 raw evidence에 대한 frame-level residual p95 분포는
`0.2844→0.5380m`로 커졌다. independent 값은 residual gate를 통과한 개별 spline만 남긴
선택 편향이 있고 shared 값은 기존에 버리던 세 track까지 함께 설명하므로 두 값은 직접적인
동일 모집단 비교가 아니다. 그래도 outlier를 정상 관측으로 승격할 위험을 뜻하므로 runtime
승격 조건으로 사용하지 않는다. 현재 상태는 `promising_offline_candidate`,
`runtime_promotion_allowed=false`다.

기존 temporal tracker에서는 대표 replay 동안 left 1회, yellow 6회의 stable-ID 교체가
집계됐다. shared 방식은 역할 이름이 구조적으로 고정되어 이 숫자를 0이라고 직접 비교할 수
없다. 실제 곡선 identity 안정성은 다음 temporal 단계에서 ego-motion 보상 후 별도로
평가한다. offline 전체 배관과 두 방법을 함께 실행한 Python audit 지연은 p50/p95
`77.34/95.03ms`이며 runtime 지연으로 해석하지 않는다.

재현 산출물은 다음에 있다. per-run `frames.jsonl`에는 관측/추정 역할, 폭, residual, map
거리와 거리 bin을 남기며 `_summary_v2`에는 집계 JSON·CSV·contact sheet가 있다.

```text
/data/rosbags/audits/shadow_lane_family_offline_v3_20260914/
└── _summary_v3/{summary.json,runs.csv,contact_sheet.jpg,metadata.json}
```

offline consumer는 `/perception/lane/debug/evidence_points`와 `/Ego_topic`만 읽고 perception,
planning 토픽을 발행하지 않는다. output 디렉터리는 기존 경로를 덮어쓰지 않는다.

```bash
roslaunch drivable_bev shadow_lane_family_offline.launch \
  run_id:=<raw_run_id> \
  output_dir:=/data/rosbags/audits/<audit_id>/<raw_run_id>

python3 $(rospack find drivable_bev)/scripts/summarize_shadow_lane_family.py \
  --input-root /data/rosbags/audits/<audit_id> \
  --output-dir /data/rosbags/audits/<audit_id>/_summary
```

### 2026-09-14 shared support·false-bridge 대표 gate

초기 shared fit이 station 범위만으로 관측을 판정하던 부분을 metric residual 기반으로
보수화했다. 경계별 sample은 다음 세 상태를 가진다.

- `observed`: 해당 역할의 실제 evidence가 fitted sample에서 0.35m 이내이고 0.5m 이상
  연속됨
- `structure_inferred`: 같은 station의 family를 다른 두 역할 이상이 지지하지만 해당 역할의
  직접 evidence가 없음
- `invalid`: local multi-role support가 없거나 0.75m를 넘는 support 공백·단일 역할 꼬리

`0.75m`는 빠진 역할을 구조적으로 추정할 수 있는 총 길이 제한이 아니라, family 자체의
두 역할 이상 support가 잠시 끊긴 **내부 공백**의 최대 연결 길이다. 두 white가 길게 보이고
yellow가 사라진 경우 yellow는 같은 유효 구간에서 길게 `structure_inferred`일 수 있으나
`observed`로 승격되지 않는다. 이 상태는 debug 시각화·진단에 연결했지만 공개 판제
메시지의 소비 정책에는 아직 연결하지 않았다.

같은 대표 4개 raw를 다시 재생한 1,200 valid frame 중 family residual p95가 0.50m 이상인
frame은 75개(6.25%)였고 75개 모두 0.35m 밖 outlier support를 포함했다. 전체 shared
frame의 outlier-rate p50/p95는 `0.000/0.145`, inlier residual p95는 `0.294m`였다. 유효
station 중 하나 이상의 경계가 구조 추정인 비율은 p50/p95 `0.341/0.526`, 세 경계가 모두
직접 관측된 비율의 frame 중앙값은 `0.659`다.

map-supported ODD 507 frame의 shared 3경계 구조 가용률은 100%였고, 유효 sample만 map과
비교한 p95는 independent/shared `0.3734/0.3278m`였다. 이 개선에는 잘못된 원거리 꼬리를
평가에서 제외한 효과가 포함되므로 모델 정확도 향상으로 해석하지 않는다. map과 비교 가능한
`structure_inferred` sample 12,593개 중 0.50m를 넘는 불일치는 없었지만, map은
heading·폭 오차가 남은 pseudo-GT이므로 이 값은 false bridge의 절대 정답이 아니라 검수
신호다.

고잔차 overlay 54장과 최악 cluster 대표 frame을 확인한 결과, 큰 residual은 주로 family
유효 범위 밖에 남은 한 경계의 원거리 outlier였다. 새 출력은 이를 red support로 기록하되
곡선은 `invalid`로 끊었다. 관측 사이의 짧은 orange 추정은 map 곡선과 같은 경계 안에
머물렀고 명백한 횡단 연결은 보이지 않았다. 따라서 대표 support gate는 통과했지만,
`runtime_promotion_allowed=false`는 유지하고 이어지는 12+4 전체 matrix에서 재확인하기로 했다.

```text
/data/rosbags/audits/shadow_lane_family_support_v4_20260914/
└── _summary_v2/
    ├── summary.json
    ├── review_candidates.csv
    ├── contact_sheet.jpg
    └── high_residual_contact_sheet_01..04.jpg
```

### 2026-09-14 shared lane-family 12+4 확대 평가

보수화한 동일 파라미터를 재튜닝하지 않고 기본 조건 12개와 boundary straddle·lateral
sweep·static avoid-return 4개를 합친 16개 raw bag에 적용했다. 총 4,075 frame 중 공지 음영
polygon 내부 1,632 frame을 map-supported ODD로 집계했다.

| 지표 | independent | shared family | 변화 |
|---|---:|---:|---:|
| 전체 가용률 | 94.90% | 100.00% | +5.10%p |
| ODD 내 3경계 complete | 84.38% | 100.00% | +15.62%p |
| ODD 내 yellow 역할 가용 | 91.91% | 100.00% | +8.09%p |
| map distance p95 | 0.4013m | 0.4077m | +0.0064m |
| 0~10m map p95 | 0.2083m | 0.2060m | -0.0022m |
| 10~15m map p95 | 0.3111m | 0.3526m | +0.0415m |
| 15~20m map p95 | 0.4230m | 0.4333m | +0.0103m |

위 표는 각 방법이 실제로 출력한 전체 표본을 따로 모은 기술 통계다. shared가 independent가
포기한 어려운 frame-role 519개를 추가하므로 이 p95 차이를 그대로 정확도 gate로 쓰면 모집단
편향이 생긴다. 같은 run·frame·role만 짝지은 뒤 run별로 층화한 평가에서는 16개 run의 전체
p95가 모두 개선됐다. run별 전체 p95 변화 중앙값은 `-0.0582m`, 전체/거리 bin을 합친 최악
악화도 `+0.0193m`였다. shared-only 519개 role의 map p95도 `0.2593m`였다. 모든 날씨·시간·
시나리오 gate가 통과했고 maneuver 4개에서는 complete가 `+19.90%p`, map p95가 약
`-0.10m` 개선됐다.

3pm 표본을 한데 섞은 10~15m pooled p95는 `+0.0686m`였지만, 3pm 네 run의 같은 구간
paired 변화는 `+0.0143/-0.0143/-0.0531/-0.0411m`로 모두 5cm gate 안이었다. 난이도가 다른
run의 비선형 p95를 다시 합친 Simpson-type 집계 편향이므로 gate는 stratified paired 비교를
정본으로 사용하고 pooled 값은 설명용으로 보존한다. 최종 상태는
`promising_offline_candidate`다. debug output-state 계약은 이식했지만 temporal propagation과
production promotion gate가 아직 없어 `runtime_promotion_allowed=false`는 유지한다.

shared inlier residual p95는 `0.301m`, frame outlier-rate p50/p95는 `0.000/0.163`, 유효 family
길이 p50은 `19.50m`였다. 구조 추정 비율 p50/p95는 `0.341/0.606`이다. residual 0.50m 이상
337 frame은 모두 outlier support를 포함했다. 고잔차 contact sheet에서 조건 전반의 129개
표본을 검수했을 때 invalid 꼬리는 끊겼고 명백한 횡단 false bridge는 보이지 않았다. map과
비교 가능한 inferred sample 43,038개 중 9개(`0.0209%`)가 0.50m를 넘었지만 map은
pseudo-GT이므로 review signal로만 사용한다.

Ego/IMU를 50ms 이내 source timestamp로 대응한 보조 audit에서는 residual 상위 10%에 큰
조향·yaw-rate·상대 pitch/roll·가감속 중 하나가 포함된 비율이 `56.1%`, 나머지는 `31.1%`였다.
map 오차와 상대 pitch 변화의 Spearman 상관은 `0.198`로 약했고 raw residual과 yaw-rate는
음의 상관이어서 동역학을 단일 원인으로 단정하지 않는다. 상위 residual의 outlier-rate p50
`0.160`(나머지 `0.000`)을 함께 보면 비차로 evidence와 자세 변화가 복합된 tail로 해석하는
편이 타당하다. shared family와 support state는 이번 debug runtime sidecar로 이식했으며,
직접 주행 검수 뒤 Ego-motion temporal prediction과 IMU 자세 보정을 각각 A/B한다.

최종 재현 산출물은 다음과 같다. `_summary`~`_summary_v4`는 집계 모집단 검증 과정의 audit
trail이고 `_summary_v5`가 run별 stratified paired gate를 포함한 정본이다.

```text
/data/rosbags/audits/shadow_lane_family_full_v5_20260914/
└── _summary_v5/
    ├── summary.json
    ├── runs.csv
    ├── conditions.csv
    ├── review_candidates.csv
    ├── dynamics_correlation.json
    ├── dynamics_by_run.csv
    ├── manual_review.json
    └── high_residual_contact_sheet_01..22.jpg
```

### 2026-09-14 shared support-state debug runtime

확대 gate를 통과한 동일 shared-family 구현을 기존 independent runtime 옆의 **debug
sidecar**로 이식했다. `three_view_perception_dev.launch`에서만 기본 활성화되고,
`lane_vector_pilot.launch`에서는 `enable_shared_family_debug:=true`를 직접 주어야 한다.
기존 `/perception/lane/boundary_observations`와 planning adapter 입력은 바꾸지 않았으며
shared 출력에는 `deployment_allowed=false`를 명시한다.

경계 sample은 상태 전환에서 분리된 segment로 발행한다. `observed`는 실제 색상(white/yellow)
실선, `structure_inferred`는 주황 점선, `invalid`는 붉은 점선으로 표시된다. invalid tail이나
긴 support 공백을 observed 선과 연결해서 그리지 않는다. rqt에서는 다음 화면을 선택한다.

```text
/perception/lane/debug/shared_family_bev/compressed
```

격리된 lateral-sweep bag 25초 smoke에서 shared marker와 diagnostics는 약 `8.4Hz`, lazy rqt
영상은 설정값대로 `5.0Hz`로 발행됐고 runtime error와 3-view sync reject는 0이었다. 같은
누적 실행에서 shared fit p50/p95는 `31.27/38.33ms`, 전체 lane-vector 처리 p50/p95는
`92.47/116.46ms`였다. 이는 기능·시각화 smoke 수치이며 production latency gate는 아니다.
따라서 shared sidecar는 직접 주행 검수용으로만 사용하고, 다음 단계에서 fitting 연산
최적화를 먼저 수행한다. Ego-motion temporal state는 판제 요청이나 측정된 dropout이 있을
때만 별도 A/B한다.

### Shared-family runtime 기준선 계약

현재 직접 시각화가 안정적이므로 Ego-motion temporal prediction과 IMU 자세 보정은 기본
개발 순서에서 제외한다. 판제 closed-loop가 짧은 전체-family dropout을 실제 문제로 보고하거나,
나머지 음영구간 기능을 완성한 뒤 필요성이 수치로 확인될 때만 별도 A/B한다. 현재 shared
geometry와 threshold는 품질 재튜닝 없이 기준선으로 동결한다.

상태의 소비 정책은 다음과 같다.

- `observed`: 실제 해당 역할 evidence가 있으므로 정상 후보
- `structure_inferred`: 다른 두 역할의 local support로 만든 강등 후보. confidence·유효거리와
  함께 제한적으로 사용
- `invalid`: 디버그 family 곡선이 도로에 맞아 보여도 실제 local support가 없으므로 **사용 금지**
- `temporal_predicted`: 구현하지 않음

runtime 기준선 지표는 family accepted rate와 최장 reject 연속 frame, 역할별 observed/inferred
가용률, 상태별 metric 길이 비율, 유효 span, 좌·우 폭, inlier/data residual, outlier rate,
shared fit 및 전체 lane-vector p50/p95, 출력률, sync reject와 error다. false bridge는 수치만으로
확정하지 않고 straddle·lateral sweep·회피/복귀 화면을 함께 검수한다.

`shadow_boundary_result`와 `camera_semantic_result`는 shared diagnostics와 Marker를 optional로
기록한다. `audit_shadow_lane_results.py`는 invalid 길이를 usable에서 제외해 위 지표를 집계한다.
headless replay에서는 다음 옵션을 명시해야 하며 기본 production 경로는 계속 꺼져 있다.

```bash
python3 $(rospack find drivable_bev)/scripts/run_shadow_lane_bag_batch.py \
  --selection <selection.yaml> \
  --checkpoint <checkpoint.pt> \
  --model-id <unique-model-id> \
  --enable-shared-family-debug \
  --result-profile shadow_runtime_profile_result
```

`shadow_runtime_profile_result`는 영상·Marker를 기록하지 않고 구조화된 camera/BEV/lane
diagnostics, boundary observation, pipeline audit와 shared-family diagnostics만 기록한다. 따라서
시각화 subscriber가 없는 독립 성능 측정에 사용한다. 화면 검수가 목적이면 기존
`shadow_boundary_result` 또는 `camera_semantic_result`를 선택한다.

4개 maneuver raw bag을 별도 ROS master에서 다시 추론·기록한 정식 debug 기준선은 총
`1,126` frame이다. family accepted rate는 `100%`, 최장 reject 연속은 `0 frame`이고 세 역할은
모든 frame에서 직접 관측 역할로 유지됐다. 다만 곡선 sample 기준으로는
`observed/structure_inferred/invalid=66.98/10.20/22.82%`이며, usable 중 구조 추정 비율은
`13.21%`다. 역할 자체가 observed여도 그 곡선 전체가 observed라는 뜻은 아니다.

shared fit p50/p95는 `39.68/51.00ms`, 유효 span p50/p95는 `20.03/21.75m`, 좌·우 폭 p50은
`3.522/3.535m`였다. inlier residual p95의 frame 분포 p95는 `0.293m`이고 runtime error는
0회였다. run별 출력률은 `9.082/8.857/8.866/9.277Hz`로 연속 출력은 유지됐지만 두 run이
9Hz 목표를 근소하게 통과하지 못했다. 35ms 3-view gate의 reject는 입력 후보 tuple을 버린
것이지 shared-family 출력 실패가 아니며, 누적 카운터는 run별 증분으로 해석한다.
ROS graph는 11312로 격리했지만 사용자의 기존 11311 live 3-view pipeline을 종료하지 않아
GPU/CPU는 공유했다. 따라서 상태·geometry 집계는 기준선으로 사용하되 위 처리시간·출력률은
보수적인 개발 동시부하 수치다. production latency 판정 전에는 live stack을 끈 standalone
조건으로 한 번 더 측정한다.

따라서 현재 결론은 **geometry/state debug 기준선 확정, production 승격 보류**다. 다음
최적화는 곡선과 상태 판정을 바꾸지 않는 프로파일링부터 시작하고, 9Hz 이상을 안정적으로
회복한 뒤 판제 closed-loop에서 `observed`와 강등된 `structure_inferred`만 시험한다.
lineage, 소비 정책, 4-bag 수치와 판정은
`artifacts/perception_eval/analysis/twinlite_v6_shared_runtime_baseline_20260914/baseline.yaml`에
고정했다. 상세 frame 집계는 같은 디렉터리의 `runtime_audit.yaml`, 실행·result bag lineage는
`batch_summary.yaml`을 따른다.

### 2026-09-14 standalone 단계별 runtime profile

위 공존부하 기준선과 같은 4개 raw bag을 사용하되 기존 11311 perception/시각화 프로세스를
종료하고, 11312의 inference+BEV+lane pipeline만 실행했다. 영상·Marker subscriber도 두지
않았다. 총 `1,264` frame에서 family accepted rate `100%`, 최장 reject `0 frame`, runtime
error `0`이었고 shared diagnostics 출력률은 run별
`10.298/10.184/9.762/10.077Hz`로 모두 9Hz gate를 통과했다.

shared fit p50/p95는 `28.56/35.37ms`, 전체 profiled body p50/p95는
`64.64/78.53ms`다. 기존 공존부하 기준선보다 shared fit은 p50 `28.0%`, p95 `30.6%`
감소했다. 각 run 종료 시 누적 rolling 전체 처리 p95도 약 `31.7~39.2%` 낮아졌다. 이는
알고리즘 최적화 결과가 아니라 공존 inference/GUI 부하 제거 효과다.

| 단계 | p50 (ms) | p95 (ms) | p50 body 비중 |
|---|---:|---:|---:|
| shared-family fit | 28.56 | 35.37 | 44.2% |
| association | 7.21 | 10.55 | 11.2% |
| independent spline fit | 6.41 | 9.53 | 9.9% |
| temporal tracking | 5.44 | 8.61 | 8.4% |
| shadow prefit | 5.13 | 8.54 | 7.9% |
| projection audit+extract | 4.48 | 5.71 | 6.9% |
| typed boundary publish | 1.69 | 3.00 | 2.6% |

상태 분포는 `observed/structure_inferred/invalid=66.95/10.24/22.80%`, 좌·우 폭 p50은
`3.522/3.536m`, inlier residual p95의 frame 분포 p95는 `0.293m`로 공존부하 기준선과
사실상 동일하다. 처리량 증가로 출력 frame 모집단이 달라졌으므로 bitwise 동일성 대신
분포 비회귀로 해석한다. 연속 실행 중 run 사이 idle time을 포함할 수 있는 rolling
`processing_fps` 대신 result topic의 run별 rate를 처리량 정본으로 사용한다.

다음 최적화는 출력과 피팅 값을 바꾸지 않는 선에서 시작한다. 첫 후보는 중복 순회하는
projection audit/extract 결합이고, 가장 큰 병목인 shared fit은 basis 구성·robust iteration·
residual lookup·linear solve 내부 시간을 먼저 더 나눈다. independent spline 경로 제거는
shared-family typed public 계약과 판제 gate 이후로 미룬다. 정본 수치와 비교 판단은
`artifacts/perception_eval/analysis/twinlite_v6_shared_runtime_standalone_profile_20260914/baseline.yaml`을
따른다.

### 2026-09-14 typed shared boundary-family 후보

shared-family를 Marker/diagnostics에서만 보지 않고 판제 후보 adapter가 손실 없이 읽을 수
있도록 다음 additive typed 토픽을 추가했다.

```text
/perception/lane/boundary_family
    asmc_msgs/LaneBoundaryFamily
```

토픽 이름은 fitting 구현명인 `shared_family_observations` 대신 결과 의미인
`boundary_family`를 쓴다. 기존 `/perception/lane/boundary_observations`와 C++ adapter는
변경하지 않았으며 새 메시지는 `deployment_allowed=false`인 후보 계약이다. 세 family member는
left outer white, road-centre yellow, right outer white의 **기하 역할**만 표현하고
home/opposing/절대 차로 번호를 정하지 않는다.

각 `polyline` sample에는 같은 길이의 `support_state`가 붙는다.

- `OBSERVED`: 해당 역할의 실제 model evidence가 지지하는 정상 후보
- `STRUCTURE_INFERRED`: accepted same-frame family와 normal-offset 구조로 생성한 강등된
  사용 가능 후보. candidate v2는 `x[0,15]m` 안의 순간 local-support 공백도 이 상태로
  유지한다. RViz의 주황색 구간이며 temporal/map prediction은 아니다.
- `INVALID`: bounded completion 밖이거나 family support가 없는 붉은 구간. 메시지에는
  검수용 좌표를 남기지만 downstream은 이 위치에서 곡선을 분할하고 사용하지 않는다.

fit reject나 pipeline error도 토픽을 침묵시키지 않고 `valid=false`, 빈 boundaries와
`status_reason`으로 발행한다. consumer는 missing/stale와 fresh reject를 구분해야 한다.
candidate만 headless로 켜려면 다음처럼 실행한다.

```bash
roslaunch drivable_bev three_view_lane_pipeline.launch \
  enable_boundary_family_candidate:=true \
  enable_shared_family_debug:=false

rostopic hz /perception/lane/boundary_family
rostopic echo -n1 /perception/lane/boundary_family
```

기존 `enable_shared_family_debug:=true`는 하위 호환을 위해 typed candidate도 함께 켠다.
`shadow_runtime_profile_result`는 새 typed 토픽을 required로, 일반 result profile은 optional로
기록한다.

2026-09-15에는 planning_control 쪽의 별도 read-only C++ adapter를 추가했다.

```bash
roslaunch lane_boundary_adapter shadow_boundary_family_candidate.launch
```

adapter는 shared sample의 네 endpoint가 모두 observed인 interval만 정상 strip으로 만들고,
structure-inferred가 포함되면 주황 degraded segment로 분리한다. invalid endpoint가 하나라도
있으면 해당 interval을 출력하지 않는다. straddle raw bag의 격리 smoke에서 398개 candidate를
contract error 없이 소비했고 debug MarkerArray는 약 10.1~10.6Hz였다. 이 결과는
`deployment_allowed=false`이며 Behavior/Frenet/MPC나 제어 출력에는 연결되지 않았다.

정밀 A/B는 시간 제약으로 생략하고 candidate v2 bounded completion을 적용했다. 다음 단계는
live shadow ODD에서 제어 미연결 관찰 gate를 수행하는 것이다. Ego-motion/IMU 보상은 계속
보류한다.

연산 최적화는 인터페이스 연결 뒤 geometry/state 비회귀 gate로 진행한다. 우선순위는
projection audit/extract 중복 순회 제거, shared fit 내부 세분 profile, 그 다음 candidate가
기본 소스로 승격될 때 independent fit 제거 여부 검토 순이다.

### 2026-09-15 판제 실제 가용구간 기준선

기존 `structure_complete=100%`는 세 역할의 family 곡선 객체가 존재한다는 뜻이지, 판제
adapter가 끊김 없이 사용할 수 있다는 뜻은 아니다. 이를 구분하기 위해 producer의 sample
상태를 C++ read-only adapter와 같은 규칙으로 다시 계측했다. 인접한 두 sample의 두 경계
endpoint가 모두 non-invalid일 때만 한 strip interval을 usable로 인정한다. `observed`만 있는
길이와 `structure_inferred`를 포함한 degraded 길이도 분리한다. 이 계측은 fitting이나 support
상태를 고치지 않는다.

대표 maneuver raw bag 4개를 동일 v6 EMA checkpoint로 격리 재생한 결과는 1,274 frame이다.
아래 `≥80%`는 다음 실험을 비교하기 위한 보고 임계값이며 production 합격 기준이 아니다.

| 구간 (`base_link` x) | 두 strip coverage p05 / p50 | ≥80% frame | 완전 연속 frame | 시작 anchor | 최대 gap p95 | 최대 internal gap p95 |
|---|---:|---:|---:|---:|---:|---:|
| 현재 adapter audit window `0~15m` | 93.92% / 100.00% | 96.00% | 69.54% | 81.63% | 0.91m | 0.00m |
| 차량 envelope `-0.79~3.845m` | 74.99% / 99.66% | 74.80% | 48.35% | 62.95% | 1.16m | 0.00m |
| 근거리 전방 `3.845~10m` | 100.00% / 100.00% | 97.41% | 96.15% | 98.98% | 0.00m | 0.00m |
| 중거리 전방 `10~15m` | 100.00% / 100.00% | 96.62% | 95.84% | 97.33% | 0.00m | 0.00m |
| 원거리 debug `15~20m` | 16.70% / 90.43% | 68.84% | 3.22% | 95.84% | 4.17m | 0.00m |

`0~15m`에서 80% 미만인 최장 연속 구간은 약 2.16초였다. static avoid-return 두 run의
coverage p05는 Sunny/Foggy에서 각각 65.58/59.58%로 straddle/lateral-sweep보다 낮았다.
반면 `3.845~15m`는 대부분의 frame에서 완전 연속이었다. 현재 좌·우·두 strip의 **가용**
coverage가 동일한 것은 세 경계에 공통 `sample_valid`를 적용하기 때문이다. 한 역할만
invalid가 되어 한쪽 strip만 사라진 것이 아니라 family support의 선두/말단이 세 경계에서
동시에 끊기는 현재 구현 특성을 수치로 확인했다.

모든 구간의 internal-gap p95가 0m이므로 `max_structure_bridge_m=0.75`를 곧바로 늘리는 것은
현재 병목을 해결하지 못한다. 다음 실험은 원거리 invalid tail을 살리지 않고, 충분한
`3.845~15m` support가 있는 frame에서 차량 근처 leading tail만 보수적으로 구조 완성하는
방향으로 제한한다. 그 전에 같은 계측을 12+4 전체 matrix로 확대해 날씨·시간·clear/static
조건에 비회귀인지 확인한다.

재현 가능한 소형 artifact에는 checkpoint와 selection SHA-256, run별/구간별 수치가 있다.

```text
artifacts/perception_eval/analysis/
└── twinlite_v6_shadow_planning_usability_baseline_v2_20260915/
    ├── summary.json
    ├── runs.csv
    └── metadata.json
```

새 계측 코드는 `planning_usability.py`와
`summarize_shadow_planning_usability.py`에 있으며, raw/result bag이나 공개 ROS 토픽은
변경하지 않았다. 현재 판정은 **대표 기준선 계측 완료, 보간 정책 변경 전**이다.

#### 12+4 전체 확대 결과

같은 checkpoint·support 파라미터를 고정해 기본 clear/static 12개와 maneuver 4개를 합친
16개 raw bag, 4,122 frame으로 확대했다. 집계용 80% coverage는 비교 지표이지 production
합격선이 아니다.

| 구간 | coverage p05 / p50 | 일부라도 가용 | ≥50% frame | ≥80% frame | 완전 연속 | 최장 ≥80% 미달 |
|---|---:|---:|---:|---:|---:|---:|
| 현재 adapter audit window `0~15m` | 63.13 / 100.00% | 100.00% | 99.27% | 93.77% | 69.84% | 2.16s |
| 근거리 전방 `3.845~10m` | 84.91 / 100.00% | 100.00% | 99.44% | 95.51% | 93.77% | 2.20s |
| 중거리 전방 `10~15m` | 72.49 / 100.00% | 96.39% | 95.61% | 94.78% | 93.43% | 2.16s |
| 원거리 audit band `15~20m` | 0.00 / 89.75% | 93.84% | 79.65% | 62.08% | 0.99% | 7.50s |

`0~15m`의 낮은 tail은 static-avoid 계열에 집중됐다. clear는 80% coverage frame이 97.5%,
기존 static-avoid는 86.5%, avoid-return은 92.5%였다. 모든 zone에서 internal gap p95는
계속 0m다. 즉 유효 segment 중간이 반복적으로 뚫리는 문제가 아니라 시작 또는 끝의
support 범위가 줄어드는 문제다.

`15~20m` 관측은 판제가 곡률과 다음 경계를 미리 보는 데 활용할 가능성이 있다. 실제로
shared family 객체와 수학적 spline은 100% frame에 존재한다. 그러나 local camera support가
전혀 없는 frame이 6.16%이고, Foggy에서는 80% coverage가 53.7%뿐이다. 따라서 거리를
기준으로 토픽을 잘라 둘로 나누지 않고 다음 동적 계약을 후보로 둔다.

- producer는 설정된 전체 polyline과 sample별 `observed/structure_inferred/invalid`를 보존한다.
- consumer는 frame마다 연속 trusted interval과 최대 candidate horizon을 계산한다. 중간 gap이
  있을 수 있으므로 단일 horizon 수치만으로 전체 가용성을 표현하지 않는다.
- 현재 C++ adapter의 `x[0,15]m` clip은 안전한 read-only 검증을 위한 임시 설정이지 최종
  perception/planning 계약이 아니다.
- 검증된 same-frame extrapolation을 추가한다면 거리 자체가 아니라 별도 저권한 상태,
  confidence와 provenance로 구분한다. support가 약할 때는 이전 승인 경계나 MGeo가 fallback을
  맡으며 camera `invalid`를 정상 관측으로 위장하지 않는다.

다음 작은 작업은 현재 red/invalid sample 자체의 map 거리와 camera overlay를 leading tail,
far trailing tail, whole-support 축소로 나눠 평가하는 것이다. 이 평가를 통과하기 전에는
`invalid`를 `structure_inferred`로 일괄 승격하거나 20m까지 무조건 연장하지 않는다.

전체 정본은 다음 artifact에 있다.

```text
artifacts/perception_eval/analysis/
└── twinlite_v6_shadow_planning_usability_full16_v1_20260915/
    ├── baseline.yaml
    ├── summary.json
    ├── runs.csv
    └── metadata.json
```

약 54MB의 per-frame 중간 audit은 Git에 넣지 않고
`/data/rosbags/audits/shadow_planning_usability_full16_v1_20260915/`에 보존한다. 같은 raw를
다시 추론하면 scheduler에 따라 frame 모집단이 조금 달라질 수 있으므로, 다음 파라미터 A/B는
한 callback 안에서 baseline과 후보를 동시에 계산해 동일 입력 frame으로 비교한다.

#### Invalid support 원인 분류

다음 작은 단계에서는 support를 보강하지 않고 기존 invalid를 원인별로 계측했다. 새
`planning_usability` schema는 각 zone의 첫/마지막 usable x와 leading/trailing/internal gap을
직접 저장한다. 분류는 다중 label이다. 한 frame에 차량 근처 leading tail과 원거리 trailing
tail이 함께 있으면 둘 다 남기며 `primary_reason`은 보고용 축약일 뿐이다.

- `vehicle_near_leading_tail`: `x=0`에서 시작하는 invalid가 차량 전방 끝 `x=3.845m` 구간과
  겹침
- `preview_trailing_tail`: 설정된 preview `x[15,20]m`의 끝에 닿는 invalid
- `preview_entry_missing`: `x=15m`에서 preview support가 시작되지 않음
- `whole_support_shrink`: preview만 약한 경우가 아니라 `3.845~10m` 또는 `10~15m` coverage가
  보고 임계값 80% 아래로 축소
- `internal_support_gap`: `0~15m` 안에서 usable interval 양쪽에 둘러싸인 invalid

16개 run을 explicit tail metric으로 재계산한 4,023 frame 결과는 다음과 같다. 이전 4,122
frame 기준선과 다른 result replay 모집단이므로 절대 frame 수를 직접 차감해 성능 변화로
해석하지 않는다.

| invalid 원인 | frame 비율 | 길이 요약 |
|---|---:|---:|
| 차량 근처 leading tail | 13.94% | 전체-frame p95 0.367m |
| preview trailing tail | 85.83% | 전체-frame p50 0.503m, p95 5.0m |
| preview 시작 미고정 | 6.39% | preview 일부/전체 선두 미지원 |
| 전체 support 축소 | 7.56% | near 또는 mid coverage <80% |
| 내부 support gap | 0.75% | 전체-frame p95 0m |

원거리 trailing 비율이 높은 것은 20m까지 항상 연속이라는 뜻이 아니라, 0.26m보다 큰 작은
말단 절단도 포함하기 때문이다. `0~15m`는 현재 adapter audit window이고 20m는 현 grid의
평가 상한이며, 둘 다 영구 인터페이스가 아니다. Foggy의 preview trailing/entry-missing
비율은 각각 90.51/10.47%로 Sunny의 82.63/3.60%보다 약하다. 반면 전체 support 축소는
7.56%여서 차량 근처 completion과
근·중거리 붕괴를 같은 정책으로 처리해서는 안 된다.

`invalid_support.py`는 이 분류만 수행하고 support state나 geometry를 바꾸지 않는다. runtime
shared-family diagnostics와 offline frame에는 같은 분류가 기록된다. typed 메시지에는 invalid
좌표가 계속 남아 판제가 명시적으로 선택하는 저권한 preview/fallback 실험에 활용할 수 있지만,
현재 기본 C++ adapter는 여전히 invalid interval을 분할·폐기한다. 이 좌표는 카메라 관측이
아니므로 `observed/structure_inferred`로 재표기하면 안 된다. zone 범위는 설정에서 읽으므로
향후 segmentation 재학습으로 유효거리가 늘면 코드 변경 없이 named audit zone을 다시
설정하고 측정할 수 있다.

```text
artifacts/perception_eval/analysis/
└── twinlite_v6_shadow_invalid_support_full16_v1_20260915/
    ├── baseline.yaml
    ├── summary.json
    ├── runs.csv
    ├── invalid_support_runs.csv
    └── metadata.json
```

per-frame 정본은
`/data/rosbags/audits/shadow_invalid_support_full16_v1_20260915/`에 보존한다. evidence가 없는
과거 maneuver result bag으로 만든 빈 시도는 삭제하지 않고
`/data/rosbags/audits/rejected_shadow_invalid_support_no_evidence_20260915/`로 이동했다.

#### Invalid support 분류 오버레이 사전검수

분류 직후 support 정책을 바꾸지 않고 clear/static-avoid, Sunny/Foggy 대표 4개 run에서
분류별 최대 2 frame을 20-frame 간격으로 추출했다. raw bag의 Ego와 기존 evidence result를
하나의 replay clock으로 재생했으므로 새 GPU 추론 변동은 없다. 빨간 실선은 shared family가
계산한 좌표 중 local camera support가 `INVALID`인 구간이며 **저신뢰 관측 차선이 아니다**.

44개 overlay에서 leading/trailing/whole-shrink/internal-gap/preview-entry 분류가 각각 의도한
저-x 말단, 고-x 말단, 근·중거리 축소, 내부 공백, 원거리 audit band 진입 공백에 대응했다.
multi-label frame도 의도대로 여러 contact sheet에 남는다. `no_material_invalid_support`는
분류 임계값을 넘는 gap이 없다는 뜻이며 0.26m 이하의 빨간 말단까지 없다는 뜻은 아니다.

이 단계는 taxonomy/geometry 위치 사전검수만 통과했다. sparse 선택 44장 중 map 선을 함께
그릴 수 있었던 14장은 legacy nearest-Ego gate 실패 때문이 아니라 선택 frame이 pseudo-GT
음영 ODD 안에 있었던 경우다. 네 run의 1,031 frame은 모두 50ms 이내 nearest Ego pose를
가졌다. 따라서 이 44장만으로 map 정확도를 판정하지 않고, ODD 전체 frame을 대상으로 별도
시간 정렬·거리 평가를 수행한다. 그 전에는 빨간 좌표를 새 저권한 상태로 승격하지 않는다.

```text
artifacts/perception_eval/analysis/
└── twinlite_v6_shadow_invalid_support_overlay_prereview_v1_20260915/
    ├── baseline.yaml
    ├── review_manifest.json
    ├── contact_sheet_overview.jpg
    └── contact_sheet_<category>.jpg
```

같은 자료를 다시 만들 때 offline launch의 기본값은 기존 동작 보존을 위해 분류 overlay가
꺼져 있다. 다음 두 인자를 명시하고 raw Ego와 evidence result를 한 `rosbag play --clock`에
넣는다.

```bash
roslaunch drivable_bev shadow_lane_family_offline.launch \
  run_id:=<run_id> output_dir:=<new_output_dir> \
  overlay_stride:=0 high_residual_overlay_limit:=0 \
  invalid_support_overlay_limit_per_category:=2 \
  invalid_support_overlay_min_frame_gap:=20

rosrun drivable_bev summarize_invalid_support_review.py \
  --input-root <per_run_audit_root> \
  --output-dir <new_summary_dir>
```

#### Source-time 위치 정렬과 INVALID–map 거리 평가

대표 clear/static-avoid Sunny/Foggy 4개 run의 1,031 frame을 raw Ego와 기존 v3 evidence result의
단일 replay clock으로 다시 평가했다. 이는 **오프라인 map 평가용 pose 정렬**이며 runtime
ego-motion prediction이나 IMU 자세 보정이 아니다. camera evidence source stamp를 앞뒤
`/Ego_topic`으로 감싸서 XY는 선형, heading은 최단각으로 보간했다. 1,030 frame은 보간됐고
1 frame만 100ms bracket 제한 때문에 nearest로 fallback했다. nearest sample age p50/p95는
`7.05/11.62ms`, bracket span은 `17.28/34.47ms`였다.

보간 pose와 nearest pose의 차이는 p95 이동 `0.121m`, heading `0.056°`였지만, 동일 INVALID
category-frame의 role-matched map 거리 p95 변화는 중앙값 `-0.0004m`, 절댓값 p95
`0.0125m`, 최대 `0.0226m`였다. 보간은 평가 시각의 모호성을 제거하지만 기존 원거리 잔차의
주원인은 아니다.

음영 polygon 안의 378 frame에서 left-white/yellow/right-white를 같은 역할의 pseudo-GT와만
대응하고, map polyline 법선방향 signed distance를 측정했다. 분류는 multi-label이라 총
408 category-frame 관측이며 독립 frame 수로 해석하지 않는다.

| INVALID 구간 | 역할별 frame-p95 중앙값 범위 | 역할별 frame-p95의 p95 범위 |
|---|---:|---:|
| 차량 근처 leading tail | 0.075~0.107m | 0.152~0.187m |
| whole-support shrink | 0.039~0.078m | 0.147~0.177m |
| internal support gap | 0.024~0.035m | 0.044~0.081m |
| preview entry missing | 0.071~0.123m | 0.194~0.200m |
| far trailing tail | 0.062~0.081m | 0.359~0.431m |

오차 구간 `≤0.35m`, `0.35~0.60m`, `>0.60m`는 각각 overlay의 supportive/review/priority-review
우선순위일 뿐 합격 기준이 아니다. 408개 category-frame 중 `385/23/0`개였고, map pseudo-GT,
평면 투영과 비동기 sensor의 잔여 오차를 포함한다. contact sheet에서 가까운 leading·whole
shrink·internal gap은 map과 대체로 겹쳤고, review band는 원거리 trailing tail에 집중됐다.
이 결과는 빨간 수학 곡선이 무작위 이탈은 아니라는 근거지만 camera support를 복구하지는
않으므로 `INVALID→STRUCTURE_INFERRED` 일괄 승격이나 hard 거리 gate에는 사용하지 않는다.

```text
/data/rosbags/audits/shadow_invalid_map_distance_representative_v1_20260915/
artifacts/perception_eval/analysis/
└── twinlite_v6_shadow_invalid_map_distance_representative_v1_20260915/
    ├── baseline.yaml
    ├── summary.json
    ├── metrics_by_run.csv
    ├── metrics_by_category_role.csv
    └── contact_sheet_<category>_<review-band>.jpg
```

재현 시 기존 동작 보존을 위해 기본 `nearest`를 유지하고, 이 audit에서만 다음 인자를 명시한다.

```bash
roslaunch drivable_bev shadow_lane_family_offline.launch \
  run_id:=<run_id> output_dir:=<new_output_dir> \
  ego_pose_alignment_mode:=interpolate ego_pose_max_bracket_ms:=100 \
  overlay_stride:=0 high_residual_overlay_limit:=0 \
  invalid_map_overlay_limit_per_category_band:=2

rosrun drivable_bev summarize_invalid_map_distance.py \
  --input-root <per_run_audit_root> \
  --output-dir <new_summary_dir>
```

#### 시간 제한 판제 후보 승격: candidate v2

2026-09-15에는 추가 A/B보다 음영구간 handoff를 우선하기로 결정했다. 앞선 대표 map audit에서
차량 근처 leading tail, whole-support shrink, internal gap과 preview-entry의 역할별 frame-p95가
최대 `0.187/0.177/0.081/0.200m`였던 점을 근거로, **accepted shared family에 한해서만**
`x[0,15]m` 안의 기존 invalid sample을 `STRUCTURE_INFERRED`로 승격한다.

- 2개 이상 역할과 최소 3m local multi-role support가 필요한 family acceptance gate는 유지한다.
- 승격 sample은 주황색이며 `OBSERVED`가 아니다.
- temporal prediction, Ego-motion 전파와 MGeo는 사용하지 않는다.
- fitted family 자체의 표본 범위를 넘겨 외삽하지 않는다.
- 15m 이후의 원거리 tail과 family reject frame은 계속 `INVALID`다.
- 기존 independent 토픽과 판제 제어 경로는 바꾸지 않는다.

설정은 `shared_lane_family.same_frame_completion_*`에 있고 candidate algorithm ID는
`twinlite_v6_shadow_boundary_family_candidate_v2`다. 이 결정은 시간 제약에 따른
fail-soft 후보이며 정밀 A/B 승인으로 간주하지 않는다. 사용자는 live 주행에서 주황 구간의
false bridge와 세 경계 연속성을 다시 확인해야 한다. read-only adapter는 inferred segment를
계속 강등해 표시하고 남은 invalid를 분할한다.

#### 엇갈린 다중 역할 support 복구: candidate v3

candidate v3는 v2의 조건을 넓게 완화하지 않는다. 기존 strict family가
`insufficient_local_multi_role_support`로만 거부됐고, 같은 frame 안에 독립적으로 배정된
역할이 둘 이상 있지만 그 종방향 support 구간만 엇갈린 경우에만 별도 gate를 평가한다.

- 기존 role assignment의 3m track-span 자격은 유지한다. 그 뒤 `x[0,15]m` 안에서 각 역할은
  최소 2m의 close support를 가져야 하고, 합집합 support는 최소 6m다.
- 내부 무관측 gap은 3m 이하, 역할 간 heading 차이는 15도 이하이어야 한다.
- inlier residual p95 `0.30m`, outlier rate `0.35`, 곡률 p95 `0.12 1/m` 상한을 모두 지킨다.
- 합격해도 해당 역할의 직접 support만 `OBSERVED`이고 나머지 family member는
  `STRUCTURE_INFERRED`다. temporal prediction과 MGeo는 사용하지 않는다.
- 한 역할만 남은 frame, 역할 배정 실패, 비단조 geometry와 다른 reject 사유는 복구하지 않는다.
- strict-accepted frame은 fallback branch에 들어가지 않으므로 기존 geometry와 state가 정확히
  유지된다.

합성 staggered-support 사례는 strict valid length 0m에서 v3 union support 14.75m로 복구됐고,
12.95도 heading 불일치 negative control은 설정한 5도 gate에서 거부됐다. 기존 12+4 감사의
4,023 frame은 모두 strict accepted였으므로 실데이터 recovery rate를 과장하지 않는다. 해당
자료는 기존 정상 frame의 비회귀 근거이고, 실제 activation/false-family 비율은 새 high-heading
주행에서 별도로 확인해야 한다. 정본 수치는
`twinlite_v6_shadow_distributed_support_candidate_v3_20260915/baseline.yaml`에 있다.

현재 candidate algorithm ID는
`twinlite_v6_shadow_boundary_family_candidate_v3`이며 diagnostics의
`acceptance_mode`가 `strict_local_multi_role` 또는 `distributed_multi_role`을 구분한다.

### 2026-09-15 candidate v3 동결과 판제 handoff

사용자의 live straddle·회피 주행에서 세 경계 연속성과 distributed-support 보완을 확인한 뒤,
v3 geometry·threshold·sample-state를 더 조정하지 않는 판제 인수 기준으로 동결했다. 같은
대표 4개 raw bag을 시각화 subscriber 없이 격리 재생한 결과는 총 1,270 frame이다.

| 항목 | 결과 |
|---|---:|
| family accepted / reject 최장 | 100% / 0 frame |
| runtime error | 0 |
| run별 family 출력 | `10.324/10.323/9.381/10.406Hz` |
| 전체 lane body p50/p95 | `76.37/96.96ms` |
| shared fit p50/p95 | `31.66/40.88ms` |
| observed/inferred/invalid sample | `67.22/11.50/21.29%` |
| valid span p50/p95 | `20.07/21.75m` |

네 bag에서는 `distributed_multi_role`가 0회였으므로 정상 strict path의 비회귀·처리량
근거로만 쓴다. 이전 standalone보다 이번 p95가 높았지만 fallback 자체는 실행되지 않았고
association, independent fit, tracking도 함께 느려져 v3 고유 회귀로 보지 않는다. 최저
9.381Hz로 9Hz gate는 통과했으므로 handoff 전에 알고리즘을 다시 최적화하지 않는다.
full-stack에서 실제로 9Hz를 깨면 legacy independent 경로의 opt-in 비활성화, projection
중복 순회 결합, shared-fit 내부 세분 profile 순으로 진행한다.

성능 result bag은 `shadow_runtime_profile_result`로 기록했고, 판제 인수 replay에는
`boundary_family + drivable_probability + coverage`를 필수로 하는
`shadow_camera_perception_handoff_result`를 사용한다. 인지는 토픽까지만 제공하며 음영구간
on/off, route/home/opposing 의미, LiDAR occupancy 결합과 closed-loop 검증 코드는 추가하지
않았다. 판제가 읽어야 할 field, fail-safe와 장애물 회피 결합 순서는
[판제 인수 가이드](../../../docs/handoffs/perception/shadow-camera.md), 정확한 config·checkpoint·
bag SHA-256은
[`freeze.yaml`](../../../artifacts/perception_eval/analysis/twinlite_v6_shadow_boundary_family_candidate_v3_profile_20260915/freeze.yaml)에
고정했다.
