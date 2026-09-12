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

개발 격자의 전체 범위는 정확도 검증 결과가 아니며 판제 ODD도 아니다. 현재 음영구간
후속 검증에서는 `0~10 m`를 제어 핵심, `10~15 m`를 preview, 그 밖을 diagnostic 영역으로
분리한다. production launch와 설정은 개발 격자 추가로 변경되지 않았다.

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

배열은 `base_link`에서 관측된 경계를 `reference_x_m`의 왼쪽(+y)부터 오른쪽(-y) 순서로
전달한다. 각 원소에는 short-term `tracking_id`, white/yellow 색상, 관측 카메라 bitmask,
metric polyline, 유효 x 범위, semantic/geometric score와 bridge 비율이 들어간다.
`order_valid=false`이면 모든 경계가 같은 reference x를 통과하지 않은 것이므로
`lateral_order`와 `reference_y_m`을 차로 의미로 사용하면 안 된다. score는 calibrated
probability가 아니다.

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

```bash
# GPU perception 컨테이너
source /root/ws/devel/setup.bash
roslaunch drivable_bev three_view_perception_dev.launch \
  checkpoint:=/root/ws/artifacts/perception_eval/runs/twinlite_medium_morai_v6_road_marking_init_b_b12_e18_20260908/best_mean.pt
```

ROS GUI 컨테이너에서는 rqt를 토픽 인자 없이 열어 dropdown으로 화면을 바꿔 본다.

```bash
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash
install -d -m 700 /tmp/asmc-xdg-runtime
export XDG_RUNTIME_DIR=/tmp/asmc-xdg-runtime
rqt_image_view
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
rqt_image_view /perception/bev/debug/front/camera_grid_overlay/compressed
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
/perception/lane/debug/camera_reprojection_mosaic/compressed
/perception/lane/diagnostics
```

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

1. **완료:** GPS 음영 왕복 2차로 yellow/outer boundary 선별, stable local ID와 12-run gate
2. 색상·왼쪽→오른쪽 순서·metric polyline·valid/confidence/stamp만 담는 provisional 판제
   메시지를 합의하고 MarkerArray와 병행 발행
3. 인접 경계 사이 local lane strip과 차량 footprint 중첩 상태를 계산하되 home/opposing
   의미와 회피 결정은 붙이지 않음
4. 음영구간 회피·반대편 진입·원차로 복귀 replay를 판제와 공동 검증
5. right outer 누락과 Foggy 11시 열화를 구간 overlay로 분석해 보수적으로 개선
6. 일반 구간은 MGeo candidate matching으로 주차칸·안전지대 표식을 제거한 뒤 확장
7. 통과한 vector 설정으로 최종 BEV 표시 범위를 재선정하고 map/Frenet adapter에 연결

센서 해상도·10/15/20 Hz 최적화는 camera-to-vector 기준선이 완성된 뒤 정확도·지연 저하
없이 부하를 줄이는 마지막 단계로 미룬다.

현재 package는 `camera_semantic_perception`, `asmc_msgs`, `src/link_node`, LBC 및 외부
`morai-roach`를 수정하거나 런타임 의존성으로 사용하지 않는다.
