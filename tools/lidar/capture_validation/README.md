# L1-B4-B Capture GT 격리 비교

Autonav Capture의 세 LiDAR 파일은 XYZ·순서가 정확히 같은 경우만 수용한다.
filter에는 **Intensity XYZI와 같은 Capture key의 IMU 자세만** 전달하며,
Instance/Semantic/box는 결과 채점 파일로 분리한다. 운영 ROS node·topic·MD5는 변경하지 않는다.

## 입력과 평가 경계

- source는 읽기 전용. 원본 파일을 이동·정규화·덮어쓰지 않는다. container 재생용으로
  bit-identical XYZI 파생 사본과 int32 GT sidecar만 별도 실험 폴더에 생성한다.
- 현재 v3의 identity LiDAR/IMU rotation과 reference mount 일치만 지원한다.
  sensor-origin FLU를 유지하고 y 반전이나 추가 base_link translation을 적용하지 않는다.
- camera BBox2D는 축 방향 교차 점검용이며 YOLO 결과가 아니다. 파일 부재도 정상 빈 검출의
  근거가 아니다. missing/empty는 **통제된 fixture**이며 실제 camera latency를 평가하지 않는다.
- 각 Capture 장면마다 filter를 reset한 후 동일 관측을 5회 반복하고 마지막 결과를 채점한다.
  같은 stamp·cloud·자세의 정차 warmup으로 wall persistence를 비교한다. 실제 5-frame 연속 주행이 아니다.
- stamp는 원래 request_wall_time_ns 정수를 **trigger surrogate**로 기록한다. 잘못된 native
  IMU nSecs를 ROS 시각으로 고치지 않는다. 센서 측정 시각·동기성·속도·track ID 정확도는 미수락이다.
- Capture GPS 좌표가 nonzero인 장면도 격리 filter를 강제로 호출한다. GPS status·운영 음영 routing 수락이 아니다.

## 점 계보와 판정

raw→voxel→cluster 대응으로 각 Instance의 ROI 제외, 지면·천장 소실, 벽 단계 소실,
최종 군집 완전성/순도와 OBB 제거 원인을 나눈다. mixed voxel에는 모든 raw 점을 포함하므로
majority label로 객체·배경 합침을 숨기지 않는다. wall-stage 분모는 upstream에서 살아남은
해당 객체 점이며, 전체 원본 점 보존율과 별개다.

`accepted_target`는 최종 cluster가 wall-stage GT 점의 80% 이상을 포함하고 raw-point 순도가
90% 이상이라는 **관측점 기준 진단**이다. full-box IoU/차량 검출 정확도/대회 수락이 아니다.
10 voxel 미만으로 보이는 객체와 미검증 GT box는 별도 표시하고 기준을 낮춰 통과시키지 않는다.
`StandingOBJ(109)`와 Instance=0을 벽 정답으로 사용하지 않는다. 해당 잔여점·GT instance로
뒷받침되지 않는 객체 수는 proxy이며 실제 오검출률로 이름 붙이지 않는다.

## 재실행

host 저장소 루트에서 실행한다. `${ASMC_DATA:-../data}`는 현재 워크스페이스의 data를 지정한다.
새 output을 사용하고 기존 실험을 덮어쓰지 않는다.

```bash
python3 -B -m unittest discover -s tools/lidar -p 'test_*.py'

python3 -B tools/lidar/capture_validation/admit.py \
  --capture-run "${ASMC_DATA:-../data}/capture_runs/kcity_capture_20260929T052220_840" \
  --lidar-root /mnt/c/MoraiLauncher_Win/MoraiLauncher_Win_Data/SaveFile/SensorData/LIDAR_5 \
  --state-root /mnt/c/MoraiLauncher_Win/SensorData \
  --sensor-profile config/morai/sensors/ASMC_sensor_set_v3.json \
  --setting "${ASMC_DATA:-../data}/lidar_assessment/20260926/jaeho_source/src/lidar+sensorfusion/src/Main/setting.yaml" \
  --output "${ASMC_DATA:-../data}/lidar_assessment/20260929/<new-capture-input>"
```

filter 빌드는 [기존 고정 SHA 평가기](../tunnel_geometry/README.md)의 절차를 사용한다.
container에는 Windows source mount가 없어도 된다. 새로 수용한 XYZI 파생 사본은 /data로 보인다.
이번 동결 입력은 `/data/lidar_assessment/20260929/l1b4b_capture_gt/inputs_v4`다.

```bash
# asmc-ros-noetic 내부, ROS 및 pinned Msg 환경 source 후
python3 -B /root/ws/tools/lidar/tunnel_geometry/run.py \
  --snapshots /data/lidar_assessment/20260929/l1b4b_capture_gt/inputs_v4/snapshots.jsonl \
  --setting /data/lidar_assessment/20260926/jaeho_source/src/lidar+sensorfusion/src/Main/setting.yaml \
  --params /data/lidar_assessment/20260926/offline_v2/resolved_params.yaml \
  --config /root/ws/config/baselines/lidar_l1b4_20260929.yaml \
  --executable /tmp/asmc-l1b4-build/tunnel_policy_eval \
  --output /data/lidar_assessment/20260929/<new-capture-filter-run>

python3 -B /root/ws/tools/lidar/capture_validation/score.py \
  --admission /data/lidar_assessment/20260929/l1b4b_capture_gt/inputs_v4 \
  --run /data/lidar_assessment/20260929/<new-capture-filter-run> \
  --output /data/lidar_assessment/20260929/<new-capture-score-run>
```

`run.py`에는 `--truth`를 주지 않는다. 그 인자는 synthetic voxel GT 용도이고 Capture raw GT와
voxel 계보가 다르다. 위 독립 scorer가 raw sidecar를 이용해 채점한다.

```bash
# host에서 source SHA와 점/군집 결과의 반복 결정성 검사
python3 -B tools/lidar/capture_validation/verify.py \
  --admission ../data/lidar_assessment/20260929/l1b4b_capture_gt/inputs_v4 \
  --run ../data/lidar_assessment/20260929/l1b4b_capture_gt/run_v1 \
  --scores ../data/lidar_assessment/20260929/l1b4b_capture_gt/scores_v1 \
  --repeat-run ../data/lidar_assessment/20260929/l1b4b_capture_gt/repeat_v1 \
  --repeat-scores ../data/lidar_assessment/20260929/l1b4b_capture_gt/repeat_scores_v1
```

`plot.py --admission ... --run ... --output ... --case <input_id>`는 GT 점·소실 단계와
유지/거부 OBB를 BEV·측면 PNG/SVG로 생성한다. 새 RViz publisher를 추가하지 않는다.
결과는 [Capture 비교 보고서](../../../docs/packages/asmc_perception/lidar-l1b4b-capture-validation-20260929.md)에서 연결한다.
