# L2-A2-V3 — 자세와 터널 지면 비교 재실행

운영 LiDAR가 아닌 `asmc_lidar` 검토 노드/오프라인 실행기를 사용한다. 정본 설정은
`src/asmc_lidar/config/ground_v3/roll_pitch_all.yaml`이다. 설정을 생략하면 기존
pitch-only V1을 정확히 유지한다. 세 방법 모두 같은 ROI→10cm voxel 계보와
`Ry(pitch) Rx(roll)` 수평화를 받고, 출력은 원본 scan 시각의 rear-axle
`base_link`다. `nonground`는 장애물 확정 결과가 아니다.

Noetic 컨테이너 `/root/ws`에서, `$ASMC_DATA=/data`일 때 예시:

```bash
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash
python3 -B tools/lidar/ground_v3/check_manifest.py --repo . --data /data
IN=/data/lidar_assessment/20261004_gt_dual_ground/sync
OUT=/data/lidar_assessment/<새-실험-디렉터리>
python3 -B tools/lidar/ground_v3/run.py "$IN" "$OUT"
```

`run.py`는 source/config/calibration/binary/result SHA를 output manifest에 기록하고
기존 출력은 덮어쓰지 않는다. raw 자료에도 동일하게 적용한다. 네이티브 V1 및
linefit sector coverage 기준 결과는 `tools/lidar/ground_tuning/run.py`로 별도
생성한 뒤, 다음처럼 **같은 원본 라벨과 자세**로 채점한다.

```bash
python3 -B tools/lidar/ground_v3/score.py \
  "$IN" /data/lidar_assessment/20261004_ground_v2/sync \
  "$OUT" /data/lidar_assessment/<새-점수-디렉터리>
```

raw/sync는 서로 합치지 않으며 10점 미만 instance는 별도 집계한다. 새 설정을
GT에 맞춰 장면별로 조정하지 않는다. GT 없는 bag에는 이 점수 도구를 적용하지
않는다. BEV/side 대표 그림은 `plot.py INPUT RESULT INPUT_ID OUTPUT.png`다.

기존 음영구간 bag의 보조 진단은 `extract_tunnel_bag.py`로 원본을 읽기 전용
추출한 뒤 `run.py`와 `tunnel.py`를 사용한다. 과거 sensor profile/장착 위치가
Autonav v3 GT와 다를 수 있고 지면·천장 라벨이 없으므로 평면 선택·시간 변동만
보고한다. `source.json`의 SHA와 센서 조건을 반드시 함께 인용한다.

실시간은 기존 bridge가 있다면 `start_bridge:=false`, 없으면 `true`로 실행한다.

```bash
roslaunch asmc_lidar ground_compare_live.launch \
  method:=all start_bridge:=true rviz:=true \
  comparison_config:=/root/ws/src/asmc_lidar/config/ground_v3/roll_pitch_all.yaml
```

RViz `Ground plane fits` 화살표는 재호의 각 RANSAC 시도 중심·법선을 보여준다.
녹색은 지면으로 채택, 적색은 법선 때문에 거부된 시도다. 위치는 `base_link`지만
평면 자체의 연속 주행 정확도나 천장 GT가 아니다. 상태의 `config_id`,
`leveling_mode`, `queue_dropped`, `source_stamp_ns`, `imu_*`를 함께 확인한다.

15Hz 처리률을 별도 ROS master에서 확인하려면 bridge 없이 review 노드를
`method:=linefit` 또는 `method:=all`로 띄우고 다음을 실행한다. bag은 읽기
전용이며 40초 bag 시간을 약 1.15배 속도로 재생한다. `replay_probe.py`가
31초 진단 구간을 동기화하고, 원래 bag의 실제 입력률을 결과에서 검증한다.

```bash
python3 -B tools/lidar/ground_v3/replay_probe.py \
  /data/rosbags/raw/20261003_185442_kcity2025_ground_normal_drive_sunny_1pm_morai_auto/bags/20261003_185442_kcity2025_ground_normal_drive_sunny_1pm_morai_auto_0.bag \
  /data/lidar_assessment/<새-실험-디렉터리>/replay15_linefit.json \
  --method linefit
```

전체 결과와 미검증 항목은 [V3 보고서](../../../docs/packages/asmc_lidar/l2a2-v3-attitude-tunnel-selection-20261005.md)에 있다.
