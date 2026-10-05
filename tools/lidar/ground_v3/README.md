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

## 새 실주행 기록 — 2026-10-05

[실주행 보고서](../../../docs/packages/asmc_lidar/l2a2-v3-live-drive-review-20261005.md)와
[manifest](../../../config/baselines/lidar_l2a2_v3_drive_20261005.yaml)를 함께 본다.
실제 터널 내부를 수집했으며 경로 밖 첫 시도는 점수에서 제외했다.
램프에서는 두 후보의 지면 잔여 한계가 드러나 전 구간 승격을 보류했다.

### 사용자가 직접 확인하기

Noetic 컨테이너 `/root/ws`에서 source 후 실행한다. 기존 bridge는 그대로 두고,
실시간 비교 노드와 기록 재생 노드는 **동시에 실행하지 않는다**.

```bash
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash
# 지금 시뮬레이터를 직접 움직이며 보기: bridge가 이미 켜져 있으면 추가 실행하지 않음
roslaunch asmc_lidar ground_compare_live.launch \
  comparison_config:=/root/ws/src/asmc_lidar/config/ground_v3/roll_pitch_all.yaml

# 위 live 노드를 종료한 뒤, 새로 수집한 실제 터널 내부부터 보기
roslaunch asmc_lidar ground_compare_recording.launch \
  bag:=/data/lidar_assessment/20261005_ground_v3_drive/tunnel_path_all/drive_0.bag \
  start:=110

# 위 재생을 종료한 뒤, 횡경사 램프 기록 보기
roslaunch asmc_lidar ground_compare_recording.launch \
  bag:=/data/lidar_assessment/20261005_ground_v3_drive/banked_pi_all/drive_0.bag \
  start:=20
```

기록 launch는 기존 RViz와 저장된 `/lidar_candidate/ground_live/` 출력만 재생한다.
원본 센서·차량 상태·제어·TF·/clock은 재생하지 않아 기존 bridge와 섞이지 않는다.
기본 반복 재생, `rate:=0.5`로 느리게 보기, `rviz:=false`로 RViz 생략 가능하다.
`start`는 bag 시작부터의 초다. 알고리즘을 다시 계산하는 것이 아니라 수집 시 출력이다.
종료는 Ctrl+C다. RViz 세 방법 그룹을 하나씩 토글한다. 모든 점은 동일 base_link다.
실시간은 `method:=jaeho|patchwork|linefit|all`로 계산할 방법도 제한할 수 있다.

### 수집 자료 재평가

원본을 덮어쓰지 않고 새 디렉터리를 지정한다. capture/source manifest에 기록된
SHA와 실제 입력을 확인한다. `tunnel_all`, `banked_path_all`은 실패 기록이라 추출기가 거부한다.

```bash
BASE=/data/lidar_assessment/20261005_ground_v3_drive
NEW=/data/lidar_assessment/<새-평가-디렉터리>
python3 -B tools/lidar/ground_v3/extract_drive_gt.py "$BASE/tunnel_path_all" "$NEW/input"
python3 -B tools/lidar/ground_tuning/run.py "$NEW/input" "$NEW/v1" --profiles v1 lf_covered
python3 -B tools/lidar/ground_v3/run.py "$NEW/input" "$NEW/v3"
python3 -B tools/lidar/ground_v3/score.py "$NEW/input" "$NEW/v1" "$NEW/v3" "$NEW/score"
python3 -B tools/lidar/ground_v3/drive_results.py \
  "$BASE/tunnel_path_all" "$NEW/input" "$NEW/v3/roll_pitch_all.jsonl" \
  "$NEW/score" "$NEW/details.json"
python3 -B tools/lidar/ground_v3/test_drive_input.py
```

`summarize_drive.py CAPTURE`는 최초 실행 시 영상·자세·구간별 상태를 저장한다.
이미 생성된 결과는 덮어쓰지 않는다. `collect_live_drive.py`는 **MORAI 전용**
경로 추종 수집 도구이며 `--confirm-simulator-move`가 필수다. 사용자 정지/키보드
상태에서만 시작하고 시험 전 위치를 복원한다. 실제 차량·운영 판제용 제어기가 아니다.
재수집이 필요하면 route/duration/speed와 컨트롤러를 검토한 뒤 사용한다.
오르막의 simulator speed mode 실패와 최종 PI 시험은 별도 조건으로 보존했다.
