# 지면분할 3종 비교 시각화 (개발용)

## 실시간 비교가 기본 사용 경로

이 문서 아래의 `ground_compare.launch`는 이전의 단일 프레임 분석 도구다.
실제 시뮬레이터를 움직이면서 보는 도구는 **`ground_compare_live.launch`**다.
동일한 입력 scan과 이미 도착한 IMU를 세 방법이 공유하며 GT 없이 실행된다.

```bash
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash
roslaunch asmc_lidar ground_compare_live.launch
# bridge를 아직 켜지 않았을 때만:
roslaunch asmc_lidar ground_compare_live.launch start_bridge:=true
# 계산 자체를 하나만 켜기:
roslaunch asmc_lidar ground_compare_live.launch method:=linefit
# 25cm 보호 전의 고정 높이 Patchwork++:
roslaunch asmc_lidar ground_compare_live.launch method:=patchwork patchwork_variant:=fixed_height
```

기본은 `method:=all rviz:=true start_bridge:=false`다. 함께 켠 bridge는
`local / competition / receive` 설정과 기존 Hz 로그를 사용한다.
기존 bridge가 `source` 시각으로 실행 중이면 그 설정을 바꾸지 않고 진단에
실제 시각 출처를 표시한다. 단순히 launch 기본값을 측정 시각의 증거로 쓰지 않는다.

### 환경 조정 설정 선택

`comparison_config`의 기본은 빈 값이며 기존 V1을 유지한다. 아래는 채택된
새 기본값이 아니라 **검토용 설정을 명시적으로 선택하는 예시**다.

```bash
# linefit 처리 누락 수정만 적용. 다른 두 방법의 설정은 기존 그대로.
roslaunch asmc_lidar ground_compare_live.launch \
  comparison_config:=/root/ws/src/asmc_lidar/config/ground_v2/lf_covered.yaml
# Patchwork++ 평면 거리만 12.5cm에서 8cm로 변경한 단일 변수 비교
roslaunch asmc_lidar ground_compare_live.launch \
  comparison_config:=/root/ws/src/asmc_lidar/config/ground_v2/pp_dist_0p08.yaml
# V3: 세 방법에 동일 roll·pitch, linefit 전체 sector 적용 (연구용)
roslaunch asmc_lidar ground_compare_live.launch \
  comparison_config:=/root/ws/src/asmc_lidar/config/ground_v3/roll_pitch_all.yaml
```

기존 RViz 토글·토픽·좌표계는 그대로다. `method:=linefit` 또는
`method:=patchwork`로 계산할 방법을 제한할 수 있다. 잘못된 파일·설정은
조용히 기본값으로 대체하지 않고 오류로 중단한다. 원인 비트와 전체 후보 목록은
[환경 조정 실험 안내](../../../tools/lidar/ground_tuning/README.md)를 본다.

### 출력 해석

점군은 `/lidar_candidate/ground_live/{jaeho,patchwork,linefit}/` 아래의
`ground`, `nonground`, `uncertain`이다. RViz의 세 그룹은 같은 rear-axle
`base_link` 위치이며 화면용 평행이동이 없다. 한 그룹만 켜거나 여러 그룹을
겹쳐 볼 수 있다. 입력 stamp·intensity·`source_index`가 보존된다.
`source_index`는 해당 원본 scan 안의 인덱스이지 지속 객체 ID가 아니다.
V3 설정에서만 `/lidar_candidate/ground_live/fit_markers`의 RANSAC 평면
법선 화살표를 추가 발행한다. 녹색은 지면 채택, 적색은 거부된 평면이다.
선택 평면·센서 중단 상태를 같은 scan의 점군과 비교하기 위한 진단이며
터널 지면 GT가 아니다. [V3 결과](l2a2-v3-attitude-tunnel-selection-20261005.md)를
함께 확인한다.

재호의 `structural_removed`는 지면 탐색에서 먼저 제거된 수직 평면이고,
`native_retained`는 이것까지 제외한 기존 후단 입력이다. 공정한 지면 비교에는
`nonground`를 쓰고, 기존 구조물 처리의 영향은 별도로 본다.
linefit/Patchwork의 structural cloud는 비어 있으며 native와 nonground가 같다.

대기 중인 scan은 최대 한 개다. 밀리면 최신 입력으로 교체하고 공통
`queue_dropped`를 기록한다. 이 카운터는 노드 내부 누락이며 UDP/ROS 구독큐
이전의 손실 총합은 아니다. 100ms 이내의 과거 유효 IMU가 없거나 stamp가
잘못되면 계산하지 않고 화면을 지운다. 센서 중단 0.5초 또는 과도한 처리 지연도
이전 화면을 지운다. 미분류·입력 무효를 정상 빈 장애물 관측으로 해석하지 않는다.

진단과 옵션 전체는 [실행 명령](../../runtime-commands.md#10-개발용-lidar-실시간-지면-비교),
[측정 보고서](l2a2-v1-live-ground-review-20261004.md)를 본다.
이 실시간 경로는 **일반도로 지면 비교**이며 터널 벽 제거·군집·OBB·추적은 실행하지 않는다.

## 이전 단일 프레임 비교 기록


손재호 RANSAC 기준선, Patchwork++, VLP16 조정 linefit을 **동일한 Autonav
v3 GT 캡처 프레임**에서 나란히 확인하는 읽기 전용 도구다. 시뮬레이터의 현재
주행이나 운영 LiDAR 토픽을 구독·변경하지 않는다. 데이터가 없는 팀원 PC에서는
실행되지 않으며, 이 화면을 판제의 장애물 입력으로 사용하면 안 된다.

`asmc-ros-noetic` 컨테이너 안에서:

```bash
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash
roslaunch asmc_lidar ground_compare.launch frame_id:=sync_001211
```

기본 Patchwork++ 열은 고정 높이 1.80m의 **원본 기준선**이다. 이번에
실험한 25cm 노면높이 보호 후보를 대신 보려면 다음처럼 결과 파일과
후보 이름을 명시한다.

```bash
roslaunch asmc_lidar ground_compare.launch \
  frame_id:=sync_001211 \
  patchwork_results:=/data/lidar_assessment/20261004_gt_dual_ground/reports/sync_patchwork_vlp16_sweep_v2.jsonl \
  patchwork_key:=pp_all_guard_25cm
```

다른 장면은 `frame_id:=sync_001231` 등으로 재실행한다. RViz 창을 직접
열지 않고 토픽만 검사하려면 `rviz:=false`를 붙인다.

| RViz 열 | 토픽 | 표시 y 오프셋 |
|---|---|---:|
| 손재호 기준선 | `/lidar_candidate/ground_compare/jaeho` | +28m |
| Patchwork++ 선택 후보 | `/lidar_candidate/ground_compare/patchwork` | 0m |
| VLP16 조정 linefit | `/lidar_candidate/ground_compare/linefit` | -28m |

회색은 지면으로 제거한 점, 주황색은 남긴 점, 분홍색은 **GT 객체점인데
지면으로 삭제된 점**, 하늘색은 **GT 아스팔트점인데 남은 점**이다. GT
semantic은 이 진단 색상에만 사용하고 알고리즘 입력에는 넣지 않는다.
`/lidar_candidate/ground_compare/labels`는 프레임과 방법 이름을 표시한다.

세 열은 비교 편의를 위해 인위적으로 좌우 28m씩 옮겼다. 따라서 고정 프레임
`ground_comparison`은 실제 `base_link`나 `map`이 아니며 이 점군에 다시
좌표변환을 적용하거나 주행 계획에 사용하면 안 된다. 도구 내부에서는
Autonav v3 장착 변환을 한 번 적용해 센서 원점 점을 rear-axle 기준으로
놓고, 그 이후 화면 배치용 오프셋만 더한다. RViz의 세 PointCloud2 display는
각각 켜고 끌 수 있다.

개발 검증: `catkin_make --pkg asmc_lidar` 통과, `rviz:=false` 실행에서
세 PointCloud2가 같은 7,377점과 `ground_comparison` 프레임으로 발행됨을
확인했다. `rviz:=true`의 RViz 프로세스 기동과 preset 파싱도 확인했다.
이는 화면 기동 확인이지 모든 장면의 의미 품질 승인 결과가 아니다.
