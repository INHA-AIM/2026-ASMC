# LiDAR read-only 평가 도구

L1-A의 pinned 실험 입력, 손재호/서준 revision과 ROS1 wire MD5는
[`config/baselines/lidar_l1a_20260926.yaml`](../../config/baselines/lidar_l1a_20260926.yaml)에
기록했다. [검증 보고서](../../docs/packages/asmc_perception/lidar-l1a-validation-20260926.md)는
live/offline 차이·ground label 분리·미션별 미검증 입력을 설명한다. repo checkout의
HEAD와 무관하게 Git의 pinned revision을 읽어 메시지 MD5와 source artifact SHA를
확인한다. bag은 `$ASMC_DATA` 아래에서만 읽는다.

```bash
python3 tools/lidar/validate_l1a_baseline.py --data-root ../data
# data가 없는 CI에서는 revision·repo config·MD5·회귀 목록만 확인
python3 tools/lidar/validate_l1a_baseline.py --skip-data
```

Git의 pinned main/손재호/서준 commit 객체가 로컬에 있어야 MD5를 재계산할 수 있다.
shallow checkout에서 객체가 없으면 검증이 실패하며, 해당 revision을 fetch한 뒤 다시
실행한다. 누락된 branch를 현재 HEAD 메시지 정의로 대신 계산하지 않는다.

`audit_tracking_bag.py`는 닫힌 ROS1 bag을 직접 읽어 LiDAR 출력의 좌표계·시간·두 출력
스트림의 전환 계약을 조사한다. ROS master와 replay는 필요 없고, 원본 bag은 변경하지 않는다.
현재 팀원 브랜치의 `/last_lidar`는 `map`, `/lidar_nogps`의 헤더는 **`velodyne`**이다.
후자는 실제로 자세 수평화된 센서 원점 좌표이므로 raw velodyne과도 다르다.
`base_link`로 간주하면 수평화 역변환과 센서 장착 오프셋이 빠진다.

```bash
# asmc-ros-noetic 컨테이너 안에서
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash
python3 /root/ws/tools/lidar/audit_tracking_bag.py \
  /data/rosbags/<run>/<closed-recording>.bag \
  --output /data/rosbags/audits/<run>/lidar_tracking_contract.json
```

파일 분할 bag은 시간순으로 나열한다. 보고서가 이미 있으면 덮어쓰지 않고 실패한다.
큰 원본 bag의 SHA는 수집 manifest에서 함께 보관한다. 이 도구는 전체 파일을 다시
해시하지 않으며 입력 경로와 크기만 기록한다.

| 옵션 | 기본값 | 의미 |
|---|---|---|
| `--map-topic` | `/last_lidar` | GPS 정상 시 판제 출력 |
| `--local-topic` | `/lidar_nogps` | GPS 음영 시 판제 출력 |
| `--expect-local-frame` | `velodyne` | 검증 대상 local frame; 향후 정식 변환 후 `base_link`로 검사 가능 |
| `--cloud-topic` | raw/ROI/RANSAC/cluster 5개 | 반복 지정하면 검사 cloud 목록을 대체 |
| `--sample-points-per-cloud` | `1000` | 균등 간격 XYZ 표본, `0`이면 payload 수치 검사를 끔 |
| `--gps-max-age-sec` | `0.5` | 출력과 비교할 GPS 수신 샘플 최대 나이 |
| `--pair-tolerance-ms` | `2` | 두 객체 배열 header stamp 짝 허용 오차 |
| `--output` | 필수 | 신규 JSON 결과 파일 |

보고서는 topic별 수신 Hz, header 역행/중복/영점, frame, bag-header 시간차,
점 수·필드 schema·유효 XYZ 표본과 track ID 재등장·중복·비정상 숫자를 담는다.
두 출력에 객체가 동시에 있는 경우, 둘 다 빈 경우, raw GPS 유효성 전환과 관측 가능한
출력 전환을 별도로 센다. 센서 stamp와 출력 stamp의 최근접 차이는 **관측치**이며, 어느
scan을 처리한 것인지 증명하거나 처리 지연으로 확정하지 않는다.
터널/bridge diagnostics의 숫자는 p50/p95/max, bool·상태 문자열은 각 값의 빈도로 모으므로
`protection_available`, 벽·천장·지면 제거 수, 처리시간을 scene별로 검토할 수 있다.
`/camera/image/compressed`, `/YOLO/BBox`의 실제 수신 수와 Hz도 함께 기록한다.

빈 배열은 센서 정상/장애물 없음/비활성 스트림을 구분할 상태 필드가 없으므로 free-space
증거로 쓰지 않는다. GPS 회복 직후 local 출력은 producer의 recovery hysteresis일 수 있다.
GPS raw 좌표와 최종 mode가 다르다는 이유만으로 오류 판정을 내리지 않는다.
GT 없는 ID 재등장은 ID switch 정확도 지표가 아니다. `base_link` 또는 `velodyne` 위치의
프레임 간 차이를 그대로 절대속도로 계산하지 않는다.

단일-frame 지면/벽 제거·객체 보존 성능은 별도 GT 평가가 필요하다. 팀원 브랜치
`feature/lidar-structural-surface-filter`의 `Main/tools/tunnel_filter_offline_eval.cpp`는
실제 처리 함수와 같은 경로를 호출하므로 필터 A/B 평가에 우선 재사용한다.

```bash
# ROS가 없는 호스트에서도 실행 가능한 순수 계약 검사
python3 -m unittest discover -s tools/lidar -p 'test_*.py'
```

`inventory_capture.py`는 Windows MORAI가 저장한 원본 `LIDAR_5`와 기존 capture manifest를
읽기 전용으로 대조한다. 전체 파일명 인벤토리와 균등 표본의 xyz 정합·센티널·semantic·
instance/box 밀도를 기록하고, 표본 Intensity/box SHA도 남긴다. 파일 이동이나 학습셋 승격은
수행하지 않는다. NumPy가 설치된 WSL 호스트에서 실행한다.

```bash
python3 tools/lidar/inventory_capture.py \
  --lidar-root /mnt/c/MoraiLauncher_Win/MoraiLauncher_Win_Data/SaveFile/SensorData/LIDAR_5 \
  --capture-root ../data/capture_runs \
  --rename-history ../data/run_id_rename_history.jsonl \
  --max-samples 100 \
  --output ../data/lidar_assessment/<experiment>/capture_inventory.json
```

`--run-prefix kcity_auto_sunny_003`처럼 원본 run을 제한할 수 있다. 개명된 run은 명시한
rename 이력과 manifest의 `custom_name`으로 연결하고 원본/manifest 프레임 집합이 같은지
별도로 기록한다. 매핑 불일치는 추정해 고치지 않는다. 디렉터리에서 발견됐다는 이유로 반려된 run을
평가셋으로 사용하면 안 된다. 점과 라벨은 단일 프레임 실험에 쓸 수 있지만, 기존 Capture
Mode는 파일 저장 동안 이동을 멈추므로 트래킹/속도/오도메트리는 연속 bag으로 평가한다.

## 26R1 정지 위치 실험 수집

`capture_stationary_scenes.py`는 gRPC로 Ego를 HD Map 중심선의 고주로/음영구간 위치로
옮기고 정지 상태로 bag과 비동기 actor snapshot을 수집한다. 개발 시뮬레이터와 bridge,
평가할 LiDAR producer가 실행된 상태에서 사용한다. 초기 Ego가 정지해 있어야 하며, 수집이
끝나거나 실패하면 원래 transform·gear·control mode 복원을 각각 시도한다.

```bash
# asmc-ros-noetic 컨테이너 안; 실행 시 실제 시뮬레이터의 Ego 위치가 바뀐다.
python3 /root/ws/tools/lidar/capture_stationary_scenes.py \
  --output /data/lidar_assessment/<experiment>/stationary \
  --seconds 35 --confirm-simulator-move

# 완료된 manifest의 측정 시간만 분석한다. 이동/3초 안정화 구간은 제외된다.
python3 /root/ws/tools/lidar/audit_stationary_scenes.py \
  --manifest /data/lidar_assessment/<experiment>/stationary/manifest.json \
  --output /data/lidar_assessment/<experiment>/stationary_scene_audit.json
```

수집기 옵션은 `--host host.docker.internal`, `--port 7789`, scene당 `--seconds 35`
(허용 5~120초)다. 출력 폴더는 신규 경로여야 한다. `initial`, `highway_A423`,
`shadow_A126_entry`, `shadow_A126_middle`, `recovered_initial`의 순서로 수집한다.
마지막 manifest의 `restore_complete`, `recording_complete`, `collection_complete`를 확인한다. 잘못 끝난 경우
`.active` bag이나 불완전한 scene을 정상 결과로 취급하지 않는다.

scene 이름과 GPS 음영 여부는 별개다. 감사 결과의 실제 `/gps` 유효 sample 수로 검증한다.
teleport 정지 실험은 벽·지면·천장 잔여와 처리량 조사용이며 연속 주행의 ego-motion,
tracking, 음영 진입/복귀 연속성 수락시험을 대신하지 않는다. actor snapshot은 evaluator
파일로만 보존되며 detector 입력으로 발행되지 않고, 동기화된 GT label로 주장하지 않는다.

## 팀원 offline evaluator 요약

`summarize_teammate_offline.py`는 `6435af1` evaluator의 `frames.jsonl` 배열 필드 정의를
명시하고 완료된 `summary.json`의 frame 수를 대조한다. pure Python으로 실행 가능하다.

```bash
python3 tools/lidar/summarize_teammate_offline.py \
  --frames ../data/lidar_assessment/<experiment>/offline/frames.jsonl \
  --scene-manifest ../data/lidar_assessment/<experiment>/stationary/manifest.json \
  --output ../data/lidar_assessment/<experiment>/offline_scene_summary.json
```

scene 경계는 원본 bag 수신 시각인 `rec`를 사용한다. evaluator의 `processing_delay`만큼
뒤에 도착한 센서 상태를 반영해 처리하므로 live callback scheduling과 동일하지는 않다.
출력의 `A`는 음영에서 이미 `B` 결과를 기존 tracker로 보낸 **통합 runtime**이다. `A`를
legacy-only baseline이라고 해석하지 않는다. 따라서 B 활성 구간에서 legacy RANSAC
replica와 다른 점군이 나오는 것도 예상된 결과다. 비교는 같은 입력의 독립 상태 sweep들로
하고, sweep 처리시간(보호 투영·ROS 출력 제외)과 live 전체 frame 시간은 구별한다.

`run_teammate_offline.py`는 26R1 기준 bag을 private ROS master에서 손재호 `6435af1`
평가기와 같은 source/params로 재실행한다. **새 출력 디렉터리**만 받으며 현재 주행 ROS
master를 재사용하지 않는다. `--expect-main-path`가 `rospack find Main`과 불일치하면
중단한다. `invocation.json`에는 해석된 command·source 파일 SHA·한계를 기록한다.
이전에 수집된 `offline_v2/invocation.json`에는 이 SHA 필드가 없으므로 소급해 있었다고
쓰지 않는다. 정확한 container source 명령과 판제 인계 판단은
[L1-A 검증 기록](../../docs/packages/asmc_perception/lidar-l1a-validation-20260926.md)을 따른다.

`fixtures/l1a_contract_cases.yaml`은 첫 frame/time 수정의 독립 합성 입력이다. GPS,
camera 경계 및 좌표·속도 기댓값을 고정하며 `test_contract_fixture.py`는 fixture 자체의
수학적 일관성만 검사한다. 현재 LiDAR producer 통과를 주장하는 테스트가 아니다.

## L1-B1 독립 좌표·시간 코어

[`frame_time/README.md`](./frame_time/README.md)의 C++/Eigen 코어와 JSONL 평가는
수평화 역변환·장착·map pose·속도 의미·정수 ns를 검증한다. 운영 토픽을 발행하지 않는다.
[결과와 한계](../../docs/packages/asmc_perception/lidar-l1b1-validation-20260927.md),
[고정 manifest](../../config/baselines/lidar_l1b1_20260927.yaml)를 함께 읽는다.

## L1-B2 GPS 전환·출력 선택 격리 후보

[`gps_transition/README.md`](./gps_transition/README.md)는 GPS fix·stale·복구·점프와
`base_link`/`map` 출력 및 판제 상태를 분리한 독립 평가다. 실제 LiDAR/bridge/판제
토픽을 변경하지 않는다. [고정 결과](../../docs/packages/asmc_perception/lidar-l1b2-validation-20260927.md)와
[manifest](../../config/baselines/lidar_l1b2_20260927.yaml)를 따른다.
