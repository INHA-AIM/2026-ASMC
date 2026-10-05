# L1-B1 — isolated frame/time core

운영 ROS 노드가 아니다. `src/`·토픽·MD5·HD맵·센서 JSON은 변경하지 않는다.
[결과](../../../docs/packages/asmc_perception/lidar-l1b1-validation-20260927.md) ·
[계약](../../../docs/contracts/perception/lidar-planning-consumer-v1.md) ·
[manifest](../../../config/baselines/lidar_l1b1_20260927.yaml).

## 빌드·실행

호스트에서 `docker exec -it asmc-ros-noetic bash`로 진입한다. 코어는 C++17/Eigen만
사용한다. JSONL CLI에는 yaml-cpp, Python 평가에는 NumPy/rosbag이 필요하다.
기존 컨테이너의 설치본을 사용했으며 새 ROS 의존성을 설치하지 않았다.

```bash
# 컨테이너 안. 기존 빌드/운영 devel을 수정하지 않는다.
source /opt/ros/noetic/setup.bash
cmake -S /root/ws/tools/lidar/frame_time -B /tmp/asmc-l1b1-core \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/asmc-l1b1-core -j2
cmake -E chdir /tmp/asmc-l1b1-core ctest --output-on-failure
FRAME_TIME_BUILD=/tmp/asmc-l1b1-core PYTHONDONTWRITEBYTECODE=1 \
  python3 -m unittest discover -s /root/ws/tools/lidar/frame_time -p test_io.py
```

원본 adapter까지 비교하려면 L1-A에서 빌드한 **고정 detached source**의 headers/messages만
추가로 사용한다. 메인 workspace를 해당 branch로 바꾸거나 nested source를 수정하지 않는다.

```bash
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash
source /data/lidar_assessment/20260926/jaeho_source/src/lidar+sensorfusion/devel/setup.bash --extend
cmake -S /root/ws/tools/lidar/frame_time -B /tmp/asmc-l1b1-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DLEGACY_MAIN_SOURCE=/data/lidar_assessment/20260926/jaeho_source/src/lidar+sensorfusion/src/Main
cmake --build /tmp/asmc-l1b1-build -j2
cmake -E chdir /tmp/asmc-l1b1-build ctest --output-on-failure
FRAME_TIME_BUILD=/tmp/asmc-l1b1-build PYTHONDONTWRITEBYTECODE=1 \
  python3 -m unittest discover -s /root/ws/tools/lidar/frame_time -p test_io.py

# 출력 디렉터리가 이미 있으면 실패한다. 평가마다 신규 이름을 지정한다.
PYTHONDONTWRITEBYTECODE=1 python3 /root/ws/tools/lidar/frame_time/run_evaluation.py \
  --build /tmp/asmc-l1b1-build --data-root /data \
  --output /data/lidar_assessment/20260927/l1b1_review_01
```

기록된 결과는 `l1b1_v3`다. 초기에 실패한 `l1b1_v1`, 예비 통과 `v2`를 덮어쓰지 않았다.
`legacy.cmake`는 원본 adapter/header의 SHA가 달라지면 configure를 거부한다. 원본의
순수 point/vector 함수만 실행하며 tracker·ROS publisher는 실행하지 않는다.
두 `Deg2Rad` helper만 probe에 같은 수식으로 제공해 무관한 ROS global 구현을 링크하지 않는다.
원본 helper float 정밀도까지 유지하므로 legacy의 pitch-only 오차는 약 `1e-7`이다.

호스트에서 저장된 기준선 검증:

```bash
python3 tools/lidar/validate_l1a_baseline.py --data-root ../data
python3 tools/lidar/frame_time/verify_evaluation.py --data-root ../data
python3 -m unittest discover -s tools/lidar -p 'test_*.py'
# 참고 checkout 감사: 호스트 Python 3.9+; 신규 출력만 가능
python3 tools/lidar/frame_time/audit_references.py \
  --baselines ../external/baselines --output /tmp/lidar_reference_review.json
```

## JSONL 입출력 계약

실제 예시는 `run_evaluation.py::snapshots()` 및 결과 폴더의 `snapshots.jsonl`이다.
stdin 한 줄은 한 관측 snapshot, stdout 한 줄은 동일 `input_id` 결과다.
`frame_time_eval < snapshots.jsonl`로 재실행 가능하다(입력 cloud 절대 경로가 같은 환경).

| 필드 | 의미 |
|---|---|
| `input_id`, `stream_id` | 원본 식별자, stamp 순서를 검사할 센서 스트림 |
| `frame` | `sensor`, `leveled_sensor`, `base_link`만 허용 |
| `base_from_sensor`, `calibration_source` | 3×3 proper rotation + translation[m], 교정 출처/SHA. sensor 입력에 필수 |
| `applied_leveling`, `leveling_source` | 실제 적용했던 회전과 근거. leveled 입력 필수; IMU 최근접 추정으로 대체하지 않음 |
| `time` | `stamp_ns`, `receipt_ns`, `processing_ns` 정수 ns, 명시적 `clock`, `stamp_provenance` |
| `pose_samples` | 도착 순서의 stamp/arrival/clock/valid/pose/reference. `gps_antenna`이면 `gps_in_base` 필수 |
| `max_pose_gap_ns` | 100,000,000 기준. cutoff까지 도착한 유효 동일 clock bracket만 보간, 외삽 없음 |
| `twist` | 선택. 동일 관측 stamp/clock의 `linear_base`, `angular_base`와 도착/유효성. 보간은 이번 코어에 없음 |
| `objects` | ID/class, center, 8개 3D corners, long_axis, velocity와 명시적 의미 |
| `cloud_path` | 선택. little-endian float32 XYZI 파일, x86 평가기. 비유한 XYZ나 잘못된 크기는 조용히 삭제하지 않고 실패 |

속도 의미는 `absolute_input_vector`(축 회전만), `base_position_derivative`(base 입력에서만
`v_rel + v_ego + ω×p`), `unknown`이다. 수평화/sensor 위치 미분은 시간에 따른 원점·회전
정보가 더 필요하므로 base 미분이라고 재표기하지 않는다. unknown 결과는 **null**, 0이 아니다.
3D 코너가 정본이고 yaw는 long_axis의 XY 투영에서 얻는 부가 값이다(수직이면 null).

상위 `valid/reason`은 관측 시간 검사의 상태, 각 객체 `valid/reason`은 local geometry 상태다.
`map`과 속도는 별도 가용성을 가지며 사유가 남는다. 잘못된 pose timing은 map을 막고
local은 보존한다. 파싱/schema 오류는 `input_error`와 종료코드 2로 드러낸다.
`objects=[]`만 보지 말고 상태를 확인해야 한다. 시간 오류를 fresh empty로 바꾸지 않는다.

bag replay는 기록된 도착 시각 순서다. raw는 수신 stamp를 유지하고 가상 지연을 더하지
않는다. 합성 snapshot의 처리 cutoff는 fixture에 명시한다. 양쪽 pose가 이미 도착했을 때만
보간할 수 있어 source stamp가 관측보다 큰 upper sample과 **미래 도착 데이터**를 구분한다.
clock 이름 일치는 호출자 계약이며 다른 clock의 동기화/offset을 자동 추정하지 않는다.

## 산출물·범위

`report.json`에 revisions, compiler/platform, source/binary SHA, 입력 SHA, 비교/성능/한계를
기록한다. JSONL에는 정수 ns·원본 ID·가용성/거부 사유를 보존한다. 재실행 시 timing 필드만
제외한 geometry/status를 canonical JSON으로 비교한다. 성능은 steady_clock, warmup 3회,
cloud 31회/object 100회 반복의 nearest-rank p50/p95/p99다. JSON decode·파일 I/O·ROS
메시지 복사·센서 지연·deskew는 포함하지 않는다. 별도 copy 측정과 copy 포함 변환을 함께 낸다.

정차 bag 2,562개 header와 5개 raw cloud, admitted Capture **ROI stage** 100개만 검증한다.
Capture의 `stamp=1`은 시험용 placeholder이며 측정 시각이라고 주장하지 않는다.
live 당시의 내부 matrix snapshot은 없어 raw cloud에 legacy leveling을 추정 적용하지 않는다.
검출/속도 정확도·연속 주행·GPS 상태기계·운영 호환 연결은 후속 작업이다.
