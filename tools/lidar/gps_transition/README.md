# L1-B2 — GPS 건강 상태와 출력 선택 격리 평가

운영 ROS 노드가 아니다. 손재호 LiDAR, bridge, 판제, 메시지/MD5, HD Map geometry를
변경하지 않는다. [결과](../../../docs/packages/asmc_perception/lidar-l1b2-validation-20260927.md) ·
[기준](../../../config/baselines/lidar_l1b2_20260927.yaml) ·
[소비 계약](../../../docs/contracts/perception/lidar-planning-consumer-v1.md).

`GpsGate`는 raw fix의 status, 좌표 유효성, 동일 clock 정수 ns stamp/arrival,
0.5초 수신 freshness, 3회 복구 확인, 비현실적 위치 점프를 구분한다. 40 m/s + 1.5 m
점프 기준은 **격리 시험용 가설**이다. 첫 점프 거부 때의 이동 허용 거리를 고정하므로
시간만 흐르는 것으로 거부가 해제되지 않는다. 다른 독립 pose 출처가 동시각·근접성을
증명하면 reanchor 후보가 될 수 있지만, raw GPS 자신은 승인자가 아니다. 텔레포트는
의도적으로 거부하므로 이 기준을 시뮬 수동 이동 기능에 그대로 적용하면 안 된다.

`select_output`은 B1의 이미 변환된 객체 snapshot을 받아 local `base_link`와 map을
**서로 다른 고정 frame**으로 선택한다. 관측 시각에 신뢰 가능한 map pose가 없으면
local 중심/8코너/ID/stamp를 보존하고 map을 이용 불가로 표시한다. 빈 배열만으로
`observed_empty`, 센서 누락, detector 미실행, 무효 입력, pose/GPS 실패를 표현하지 않는다.
실제 트래커 상태를 수정하거나 ID를 새로 할당하지 않는다. `planner_ready`에는 객체 map
가용성뿐 아니라 vehicle state와 trajectory의 별도 준비 상태를 요구한다. 이는 합성
수락조건일 뿐 판제 기동/정지 규칙을 바꾸지 않는다.

```bash
# 호스트: ROS 없는 합성 계약 검사와 고정 산출물 확인
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s tools/lidar/gps_transition -p 'test_*.py'
python3 tools/lidar/gps_transition/verify_evaluation.py --data-root ../data

# asmc-ros-noetic 컨테이너: ROS master 없이 원본 bag 읽기만 수행
source /opt/ros/noetic/setup.bash
python3 /root/ws/tools/lidar/gps_transition/audit_bag.py \
  --bag /data/lidar_assessment/20260926/stationary_v2/stationary_0.bag \
  --scene-manifest /data/lidar_assessment/20260926/stationary_v2/manifest.json \
  --output /data/lidar_assessment/20260927/<new-run>/audit.json
python3 /root/ws/tools/lidar/gps_transition/run_snapshot.py \
  --input /data/lidar_assessment/20260927/l1b1_v3/results.jsonl \
  --output /data/lidar_assessment/20260927/<new-run>/snapshot.json
```

출력 파일은 덮어쓰지 않는다. 고정 실행본은 `l1b2_v4`다. v1은 점프 거부
허용 거리가 시간에 따라 다시 커지는 결함을 발견한 **실패 실험**이다. v2는 점프
수정 후, v3는 arrival-order 검사 후, v4는 잘못된 관측 stamp 처리 후 실행이다.
v2~v4의 결과 JSON은 byte-for-byte 동일하다.
`audit.json`은 측정 window별 GPS/scan 상태와 output topic **메시지 개수**만 센다.
두 출력 topic의 메시지 수가 같아도 각 메시지의 객체가 유효하다는 뜻은 아니다.
bag에는 당시 수평화 행렬과 관측시각 rear-axle pose가 없어서 map 변환·tracker/판제
정확도를 평가하지 않는다. 정차 구간 사이에는 gRPC 텔레포트가 있어 연속주행 jump
검증의 정답으로 사용하지 않는다.
