# L1-B3 — YOLO/LiDAR 도착 순서 매칭 격리 평가

이 디렉터리는 운영 노드가 아니다. /YOLO/BBox는 YOLO11m 객체 box이며 TwinLite 차선·주행가능영역,
신호등 결과가 아니다. 손재호 pinned 6435af1의 보호점·tunnel filter 원본은 변경하지 않는다.
run_evaluation.py는 인과적 FIFO scheduler를 검사하고 run_geometry.py는 scan 도착 시 고정한
점군·pose·선택 box로 pinned filter를 실제 재실행한다. 모두 격리 평가이며 운영 node에 연결하지
않는다. 130ms offline sweep은 별도 조건의 참고값이다.

[L1-B3 기록](../../../docs/packages/asmc_perception/lidar-l1b3-validation-20260928.md) ·
[L1-B3-G 기하/AIM 결과](../../../docs/packages/asmc_perception/lidar-l1b3g-validation-20260928.md) ·
[고정 기준](../../../config/baselines/lidar_l1b3_20260928.yaml) ·
[공통 소비 계약](../../../docs/contracts/perception/lidar-planning-consumer-v1.md)

v1 산출물과 source SHA는 historical revision에서 검사한다. 현재 스케줄러는
결과 정렬 없이 FIFO를 유지하며, 중복/역행 양수 YOLO stamp는 원본과 같이
수용한다. 원본 scan stamp와 밀리초 절삭 matching stamp를 분리한다.
`q[0]`을 벽 잔여점으로 이름 붙였던 해석은 정정되었다.

## 재실행

호스트에서는 합성 회귀와 저장된 SHA·결정을 읽기 전용으로 확인한다.

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s tools/lidar/camera_sync -p 'test_*.py'
PYTHONDONTWRITEBYTECODE=1 python3 \
  tools/lidar/camera_sync/verify_evaluation.py --data-root ../data
```

bag 재생은 `asmc-ros-noetic` 안에서 실행한다. 원본 bag을 복사·수정하지 않고
출력 디렉터리는 기존 경로면 실패한다. 실행 전 저장소·detached nested
workspace overlay를 source해야 `Msg/Yolo_bbox_arr`를 읽을 수 있다.

```bash
docker exec -it asmc-ros-noetic bash
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash
source /data/lidar_assessment/20260926/jaeho_source/src/lidar+sensorfusion/devel/setup.bash --extend
PYTHONDONTWRITEBYTECODE=1 python3 /root/ws/tools/lidar/camera_sync/run_evaluation.py \
  --bag /data/lidar_assessment/20260926/stationary_v2/stationary_0.bag \
  --scene-manifest /data/lidar_assessment/20260926/stationary_v2/manifest.json \
  --reference-130ms /data/lidar_assessment/20260926/offline_v2/frames.jsonl \
  --output /data/lidar_assessment/20260928/<new-run>
```

`decisions.jsonl`은 원본 scan 식별자·메시지 SHA·점 수·frame,
scan/YOLO 관측·도착·결정 정수 ns, 선택 box 수, 거부 사유와 130ms
conditional reference를 담는다. `summary.json`에는 장면별 count,
대기 p50/p95/p99, 130ms 필터 처리시간 **참고값**, wait sensitivity가
따로 기록된다. `manifest.json`은 입력 SHA와 한계를 보존한다.
wait p50/p95/p99와 offline filter-stage CPU 시간은 분리 기록한다. ROS publish·tracker·동시 GPU 부하를 포함한 end-to-end latency는 측정하지 않았다. same-scan geometry 결과는 L1-B3-G 보고서를 참조한다.

49/50/51ms stamp 경계, 0/10/25/26/50/130/200ms 도착 지연,
유효 빈 검출, 영점·중복·역행 stamp, queue overflow, 보호 OFF와
미래 자료 미사용을 합성 시험한다. 실제 bag에는 invalid YOLO stamp가
없으므로 그 경로는 합성 회귀다. 기존 정차 5개 장면으로 연속 주행·
차량/보행자 검출 정확도를 결론 내리지 않는다.


## L1-B3-G 재현: filter geometry와 AIM audit

ROS/PCL build dependency는 asmc-ros-noetic container를 사용하고 evaluator는 /tmp에 빌드한다.
pinned source는 읽기 전용으로 source한다.

    docker exec -it asmc-ros-noetic bash
    source /opt/ros/noetic/setup.bash
    source /root/ws/devel/setup.bash
    source /data/lidar_assessment/20260926/jaeho_source/src/lidar+sensorfusion/devel/setup.bash --extend
    cmake -S /root/ws/tools/lidar/camera_sync -B /tmp/asmc-l1b3g-build -DLEGACY_MAIN_SOURCE=/data/lidar_assessment/20260926/jaeho_source/src/lidar+sensorfusion/src/Main
    cmake --build /tmp/asmc-l1b3g-build --parallel 2

run_evaluation.py에서 새 FIFO 결정을 만들고 run_geometry.py에서 같은 scan의 frozen snapshot으로
filter를 비교한다. 각 output 경로는 새 이름이어야 하며 덮어쓰지 않는다.

    python3 /root/ws/tools/lidar/camera_sync/run_evaluation.py --bag /data/lidar_assessment/20260926/stationary_v2/stationary_0.bag --scene-manifest /data/lidar_assessment/20260926/stationary_v2/manifest.json --reference-130ms /data/lidar_assessment/20260926/offline_v2/frames.jsonl --output /data/lidar_assessment/20260928/<new-fifo-run>

    python3 /root/ws/tools/lidar/camera_sync/run_geometry.py --bag /data/lidar_assessment/20260926/stationary_v2/stationary_0.bag --scene-manifest /data/lidar_assessment/20260926/stationary_v2/manifest.json --decisions /data/lidar_assessment/20260928/<fifo-run>/decisions.jsonl --setting /data/lidar_assessment/20260926/jaeho_source/src/lidar+sensorfusion/src/Main/setting.yaml --params /data/lidar_assessment/20260926/offline_v2/resolved_params.yaml --executable /tmp/asmc-l1b3g-build/tunnel_geometry_eval --output /data/lidar_assessment/20260928/<new-geometry-run>

run_synthetic_geometry.py는 동일 setting/params/binary로 positive box, valid-empty, 벽 근접 car/person/low-object fixture를 만든다. 매번 새 output 경로를 지정한다.

    python3 /root/ws/tools/lidar/camera_sync/run_synthetic_geometry.py --setting /data/lidar_assessment/20260926/jaeho_source/src/lidar+sensorfusion/src/Main/setting.yaml --params /data/lidar_assessment/20260926/offline_v2/resolved_params.yaml --executable /tmp/asmc-l1b3g-build/tunnel_geometry_eval --output /data/lidar_assessment/20260928/<new-synthetic-run>

AIM audit는 native data를 복사하지 않고 metadata와 표본만 읽는다. host에서:

    ASMC_AIM_DATA=/mnt/d/asmc_aim/dataset python3 -B tools/lidar/camera_sync/audit_aim.py --autonav-sensors config/morai/sensors/ASMC_sensor_set_v3.json --output "$ASMC_DATA/lidar_assessment/20260928/aim_audit_<new-run>"

다른 host는 ASMC_AIM_DATA만 바꾼다. profile/session은 contract와 producer report 기준이며 정확도 학습 승인 또는 물리 동기화 증명은 아니다. output 경로가 이미 있으면 audit는 거부한다.

    PYTHONDONTWRITEBYTECODE=1 python3 -B -m unittest discover -s tools/lidar/camera_sync -p 'test_*.py'
    PYTHONDONTWRITEBYTECODE=1 python3 -B tools/lidar/camera_sync/verify_evaluation.py --data-root ../data
