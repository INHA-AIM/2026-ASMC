# L1-B3 — YOLO/LiDAR 도착 순서 매칭 격리 평가

이 디렉터리는 운영 노드가 아니다. `/YOLO/BBox`는 YOLO11m 객체 box이고,
TwinLite 차선·주행가능영역이나 신호등 결과가 아니다. 손재호
`6435af1`의 보호점 계산·터널 필터 원본은 변경하지 않는다. 이 평가기의
25ms 후보는 **스케줄러**만 시험하며, 실제 25ms 후보의 터널 점군을 다시
처리하지 않는다. 130ms offline sweep의 벽·cluster 수치는 같은 scan의
조건부 참고값이지 후보의 before/after geometry가 아니다.

[결과](../../../docs/packages/asmc_perception/lidar-l1b3-validation-20260928.md) ·
[고정 기준](../../../config/baselines/lidar_l1b3_20260928.yaml) ·
[공통 소비 계약](../../../docs/contracts/perception/lidar-planning-consumer-v1.md)

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
wall-clock 처리시간과 실제 25ms 후 geometry는 아직 측정한 값이 아니다.

49/50/51ms stamp 경계, 0/10/25/26/50/130/200ms 도착 지연,
유효 빈 검출, 영점·중복·역행 stamp, queue overflow, 보호 OFF와
미래 자료 미사용을 합성 시험한다. 실제 bag에는 invalid YOLO stamp가
없으므로 그 경로는 합성 회귀다. 기존 정차 5개 장면으로 연속 주행·
차량/보행자 검출 정확도를 결론 내리지 않는다.
