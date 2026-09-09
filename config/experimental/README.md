# Experimental MORAI sensor sets

이 디렉터리는 팀 기본 설정이 아닌 transport/calibration pilot용 UI export만 둔다.

ROS Streaming GT를 시험할 때 MORAI UI에서 RGB·Semantic·Intensity LiDAR·Instance LiDAR를
각각 ROS로 연결하고 **export한 JSON**을 `ASMC_sensor_set_GT_ros.json`으로 추가한다.
`commType` enum을 추정해 `config/ASMC_sensor_set_GT.json`을 직접 편집하지 않는다.

최소 조건:

- RGB와 Semantic 각 view의 topic과 frame ID가 서로 다름
- ID 5와 ID 11 topic이 서로 다름
- ID 11 raw topic은 `/morai/gt/lidar_instance_raw`
- `ws://127.0.0.1:9090` rosbridge 연결 확인
- 대회용 v2와 UDP GT 정본은 변경하지 않음

실험 절차와 승격 gate는
[`docs/rosbag-evaluation-streaming-gt.md`](../../docs/rosbag-evaluation-streaming-gt.md)를 따른다.

2026-09-09 첫 ROS 연결에서는 Network Settings만 정상 advertise됐고 센서 topic은 생성되지
않았다. 또한 MORAI의 `/Ego_topic` payload에는 현재 beta 메시지에 없는 `angular_velocity`가
포함돼 rosbridge 변환이 실패했다. 다음 실험 파일은 단순 transport 전환본이 아니라 UI에서
각 sensor topic/frame을 확인해 export한 파일이어야 하며, 메시지 버전 차이는 공용 정의 변경
전에 adapter 경계를 먼저 검토한다.
