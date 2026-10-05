# asmc_lidar — 손재호 기능 이전용 골격

현재는 **빌드 가능한 좌표·시간 라이브러리와 계약 시험**이다. 검출·군집·추적·fusion·
ROS 발행 노드는 손재호가 기존 `lidar+sensorfusion`에서 이전하며 구현한다.
실제 주행에는 기존 패키지를 사용한다. 이 패키지에는 실행 launch가 없다.

담당 작업과 완료 조건은 [LiDAR 인계 문서](../../docs/packages/asmc_lidar/README.md),
카메라와의 역할 분담은 [통합 로드맵](../../docs/architecture/consolidation-roadmap.md)에 있다.

## 남긴 구성

| 경로 | 역할 |
|---|---|
| `include/asmc_lidar/frame_time.hpp`, `src/frame_time_core.cpp` | 실제 수평화 역변환·장착 변환·관측시점 pose 보간·속도 의미 변환 |
| `include/asmc_lidar/imu_selection.hpp` | 이미 도착한 IMU 중 scan 시각 이전의 최신 유효 표본 선택, 누락/stale 진단 |
| `config/autonav_v3.yaml` | 장착 변환의 출처/SHA와 값. 후속 ROS 어댑터에서 검증·로딩할 참고 설정 |
| `test/` | 중심/8코너·시간·pose·속도·IMU 선택의 회귀시험 |

`asmc::lidar::transform()`은 local 결과와 map 가용성을 분리한다. 호출자는 실제 적용한
수평화 행렬과 장착 변환을 넣어야 한다. `PoseHistory`는 동일 clock의 도착 완료 pose만
최대 100ms 간격에서 보간하고 외삽하지 않는다. map 기준 높이와 pose 출처 검증은
호출자의 책임이며, 이 라이브러리가 GPS/차량 상태를 구독하거나 추정하지는 않는다.
`SelectImu()`의 `newer_invalid_seen`도 진단에 전달해야 한다.

점 변환 함수는 XYZ/intensity를 보존한다. 후속 point-cloud 어댑터는 ring/time 등 추가
필드와 원본 index 계보를 별도로 보존해야 한다. 점별 시간이나 deskew 기능은 없다.

## 빌드·시험

Noetic 컨테이너의 저장소 루트에서 실행한다. 기존 build 산출물을 재사용하지 않고
별도 build/devel 경로로 현재 소스만 검사한다.

```bash
source /opt/ros/noetic/setup.bash
cd /root/ws
catkin_make --build /tmp/asmc_lidar_handoff_build \
  -DCATKIN_DEVEL_PREFIX=/tmp/asmc_lidar_handoff_devel \
  --only-pkg-with-deps asmc_lidar -DCATKIN_ENABLE_TESTING=ON -j2
(cd /tmp/asmc_lidar_handoff_build/asmc_lidar && ctest --output-on-failure)
source /tmp/asmc_lidar_handoff_devel/setup.bash
python3 src/asmc_msgs/test/test_message_serialization.py
```

새 골격은 PCL·외부 지면분할 라이브러리·`Main`·`Msg` nested workspace에 의존하지 않는다.
손재호가 실제 처리 단계를 추가할 때 필요한 의존성만 명시한다. 공용 메시지는
`asmc_msgs`에 정의돼 있지만 이 골격이 토픽을 발행하지는 않는다.

이전 연구 소스·보고서는 정리 전 커밋 `020f6c2`에 남아 있다. 선택적 복구는 별도
checkout에서 검토한다. 예전 `unified.launch`와 `ground_compare_live.launch`는 삭제됐다.
기존 devel에 같은 이름의 실행파일이 남아 있어도 현재 패키지의 지원 기능으로 사용하지 않는다.
