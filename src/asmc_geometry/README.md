# asmc_geometry

ROS에 독립적인 헤더 라이브러리다. 노드·메시지·모델·추정 이력을 만들지 않는다.

- `geodesy.hpp`: WGS84→ENU/UTM52, true ENU→grid yaw, GPS0 유효성.
- `time.hpp`: 같은 clock의 ns 표본 조회와 inclusive age 한계. 다른 clock의 수치는 직접 빼지 않는다.
- 기존 bridge/planning의 동일 수학을 옮겼다. LiDAR의 GeographicLib 투영은 유지하며 grid yaw 계산 절차를 공유한다.
- 위치 기준점·GPS lever arm·gain·예측/복구 정책은 소비자가 소유한다. bridge 안테나 상태를 판제 후륜축 pose로 무조건 치환하지 않는다.
- 카메라는 기존 source clock/session SLERP와 원본 크기 보정을 유지한다. C0의 공통 fixture로 C++/Python 수치를 대조하며 Python binding은 추가하지 않는다.

[좌표·시각 책임과 검증](../../docs/architecture/perception-coordinate-refactoring.md)을 따른다.
