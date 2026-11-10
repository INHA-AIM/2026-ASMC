# LiDAR 정본 입력과 재밍 연결

PR #114의 속도 품질·GPS 복귀·재밍 출구 처리를 B4의 단일 실행 경로에 통합했다.
현재 명령은 [commands](commands.md), 타입·무효 처리·복구는
[LiDAR 실행 정본](../../src/asmc_lidar/docs/planning-runtime.md)을 따른다.

## 현재 연결

`asmc_lidar/lidar.launch` → `/perception/lidar/planning_observations_map`
(`asmc_msgs/LidarPlanningObservation`) → `planning_objects_adapter` → `/planning/objects`.
카메라·LiDAR·판제를 별도로 한 번씩 실행한다. `planning_output`, `lidar_backend`,
`lidar_source_policy`, `asmc_lidar_companion` 선택과 `autonomy_asmc_lidar.launch`는 정리했다.
판제 담당은 기본 `autonomy_no_bridges_v2.launch`를 사용하고 구 overlay/JSON/wire를 연결하지 않는다.
시험 시 입력 remap과 `namespace:=/test/lidar`만 사용한다.

`tracks_local/map`의 기존 공용 추적 계약은 분석용으로 유지한다. 판제용 KF 속도와 일반 classifier
추정값은 같다고 가정하지 않는다. 속도 미확정의 유효 기하는 UNKNOWN과 `velocity_unavailable`로
남기며 측정 속도0이나 정상 빈 도로로 바꾸지 않는다. 같은 scan의 필터 결과는 모든 출력에 동일 적용한다.

## main 병합 후 입력 선택

과거 기록의 이 anchor는 보존한다. 현재는 입력 선택이 없으며 위 단일 공용 관측을 따른다.
과거 companion/구 Main 실행 인자는 현행 launch에 전달하지 않는다.

## 유지한 판제 수정과 과거 근거

카메라 BBox frame bridge는 전방 `/perception/camera/objects`를 직접 사용하고 영상과 동일 stamp만
받는다. 정상 빈 검출도 전달한다. 재밍 어댑터의 진입 차로 복귀와 GPS recovery 시험을 보존한다.
위치 무효를 정상으로 꾸미거나 기존 장애물/오류 STOP을 해제하지 않는다.

[04:18 주행 bag](../records/lidar/2026-10-26-full-drive-bag-validation.md),
[초기 실행 기록](../records/lidar/2026-10-26-cutover-validation.md)은 PR #114 이전 구성의 증거다.
이번 통합의 전체 주행 수락으로 재사용하지 않는다. 모델/속도 추정 알고리즘을 다시 변경한 것은 아니다.
구 Main 복구는 고정 revision의 별도 checkout에서만 한다. 현행 구 패키지·중복 필터를 복원하지 않는다.
