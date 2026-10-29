# 카메라 근거를 사용하는 실측점 정제

기본 배포의 `fusion.views: all`, `mission15_no_cone_v1`, 전·후방 군집 허용거리와 semantic-motion 정책은 유지한다. 새 정제는 기본적으로 꺼져 있다. 원본 sensor ROI Z는 사용자 요청대로 [-2,+2]m다. 기존 전체 YAML을 복사해 카메라 모델이나 융합 정책을 되돌리지 않는다.

When `tunnel_only_updates: true` is selected (the `lidar_wall_vehicle.yaml`
profile), camera-owned point extraction, residual splitting, steel fragment
merging and measured-vehicle history run only inside the existing
`pre_entry_wall` route window. Its configured `lead_m: 30.0`,
`exit_buffer_m: 10.0`, lateral/direction checks and hysteresis are unchanged.
GPS loss retains a previously verified window; GPS loss elsewhere cannot arm
it. Outside that window refinement reports `enabled: false` and projects no
points. Global camera CAR classification and HD-map delivery protection are
separate and remain enabled, so the monitor can still show YOLO vehicle
protection or joint point/OBB support outside a tunnel.

## 프로필 생성

선택한 main 설정을 입력으로 새 실행 설정 파일을 생성한다. 입력과 기존 출력 파일은 덮어쓰지 않는다.

```bash
python3 src/asmc_lidar/tools/configure_point_refinement.py \
  --input src/asmc_lidar/config/lidar_pipeline.yaml \
  --output /tmp/lidar-integrated.yaml \
  --profile integrated --tuning keep
```

`--profile`은 disabled / steel / wall / integrated다. `keep`은 main의 기존 수치·모델·뷰를 보존한다. 이전 저상 장애물 튜닝은 `--tuning low-obstacle`로 명시적으로 선택한다: 일반 최소3점, 지면 near .23/far .40/끝32m, 터널 최소3점/제거폭 .20m. 전·후방 연결거리와 voxel .1m는 바꾸지 않는다. 결과 YAML은 `lidar.launch pipeline_config:=...`에 선택할 수 있으며, 파일 생성 자체가 실행 중인 노드를 변경하지는 않는다.

## 적용 범위

`camera_point_refinement.enabled`는 일반 표면 정제를 활성화한다. 차량의 여러 스캔 줄은 제한된 깊이·각도·크기 안에서 같은 영상 인스턴스에 속할 때만 결합한다. `independent_wall_cleanup`은 확정된 벽 구간의 실측점을 정리하고, `detached_wall_cleanup`은 전경 근거와 충분한 높이가 있는 별도 벽 군집만 처리한다. 불확실한 다른 객체나 사각형 안의 배경을 일괄 보호·삭제하지 않는다.

`steel_fragment_merge`는 원래 클래스 `steel_barricade`에만 적용한다. 원본 부모 군집의 80% 이상 소유권, 각각3..48점의 좁은 수직 지지대, 정확히 두 부모, 높이·거리·전체 형상·투영 점유 조건이 필요하다. 다른 클래스/인스턴스와 경쟁하면 보류한다. 남은 원본 군집을 버리지 않는다. OBB는 현재 스캔의 실제 점으로 계산하며 가상 점이나 GT 크기를 넣지 않는다.

`fusion.views: all`에서는 전방·좌·우 각각의 보정값과 인과적 영상 선택을 사용해 점 정제를 수행한다. 각 뷰는 동일한 현재 점군에 대해 후보를 만들고, 겹치는 후보는 신뢰도 순서로 하나만 선택한다. 다른 뷰의 미확정 고신뢰도 박스 안 점과 어느 뷰든 객체로 선택한 점은 벽 삭제에서 보호한다. 전방 rollback 경로는 유지한다. 후단 분류·보호는 기존 전방/좌/우 융합을 그대로 사용한다. 3뷰 분류는 정제 단계가 검증한 **동일 이미지·epoch·원본 박스 번호·원래 클래스**를 가진 실측점 소유권을 일반 IoU와 함께 사용할 수 있다. 기존 이미지 소비/±50ms 동기화·트랙+스캔 단위 확정·다른 뷰와의 충돌 정책을 우회하지 않는다. 같은 이미지 반복으로 다섯 번 확정하지 않고, mission15에서 제외한 cone도 다시 허용하지 않는다.

`/perception/lidar/trace`의 `camera_point_refinement.views`에 카메라별 영상 시각, 투영 행렬, 투영점 수, 최종 추출 수와 벽 제거 수를 기록한다. `decisions`의 `camera_id`와 `box`로 원본 YOLO 박스를 구분한다. `below_confidence`, `insufficient_nonwall_support`, `no_supported_surface`, `cross_view_overlap_conflict`를 구분해 검증할 수 있다. 투영점 합계는 카메라 간 중복 투영을 포함하며 최종 객체 점의 중복을 의미하지 않는다.

## 시간과 보정

### 차량의 실측점 이력 연결

별도 `camera_point_refinement.vehicle_history_enabled`가 true일 때, age 20 이상이고 vehicle로 확정된 트랙에서 서로 다른 카메라 이미지의 현재 점 분리가 5회 이상 확인되면 이력을 만든다. 다음 스캔의 검증된 IMU/CAN 적분으로 표면점을 옮기고, 현재 점에서 기본 0.25m 이내 대응점이 10개 이상이며 이전 표면 15% 이상을 지지할 때만 차량을 분리한다. 과거 점이나 예측 박스를 현재 측정으로 출력하지 않는다. 벽면 점, 다른 현재 카메라 객체 또는 서로 경쟁하는 이력은 제외한다.

확인된 현재 점은 원래 트랙 ID에만 연결하고, 작은 잔여 군집이 그 ID를 가져가거나 같은 스캔의 통합 단계가 다시 합치지 못하게 한다. 새 카메라 객체가 같은 이력 점과 대응할 때도 현재 이미지의 소유권은 유지하면서 원래 ID에 연결한다. 카메라 관측 시각·확정 횟수·miss 카운터를 갱신하지 않고 현재 실측 대응이 있는 객체의 vehicle 표시만 유지한다. 다른 현재 클래스와의 충돌은 덮어쓰지 않는다.

스캔 간 공백은 최대 0.2초, 마지막 카메라 확인부터의 수명은 `vehicle_history_max_camera_age_s` 기본 10초다. 차량 이력의 회전 보정은 검증된 샘플에서 최대 0.2rad까지 허용하며, 기존 카메라 투영용 0.05rad 제한은 바꾸지 않는다. 현재 측정이나 유효 보정이 끊기면 이력을 종료한다. 따라서 아직 카메라로 분리하지 못한 차량, 긴 가림, 높은 상대 운동 또는 이력 수명 초과까지 해결하는 기능은 아니다. 기능 기본값은 false이며 실행 프로필에서 명시적으로 켠다.

일반 정제와 벽 정리는 인과적으로 보정된150ms 안의 입력만 사용한다. Steel 기하는 검증된 IMU/CAN 적분 조건에서만 최대250ms다. `general_current` 가드가 steel의 긴 창으로 일반 벽 정리를 활성화하지 못하게 한다. 터널 `protection.motion_compensated_fallback`도 최대150ms이며, 3뷰 중 빠진 전방 입력만 대체하고 측면 입력을 덮지 않는다.

정차 CAN 속도 <=.1m/s, 연속 스캔 간격 <=.2초에서는 이전 실측점과 현재 실측점을 대조해 잠깐의 이미지 누락을 견딘다. 같은 epoch/현재 소유권을 확인하고, 원래 이미지750ms 상한을 갱신하지 않는다. 과거 박스만 연장하거나 사라진 다리의 점을 만들어내지 않는다. 좌표는 기존 ScanFrame 변환을 사용한다.

## 한계와 검증

부모 군집을 만들지 못한 원거리 고립점, 한 YOLO 사각형 안에 여러 바리케이드가 겹친 경우, 한쪽 지지대만 측정된 경우는 완전한 객체 복구를 보장하지 않는다. 측정점으로 만든 부분 OBB가 실물 전체 크기의 정답이라는 의미도 아니다. voxel5cm·최소1점 실험이나 카메라 추정 GT 박스는 포함하지 않는다.

회귀 검사는 실제 wall/car 실패를 축소한 `test/pipeline/data/wall_car_28_07.json`, 회전·경쟁 인스턴스·epoch·만료·현재 점 소유권 테스트와 main의3뷰·mission15·실행파일 신원 검사를 함께 사용한다. 프로필 생성기는 모델/뷰/전후방 허용거리를 보존하는지 검사한다. 전체 물리 검출 정확도는 bag별 정답 주석이나 별도 주행 검증이 필요하다.

## ??? ?? ?? ?? (2026-10-29)

experimental.tunnel_filter.wall_geometry_precedence? ?? false?? wall/integrated ?? ????? true?. ?? ???? ?? ??? ??? ??? ??? ????, ?? ?? ?? ?? ?? ?? ???? ??? wall_remove_distance_m ?? ?? ?? YOLO ?? ?? ??? ??? ???? ???? ???. ?? ?? ? ??? ?? ???? ??? ?? ?? ??? ?? ?? ?? ??? ????.

??? ??? scan ?? ?? ??? ???? car? ?? ??? 9m/??3.5m, bus?truck?22m/5m? ?? ??? ?? IoU? ?? ????? ?? ???? ??? ???. ? ??? ?? ?? ??? ???.

?? ?? ??? ??: roslaunch asmc_lidar lidar_wall_vehicle.launch. ?? lidar.launch? ??/???? ??? ???? ???. ? bag ?? ??? artifacts/live_wall_audit_20261029/wall_fix_validation/REPORT.md? ??.
