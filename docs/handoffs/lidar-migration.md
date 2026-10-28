# LiDAR 공동 개발 인계

**현재 후속:** 3뷰 matching은 PR #122에서 기본 적용됐다. 이제 구 패키지 이관을 다시 하지 않는다.
[클래스·시각 전달/음영 콘솔과 다음 작업](../architecture/shadow-planning-refactoring.md)을 따른다.
음영 local 객체는 제공하지만 현재 bbox 회피 제어기가 이를 소비한 상태는 아니다.

`asmc_lidar`가 판제·수집·시나리오의 LiDAR 정본이다. 브릿지와 카메라는 각각 실행하고,
판제는 `/perception/lidar/planning_observations_map`의 공용 추적 관측을 직접 소비한다. 구 Main/Msg/nested workspace는
B4에서 폐기했다. 사용자와 판제의 새 경로 주행 확인을 기반으로 정본화했으며, 전체 미션·속도 정확도 수락은 별도다.

## 현재 실행과 계약

```text
bridge → asmc_lidar (검출·추적·3뷰 카메라 fusion·전달 필터)
       → /perception/lidar/planning_observations_map → planning_inputs → /planning/objects → 판제
       → /perception/lidar/tracks_local (GPS 무효에도 local 기하 유지)
asmc_camera → /perception/camera/objects/observations → asmc_lidar
```

- [실행·메시지·속도 의미·검증·복구](../../src/asmc_lidar/docs/planning-runtime.md)
- [작업표와 공동 개발 경계](../../src/asmc_lidar/docs/migration-tasks.md)
- [카메라 연결](camera-to-lidar.md) · [공용 소비 계약](../contracts/lidar-planning-consumer-v1.md)
- [PR #114 통합·판제 변경 대응표](../records/lidar/2026-10-26-pr114-canonical-integration.md)
- [직접 실행 명령](../operations/commands.md) · [정본화 검증 기록](../records/lidar/runtime-canonicalization.md)
- [C0 좌표·시각 대조](../records/lidar/2026-10-26-frame-time-baseline.md)
- [공용 좌표 수학·IMU26건 원인과 리팩토링 검증](../records/lidar/2026-10-27-frame-time-refactor.md)

- [F1 전방 fusion 검증·수정과 남은 제한](../records/lidar/2026-10-27-front-fusion-verification.md)

## 담당과 후속 순서

| 담당 | 다음 작업 |
|---|---|
| 안승현 | M1 3뷰 구현 이후 **음영 판제 기반 개선 우선**. 카메라/시각/라벨·HD맵 연결 지원 |
| 손재호 | 기존 M2 터널·M3 관측 위치/OBB·M4 추적/속도 인수안의 기술 검토 창구. 후속 재개 담당·시점은 별도 확인 |
| 장원태 | YOLO detect 추가학습·동일 val 회귀·배포 인수. seg는 별도 라벨/실행 pilot 이후 판단 |
| 판제 | TTC·추종·정지·회피·센서 장애 대응과 local/map의 미션별 소비 |
| 공동 | 좌표·시간·공용 계약 변경과 전체 미션 검증 |

인지 전달 필터는 HD맵 v7·안쪽10cm·점군3점/10% AND OBB10%·상부 제외와 관측 품질 확인을
적용한다. 원본 tracker·OBB를 유지하고 동일 ID를 모든 발행 계열에서 제외한다. 유효한 카메라
vehicle/dynamic_obstacle 매칭은 보호한다. LiDAR 속도 판정만으로 보호하지 않는다.

단위별 선행 조건·완료 기준·고전 기법 후보는 [개선 계획](../../src/asmc_lidar/docs/improvement-milestones.md)이 정본이다.
[보류 과제 인수 단위](../../src/asmc_lidar/docs/improvement-milestones.md#보류-과제-인수-단위)에
M2/R1/M3/M4/V1/Q1의 시작 근거·변경 범위·검사·복구·인계 항목을 둔다.
누구든 인수 시 담당/검토자를 기록하고 재개할 수 있으며, 보류를 구현 삭제나 검증 완료로 해석하지 않는다.
[참고 논문·Autoware/PCL 구현 목록](../../src/asmc_lidar/docs/improvement-milestones.md#우선-참고할-기준선과-적용-조건)에서
M3의 Autoware shape estimation과 M4의 tracker 참고를 구분한다.
[연구 브랜치 선별 통합](../records/lidar/2026-10-27-research-integration.md)은 과거 근거 보존이며 새 운영 성과가 아니다.

## GPS 없는 터널 개선 원칙

PR #119는 `52017fe2`에 병합됐다. 터널 알고리즘의 전체 실시간 수락과 M1 카메라 보호 확대 수락은 별도로 남긴다.
현재 local 기하 제공과 벽/천장 제거의 품질 수락은 구분한다. 새 map 위치를 만들어내거나 마지막
GPS를 계속 사용하지 않는다. 세부 실패·검증 목록은 [보존한 터널 인계](../records/lidar/cutover-foundation-through-pr111.md#손재호-gps-없는-터널-local-인식)를 따른다.
카메라 drivable/family는 보조 후보이며 아직 LiDAR 구조물 제거에 연결되지 않았다. 바깥 흰선과
벽 사이의 실제 객체를 보존하고, 선 밖 점군이나 semantic 구멍을 무조건 삭제하지 않는다.

## 구 실행 재현

구 실행과 연구 결과는 [삭제 전 소스·시험 목록](../../config/baselines/lidar_canonicalization_v1.json)과
[PR #111까지의 이관 기록](../records/lidar/migration-through-pr111.md)에 보존한다. 재현은 고정 revision의
별도 checkout에서 수행하며 현행 실행에 구 overlay나 두 번째 객체 생산자를 연결하지 않는다.

## M1 공동 확인

[3뷰 fusion·웹·정적 보조 구현과 복구](../../src/asmc_lidar/docs/three-view-fusion.md)를 따른다.
M1 당시 승현의 관측 연결·semantic 보조 후보와 재호의 M3/M4 인수안을 보존한다.
현재 재개 담당과 시점은 보류 작업표에서 확인한다.
M1은 공용 판제 메시지를 유지하며 최적화 후 사용자 승인으로3뷰를 기본 승격했다. 기존 운동 판단은 유지하며
semantic 보조는 OFF다. local은 GPS와 독립이고 map은 유효 pose에서만 제공한다.
[얇은 물체 복구·벽 폭 사전정보 검토](../../src/asmc_lidar/docs/fusion-recovery.md)는 후속 PR로 분리한다.
