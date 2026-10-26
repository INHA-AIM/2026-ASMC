# LiDAR 공동 개발 인계

`asmc_lidar`가 판제·수집·시나리오의 LiDAR 정본이다. 브릿지와 카메라는 각각 실행하고,
판제는 `/perception/lidar/tracks_map`의 공용 추적 관측을 직접 소비한다. 구 Main/Msg/nested workspace는
B4에서 폐기한다. 사용자와 판제의 새 경로 주행 확인을 기반으로 정본화하며, 전체 미션·속도 정확도 수락은 별도다.

## 현재 실행과 계약

```text
bridge → asmc_lidar (검출·추적·전방 카메라 fusion·전달 필터)
       → /perception/lidar/tracks_map → planning_inputs → /planning/objects → 판제
       → /perception/lidar/tracks_local (GPS 무효에도 local 기하 유지)
asmc_camera → /perception/camera/objects/observations → asmc_lidar
```

- [실행·메시지·속도 의미·검증·복구](../../src/asmc_lidar/docs/planning-runtime.md)
- [작업표와 공동 개발 경계](../../src/asmc_lidar/docs/migration-tasks.md)
- [카메라 연결](camera-to-lidar.md) · [공용 소비 계약](../contracts/lidar-planning-consumer-v1.md)
- [직접 실행 명령](../operations/commands.md) · [정본화 검증 기록](../records/lidar/runtime-canonicalization.md)

## 담당과 후속 순서

| 담당 | 다음 작업 |
|---|---|
| 안승현 | HD맵 필터·카메라 수신/보정/시각/라벨 연결 지원. 좌표·IMU 공통화는 기존 설계의 C0 조사부터 |
| 손재호 | 추적·속도·운동 상태 정확도, GPS 없는 터널 local 객체·벽/천장 처리, 이후 3뷰 matching/fusion |
| 판제 | TTC·추종·정지·회피·센서 장애 대응과 local/map의 미션별 소비 |
| 공동 | 좌표·시간·공용 계약 변경과 전체 미션 검증 |

인지 전달 필터는 HD맵 v7·안쪽10cm·점군3점/10% AND OBB10%·상부 제외와 관측 품질 확인을
적용한다. 원본 tracker·OBB를 유지하고 동일 ID를 모든 발행 계열에서 제외한다. 유효한 카메라
vehicle/dynamic_obstacle 매칭은 보호한다. LiDAR 속도 판정만으로 보호하지 않는다.

## GPS 없는 터널 개선 원칙

현재 local 기하 제공과 벽/천장 제거의 품질 수락은 구분한다. 새 map 위치를 만들어내거나 마지막
GPS를 계속 사용하지 않는다. 세부 실패·검증 목록은 [보존한 터널 인계](../records/lidar/cutover-foundation-through-pr111.md#손재호-gps-없는-터널-local-인식)를 따른다.
카메라 drivable/family는 보조 후보이며 아직 LiDAR 구조물 제거에 연결되지 않았다. 바깥 흰선과
벽 사이의 실제 객체를 보존하고, 선 밖 점군이나 semantic 구멍을 무조건 삭제하지 않는다.

## 구 실행 재현

구 실행과 연구 결과는 [삭제 전 소스·시험 목록](../../config/baselines/lidar_canonicalization_v1.json)과
[PR #111까지의 이관 기록](../records/lidar/migration-through-pr111.md)에 보존한다. 재현은 고정 revision의
별도 checkout에서 수행하며 현행 실행에 구 overlay나 두 번째 객체 생산자를 연결하지 않는다.
