# LiDAR 정본화 기반

구 Main의 필요한 기능을 보존 이식하고 공용 관측 직접 소비를 검증한 뒤 B4에서 정본화한다.
현행 실행은 [planning-runtime](planning-runtime.md), 과거 기능 차이·B0–B3 증거는
[PR #111까지의 기반 기록](../../../docs/records/lidar/cutover-foundation-through-pr111.md)에 있다.

## 전환 기반 작업 단위

| 단위 | 상태와 결과 |
|---|---|
| B0 | 완료: 기능 차이·담당·계약 조사 |
| B1 | 완료: 기존 적용 속도·4단계 상태·관측 품질 보존 이식 |
| B2 | 완료: 공용 관측 직접 소비, Main/Msg 없는 새 실행 |
| B3 | 새 경로 사용자/판제 주행 기능 확인. 전체 미션·속도 정확도·터널 품질 인수는 후속 |
| B4 | 정본 입력·수집/시나리오 전환·구 소스/전용 의존성 폐기. 검증 결과는 아래 기록 |

[B4 소스 목록·검증 결과](../../../docs/records/lidar/runtime-canonicalization.md)를 따른다.
기능 확인을 전체 대회 수락으로 확대하지 않으며, 후속 개선은 [작업표](migration-tasks.md)에서 공동 진행한다.

## 손재호: GPS 없는 터널 local 인식

local 검출은 GPS와 독립적으로 유지한다. 벽/천장·혼합 군집·OBB jitter는 후속 개선이다.
[보존한 터널 사례·시험 조건](../../../docs/records/lidar/cutover-foundation-through-pr111.md#손재호-gps-없는-터널-local-인식)을 재사용한다.
카메라 drivable/family를 사용하더라도 바깥 흰선을 도로 끝이나 벽의 정답으로 취급하지 않고
흰선과 벽 사이의 실제 객체를 보존한다. IMU 자세 보정·ego motion 정렬·시간 누적은 서로 다른 작업이다.
