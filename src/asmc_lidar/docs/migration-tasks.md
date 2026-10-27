# LiDAR 공동 작업표

`asmc_lidar`는 LiDAR 정본이다. 현재 실행·공용 메시지는 [실행 기준선](planning-runtime.md),
과거 L0–L8/A1 작업과 증거는 [PR #111까지의 작업 기록](../../../docs/records/lidar/migration-tasks-through-pr111.md)에 있다.

## 공동 개발 인수와 우선 작업

| 순서 | 담당 | 작업과 완료 근거 |
|---|---|---|
| B4 | 안승현, 판제·재호 검토 | 정본 입력·수집·시나리오 전환, 구 Main/Msg/전용 의존성 제거. [검증 기록](../../../docs/records/lidar/runtime-canonicalization.md) |
| C0 · 완료 | 안승현, 손재호 대조 자료 인계 | [좌표·시각 결과와 fixture](../../../docs/records/lidar/2026-10-26-frame-time-baseline.md). 857 scan 동등성·54 투영·실시간 GPS0 대조. 운영 수정 없음 |
| C0 후속 수정 · 완료 | 안승현 | [bridge GPS0 유효성·복귀](../../../docs/records/bridge/2026-10-26-gps-validity-recovery.md). 수정 전 v1 근거를 보존하고 v2·ROS 발행 회귀로 검증 |
| C0 리팩토링 · 완료 | 안승현 | [공용 수학·IMU causal 이력](../../../docs/records/lidar/2026-10-27-frame-time-refactor.md). stale26은 텔레포트 yaw 거부로 확인, 오래된 자세 사용3건 차단. scan 변환 재사용·점군 복사 제거 |
| F1 · 완료(공급 제한 명시) | 안승현 | [전방 native 결정·두 연결 결함·동시 기록](../../../docs/records/lidar/2026-10-27-front-fusion-verification.md). 좌우는 수신·투영까지만, matching 확대 아님 |
| M0 · 미착수 | 승현, 재호 검수 | C0/F1·터널 자료의 선택 실패 사례만 단계별 연결. 새 광범위 진단/수집은 아님 |
| M1 · 검증 대기 | **안승현** | [3뷰·웹·정적 보조 구현](three-view-fusion.md). 전방 복구 유지, 후보 기본 승격은 검증 결과에 따름 |
| M2 · 검증 대기 | 손재호, 승현 통합 지원 | PR #119 병합 완료(`52017fe2`). 회귀·실물 보존/확인 지연·실시간 확인 필요 |
| M3 · 미착수 | **손재호** | 관측 위치·OBB 중심/yaw/치수 안정화. 기존 L-shape·참조점 대조 |
| M4 · 미착수 | **손재호** | M3 이후 연관·miss·재초기화·속도/운동 상태. 오류 원인 하나씩 대조 |
| 병행 · 진행 | 장원태 | YOLO detect 추가학습. 모델 인수/성능 수락 대기, seg는 조건부 후속 |
| M5 · 미착수 | 판제 포함 공동 | 전체 미션·GPS0/복귀·센서 중단/재시작·속도 정확도와 공급 공동 수락 |
| 근거 확보 시 · 보류 | 승현·재호 공동 | 실제 source clock 대응·ego motion. receive 기준 M1과 구분 |

상태는 미착수/진행/검증 대기/완료/보류로 기록한다. 기본 알고리즘·임계값을 바꾸는 개선은
정본화와 별도 커밋·자료로 비교한다. 상세 범위·선행 조건·후보와 인수 기준은
[M0–M5 개선 계획](improvement-milestones.md)에 한 번만 관리한다. 판제는 TTC·추종·정지·회피·센서 장애 대응을 소유한다.

## 현재 완료된 기반

책임별 디렉토리·공용 식별 계약, local/map 출력, 전방 fusion·3뷰 수신 API,
HD맵 v7·10cm·점군3점/10% AND OBB10%·상부 제외·관측 확인3/5가 연결돼 있다.
필터는 전달 배열만 줄이고 tracker·측정 OBB는 보존한다. 3뷰 matching은 M1 내부 후보이며 운영 기본 승격과 구분한다.

C0 시험 도구는 [frame_time](../../../tools/perception/frame_time/README.md)에 있다. 재호는 고정
projection/scan/pose fixture를 추적·fusion 변경의 좌표 회귀에 재사용할 수 있다. 서로 다른 속도 추정값의
차이는 정확도 오류로 확정하지 않았다. F1에서 동시 YOLO 관측을 확보했지만 **원본 clock 대응과 독립 속도 GT는 미확보**다.
원래 IMU stale26 기록은 보존하고 후속에서 원인과 수정 결과를 연결했다. 오류를 정상으로 승격하지 않았으며
의도적인 텔레포트 시험만 정차 후 `/perception/lidar/lidar_pipeline/reset`으로 새 이력을 시작한다.
터널 전체 수락 완료로 표시하지 않는다.

## 복구

구 패키지 실행은 삭제 전 고정 revision에서만 재현한다. 현재 checkout에 구 overlay나
두 번째 LiDAR 생산자를 연결하지 않는다. [정본화 manifest](../../../config/baselines/lidar_canonicalization_v1.json)를 따른다.
