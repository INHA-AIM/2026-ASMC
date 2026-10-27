# LiDAR 공동 작업표

**현재 다음 작업:** 3뷰 구현·기본 적용 이후 [음영 판제 기반/후속 계획](../../../docs/architecture/shadow-planning-refactoring.md)을 진행한다.
객체 전달 문자열과 식별자·콘솔을 우선 정리하며 터널 local 회피 및 패키지 흡수는 다음 단위다.
터널 품질·얇은 장애물·OBB/추적·semantic 보조는
[보류 과제 인수 단위](improvement-milestones.md#보류-과제-인수-단위)에 착수 조건·완료 기준·복구 경계와 함께 보존한다.

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
| M0 · 보류 | 착수 시 담당/검토자 확인 | C0/F1·터널 자료의 선택 실패 사례만 단계별 연결. 새 광범위 진단/수집은 아님 |
| M1 · 기본 승격/공동 검증 대기 | **안승현** | [3뷰·웹·정적 보조 구현](three-view-fusion.md). 3뷰 기본·전방 복구, semantic 보조 OFF |
| M2 · 보류 | 기존 검토 창구 손재호/승현, 재개 시 확인 | PR #119·#122 구현됨. 실제 물체 보존/확인 지연·local 공급 검증 잔여 |
| M3 · 보류 | 기존 인수안 손재호/승현, 재개 시 확인 | 새 개선 미착수. 관측 위치·OBB 중심/yaw/치수 안정화 |
| M4 · 보류 | 기존 인수안 손재호, 재개 시 확인 | 새 개선 미착수. M3 관측 고정 후 연관→속도→운동 상태 순서 |
| R1 · 보류 | 재개 담당 미정 | 얇은 장애물 복구 설계 보존. 첫 소실 단계 확인 후 실제 점 지지가 있을 때만 후보 검증 |
| V1 · 보류 | 재개 담당 미정 | semantic 정적 보조 구현됨/기본 OFF. 기하·속도 기준선 고정 후 OFF/ON 수락 검증 |
| Q1 · 보류 | 인지/실행 공동 검토, 착수자 확인 | 기존473.47ms 공백 원인과 비용. 음영 정지에 직접 연결되면 우선 재개 |
| 병행 · 진행 | 장원태 | YOLO detect 추가학습. 모델 인수/성능 수락 대기, seg는 조건부 후속 |
| M5 · 보류 | 판제 포함 공동 | 채택된 개선의 전체 미션·GPS0/복귀·센서 중단/재시작·공급 수락. 모든 후보 완료를 전제로 하지 않음 |
| 근거 확보 시 · 보류 | 승현·재호 공동 | 실제 source clock 대응·ego motion. receive 기준 M1과 구분 |

상태는 미착수/진행/검증 대기/완료/보류로 기록한다. 기본 알고리즘·임계값을 바꾸는 개선은
정본화와 별도 커밋·자료로 비교한다. 상세 범위·선행 조건·후보와 인수 기준은
[개선 계획과 보류 인수 단위](improvement-milestones.md)에 한 번만 관리한다.
보류는 후속 계획의 우선순위이며 다른 팀원의 별도 개발 중단 지시가 아니다.
판제는 TTC·추종·정지·회피·센서 장애 대응을 소유한다.

## 현재 완료된 기반

책임별 디렉토리·공용 식별 계약, local/map 출력, 3뷰 fusion·수신 API,
HD맵 v7·10cm·점군3점/10% AND OBB10%·상부 제외·관측 확인3/5가 연결돼 있다.
필터는 전달 배열만 줄이고 tracker·측정 OBB는 보존한다. 3뷰 matching은 최적화 후 기본 승격했다. 전체 장면·공급 공동 수락과 구분한다.

C0 시험 도구는 [frame_time](../../../tools/perception/frame_time/README.md)에 있다. 재호는 고정
projection/scan/pose fixture를 추적·fusion 변경의 좌표 회귀에 재사용할 수 있다. 서로 다른 속도 추정값의
차이는 정확도 오류로 확정하지 않았다. F1에서 동시 YOLO 관측을 확보했지만 **원본 clock 대응과 독립 속도 GT는 미확보**다.
원래 IMU stale26 기록은 보존하고 후속에서 원인과 수정 결과를 연결했다. 오류를 정상으로 승격하지 않았으며
의도적인 텔레포트 시험만 정차 후 `/perception/lidar/lidar_pipeline/reset`으로 새 이력을 시작한다.
터널 전체 수락 완료로 표시하지 않는다.

## 복구

구 패키지 실행은 삭제 전 고정 revision에서만 재현한다. 현재 checkout에 구 overlay나
두 번째 LiDAR 생산자를 연결하지 않는다. [정본화 manifest](../../../config/baselines/lidar_canonicalization_v1.json)를 따른다.

2026-10-28: 사용자 승인으로 최적화한3뷰를 기본 승격했다. [승격 검증·후속 복구 설계](fusion-recovery.md)를 따른다.
얇은 장애물의 미연결 점 복구·벽 폭 사전정보는 보류 후보이며 바로 다음 PR의 확정 범위가 아니다.
semantic 운동 보조는 OFF를 유지한다.


## PR #122 이후 기본 실행 정리

3뷰·터널 최적화 소스가 병합돼도 기존 바이너리와 실행 프로세스는 자동 갱신되지 않는다.
[빌드 일치·검증 기록](../../../docs/records/lidar/2026-10-28-canonical-runtime-build.md)과
[직접 점검 명령](three-view-fusion.md#빌드와-실제-실행파일-확인)을 따른다.
다음 작업 선택은 [음영 정지 경계 우선 제안](improvement-milestones.md#pr-122-이후-실행-기반과-다음-단위-제안--2026-10-28)을 검토하며,
실행 기반 정리를 전체 음영 주행 수락이나 RANSAC 결함 해결로 표시하지 않는다.
