# LiDAR 공동 작업표

`asmc_lidar`는 LiDAR 정본이다. 현재 실행·공용 메시지는 [실행 기준선](planning-runtime.md),
과거 L0–L8/A1 작업과 증거는 [PR #111까지의 작업 기록](../../../docs/records/lidar/migration-tasks-through-pr111.md)에 있다.

## 공동 개발 인수와 우선 작업

| 순서 | 담당 | 작업과 완료 근거 |
|---|---|---|
| B4 | 안승현, 판제·재호 검토 | 정본 입력·수집·시나리오 전환, 구 Main/Msg/전용 의존성 제거. [검증 기록](../../../docs/records/lidar/runtime-canonicalization.md) |
| 병행 1 | 손재호 | GPS 없는 터널 local 객체·벽/천장 제거, 실제 장애물·흰선 밖 여유 공간 객체 보존 |
| 병행 2 | 손재호 | 추적·속도·운동 상태: 후진 부호, yaw/가시 면 변화, IMU 무효 초기 경계와 정확도 |
| 다음 | 안승현·손재호 | [공통 좌표/IMU 설계](../../../docs/architecture/perception-coordinate-refactoring.md)의 C0 입력·좌표·시각 목록/fixture부터 |
| 다음 | 손재호, 안승현 연결 지원 | 뷰별 비차단 matching과 관측 ID·보정·시각을 보존하는 3뷰 fusion |
| 실패 근거에 따라 | 안승현 | HD맵 경계·점군/OBB 근거·카메라 보호·계산 비용 점검 |
| 공동 | 판제 포함 | 전 미션·GPS0/복귀·센서 중단/재시작·속도 정확도와 합동 수락 |

상태는 미착수/진행/검증 대기/완료/보류로 기록한다. 기본 알고리즘·임계값을 바꾸는 개선은
정본화와 별도 커밋·자료로 비교한다. 판제는 TTC·추종·정지·회피·센서 장애 대응을 소유한다.

## 현재 완료된 기반

책임별 디렉토리·공용 식별 계약, local/map 출력, 전방 fusion·3뷰 수신 API,
HD맵 v7·10cm·점군3점/10% AND OBB10%·상부 제외·관측 확인3/5가 연결돼 있다.
필터는 전달 배열만 줄이고 tracker·측정 OBB는 보존한다. 3뷰 수신 API 제공은 3뷰 fusion 완료를 뜻하지 않는다.

## 복구

구 패키지 실행은 삭제 전 고정 revision에서만 재현한다. 현재 checkout에 구 overlay나
두 번째 LiDAR 생산자를 연결하지 않는다. [정본화 manifest](../../../config/baselines/lidar_canonicalization_v1.json)를 따른다.
