# 파트 간 인계

현재 실행과 이전 목표를 구분합니다. Active 인계는 소비 규칙을 뜻하며
독립 수락·전체 이전·공동 자율주행 완료를 뜻하지 않습니다.

| 경계 | 현재 문서 | 남은 책임 |
|---|---|---|
| 인지→판제 구간 소비 | [상시 발행·소비 범위](perception-to-planning.md) | 판제: 상시 입력에서 미션별 사용·예외 결정, A153 끝·A126 진입 전 준비 |
| 카메라→Main/LiDAR | [전방 연결·3뷰 관측](camera-to-lidar.md) | 손재호: 카메라별 버퍼/보정/3뷰 fusion·전체 asmc_lidar 이전 |
| LiDAR 전체 이전 | [이전 계획](lidar-migration.md) | 기존 Main 동작 보존·구조물/추적/유효성·판제 소비 검증 |
| 카메라 family→판제 | [음영 인계](camera-to-planning.md) | 판제: ODD·모드·차로/corridor·fallback·실제 제어 수락 |
| HD Map→판제 | [지도 인계](hd-map-to-planning.md) | map copy/parity·좌표·route·GPS 무효/복구 소비 |
| 미션 지도→시나리오 | [미션 시나리오](hd-map-mission-scenario.md) | 구간/배치·공식 checkpoint와 시나리오 완주 정의 구분 |
| 대회 규정→전체 | [v1.2 영향](../competition/rule-v1.2-impacts.md), [에셋/라벨](../competition/obstacle-assets-and-labels.md) | 최신 주최 공지·실제 F2 범위 확인 |

판제 담당자는 [HD 적용 확인표](hd-map-to-planning.md#판제-적용-전-확인--2026-10-20-코드-대조)와
[원본 클래스·그룹·유효성 확인표](perception-to-planning.md#판제-적용-확인표)를 함께 검토한다.
두 PC/채점 프로그램 연결은 [LAN2LAN 안내](../operations/lan2lan-rehearsal.md)를 따른다.
3뷰 fusion은 손재호, YOLO 학습은 장원태가 진행하며 카메라 담당은 기존 계약·자료·재현 지원을 제공한다.

인계에는 producer/consumer·topic/type·원본/그룹·frame/단위·source 시각·freshness·무효/빈 상태,
모델/설정/보정 ID·실제 검증·실패·rollback과 담당별 완료 조건을 남깁니다.
Marker/GUI를 정본 planning 데이터로 사용하지 않습니다.
현재 전체 실행은 [명령 안내](../operations/commands.md), 담당/우선순위는
[대회 완료 로드맵](../architecture/consolidation-roadmap.md)을 봅니다.
