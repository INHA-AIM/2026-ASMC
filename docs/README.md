# ASMC 문서

현재 코드의 실행·소비 규칙과 남은 대회 준비를 안내합니다. 과거 실험값은 `records/`에서 시점과 조건을 확인합니다.

| 하려는 작업 | 시작점 |
|---|---|
| 처음 환경을 준비하고 빌드 | [온보딩](onboarding/README.md) |
| 운영·전체 시험 빌드 선택 | [빌드 모드·cache·중복 실행 방지](operations/build.md) |
| 개별·통합 실행과 옵션 확인 | [실행 명령](operations/commands.md), [launch 인자 목록](operations/launch-reference.md) |
| 전체 구조·담당·우선순위 확인 | [시스템 구조](architecture/system-overview.md), [대회 완료 로드맵](architecture/consolidation-roadmap.md), [ownership](onboarding/ownership.md) |
| 카메라 실행·시각화 | [카메라 README](../src/asmc_camera/README.md), [배포 모델](operations/camera-deployment.md), [IMU·피팅 기본 전환](records/camera/2026-10-18-joint-family-stabilization.md) |
| 인지 상시 발행·미션별 소비 | [인지→판제 소비 범위](handoffs/perception-to-planning.md) |
| 판제 적용 시 좌표·클래스 확인 | [HD 적용 확인표](handoffs/hd-map-to-planning.md#판제-적용-전-확인--2026-10-20-코드-대조), [객체·유효성 확인표](handoffs/perception-to-planning.md#판제-적용-확인표) |
| 두 Windows PC·UDP·채점 연결 | [LAN2LAN 준비](operations/lan2lan-rehearsal.md), [채널 대조](records/bridge/2026-10-20-lan-channel-review.md) |
| 신호등 ResNet 실행·최적화 검토 | [현재 BF16 경로와 TensorRT 후보](../src/asmc_camera/docs/traffic-light-runtime.md) |
| 카메라와 LiDAR 연결 | [카메라→LiDAR](handoffs/camera-to-lidar.md), [LiDAR 통합 이관](handoffs/lidar-migration.md), [공동 작업표](../src/asmc_lidar/docs/migration-tasks.md), [3뷰·OBB·추적 개선 계획](../src/asmc_lidar/docs/improvement-milestones.md), [L1 설계](../src/asmc_lidar/docs/migration-design.md), [현재 판제 연결·속도/상태·복구](../src/asmc_lidar/docs/planning-runtime.md), [전환 기반·터널 후속](../src/asmc_lidar/docs/cutover-foundation.md) |
| 음영구간 정지 진단·local 경로·패키지 통합 계획 | [현재 구현과 후속 단계](architecture/shadow-planning-refactoring.md), [LOCAL-2A 입력 정렬·벽 간격 측정](records/planning/2026-10-28-local-input-alignment.md), [LOCAL-2B 경로·속도·실패 검수](records/planning/2026-10-28-local-path-stability.md) |
| 음영구간 차선·판제 연결 | [카메라→판제](handoffs/camera-to-planning.md), [판제 운영](operations/planning-control.md) |
| 공용 메시지·좌표·시간 확인 | [계약 목록](contracts/README.md), [센서 원본/수신 시각](architecture/bridge-sensor-timing.md), [IMU 지면 투영](architecture/camera-imu-projection.md) |
| 인지·판제 좌표계와 IMU 공통화 후속 검토 | [현행 경계·pose 차이·단계별 리팩토링 후보](architecture/perception-coordinate-refactoring.md) — 판제 전환 기반 마련 이후 착수 |
| LiDAR·fusion 개선의 참고 논문과 구현 | [선별 baseline·Autoware·적용 조건](../src/asmc_lidar/docs/improvement-milestones.md#우선-참고할-기준선과-적용-조건), [과거 실측과 한계](records/lidar/2026-10-27-research-integration.md) |
| 대회 입력·객체 범위 확인 | [대회 환경](competition/README.md), [에셋과 라벨](competition/obstacle-assets-and-labels.md) |
| 데이터 수집·라벨·재현 | [데이터 준비](operations/data-preparation.md), [데이터 계약](contracts/dataset-labeling-split.md), [bag 평가](operations/rosbag-evaluation.md) |
| 실험 결과·실패 근거 확인 | [기록 목록](records/README.md) |
| CI 비용·병렬 회귀·실패 로그 확인 | [CI 운영](operations/ci.md) |

## 문서 배치와 갱신

중앙에는 `onboarding`, `architecture`, `operations`, `contracts`, `handoffs`, `competition`, `records`의 일곱 폴더만 둡니다.
안내는 각 폴더 바로 아래, 기록은 분야별 한 단계 아래에 둡니다. 패키지 빠른 시작은 패키지 README,
내부 설계는 `src/<package>/docs/`, 다른 파트가 소비하는 규칙은 중앙 계약·인계에서 관리합니다.

파일명은 영문 소문자·하이픈이며 시작점만 `README.md`입니다. 현행 안내에는 날짜·PR 번호·latest·final을 붙이지 않습니다.
계약의 버전은 호환성 변화에만 올립니다. 새 기록은 실제 실험일·주제를 쓰고 기존 기록의 식별자는 유지합니다.
옛 경로의 호환 안내 파일은 제거했으므로 새 문서에 옛 링크를 추가하지 않습니다.

규정의 권위는 주최측 원문이 우선입니다. 실행 기본값은 현재 launch/YAML, wire 형식은 `.msg/.srv`와 기계 계약을 대조합니다.
문서와 구현이 다르면 어느 쪽이 잘못됐는지 검사하고 producer·consumer를 함께 수정합니다.
기록·후보안은 현재 실행을 덮어쓰지 않으며, 구현·오프라인 검증·실시간 수락·미완료를 구분합니다.

코드 PR에는 바뀐 경계의 주석, 패키지 README, 해당 운영·계약 문서와 검증 근거를 함께 갱신합니다.
수치를 여러 안내에 복사하지 말고 원본 기록으로 연결합니다. launch 인자 목록은 생성기로 갱신하고 CI에서 비교합니다.

[문서 구조 이전](records/repository/documentation-restructure-v2.md) ·
[이번 현행화·소스 주석 검토](records/repository/2026-10-16-documentation-refresh.md) ·
[협업 규칙](onboarding/collaboration.md)
