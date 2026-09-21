# 현재 팀 문서 인벤토리와 제안 위치

> **기준:** `main@577231b` · 2026-09-22
>
> **주의:** 아래 표는 이동 계획이며 현재 파일은 그대로 유지한다.

## 루트 문서

| 현재 파일 | 현재 역할 | 대상 | 제안 위치 | 다음 단계 |
|---|---|---|---|---|
| `README.md` | 문서 허브·정본 우선순위 | 전원 | `README.md` | 유지 |
| `getting-started.md` | 첫 build/run | 신규 팀원 | `onboarding/getting-started.md` | 이동 |
| `development-environment.md` | OS·ROS·데이터 경로 | 개발자 | `onboarding/development-environment.md` | 이동 |
| `team-dev-guide.md` | 담당 경로·검증 책임 | 전원 | `onboarding/team-dev-guide.md` | 이동 후 ownership 중복 검토 |
| `collaboration.md` | Git·ownership 정책 정본 | 전원 | `governance/collaboration.md` | 이동 |
| `git-workflow.md` | Git 명령 예시 | 전원 | `governance/git-workflow.md` | 이동 후 collaboration과 중복 축소 |
| `competition-environment.md` | 규정·허용 환경 요약 | 전원 | `competition/environment.md` | 이동 |
| `competition-data.md` | map/path/scenario 자산 | 전원 | `competition/assets.md` | 이동 |
| `architecture-overview.md` | 전체 시스템 개요 | 전원 | `architecture/system-overview.md` | 이동 |
| `repository-layout.md` | 현재 코드·ownership | 전원 | `architecture/repository-layout.md` | 이동 |
| `repository-consolidation-roadmap.md` | 6단계 통합·배포 계획 | 통합 담당 | `architecture/consolidation-roadmap.md` | 이동, Phase 2 완료 상태만 후속 갱신 |
| `integration-plan.md` | 과거 Wave 1/2 이식 기록 | 통합 담당 | `archive/integration/wave1-wave2-plan.md` | 이동, Legacy banner 검토 |
| `network-udp-ros.md` | UDP/ROS·포트 정본 | 통합·브리지 | `operations/network-udp-ros.md` | 이동 |
| `docker-guide.md` | 컨테이너 실행·장애 대응 | 전원 | `operations/docker.md` | 이동 |
| `sim-verification-checklist.md` | 시뮬 통합 검증 | 전원 | `operations/sim-verification-checklist.md` | 이동, 레거시 항목 후속 분리 |
| `rosbag-evaluation-streaming-gt.md` | bag 재현 평가·과거 GT 실험 | 인지·통합 | `operations/rosbag-evaluation.md` | 이동 후 archived experiment 절 분리 검토 |
| `morai-msgs-beta-policy.md` | MORAI message 호환 정책 | 브리지·통합 | `contracts/simulator/morai-msgs-beta-policy.md` | 이동 |
| `morai-msgs-versions.md` | 버전 차이 조사 기록 | 브리지·통합 | `records/simulator/morai-msgs-versions.md` | 이동 |
| `planning-control-udp.md` | standalone planner/control 실행 정본 | 판제 | `operations/planning-control.md` | 이동 |
| `planning-control-refactoring-guidelines.md` | 판제 구조·dependency 원칙 | 판제 | `packages/planning-control/refactoring.md` | 이동 |
| `fsm-frenet-interface-v2.md` | FSM/Frenet 상세 계약 | 판제·통합 | `contracts/planning/fsm-frenet-v2.md` | 이동, 계약/설계 절 분리 여부 후속 검토 |
| `planning-behavior-handoff.md` | 이전 ROS 통합 인수인계 | 판제 | `handoffs/planning/behavior-legacy.md` | 이동, Legacy 유지 |
| `d1-legacy-behavior-boundary.md` | 과거 D1 경계 기록 | 판제 | `archive/planning/d1-legacy-behavior-boundary.md` | 이동 |
| `perception-map-fusion-roadmap.md` | 인지 전체 개발 로드맵 | 인지·통합 | `packages/asmc_perception/roadmap.md` | 이동 |
| `perception-bev-lane-interface-plan.md` | BEV·차로 상세 설계안 | 인지·판제 | `packages/asmc_perception/bev-lane-interface-plan.md` | 이동, 확정 계약만 contracts로 후속 승격 |
| `perception-dataset-labeling-split.md` | 데이터셋·라벨·split 정본 | 인지 | `contracts/perception/dataset-labeling-split.md` | 이동 |
| `perception-v4-training-evaluation.md` | TwinLiteNet+ v4 실험 기록 | 인지 | `records/perception/twinlitenet-v4-evaluation.md` | 이동 |
| `perception-agentic-loop.md` | 초기 반복 평가 아이디어 | 인지 | `archive/perception/agentic-loop.md` | 이동, Research Reference 유지 |
| `camera-object-detection-handoff.md` | 객체 검출 인수인계 | 인지·LiDAR | `handoffs/perception/camera-object-detection.md` | 이동 |
| `shadow-camera-perception-handoff.md` | 음영구간 인지→판제 계약 | 인지·판제 | `handoffs/perception/shadow-camera.md` | 이동 |
| `hd-map-route-curation-review-20260917.md` | 차선 선택 검수 이력 | HD Map | `records/hd-map/kcity2025-route-curation-20260917.md` | 이동 |

## 이미 하위 디렉터리에 있는 문서

| 현재 파일 | 역할 | 제안 위치 | 다음 단계 |
|---|---|---|---|
| `contracts/hd-map-topic-contract-v1.md` | HD Map message/topic 동결 계약 | `contracts/hd-map/topic-v1.md` | 이동 |
| `contracts/hd-map-runtime-visualization-v1.md` | HD Map runtime 소비 계약 | `contracts/hd-map/runtime-v1.md` | 이동 |
| `contracts/kcity2025-competition-metadata-v1.md` | 체크포인트·미션 정본 | `contracts/competition/kcity2025-metadata-v1.md` | 이동 |
| `baselines/runtime-baseline-v1.md` | P1-A 저장소 회귀 기준 | `records/baselines/runtime-v1.md` | 이동 |
| `baselines/kcity2025-drivable-area-input-audit-v1.md` | P2-A 입력 audit | `records/baselines/kcity2025-drivable-input-v1.md` | 이동 |
| `baselines/kcity2025-compact-drivable-map-v1.md` | P2-B 최종 주행면 product | `records/baselines/kcity2025-compact-drivable-v1.md` | 이동 |
| `asmc_msg/BehaviorCommand.msg.txt` | 과거 message 텍스트 사본 | `archive/generated-msg/BehaviorCommand.msg.txt` | `src/asmc_msgs` parity 확인 뒤 삭제 후보 |
| `asmc_msg/FrenetFeedback.msg.txt` | 과거 message 텍스트 사본 | `archive/generated-msg/FrenetFeedback.msg.txt` | `src/asmc_msgs` parity 확인 뒤 삭제 후보 |

## 저장소 밖 워크스페이스 문서

워크스페이스 상위 `docs/`에는 공식 PDF, MORAI 원문, 논문, 개인 분석과 제출 자료가 있다.
이 파일은 팀 저장소로 이동하지 않는다. 팀원이 반드시 알아야 하는 결론만 위 구조의
`competition/`, `contracts/`, `packages/` 문서에 출처와 함께 요약한다.

현재 main에 없는 LiDAR 평가 문서는 로컬 archive 브랜치
`archive/local-lidar-assessment-20260921`의 commit `997ba8e`에 원문을 보존했다. 팀 문서로
승격할지는 `packages/asmc_perception/` 정리 단계에서 별도로 검토한다.
