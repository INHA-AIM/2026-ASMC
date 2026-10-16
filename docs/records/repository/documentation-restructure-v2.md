# 문서 구조 이전 v2 — 2026-10-16

이번 변경은 문서 위치·이름·링크·탐색 목차 정리다. 본문 현행화·통합·실질적인 legacy 삭제는 다음 단계다.
실행 설정·실험 수치·완료 상태·계약 의미와 과거 명령을 변경하지 않았다.

## 기준과 검증 범위

- 시작 commit: `d36f29b6e5dad468c2668ff2e3823e5de8de49b5` (PR #87 병합 main).
- 시작 중앙 문서: Markdown 122개, 전체 파일 129개.
- 중앙 상위 폴더는 7개, Markdown 경로는 최대 두 단계다.
- 아래 표와 [기계 이동 목록](../../../config/docs/documentation_relocation_v2.json)은 이번 이전의 스냅샷이다.
- 현행 문서 전체를 이 스냅샷의 SHA로 영구 동결하지 않는다. 상시 검사와 일회성 내용 보존 검사를 구분한다.
- 기존 카메라/재밍 이력은 중앙 records로, 패키지 내부 설명은 해당 패키지 docs로 이동했다.
- 기록이 없는 records/planning 폴더는 만들지 않았다. 분야 기록이 생기면 같은 깊이에 추가한다.

## 기존 history 검사 선언의 정정

이동 전 `--with-history`가 HD맵 snapshot 2개에서 실패했다. 이전 manifest의 byte_identical 선언과 달리
이미 기준 main에 수치·product SHA 정정이 들어 있었다. 이번 작업에서 그 파일 내용을 고치지 않았다.

- compact-drivable: 과거 원문 SHA `95430c5b3fbc5beb9a4517faae11b1de335358a9dbbdd7a9e2d72a3071a0d9ff`,
  현재 동결 SHA `2ffa747d15caede30767b555ae26816e48e62a8c24b9d7ad3f96d5ec96cc3cc6`.
  기존 변경 이력: `4add3a55`, `147d7f8c`, `9105954a` 등.
- drivable-input: 과거 원문 SHA `b0ace45d45af4a5b56317950f958bbb206b5a4fc651f33868b590850d8d90bf4`,
  현재 동결 SHA `beac0b15b194f8a82496c90595da8df834a2137103cb5e880a51b6294576d72f`.
  기존 정정: `db87e1fd`.

두 항목은 과거 원문 SHA와 시작 commit의 정정본 SHA를 각각 검증하도록 선언을 바로잡았다.
동결 자료 5개의 기존 SHA는 그대로 유지한다. 기준 commit과 기존 원본 목록도 바꾸지 않는다.

## 재현 검사

```bash
# 상시 검사: 현행 링크·anchor·구조와 필요한 동결 보존
python3 tools/audit/check_documentation_migration.py
# 과거 원문 및 시작 main의 정정본 근거 확인
python3 tools/audit/check_documentation_migration.py --with-history
# 이번 이전 당시만: 링크·추가 탐색 블록 외의 본문 보존
python3 tools/audit/check_documentation_migration.py --with-history --verify-relocation
```

마지막 명령은 후속 본문 현행화 이후의 상시 수락 명령이 아니다.
이전 전후 파일명을 고정한 아래 목록과 source SHA를 확인하는 일회성 검사다.

## 이동 목록

집계: `{"keep": 19, "move": 104, "remove_redirect": 12, "update_references": 26}`.
`remove_redirect`는 본문을 가진 문서 삭제가 아니라 기존 4줄 호환 안내 제거다.
`update_references`는 위치를 유지한 README 등의 링크 목적지만 수정한 경우다.

| 이전 경로 (시작 commit) | 최종 경로·대체 경로 | 처리 |
|---|---|---|
| [.github/pull_request_template.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/.github/pull_request_template.md) | [.github/pull_request_template.md](../../../.github/pull_request_template.md) | update_references |
| [CONTRIBUTING.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/CONTRIBUTING.md) | [CONTRIBUTING.md](../../../CONTRIBUTING.md) | update_references |
| [README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/README.md) | [README.md](../../../README.md) | update_references |
| [config/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/config/README.md) | [config/README.md](../../../config/README.md) | update_references |
| [config/baselines/evidence/camera_yoloe_l_first_v1/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/config/baselines/evidence/camera_yoloe_l_first_v1/README.md) | [config/baselines/evidence/camera_yoloe_l_first_v1/README.md](../../../config/baselines/evidence/camera_yoloe_l_first_v1/README.md) | update_references |
| [config/baselines/evidence/lidar_camera_frame_boundaries_v1/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/config/baselines/evidence/lidar_camera_frame_boundaries_v1/README.md) | [config/baselines/evidence/lidar_camera_frame_boundaries_v1/README.md](../../../config/baselines/evidence/lidar_camera_frame_boundaries_v1/README.md) | update_references |
| [config/baselines/evidence/lidar_continuous_regression_v1/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/config/baselines/evidence/lidar_continuous_regression_v1/README.md) | [config/baselines/evidence/lidar_continuous_regression_v1/README.md](../../../config/baselines/evidence/lidar_continuous_regression_v1/README.md) | update_references |
| [config/bridge/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/config/bridge/README.md) | [config/bridge/README.md](../../../config/bridge/README.md) | update_references |
| [config/morai/udp-sim-cheat-sheet.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/config/morai/udp-sim-cheat-sheet.md) | [config/morai/udp-sim-cheat-sheet.md](../../../config/morai/udp-sim-cheat-sheet.md) | update_references |
| [docs/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/README.md) | [docs/README.md](../../README.md) | keep |
| [docs/architecture/consolidation-roadmap.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/architecture/consolidation-roadmap.md) | [docs/architecture/consolidation-roadmap.md](../../architecture/consolidation-roadmap.md) | keep |
| [docs/architecture/repository-layout.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/architecture/repository-layout.md) | [docs/architecture/repository-layout.md](../../architecture/repository-layout.md) | keep |
| [docs/architecture/system-overview.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/architecture/system-overview.md) | [docs/architecture/system-overview.md](../../architecture/system-overview.md) | keep |
| [docs/archive/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/archive/README.md) | [docs/records/archive/README.md](../archive/README.md) | move |
| [docs/archive/generated-msg/BehaviorCommand.msg.txt](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/archive/generated-msg/BehaviorCommand.msg.txt) | [docs/records/archive/BehaviorCommand.msg.txt](../archive/BehaviorCommand.msg.txt) | move |
| [docs/archive/generated-msg/FrenetFeedback.msg.txt](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/archive/generated-msg/FrenetFeedback.msg.txt) | [docs/records/archive/FrenetFeedback.msg.txt](../archive/FrenetFeedback.msg.txt) | move |
| [docs/archive/generated-msg/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/archive/generated-msg/README.md) | [docs/records/archive/generated-msg-index.md](../archive/generated-msg-index.md) | move |
| [docs/archive/integration/wave1-wave2-plan.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/archive/integration/wave1-wave2-plan.md) | [docs/records/archive/wave1-wave2-plan.md](../archive/wave1-wave2-plan.md) | move |
| [docs/archive/perception/agentic-loop.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/archive/perception/agentic-loop.md) | [docs/records/archive/agentic-loop.md](../archive/agentic-loop.md) | move |
| [docs/archive/planning/d1-legacy-behavior-boundary.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/archive/planning/d1-legacy-behavior-boundary.md) | [docs/records/archive/d1-legacy-behavior-boundary.md](../archive/d1-legacy-behavior-boundary.md) | move |
| [docs/archive/planning/standalone-udp-baseline.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/archive/planning/standalone-udp-baseline.md) | [docs/records/archive/standalone-udp-baseline.md](../archive/standalone-udp-baseline.md) | move |
| [docs/collaboration.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/collaboration.md) | [docs/onboarding/collaboration.md](../../onboarding/collaboration.md) | remove_redirect |
| [docs/competition/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/competition/README.md) | [docs/competition/README.md](../../competition/README.md) | keep |
| [docs/competition/images/image-20230408-141822.png](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/competition/images/image-20230408-141822.png) | [docs/competition/images/image-20230408-141822.png](../../competition/images/image-20230408-141822.png) | keep |
| [docs/competition/images/스크린샷 2025-08-08 112023.png](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/competition/images/%EC%8A%A4%ED%81%AC%EB%A6%B0%EC%83%B7%202025-08-08%20112023.png) | [docs/competition/images/스크린샷 2025-08-08 112023.png](../../competition/images/%EC%8A%A4%ED%81%AC%EB%A6%B0%EC%83%B7%202025-08-08%20112023.png) | keep |
| [docs/competition/images/스크린샷 2026-06-30 111239.png](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/competition/images/%EC%8A%A4%ED%81%AC%EB%A6%B0%EC%83%B7%202026-06-30%20111239.png) | [docs/competition/images/스크린샷 2026-06-30 111239.png](../../competition/images/%EC%8A%A4%ED%81%AC%EB%A6%B0%EC%83%B7%202026-06-30%20111239.png) | keep |
| [docs/competition/images/스크린샷 2026-07-10 124338.png](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/competition/images/%EC%8A%A4%ED%81%AC%EB%A6%B0%EC%83%B7%202026-07-10%20124338.png) | [docs/competition/images/스크린샷 2026-07-10 124338.png](../../competition/images/%EC%8A%A4%ED%81%AC%EB%A6%B0%EC%83%B7%202026-07-10%20124338.png) | keep |
| [docs/competition/images/스크린샷 2026-07-13 162644.png](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/competition/images/%EC%8A%A4%ED%81%AC%EB%A6%B0%EC%83%B7%202026-07-13%20162644.png) | [docs/competition/images/스크린샷 2026-07-13 162644.png](../../competition/images/%EC%8A%A4%ED%81%AC%EB%A6%B0%EC%83%B7%202026-07-13%20162644.png) | keep |
| [docs/contracts/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/contracts/README.md) | [docs/contracts/README.md](../../contracts/README.md) | keep |
| [docs/contracts/competition/kcity2025-metadata-v1.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/contracts/competition/kcity2025-metadata-v1.md) | [docs/contracts/competition-kcity2025-metadata-v1.md](../../contracts/competition-kcity2025-metadata-v1.md) | move |
| [docs/contracts/hd-map-runtime-visualization-v1.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/contracts/hd-map-runtime-visualization-v1.md) | [docs/contracts/hd-map-runtime-v1.md](../../contracts/hd-map-runtime-v1.md) | remove_redirect |
| [docs/contracts/hd-map-topic-contract-v1.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/contracts/hd-map-topic-contract-v1.md) | [docs/contracts/hd-map-topic-v1.md](../../contracts/hd-map-topic-v1.md) | remove_redirect |
| [docs/contracts/hd-map/runtime-v1.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/contracts/hd-map/runtime-v1.md) | [docs/contracts/hd-map-runtime-v1.md](../../contracts/hd-map-runtime-v1.md) | move |
| [docs/contracts/hd-map/topic-v1.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/contracts/hd-map/topic-v1.md) | [docs/contracts/hd-map-topic-v1.md](../../contracts/hd-map-topic-v1.md) | move |
| [docs/contracts/kcity2025-competition-metadata-v1.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/contracts/kcity2025-competition-metadata-v1.md) | [docs/contracts/competition-kcity2025-metadata-v1.md](../../contracts/competition-kcity2025-metadata-v1.md) | remove_redirect |
| [docs/contracts/perception/camera-runtime-v1.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/contracts/perception/camera-runtime-v1.md) | [docs/contracts/camera-runtime-v1.md](../../contracts/camera-runtime-v1.md) | move |
| [docs/contracts/perception/dataset-labeling-split.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/contracts/perception/dataset-labeling-split.md) | [docs/contracts/dataset-labeling-split.md](../../contracts/dataset-labeling-split.md) | move |
| [docs/contracts/perception/lidar-planning-consumer-v1.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/contracts/perception/lidar-planning-consumer-v1.md) | [docs/contracts/lidar-planning-consumer-v1.md](../../contracts/lidar-planning-consumer-v1.md) | move |
| [docs/contracts/perception/runtime-v1.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/contracts/perception/runtime-v1.md) | [docs/contracts/perception-runtime-v1.md](../../contracts/perception-runtime-v1.md) | move |
| [docs/contracts/planning/fsm-frenet-v2.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/contracts/planning/fsm-frenet-v2.md) | [docs/contracts/planning-fsm-frenet-v2.md](../../contracts/planning-fsm-frenet-v2.md) | move |
| [docs/contracts/simulator/morai-msgs-beta-policy.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/contracts/simulator/morai-msgs-beta-policy.md) | [docs/contracts/simulator-morai-msgs-beta-policy.md](../../contracts/simulator-morai-msgs-beta-policy.md) | move |
| [docs/getting-started.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/getting-started.md) | [docs/onboarding/README.md](../../onboarding/README.md) | remove_redirect |
| [docs/governance/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/governance/README.md) | [docs/onboarding/collaboration.md](../../onboarding/collaboration.md) | move |
| [docs/handoffs/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/README.md) | [docs/handoffs/README.md](../../handoffs/README.md) | keep |
| [docs/handoffs/competition/obstacle-assets-and-labels.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/competition/obstacle-assets-and-labels.md) | [docs/competition/obstacle-assets-and-labels.md](../../competition/obstacle-assets-and-labels.md) | move |
| [docs/handoffs/competition/rule-v1.2-impacts.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/competition/rule-v1.2-impacts.md) | [docs/competition/rule-v1.2-impacts.md](../../competition/rule-v1.2-impacts.md) | move |
| [docs/handoffs/hd-map/mission-scenario-spec.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/hd-map/mission-scenario-spec.md) | [docs/handoffs/hd-map-mission-scenario.md](../../handoffs/hd-map-mission-scenario.md) | move |
| [docs/handoffs/hd-map/planning-consumption-v1.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/hd-map/planning-consumption-v1.md) | [docs/handoffs/hd-map-to-planning.md](../../handoffs/hd-map-to-planning.md) | move |
| [docs/handoffs/perception/camera-coco-class-expansion.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-coco-class-expansion.md) | [docs/records/camera/camera-coco-class-expansion.md](../camera/camera-coco-class-expansion.md) | move |
| [docs/handoffs/perception/camera-data-integration-prep.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-data-integration-prep.md) | [docs/records/camera/camera-data-integration-prep.md](../camera/camera-data-integration-prep.md) | move |
| [docs/handoffs/perception/camera-deployment-roadmap-20261014.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-deployment-roadmap-20261014.md) | [docs/records/camera/camera-deployment-roadmap-20261014.md](../camera/camera-deployment-roadmap-20261014.md) | move |
| [docs/handoffs/perception/camera-family-supply-optimization.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-family-supply-optimization.md) | [docs/records/camera/camera-family-supply-optimization.md](../camera/camera-family-supply-optimization.md) | move |
| [docs/handoffs/perception/camera-front960-validation.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-front960-validation.md) | [docs/records/camera/camera-front960-validation.md](../camera/camera-front960-validation.md) | move |
| [docs/handoffs/perception/camera-inference-control.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-inference-control.md) | [docs/records/camera/camera-inference-control.md](../camera/camera-inference-control.md) | move |
| [docs/handoffs/perception/camera-lidar-b0.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-lidar-b0.md) | [docs/records/camera/camera-lidar-b0.md](../camera/camera-lidar-b0.md) | move |
| [docs/handoffs/perception/camera-lidar-b1a.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-lidar-b1a.md) | [docs/records/camera/camera-lidar-b1a.md](../camera/camera-lidar-b1a.md) | move |
| [docs/handoffs/perception/camera-lidar-coordinate-reference-review.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-lidar-coordinate-reference-review.md) | [docs/records/camera/camera-lidar-coordinate-reference-review.md](../camera/camera-lidar-coordinate-reference-review.md) | move |
| [docs/handoffs/perception/camera-lidar-fusion-study.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-lidar-fusion-study.md) | [docs/records/camera/camera-lidar-fusion-study.md](../camera/camera-lidar-fusion-study.md) | move |
| [docs/handoffs/perception/camera-mission16-deployment.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-mission16-deployment.md) | [docs/operations/camera-deployment.md](../../operations/camera-deployment.md) | move |
| [docs/handoffs/perception/camera-mpc-object-collection-pilot.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-mpc-object-collection-pilot.md) | [docs/records/camera/camera-mpc-object-collection-pilot.md](../camera/camera-mpc-object-collection-pilot.md) | move |
| [docs/handoffs/perception/camera-object-detection.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-object-detection.md) | [docs/records/camera/camera-object-detection.md](../camera/camera-object-detection.md) | move |
| [docs/handoffs/perception/camera-objects-p2a.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-objects-p2a.md) | [docs/records/camera/camera-objects-p2a.md](../camera/camera-objects-p2a.md) | move |
| [docs/handoffs/perception/camera-rate-confidence-followup.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-rate-confidence-followup.md) | [docs/records/camera/camera-rate-confidence-followup.md](../camera/camera-rate-confidence-followup.md) | move |
| [docs/handoffs/perception/camera-runtime-plan.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-runtime-plan.md) | [docs/records/camera/camera-runtime-plan.md](../camera/camera-runtime-plan.md) | move |
| [docs/handoffs/perception/camera-runtime-structure-validation.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-runtime-structure-validation.md) | [docs/records/camera/camera-runtime-structure-validation.md](../camera/camera-runtime-structure-validation.md) | move |
| [docs/handoffs/perception/camera-seg-model-selection.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-seg-model-selection.md) | [docs/records/camera/camera-seg-model-selection.md](../camera/camera-seg-model-selection.md) | move |
| [docs/handoffs/perception/camera-shadow-family-optimization.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-shadow-family-optimization.md) | [docs/records/camera/camera-shadow-family-optimization.md](../camera/camera-shadow-family-optimization.md) | move |
| [docs/handoffs/perception/camera-shared-input-design.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-shared-input-design.md) | [docs/records/camera/camera-shared-input-design.md](../camera/camera-shared-input-design.md) | move |
| [docs/handoffs/perception/camera-shared960-validation.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-shared960-validation.md) | [docs/records/camera/camera-shared960-validation.md](../camera/camera-shared960-validation.md) | move |
| [docs/handoffs/perception/camera-three-view-coco-runtime.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-three-view-coco-runtime.md) | [docs/records/camera/camera-three-view-coco-runtime.md](../camera/camera-three-view-coco-runtime.md) | move |
| [docs/handoffs/perception/camera-training-data-audit.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-training-data-audit.md) | [docs/operations/data-preparation.md](../../operations/data-preparation.md) | move |
| [docs/handoffs/perception/camera-twinlite-tensorrt.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-twinlite-tensorrt.md) | [docs/records/camera/camera-twinlite-tensorrt.md](../camera/camera-twinlite-tensorrt.md) | move |
| [docs/handoffs/perception/camera-visualization-validation.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-visualization-validation.md) | [docs/records/camera/camera-visualization-validation.md](../camera/camera-visualization-validation.md) | move |
| [docs/handoffs/perception/camera-yoloe-l-first-run.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-yoloe-l-first-run.md) | [docs/records/camera/camera-yoloe-l-first-run.md](../camera/camera-yoloe-l-first-run.md) | move |
| [docs/handoffs/perception/camera-yoloe-l-prompt-cleanup.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-yoloe-l-prompt-cleanup.md) | [docs/records/camera/camera-yoloe-l-prompt-cleanup.md](../camera/camera-yoloe-l-prompt-cleanup.md) | move |
| [docs/handoffs/perception/camera-yoloe-obstacle-selection.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/camera-yoloe-obstacle-selection.md) | [docs/records/camera/camera-yoloe-obstacle-selection.md](../camera/camera-yoloe-obstacle-selection.md) | move |
| [docs/handoffs/perception/lidar-camera-frame-boundaries.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/lidar-camera-frame-boundaries.md) | [docs/records/lidar/lidar-camera-frame-boundaries.md](../lidar/lidar-camera-frame-boundaries.md) | move |
| [docs/handoffs/perception/lidar-continuous-regression.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/lidar-continuous-regression.md) | [docs/records/lidar/lidar-continuous-regression.md](../lidar/lidar-continuous-regression.md) | move |
| [docs/handoffs/perception/shadow-camera.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/perception/shadow-camera.md) | [docs/handoffs/camera-to-planning.md](../../handoffs/camera-to-planning.md) | move |
| [docs/handoffs/planning/behavior-legacy.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/handoffs/planning/behavior-legacy.md) | [docs/records/archive/planning-behavior-legacy.md](../archive/planning-behavior-legacy.md) | move |
| [docs/network-udp-ros.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/network-udp-ros.md) | [docs/operations/README.md](../../operations/README.md) | remove_redirect |
| [docs/onboarding/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/onboarding/README.md) | [docs/onboarding/README.md](../../onboarding/README.md) | keep |
| [docs/onboarding/ownership.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/onboarding/ownership.md) | [docs/onboarding/ownership.md](../../onboarding/ownership.md) | keep |
| [docs/operations/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/operations/README.md) | [docs/operations/README.md](../../operations/README.md) | keep |
| [docs/operations/planning-control.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/operations/planning-control.md) | [docs/operations/planning-control.md](../../operations/planning-control.md) | keep |
| [docs/operations/rosbag-evaluation.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/operations/rosbag-evaluation.md) | [docs/operations/rosbag-evaluation.md](../../operations/rosbag-evaluation.md) | keep |
| [docs/packages/asmc_bridge/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/packages/asmc_bridge/README.md) | [src/asmc_bridge/docs/integration.md](../../../src/asmc_bridge/docs/integration.md) | move |
| [docs/packages/asmc_lidar/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/packages/asmc_lidar/README.md) | [docs/handoffs/lidar-migration.md](../../handoffs/lidar-migration.md) | move |
| [docs/packages/asmc_lidar/b1a-improvement-tasks.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/packages/asmc_lidar/b1a-improvement-tasks.md) | [src/asmc_lidar/docs/migration-tasks.md](../../../src/asmc_lidar/docs/migration-tasks.md) | move |
| [docs/packages/asmc_lidar/camera-handoff.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/packages/asmc_lidar/camera-handoff.md) | [docs/handoffs/camera-to-lidar.md](../../handoffs/camera-to-lidar.md) | move |
| [docs/packages/asmc_perception/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/packages/asmc_perception/README.md) | [src/asmc_camera/docs/integration.md](../../../src/asmc_camera/docs/integration.md) | move |
| [docs/packages/asmc_perception/bev-lane-interface-plan.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/packages/asmc_perception/bev-lane-interface-plan.md) | [src/asmc_camera/docs/bev-lane-design.md](../../../src/asmc_camera/docs/bev-lane-design.md) | move |
| [docs/packages/asmc_perception/roadmap.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/packages/asmc_perception/roadmap.md) | [docs/architecture/perception-roadmap.md](../../architecture/perception-roadmap.md) | move |
| [docs/packages/planning-control/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/packages/planning-control/README.md) | [docs/architecture/planning-control.md](../../architecture/planning-control.md) | move |
| [docs/packages/planning-control/refactoring.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/packages/planning-control/refactoring.md) | [docs/architecture/planning-control-refactoring.md](../../architecture/planning-control-refactoring.md) | move |
| [docs/packages/planning-control/roundabout-merge-implementation.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/packages/planning-control/roundabout-merge-implementation.md) | [src/asmc_planner/docs/roundabout-merge.md](../../../src/asmc_planner/docs/roundabout-merge.md) | move |
| [docs/perception-dataset-labeling-split.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/perception-dataset-labeling-split.md) | [docs/contracts/dataset-labeling-split.md](../../contracts/dataset-labeling-split.md) | remove_redirect |
| [docs/perception-map-fusion-roadmap.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/perception-map-fusion-roadmap.md) | [docs/architecture/perception-roadmap.md](../../architecture/perception-roadmap.md) | remove_redirect |
| [docs/planning-control-udp.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/planning-control-udp.md) | [docs/operations/planning-control.md](../../operations/planning-control.md) | remove_redirect |
| [docs/records/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/records/README.md) | [docs/records/README.md](../README.md) | keep |
| [docs/records/baselines/bridge-p3a-v1.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/records/baselines/bridge-p3a-v1.md) | [docs/records/bridge/bridge-p3a-v1.md](../bridge/bridge-p3a-v1.md) | move |
| [docs/records/baselines/bridge-p3b-v1.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/records/baselines/bridge-p3b-v1.md) | [docs/records/bridge/bridge-p3b-v1.md](../bridge/bridge-p3b-v1.md) | move |
| [docs/records/baselines/bridge-p3c-v1.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/records/baselines/bridge-p3c-v1.md) | [docs/records/bridge/bridge-p3c-v1.md](../bridge/bridge-p3c-v1.md) | move |
| [docs/records/baselines/bridge-p3d-v1.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/records/baselines/bridge-p3d-v1.md) | [docs/records/bridge/bridge-p3d-v1.md](../bridge/bridge-p3d-v1.md) | move |
| [docs/records/baselines/bridge-p3e-local-live-v1.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/records/baselines/bridge-p3e-local-live-v1.md) | [docs/records/bridge/bridge-p3e-local-live-v1.md](../bridge/bridge-p3e-local-live-v1.md) | move |
| [docs/records/baselines/bridge-p3f-consolidated-v1.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/records/baselines/bridge-p3f-consolidated-v1.md) | [docs/records/bridge/bridge-p3f-consolidated-v1.md](../bridge/bridge-p3f-consolidated-v1.md) | move |
| [docs/records/baselines/bridge-p3g-local-closeout-v1.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/records/baselines/bridge-p3g-local-closeout-v1.md) | [docs/records/bridge/bridge-p3g-local-closeout-v1.md](../bridge/bridge-p3g-local-closeout-v1.md) | move |
| [docs/records/baselines/kcity2025-compact-drivable-v1.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/records/baselines/kcity2025-compact-drivable-v1.md) | [docs/records/hd-map/kcity2025-compact-drivable-v1.md](../hd-map/kcity2025-compact-drivable-v1.md) | move |
| [docs/records/baselines/kcity2025-drivable-input-v1.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/records/baselines/kcity2025-drivable-input-v1.md) | [docs/records/hd-map/kcity2025-drivable-input-v1.md](../hd-map/kcity2025-drivable-input-v1.md) | move |
| [docs/records/baselines/perception-p4a-runtime-v1.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/records/baselines/perception-p4a-runtime-v1.md) | [docs/records/camera/perception-p4a-runtime-v1.md](../camera/perception-p4a-runtime-v1.md) | move |
| [docs/records/baselines/perception-p5a-topic-audit-v1.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/records/baselines/perception-p5a-topic-audit-v1.md) | [docs/records/camera/perception-p5a-topic-audit-v1.md](../camera/perception-p5a-topic-audit-v1.md) | move |
| [docs/records/baselines/perception-p5b-rqt-v1.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/records/baselines/perception-p5b-rqt-v1.md) | [docs/records/camera/perception-p5b-rqt-v1.md](../camera/perception-p5b-rqt-v1.md) | move |
| [docs/records/baselines/perception-p5c-runtime-acceptance-v1.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/records/baselines/perception-p5c-runtime-acceptance-v1.md) | [docs/records/camera/perception-p5c-runtime-acceptance-v1.md](../camera/perception-p5c-runtime-acceptance-v1.md) | move |
| [docs/records/baselines/perception-p5de-closeout-v1.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/records/baselines/perception-p5de-closeout-v1.md) | [docs/records/camera/perception-p5de-closeout-v1.md](../camera/perception-p5de-closeout-v1.md) | move |
| [docs/records/baselines/runtime-v1.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/records/baselines/runtime-v1.md) | [docs/records/repository/runtime-v1.md](runtime-v1.md) | move |
| [docs/records/hd-map/kcity2025-route-curation-20260917.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/records/hd-map/kcity2025-route-curation-20260917.md) | [docs/records/hd-map/kcity2025-route-curation-20260917.md](../hd-map/kcity2025-route-curation-20260917.md) | keep |
| [docs/records/perception/twinlitenet-v4-evaluation.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/records/perception/twinlitenet-v4-evaluation.md) | [docs/records/camera/twinlitenet-v4-evaluation.md](../camera/twinlitenet-v4-evaluation.md) | move |
| [docs/records/simulator/morai-msgs-versions.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/records/simulator/morai-msgs-versions.md) | [docs/records/bridge/morai-msgs-versions.md](../bridge/morai-msgs-versions.md) | move |
| [docs/records/simulator/runtime-transport-2026-09.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/records/simulator/runtime-transport-2026-09.md) | [docs/records/bridge/runtime-transport-2026-09.md](../bridge/runtime-transport-2026-09.md) | move |
| [docs/repository-consolidation-roadmap.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/repository-consolidation-roadmap.md) | [docs/architecture/consolidation-roadmap.md](../../architecture/consolidation-roadmap.md) | remove_redirect |
| [docs/runtime-commands.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/runtime-commands.md) | [docs/operations/commands.md](../../operations/commands.md) | move |
| [docs/sim-verification-checklist.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/sim-verification-checklist.md) | [docs/operations/README.md](../../operations/README.md) | remove_redirect |
| [docs/structure/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/structure/README.md) | [docs/records/repository/documentation-index.md](documentation-index.md) | move |
| [docs/structure/current-inventory.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/structure/current-inventory.md) | [docs/records/repository/documentation-current-inventory.md](documentation-current-inventory.md) | move |
| [docs/structure/d1-path-migration-report.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/structure/d1-path-migration-report.md) | [docs/records/repository/documentation-d1-path-migration-report.md](documentation-d1-path-migration-report.md) | move |
| [docs/structure/d2-bridge-report.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/structure/d2-bridge-report.md) | [docs/records/repository/documentation-d2-bridge-report.md](documentation-d2-bridge-report.md) | move |
| [docs/structure/d2-competition-operations-report.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/structure/d2-competition-operations-report.md) | [docs/records/repository/documentation-d2-competition-operations-report.md](documentation-d2-competition-operations-report.md) | move |
| [docs/structure/d2-content-parity-audit.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/structure/d2-content-parity-audit.md) | [docs/records/repository/documentation-d2-content-parity-audit.md](documentation-d2-content-parity-audit.md) | move |
| [docs/structure/d2-contracts-report.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/structure/d2-contracts-report.md) | [docs/records/repository/documentation-d2-contracts-report.md](documentation-d2-contracts-report.md) | move |
| [docs/structure/d2-hd-map-msgs-report.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/structure/d2-hd-map-msgs-report.md) | [docs/records/repository/documentation-d2-hd-map-msgs-report.md](documentation-d2-hd-map-msgs-report.md) | move |
| [docs/structure/d2-onboarding-governance-report.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/structure/d2-onboarding-governance-report.md) | [docs/records/repository/documentation-d2-onboarding-governance-report.md](documentation-d2-onboarding-governance-report.md) | move |
| [docs/structure/d2-perception-report.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/structure/d2-perception-report.md) | [docs/records/repository/documentation-d2-perception-report.md](documentation-d2-perception-report.md) | move |
| [docs/structure/d2-planning-handoffs-report.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/structure/d2-planning-handoffs-report.md) | [docs/records/repository/documentation-d2-planning-handoffs-report.md](documentation-d2-planning-handoffs-report.md) | move |
| [docs/structure/d2-records-archive-report.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/structure/d2-records-archive-report.md) | [docs/records/repository/documentation-d2-records-archive-report.md](documentation-d2-records-archive-report.md) | move |
| [docs/structure/d3-retention-compatibility-report.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/structure/d3-retention-compatibility-report.md) | [docs/records/repository/documentation-d3-retention-compatibility-report.md](documentation-d3-retention-compatibility-report.md) | move |
| [docs/structure/d4-package-readme-report.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/structure/d4-package-readme-report.md) | [docs/records/repository/documentation-d4-package-readme-report.md](documentation-d4-package-readme-report.md) | move |
| [docs/structure/migration-gates.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/structure/migration-gates.md) | [docs/records/repository/documentation-migration-gates.md](documentation-migration-gates.md) | move |
| [docs/team-dev-guide.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/docs/team-dev-guide.md) | [docs/onboarding/ownership.md](../../onboarding/ownership.md) | remove_redirect |
| [src/asmc_camera/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/src/asmc_camera/README.md) | [src/asmc_camera/README.md](../../../src/asmc_camera/README.md) | update_references |
| [src/asmc_camera/config/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/src/asmc_camera/config/README.md) | [src/asmc_camera/config/README.md](../../../src/asmc_camera/config/README.md) | update_references |
| [src/asmc_camera/docs/bev-development-history.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/src/asmc_camera/docs/bev-development-history.md) | [docs/records/camera/bev-development-history.md](../camera/bev-development-history.md) | move |
| [src/asmc_camera/docs/camera-semantic-history.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/src/asmc_camera/docs/camera-semantic-history.md) | [docs/records/camera/camera-semantic-history.md](../camera/camera-semantic-history.md) | move |
| [src/asmc_camera/docs/traffic-light-history.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/src/asmc_camera/docs/traffic-light-history.md) | [docs/records/camera/traffic-light-history.md](../camera/traffic-light-history.md) | move |
| [src/asmc_camera/models/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/src/asmc_camera/models/README.md) | [src/asmc_camera/models/README.md](../../../src/asmc_camera/models/README.md) | update_references |
| [src/asmc_camera/training/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/src/asmc_camera/training/README.md) | [src/asmc_camera/training/README.md](../../../src/asmc_camera/training/README.md) | update_references |
| [src/asmc_controller/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/src/asmc_controller/README.md) | [src/asmc_controller/README.md](../../../src/asmc_controller/README.md) | update_references |
| [src/asmc_hd_map/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/src/asmc_hd_map/README.md) | [src/asmc_hd_map/README.md](../../../src/asmc_hd_map/README.md) | update_references |
| [src/asmc_hd_map/development/won_tae_link_node/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/src/asmc_hd_map/development/won_tae_link_node/README.md) | [src/asmc_hd_map/development/won_tae_link_node/README.md](../../../src/asmc_hd_map/development/won_tae_link_node/README.md) | update_references |
| [src/asmc_lidar/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/src/asmc_lidar/README.md) | [src/asmc_lidar/README.md](../../../src/asmc_lidar/README.md) | update_references |
| [src/asmc_msgs/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/src/asmc_msgs/README.md) | [src/asmc_msgs/README.md](../../../src/asmc_msgs/README.md) | update_references |
| [src/data_collection/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/src/data_collection/README.md) | [src/data_collection/README.md](../../../src/data_collection/README.md) | update_references |
| [src/integration/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/src/integration/README.md) | [src/integration/README.md](../../../src/integration/README.md) | update_references |
| [src/interfaces/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/src/interfaces/README.md) | [src/interfaces/README.md](../../../src/interfaces/README.md) | update_references |
| [src/jamming_lane_boundary_adapter/CAMERA_BBOX_LANE.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/src/jamming_lane_boundary_adapter/CAMERA_BBOX_LANE.md) | [src/jamming_lane_boundary_adapter/docs/bbox-lane.md](../../../src/jamming_lane_boundary_adapter/docs/bbox-lane.md) | move |
| [src/jamming_lane_boundary_adapter/CAMERA_BBOX_LANE_CHANGE.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/src/jamming_lane_boundary_adapter/CAMERA_BBOX_LANE_CHANGE.md) | [src/jamming_lane_boundary_adapter/docs/bbox-lane-change.md](../../../src/jamming_lane_boundary_adapter/docs/bbox-lane-change.md) | move |
| [src/jamming_lane_boundary_adapter/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/src/jamming_lane_boundary_adapter/README.md) | [src/jamming_lane_boundary_adapter/README.md](../../../src/jamming_lane_boundary_adapter/README.md) | update_references |
| [src/jamming_lane_boundary_adapter/docs/camera-lane-cleanup-audit.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/src/jamming_lane_boundary_adapter/docs/camera-lane-cleanup-audit.md) | [docs/records/repository/jamming-camera-lane-cleanup-audit.md](jamming-camera-lane-cleanup-audit.md) | move |
| [src/jamming_lane_boundary_adapter/docs/local-planning-control.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/src/jamming_lane_boundary_adapter/docs/local-planning-control.md) | [src/jamming_lane_boundary_adapter/docs/local-planning-control.md](../../../src/jamming_lane_boundary_adapter/docs/local-planning-control.md) | update_references |
| [src/lidar+sensorfusion/RUN_SENSOR_FUSION.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/src/lidar+sensorfusion/RUN_SENSOR_FUSION.md) | [src/lidar+sensorfusion/RUN_SENSOR_FUSION.md](../../../src/lidar+sensorfusion/RUN_SENSOR_FUSION.md) | update_references |
| [tools/perception_eval/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/tools/perception_eval/README.md) | [tools/perception_eval/README.md](../../../tools/perception_eval/README.md) | update_references |
| [tools/scen_runner/README.md](https://github.com/INHAautonav/2026-ASMC/blob/d36f29b6e5dad468c2668ff2e3823e5de8de49b5/tools/scen_runner/README.md) | [tools/scen_runner/README.md](../../../tools/scen_runner/README.md) | update_references |

## 다음 내용 검토

이동 목록의 followup_content_review 항목은 후속 검토 대상이다. 긴 문서에 현재 안내와 과거 이력이
혼합된 경우도 이번에는 분해하거나 수락 상태를 다시 판정하지 않았다.
카메라·Bridge·Planning 가이드, 데이터 준비 안내, 로드맵과 기존 목차부터 실제 코드와 대조한다.

대용량 자료·모델·ROS 인터페이스·컨테이너 마운트·재현 checkout은 이 이전의 대상이 아니다.

## 상위 workspace 참고 문서

개인 참고 문서 34개의 링크 79개를 현재 작업 checkout의 실제 경로 또는 확인된 Git commit으로 연결했다.
이 문서들은 팀 저장소에 복사하지 않았다. 본문은 링크 목적지를 제외하고 보존했다.
main/repro checkout과 컨테이너 마운트는 변경하지 않았다. PR 병합 전 검토 가능한 로컬 링크는
2026-ASMC-work를 가리킨다. 원본 경로·해시와 갱신 결과는 로컬 data/runtime/docs-restructure-v2에 둔다.

기존 개인 논문 메모에 남은 아래 3개 참조는 현재 파일과 검색한 Git 이력에 없어 후속 검토로 남겼다.
이번에 이동한 파일이나 제거한 호환 안내를 가리키는 링크가 아니며, 연구 결과를 다른 문서로 대신하지 않았다.

- `docs/papers/lidar-reference-readiness-20260926.md` → `docs/packages/asmc_perception/lidar-l1b1-validation-20260927.md`
- `docs/papers/lidar-reference-readiness-20260926.md` → `tools/lidar/clustering_compare/README.md`
- `docs/papers/lidar-reference-readiness-20260926.md` → `docs/packages/asmc_perception/lidar-l1b5a-clustering-validation-20260929.md`

## 링크 검사 보완

기존 파일 검사에 image/reference/HTML 링크와 GitHub-style anchor 검사를 추가했다.
과거 명령이 들어 있는 fenced/inline code는 링크 이전 대상으로 해석하지 않는다.
기존에 깨져 있던 anchor 4개는 존재하는 절 또는 문서 시작점으로만 수정했다.
README·handoff·records 목차의 추가 탐색 블록만 일회성 본문 비교에서 허용한다.

검사기 회귀 7개는 원본 내용 손실, 동결 파일 변경, 구 stub 재등장, 누락 anchor/image,
과거 명령 보존과 현행 문서의 후속 정상 수정을 확인한다.

검사 요약은 [v2 결과](../../../config/docs/documentation_relocation_v2_result.json)에 있다.
followup_content_review는 현행 안내의 검토·분리 대상이며 과거 실측 결과를 다시 쓰라는 지시가 아니다.
