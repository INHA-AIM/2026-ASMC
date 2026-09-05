## Summary

<!-- 무엇을 변경했는지 한 줄 요약 -->

## Path (필수)

문서: [docs/collaboration.md](../docs/collaboration.md)

- [ ] **Path B** — 아래 중 하나 이상 (이 PR이 맞는 경우)
  - [ ] `interfaces/` · msg/토픽 계약
  - [ ] `integration_launch/` · 타 파트 연결
  - [ ] `docker/` · `.github/` · submodule 핀
  - [ ] UDP/시뮬 경계 · bridge 프로토콜
  - [ ] 교차 모듈 · 대형/방향 변경 · 실험
- [ ] **Path C (선택 리뷰)** — 소유 모듈이지만 리뷰를 원함
  <!-- Path A(main 직푸시)는 PR 없이 진행. 이 템플릿은 Path B/C용 -->

## Git 규약 확인

- [ ] **빌드·기본 실행이 되는 상태**만 Ready PR / `main` 머지 (WIP는 Draft — collaboration §2.5)
- [ ] `main`과 rebase/merge로 맞춤 · 브랜치 수명 짧게
- [ ] Phase 0이면 개인 레포 작업 후 이식인지 / ASMC 인프라인지 명시
- [ ] 개인 `aim_ws` 누적이 아닌 **합의된 upstream** 또는 ASMC

## 담당 파트 (해당 항목만 체크)

- [ ] planning — `behavior_planner/` (정윤태·양서준)
- [ ] planning — `mpc_controller/` (정윤태·양서준)
- [ ] planning — `planner/` (이강태)
- [ ] planning — `integration_launch/` (정윤태)
- [ ] learning — `learning_by_cheating/` (안승현)
- [ ] perception — `perception/morai_3d_detection/` (손재호)
- [ ] scenario — `tools/aim_scenario_runner/` (장원태)
- [ ] gRPC — `tools/grpc_inha_univ/` (장원태)
- [ ] 통합 — `docker/`, `interfaces/`, submodule, UDP bridge (통합 리드)

## 변경 범위 확인

- [ ] [team-dev-guide.md](../docs/team-dev-guide.md) 담당·합의 경로만 수정
- [ ] `src/MORAI-ROS_morai_msgs/` submodule을 직접 커밋하지 않음
- [ ] 호스트 절대경로(`/home/…`) 없음
- [ ] 대용량 데이터·체크포인트 없음

## 테스트

- [ ] `./scripts/build_ws.sh` (또는 해당 패키지) 성공
- [ ] morai_msgs / msg 변경 시 [morai-msgs-beta-policy.md](../docs/morai-msgs-beta-policy.md)
- [ ] (가능 시) [sim-verification-checklist.md](../docs/sim-verification-checklist.md) 해당 §

## Breaking change (해당 시)

- [ ] `src/interfaces/` 또는 토픽/msg 스키마 변경 → 팀 공지

## 스크린샷 / 로그 (선택)

<!-- rostopic, MORAI, build 로그 -->
