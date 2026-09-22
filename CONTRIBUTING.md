# 기여 가이드 (CONTRIBUTING)

## 빠른 링크

- [docs/governance/collaboration.md](docs/governance/collaboration.md) ← **★ Trunk-based · 모듈 소유 · 언제 PR인가 (필독)**
- [docs/onboarding/getting-started.md](docs/onboarding/getting-started.md)
- [docs/onboarding/team-dev-guide.md](docs/onboarding/team-dev-guide.md) ← **팀원별 개발·검증**
- [docs/governance/git-workflow.md](docs/governance/git-workflow.md)
- [docs/architecture/repository-layout.md](docs/architecture/repository-layout.md)
- [docs/operations/sim-verification-checklist.md](docs/operations/sim-verification-checklist.md)
- [docs/onboarding/development-environment.md](docs/onboarding/development-environment.md)
- [docs/contracts/simulator/morai-msgs-beta-policy.md](docs/contracts/simulator/morai-msgs-beta-policy.md)

## 필수 규칙

1. **기본은 trunk (Phase 1).** Phase 0에는 개인 레포 병행 — [collaboration.md §0.5](docs/governance/collaboration.md).
   소유 모듈 + **빌드 통과** 시에만 `main` 푸시. WIP·미완성은 Draft PR 또는 개인 레포 (§2.5).
2. **담당 파트 외 수정 금지** (합의·공동 PR 예외) — [team-dev-guide.md](docs/onboarding/team-dev-guide.md)
3. **대용량·체크포인트 커밋 금지.**
4. **privileged를 배포 경로에 넣지 않는다.**
5. **Python 3.8 / ROS Noetic** 기준.
6. **morai_msgs는 `beta_drive`만.**
7. Phase 1에서 새 기능은 ASMC; Phase 0 기능 완성은 개인 레포 OK.
8. **`main`을 빨간 채로 방치하지 않는다.**

## Path A (`main` 직푸시) 전

- [ ] 빌드 성공 (WIP 아님)
- [ ] ownership 경로만
- [ ] interfaces/docker/UDP 경계 아님
- [ ] `git push origin main`

## Path B (PR) 전

- [ ] Path B 사유에 해당 ([collaboration.md §3.2](docs/governance/collaboration.md#32-path-b--브랜치--pr-필수-게이트))
- [ ] `feature/<이름>-…` 수명 짧게 (목표 1~2일 merge)
- [ ] 빌드·(가능 시) 시뮬
- [ ] [.github/pull_request_template.md](.github/pull_request_template.md) 작성
- [ ] CODEOWNERS 리뷰 후 merge · 브랜치 삭제

## 리뷰어

| 파트 | 기본 리뷰어 | GitHub |
|------|-------------|--------|
| planning (`behavior_planner`, `mpc_controller`, `integration_launch`) | 정윤태 · 양서준 | @yuntae12-sudo · @yangseojun |
| Frenet (`planner`) | 이강태 · 정윤태 | @kante2 · @yuntae12-sudo |
| perception (`morai_3d_detection`) | 손재호 | @sonshiny |
| scenario / gRPC (`tools/`) | 장원태 | @wkddnjsxo |
| learning·bridge·docs·docker·interfaces | 안승현 | @ahnsh03 |

전체: [docs/governance/collaboration.md §0](docs/governance/collaboration.md#0-팀-github-계정)

## 통합 문의

공통 인터페이스(τ·tracks·TL)·UDP 경계·레거시 이식은 **통합 리드(안승현)** 에게 먼저 공유하세요.
