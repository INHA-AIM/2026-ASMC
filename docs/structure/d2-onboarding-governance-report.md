# D2 온보딩·협업 문서 통합 기록

> **상태:** 2026-09-22 통합 완료
>
> **범위:** `docs/onboarding/`, `docs/governance/` 및 직접 연결된 진입 문서

## 결과

| 이전 문서 | 새 정본 | 처리 |
|---|---|---|
| `onboarding/getting-started.md` | `onboarding/README.md` | clone, Docker, 첫 빌드 보존 |
| `onboarding/development-environment.md` | `onboarding/README.md` | OS·언어·image·데이터 기준 통합 |
| `onboarding/team-dev-guide.md` | `onboarding/ownership.md` | 팀·경로·검증 책임만 현재 상태로 정리 |
| `governance/collaboration.md` | `governance/README.md` | 현재 정책과 review gate 통합 |
| `governance/git-workflow.md` | `governance/README.md` | 중복 명령을 한 workflow로 통합 |

기존 5개 정본 1,289줄은 새 정본 3개 330줄로 줄였다. 기존 외부 링크를 위해
`docs/getting-started.md`, `docs/team-dev-guide.md`, `docs/collaboration.md` 호환 안내는 유지한다.

## 의미 보존과 현재화

- 팀원 6명의 GitHub 계정과 담당 영역을 보존했다.
- WSL/native Linux, Docker host network, Python 3.8, CUDA image, `$ASMC_DATA`, pinned
  `morai_msgs` 기준을 보존했다.
- 상세 실행 파일 목록은 각 package README가 더 정확하므로 중복 목록 대신 정본 링크로
  바꿨다.
- 존재하지 않는 `src/planner/` 표기는 현재 `src/frenet_planner/`로 교정했다.
- Phase 0 개인 레포와 Wave 1/2 설명은 현재 지침에서 제외하고 archive/Git 이력으로 연결했다.
- 최근 main 이력이 PR merge 중심이고 이후 통합 단계도 별도 PR을 요구하므로, 과거
  `main` 직푸시 Path A를 현재 규칙에서 제외하고 짧은 브랜치+PR로 단일화했다.
- `.github/CODEOWNERS`, PR template과 `CONTRIBUTING.md`를 같은 경로·정책으로 맞췄다.
- 네트워크 숫자와 package 통합 목표는 온보딩 문서에 복제하지 않고 각각 operations와
  architecture 정본으로 연결했다.
- IDE 개인 선호와 날짜가 지난 “8월 이전 주 1회” 문구는 팀 계약이 아니므로 제거했다.

원문의 상세 문장은 commit `b84335b` 이전 이력에서 확인할 수 있다. 삭제한 현재 문서의
고유 정책은 위 새 정본 또는 명시된 archive로 대응되며, 내용 없이 버린 계약은 없다.

## 검증 결과

- 저장소 내부 Markdown target 428개가 모두 존재한다.
- 삭제한 5개 정본 경로의 실행 링크가 남지 않았다.
- CODEOWNERS의 모든 경로가 현재 working tree에 존재한다.
- `git diff --check`와 `bash -n scripts/build_ws.sh`를 통과했다.
- 비어 있던 `docs/asmc_msg/`, `docs/baselines/`는 참조를 실제 정본으로 교정한 뒤 제거했다.
- 비어 있던 과거 perception output 디렉터리
  `artifacts/perception_eval/analysis/shadow_lane_map_alignment_holdout_old_20260913/`도 제거했다.
  build tree의 빈 디렉터리는 생성물이므로 정리 대상에서 제외했다.

다음 D2 묶음은 `competition/`과 `operations/`다. 포트·규정 수치처럼 동작에 영향을 주는
내용은 현장 자료 및 현재 설정과 다시 대조한 뒤 통합한다.
