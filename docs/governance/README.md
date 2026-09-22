# 협업과 Git 규칙

> **문서 역할:** Canonical — 브랜치, PR, review와 main 안정성 규칙
>
> **현재 정책:** 짧은 브랜치 + PR. 과거 Phase 0와 `main` 직푸시 Path A는 현재 정책이 아니다.

담당 경로는 [팀 ownership](../onboarding/ownership.md), 자동 리뷰 규칙은
[CODEOWNERS](../../.github/CODEOWNERS), PR 입력 항목은
[pull request template](../../.github/pull_request_template.md)을 따른다.

## 1. 원칙

1. `main`은 빌드·검증된 통합 기준선으로 유지한다.
2. 모든 변경은 최신 `main`에서 만든 짧은 브랜치와 PR로 합류한다.
3. WIP 공유는 Draft PR로 하고, 실행 불가능한 상태를 Ready로 전환하지 않는다.
4. 한 PR은 한 목적만 다루며 rename과 동작 변경을 분리한다.
5. 공용 계약·UDP·Docker·CI·여러 파트 변경은 관련 owner의 리뷰를 받는다.
6. bag, dataset, checkpoint, 비밀값과 호스트 절대경로를 커밋하지 않는다.

## 2. Pull request workflow

```bash
git switch main
git pull --ff-only origin main
git submodule update --init --recursive
git switch -c feature/<이름>-<작업>

# 구현과 검증
git status --short
git add <이번-작업-경로>
git commit -m "feat(scope): concise summary"
git push -u origin HEAD
gh pr create --fill
```

리뷰 중 main이 앞서면:

```bash
git fetch origin
git rebase origin/main
git push --force-with-lease
```

merge 후:

```bash
git switch main
git pull --ff-only origin main
git branch -d <작업-브랜치>
```

충돌 해결에 `git reset --hard`나 다른 사람의 변경을 지우는 명령을 쓰지 않는다. 해결이
불확실하면 해당 파일 owner와 함께 rebase한다.

## 3. PR 범위와 리뷰

| 변경 | 요구 |
|---|---|
| 소유 패키지 내부 소규모 수정 | owner 리뷰 + 관련 test |
| message/topic/좌표계 | producer·consumer owner 공동 리뷰 |
| UDP 포트·packet·sensor profile | bridge·인지·판제 공동 리뷰 |
| Docker, CI, submodule | 통합 lead 리뷰와 clean build |
| 여러 패키지 이동·삭제 | parity, rollback, 후속 삭제 gate |
| 문서 정본·규정·계약 | 내용 owner와 통합 lead 리뷰 |

리뷰가 필요하지만 미완성인 경우:

```bash
gh pr create --draft --fill
```

PR 본문에는 목적, 영향받는 producer/consumer, 실제 실행한 검증, 미검증 항목과 rollback을
적는다. “테스트 예정”만 적은 PR은 merge하지 않는다.

## 4. 브랜치와 커밋

| 종류 | 예 |
|---|---|
| 기능 | `feature/jaeho-lidar-tracking` |
| 수정 | `fix/yuntae-stop-continuity` |
| 문서 | `docs/documentation-consolidation` |
| 통합 | `integrate/asmc-bridge` |

브랜치는 수시간에서 2~3일을 목표로 하며, 길어지면 독립적으로 검증 가능한 PR로 나눈다.

```text
feat(scope): 기능
fix(scope): 버그 수정
refactor(scope): 동작을 유지한 구조 변경
docs: 문서
test(scope): 검증
chore(scope): 빌드·설정
```

한 커밋에는 한 의도를 담는다. 자동 생성물은 생성 도구와 함께 갱신하고, 의미 없는 WIP
커밋은 Ready 전 정리한다.

## 5. 최소 검증

변경 범위에 맞는 항목만 실행하되 실행하지 못한 항목은 PR에 이유를 남긴다.

```bash
# ROS/catkin
docker exec -it asmc-ros-noetic bash -lc 'cd /root/ws && ./scripts/build_ws.sh'
```

- 문서 이동: 로컬 Markdown 링크, `git diff --check`와
  `python3 tools/audit/check_documentation_migration.py`
- HD Map: deterministic builder/test와 topic/RViz smoke
- perception: 동일 bag replay, 출력 rate·latency·message contract
- bridge: packet fixture, topic type/rate, 기존 profile 회귀
- 공용 message: MD5/serialization과 모든 consumer build

통합 검증은 [시뮬 체크리스트](../operations/README.md)를 따른다.

## 6. main 장애와 긴급 수정

1. 실패를 팀 채널과 PR/commit 링크로 알린다.
2. 새 `fix/...` 브랜치에서 최소 수정 또는 `git revert` 커밋을 만든다.
3. 영향 owner가 빠르게 리뷰한 PR로 복구한다.
4. 원인과 누락된 회귀 test를 후속 기록에 남긴다.

긴급 상황도 공유 이력을 지우는 force push로 해결하지 않는다.

## 7. 과거 정책의 위치

Phase 0 개인 레포, Wave 1/2 이식과 `main` 직푸시 Path A 설명은 현재 실행 규칙이 아니다.
역사적 판단은 [Wave 1/2 기록](../archive/integration/wave1-wave2-plan.md)과 Git 이력에 남긴다.
현재 구현 순서는 [통합 로드맵](../architecture/consolidation-roadmap.md)을 따른다.
