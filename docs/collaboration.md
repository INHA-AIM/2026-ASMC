# 협업 가이드 (브랜치 · PR · 충돌 방지)

> **목표**: PC(Docker)에서 개발 → GitHub PR → MORAI 시뮬 검증  
> **필독**: 코드 수정 전 **§1 Git 규약**을 따릅니다.  
> 상세 경로·검증: [team-dev-guide.md](./team-dev-guide.md)

---

## 0. 팀 GitHub 계정

| GitHub | 이름 | 담당 |
|--------|------|------|
| [@ahnsh03](https://github.com/ahnsh03) | 안승현 | 통합 리드 · LBC · bridge · Docker · docs |
| [@yuntae12-sudo](https://github.com/yuntae12-sudo) | 정윤태 | planning lead · behavior · MPC · launch |
| [@yangseojun](https://github.com/yangseojun) | 양서준 | MPC · behavior 공조 |
| [@kante2](https://github.com/kante2) | 이강태 | Frenet planner |
| [@wkddnjsxo](https://github.com/wkddnjsxo) | 장원태 | scenario runner · gRPC |
| [@sonshiny](https://github.com/sonshiny) | 손재호 | 3D perception |

> 레거시 `kante2/aim_ws` 브랜치명(`jang`, `ochang`, `va_seunghyun`)은 **과거 작업 분기**일 뿐 GitHub 계정과 1:1이 아닙니다.  
> 예: `jang` 브랜치 = 장원태(`wkddnjsxo`) 시나리오 작업, `ochang` = 이강태(`kante2`) 구버전(미이식).

---

## 1. 팀 Git 규약 (필수)

### 1.1 기본 원칙

| 규칙 | 설명 |
|------|------|
| **`main` 직접 push 금지** | PR merge로만 반영 |
| **브랜치에서만 개발** | `feature/<이름>-<기능>` 생성 후 작업 |
| **작은 PR** | 1 PR = 1 기능 또는 1 버그 (담당 파트 단위) |
| **ASMC에서만 개발** | 개인 `aim_ws`/`mpc_ws`에 새 기능 누적 금지 |
| **merge 후 정리** | feature 브랜치 삭제, `main` pull |

```
main (안정)
  └── feature/<이름>-<기능>
         ├── 담당 src/·tools/ 수정
         ├── commit · push
         ├── Pull Request (+ CI)
         ├── (파트 리드) 리뷰
         └── merge → main
```

> **요약**: `브랜치 생성 → 작업 → commit → push → PR → merge`  
> **`main`에 직접 push하지 않습니다.**

### 1.2 표준 개발 절차

```bash
cd <clone한-2026-ASMC-경로>   # 선택: export ASMC=... 후 cd "$ASMC"

git checkout main && git pull
git submodule update --init --recursive
git checkout -b feature/seunghyun-lbc-train

# Docker (권장)
./scripts/docker_ros_up.sh up
docker exec -it asmc-ros-noetic bash -c './scripts/build_ws.sh'

# 담당 경로만 수정 후
git add src/learning_by_cheating/
git commit -m "feat(lbc): improve BEV vehicle channel sync"
git push -u origin feature/seunghyun-lbc-train

gh pr create --title "feat(lbc): vehicle channel sync" --body "..."
```

merge 권한: **파트 리드** 또는 **통합 리드(안승현)**.  
PR 작성 시 [.github/pull_request_template.md](../.github/pull_request_template.md) 체크리스트를 채웁니다.

### 1.3 브랜치 이름

| 패턴 | 예 | 사용 |
|------|-----|------|
| `feature/<이름>-<기능>` | `feature/yuntae-mpc-udp` | **일상 개발 (기본)** |
| `fix/<이름>-<이슈>` | `fix/jang-scenario-timeout` | 버그 수정 |
| `docs/<이름>-<주제>` | `docs/seunghyun-git-workflow` | 문서만 |
| `integrate/<모듈>` | `integrate/interfaces` | 통합 리드 (대규모 이식) |

**팀원별 예**

| 담당 | 브랜치 예 |
|------|-----------|
| 정윤태 | `feature/yuntae-behavior-fsm` |
| 양서준 | `feature/seojun-mpc-tune` |
| 이강태 | `feature/kangtae-frenet-cost` |
| 안승현 | `feature/seunghyun-lbc-train` |
| 장원태 | `feature/jang-scenario-dataset` |
| 손재호 | `feature/jaeho-3d-detect` |

**비권장:** `va_seunghyun`, `jang` 등 개인 단일 브랜치에 모든 작업 누적.

### 1.4 커밋 메시지

```
feat(scope): 한 줄 요약
fix(scope): 버그 수정
docs: 문서만
chore(docker): Docker 설정
refactor(planner): 동작 변경 없는 리팩터
```

`scope` 예: `mpc`, `planner`, `behavior`, `lbc`, `perception`, `scenario`, `docker`, `integration`

### 1.5 PR 타이밍

| 상황 | 행동 |
|------|------|
| `./scripts/build_ws.sh` 통과하는 최소 단위 | PR 생성 |
| 방향 피드백 필요·미완 | **Draft PR** |
| 리뷰·merge 가능 | Ready for review |

### 1.6 도구 역할

| 도구 | 역할 |
|------|------|
| **Cursor / 터미널** | 브랜치, commit, push, `gh pr create` |
| **Docker** (`asmc-ros-noetic`) | ROS Noetic 빌드·실행 (팀 표준) |
| **GitHub 웹** | PR 리뷰·merge |
| **Sublime Merge 등** | diff·merge conflict 해결 |

---

## 2. 코드 수정 규칙 (충돌 방지)

| 규칙 | 설명 |
|------|------|
| **파트는 각자, 공통은 합의** | [team-dev-guide.md](./team-dev-guide.md) 담당 경로만 |
| **submodule 직접 커밋 금지** | `src/MORAI-ROS_morai_msgs/` — 통합 리드 |
| **공통 launch·interfaces** | `integration_launch/`, `interfaces/` — 합의 후 |

### 수정 권한 요약

| 경로 | 주 담당 |
|------|---------|
| `src/behavior_planner/`, `src/mpc_controller/`, `src/integration_launch/` | 정윤태·양서준 |
| `src/frenet_planner/` | 이강태 |
| `src/learning_by_cheating/`, `scripts/bridge*` | 안승현 |
| `src/perception/morai_3d_detection/` | 손재호 |
| `tools/aim_scenario_runner/`, `tools/grpc_inha_univ/` | 장원태 |
| `docker/`, `docs/` (규약), `interfaces/` | 통합 리드 |

전체 매트릭스: [repository-layout.md](./repository-layout.md) §3

---

## 3. 개발 환경

| 환경 | 역할 |
|------|------|
| **PC + Docker** (권장) | 일상 개발, `build_ws.sh`, PR 전 검증 |
| **Ubuntu 20.04 네이티브** | 대회 리허설 PC |
| **WSL** | Docker Desktop WSL Integration 필수 |

```bash
# PR 전 최소 검증
./scripts/docker_ros_up.sh up
docker exec -it asmc-ros-noetic bash -c './scripts/build_ws.sh'
```

MORAI 시뮬 상세: [sim-verification-checklist.md](./sim-verification-checklist.md)  
클론·셸 변수·Docker: [getting-started.md](./getting-started.md), [development-environment.md](./development-environment.md)

---

## 4. 충돌·리베이스

| 상황 | 해결 |
|------|------|
| 같은 패키지를 두 명이 수정 | **파트 격리** 준수, 합의 후 한쪽 rebase |
| `integration_launch` conflict | planning·통합 리드가 조율 |
| feature가 main보다 뒤처짐 | rebase 후 push |

```bash
git checkout feature/my-branch
git fetch origin
git rebase origin/main
# conflict → 담당 파일만 해결 → git rebase --continue
git push --force-with-lease
```

---

## 5. PR 체크리스트 (요약)

- [ ] `main`에서 feature 브랜치 생성
- [ ] **`main` 직접 push 안 함**
- [ ] 담당 `src/`·`tools/` 경로만 변경
- [ ] `./scripts/build_ws.sh` 성공
- [ ] PR template · CODEOWNERS 리뷰
- [ ] merge 후 로컬 feature 브랜치 삭제

---

## 6. CODEOWNERS · CI

- **CODEOWNERS**: [.github/CODEOWNERS](../.github/CODEOWNERS) — §0 계정 기준 자동 리뷰 요청
- **CI**: PR·`main` push 시 Docker(`ros:noetic`)에서 submodule 포함 `catkin_make` ([`.github/workflows/ci.yml`](../.github/workflows/ci.yml))

---

## 관련 문서

- [git-workflow.md](./git-workflow.md) — 브랜치·커밋 상세
- [CONTRIBUTING.md](../CONTRIBUTING.md) — 기여 규칙 요약
- [team-dev-guide.md](./team-dev-guide.md) — 팀원별 경로·검증
- [getting-started.md](./getting-started.md) — 클론·첫 빌드
