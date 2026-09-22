# Git 워크플로 (Trunk-based 상세)

> **문서 역할:** Command Reference — 협업 정책을 실행하는 명령 예시
> **정본:** 정책이 충돌하면 [collaboration.md](./collaboration.md)를 따른다.
> **문서 허브:** [README.md](../README.md)
> **필독 통합본:** [collaboration.md](./collaboration.md)
> PR 템플릿: [.github/pull_request_template.md](../../.github/pull_request_template.md)
> **최종 수정:** 2026-07-25

---

## 1. 기본 원칙

| 원칙 | 설명 |
|------|------|
| **기본은 `main` (Phase 1 trunk)** | 소유 모듈 + **빌드 통과** 시 자주 푸시 |
| **WIP는 main에 올리지 않음** | 로컬 커밋 OK · 공유=Draft PR ([collaboration §2.5](./collaboration.md)) |
| **Phase 0** | 개인 레포에서 기능 완성 허용 |
| **위험·공유만 PR** | interfaces / launch / docker / UDP / 교차 / 대형 |
| **파트 격리** | [team-dev-guide.md](../onboarding/team-dev-guide.md) |
| **짧은 수명** | 브랜치 쓰면 수시간~2~3일 내 merge |
| **장기 개인 브랜치 금지** | 이름만 있는 누적 브랜치 금지 |

이전 “모든 작업 feature + main 직푸시 금지”는 **폐기**. Phase 0/1·WIP: [collaboration.md](./collaboration.md).

---

## 2. Path 선택 가이드

```
변경이 내 소유 디렉터리만? ─No──► Path B (PR)
         │ Yes
공유 인프라·interfaces·UDP·launch 계약? ─Yes──► Path B
         │ No
main을 며칠 깨뜨릴 실험/대형 기능? ─Yes──► Path B
         │ No
         └──────────────────────────► Path A (main 직푸시)
```

| Path | 명령 요약 |
|------|-----------|
| **A** | `checkout main` → pull → 수정 → build → commit → `push origin main` |
| **B** | `checkout -b feature/...` → push → `gh pr create` → review → merge |

---

## 3. Path A — 명령 전체

```bash
cd "$ASMC"   # 또는 clone 경로
git checkout main
git pull --rebase origin main
git submodule update --init --recursive

# 담당 경로만 수정 (예: mpc)
# ...

./scripts/docker_ros_up.sh up
docker exec -it asmc-ros-noetic bash -c './scripts/build_ws.sh'

git status   # 남 영역 파일이 섞였는지 확인
git add src/mpc_controller/
git commit -m "fix(mpc): clamp accel on red light stop"
git pull --rebase origin main
git push origin main
```

### push가 rejected 될 때

다른 사람이 먼저 push한 경우:

```bash
git pull --rebase origin main
# 충돌 해결
git push origin main
```

### 실수로 공유 파일을 고쳤을 때

```bash
# 아직 push 전
git restore src/interfaces/README.md   # 예시
# 또는 stash 후 Path B로 다시

# 이미 push했다면
git revert <commit>   # 또는 즉시 fix PR
```

---

## 4. Path B — 명령 전체

```bash
git checkout main
git pull origin main
git checkout -b feature/jaeho-det3d-nms

# 작업 ...
git add src/perception/morai_3d_detection/
git commit -m "feat(perception): NMS threshold for VLP16"
git push -u origin feature/jaeho-det3d-nms

gh pr create --title "feat(perception): NMS for VLP16" --body "$(cat <<'EOF'
## Path
- [x] Path B (사유: 출력 토픽 계약이 planner와 맞물림)

## Summary
- ...

## Test
- [ ] ./scripts/build_ws.sh
- [ ] sim-verification-checklist 해당 §
EOF
)"
```

### merge 후 정리

```bash
git checkout main
git pull origin main
git branch -d feature/jaeho-det3d-nms
git push origin --delete feature/jaeho-det3d-nms   # 원격에 남았으면
git submodule update --init --recursive
```

### rebase (PR이 main보다 뒤처질 때)

```bash
git checkout feature/jaeho-det3d-nms
git fetch origin
git rebase origin/main
git push --force-with-lease
```

---

## 5. 브랜치 이름 (Path B)

```
main                          ← trunk (일상 합류점)
├── feature/<이름>-<기능>     ← Path B
├── fix/<이름>-<이슈>
├── docs/<이름>-<주제>
└── integrate/<모듈>          ← 통합 리드 (계약·이식)
```

| 예 | 언제 |
|----|------|
| `feature/yuntae-udp-ctrl-cmd` | UDP 제어 경로 (시뮬 경계) |
| `feature/seunghyun-interfaces-tracks` | tracks 스펙 |
| `fix/jang-scenario-timeout` | 시나리오 타임아웃 (교차 가능 시) |
| `integrate/tl-fsm` | TL-Net ↔ Behavior FSM 최초 연결 |

---

## 6. 커밋 메시지

```
feat(scope): 한 줄 요약
fix(scope): 버그 수정
docs: 문서만
chore(docker): Docker 설정
refactor(planner): 리팩터 (동작 변경 없음)
```

`scope` 예: `mpc`, `planner`, `behavior`, `perception`, `scenario`, `bridge`, `udp`, `lbc`, `docker`, `integration`

---

## 7. 충돌 방지 규칙

| 규칙 | 설명 |
|------|------|
| 담당 디렉터리만 | [team-dev-guide.md](../onboarding/team-dev-guide.md) · [repository-layout.md](../architecture/repository-layout.md) |
| 공통 파일 | `integration_launch/`, `interfaces/`, `docker/` → **Path B + 합의** |
| msg 스키마 | `behavior_planner/msg/*` 변경 시 planner bridge **동시 PR** |
| 동시 작업 | 같은 파일은 시간 나눠 쓰거나 한 사람이 owner |

---

## 8. 통합 리드 역할 (안승현)

- Path B 필수 구간(interfaces·docker·submodule·UDP) 조율
- GitHub `main` 푸시 권한·CI 설정이 문서와 일치하는지 유지
- `main`이 빨간 상태로 방치되지 않게 에스컬레이션
- 아키텍처·하이브리드 Phase gate 공지

---

## 9. GitHub · 자동화

| 항목 | 위치 |
|------|------|
| 조직 | [INHAautonav](https://github.com/INHAautonav) |
| Repo | [2026-ASMC](https://github.com/INHAautonav/2026-ASMC) |
| CODEOWNERS | [.github/CODEOWNERS](../../.github/CODEOWNERS) |
| CI | [.github/workflows/ci.yml](../../.github/workflows/ci.yml) — **PR + main push** |

협업 철학·ownership 표: [collaboration.md](./collaboration.md)
