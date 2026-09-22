# D4 패키지 README 정본화 결과

> **상태:** 완료 · 실제 package는 `src/<package>/README.md`, package group은 `docs/packages/`

## 판단 원칙

실제 catkin package 한 개의 목적·구조·실행·설정·변경·검증은 source와 함께 바뀌므로 package
루트 README를 정본으로 둔다. 여러 현재 package를 묶거나 아직 존재하지 않는 통합 package의
경계·이관 gate는 중앙 `docs/packages/`에 둔다. 계약·실험·과거 결정은 각각 `contracts/`,
`records/`, `archive/` 책임을 유지한다.

| 문서 | 판단 | 현재 정본 |
|---|---|---|
| `asmc_hd_map` 1:1 개발 가이드 | package README로 통합 | `src/asmc_hd_map/README.md` |
| `asmc_msgs` 1:1 개발 가이드 | package README로 통합 | `src/asmc_msgs/README.md` |
| bridge 통합 가이드 | 현재 3개 runtime package와 launch를 묶음 | `docs/packages/asmc_bridge/README.md` |
| perception 통합 가이드 | 현재 여러 runtime·학습·수집 package를 묶음 | `docs/packages/asmc_perception/README.md` |
| Planning/Control 가이드 | Behavior·Frenet·MPC 공통 경계 | `docs/packages/planning-control/README.md` |

향후 실제 `asmc_bridge`나 `asmc_perception` package가 생기면 해당 package의 실행·변경 절차는
새 source README로 옮기고, 중앙 문서에는 교차 package 설계와 migration 기록만 남긴다.

## 내용 보존

기존 중앙 HD Map guide 215줄의 builder/runtime, authority, pose, current-lane, source comparison,
변경 gate와 과거 근거 행선지를 package README에 합쳤다. 기존 source README의 빠른 실행,
제품 개수와 판제 소비 설명도 유지해 통합본은 234줄이다.

기존 중앙 message guide 95줄의 25개 타입 분류, ROS1 MD5 규칙, 기계 정본, 검증과 신규 message
추가 절차를 package README에 합쳤다. 기존 source README의 HD Map global/local 소비 경계도
유지해 통합본은 107줄이다. 계약 의미 자체는 계속 `docs/contracts/`가 정본이다.

두 중앙 원문의 비어 있지 않은 행을 통합본과 대조했다. literal하게 달라진 행은 제목·절 번호,
`docs/` 기준 상대 링크, 두 문서를 합치면서 다시 쓴 소개 문장뿐이었다. feature count, 좌표·거리,
명령, topic, 변경 순서와 provenance 표에서 사라진 고유 값은 없었다.

중앙 guide를 stub으로 남기지 않은 이유는 두 경로가 이 migration 브랜치에서만 생겼고 아직
`main`이나 배포에 포함되지 않았기 때문이다. 저장소와 관리 중인 상위 workspace의 링크는 모두
source README로 직접 교정했다.

## 호환 stub 정리

D3의 17개 중 다음 5개도 D1 뒤 이 브랜치에서만 생긴 중간 경로라 제거했다.

- `docs/competition/assets.md`
- `docs/competition/environment.md`
- `docs/operations/docker.md`
- `docs/operations/network-udp-ros.md`
- `docs/operations/sim-verification-checklist.md`

과거 `main@577231b`에 실제로 존재했던 12개 경로는 한 번의 후속 팀 배포가 끝날 때까지
유지한다. machine manifest와 CI checker도 12개를 정확히 요구한다.

## 검증

```bash
python3 tools/audit/check_documentation_migration.py
python3 tools/audit/check_documentation_migration.py --with-history
python3 tools/audit/validate_p1b_contract.py
python3 -m unittest tools/audit/test_p1b_contract.py
git diff --check
```

추가로 저장소 Markdown target, 삭제한 중앙 guide 역참조, 빈 `docs/` 디렉터리와 두 README의
원문 heading·고유 수치·명령 보존 여부를 검사한다.
