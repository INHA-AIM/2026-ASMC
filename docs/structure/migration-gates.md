# 문서 구조 이전 게이트

## D0 — 인벤토리 검토 (완료)

- 기존 문서 본문과 경로를 변경하지 않는다.
- 모든 문서에 제안 위치와 상태를 부여한다.
- 팀 저장소와 상위 워크스페이스 `docs/`의 경계를 명시한다.
- 로컬에만 있는 내용을 commit/branch 식별자와 함께 보존한다.

완료 조건은 사용자가 [목표 구조](./README.md)와
[파일별 이동표](./current-inventory.md)를 확인하는 것이다.

## D1 — 경로만 이전 (완료)

- 별도 PR과 단일 목적 브랜치에서 `git mv`만 사용한다.
- 각 문서의 본문 의미는 변경하지 않는다.
- 저장소 내부 상대 링크, package README 링크와 GitHub Actions 참조를 전수 교정한다.
- 기존 주요 진입 경로에는 한 릴리스 동안 새 정본 링크만 담은 compatibility stub을 둔다.
- `rg` 기반 dead-link 검사와 Markdown link checker 결과를 PR에 첨부한다.

## D2 — 디렉터리별 내용 통합

한 PR에 아래 한 묶음만 다룬다.

1. `onboarding/`와 `governance/` — 완료
2. `competition/`와 `operations/` — 완료
3. `contracts/` — 완료
4. `packages/asmc_hd_map/`와 `packages/asmc_msgs/` — 완료
5. `packages/asmc_bridge/` — 완료
6. `packages/asmc_perception/`
7. `packages/planning-control/`와 `handoffs/`
8. `records/`와 `archive/`

각 PR은 원문 절→새 문서 절 대응표를 남겨 누락 여부를 검토한다. 수치·경로·명령을 바꾸는
경우 구조 정리와 분리해 별도 구현 근거를 요구한다.

## D3 — 중복·옛 문서 제거

- 정본에 모든 고유 내용이 옮겨졌는지 절 단위로 확인한다.
- 삭제 대상은 마지막 commit과 대체 문서를 archive index에 기록한다.
- `Historical/Legacy` 자체는 오래됐다는 이유만으로 삭제하지 않는다.
- 개인 작업에만 필요한 상세 조사·원문·대용량 자료는 상위 워크스페이스 `docs/`에 남기고,
  팀 저장소에는 결론·계약·재현 방법만 보존한다.

## 공통 금지 사항

- 이동과 대규모 본문 개정을 같은 커밋에 넣지 않는다.
- Working Proposal의 문장을 근거 없이 확정 계약으로 바꾸지 않는다.
- 날짜가 있는 실험 결과를 최신 성능처럼 덮어쓰지 않는다.
- 외부 원문을 출처 없이 요약 문서에 복사하지 않는다.
- 링크가 깨진 상태로 중간 migration commit을 main에 병합하지 않는다.
