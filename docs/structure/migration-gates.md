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
6. `packages/asmc_perception/` — 완료. 상세 분석·실험 본문은 축약하지 않고 진입점과 권위만 통합
7. `packages/planning-control/`와 `handoffs/` — 완료. 현재 ROS runtime과 legacy handoff 분리
8. `records/`와 `archive/` — 완료. snapshot/product/experiment/history 갱신 정책 분리

각 PR은 원문 절→새 문서 절 대응표를 남겨 누락 여부를 검토한다. 수치·경로·명령을 바꾸는
경우 구조 정리와 분리해 별도 구현 근거를 요구한다.

## D3 — 중복·옛 문서 제거 (감사 완료, 후속 배포 대기)

- 정본에 모든 고유 내용이 옮겨졌는지 절 단위로 확인한다.
- 삭제 대상은 마지막 commit과 대체 문서를 archive index에 기록한다.
- `Historical/Legacy` 자체는 오래됐다는 이유만으로 삭제하지 않는다.
- 개인 작업에만 필요한 상세 조사·원문·대용량 자료는 상위 워크스페이스 `docs/`에 남기고,
  팀 저장소에는 결론·계약·재현 방법만 보존한다.

[D3 보존·호환 감사](./d3-retention-compatibility-report.md)에서 D1 이전 Markdown 37개와
generated text 2개의 행선지를 모두 확인했다. 문서 migration은 PR #45, merge commit
`7107531b`로 `main`에 반영됐다. 아직 그 뒤의 팀 배포가 완료되지 않았으므로 과거 `main`
경로 stub 12개와 generated export는 삭제하지 않는다. 제거 gate는
`config/docs/documentation_migration_v1.json`에 기계 판독 형태로 고정한다.

## D4 — 패키지 README 정본화 (완료)

- 실제 package 한 개의 실행·구조·변경·검증은 `src/<package>/README.md`에 둔다.
- 여러 package를 묶는 통합 경계와 아직 없는 목표 package는 `docs/packages/`에 둔다.
- `asmc_hd_map`·`asmc_msgs` 중앙 guide의 고유 내용을 source README에 합친 뒤 제거한다.
- 이 브랜치에만 존재한 중간 호환 stub 5개는 제거하고, 과거 `main` 경로 12개만 유지한다.
- 결과와 보존 검증은 [D4 보고서](./d4-package-readme-report.md)에 기록한다.

## D5 — 병합 상태 closeout (완료)

- PR #45와 merge commit `7107531b`를 migration manifest와 D3 감사 기록에 남긴다.
- 호환 유예 시작 상태를 `main 병합 완료·후속 팀 배포 대기`로 갱신한다.
- 호환 안내 12개와 historical 자료의 삭제·이동은 이번 단계에 포함하지 않는다.
- 이후 제거는 D3 gate를 모두 만족하는 별도 PR에서만 수행한다.

## 공통 금지 사항

- 이동과 대규모 본문 개정을 같은 커밋에 넣지 않는다.
- Working Proposal의 문장을 근거 없이 확정 계약으로 바꾸지 않는다.
- 날짜가 있는 실험 결과를 최신 성능처럼 덮어쓰지 않는다.
- 외부 원문을 출처 없이 요약 문서에 복사하지 않는다.
- 링크가 깨진 상태로 중간 migration commit을 main에 병합하지 않는다.
