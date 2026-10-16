# D1 문서 경로 이전 결과

> **기준:** `main@577231b`에서 작성한 구조 검토안 `65d505b`
>
> **상태:** 경로 이전과 내부 참조 교정 완료, 본문 통합 미착수

## 적용 범위

- D0 인벤토리에 승인된 문서 38개를 `git mv`로 이동했다.
- Markdown 본문은 새 위치에 한 벌만 유지한다.
- 문서·소스 주석·설정·테스트에 있는 저장소 내부 경로를 새 위치로 교정했다.
- 기존부터 상대 경로 깊이가 잘못됐던 원태 HD Map 개발 README 링크 4개도 함께 교정했다.
- frozen runtime baseline YAML은 당시 저장소를 기록한 산출물이므로 내용을 다시 쓰지 않았다.

## 한 릴리스 호환 안내

외부 북마크와 과거 PR 링크 보호를 위해 다음 기존 경로에는 정본으로 연결하는 짧은 stub만
남겼다. 새 코드와 문서는 이 경로를 참조하지 않는다.

- `docs/getting-started.md`
- `docs/team-dev-guide.md`
- `docs/collaboration.md`
- `docs/network-udp-ros.md`
- `docs/planning-control-udp.md`
- `docs/sim-verification-checklist.md`
- `docs/repository-consolidation-roadmap.md`
- `docs/perception-map-fusion-roadmap.md`
- `docs/perception-dataset-labeling-split.md`
- `docs/contracts/hd-map-topic-contract-v1.md`
- `docs/contracts/hd-map-runtime-visualization-v1.md`
- `docs/contracts/kcity2025-competition-metadata-v1.md`

이 stub은 D2 본문 통합과 링크 안정화가 끝난 뒤 D3 삭제 검토 대상으로 다룬다.

## 검증 기준

- 로컬 Markdown 링크는 저장소 밖 워크스페이스 자료를 포함해 실제 파일 존재 여부를 검사한다.
- `git diff --check`가 통과해야 한다.
- HD Map audit 테스트와 경로를 직접 소비하는 도구의 구문 검사를 통과해야 한다.
- D2 전에는 여러 문서의 내용을 합치거나 오래된 기록을 삭제하지 않는다.

## 실행 결과

- 로컬 Markdown 링크: 495개 검사, 누락 0개
- `git diff --check`: 통과
- shell/Python 구문 검사: 통과
- `test_drivable_area_audit_workspace.py`: 3개 통과
- `test_compact_drivable_workspace.py`: 9개 통과
- `build_drivable_area_input_audit.py --check`: deterministic/current
- `build_compact_drivable_map.py --check`: deterministic/current

P1-A runtime baseline v1은 당시 package/topic/HD Map 상태를 기록한 frozen snapshot이다. 현재
`main`의 후속 HD Map runtime 변경으로 이미 재생성 결과와 차이가 있으므로 D1에서 덮어쓰지
않는다. 다음 계약 baseline 버전 갱신은 문서 이동과 분리해 수행한다.
