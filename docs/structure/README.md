# 팀 문서 정보 구조

> **상태:** D5 Post-merge Closeout — 문서 구조 정리 완료, 호환 유예 진행 중
>
> **최종 갱신:** 2026-09-23
> **진행:** D1~D4 정리 결과가 PR #45로 `main`에 병합됐다. 과거 경로 12개는 후속 팀 배포
> 1회가 끝날 때까지 호환 안내로 유지한다.

## 목적

현재 `docs/` 루트에는 정본 계약, 실행 가이드, 패키지 설계, 인수인계, 실험 기록과 과거
의사결정이 함께 있다. 내용 손실과 의미 왜곡 없이 정리하기 위해 다음 순서를 고정한다.

1. 현재 파일을 그대로 둔 채 역할·대상·정본성을 전수 분류한다.
2. 아래 목표 디렉터리와 파일별 이동표를 팀이 검토한다.
3. 별도 PR에서 `git mv`와 링크 교정만 수행한다. 본문은 바꾸지 않는다.
4. 이동 결과를 확인한 뒤 디렉터리별로 중복 제거·통합·상태 갱신을 진행한다.
5. 삭제 후보는 대체 정본과 내용 parity가 확인된 뒤에만 제거한다.

현재 파일별 분류와 제안 경로는 [current-inventory.md](./current-inventory.md), 실제 이동 시
검증 절차는 [migration-gates.md](./migration-gates.md)를 따른다.
이동 결과와 호환 stub 목록은 [d1-path-migration-report.md](./d1-path-migration-report.md)에 기록한다.
본문 통합 결과는 [온보딩·협업 보고서](./d2-onboarding-governance-report.md)와
[대회·운영 보고서](./d2-competition-operations-report.md),
[계약 보고서](./d2-contracts-report.md),
[HD Map·message 보고서](./d2-hd-map-msgs-report.md),
[bridge 보고서](./d2-bridge-report.md),
[perception 보고서](./d2-perception-report.md),
[Planning/Control·handoff 보고서](./d2-planning-handoffs-report.md),
[records·archive 보고서](./d2-records-archive-report.md)처럼 묶음별로 기록한다. 큰 폭으로 줄인 앞선 두 묶음은
[본문 축약 보존 감사](./d2-content-parity-audit.md)에서 원문 절과 현재 행선지를 다시 확인했다.
D3의 전체 원문 inventory와 외부 경로 유지 판단은
[정보 보존·경로 호환 감사](./d3-retention-compatibility-report.md)를 따른다.
[D4 package README 결과](./d4-package-readme-report.md)는 source-local 문서와 중앙 package-group
문서의 경계를 기록한다.

## 목표 구조

```text
docs/
├── README.md                         전체 문서 허브와 정본 우선순위
├── onboarding/                       처음 참여한 팀원을 위한 최소 경로
├── governance/                       Git, ownership, 문서 관리 정책
├── competition/                      공식 규정 요약, 대회 자산과 시뮬 기준
├── architecture/                     시스템·저장소 목표 구조와 단계별 로드맵
├── contracts/
│   ├── common/                       공용 메시지·좌표계·시간 규약
│   ├── competition/                  체크포인트·미션 metadata
│   ├── hd-map/                       HD Map topic/runtime 소비 계약
│   ├── perception/                   데이터셋·인지 출력 계약
│   ├── planning/                     FSM/Frenet/행동 계약
│   └── simulator/                    MORAI message 호환 정책
├── packages/
│   ├── asmc_bridge/                  UDP·센서 decoder·환경 profile
│   ├── asmc_camera/              camera/LiDAR/BEV/fusion 개발 문서
│   └── planning-control/             planner/controller 설계 참고
├── operations/                       Docker, 네트워크, rosbag, 시뮬 검증·배포
├── handoffs/
│   ├── perception/                   인지 출력 인수인계
│   └── planning/                     판제 연동 인수인계
├── records/
│   ├── baselines/                    동결된 회귀 기준
│   ├── hd-map/                       날짜가 있는 지도 검수·audit 기록
│   ├── perception/                   모델·데이터 실험 결과
│   └── simulator/                    버전 조사 기록
└── archive/                          현재 실행에 쓰지 않는 과거 설계·이식 기록
```

## 분류 원칙

- `contracts/`는 다른 파트가 구현을 맞춰야 하는 인터페이스만 둔다.
- 실제 package 한 개의 개발·검증은 `src/<package>/README.md`에 둔다.
- `packages/`는 여러 현재 package의 공통 경계 또는 아직 없는 통합 package 계획을 둔다.
- `operations/`는 여러 패키지를 함께 실행하거나 현장에 배포하는 절차를 둔다.
- `handoffs/`는 생산자와 소비자 사이의 현재 인수 조건을 둔다.
- `records/`는 특정 날짜·버전의 사실을 보존하며 최신 설정으로 해석하지 않는다.
- `archive/`는 역사적 맥락을 보존하되 문서 상단에서 현재 정본으로 연결한다.
- 대용량 원문 PDF, 개인 연구 자료와 작업 중 초안은 팀 저장소가 아니라 워크스페이스 상위
  `docs/`에서 관리하고 팀 문서에는 필요한 결론과 출처만 남긴다.

## 진행 현황

- D1: 경로 이전과 호환 안내 완료
- D2-1: `onboarding/`·`governance/` 통합 완료
- D2-2: `competition/`·`operations/` 통합 완료
- D2-3: `contracts/` 정리 완료
- D2-4: `packages/asmc_hd_map/`·`packages/asmc_msgs/` 통합 완료
- D2-5: `packages/asmc_bridge/` 통합 완료
- D2-6: `packages/asmc_perception/` 진입점·권위 통합 완료. 세부 분석 본문은 원문 보존
- D2-7: `packages/planning-control/`·`handoffs/` 통합 완료. standalone UDP 원문은 archive 보존
- D2-8: `records/`·`archive/` 통합 완료. 기존 실측·분석 본문과 generated export 보존
- D3: 원문 37개와 generated export 2개의 행선지·byte/절 parity 감사 완료. 과거 main 경로
  호환 안내와 historical export는 후속 팀 배포 gate까지 유지
- D4: `asmc_hd_map`·`asmc_msgs` 1:1 가이드를 source README에 통합하고 branch-only stub 5개
  제거. 실제 package와 package-group 문서 경계 확정
- D5: PR #45·merge commit `7107531b`를 기록하고 호환 유예를 `main 병합 완료·후속 배포 대기`
  상태로 전환

각 D2 묶음은 원문→정본 대응표와 링크 검증 결과를 남긴다. Working Proposal을 근거 없이
Canonical로 바꾸거나 날짜가 있는 기록을 최신 설정처럼 덮어쓰지 않는다.
