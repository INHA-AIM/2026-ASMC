## Summary

<!-- 무엇을 왜 변경했는지 간결하게 적어 주세요. -->

## Scope and owners

- [ ] 단일 패키지 내부 변경
- [ ] 공용 message/topic/좌표계 계약
- [ ] UDP·센서·시뮬레이터 경계
- [ ] Docker·CI·submodule
- [ ] 여러 패키지 또는 삭제·이동

담당 owner / reviewer:

## Contract impact

<!-- 해당 없으면 N/A. producer, consumer, 호환성, 적용 순서와 rollback을 적어 주세요. -->

## Validation

실행한 명령과 결과:

- [ ] 해당 패키지 build/unit test
- [ ] bag replay 또는 simulator smoke test
- [ ] message MD5/topic rate/geometry 등 회귀 기준 비교
- [ ] 문서·설정·실행법 동기화

실행하지 못한 검증과 이유:

## Ready check

- [ ] 최신 `main`과 정렬했고 WIP가 아니다. WIP이면 Draft PR이다.
- [ ] 한 목적만 포함하며 무관한 diff가 없다.
- [ ] 데이터·checkpoint·비밀값·호스트 절대경로가 없다.
- [ ] `build/`, `devel/`, cache 등 생성물을 추가하지 않았다.
- [ ] submodule pointer 변경 의도와 대상 commit을 확인했다.

정책: [협업 규칙](../docs/governance/README.md) ·
[팀 ownership](../docs/onboarding/ownership.md) ·
[시뮬 검증](../docs/operations/sim-verification-checklist.md)
