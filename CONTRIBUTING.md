# 기여 가이드

현재 Git·PR 정본은 [협업 규칙](docs/governance/README.md), 담당 경계는
[팀 ownership](docs/onboarding/ownership.md)이다.

## 작업 순서

```bash
git switch main
git pull --ff-only origin main
git submodule update --init --recursive
git switch -c feature/<이름>-<작업>

# 구현과 검증 후
git add <이번-작업-경로>
git commit -m "feat(scope): concise summary"
git push -u origin HEAD
gh pr create --fill
```

- 모든 변경은 짧은 브랜치와 PR로 합류한다.
- 작업 중인 변경은 Draft PR로 공유한다.
- 한 PR에는 한 목적만 담고, 이동·이름 변경과 동작 변경을 분리한다.
- 공용 message/topic, 좌표계, UDP, Docker/CI와 여러 패키지 변경은 관련 owner가 함께
  검토한다.
- 테스트하지 못한 항목과 rollback 방법을 PR에 명시한다.

## 커밋하지 않는 것

- bag, dataset, model checkpoint와 기타 대용량 산출물
- 비밀값, 개인 IP/profile과 호스트 절대경로
- `build/`, `devel/`, cache와 생성 중간물
- `src/MORAI-ROS_morai_msgs/` submodule 내부의 직접 변경
- 합의되지 않은 privileged 배포 설정

## PR 전 확인

- [ ] 최신 `main`과 정렬했고 이번 작업과 무관한 diff가 없다.
- [ ] 담당 owner와 영향받는 producer/consumer를 적었다.
- [ ] 범위에 맞는 build, unit test 또는 bag/sim 검증을 실행했다.
- [ ] message/topic 변경은 MD5·serialization과 consumer build를 확인했다.
- [ ] 설정·실행법·계약이 바뀌었다면 관련 문서도 갱신했다.
- [ ] 데이터, 비밀값, 절대경로와 불필요한 생성물이 없다.
- [ ] submodule pointer 변경 의도와 대상 commit을 확인했다.

통합 검증은 [시뮬 검증 체크리스트](./docs/operations/README.md), MORAI message
변경은 [beta_drive 정책](docs/contracts/simulator/morai-msgs-beta-policy.md)을 따른다.
