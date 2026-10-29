# PR #143 로터리 런타임 롤백

사용자 요청: LiDAR를 터널에 한정한 뒤에도 로터리 진입·진입 후 정지 거동이 이전보다 악화되어 PR #143 도입 전 구현으로 복귀한다. 사고 원인이 PR #143이라고 확정한 변경은 아니다.

## 범위

- 적용 전 런타임: `73090859180ebd92cb2460e6f90adfb4bae7ef21`.
- 취소 대상: 통합 merge `f0a600476f45febfb55c87e28a5c7469b962f2e9`의 `src/asmc_planner` 변경 10개 파일.
- 복원 기준: 해당 merge 첫 부모 `5076c3f9`. 대상 파일은 바이트 단위로 이 기준과 동일함을 확인했다.
- READY 알림 보존, 추가 departure certificate, entry_reason 진단과 그 기능 전용 시험을 함께 원복한다. 기존 로터리 transaction/candidate/execution/prediction/departure bounds 시험은 유지한다.
- 전방/후방 시간 간격: `0.6/1.0s` → `1.0/2.0s`. 이외 차량 제원·일반 주행·재밍 설정은 변경하지 않는다.
- PR #142 camera, PR #144 LiDAR와 후속 sparse/tunnel-only 수정, 세 방식 A448 인계는 유지한다.
- 기록 스크립트와 시나리오 반복 횟수는 주행 정책이 아니므로 유지한다.

## 검증·운영

별도 빌드 컨테이너 `asmc-jamming-pr141-sync-build`에서 production planner, localization, LOCAL과 관련 회귀를 빌드한다. 시험은 운영 master 11373에 연결하지 않는다. 실제 MORAI 재주행은 이번 롤백의 수락 근거에 포함하지 않는다.

결과 정본: `data/runtime/roundabout-pr143-rollback-20261030-r02` (D 저장소 기준).
기존 실행 overlay 보존: 비교 worktree `.runtime/pre-roundabout-rollback-73090859-20261030`.
롤백을 취소하려면 이 변경 커밋을 revert하고 동일 source로 재빌드한 뒤 정지된 autonomy에 적용한다. 실행 중인 shared library를 덮어쓰지 않는다.

원격 PR #141에는 현재 로컬에 없는 후속 로터리 변경 `0b5b20c6`가 있다. 로컬 롤백을 원격 변경 전체의 롤백으로 표현하지 않는다. 원격 적용 여부와 실행 identity는 위 결과 manifest에 따로 기록한다.

### 로컬 검증 결과

- Release 빌드: `asmc_planner_node`, `asmc_shadow_localization_node`, `asmc_local_planner_node` 성공.
- 로터리 5개 회귀(transaction/candidate/execution/prediction/departure_bounds): 모두 exit 0.
- 공통 회귀(jamming_policy/local_planner/jamming_wall_scope/shadow_localization 15): 모두 exit 0.
- 세 옵션 launch graph와 jamming launch contract(6개): 통과.
- `git diff --check`: 통과. 실제 차량 재주행 및 최신 원격 전체 CI는 이 로컬 검증에 포함하지 않는다.
