# 즉시 LOCAL 복구와 음영 인계 순서 보완

기준: `5e03ba27` (원격 push 완료). 통합 브랜치 `integrate/jamming-three-way-20261029`, 컨테이너 `asmc-jamming-compare`에서만 변경했다.

## 수정

- localization의 회복 가능한 실패와 최신 검증 LOCAL 후보가 동시에 있으면 첫 실패 계획에서 LOCAL을 선택한다. 연속 2회/80ms 대기를 제거했다. native 복귀는 기존 1초 안정화 조건을 유지한다.
- 동적/미확정 운동 위험, 센서·시각 무효, LOCAL 후보 무효는 기존처럼 전환을 허용하지 않는다. 아직 native가 정상이라고 판단하는 미검출 장애물은 이 수정만으로 선제 검출되지 않는다.
- selector에서 모드 메시지가 준비 요청보다 먼저 도착하면 즉시 최대 제동하던 순서 의존성을 수정했다. 기존 100ms 인계 창과 현재 명령 freshness 안에서 요청·후보를 기다린다. 이전 명령을 재발행하거나 watchdog을 갱신하지 않는다.
- 인계 후보는 현재 요청의 대상 모드·epoch·수신 나이와 일치해야 한다. 초기 기동, 입력 단절, 준비 실패의 보호 동작은 유지한다.
- 이전 MPC의 실제 발행 이력 기반 준비와 감속 상한 보완, 통합 launch의 smooth_handoff=true가 유지됨을 확인했다.

## 검증

- LOCAL 복구 정책 35개 검사 성공. 첫 실패 즉시 전환, 이동 중 전환, 동적 차단, stale/세대/시각 조건과 native 복귀 검사 포함.
- localization node 및 selector 빌드 성공.
- 독립 ROS 시험: localization_handoff.test 성공 (준비 진입·복귀·epoch·미준비 실패·250ms watchdog).
- 독립 ROS 시험: handoff_mode_first.test 성공 (모드→20ms→요청→20ms→후보에서 최대 제동 삽입 0, 이후 명령 중단 시 watchdog 제동).
- 임시 검사 로그: 컨테이너 `/tmp/immediate-local-{build,test,handoff-test}.log`, `/tmp/handoff-{race-build,mode-first-test}.log`.

## 현재 실행과 한계

07:26 KST 조회에서 master11373의 모든 MORAI 센서 및 Ego 수신이 0Hz이고 약 2분 이상 중단돼 있었다. localization=position_state_stale, LOCAL=motion_unavailable, controller=local_receive_stale/brake1이었다. 현재 정지는 이 입력 단절과 구분해야 한다.

현재 실행 노드를 재시작하거나 시뮬 배치를 바꾸지 않았다. 위 검사는 별도 master에서 수행했으며 실주행 진입·탈출 수락은 아니다. 기존 노드는 시작 시 로드한 바이너리/설정을 계속 쓰므로 새 코드는 autonomy 재시작 후 적용된다.

복구는 이 변경 커밋을 revert하고 localization 및 selector를 재빌드한 후 autonomy를 재시작한다. 앞선 0m margin 변경은 별도 커밋 `5e03ba27`로 독립 복구할 수 있다.
