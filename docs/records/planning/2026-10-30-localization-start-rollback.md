# Localization 시작 코드 복원과 전역 출발 재현

## 체크포인트

- 사용자 요청: 2026-10-30 05:10까지 시작 코드 복원 및 전역 정지 조사.
- 시작 코드: `b001c2c1bf01600e9ce2f73eb459f78335cde714`.
- 철회한 실험: `75160004`, `08a43ca7`. 보존 브랜치 `archive/localization-local-recovery-20261030`.
- 복원 커밋: `08381935`, `0e3ac8f3`. 문서 추가 전 `git diff b001c2c1 --exit-code` 통과.
- LOCAL recovery 자동 전환 실험은 현재 실행 코드에 없음. 기본 camera와 명시적 localization 선택은 시작 코드 그대로.
- `asmc-jamming-compare`의 localization/planner/MPC/selector 재빌드 완료. 다른 worktree 변경 없음.

## 실제 확인

공유되지 않은 master 11373에서 bridge/camera/LiDAR와
`autonomy_no_bridges_v2.launch jamming_planner:=localization dashboard:=false jamming_speed_kph:=10`
을 실행했다. 모니터는 8089, LiDAR는 8097이다.

센서 5초 수신: GPS 21, IMU 175, LiDAR 54, 전방영상 89, Ego 속도 174.
GPS state valid, localization anchor_ready=true, selector NORMAL, command_fresh=true.
처음 정지 원인은 NORMAL KEEP의 저속 경로 생성 실패였다:
`generated=0 blocking_id=-1`. 관측된 상태 yaw -1.8857rad와 시작 경로 접선 약61.3도는
약169도 차이가 있었다. 장애물 차단이나 LOCAL 제어 소유권 때문에 멈춘 것으로 확인되지는 않았다.

시나리오 도구로 시작 자세를 62.515도로 맞추자 경로와 가속 명령은 생성됐다.
그러나 실제 짧은 주행에서 출발 위치 대비 약31.17m 이동 방향은 -113.51도였고,
출발 시 native/IMU 보고 방향은 +62.51도였다. 약176도의 불일치가 있어 시험을 중단했다.
주행 중 localization도 `reanchor_wait_stopped`, anchor_ready=false로 바뀌었다.
이는 GPS 실측 이동과 상대 이동 예측이 어긋난 추가 증거이며, 이 보호를 제거하지 않았다.
따라서 이것은 전역경로 추종 성공이 아니다. native velocity 필드 단위를 m/s로 가정하지 않는다.
시뮬레이터 자세/진행 방향 또는 reset 경계 문제를 추가 분리해야 하며,
원인 확인 없이 yaw/조향 부호를 뒤집거나 경로 검사를 우회하지 않았다.
시작 자세 문제만으로 전체 원인이 해결됐다는 앞선 잠정 가설은 이 주행 결과로 기각한다.

## 결과와 남은 범위

- 시작 소스 복원과 바이너리 재빌드 완료.
- 실제 센서 입력, anchor, NORMAL 선택, 경로/명령 생성 확인.
- 전역 추종, A448 무정지 진입, localization↔LOCAL 복구는 **미수락**.
- 시험 autonomy는 종료하고 ego를 시작 위치/P 기어로 놓았다. pause는 해제했다.
- bridge, camera, LiDAR, master, 두 웹은 관측을 위해 유지한다.
- 시험용 MPC validation_speed_cap_kph 파라미터는 종료 후 삭제한다.
- push/merge 없음. 다음 작업은 동일 자세에서 실제 이동 벡터와 GPS/IMU 방향을 먼저 맞추는 것.

## 근거

`D:/morai dataset/shahn03/artifacts/jamming-three-way-20261029-r02/restore-global-20261030-0503/`

- events.jsonl: 복원 후 상태/selector/anchor/trace.
- aligned-drive.jsonl: 방향 재설정 뒤 native 위치/속도/방향 및 진단; 실패 근거.
- baseline-autonomy.log, aligned-autonomy.log: 경로 생성 실패와 재설정 후 명령 생성.

복원은 두 revert 커밋으로 이력을 보존했다. 이전 실험을 재검토하려면 archive 참조에서
별도 비교하고, 이번 문서를 주행 수락으로 사용하지 않는다.
