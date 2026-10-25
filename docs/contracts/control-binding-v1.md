# 명령과 실제 제어 궤적의 연결 v2

구현: [control_binding.hpp](../../src/asmc_msgs/include/asmc_msgs/control_binding.hpp).
기존 `BehaviorCommand`, `FrenetFeedback`, `Trajectory` 정의와 필드 의미는 유지한다.
시간은 신선도에만 사용한다. `frame_id`, 설명 문자열을 명령 ID로 재해석하지 않는다.

## 전송과 활성화

`/control_binding/enabled=true`에서만 추가 계약을 활성화한다. 기본은 false다.
활성 Controller와 외부 입력 Planner는 기존 명령·궤적 토픽을 호환용 mirror로만
취급한다. 기존 토픽으로 연결 검사를 우회할 수 없다. 모든 producer/consumer를
같은 버전으로 빌드하고 함께 재시작해야 한다.

| 토픽 | 기존 표준 ROS 형식 | 내용 |
|---|---|---|
| `/control/binding_epoch` | `std_msgs/String`, latched | Controller 시작마다 생성하는 128비트 challenge |
| `/behavior/control_request` | `std_msgs/UInt8MultiArray` | 명령 C와 정책의 해제·정지 제약 |
| `/planning/control_plan` | `std_msgs/UInt8MultiArray` | 수락한 C의 전체 사본, 실제 궤적 T, 같은 cycle의 수락·경로 결과 |
| `/control/binding_status` | `std_msgs/UInt8MultiArray` | 현재 token, 사용 계획, 실제 Controller 출력과 실행 감시 |

UInt8MultiArray의 layout은 비우고 data_offset은 0으로 한다. 최대 payload는 4MiB다.
payload는 ROS1 little-endian serialization을 사용하며 magic `0x41534d43`,
버전 `uint8(2)`, 종류 `uint8(1=request, 2=plan, 3=status)`로 시작한다.
이후 필드 순서와 형식의 정본은 위 헤더의 `PutRequest`, `Encode`, `Decode`다.
바이너리 내용을 임의로 이어 붙인 별도 producer는 허용하지 않는다. 종류·버전·크기·
layout·끝 길이가 맞지 않는 메시지는 폐기한다. 시험 기록기는 별도 읽기 전용
[Python decoder](../../tools/scen_runner/scen_runner/records/control_binding.py)를 사용한다.

v2는 Plan 끝에 `merge_waiting_path_valid` bool을 추가한다. 기존 ROS `.msg`와
feedback의 `plan_valid` 의미를 바꾸지 않는다. 실제 제어 producer/consumer는 v2만
수락하며 함께 재시작한다. 읽기 전용 Python 기록 분석만 과거 v1도 해석한다.

## 소속 증명과 전이

token은 `(Controller epoch, Behavior epoch, uint64 revision)`이다. 새 Controller와
Behavior 시작 시 각각 새 난수 epoch를 사용한다. revision은 명령의 실행 의미,
목표 revision, 물리 정지선, 해제 자격 또는 제동 단계가 바뀔 때 증가한다.
heartbeat의 header 시각·seq, 유효 기한·설명 갱신은 revision을 바꾸지 않는다.
같은 token으로 실행 의미를 바꾸면 거부한다.

1. 중재기가 C 발급. Controller는 새 token을 수신하고 기존 제동을 유지한다.
2. Planner는 C를 실제 수락하고, 해당 cycle의 입력 소유권을 고정한다.
3. C의 T를 생성하고, C 사본·수락 결과·T를 한 메시지로 발행한다.
4. Controller는 현재 C와 사본의 token 및 모든 실행 의미가 같은지 확인한다.
   feedback의 command ID·실제 active maneuver도 일치해야 한다.
5. 새 유효 경로 또는 현재 기동의 검증된 LOCKED 경로, 입력·명령·경로 신선도,
   변환·참조 경로 검증과 중재기의 `release_authorized`를 만족한 뒤 제동을 해제한다.

수락 피드백만 받거나 최신 차량 시각의 궤적을 받는 것은 4번을 대신하지 못한다.
REUSED/SAFE_STOP, 거부된 요청, 현재 명령과 다른 기동의 결과는 해제에 쓰지 않는다.
정지 완료 경로의 예외는 STOP·COMPLETED·모든 속도 0인 현재 위치 HOLD만 허용한다.
이 결과는 목표 달성이나 이동 허가를 뜻하지 않는다.

회전교차로 MERGE의 먼 PREPARE에서는 전체 이탈 후보가 아직 없더라도, 같은
cycle의 도로·충돌·동역학 검사를 통과하고 진입 대기 경계 전에 완전히 정지하는
접근 경로를 사용할 수 있다. Planner가 이 경로에만 `merge_waiting_path_valid`를
설정한다. Controller는 ROUNDABOUT·PREPARE·PREPARING/WAITING·NEW와 0 속도 끝점을
요구하고 SIGNAL 제약이나 복구 이탈에는 이 예외를 허용하지 않는다. 일반 C–T
연결·신선도·해제 자격 검사는 동일하다. 이 표시는 진입 후보 성공, READY,
EXECUTE 또는 COMMIT을 의미하지 않으며, `feedback.plan_valid`와 `gap_safe`는
기존의 전체 진입 후보 판정을 유지한다. AVOID·LANE_CHANGE·고속도로 MERGE나
임의 fallback에는 적용하지 않는다.

AVOID 준비 중 `plan_valid=false`는 횡방향 회피 후보 미확정을 뜻하며, 같은 cycle의
유효한 감속 접근 경로까지 무효라는 뜻은 아니다. 현재 명령의 AVOID·PREPARING/WAITING,
RESULT_NONE·TRAJECTORY_NEW이고, 끝 속도가 0인 새 유효 경로만 Controller에 연결한다.
Planner는 기존 접근 경로에 공통 jerk 제한 정지 구간을 연결한 뒤 전체 경로를 원래
관측 시각 기준의 장애물·도로 조건으로 재검증한다. 그 접근 구간 뒤에 안전하게
정지할 수 없으면 현재 측정 상태부터 공통 제동 경로를 평가한다. 정지 구간을 포함할 수 없는
짧은 경로는 해제 근거가 되지 않는다. 신호 제약·복구 이탈·별도 제동 요구가 있거나
COMMITTED·SAFE_STOP·실패 경로인 경우 이 예외를 적용하지 않는다. 기존 메시지·전송
필드로 구분하며 READY/COMMIT 상태나 `plan_valid`를 성공으로 바꾸지 않는다.

Plan이 먼저 오면 pending에 보관하고 C가 와야 연결한다. C가 먼저 오면 해당 Plan을
기다린다. 과거 revision·이미 폐기된 Behavior/Planner epoch·역순 계획 sequence는
해제에 쓰지 않는다. 늦은 과거 결과는 현재 경로의 신선도를 갱신하지 않는다.
heartbeat는 연결·최초 응답 대기 시각을 초기화하지 않는다.

통합 runtime은 spinner 시작 전에 native 명령 입력을 단일 owner로 지정하고
공개 요청 subscriber를 닫는다. 공개 토픽은 Controller·기록기용 mirror다.
분리 실행 Planner만 공개 요청을 입력으로 받는다. feedback callback 유무나
새 요청의 수신 시각으로 native 소유권을 추정하지 않는다.

## 정지와 해제의 소유권

SIGNAL(K), 충돌, 보행자, 입력 무효, 실행 실패는 중재기가 독립 유지한다.
`release_authorized=false`의 움직이는 후보는 경로 사전 검증용이며 제동을 풀지 않는다.
일반 비상제동 재출발은 중재기에서 정지·초록 3원본·다른 원인 해제를 확인한다.
검증된 교차로 내부 복구는 해당 K의 신호 요구만 제외하고, 초록·완전 정지를
강제하지 않는다. 두 경우 모두 현재 C–T 연결과 Controller 검사를 통과해야 한다.

정지선 초과 뒤 충돌 영역 전에서 멈춘 차량도 신규 초록 3원본·완전 정지·다른
원인 해제를 만족하면 일반 재진입을 준비할 수 있다. 이전 정지 목표를 달성한
것으로 바꾸지 않는다. 물리선 이전 정지 증명을 다시 만들 수 없는 위치에서는
새 명령의 **전체 이탈 경로**를 검증한 뒤 경로 실패만 owner ACK로 해제한다.
전송의 `recovery_release`는 이 전체 경로·저속 제한도 요구한다. 영역 밖의 이
재진입은 내부 적색 복구와 달리 초록 허가를 계속 요구하며, 검증된 출발 후
움직이기 시작했다는 이유로 다시 정지/재출발을 반복하지 않는다. 위치 이력
공백·신호 철회·새 위험·연결 또는 전체 경로 상실은 진입을 중단한다. 초과·
불확실 기록은 그대로다.

복구 사전 검증이 성공하면 중재기는 동일 K·해제 종류에서 최대 0.5초 동안
해제 명령의 수락·생성을 기다릴 수 있다. 이는 최초 조건부 승인부터의 단조
deadline이며 heartbeat가 연장하지 않는다. 새 위험·K 변경·timeout은 즉시
철회한다. 이 기간에도 Controller는 **새 C 자체의** 현재 이탈 경로 증명과
연결을 받기 전 기존 제동을 유지한다. 이전 사전 검증 경로는 해제에 쓰지 않는다.
이 0.5초는 제동 응답 예산과 별개다.

현재 K에서 검증된 복구의 시작 사실은 차체 전체가 출구를 벗어날 때까지 유지한다.
확실한 내부 점유에서 가능한 점유만 남는 출구 오차 구간으로 이동했다는 이유로
복구를 새 진입으로 바꾸지 않는다. 연속된 위치 이력·현재 경로·최신 위험·다른
원인 해제가 계속 필요하며, 위치 공백·재배치는 이 지속 자격을 폐기한다.

복구 KEEP은 현재 차량에서 차체 뒤끝·오차 여유가 이탈 경계를 넘는 전체 경로를
요구한다. 복구 속도 상한은 기존 저속 생성기의 2m/s 초기값을 사용한다.
Behavior의 현재 복구 명령에도 이 목표 속도를 명시한다. Controller는 현재
측정 속도·양의 가속도와 응답 예산으로 추가 가속 상한을 계산하여, 재계획이
측정 가속도를 반복 주입해 목표보다 계속 빨라지지 않게 한다. 0과 음의 가속
상한도 solver·warm start에 적용한다. 다른 원인의 제동이나 연결 검사를
이 제한으로 완화하지 않는다.
복구 속도 제한의 진입·이탈은 명령 관리자의 별도 이산 정책 전환이다. 동일 KEEP
거래의 속도 고정 때문에 이탈 후에도 2m/s가 남지 않도록 새 command ID를 발급한다.
같은 복구 단계의 heartbeat는 ID나 속도 payload를 변경하지 않는다.
복구는 모든 속도에서 거리 기준 횡경로와 측정 속도·가속도에서 시작하는 공통
jerk 제한 속도 적분으로 전체 출구까지 생성한다. 긴 시간 horizon을 반복 변경하여
초기 가속을 뒤로 미루거나, 0.1m/s 경계의 곡률 계산 전환으로 조향이 불연속인 경로를
쓰지 않는다. 모두 도로·동역학·조향·충돌 검사를 유지하며 짧은 저속
crawl fallback을 전체 이탈 증명으로 인정하지 않는다. 현재 C의 Plan에 담긴
`exit_path_valid`를 Controller에서도 확인한다.

신호 STOP의 물리선·목표 revision·braking_only는 Request 안의 명시적 추가 구조로
전달한다. 기존 target pose의 의미는 그대로다. 제동 중 목표 갱신은 제동을 유지하고,
안전하게 정지한 HOLD는 저장 목표만 바꾸며 현재 위치에 머문다.
Behavior의 횡기동 lifecycle도 실제 신호 정지·복구 전환에 맞춰 정리한다.
이미 COMMITTED/EXECUTING인 횡기동은 먼저 실제 신호 제동으로 인계한 뒤 새 KEEP
경로를 검증한다. 과거 AVOID/차선 변경/MERGE가 이탈 후 되살아나지 않으며,
충돌 비상·FOLLOW의 별도 유지 상태는 이 정리로 해제하지 않는다.

## 실행 응답과 실패

Request는 제동 generation·단계·최초 판단 tD·당시 원본 상태와
`max(0.5s, 실측 최대 tD→tR + 0.1s)` 예산을 전달한다. Controller는 공통 ROS
시계에서 전달 지연을 한 번 계산한 뒤 단조 deadline으로 감시한다. 목표 변경과
heartbeat로 다시 시작하지 않는다. 강화 요청에는 강화된 물리 응답이 필요하다.
일반 STOP은 이미 적용 중인 일반 감속을 계속할 수 있다. 새 원본 상태에서
실제 음의 가속도와 속도 감소를 함께 확인하며, 이전 상태나 시간 경과로 인정하지
않는다. 최대제동 강화는 기존 감속보다 추가로 강해진 반응을 요구한다. 두 모듈은
동일한 `asmc_msgs/braking_response.hpp`의 검출 기준을 사용한다. 이는 응답 확인이며
정지 목표 달성·감속 성능 충분성을 대신하지 않는다. 실측 기록에서는 기존 감속의
유지 확인과 새 제동 응답 시작을 구분한다.
해제하거나 owner가 복구를 확인한 제동 generation은 해당 epoch에서 소비 완료로
기억한다. 오래된 heartbeat가 그 tD를 다시 감시하여 가짜 timeout을 만들지 않는다.
다음 generation이나 새 Behavior epoch의 실제 제동 요청은 새로 감시한다.

상태에는 해당 제동 generation·단계, 실제 응답 검출 여부·남은 지연을 전달한다.
Behavior는 현재 token·단계의 신선한 상태만 정지 계산에 반영한다. 이미 관측한
제동에 지연 전체를 다시 더하지 않는다. 시간 경과만으로 응답을 인정하지 않는다.

응답 timeout(1), 경로 실패(2), 정지 유지 중 움직임(4), 시각/기준 상태 오류(8)는
실패 bitmask로 유지하며 최대제동한다. 초록은 실패를 해제하지 않는다. 경로 실패만
단독으로 남은 경우, 정지·안전한 새 경로·중재기의 명시적 acknowledgment와
Controller의 전체 참조 검증 후 현재 실패 요구를 해제할 수 있다. 과거 사건은
삭제하지 않는다. 응답 timeout 또는 복합 실패는 이 acknowledgment로 해제하지 않는다.

raw Controller의 `execution_recovery_safe`는 후보 증거다. 정책이 신선한 위치·
위험 정보·정지 확인·현재 경로로 자신의 경로 실패를 실제 해제했다고 확인한
뒤에만 Request의 acknowledgment를 발급한다. 다른 정지 원인이 남아 있더라도
이 경로 실패의 확인 자체는 전달할 수 있지만, 최종 제동은 해당 원인을 계속
반영한다. 새 ACK 명령의 경로를 기다리는 동안 같은 경로 실패 상태가 한 번 더
도착해도 승인 거래를 초기화하지 않는다. 현재 ACK의 성공 상태를 받아야 거래가
끝나며, 이후 새 실패나 더 강한 실패는 다시 유지한다. 초록이나 미확인 입력으로
Controller만 먼저 실패를 지워 정책이 영구 정지하는 전이는 허용하지 않는다.

이 계약의 수락 시험은 최신 시각의 이전 C 경로, 전달 순서 변경, 과거 결과·재시작,
heartbeat, 해제 자격 없는 후보, 잘못된 참조와 최초 비상 출력을 포함한다.
실제 제동 성능과 센서·위치 오차는 MORAI 측정 결과로 별도 검증한다.
