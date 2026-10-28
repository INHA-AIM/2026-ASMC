# 회전교차로 MERGE 구현과 검증

2026-10-04 기준 주행 검증용 구현이다. 사용자 요청에 따라 기존 작업 브랜치에서
두 노드의 `roundabout_merge.enabled`를 함께 `true`로 설정했다. 실제 ROS
빌드·launch와 MORAI 폐루프 주행은 이 환경에서 확인하지 않았다.

## 적용 범위와 대기 위치

첫 단계는 고정 global reference를 사용하는 회전교차로 하나다.
입구 링크 `A2256W000148`, 합류 링크 `A2256W000146`, 입구 정지선 노드
`A1256W000709`를 명령과 지도 양쪽에서 확인한다. 회전 차량의 통로는
`A2256W000134`와 `A2256W000151`의 링크별 주행 영역이다.
교차로 전체 포장면을 회전 차량의 통로로 간주하지 않는다.
전체 궤적의 도로 포함 여부는 별도의 기존 HD map grid로 검사한다.
지도나 차량 제원은 변경하지 않았다.

현재 reference에서 정지선은 s=883.054 m다. 기존 차량 footprint와 충돌 여유를
포함해도 후륜을 이 위치에 맞춘 차량은 회전 통로와 겹치지 않고 도로 안에 있다.
차체가 회전 통로와 처음 겹치는 위치는 s=884.954 m로 대기 위치보다 1.9 m 앞이다.
후단 이탈 기준은 s=914.154 m이며, 완료 시에는 실제 현재 footprint가 통로와
겹치지 않는지도 다시 확인한다. 이는 현재 지도/reference 기준 검증이다.
시뮬레이터의 실제 지도 정합성까지 입증하는 결과는 아니다.

## 동작

1. 입구 전부터 MERGE PREPARE로 접근한다. 목표 상한은 25 km/h이며 곡률에 따른
   속도 제한을 함께 적용한다. 접근 검사는 최소 45 m 전부터 시작하고,
   현재 속도가 높으면 편안한 감속거리 기준으로 더 일찍 시작한다.
2. 준비 중 출력은 정지선에서 멈출 수 있는 STOP 접근/대기 궤적이다.
   일반 신호등의 정지 위치 설정과 분리하여 후륜 기준 정지선에 맞춘다.
3. 동일 reference에 네 가지 속도 후보를 생성한다. 정지 상태에서도 출발하고
   후단 이탈까지 최대 10초 안에 연결되는 후보를 검사한다. 도로 포함,
   곡률·횡가속 제한, 전체 관측 객체와의 충돌을 함께 검사한다.
4. 후보의 실제 차체 점유 시각으로 진입과 이탈을 찾는다. 준비 중에는 점유
   구간 전체에 전방 1.5초/후방 2초 여유를 적용한다. 정적 객체와 ID가 바뀐
   객체도 포함하며, 특정 FOLLOW 객체 하나에 판단을 묶지 않는다.
5. 서로 다른 최신 객체 관측 세 번에서 안전한 후보가 확인되고 진입 시각이
   안정적일 때 READY가 된다. EXECUTE 수신 시 현재 후보, 입력 신선도,
   궤적 메시지 변환을 다시 확인한 후에만 commit한다. 입력 지연 또는
   새 객체로 READY가 취소되면 과거 READY 토큰으로 출발할 수 없다.
6. commit 뒤에는 FOLLOW/KEEP 점수 변화로 합류를 중단하지 않는다.
   물리적 충돌 검사는 계속하며 막히면 기존 충돌 제동 경로를 사용한다.
   후단이 통로를 벗어나고 KEEP 경로 생성까지 성공해야 완료한다.
   긴급정지는 기존처럼 선점할 수 있다.

고주로 합류와 차선 변경은 이번 실행 코드로 열지 않는다. 다른 합류 유형,
다른 source/target 조합 또는 지정 gap ID는 지원하지 않는 명령으로 처리한다.
기존 메시지 필드를 사용하므로 `.msg` 변경은 없다.

## 검사 결과

| 검사 | 결과 |
|---|---:|
| 회전교차로 후보 안전 검사 | 16/16 통과 |
| 대기·READY·취소·출발·후단 이탈 검사 | 32/32 통과 |
| Behavior 접근·명령 유지·긴급정지 검사 | 13/13 통과 |
| 기존 FSM command binding 검사 | 8/8 통과 |
| 기존 adapter/transaction/merge 계약 검사 | 290개 통과 |
| 기존 주행 회귀 검사 | 14개 suite 통과 |

실제 core 34개 translation unit를 빌드했다. ROS가 없는 환경이므로 runtime의
메시지 연결은 저장소 `.msg`에서 만든 대체 구조체를 사용한 문법 검사로 확인했다.
`v2_planning`, `feedback_builder`, `frenet_planner_node`,
`behavior_command_adapter`는 통과했다. 실제 catkin 빌드와 serialization,
launch parameter 전달, callback 순서를 검증한 것은 아니다.

## 기존 세 bag의 최종 코드 재생

아래 세 bag 수치는 첫 MERGE 구현(7fb6393)의 준비 단계 결과이며,
후속 보완 이후의 성공률이나 현재 후보 개수로 해석하지 않는다.
객체는 각 계획 시점 이전에 수신된 최신 관측을 사용했다. 같은 객체 timestamp를
여러 계획 사이클에서 재사용해도 READY 확인 횟수가 늘어나지 않는다.
재생은 PREPARE/READY 평가만 수행하고 commit하거나 ROS 명령을 발행하지 않는다.

| 기록 | 평가 사이클 | 접근/대기 궤적 생성 | 전체 통과 후보 존재 | READY |
|---|---:|---:|---:|---:|
| 221114 | 127 | 122 | 60 | 26 |
| 221649 | 116 | 111 | 27 | 14 |
| 230215 | 110 | 100 | 46 | 14 |

후보가 없을 때 READY를 내지 않았고 세 기록 모두 안전 후보가 확인되는 구간에서
READY가 나왔다. 접근/대기 경로도 생성하지 못한 사이클은 각각 5/5/10개로,
runtime에서는 안전정지 출력 대상이다. 이 수치를 숨기거나 완주 성공률로
해석하지 않는다. 기록 속 ego는 기존 KEEP/FOLLOW 동작으로 이동했기 때문에
새 MERGE의 실제 감속·대기·출발 후 관측은 재현되지 않는다.

재현 도구는 `src/asmc_planner/tools/frenet/roundabout_execution_shadow.cpp`다.
외부 decoded bag을 `extract_merge_shadow_frames.py`로 변환한 frame JSON과
경로가 해석된 parameter YAML을 입력한다.

```sh
python3 extract_merge_shadow_frames.py decoded_bag.json frames.json
roundabout_execution_shadow frames.json resolved_params.yaml > shadow.log
```

원본 bag, 관측 frame JSON과 상세 객체 좌표 로그는 저장소에 포함하지 않는다.

## 실제 적용 전 남은 확인

실제 ROS 환경에서 pull 후 catkin 빌드하고 노드를 재시작해야 한다.
`src/asmc_planner/config/frenet/params.yaml`과
`src/asmc_planner/config/behavior/behavior_params.yaml`의
`roundabout_merge.enabled`는 모두 켜져 있다. 다른 parameter 파일을 사용하는
launch라면 해당 파일에도 동일 설정이 필요하다. 한 노드만 켜는 방식은 사용하지 않는다.

첫 신규 주행에서는 정지선 대기 위치, WAIT→READY→COMMITTED→EXECUTING→COMPLETED,
앞차/후방 회전 차량에 따른 대기 및 출발, 후단 이탈까지의 지속 실행을 기록한다.
이 신규 bag이 있어야 실제 합류와 완주 성공을 판단할 수 있다.

## 첫 실제 주행 이후 보완

- 대시보드 행동 이름 목록에 MERGE를 추가했다. 알 수 없는 행동 값을 WAITING으로
  표시하던 문제이며, 실제 FSM PREPARE/EXECUTE 전이와는 별개다.
- 출발 확정은 입구 정지선 1 m 이내에서 최신 관측 세 번으로 확인한다.
  원거리 안전 후보가 있어도 먼저 commit하여 입구 이전에 다시 제동하지 않는다.
- 속도 preview는 실제 횡방향 후보의 곡률을 사용하고 jerk 응답거리를 포함한다.
  정상 연속 후보가 모두 운동학적으로 탈락하면 양의 가속도 기억을 해제한
  후보를 추가한다. 가속도 크기·곡률·횡가속도 제한은 그대로 유지한다.
  저속 제동이 0속도에 도달하는 것을 역주행 후보로 탈락시키지 않는다.
- 합류 중 옆 차량의 안전 여유와 이미 겹친 경우에는 기존 clearance recovery를
  적용한다. 실제 차체 충돌, 다른 객체 충돌 또는 다시 가까워지는 궤적은 금지한다.
  진입 전 준비 단계의 gap 검사는 완화하지 않는다.
- 도로 안에 있는 서로 다른 복귀 거리/소폭 횡방향 후보를 먼저 찾고, 실선 접촉
  시간을 비용에 반영한다. 실행 중 선택한 횡방향 목표를 다음 계획에서도 우선한다.
- 정상 후보가 전부 막힌 committed MERGE에 한해 작은 raster 경계 접촉을
  복구할 수 있다. 최대 깊이 0.15 m·footprint 10%·실제 이동 6 m·5초 이내에
  원래 grid로 돌아와 여유를 둔 차체가 0.5 m 이상 이동해야 한다.
  예산은 실제 경계 접촉부터 한 transaction 안에서 한 번만 사용하며 재계획으로
  갱신하지 않는다. 지도 자체를 넓히거나 충돌 검사를 생략하지 않는다.
  기존 AVOID 경계 정책은 변경하지 않았다.
- 후보를 대신한 제동 출력은 피드백에도 SAFE_STOP과 collision braking으로
  표시한다. 실행 상태는 유지하여 안전 후보가 회복되면 다시 진행한다.

신규 bag의 고정 ego/관측으로 후보 탈락을 재현하고 수정 후 경로 생성을 비교했다.
실제 차량 접근 중 필요한 제동과 정지 후 불필요한 margin 고착을 구분했으며,
출구에서의 경로 복구도 확인했다. 과거 ego를 정지 상태로 계속 고정하면 복구
시간 예산이 소진되는 것이 정상이다. 이를 새 제어의 완주 성공률로 해석하지 않는다.


## 대기 자세 경계 복귀와 접근 중심선 추종

회전교차로 대기 STOP은 일반 STOP의 횡방향 탐색과 저속 위치 유지 대신
중심선 복귀를 목표로 사용한다. 긴 접근 중 정지 후보가 없어 현재 횡위치를
유지하는 대체 제동 경로를 만들면, 같은 종방향 정지 일정과 실제 초기 방향을
사용하는 공간 기반 중심선 복귀 후보를 추가한다. 곡률·횡가속도·종가속도,
원래 지도 주행영역과 전체 객체 충돌 검사를 통과한 경우에만 교체한다.
거리가 짧거나 복귀 후보가 안전하지 않으면 기존 정지 출력을 유지한다.

진입 전에는 정지선 1 m 이내에서 이미 조금 벗어난 차체만 경계 복귀를 포함한
전체 통과 후보를 인증할 수 있다. 깊이 15 cm·차체 10%·이동 6 m·5초 제한과
원래 영역으로의 안정된 복귀 조건은 유지한다. 이미 영역 안에 있는 대기 자세가
새로운 영역 이탈을 시작하는 용도로 이 예외를 사용할 수는 없다.
PREPARE 출력은 계속 접근·정지 궤적이며, 전체 차체 충돌과 앞뒤 시간 간격 검사,
최신 관측 세 번의 확인 후에만 진입한다. 정지해서 기다리는 시간은 이동 복구
예산으로 계산하지 않는다. commit 후에는 기존의 실제 이동·시간 예산이 적용된다.

후보 검사 16개와 실행 검사 32개가 통과했다. 공개 지도에서 만든 합성 자세로
진입 전 경계 복귀, 점유 차량 차단, 큰 영역 이탈 거부, 중심선 정지 및 대체
제동의 초기 방향 연속성을 검사했다. 기존 KEEP·STOP·FOLLOW·AVOID와
속도·곡률 회귀 검사도 통과했다. 추가 bag 두 개의 고정 입력 재생에서 READY가
다시 나오는 것을 확인했으며, 이를 새 제어의 완주 결과로 해석하지 않는다.
실제 ROS/catkin 빌드와 새로운 MORAI 주행 검증은 남아 있다.

## 출구 감속과 실행 중 조향 가능한 제동

회전교차로 종방향 생성은 KEEP에서 사용하는 공간 속도 envelope 적분기를
사용한다. 실제 횡방향 후보의 물리적 호 길이와 곡률로 속도 상한을 만들고,
상한의 감소 기울기를 가속도에 반영한다. 곡선에 들어가 속도 오차가 생긴
뒤에만 감속하던 응답을 줄인다. 곡률·횡가속도 제한은 유지한다.
거의 정지한 상태에서 측정 감속도가 jerk 제한 정지 경계를 벗어나면 초기
감속 기억을 정지 가능한 경계까지 완화하여 역주행 후보가 되지 않게 한다.

이미 진입한 MERGE에서 전체 통과 후보가 탈락하면, 같은 기준 경로를 따라
정지하는 다점 제동 후보를 추가한다. 일반 주행 가속도 한도보다 큰 측정
감속도도 설정된 서비스 제동 한도 내에서 처리한다. 원래 지도 또는 기존
제한된 경계 복귀 정책과 전체 객체 충돌 검사를 통과한 경우에만 사용한다.
새 후보가 없으면 기존 충돌 검사된 제동 후보도 시도한다. 모든 안전 후보가
없는 경우까지 조향 경로를 보장하지는 않는다. SAFE_STOP 피드백과 실행
transaction 유지 정책은 그대로다.

후보 검사 16개와 실행 검사 36개, 기존 회귀 검사 16종이 통과했다.
새 실행 검사는 출구 전 감속, 생성 경로의 첫 구간을 반복 추종한 이상적인
출구 통과와 KEEP 전환, 강한 측정 제동 중 다점 정지 경로, 점유 객체의
물리적 충돌 검사를 확인한다. 실제 MPC/차량 응답을 포함한 폐루프 검사는
아니다. 기록된 출구 고속 자세 일부에서도 다점 제동 경로가 나왔지만,
이미 큰 영역 이탈이 발생한 모든 과거 자세를 복구한다고 해석하지 않는다.
ROS runtime 소스는 메시지 stub으로 문법 검사했으며 catkin 빌드는 별도다.

회전 차량 예측은 이 변경에 포함하지 않는다. OBB 축 방향과 실제 이동
방향을 분리하고, 새 관측의 timestamp로 회전율을 추정한 뒤 미래 실제
관측 위치와 예측 위치를 비교하는 별도 검증이 필요하다.

## 정상 통과 중 제동 해제 상태 유지

committed 회전교차로 MERGE에서만 직전에 안전 검사까지 통과한 통과 경로의
가속도 배열을 보관한다. 새 계획의 실제 경과 시간으로 가속도를 보간하여,
측정 감속이 반복되어도 제동 해제의 jerk 진행을 매번 처음으로 되돌리지 않는다.
현재 위치·방향·속도는 측정값이며, 새 곡률 속도 envelope와 도로·전체 충돌
검사를 다시 수행한다. 현재 속도가 후보 상한과 공간 상한보다 충분히 낮을
때만 이 이력을 사용한다. 감속이 필요한 상한에서는 측정 감속을 유지한다.

첫 계획, 중복/역행 시간, 0.5초보다 오래된 이력은 측정 가속도를 사용한다.
전체 통과 후보 탈락, invalid command, transaction reset 및 KEEP handoff에서
이력을 지운다. 안전 제동 후보는 항상 측정 가속도에서 생성한다.
정상 통과의 선택 경로 한 개만 저장하며 후보 수와 충돌 검사 horizon은 늘리지 않는다.

다른 모드도 검토했다. KEEP은 SpeedProfiler 상태 유지와
ResolveEnvelopeBoundaryAcceleration의 가속 의도 보정이 이미 있다.
STOP과 FOLLOW는 정지선/리더에 대한 별도 종방향 생성기를 사용한다.
지도 기반 AVOID는 양의 가속도 재주입을 막는 이력이 있지만 음의 측정 가속도
해제 정책은 MERGE와 같지 않다. 같은 증상의 가능성을 배제하지 않으며,
이번 bag에는 그 구간 재현 근거가 없어 이 수정으로 일괄 변경하지 않는다.

실행 검사는 45개로 늘었다. 지속적인 음의 측정값 하에서도 제동 해제가
진행되고, 새 물리적 장애물은 계속 차단하며, 차단 이후 측정 제동으로 복귀하는
합성 검사를 추가했다. 두 신규 bag의 고정 입력 재생에서도 내부 저속 자세의
불필요한 추가 감속이 줄었다. 기록된 ego는 새 제어의 응답이 아니므로 실제
정지 제거와 완주 성공은 신규 MORAI 주행으로 확인해야 한다.
회전 차량 예측 및 그로 인한 안전 차단은 이번 변경에 포함하지 않는다.

## 차체 경계를 고려한 정지 방향

PREPARE의 정지 좌표와 종방향 속도·가속도 배열은 유지하고, 정지 위치에서
차체 전체가 지도 주행 영역과 보호 차선에 닿지 않는 방향을 탐색한다.
중심선 방향의 ±20도 범위에서 2도 간격으로 검사하며, 차체를 같은 중심에서
팽창시켜 확인한 여유가 큰 방향을 선택한다. 지도 래스터 해상도와 0.05m
탐색 간격 때문에 이는 정확한 실측 차선 거리로 해석하지 않는다.
팽창 여유는 이진 탐색으로 검사하여 방향당 지도 검사 횟수를 제한한다.

정지점까지의 공간 quintic으로 현재 위치·방향에서 선택된 대기 방향까지
연결한다. 시간 미분만으로는 속도 0에서 목표 방향을 표현할 수 없으므로
거리 기준 횡방향 기울기를 사용한다. 현재 방향을 순간 변경하거나 정차 중
회전시키지 않는다. 정지점에 도달하는 다점 정지 궤적만 대상으로 하며,
접근 전체의 곡률·횡가속도·도로 영역·차선 접촉 시간과 모든 예측 객체의
충돌 검사를 다시 통과해야 한다. 너무 짧거나 안전한 연결이 없으면 원래
검사된 정지 궤적을 유지한다. 통과 후보, READY/commit 조건은 바꾸지 않는다.

후보 검사 16개와 실행 검사 50개가 통과했다. 정지 위치 보존, 시작 방향
연속성, 전체 차체와 접근의 지도 검사, 목표 방향 변경 시 기존 종방향
배열 보존, 정차 중 방향 유지 검사를 추가했다. 기존 세 bag의 고정 입력
재생으로 정지 위치 보존과 합류 판정 유지, 선택된 대기 자세의 지도 경계
검사를 비교했다. 기록된 ego의 실제 주행은 새 궤적을 추종한 결과가 아니다.
MPC 응답을 포함한 접근 중 선 접촉과 최종 정차 방향은 신규 MORAI 주행에서
확인해야 하며, 이 변경이 과거 모든 접근 자세를 복구한다는 의미는 아니다.

## 접근 한계와 불필요한 재정지 분리

대기 중 후륜 정지 목표는 지도 정지선 자체다. 기존 STOP buffer 보상은
종방향 stopping 생성기의 목표를 정지선 뒤로 보내므로 여기서는 사용하지
않는다. 2~8초의 13개 제한된 종료시간 후보로 정지선에서 속도·가속도 0이
되는 완전한 접근을 생성한다. 현재 방향을 보존하는 공간 횡방향 곡선과
지도 기반 대기 방향 정렬을 결합하고, 종방향 역행/정지선 초과/Frenet jerk,
Cartesian 속도·가감속·곡률·횡가속도, 전체 지도와 모든 객체 충돌을 검사한다.
합류 통과 후보의 불합격 자체는 안전한 정지선 접근을 금지하지 않는다.

새 접근이 불가능하면 기존 STOP을 시도하되, 모든 전달 객체로 재검사하고
정지선보다 뒤로 향하는 경로를 거절한다. 이미 정지선을 지난 실제 자세를
뒤로 되돌리지는 않으며 그 자세보다 더 진행하는 대기 경로도 거절한다.
투영 수치 허용치는 0.02m다. 지도와 충돌 검사를 완화하거나 차량을 제외하는
정책은 아니다. 접근 자체가 위험하거나 안전한 제동 연결이 없으면 기존
SAFE_STOP을 유지하므로 위험 예측에 의한 조기 정지는 여전히 가능하다.
정지선까지의 도달 경로가 탈락하면 기존 committed 서비스 제동 생성기로
정지선 이전의 조향 가능한 정지를 두 형태까지 시도한다. 서비스 감속 한도와
지도·모든 객체 검사를 유지하며 이 경로도 정지선을 넘지 않아야 한다.

정지선 1m 이내의 기존 READY 거리 제한을 차체 길이 이내로 바꾼다.
이를 통해 대기 위치 부근에서 안전한 rolling 진입 또는 한 번 정지한 뒤
재출발을 허용한다. 먼 접근 구간에서는 계속 대기 가능한 접근을 계획하며
미리 commit하지 않는다. 정지선 1m보다 앞에서 출발하려면 기존 회전 예측과
회전율 0의 속도 예측 모두에서 전체 통과/앞뒤 시간 여유 검사를 통과해야
한다. 기존 fresh observation 연속 확인, 동일 후보 재검사, stale/점유 객체의
READY 취소와 원자적 EXECUTE commit은 유지한다. 속도 0이라는 별도 조건이나
한 번 정지했다는 이유만으로 출발을 허가하지 않는다.

대기 구간(정지선 앞 차체 길이 이내)에서 실제 속도 0.1m/s 이하로 멈추면
대기를 latch한다. 전체 통과가 허용될 때까지 현재 자세를 HOLD하고 정지선까지
다시 전진해 정지하지 않는다. reset/commit에서 latch를 지운다. 대기 구간보다
먼 곳에서는 이 latch를 적용하지 않아 제한된 통과 horizon으로 인한 교착을
피한다.

commit 이후에는 정지선 대기 절차를 반복하지 않고 실제 현재 자세에서 통과
경로를 재계획한다. 새 위험이 발견되면 기존 안전 제동을 유지한다. 따라서
불필요한 두 번째 정지를 제거하는 정책이며 정지 횟수를 강제로 제한하지 않는다.
센서와 예측으로 위험이 전혀 없음을 절대적으로 보장하는 기능은 아니다.

후보 16개, 실행 56개, 기존 회귀 검사 16종 및 runtime stub 문법 검사가
통과했다. 정지선 초과 없는 접근, 측정 속도가 양수인 진입 허가, 새 차량에
의한 이전 허가 취소, 정지선 앞에서 한 번 대기한 뒤 이상적인 prefix 추종으로
추가 의무 정지 없이 KEEP까지 통과하는 검사를 포함한다. 세 bag의 고정 입력
전후 재생은 정책 적용과 유지되는 안전 차단을 확인하며, 실제 정지 횟수와
주행 성공은 신규 MORAI 폐루프 주행으로 확인해야 한다.

### 부분 차량 관측과 회전 예측의 불확실성

회전교차로에서는 별도 `RoundaboutPrediction` 이력을 사용한다. KEEP/FOLLOW/
AVOID 및 고주로의 기존 객체 예측에는 적용하지 않는다. 센서 관측 timestamp가
증가할 때만 갱신하고, 같은 관측의 반복 재계획은 회전율과 신뢰도를 바꾸지
않는다. 시간을 역행하면 이력을 지우며, 동일 ID의 관측이 0.5초보다 오래 끊기면
새 이력으로 시작한다. reset은 합류 transaction reset과 함께 수행한다.

회전율은 OBB heading 차분 대신 map-frame 속도 `atan2(vy,vx)`를 unwrap한 뒤
실제 관측 시각에 대한 기울기로 계산한다. 객체당 최근 0.8초/최대 8개 표본만
사용하고 기존 yaw-rate clamp를 유지한다. 정지 속도는 회전율 0으로 다룬다.
충돌 형상은 여전히 perception의 authoritative corners이며 크기/중심/yaw로
차체를 다시 만들거나 차량 제원을 축소하지 않는다.

출발 허가에는 가까운 이동 차량의 관측 신뢰도도 필요하다. 최소 4개 fresh
표본과 0.25초 이력, 방향 fit 잔차 0.20rad 이하, 속도 범위 2m/s 이하,
기존 clamp 이내의 회전율을 요구한다. 길이 3m/폭 1.2m보다 작은 부분 형상이나
최근 길이 변화 1m/폭 변화 0.6m를 넘는 차량은 확정된 차량 형상으로 취급하지
않는다. 이는 인식 결과를 교정하는 기능이 아니라 출발 허가의 신뢰도 조건이다.
불확실한 이동 차량이 지도 conflict-start의 `12m + 속도 × 2초` 범위에 있으면
READY를 취소한다. 작은 정적 시설물 모두를 차량으로 분류하지는 않는다.
이동 표시나 속도가 있었던 객체의 단발 0속도 분류/관측 누락은 1초 동안
그 이력을 유지해 즉시 허가가 풀리지 않게 한다. 1초 이후의 장기 누락까지
안전을 보장하는 추적 기능은 아니며 perception 개선을 대체하지 않는다.

무정지 진입은 측정 가속도 -0.5m/s² 미만의 감속 중에는 허가하지 않는다.
정지한 차량에는 이 감속 제한을 적용하지 않지만 나머지 안전 조건은 같다.
관측 age 0.25초 이내와 0.05초 이내의 timestamp 오차, 기존 fresh 연속 확인/
도착 시간 일관성/전체 통과 조건을 요구한다. PREPARE 출발 후보는 명목 회전
예측 외에 직선 속도 예측과 회전율 ±0.15rad/s 예측에서도 기존 충돌 및 앞뒤
시간 여유 검사를 통과해야 한다. 따라서 한 번 대기했다는 이유로 다음 차량을
무시하지 않고, 차량이 없고 안정적인 접근이면 정지 없이 출발할 수 있다.
commit 이후에도 새 관측 기반 회전율과 기존 물리 충돌/안전 제동을 유지한다.

이력 갱신 비용은 객체당 최대 8표본이며 출발 후보 추가 검사는 최대 3회다.
장기 누락 이력은 1초 후 제거한다. 기존 차선 계획이나 재발행 주기를 변경하지
않는다. 후보/실행/예측 synthetic 검사 91개, 기존 회귀 검사 16종과 runtime
stub 문법 검사를 수행한다. 기록 입력 재생에서는 불안정한 조기 진입 허가를
거절하고, 대기 자세를 고정한 입력에서도 fresh clearance 후 READY를 회복하는지
확인한다. 고정 입력의 대기 자세 가정은 실제 주변차량/라이다 피드백이 달라지는
폐루프가 아니므로 실패 주행의 완주 개선과 실시간 주기는 MORAI에서 재검증한다.
기존 성공 기록의 commit을 강제 유지한 재생에서는 출구 곡선의 제동 fallback이
한 프레임 추가로 탈락했다. 같은 입력에서 정상 통과 후보는 수정 전후 모두
생성되지 않았고, 새로운 회전 예측에서 후행 차량의 미래 안전 여유와 제동
경로가 겹쳐 거절됐다. 그 검사를 완화하지 않는다. 출발 시점이 달라진 실제
주행의 출구 정지 여부는 고정 입력에서 확정할 수 없으므로 신규 주행에서 확인한다.

### Exit continuity and departure confidence

Departure continues to check every observed object with both the 1.5 s front
and 2.0 s rear time buffers. A reliably receding vehicle can stop blocking the
separate confidence gate only when a conservative 6 m by 3 m body remains outside
the conflict corridor for two seconds under straight and bounded-turn models.
This does not remove it from candidate collision checks. Vehicle-sized UNKNOWN
zero-velocity observations within 25 m of the conflict and within 2 m of its lane
polygon with authoritative perception geometry block certification until their
classification/motion is established. Small fixtures and distant roadside objects
do not acquire vehicle semantics. The observation guard never expands drivable
pavement.

A small existing lateral-acceleration excess (at most 0.15 m/s²) may retain a
MERGE path only while the excess decreases under braking and disappears before
0.5 s. Curvature, road, collision, and future lateral limits remain binding.
Larger excesses cannot certify traversal. This is recovery of the observed state,
not a global increase of the 2.6 m/s² limit.

On successful MERGE completion, KEEP retains the sensor-timestamp velocity
history for at most 5 s or 20 m beyond rear clearance. Other maneuvers and
cancellation discard it. Runtime feeds the actual object timestamp through
ObserveRoundaboutExit; repeated observations never fabricate turn samples.
Validated planner STOP clears MPC optimization and acceleration memory while
retaining the steering actually published as the next rate-limit reference.
Invalid input/watchdog/emergency failure still performs the full reset.

Validation uses public-map synthetic candidate/execution/prediction cases,
controller restart rate-limit tests, existing planner/controller regressions,
and fixed-input replay of four runs. Replay confirms recovery at the third and
fourth runs' initial lateral-limit failures and removal of the first run's KEEP
prediction-handoff blockage. Two isolated first-run exit states still reject
paths on map containment; those road checks remain active. Historical ego poses
are fixed inputs, so replay does not prove closed-loop MORAI performance or
retimed interaction with following vehicles.

### Bounded body memory for partial observations

Departure confidence no longer requires the observed vehicle width to remain
constant. Reliable motion retains the maximum full-body bounds from up to eight
observations in the preceding second. Partial observations do not refresh that
one-second deadline; a sensor gap over 0.5 s resets both motion and body history.
Invalid geometry, inconsistent fragment position, and partial-only new tracks
cannot obtain a full-body certificate.

The remembered body centre is propagated with measured velocity rather than
following a shifting fragment centre. Bounds grow by 0.25 m plus 0.5 m per second
of body age. These are supplemental planning rectangles: authoritative detected
corners remain unchanged. Before departure, original objects and supplemental
bodies must pass the existing straight and two perturbed-turn collision checks,
including the existing front/rear time buffers. A remembered body therefore
cannot bypass collision checks merely by restoring observation confidence.

Validation for this change: 27 prediction cases and 27 candidate cases passed,
including a narrow fragment that clears the route while its remembered body
correctly rejects departure. Candidate checks use the production collision and
gap implementation; the standalone build excludes only the YAML map loader.
The changed execution translation unit passed a syntax check. Full ROS/catkin
and historical bag replay were not rerun for this change: the prior workspace
and bag copies were unavailable, and the uploaded bag could not be recovered.

### Slower waiting approaches before braking fallback

If all ordinary frontier-stop candidates fail, waiting approach generation
tries bounded two-stage profiles: decelerate to 0.5, 1 or 2 m/s, then stop at
the original map frontier. The second stage cannot accelerate above the chosen
rolling speed. If those full arrivals also fail, a rolling approach with its
own complete braking tail may end before the frontier. Among safe partial
approaches, forward progress is preferred. These are checked whole trajectories,
not unchecked coasting beyond a truncated horizon.

Additional search occurs only after the existing frontier candidates fail,
with at most 39 full arrivals and 24 partial approaches within the existing
horizon, capped at ten seconds. Every candidate retains initial boundary
conditions and the existing monotonic station, jerk, acceleration, curvature,
lateral acceleration, map and authoritative-object collision checks. Failed
search retains the existing STOP/braking fallback. Entry confirmation, waiting
pose latch, committed traversal and the latest parameter files are unchanged.

Validation includes a public-map crossing fixture where slower arrival reaches
the frontier after traffic clears, initial speed and final stop preservation,
collision checks, frontier containment, unchanged entry gating and rejection
of a physical blocker. Fixed-input comparison of 300 recorded approach states
recovered one previously failed output with no newly failed outputs or READY
changes. It does not establish closed-loop elimination of the observed stops;
many states remain blocked by the existing collision predictions. Diagnostic
copies and recorded inputs are not committed.

## PREPARE braking with disputed object velocity

Approach prediction now cross-checks reported object velocity against a bounded
least-squares fit of fresh observed centres (at most eight samples/0.8 s). It
requires at least four samples over 0.25 s, 0.5 m of motion and centre residuals
within 0.5 m. Only a position-supported velocity substantially faster than the
reported velocity is treated as a disputed velocity decay. Its comparison band
comes from the position-fit residual and time span, with a 0.5 m/s floor; it is
not a probabilistic safety guarantee. A slower fragment-centre fit does not
replace the faster sensor hypothesis.

A disputed fit cannot certify entry or a passed vehicle. During PREPARE,
collision checks include both position-supported translation and the original
sensor velocity/turn for at most the observed history span, capped at 0.5 s,
followed continuously by the position-supported velocity. Both copies retain
the original OBB corners and existing collision margins. Consistent motion,
committed traversal prediction, perception topics and configured gaps remain
unchanged. Internal prediction fields default to the existing constant-turn
model and are not ROS message fields.

If frontier and slow-approach candidates fail, direct stopping durations are
also searched before the strongest service-brake fallback. They retain the
current pose/speed, remain behind the frontier, check jerk/acceleration/curvature,
road and all object hypotheses, and include a stationary tail to the same
maximum horizon. Near standstill, the non-reversing acceleration boundary and
its exact jerk-release duration are included rather than relying only on a
half-second grid. A failed collision check still cannot authorize departure.

Standalone regression: 36 core suites / 938 assertions pass, including real
circular motion, disputed velocity, fragment-centre jumps, stationary objects,
near-term prediction continuity, blocked-frontier braking and non-reversing
near-standstill stopping. ROS-generated message/transport tests and a full
catkin build are not available in this analysis environment.

Fixed-input comparisons with original objects, observation timestamps and
measured initial state:

| Recording | MERGE inputs | Previous failures | New failures | Newly failing inputs |
| --- | ---: | ---: | ---: | ---: |
| 20261010-163557 | 148 | 19 | 18 | 0 |
| 20261010-164058 | 186 | 11 | 4 | 0 |
| 20261010-155213 | 520 | 32 | 30 | 0 |

The two latest recordings both produce valid waiting outputs for every input
when objects are removed as a diagnostic only. No newly READY input is observed
in the three comparisons. Residual failures with objects remain; this change
does not establish closed-loop elimination of abrupt stops, and the separate
pre-stop steering oscillation has not been changed. Simulator driving must
still verify the resulting stop approach and departure timing.

## 2026-10-10: 일반 STOP 계산을 재사용하는 MERGE 대기 제동

PREPARE 대기는 일반 STOP의 `GenerateSpatialStoppingCandidates`를 먼저 호출한다.
공통 생성기와 일반 STOP의 기존 호출 경로는 수정하지 않았다. 현재 속도·가속도와
남은 경계 거리로 하나의 제동 프로파일을 정하므로, horizon 선택 때문에 제동이
반복해서 뒤로 밀리는 것을 줄인다. 후륜 기준 대기 위치는 그대로이며 일반 STOP의
stop buffer나 장애물 제외 정책을 복사하지 않는다. MERGE의 지도·차체·곡률·횡가속·
jerk·전체 객체 충돌 검사와 정지 방향 정렬을 통과한 완전 정지 궤적만 사용한다.
공유 프로파일이 거절되면 기존 느린 접근/직접 정지/안전 정지 경로로 돌아간다.
현재 PREPARE 제동 경계에는 실측 Frenet 가속도를 공급하여 일반 주행용 초기 경계
클램프가 이미 이뤄진 서비스 제동을 버리지 않게 한다. 가속도·비역주행 경계는
공유 생성기의 한계 및 기존 서비스 제동 한계로 제한한다.

제어기는 기존 일반 STOP 응답 보정을 회전교차로 `MERGE + PREPARE`에만 재사용한다.
일반 STOP / 회전교차로 대기 / 비적용의 범위를 구분하며 전환 때 추정기를 초기화한다.
EXECUTE·CANCEL·고주로 MERGE·KEEP·FOLLOW·AVOID 및 고정 안전 제동은 보정 범위에
들어가지 않는다. 일반 STOP의 보정 계산과 파라미터, MPC solver는 변경하지 않았다.
이 보정에는 기존 양의 가속도 상한도 포함되므로 대기 위치 재접근의 급출발을 제한한다.

동일 관측 입력과 객체를 유지한 PREPARE 재생 실패는 145900:24→24,
155213:30→25, 163557:18→18, 164058:4→4, 200113:23→21이었다. 새 실패와 새 READY는
없었다. 200113의 최초 실패 입력(164.131초)은 경계까지 정렬된 다점 정지 궤적을
생성했다. 고정 입력에는 이전 실행에서 누적된 속도·방향 오차가 그대로 남기 때문에
폐루프 개선을 보장하는 결과는 아니다. 객체와 경계로 막힌 실패는 여전히 남는다.
일반 STOP 세 bag의 306개 입력에서 비교한 계획 출력은 실행 시간을 제외하고 동일했다.
standalone core 36개 suite, 추가 대기 정렬 및 제어 적용 범위/응답 시험으로 회귀를
검증했다. 전체 catkin build 및 실제 MORAI 주행은 이 환경에서 실행하지 못했다.

## 2026-10-10: STOP 횡방향 계산을 공유하는 MERGE 대기 정렬

일반 STOP의 중심선/현재 횡위치 두 종단 목표와 전체/짧은 정렬 거리 계산을
`Generator/StoppingLateralGeometry.hpp`로 추출했다. 일반 STOP의 후보 순서,
다항식 경계값, 선택 방식과 파라미터는 유지한다. 저속 횡위치 유지 조건도 같은
함수로 공유한다. `lateral_hold_speed=1.0m/s`, `lateral_error_tolerance=0.25m`를
늘려야 한다는 일관된 bag 근거가 없어 두 값은 변경하지 않았다.

MERGE PREPARE는 최대 네 가지 횡방향 형상을 각 제동 프로파일에 결합한다.
시작 횡방향 2차 미분은 실제 곡률에서 변환된 값을 사용하고, 최종 지도 기반
방향 정렬에서도 그 값을 유지한다. 저속이며 허용 횡오차 안이면 현재 d를
목표로 우선하며, 안전 검사를 통과한 후보는 곡률 제곱의 거리 적분과 곡률
변화량으로 순위를 매긴다. 허용 범위 안의 작은 횡오차에는 약한 비용을,
범위 밖에는 더 큰 중심선 복귀 비용을 둔다. 후보를 평가하는 동안 같은
방향 정렬을 반복하지 않고 선택한 출력에 기존 지도 방향 정렬을 적용한다.
큰 횡오차에서의 중심선 복귀와 작은 횡오차에서의 조향 연속성을 함께 검증했다.

기존 후륜 대기 경계, 정지 유지 latch, 회전 객체 예측, 진입 gap/신뢰도/READY
계약과 충돌·주행영역·곡률·횡가속·종가속·jerk 한계는 변경하지 않았다.
일반 STOP 및 KEEP/FOLLOW/AVOID, MERGE EXECUTE, MPC 설정도 변경하지 않았다.

검증: standalone core 36 suites / 946 assertions 통과. 일반 STOP 세 bag의
306개 입력에서 결과 성공 여부와 전체 x/y/yaw/kappa/v/a 배열 해시가 동일했다.
MERGE 여섯 bag 1,466개 고정 입력에서 새 실패와 새 READY는 없었다. 기록된
마지막 오차를 그대로 재사용하므로 기존 실패 100개는 유지되며 폐루프 해결을
주장하지 않는다. 212224 bag 162.015초 입력에서는 최대 곡률 0.16855→0.06776/m,
0.5초 뒤 계획 헤딩 89.25→82.66도로 바뀌어 중심선 복귀를 위한 역방향 굴곡을
줄였다. 실제 정지 헤딩과 잔여 안전 정지는 MORAI 재주행 검증이 필요하다.
전체 ROS/catkin build와 센서-제어 폐루프 시험은 이 환경에서 실행하지 못했다.

## 2026-10-10: integrated main ownership port

Port the validated `549e458` behavior onto main `835d56e` and its single
`asmc_planner` production owner. Removed legacy packages stay removed.
Frenet retains geometry and safety calculations; phase and completion updates
use the existing Execution owner. The bound-state regression also verifies
that an independent execution snapshot cannot reset the production owner.
Measured vehicle curvature is connected in `Parser/PlanningInput.cpp`; a
certified braking path from a blocked AVOID transaction is forwarded by
`Planning/V2Cycle.hpp` with the blocked feedback retained.

Main's mode detection, normal/jamming command selector, launch wiring, sensor
producers, message definitions and HD-map assets remain unchanged. Preserve
normal controller output on `/ctrl_cmd/normal`; the selector remains the
final `/ctrl_cmd` publisher. The only Frenet YAML changes from main are the
validated 0.45 m physical clearance and ordinary STOP reapproach speed/accel.

Validation: 36 standalone core suites / 955 assertions and 63 controller
response/acceleration-limit assertions pass. Ordinary STOP preserves full
x/y/yaw/kappa/v/a hashes across 306 recorded inputs. Roundabout MERGE preserves
full path hashes, candidate validity, READY, completion and failure results
across 1,764 inputs from eight bags, including the bound production Execution
owner. Local HD-map blobs match main exactly. Full catkin/ROS build and
closed-loop MORAI driving of this integrated revision remain unexecuted in
this environment. No claim of new safety clearance or performance improvement
is made by the structural port.

Two AVOID bag histories (474 inputs until completion) also retain candidate
validity, blocker, target offset, candidate count, path length, phase and
completion outputs with a controlled search clock/budget diagnostic. This
diagnostic removes wall-clock scheduling variability and is not a production
latency or closed-loop tracking test. The original 15-commit history remains
in `backup/avoid-stop-merge-validated-20261010` at `549e458`; the structural
port is recorded as one integration commit on the latest main base.


## 2026-10-24: v6 공통 지도와 후보별 출발 근거

이 절이 앞선 관측 신뢰도 설명을 보완한다. 메시지/FSM 전환, 일반 STOP,
AVOID, 고주로 후보/간격/비용 정책은 변경하지 않는다.

모든 모드가 공식 v6 벡터·격자를 사용한다. 지도 변경은 A111/A226의 검토된
포장면 확장이고 회전교차로 영역은 v5와 같다. v6 bundle 동기화와 혼합 버전
거부 검사는 `config/frenet/hd_map/README.md`를 따른다.

이전에는 근처 차량의 전체 차체가 안 보이거나 위치 기반 속도가 입력 속도보다
빠르면, 해당 후보와의 충돌 여부와 무관하게 연속 출발 확인을 지웠다. 이제
기존 엄격한 관측 판정 자체는 유지하면서, 다음 조건에서만 추가 점유 영역을
만들어 해당 ID의 일괄 거부를 후보의 최종 검사로 넘긴다.

- 서로 다른 sensor stamp의 최소 4개·0.25초 이동 이력, 기존 방향/회전율/속도
  변화 검사, 위치 이동 fit이 유지되어야 한다. ID 불연속·미관측·반대 방향의
  속도 불일치는 추가 출발 근거를 만들지 않는다.
- 같은 방향의 위치/입력 속도 차이는 두 측정과 위치 fit 오차를 포함하는 속도
  구간으로 전달한다. 위치 지지 직진·센서 단기 회전 가설과 기존 turn 모델도
  최종 충돌/시간 여유 검사에서 유지한다.
- 최근 전체 차체 기억이 있으면 그 중심·크기와 현재 관측 코너를 모두 포함한다.
  전체 차체 기억이 없는 부분 관측은 실제 위치 변화가 있고 v6의 순환 차로
  영역 부근에서 관측된 경우에만 보완한다. A151 자차 출구를 순환 차량의
  후속 차로로 가정하지 않는다.
- 부분 관측 차체 크기 prior는 기존 passed-vehicle 검사와 같은 6 m × 3 m다.
  이는 검증된 센서 오차나 모든 차량의 크기 보장이 아니다. 조각 중심과 차체
  중심의 불일치를 포함하기 위해 부분 관측만 있는 점유의 **반길이·반폭**에
  이 전체 크기를 사용한다. 더 큰 실제 관측 코너는 잘라내지 않는다.
  nominal metadata 높이로 전체 차체를 관측했다고 꾸미지 않는다.
- 원본 객체 코너는 지우거나 축소하지 않는다. 보완 점유도 최종 충돌 검사에
  포함한 경우에만 신뢰도 거부를 대체한다. 근거가 부족하면 기존대로 대기한다.

기존에 생성하는 속도 후보를 최종 강화 검사까지 통과시킨 뒤 비용으로 고른다.
선택한 하나가 마지막 검사에서 실패했다고 이미 생성한 다른 안전 속도 후보까지
버리지 않는다. 속도 격자·기하 후보·예측 시간·시간 간격은 늘리거나 완화하지
않는다. 일반 검사에서 선택하던 기하 family의 종료 순서도 유지한다.

강한 감속 중 진입 제한, rear-axle 대기 위치, source freshness, 서로 다른 최신
관측 3회 확인, READY 토큰 검증, commit 이후 충돌 검사와 제동은 유지한다.
따라서 이 수정이 모든 긴 대기나 정지 없는 진입을 해결했다고 해석하지 않는다.
시뮬레이터 폐루프 검증과 실제 순환 차량 크기 가정 검증은 별도다.


### 이 변경의 오프라인 검증

- 실제 Frenet/참조선 core 47개 translation unit 빌드.
- 회전교차로, 일반 STOP/KEEP/FOLLOW, AVOID, 고주로 합류·좌우 차선변경·인계 등
  native test executable 32개 통과. 서행 중 최신 안전 관측으로 READY가 되고
  commit 후 정지용 접근 대신 통과 경로를 출력하는 계약도 검사했다.
- 지도 bundle 일치/잘못된 NPZ/격자 변경 거부 3개와 기존 고주로 asset gate
  11개 Python 검사 통과. 실제 native highway preflight도 v6에서 통과했다.
- 과거 회전교차로 두 bag의 진입 전 508개 입력을 수정 전 전체 core와 비교했다.
  기록 시각·ego header 시각을 각각 사용했으며, 준비/대기 출력 궤적은 같고
  생성 후보 개수는 늘지 않았다. 기록 시각 호환 재생에서는 두 번째 bag에
  READY 한 프레임이 추가됐지만 두 bag 모두 최초 READY 시점은 같았다.
- 과거 bag의 ego/object 시각 대응을 현재 runtime과 완전히 복원할 수 없고,
  header 시각 재생은 수정 전후 모두 연속 READY를 만들지 못했다. 따라서
  실제 출발 지연 감소 또는 진입 실패 해결을 이 재생으로 입증하지 않는다.

이 환경에서는 ROS/catkin 전체 build와 MORAI 폐루프 주행을 실행하지 않았다.
