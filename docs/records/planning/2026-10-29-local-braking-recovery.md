# LOCAL-2F — 복구 경로의 제동 일관성과 입력·MPC 점검

기준은 LOCAL-2E `1aba8dbe`다. GPS 독립 상대측위 LOCAL을 개선하며 다른 localization
worktree, NORMAL 행동, 공용 메시지와 모델은 변경하지 않는다. 당시2E 결과는
[이전 검수](2026-10-29-local-avoidance-continuity.md)에 보존한다.

## 구현 단위와 구조

`asmc_planner/src/local/speed_profile.{hpp,cpp}`로 실제 jerk 제동 거리·공간 속도 한계·
물리적 속도 궤적을 분리했다. ROS 입출력은 `node.cpp`, 경로 탐색·sweep은
`planner.cpp`, MPC 입력 경계는 `asmc_controller/src/local`에 유지한다. 별도 노드나
두 번째 controller/tracker는 추가하지 않는다. 모든 종류의 후보에 같은 제동 함수를
적용하며, 후륜축 이동 거리에 차체 앞부분을 두 번 빼지 않는다.

기존 복구에는 두 문제가 있었다. 유효한 swept 경로도6m 미만이면 버리고4m arc로
대체했고, 복구의 실제-state 제동 검사에서는0.5초 실행 지연을 생략했다. 합성 장면의
10km/h·실행 시각 정렬 상태는4.389m가 필요했지만4m arc가 유효라고 나왔다.
수정 후5.2m의 검증된 aided 경로를 사용한다. `before-recovery-test2.log`에 수정 전
실패, `after-recovery-test2.log`에 수정 후 통과를 보존한다.

- 최소4m를 넘어 관측 근거가 있는 aided sweep을 실제 제동 가능 여부로 채택한다.
- generic 복구는 필요 제동거리+25cm까지, 최대8m를 비교한다. 기존 soft 근거리4m를
  넘는 부분은 현재의 엄격한 차체 sweep를 통과해야 한다. 미관측 범위를 늘리지 않는다.
- 복구도 실제 속도·가속·jerk·실행 지연을 모두 담아야 한다. 통과 후보만 선택하고
  3km/h의 명시적 복구 상한은 유지한다. 측정 OBB의 실제 겹침은 항상 거부한다.
- source freshness350ms, motion forecast100ms, selector watchdog250ms를 유지한다.
- 상세 기록에 필요/확보 제동거리와 복구 연장량을 추가한다. 무효 입력을 새 시각으로
  재발행하거나 STOP을 단순 무시하는 방식은 사용하지 않는다.

이는 아직 simulation recovery 후보이며, 근거리 soft 노면 정책이 포함된 주행을
전 범위 충돌 없는 공간의 인증으로 표현하지 않는다.

## 인지 입력과 클래스

현재 E3 모델의15종을 카메라와 LiDAR fusion이 모두 취급한다. vehicle은 car/bus/truck,
dynamic_obstacle은 person/dog/deer/bicycle/stroller, static_obstacle은
motorcycle/bollard/box/concrete_barrier/steel_barricade/traffic_drum/plastic_barrier다.
traffic_cone은 이전16종 모델의 클래스이며 현재15종 모델에서 빠진 것을 fusion 누락으로
해석하지 않는다. 실제3뷰 association과 그룹 충돌 정책은 기존 구현을 유지한다.

LOCAL은 실제 관측된8코너, 원본 클래스·그룹, 속도 validity를 받는다. 그룹은 현재
장애물 삭제나 절대 운동 상태의 근거로 사용하지 않는다. motorcycle이 static 그룹이고
차량/사람도 정적으로 배치될 수 있기 때문이다. public 객체에는 최근 양성 YOLO 시각이
없으므로 retained 클래스를 새 양성 근거로 간주하지 않는다. 추정 track은 현재 필수
충돌 입력과 구분한다. dynamic 판단 확대는 관측 위치·속도 신뢰성 검증 뒤 진행한다.

미확정 속도는5m/s의 나이별 이동 envelope를 유지한다. 이는100ms에50cm로 커질 수 있어
좁은 곳에서 보수적이다. 이번에 semantic 이름만으로0으로 바꾸지 않는다. 후속 후보는
동일 track의 실제 기하·양쪽 면 이동과 안정된 시간 근거에 따른 planning uncertainty
선택이며, 새 KF/운동 분류기나 물체 삭제 방식은 피한다.

## MPC 점검 방법

현재3m 축거, curvature gain0.844, 조향 지연0.15초,50ms step의 kinematic bicycle을
사용한다. LOCAL horizon은2.4초다. 지연/issue steering 이력과 선형화에 같은 gain을
적용한다. 저속10–15km/h에서 새 동역학 모델을 추가하기보다 실제 yaw-rate와 속도·
조향 명령의 대응을 먼저 평가한다.

`scen_runner.records.local_model_audit`는 실제bag의IMU/속도와 최종 wire 명령을 읽어
미래 표본을 사용하지 않고0–300ms effective delay와 gain의 민감도를 확인한다.
전반부로 선택하고 후반부 오차를 따로 기록한다. 수신 시각과 명령 관측에 포함된 지연을
순수 actuator delay나 측정 wheel angle로 부르지 않는다. 설정 자동 수정은 하지 않는다.

구조 참고: [Autoware MPC](https://autowarefoundation.github.io/autoware_universe/main/control/autoware_mpc_lateral_controller/model_predictive_control_algorithm/),
[Path Sampler](https://autowarefoundation.github.io/autoware_universe/main/planning/sampling_based_planner/autoware_path_sampler/).
후보 생성·차체 충돌 검사·비용·속도 실행을 분리하는 현재 구조를 유지한다. 외부 ROS2
의존성을 추가하거나 논문 성능을 우리 실측으로 표현하지 않는다.

## 검증·후속

등록 run은 `local2f-20261029-*`다. 코드·프로필 SHA, 독립 master와 물리 lease, 실제
최종 wire, 접촉과 정지/복구, 본차로 복귀를 구분한다. 새로운 실행 전에 바이너리와
소스가 일치하는지 확인하며 다른 worktree의 시뮬 시험과 겹치지 않는다.

다음 작업의 우선순위는 실제 시험의 실패 원인으로 결정한다. 반복 정지와 OBB/semantic
관측 실패, 단순 복구 주행과 정상 sweep 회피를 섞어 성공으로 표시하지 않는다.
운영 기본 전환·push·병합은 이번 단위에 포함하지 않는다.

## 짧은 관측 소실 보조 후보

12km/h 우측 시험 drive02의scan59–62/77–78에서 실제 관측 배열에서barrier가
잠깐 사라졌다 재등장하며 선택 경로가7→2.6m로 짧아졌다. 이를 단순 조향 모델 오류로
해석하지 않는다. 현재 typed LiDAR 출력은실제 관측만 제공하므로 LOCAL에 후보
`obstacle_memory.hpp`를 추가했다. 기본은 OFF이며 별도 등록 profile에서 대조한다.

같은 생산자/세대/relative frame에서 실제scan3개에 나타난 box/bollard/drum/barrier의
원본 hull만 원래 시각부터최대200ms 보존한다. 동일scan 재계산은 확인 횟수/시각을
갱신하지 않는다. 현재 재관측이 우선하고, 다른 클래스·운동확인·역행·reset이면 비운다.
5m/s 이상의 원래 이동 불확실성을 그대로 증가시키므로 절대 정지 판단이 아니다.
변환은 기존 상대측위를 이용하며 공용 객체나tracker를 수정하지 않는다. 상태는최대64개다.
unknown/vehicle/dynamic/motorcycle을 정적 객체로 간주하지 않고, public 클래스가
retained인지 여부를 새 YOLO 양성 확인으로 해석하지 않는다. 웹에는 과거 측정 사용과
원래 나이를 구분한다. 필요한 객체를 줄이는 필터가 아니라 일시 소실 중 가속·복귀를
늦추는 planning 보조다. false OBB를 지우는 방법과 혼동하지 않는다.

## 정지 실패의 짧은 재배치 후보

`drive07`은9.82m에서 정지해12초stationary 판정이 났다. 실패는 보존한다.
새 관측이 들어와도 가까운barrier 앞의 짧은aided prefix를4m 조건으로 버렸다.
추가 후보는 후륜축0.6m 이상의 경로와 미래 차체 앞부분의 현재 노면 셀3개 이상을
요구한다. 이미 점유한 차체 내부나unknown은 근거로 세지 않는다. 현재 속도의 제동
가능성·기존sweep의 비노면/OBB 거부·3km/h 상한을 유지하고
`near_field_assisted_prefix`, `bounded_recovery_prefix`로 구분한다.

ROS 정합 시험의 모든plan에서LiDAR–family 차이가150ms를 넘어야 한다는 가정도
바로잡았다. 네topic callback은 원자적이지 않다. 실제 선택stamp 차이와age 차이의
일치·모든 표본의 기존freshness 준수·시험 전체의150ms 초과 독립시각 사례 존재를
검사한다. 운영 시각 한계를 완화하지 않는다.

### 정지 장면의 두 번째 보완

동일 위치로 시작한drive09도준비 실패였다. 당시source scene을그대로 C++ core에 넣었을
때 aided 경로는0.6m였지만 미래 차체 앞부분이 모두가려져 현재front-road 셀 조건을
충족하지 못했다. 이는 단순 계산 예산 부족이 아니며250ms 예산도같이실패했다.

기존generic 복구는centroid arc 전체가충돌하면 앞의유효 prefix까지버렸다. 현재는
실제충돌 pose 이전까지만 잘라 제동 검사를하고, 관측한 opening 방향의최대곡률 후보도
추가한다. 원점 차체가 이미 OBB와겹치면복구하지 않는다. 실제 차체는매10cm의각pose만
검사하던 것에서기존연속sweep 검사로 바꿨다. 정지·1km/h 이하에서만 opening 근거에 따른
짧은재배치 예외를쓰고, 나머지속도는기존관측/제동 조건을유지한다.

또한복구가 내부그림자뿐 아니라도로외곽 비노면까지비용으로만 처리하던부분을제한했다.
현재raster의각행에서노면 양끝밖에있는 관측비노면은차체+기존최소여유에대해 hard veto다.
내부hole만복구의soft cost로남는다. 이는새벽 검출이나실제터널벽의독립측정이아니다.
원본probability/coverage를채우거나수정하지않으며 추가색인은40kB/scene이고재귀탐색에서
공유한다. 미관측전체·실제OBB·도로외곽·분리된partial-road 반례를유지한다.

`local_planner_test`에실제정지scene의결정동등fixture를추가했다. 원본8bit는D에보존하고
153/166의운영threshold에서동등한3레벨RLE를Gitfixture로사용한다. coverage와geometry는
그대로다. 직접C++대조와전구간sweep 검사로이fixture의재배치경로를확인한다. 준비실패와
실제폐루프검증은서로다른결과로남긴다.

## 쾌적 감속과 실제 제동 후보의 분리

최종 경로가 짧아진drive11의69/70계획은실제속도2.87/2.82m/s와이미감속중인상태에서
쾌적제동1.5m/s²가필요한4.48/4.31m보다경로3.81/3.61m가짧아무효가됐다.
이는충돌이나차량물리제동불가와동일하지않다. 기존MPC의음가속한계는5m/s²이고
공용pedal LUT에는0.20→2.71,0.30→4.75m/s² 보정이있다. 기존/이번native기록의
안정된0.12–0.20pedal window54개에서는감속중앙값3.15m/s²였다. 범위1.76–3.90으로
편차가있으며정식actuator identification이나3m/s²의전조건보장은아니다.

별도profile에서만`controlled_braking_limit_mps2: 3.0`을대조한다. package기본은1.5다.
평소공간속도상한은1.5와기존overshoot reserve를그대로사용한다. 실제현재상태만쾌적
범위에안들어갈때3.0후보로정지가능성을다시확인하고동일jerk·실행지연·충돌경로안에
속도궤적을생성한다. 3.0으로도부족하면무효다. 순간1.5불충족을항상full-brake로
바꾸는것과구분하며, 실제선택감속·쾌적제동거리·필요/확보거리를진단과웹에기록한다.
일반MPC의LUT나행동정책을수정하지않는다. native반례는cruise 상한불변·jerk·최대감속과
더강한제동으로도불가능한경우의거부를검사한다.
