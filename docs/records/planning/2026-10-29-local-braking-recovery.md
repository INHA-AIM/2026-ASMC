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
