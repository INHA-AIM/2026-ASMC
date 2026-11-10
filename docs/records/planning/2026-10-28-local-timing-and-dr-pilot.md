# LOCAL-1 시간·속도 정책과 짧은 DR 대조

상태: 읽기 전용 재생·정책 probe 완료. 운영 변경·25km/h 주행 수락 아님.
기준 구현 `7bda2c26`, 기존 [LOCAL-1 결과](2026-10-28-local-planner.md)를 보존한다.
새 근거는 D `artifacts/local1-20261028-dr-pilot01`이다. 기존 bag과 native 기록은
읽기만 했으며 새 시뮬 제어·센서 소켓·운영 ROS publisher를 만들지 않았다.

## 확인한 원인

이 문서는 LOCAL-2A 구현 **전** 분석을 보존한다. 이후 변경·35m 반복 주행3회와
흰선–벽 측정은 [LOCAL-2A 결과](2026-10-28-local-input-alignment.md)를 따른다.
아래 추가10Hz·50ms join·후진 reset 설명을 현재 실행 설정으로 다시 적용하지 않는다.

| 증상 | 현재 코드·probe 근거 | 처리 책임 |
|---|---|---|
| 후진 시 local 그림 소실 | MotionHistory는 v<-0.05에서 epoch reset. 음수51표본의 native probe 출력0개 | 상대 상태는 signed motion을 지원하고 관측 표시는 전진 계획 유효성과 분리 |
|10km/h 이상에서 입력 공백이 크게 드러남 | source350ms·LiDAR/family ±50ms·상태150ms, 회복 경로 grace는≤5km/h일 때만 | 로컬 입력 선택/정렬과 제어 상태 정책 |
| 빈 도로인데3km/h | 자차 가림→near_field_assisted_sweep→3km/h 상한. clear raster+hood만으로도 재현 | 가림·미관측 정책과 관측 거리/속도 계약 |
| 완전 free raster에서도25km/h 불가 |20m grid에서 앞부분을 뺀 약16m sweep. age180ms에서 clear cap20.08km/h, 실측25입력은 insufficient_stopping_space | 제동/지연·관측 범위·reference 생성 |
| 넓은 가짜 OBB가 감속 유발 | 합성 OBB가 도로폭을 막으면 정지 상태cap2.62km/h,10km/h 입력은 invalid. 유효 속도나 unknown 이름만으로 거짓 OBB인지 판별할 수 없음 | 인지 품질/구조물 판단 + planner의 위험 단계별 응답 |

위 속도 probe는 현재 C++ Plan을 직접 호출하며 운영 설정을 바꾸지 않는다.
합성 free raster는 실제 터널의 빈 공간·충돌 안전을 입증하지 않는다.

### 최신 입력과 지연

최신 **완료된 관측을 선택**할 수 있지만 이를 현재 시각 관측으로 바꾸면 안 된다.
25km/h에서185ms는1.28m 이동이다. 원본 header·상대 이동 이력·plane을 보존하여
기준 시각으로 한 번 정렬해야 한다. family/plane/probability/coverage는 같은
카메라 bundle을 유지하고, LiDAR는 자신의 source·age로 선택·변환한다.
LiDAR와 camera header의50ms 차이를 로컬 경로 생성의 필수 join으로 둘 필요와,
F1 카메라↔LiDAR matching의 ±50ms 계약은 서로 구분해 검증해야 한다.
동적/속도 무효 OBB를 오래된 정적 물체로 간주하거나 원래 시각을 덮어쓰지 않는다.

### 시각화와 계산 경로

현재 흐름은 model 원본 probability + family worker의 projection context →
semantic_space_trial의 **RGB 없는 확률/coverage 투영** → C++ local planner → MPC다.
웹·JPEG·rqt 이미지를 다시 planner 입력으로 읽는 경로는 없다.
ComparisonProjection/RenderWorker라는 유틸리티 이름을 쓰지만 drivable()은
영상 표시·JPEG encode를 하지 않는다. 단, 별도 실험 producer가 RenderWorker의10Hz 상한을 사용하므로 계산 입력까지
10Hz에 묶인다. same-stamp bundle 준비 대기도 실제 실행 경로에 포함된다.
LOCAL-2A에서는 계산 입력 공급과 웹 표시 주기를 분리해 검증한다.

읽기 전용 live 진단 한 표본에서 family callback 전체 p95 약47.5ms,
association14.7ms, shared family fit16.7ms, projection8.8ms였다. 로컬 planner는
이미 C++이며 정렬 약0.2ms/후보·sweep 약29ms다. Python 전체를 먼저 옮기기보다
association·fit hot path를 같은 입력의 수치 회귀와 함께 좁혀 최적화할 수 있다.
그 최적화로3km/h 정책이나 제동거리 부족이 해결되지는 않는다.
진단 원본은 run의 perception-profile.json에 보존한다.

## AIMDrive 참고 범위

공식 저장소 `https://github.com/INHA-AIM/AIMDrive`를 workspace external/baselines에
code-only sparse checkout했고 revision은 `1606844179d0528c673e7a81a7820a980e2372bb`다.
브라우저의 비인증 접근은404였지만 사용자의 Git 인증으로 소스를 확인했다.

- AIM2026/Localization/gps_aligned_hold.py: IMU 자세로 body-forward를 얻고
  signed 차속을 source clock으로 적분하는 MotionTrack, GPS offset의 causal
  window anchor와 복귀 시 smoothing.
- gps_blackout_hold.py: 초기 GPS anchor, 시간/거리/입력 중단 한계.
- adapters.py: 허용된 차속·IMU만 전달. GT position/heading은 DR 입력에서 제외.
- Expert/src/planning/jamming: 기존 정적/lattice 코어 재사용과 기록 경로·통로
  계획. 새로운 네트워크 모델을 필수로 두는 방식이 아니다.

참고 대상에 root의 통일된 라이선스를 확인하지 못했으므로 코드를 팀 repo나
운영 의존성으로 복사하지 않았다. 로컬 숫자 대조에서만 원본 MotionTrack을
직접 호출했다. 사용한 파일 SHA는 external-baseline.json에 기록했다.
전체 LocalizationService·GPS window smoothing·Expert 제어를 이식/수락한 것이 아니다.

## 짧은 기록 재생 결과

두 코어에 같은 causal speed-at-IMU resampling을 주고 비교했다. 기존 C++
MotionHistory는 운영 헤더를 직접 include했다. 양쪽에 넣은 시각은 bag의 published
ROS header/수신 시각이며 raw 센서 acquisition clock을 새로 복원하지 않았다. AIM은 원본 MotionTrack을 직접
사용했다. 상대 오차는 **첫 native 평가 위치에 평행이동 한 번만 맞춘 결과**다.
GT yaw/scale 조정, 나중 GT 보정, 터널 GPS/벽/map snapping은 사용하지 않았다.

| 기록 | 실제 이동 / 기록 최고속도 | 현재 C++ 상대오차 p95 | AIM 상대오차 p95 |
|---|---|---|---|
| LOCAL-1 drive20 |35.07m /3.73km/h |21.34cm |21.50cm |
| F1 straight |18.85m /13.92km/h |7.73cm |7.83cm |
| F1 tunnel |18.61m /14.83km/h |4.94cm |4.84cm |

최종 재생은 `*-causal/summary.json`이며, 앞의 `*-lateral`은 GPS anchor의
오프라인 보간을 사용한 중간 대조로 보존한다. 최종 GPS anchor는 이미 수신한
과거 상태와 최대50ms 제한 예측만 사용하며 미래 IMU 수신을 미리 사용하지 않는다.

초기 GT 위치 정렬은 상대 drift 평가용이며 deploy 가능한 GPS anchor를 입증하지
않는다.35m 사례는 상대 횡오차 p95 약21cm로, 단순 종방향 오차만 남은 것도 아니다.
별도 단일 GPS anchor 뒤 나머지 GPS를 가린 비교에서35m의2D 오차 p95는
약13.5/13.6cm, 횡오차는13.0/13.2cm였다. GPS acquisition 지연과 SDK 기준점
일치가 독립 검증되지 않았다는 제한을 함께 보존한다. F1 tunnel은 초기 유효
GPS가 없어 GPS-anchored 결과를 만들지 않았다.

정상 입력에서 두 방식의 차이는 작았다. 따라서 현재 상대 상태 기반을 버리고
외부 패키지를 통째로 들여올 근거는 없다. 반대로120m 전체·25km/h·긴 장애물
회피를 DR만으로 충분히 버틴다고 결론낼 자료도 없다. 좁은 여유10cm를 기준으로
보면 현재 drift는 무시할 수 없다.

### 재현

ROS bag Python과 pyproj가 있는 outer ROS 환경에서 새 run을 등록한 뒤 실행한다.
입력·출력·external 경로는 실행자가 명시하며 기존 결과를 덮어쓰지 않는다.

```bash
g++ -std=c++17 -O2 -Isrc/asmc_planner/src -Isrc/asmc_geometry/include \
  tools/perception/frame_time/short_dr_probe.cpp -o /tmp/short_dr_probe
python3 tools/perception/audit_short_dr.py \
  <input.bag> <same-record-native.jsonl> <new-run-output> \
  --aimdrive <pinned-external-checkout> --probe /tmp/short_dr_probe
```

이 도구는 ROS master에 연결하거나 제어 명령을 발행하지 않는다. 같은 기록의
native SDK evaluation 파일만 허용하며 별도 기록을 합쳐 동시 데이터로 취급하지 않는다.

## 제안하는 후속 단위

각 단위는 별도 커밋·고정 입력·실험 수락 뒤 진행한다. 이번에는 아래 운영
변경을 실행하지 않았다.

| 단위 | 범위 | 완료 기준 |
|---|---|---|
| LOCAL-2A: 상태·입력 분리 | signed 상대 이동, 관측용 웹을 전진 planner validity와 분리. 최신 완료 camera bundle과 별도 LiDAR snapshot을 공통 기준시각에 정렬. 실제 근거 부족과 단순 join 실패를 구분 | 후진에도 관측 유지, 전진 시간 역행/재시작·고속 source 결합 회귀. 불필요50ms join 때문에 전체 장면이 무효가 되는 사례 제거, 근거 age는 그대로 기록 |
| LOCAL-2B: 무장애25km/h 기준선 | 우측 본차로를 부드러운 reference로 두고, 실제 주행가능 공간에서 연속 후보 탐색.3/5개 추종선 선택을 도입하지 않음. family 공통 support와 물리 통로를 분리하고 prepared scene 재사용. entry route/DR은 같은 planner/MPC 경계의 보조 대조. 가림/제동·지연 정책 정리 | 실제 터널 길이에서5→10→15→20→25 단계, 반복3회 횡오차·조향·불필요 제동·접촉·명령 공백 측정.25 미달 제한 사유 수치화 |
| LOCAL-2C: 인지 노이즈 응답 | 벽/천장·가짜 OBB의 점 지지·높이·지속성·semantic 모순을 분석. 위험 단계와 속도 응답을 나누고 실제 객체 보존 확인 | 무장애 오탐 주입에서 불필요 급제동 억제, 실제 차량/사람/드럼/얇은 barrier 누락0. unknown이라는 이유로 제거하지 않음 |
| LOCAL-2D: 단일 정적 회피·복귀 | 검증한 reference 위에서 기존 Frenet candidate/collision/speed 및 MPC 재사용. 현재 local 메시지 경계 유지 | 좌우 각3회, 관측 시각·회피 시작·차체 최저 여유·복귀 오차를 검수 |
| 조건부 성능 단위 | family association/fit·projection와 planner 반복 sweep의 hot path를 개별 최적화 | 같은 입력의 기하/역할·support 보존, p95 개선을 비교하고 새 공급 공백0 |

첫 구현은 **LOCAL-2A**가 적절하다. 그 뒤 동일한 상태/입력 기반에서2B의
reference를 선택한다. 기존 map VehicleState의 GPS invalid 검사를 풀거나
DR pose를 정상 GPS로 위장해서 NORMAL 실행에 바로 주입하지 않는다.

DR로 전역경로를 계속 사용해도 실제 장애물 관측 범위가 늘어나는 것은 아니다.
map 도로 형상을 알고 있다는 사실을 장애물 없는 공간의 증명으로 사용하지 않는다.
제동 성능·진입 anchor·기준점·yaw basis·GPS 복귀 전환을 별도 검증한다.


## LOCAL-2A의 첫 구현 경계

이번 검토에서는 운영 입력·메시지·모델·속도 기본값을 변경하지 않았다.
다음은 다음 구현 단위의 구체화이며,2B의25km/h 폐루프 수락과 구분한다.

1. **상대 상태와 관측 표시 분리:** MotionHistory는 signed 차속을 적분한다.
   후진 때문에 카메라/OBB 관측 캐시를 비우지 않는다. 자동 후진 경로·MPC는
   이번에 추가하지 않고 `reverse_control_not_supported` 등 명시 사유를 쓴다.
   웹 원본 관측은 전진 계획 valid와 독립적으로 표시한다.
2. **비차단 입력 snapshot:** camera의 plane/family/probability/coverage는
   같은 source bundle로 유지한다. 최신 유효 LiDAR snapshot은 별도로 선택하여
   공통 plan 시각의 상대 frame으로 한 번 변환한다. F1 fusion의50ms matching
   계약은 그대로 두고, 로컬 planner에 복제된 join 필수조건을 분리한다.
   움직이는 객체·속도 무효 객체의 나이/불확실성을 같이 전달한다.
3. **시간 의미 고정:** LocalTrajectory.header는 실제 계획 기준시각,
   scene_stamp와 diagnostics는 원본 관측 시각이다. ego_at_plan과 points는
   header 시각의 같은 상대 frame이다. 이전 관측을 now로 이름만 바꾸지 않는다.
   MPC에서 원본 camera 지연을 다시 더해 이중 정렬하지 않는다.
4. **계산 공급과 표시 스케줄 분리:** semantic 전용 producer의10Hz 제한을
   추론 공급 기준의 latest-complete 작업으로 대조한다. 웹 렌더링 상한은 별도다.
   중복 추론·RGB 인코딩·두 번째 IMU 보정은 추가하지 않는다. callback 대기
   p95가 실제 증가하지 않는 한 AsyncSpinner/새 lock 구조부터 도입하지 않는다.
5. **속도 정책의 원인 가시성:** 관측 소실, 실제 도로/장애물 차단, 가림 보조,
   제동거리 부족, 제어 상태 무효를 분리한다.25km/h를 통과시키려고 관측 거리나
   감속 능력을 늘려 표시하지 않는다. 고속 path grace 확대는2B의 잔여 sweep·
   제동 검증 뒤로 둔다.

2A 수락은 후진 관측 표시 유지,10/25km/h 기록/합성 입력에서 불필요 join
무효화 제거, 원본 시각·세션·무효 의미 보존,20Hz preview 제어 공급과
계산 p95≤50ms 유지다. 실제25km/h 주행 수락은2B에서 한다.

2B의 기본 reference는 사용자가 지정한 **우측 본차로**다. 외곽 흰선을 물리 도로
경계로 쓰지 않는다. 연결된 노면 공간마다 목표를 만들고 본차로 근접·복귀는
비용으로 평가한다. 전체 노면 무게중심이나3/5개 선 선택을 제어 목표로 고정하지
않는다. [실행 단일성과 자유공간 설계 정본](../../architecture/shadow-planning-refactoring.md#local-실행-단일성과-기준-경로자유공간-설계)에
기존 코어 재사용·구 제어 배제·터널폭 활용·남은 내부 반복 계산을 정리했다.

조건부 entry-route 대조는 정상 GPS가 있는 A448 접근 때 전역경로를 세션 local
frame으로 한 번 가져오고 signed 차속+IMU로 진행한다. family의 횡방향 근거를
보조로 쓸 수 있다. 같은 LocalTrajectory/MPC 경계와 기존 Frenet 수학을
재사용하며 별도의 전체 NORMAL/DR planner를 두 벌 실행하지 않는다.
실제 GPS 복귀와 mode 전환은 명시적으로 인수한다. 원래 map 상태를 유효한
GPS처럼 위장해서 공급하지 않는다.

## 실행 분리·흰선 경계 회귀

위 pilot 이후 사용자 설계 조건을 반영하여 새 local의 실행 경계를 점검했다.
현재 ROS graph는 새 local 두 노드만 실행하는 preview이며 구 map/BBox 제어는
실행하지 않았다. `local.launch`는 그 구 노드를 include하지 않는다.
제어 활성 시 기존 JAMMING 슬롯의 중복 생산자뿐 아니라 NORMAL 제어 생산자도
거부하도록 검사 범위를 보완했다. 판제 행동·경로·속도·MPC 수학은 바꾸지 않았다.

컨테이너의 `asmc_local_controller_node`·`local_planner_test`를 재빌드하고 다음을 확인했다.

- 독립 master11349에서 기존 시각·유효성·저속 grace·preview 비발행 회귀 통과.
- 다른 NORMAL/JAMMING 생산자가 있을 때 local 자신이 제어를 거부함.
- local 자신과 최종 selector 출력만 있을 때 정상 실행하며, 나중에 NORMAL
  생산자가 추가돼도 거부함. 운영 master에 시험 제어 토픽을 발행하지 않음.
- 코어에서 가용 노면±3.5m·표시 흰선±1.75m·중앙 OBB를 주었을 때 흰선을
  넘는 차체 sweep의 유효 경로 생성·충돌 검사 통과. 실제 터널 외곽의 가용 폭을
  검증하거나 물리 통로를 측정한 결과는 아님.
- 문서 보존 검사와 현재 실행 명령 검사 통과. hosted CI는 실행하지 않음.

이 보완은 LOCAL-2A/B의 시각 정렬·공통 support 분리·단일 scene 재사용을
구현한 것이 아니며25km/h 폐루프 수락도 아니다. 이번에는 새 시뮬 주행·녹화를
추가하지 않았고 frozen pilot/run을 수정하지 않았다.
