# B1a 검증에서 공통 기반 구현으로 넘기는 과제

2026-10-08 기준. 검증 대상은 재호 `8dde9b6`, 운영 대조군은 main의 nested `Main`이다.
[44회 검증과 원본 증거](../../../docs/records/camera/camera-lidar-b1a.md)를 보존한다.
아래 제안은 구현 완료나 필터 운영 수락을 뜻하지 않는다.

현재 카메라 수신·버퍼·투영 지원은 [camera-input](camera-input.md)을 따른다.
아래 B1a 기록과 후속 LiDAR 알고리즘 과제는 그대로 보존한다.
2026-10-22에 Main common-geometry LiDAR 경로를 [shadow 노드](shadow-pipeline.md)로 동작 보존 이식했다(운영 미연결).

## 확인된 경계와 먼저 고칠 사항

| 경계 | 확인한 사실 | 공통 기반 수정·시험 |
|---|---|---|
| GPS 무효 | 재호 경로는 local 처리 중에도 최대 약 4,124,175m의 잘못된 map 좌표를 발행 | 무효 GPS 즉시 map 차단, typed `POSE_UNAVAILABLE`, legacy 보류 |
| GPS 전달 중단 | 정상 GPS 이후 메시지만 중단하면 재호 터널 필터가 활성화되지 않음. main은 stale pose gate로 local 처리도 중단 | GPS 가용성과 local 입력 준비를 분리. 무효 샘플·침묵을 별도 시험 |
| 수평화 | 일반 경로는 pitch, 터널은 `Ry(pitch) Rx(roll)`. 기존 map 역변환은 pitch만 복원 | 관측에 실제 적용 행렬을 저장. 공통 roll/pitch는 별도 비교 후 승격 |
| 카메라 클래스 fusion | raw 센서 좌표용 고정 투영 행렬에 수평화된 군집 점을 직접 넣음 | 해당 관측의 역수평화와 장착·광학 변환을 합성. 실제 영상 오차 크기는 추가 측정 |
| 터널 YOLO 보호 | 수평화 이전 raw voxel 점을 투영 | 위 클래스 fusion과 다른 경로. 역수평화를 중복 적용하지 않음 |
| 추적·동정적 분류 | main은 map 거리 연관·중복 정리·map 위치 차분을 사용 | local 연관 통일, GPS 변경에 ID reset 금지. map 근거 무효·재준비 분리 |
| 초기 시간 | 29회에 초기 ROS-now 빈 출력→첫 source stamp 역행, 세 출력 총 87건 | 초기 가짜 관측 금지. 센서 침묵을 새 stamp 빈 관측으로 표현하지 않음 |

## 좌표·출력 역할

내부 raw/수평화 센서 좌표를 유지할 수 있지만 공개 local 객체는 후륜축 `base_link`다.
`/lidar_nogps`의 수평화 센서 원점을 `base_link`로 이름만 바꾸면 안 된다.
수평화 역변환→센서 장착 변환→유효 pose map 변환을 중심·8코너·축에 각각 한 번 적용한다.
카메라에는 합성 행렬로 직접 투영하며 중간 점군 전체 복사를 요구하지 않는다.

검출·추적은 한 번 수행한다. map 변환은 동일 프로세스의 독립 모듈로 두고, typed local/map
출력은 구독자에게 제공한다. local 처리 지속과 토픽 상시 직렬화·발행은 구분한다.
정상 구간 판제는 map 출력을 계속 소비할 수 있다. local 연관은 자차 회전·이동 회귀가
필수이며 map 연관보다 항상 정확하거나 빠르다고 주장하지 않는다.

카메라 YOLO는 픽셀 박스다. 카메라 BEV/차선은 `base_link`이고 HD맵 local crop도 `map`이다.
강태의 어댑터는 차선을 영상에 투영해 박스 하단의 차로 점유를 판단하며 현재 LiDAR를
소비하지 않는다. 현재는 카메라 외부 YOLO의 CPU 변환을 사용하고 기본 crop0/Ego mask를 적용한다.
아래 B1a 수치와 당시 경로는 역사적 비교이며 PR #87 이후 독립 YOLO는 제거됐다.

## 재호의 후속 과제

1. **터널 처리 이전:** 고정 브랜치의 오래된 실행 루프 전체를 복사하지 않고 검증된 core를
   연결한다. GPS/미션으로 tracker·검출 경로를 reset하지 않는 단일 처리 구조를 목표로 한다.
2. **잔여 벽 혼입:** 벽에 붙은 보호 후보 79개 관측을 고정 fixture로 사용한다. box 내부
   깊이·연결성, 차량 돌출부와 배경을 비교한다. 79개를 독립 차량 수로 계산하지 않는다.
3. **보호·높이 제한:** 보호점 30개가 천장 절단 이전에 사라진 합성 반례를 유지한다.
   대회 음영 ODD의 높이 가정과 ODD 밖 한계를 구분한다. 장착 z는 노면 기준 높이가 아니다.
4. **실제 객체 보존:** 대표 차량 유지와 벽/천장 제거는 확인했으나 독립 GT recall은 없다.
   음영 상자의 독립 군집·카메라 검출 부재를 T3의 신규 누락 정답으로 단정하지 않는다.
5. **충돌 경계:** 제거한 벽/천장을 자유 공간으로 취급하지 않는다. 구조물 경계 계약은
   판제와 별도로 정하며 현재 객체 메시지에 억지로 넣지 않는다.
6. **전체 패키지 이전:** 검출·OBB·추적·fusion을 작은 동작 보존 단위로 `asmc_lidar`에 이전한다.

구간·결과 소비·차량 제어는 판제 책임이다. 차선 피팅은 음영 ODD에서만 소비한다.
YOLO26 E8·3뷰·TensorRT 배포는 현재 카메라 기준선이다. 추가 모델/학습은 별도 후속이며
이전 B1a 실험 수락으로 대체하지 않는다.

## 공통 기반 연결 상태

[Main의 opt-in 좌표·GPS 경계](../../../docs/records/lidar/lidar-camera-frame-boundaries.md)에
관측 행렬·local 연관·map 가용성·typed 출력과 검증 보류를 정리했다. 터널 필터
입력은 raw/leveled 좌표와 행렬을 명시하고, 기존 전방 YOLO 보호와 후단 클래스
fusion을 구분해 이 지점에 연결한다. GPS에 따른 별도 검출 경로는 도입하지 않는다.
B1a 원본·고정 재호 브랜치는 대조군으로 보존한다.

## 참고 구현의 좌표와 pose 인계

[external 6개 좌표 구현 검토](../../../docs/records/camera/camera-lidar-coordinate-reference-review.md)를
함께 읽는다. 실제 camera Python/LiDAR C++의 54조건 투영 일치는 확인했지만 센서별
관측 시각 정렬은 별도다. bridge 수신 stamp와 scan ray 시각을 혼동하지 않고, 각 scan의
수평화 행렬을 역변환한다. 공개 후륜축 `base_link`와 내부 `lidar_leveled`는 다르다.

기존 bridge의 기본 OFF map TF와 GPS/IMU VehicleState는 안테나 위치를 쓰는 부분이
있다. HD맵/새 LiDAR의 후륜축 pose와 혼용하지 않는다. 판제와 권위 pose·lever·높이
정책을 확인하며 중복 map TF를 발행하지 않는다. KITTI stereo baseline/rectification,
nuScenes reference LiDAR 원점을 우리 MORAI 설정으로 복사하지 않는다.

시간 연결은 PR #72에서 공통 경로의 원본 source ns와 실제 callback receipt를 분리했다.
strict 100ms 보간과 Main bounded 300ms 보정을 구분하고 ROS/steady 시각을 혼용하지 않는다.
[연속 회귀·속도 진단](../../../docs/records/lidar/lidar-continuous-regression.md)에서
실제 처리 함수 재현, 지면 실패 fixture, bag 결과와 기본값 승격 보류를 확인한다.
기존 78회 실패 증거와 고정 재호 브랜치는 그대로 보존한다.

## 재호가 먼저 고를 작은 작업

1. **터널 core 연결:** Main 지면 이후·군집 이전에 재호 core만 연결한다. raw YOLO 보호와
   수평화 후 fusion 좌표를 각각 확인하고 동일 GPS 대조를 수행한다.
2. **벽과 객체 혼합 군집:** B1a 벽 근처 보호 후보에서 깊이·연결성과 돌출부를 비교한다.
   벽 전체 OBB 삭제 대신 객체 지지점 보존을 검증한다.
3. **OBB·추적·속도:** `PipelineTrace.hpp`의 voxel 대응·OBB·miss·KF 초기화와 source dt로
   군집 중심 이동, 오연결, 초기화 급변과 자세 변화를 분리한다. 속도 필터를 먼저 추가하지
   않는다. 현재 공개 velocity/dynamic validity는 false다.
4. **패키지 이전:** 기능을 작은 동작 보존 단위로 `asmc_lidar`에 옮긴다. 지면·군집·OBB·tracker·전방
   fusion은 [shadow 노드](shadow-pipeline.md)로 옮겼고, 운영 연결·구조물 분리·3뷰 fusion은 남아 있다.

## PR #72에서 새로 고정한 회귀 입력

- 현재 지면 fit 실패 + 직전 유효 평면 + roll context 10°: 합성 unknown 24점의 fallback 삭제와 common 보존을 비교했다. 입력은 이전 지면 높이에 놓인 일직선 점이며 장애물 정답이 아니다. 평면을 운반할 자세/이동 근거가 없으면 unknown으로 보존하는 보수적 정책이다. 실제 객체 소실 수정으로 확대하지 않는다.
- 노면 접촉 30cm 물체·보행자 재진입·차량·벽의 합성 입력: roll 후보의 미대응 관측이 99→106으로 증가했다. 지면 하부 삭제와 군집 합침을 따로 확인한다.
- 고정 shadow case-01: 유지점 수는 7037→7031이지만 후보는 23→13이다. 후보 수를 차량 정답 수로 사용하지 않고 point→cluster→OBB→연관을 추적한다.
- 주 bag의 roll 신규 분류 차이 11관측은 [원본 영상·점군 갤러리](../../../config/baselines/evidence/lidar_continuous_regression_v1/user-review/index.html)로 검토한다. 10관측은 car→unknown, 1관측은 반대이고 같은 객체의 연속 관측이 포함된다. 숫자 ID를 실행 간 객체 정답으로 사용하지 않는다.
- CA 상대속도 변화 >2m/s는 주 bag 657→599(-8.8%), 음영 527→555(+5.3%)다. 사용자는 음영의 증가를 이 지표에 한해 허용했다. 신규 분류 차이·객체 보존·속도 정확도 승인으로 확대하지 않는다. 공개 속도 validity는 false이며 기본 roll은 OFF다.

## 사용자 영상 검토에서 먼저 선택할 실제 사례

갤러리의 **연속 구간 1·2·3에 공통으로 남은 벽 조각 OBB**를 구조물/객체 분리의 첫
입력으로 사용한다. 같은 stamp의 raw→지면 제거→군집 지지점→OBB를 연결하고 벽 옆
차량 점을 보존한다. 작은 조각 OBB를 마지막 단계에서 통째로 삭제하는 정책부터 넣지 않는다.

**분류 10번의 작은 OBB**는 사용자 영상에서 차량을 충분히 표현하지 못하는 사례다.
차량 부분 관측/군집 분할인지, 다른 구조물 점이 차량의 ID·분류를 이어받은 것인지 먼저
확인한다. 실제 지지점이 없는 차량 전체 치수를 임의로 확장하지 않는다. 카메라 YOLO의
라벨 보완과 3D 점/OBB/연관 개선은 담당을 나눠 수행한다.

바닥 OBB 노이즈는 사용자 판독에서 구간 1은 pitch-only, 구간 2·3은 roll+pitch가
나았다. 정량 노이즈 제거율이 아닌 구간별 관찰로 기록한다. 속도 튐의 종합 개선 의견도
실제 속도 정확도와 구분한다. 합성 unknown 24점은 대응 사진이 없으며 실제 장애물
소실 사례로 사용하지 않는다.

## 구조물 표현과 속도 급변: LiDAR·판제의 책임

아래는 후속 설계/시험 제안이다. 이번 PR에서 새 구조물 출력이나 속도 필터를 구현한
것은 아니다. 갤러리는 [패키지 인계 README](../../../docs/handoffs/lidar-migration.md)의 브라우저 안내대로 먼저 연다.

### 벽을 꼭 객체 OBB로 만들어야 하는가

현재 Main의 일반 경로는 남은 점을 거리 기반으로 군집화하고 각 군집에 L-Shape OBB를
씌운다. 벽 조각이 나오는 것은 이 경로에 벽 점이 남았기 때문이며, 벽을 의미적으로
판별해 구조물 경계를 제공하는 구현이라고 보지 않는다. 정지 차량도 있으므로 정적이라는
이유만으로 구조물로 전환하지 않는다.

| 대상/목적 | 비교할 표현 |
|---|---|
| 차량·보행자·독립 장애물 | 객체 ID와 관측 근거가 있는 OBB/형상, 추적 상태 |
| 벽·가드레일·터널 표면 | 실제 지지 범위의 표면·선분·다각형 경계 또는 점유 grid/voxel |
| 구조물인지 객체인지 미확정 | 점/voxel 점유 근거 유지, 삭제/자유 공간으로 처리하지 않음 |

OBB로 구조물을 근사하는 것 자체가 오류는 아니다. 충돌 검사나 debug에 쓸 수 있지만
잘게 쪼갠 벽에 차량용 ID·속도를 부여하거나 거대한 벽 OBB로 도로를 덮는 표현은 피한다.
벽을 객체 목록에서 분리하더라도 충돌 경계는 계속 제공한다. raw voxel을 남기는 것만으로
판제 계약이 완성되지는 않으며 좌표·stamp·해상도·점유/미관측 의미와 소비 경로를 정한다.
GPS 독립 local 관측과 필요한 map 변환 경계는 기존 설계를 따른다.

참고 구현으로 Autoware는 [점군 기반 occupancy grid](https://autowarefoundation.github.io/autoware_universe/latest/perception/autoware_probabilistic_occupancy_grid_map/)와
[라벨/군집 기반 객체 형상 추정](https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_shape_estimation/)을 별도 모듈로 제공한다.
우리의 객체/구조물 경계 분리 제안은 이 구분을 참고한 설계이며 유일한 정석을 주장하지 않는다.

### 박스가 흔들리면 속도도 흔들리는가

현재 `Clustering.cpp:Clustering`은 관측 군집점의 평균을 만들고,
`Tracking.cpp:Objectinfo_for_km`은 이를 `f32_CoreX/Y/Z`에 넣는다. L-Shape의 기하학적 중심은
별도 `f32_OBB_CenterX/Y/Z`다. `CompareCentroids`는 군집 중심을 관측으로 연관/KF에
넣고 `Kalman.cpp:kalman_filter_update`는 상대속도를 추정한다. KF 후 공개 상태의
Core X/Y는 필터 위치이므로 raw 군집 중심·OBB 중심·필터 위치를 구분해 기록한다.

시야·가림·지면 제거로 같은 차량의 관측 면이 달라지면 OBB와 평균 군집 중심이 함께
움직일 수 있다. 하지만 OBB 크기 변화가 직접 속도를 계산하는 구현은 아니다.
군집 분할/합침, 다른 객체 연결, KF 초기화, miss 후 재관측, 자차 운동/시간 처리도 함께
분리한다. 주 bag의 최대 급변은 pitch에서 45.75m/s(초기화 후보, 기록 중심 변화 3.22m,
scan dt 50.35ms), roll에서 36.20m/s(초기화 후보, 중심 변화 2.27m, scan dt 62.38ms)다.
중심 변화는 기록된 3D 상태의 거리이며 실제 이동이나 2D 속도의 독립 정답이 아니다.
박스 치수 변화만이 원인이라고 확정하지 않는다. 관측 공백이 있을 때 초기 속도에 사용한
dt가 마지막 실측 이후 경과시간과 맞는지도 고정 trace로 확인한다.

- **LiDAR:** 지지점·군집/연관을 개선하고, 위치/형상 안정성과 초기화·재관측 dt·자차 보상을
  검증한다. 결과의 source stamp·관측/예측 구분·속도 유효성/품질을 계약으로 전달한다.
- **판제:** age·유효성·불확실성을 확인해 소비하고 기동 결정에 히스테리시스/보수적 여유를
  둔다. 검증되지 않은 속도를 그대로 장래 궤적에 쓰거나 unknown을 0속도/static으로 간주하지 않는다.

판제의 소비 안정화와 LiDAR 관측/추적 정확도 개선은 각각 검증한다. 현재 typed 공개
velocity/dynamic validity는 false로 유지한다.

### 위치·속도 관측 조사와 재호의 비교 순서 — 2026-10-08

**칼만 필터는 추정 수단이며 관측 군집의 평균이 물체의 고정된 중심이라는 보장은 없다.**
정지 차량도 뒤쪽 면만 보이다가 옆면이 드러나면 점 밀도와 평균 중심이 이동한다.
이 관측 편향은 영평균 잡음과 다르므로 평활화를 강화하는 것만으로 해결하지 않는다.
현재 `Kalman.cpp:kalman_filter_update`는 CA 상태의 x/y를 관측하고 고정
`R = 0.01 I`, `Q = 0.003 I`를 사용한다. OBB 기하 중심으로 관측을 바꾸더라도
매 프레임 부분 점군으로 만든 박스의 치수·중심이 변하면 같은 문제가 남는다.

아래 후보는 **공식 구현·논문을 읽은 설계 제안**이며 우리 VLP-16/MORAI 입력에서
빌드·정확도·부하를 검증하거나 이번 PR에 구현한 기능은 아니다.

| 근거의 종류 | 참고 내용 | 우리에게 적용할 때 확인할 것 |
|---|---|---|
| [Autoware 공개 추적 구현](https://github.com/autowarefoundation/autoware_universe/blob/main/perception/autoware_multi_object_tracker/README.md) | 객체 연결 + EKF, 클래스별 운동 모델과 Mahalanobis/거리 등의 gate | 공개 프로젝트의 구현 사례다. 업계 전체의 유일한 표준이나 우리 센서의 성능 보장이 아니다. |
| [Autoware 공개 형상 추정](https://github.com/autowarefoundation/autoware_universe/blob/main/perception/autoware_shape_estimation/README.md) | L-shape·클래스별 형상과 참조 방향/치수 보정 | 우리도 L-shape가 있다. 새 도입보다 이전 형상·방향을 활용한 위치 관측의 일관성을 비교한다. |
| [AB3DMOT 연구 기준선](https://arxiv.org/abs/2008.08063) | 3D 검출 + KF + Hungarian 연결 | 학습 검출의 box 중심과 보이는 raw 점의 평균은 같은 관측이 아니다. 논문의 tracker 속도를 전체 추론 속도로 사용하지 않는다. |
| [확률적 3D 추적 연구](https://arxiv.org/abs/2001.05673) | 관측/상태 불확실성을 반영하는 KF와 Mahalanobis 연결 | 관측 오차와 연결 임계값을 독립적으로 비교한다. 점 수만으로 보정되지 않은 confidence를 만들지 않는다. |
| [CenterPoint 연구](https://openaccess.thecvf.com/content/CVPR2021/papers/Yin_Center-Based_3D_Object_Detection_and_Tracking_CVPR_2021_paper.pdf) | 학습된 3D 중심·크기·방향·속도 검출 | 별도 GT·학습/센서 적합성·GPU 부하가 필요하다. 공개 benchmark 결과를 우리 16채널 입력에 그대로 적용하지 않는다. |

재호가 작업을 고를 때는 다음 순서를 권한다. 각 후보는 고정 입력의 기존 평균 중심
기준선과 별도 커밋·실험으로 비교하며 필터·군집·연결을 한 번에 바꾸지 않는다.

1. **시간·초기화·연결:** 최대 급변 trace에서 마지막 실측 이후 객체별 경과시간과
   scan dt, miss 후 재관측/초기 속도, source stamp와 자차 보상을 확인한다. 시간 오류와
   다른 객체 연결을 필터 평활화로 숨기지 않는다. 현재 기록은 원인 후보이지 확정 결함이 아니다.
2. **구조물·객체 군집:** 갤러리 구간 1·2·3의 벽 조각과 분류 10번을 우선 고정한다.
   벽 혼입·분할/합침·부분 관측을 분리하고 구조물 충돌 근거와 실제 객체 지지점을 보존한다.
3. **일관된 위치 관측:** 평균 중심과 이전 방향/형상에 기반한 중심 또는 관측 가능한
   면/모서리 기준점을 비교한다. 기준점 변경과 물체 중심 보정의 불확실성을 기록하고,
   보이지 않은 차량 전체 치수를 검증 없이 늘리지 않는다. 관측 점유 형상과 추정 기준점은 구분한다.
4. **불확실성·연결:** 형상 잔차·가림·예측 불일치 등을 반영한 관측 오차와
   Mahalanobis gate를 각각 비교한다. CV/CA/IMM 등의 모델 비교는 관측·시간 오류를
   먼저 분리한 뒤 수행하며 EKF/UKF로 바꾸는 것 자체를 정확도 개선으로 간주하지 않는다.

수락에는 위치·속도 오차, 가속/감속에 대한 반응 지연, ID 연속성·오연결·재등장을
함께 사용한다. 합성 정답 구간과 실제 bag의 GT 없는 안정성 통계를 분리하고,
속도 변화 >2m/s 횟수만 줄었다고 정확도가 개선됐다고 판정하지 않는다.
GPS 조건만 바꾼 local 상태 동등성·관측별 좌표와 시간 계약도 계속 검사한다.

입력·결과·검증 범위는 [연속 회귀 문서](../../../docs/records/lidar/lidar-continuous-regression.md)와 그 baseline을 따른다.

### MORAI 속도 GT 기준 — 2026-10-06 전달 Q&A 반영

운영진은 Object Info의 순간 선속도를 **후륜 차축 중심 기준**으로 답변했고 위치 미분보다
Velocity_XYZ의 직접 사용을 권장했다. 원문 출처·시각, 높이·출제 범위와 변환 조건은
[객체 에셋·속도 GT 안내](../../../docs/competition/obstacle-assets-and-labels.md#lidar-속도-검증에-연결할-기준)에 모았다.

현재 브릿지의 `/Object_topic` velocity는 km/h이므로 이 수집 경로에서는 성분별 `/3.6`이
필요하다. 이미 m/s인 Ego velocity와 혼동하지 않는다. 후륜축 속도를 부분 군집 평균이나
추정 OBB 중심 속도와 그대로 비교하지 않고 기준점·좌표·source 시간·자차 보상을 맞춘다.
기하 중심 속도가 필요하면 같은 시점의 객체 각속도·제원으로 `v_P = v_R + omega × r_RP`를
사용한다. 현재 ObjectStatus에는 객체 yaw rate가 없으며 Ego IMU로 대신하지 않는다.
독립 각속도/기준점이 없으면 중심 GT는 미확보로 기록한다.

재호의 우선 과제는 **GT 단위/기준점 정렬 → 초기화·재관측 dt/연결 검증 → 위치 관측 개선**이다.
Object GT는 개발 평가 전용이며 본선 입력으로 사용하지 않는다. 이번 문서 반영은 속도
알고리즘 수정이나 공개 velocity/dynamic validity 승격이 아니다.
