# B1a 검증에서 공통 기반 구현으로 넘기는 과제

2026-10-08 기준. 검증 대상은 재호 `8dde9b6`, 운영 대조군은 main의 nested `Main`이다.
[44회 검증과 원본 증거](../../handoffs/perception/camera-lidar-b1a.md)를 보존한다.
아래 제안은 구현 완료나 필터 운영 수락을 뜻하지 않는다.

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
소비하지 않는다. 별도 YOLO·21% crop·주행 코드는 유지한다.

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
COCO 확대, YOLO26, 카메라 수·주기, 학습·TensorRT는 독립적인 후속 실험이다.

## 공통 기반 연결 상태

[Main의 opt-in 좌표·GPS 경계](../../handoffs/perception/lidar-camera-frame-boundaries.md)에
관측 행렬·local 연관·map 가용성·typed 출력과 검증 보류를 정리했다. 터널 필터
입력은 raw/leveled 좌표와 행렬을 명시하고, 기존 전방 YOLO 보호와 후단 클래스
fusion을 구분해 이 지점에 연결한다. GPS에 따른 별도 검출 경로는 도입하지 않는다.
B1a 원본·고정 재호 브랜치는 대조군으로 보존한다.

## 참고 구현의 좌표와 pose 인계

[external 6개 좌표 구현 검토](../../handoffs/perception/camera-lidar-coordinate-reference-review.md)를
함께 읽는다. 실제 camera Python/LiDAR C++의 54조건 투영 일치는 확인했지만 센서별
관측 시각 정렬은 별도다. bridge 수신 stamp와 scan ray 시각을 혼동하지 않고, 각 scan의
수평화 행렬을 역변환한다. 공개 후륜축 `base_link`와 내부 `lidar_leveled`는 다르다.

기존 bridge의 기본 OFF map TF와 GPS/IMU VehicleState는 안테나 위치를 쓰는 부분이
있다. HD맵/새 LiDAR의 후륜축 pose와 혼용하지 않는다. 판제와 권위 pose·lever·높이
정책을 확인하며 중복 map TF를 발행하지 않는다. KITTI stereo baseline/rectification,
nuScenes reference LiDAR 원점을 우리 MORAI 설정으로 복사하지 않는다.

시간 연결은 PR #72에서 공통 경로의 원본 source ns와 실제 callback receipt를 분리했다.
strict 100ms 보간과 Main bounded 300ms 보정을 구분하고 ROS/steady 시각을 혼용하지 않는다.
[연속 회귀·속도 진단](../../handoffs/perception/lidar-continuous-regression.md)에서
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
4. **패키지 이전:** 기능을 작은 동작 보존 단위로 `asmc_lidar`에 옮긴다. 좌표 코어는
   준비됐지만 지면·군집·tracker 노드가 이미 이전된 것은 아니다.

## PR #72에서 새로 고정한 회귀 입력

- 현재 지면 fit 실패 + 직전 유효 평면 + roll context 10°: 합성 unknown 24점의 fallback 삭제와 common 보존을 비교했다. 입력은 이전 지면 높이에 놓인 일직선 점이며 장애물 정답이 아니다. 평면을 운반할 자세/이동 근거가 없으면 unknown으로 보존하는 보수적 정책이다. 실제 객체 소실 수정으로 확대하지 않는다.
- 노면 접촉 30cm 물체·보행자 재진입·차량·벽의 합성 입력: roll 후보의 미대응 관측이 99→106으로 증가했다. 지면 하부 삭제와 군집 합침을 따로 확인한다.
- 고정 shadow case-01: 유지점 수는 7037→7031이지만 후보는 23→13이다. 후보 수를 차량 정답 수로 사용하지 않고 point→cluster→OBB→연관을 추적한다.
- 주 bag의 roll 신규 분류 차이 11관측은 [원본 영상·점군 갤러리](../../../config/baselines/evidence/lidar_continuous_regression_v1/user-review/index.html)로 검토한다. 10관측은 car→unknown, 1관측은 반대이고 같은 객체의 연속 관측이 포함된다. 숫자 ID를 실행 간 객체 정답으로 사용하지 않는다.
- CA 상대속도 변화 >2m/s는 주 bag 657→599(-8.8%), 음영 527→555(+5.3%)다. 사용자는 음영의 증가를 이 지표에 한해 허용했다. 신규 분류 차이·객체 보존·속도 정확도 승인으로 확대하지 않는다. 공개 속도 validity는 false이며 기본 roll은 OFF다.

입력·결과·검증 범위는 [연속 회귀 문서](../../handoffs/perception/lidar-continuous-regression.md)와 그 baseline을 따른다.
