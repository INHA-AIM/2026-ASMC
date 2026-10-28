# LOCAL-1: GPS 독립 local 경로·MPC 실험

상태: 구현·검증 진행. 운영 기본 전환 및 터널 통과 수락 아님.

기준선: PR126 `1125582c`; 착수 시 main `3b0bc98f`. 기존 S3 r10/r11 실패를
보존한다. 목표는 semantic/family/local OBB와 속도·IMU의 공통 시각에서
경로를 생성하고 기존 mpc_core를 별도 local 입력 경계로 사용하는 것이다.
NORMAL 및 기존 음영 생산자는 수정하지 않는다. 이강태 1028 브랜치와 독립이다.

## 고정 조건

- 목표25km/h, 감속1.5m/s²에서 출발. 곡률·관측 종료·충돌·지연이 더 낮은 상한을 결정한다.
- YOLO bbox 차로 투영과 검출 즉시5km/h 규칙은 새 local 경로의 입력이 아니다.
- VehicleSpec 폭1.892m와 SDK bounding box 폭2.1608348m를 구분한다.
  bounding box가 충돌 collider와 같은지는 미검증이다. 물리 외곽 검증 전
  더 큰 박스를 실험 경계로 사용하며 일반 판제 치수는 바꾸지 않는다.
- nominal 지면0.36m, 후륜축 기준 전3.845/후0.790m, 축거3m.
- S3 근거리 미관측 보조는 명시적 비교 옵션이며 이때만5km/h 상한을 유지한다.
- 외부 GT는 평가 전용. GPS·map pose를 local 제어에 사용하지 않는다.
- 벽 RANSAC/출구 검출 추가 없음. 벽 소실은 출구 증명이 아니다.

## 참고 소스

외부 코드는 workspace external/baselines에 고정하고 ROS 실행 의존성으로 넣지 않는다.
- `sbpl`: `4d654845aa3b92aa1b4282a342e9011bac95aeb9`; license files: []
- `commonroad-reactive-planner`: `6f2f0d91d4b9370f8e716349833308271365e7d1`; license files: ['LICENSE']
- `autoware_universe`: `bafa06574d97e6255bc7f5d8c36683574fa07ebb`; license files: ['LICENSE']

SBPL/state lattice의 실행 가능한 motion primitive와 search, CommonRoad의
후보 검증 분리, Autoware Path Sampler의 후보→제약→비용 구조를 참고한다.
현재 구현·수치 성능은 해당 논문의 결과와 구분한다.

## 재현 저장

새 run: `local1-20261028-audit01`, D artifacts. 사용자 명시 승인으로 이번
LOCAL-1은50GiB 미만 저장 허용, 전역 정책 변경 없음. D→G 이동 중인
SensorData는 수정하지 않는다. 실험별 ID를 새로 등록하며 기존 run은 동결 보존한다.

## 완료 조건

좌우 단일 장애물 각3회 통과·복귀, NORMAL 회귀 없음, 계획 p95≤50ms,
MPC20Hz/명령 최대공백≤250ms. 입력·경로·마진·속도·제어 실패를 각각
기록한다. 통과/치수/제동 검증 전에는 기본 활성화하지 않는다.
