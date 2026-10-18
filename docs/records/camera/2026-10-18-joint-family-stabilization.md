# 원본 시각 IMU·폭 공동 계산과 family 안정화

## 변경과 검증 범위

이 기록은 [이전 폭 완화 후보](2026-10-18-imu-width-policy.md)의 측방 잔차와 지원 범위 손실을
수치 계산에서 개선하는 후속 작업이다. 모델·engine·해상도·원본 header·공개 메시지/서비스/MD5는 유지한다.
학습·시간 누적·병진/yaw 정렬·높이 추정은 추가하지 않는다. 기본 전환 상태는 아래 실시간 결과로 판정한다.

지면 법선 거리는 **0.36m**다. `width_prior_strength=0.35`는 단위 없는 폭 당김 강도다.
원본 시각 IMU가 세 뷰에서 유효하면 prior0과 공동 폭 계산을 적용한다. 근거 부족 시 전 뷰를
기존 M0/prior0.35로 처리한다. 100ms 보간 간격·35ms 뷰 시각차·350ms freshness를 유지한다.

기존 fitter는 곡선과 폭을 번갈아 갱신했다. 같은 robust 반복 횟수 안에서 폭과 곡선이 서로 영향을
주면서 잔차가 남았다. 새 후보는 같은 B-spline 제어점과 관측한 폭을 하나의 가중 최소제곱 문제로
계산한다. 폭2.4–4.6m의 경계에 닿으면 해당 폭을 고정하고 나머지를 다시 푼다. 반복 횟수·근거 점 수·
곡선 smoothing·관측/추정 구분·support 판정을 줄이거나 완화하지 않았다.

공동 계산은 측정된 황색 기준 역할이 있을 때만 사용한다. 두 흰선만 있는 프레임은 기준선과 폭의
분리가 모호하므로 기존 계산을 유지한다. 관측하지 못한 폭을 측정값으로 만들지 않는다.
LaneCPP의 새 네트워크나 학습 loss를 가져온 변경은 아니다. 현재 family 표현을 유지한 수치 계산 개선이다.

## 동일 원본 시각226묶음

같은 원본 RGB·class/confidence grid·IMU를 사용해 M0, 이전 M1+prior0, 새 공동 계산 M1을 비교했다.
Noetic에서 세 구성 모두226/226 유효했다. 새로운 역할 오류·지원 손실·폭 범위 위반이 없었다.
기존의 엄격한 품질 판정도 통과했다. 허용 pixel이나 support 손실 기준을 늘려 통과시키지 않았다.

| 지표 | M0 | 이전 M1+prior0 | 공동 계산 M1 |
|---|---:|---:|---:|
| 오른쪽 흰선의 프레임별 거리 중앙값 분포 p95 | 0px | 2px | 0px |
| 왼쪽 흰선 동일 지표 | 1px | 1px | 1px |
| 0–15m 지원 범위 손실 | — | 1프레임 약3.1cm | 0건 |
| 공통 위치의 시간 변화량 p95, 3후보 비교 | 24.64cm | 4.03cm | 4.10cm |
| 프레임별 inlier 잔차 p95의 중앙값 | 8.50cm | 12.44cm | 8.25cm |
| 프레임별 inlier 잔차 p95의 분포 p95 | 17.70cm | 17.69cm | 15.04cm |

Inlier 집합은 각 후보의 기존 robust 판정으로 결정된다. 위 잔차는 관측 근거와 피팅의 일치이며
독립 GT 오차가 아니다. 시간 변화는 개발용 Ego pose로 같은 지면 위치를 SE(2) 정렬한 뒤 측정한 **내부 일관성**이다.
독립 GT의 절대 위치 오차가 아니다. 네 후보 공통 support로 재판정한 수치는 반올림 차이와 별개로
비교 구간이 달라질 수 있다. 최종 판정 JSON에 후보 이름의 remap과 원본 비교 구성을 기록했다.

| 운동 구간 | M0 | 공동 계산 M1 |
|---|---:|---:|
| 첫 가속 | 28.23cm | 6.42cm |
| 첫 제동 | 55.53cm | 9.65cm |
| 재가속 | 25.14cm | 3.87cm |
| 두 번째 제동 | 22.27cm | 3.35cm |
| 정속 | 1.93cm | 1.51cm |
| 작은 조향 | 1.34cm | 1.44cm |

SUNNY/FOGGY×11/13/15시의 구형 bag216묶음에서도 새 무효 family가 없었다.
터널 진입/내부/출구의 시간 변화 p95는2.27/21.76/20.84cm에서1.90/5.61/6.66cm로 감소했다.
이 bag은 receive 시각만 있어 원본 시각 정합의 증거로 사용하지 않는다.

## IMU 도착 순서와 M0 전환 진단

첫 교차 시험에서 보정 적용률은99% 이상이고 공급·readiness 기준을 통과했다. 그러나 잠깐 M0로
돌아간 인접 결과의15m 곡선 차이가 최대약32cm인 사례를 별도로 확인했다. 이를 평균 Hz로 상쇄하지 않는다.
IMU 실패 trace에서 camera source가 마지막 IMU source보다3ms 앞서 있고, 마지막 수신 age가
약36–47ms인 사례가 있었다. 다음 실제 IMU가 도착하기 직전에 조회한 경우였다.

후속 후보는 세 뷰 전체에 합계10ms 이내의 대기 예산을 둔다. source가 유효한 IMU 꼬리보다 앞선
경우에만 Condition을 기다리고, lock을 놓아 wire callback이 새 샘플을 연결하도록 한다. 실제 두 샘플이
확보되면 원래100ms bracket·350ms receipt·35ms source skew 검사를 다시 통과해야 한다.
시간을 넘기거나 오래된 입력/세션 오류/가림/잘못된 시각이면 기존 M0 복귀 정책을 유지한다.
최신 자세 대입·외삽·freshness 연장·곡선 평활화는 하지 않는다. `wait_requested/recovered/unresolved`를
진단 counters에 남긴다. 개선 전후의 실행/설정 SHA를 구분하고, 변경 후 후보를 다시 반복 측정한다.

## 실시간 시험

M0와 공동 계산 M1을 SUNNY/FOGGY 각각3회60초 교차 측정해12회 모두 통과했다.
10ms 대기 후보도 두 날씨 각각3회60초를 통과했다. 최종 설정의 실제 첫10분 시험은
FOGGY13시·전체 시각화 수요·rqt1개·Main/LiDAR 부하로 수행했다.

| 첫10분 결과 | 실측 | 판정 |
|---|---:|---|
| 정상 model stream의 최소5초 공급 | YOLO/차선≥10Hz, 신호등9.8Hz | 통과 |
| 유효 family 최소5초 공급 | 13.4Hz | 통과 |
| 모델 입력 callback→발행 p95 최댓값 | 136.85ms | 통과 |
| 유효 family 최대 수신 공백 | 194.82ms | 통과 |
| 전체 GPU VRAM peak | 65.21% | 통과 |
| 원본 시각 IMU 적용 | 8,705/8,720프레임(99.83%) | 복귀15프레임 별도 보존 |
| 정상 ACTIVE readiness 저하 | 1건 | **미통과** |

Readiness 사건은 `source_stale`이며 family source age370.78ms, 수신 후 age87.36ms였다.
피팅은 유효 결과를 계속 발행했고, 같은 시점의 메시지 조립에서 generation2 GC21.08ms를 기록했다.
GC가 기여한 정황이며 전체 지연의 유일한 원인으로 확정하지 않는다. 350ms freshness를 늘리지 않았다.

후속 수정은 단일 직렬 worker·non-latched Publisher의 발행 전용 메시지/Point32 버퍼를 재사용한다.
설치된 Noetic은 `publish()` 반환 전에 immutable bytes로 직렬화한다. 다른 계산/legacy 호출은
기존 fresh-message 소유권을 유지한다. 같은 wire bytes·유효/무효 전환·길이 축소/복귀·자세별xyz·
이전 직렬화 데이터 보존을 검사했다. 원래 계산이나 message MD5를 변경하지 않았다.

**후속 할당 개선의 실시간 시험과 기본 전환은 대기 중이다.** 재시험 시작 전 Ego가 시험 위치에서
22.53m 이동하고 키보드/R 기어로 움직이는 것을 guard가 확인해, 모델 시험을 시작하지 않고 중단했다.
외부 주행을 덮어쓰지 않도록 차량 이동·모드·기어 변경과 원위치 복원을 보류했다.
기존 운영 runtime의 M0 `camera.launch`를 다시 실행했다. 세 모델 ACTIVE·loaded·prepared·오류0과
결과 토픽별 단일 발행자를 확인했다. 새 설정의 enabled 기본값은 여전히false다.
[기존 실행 복원 확인](../../../config/baselines/evidence/camera_imu_joint_v1/baseline-restored-status.json)을 보존했다.

실험 경로는 `data/runtime/camera-imu-stabilize-20261018-r1/`이다. `live/original-state.json`에 처음
위치·모드·기어·날씨·시간을 보존했다. 이후 외부 주행으로 복원 기준을 다시 확인해야 한다.
미착수 guard 실패는 `live/pool-matrix/fixture-condition-failure.json`에 보존했다.
완료한19회와 미착수 조건을 [집계](../../../config/baselines/evidence/camera_imu_joint_v1/runtime.json)에
구분했다. 초기10분 실패를 후속 통과나 기본 탑재 완료로 표시하지 않는다.

Noetic CPU56개는 모두 통과했고 누락·skip은 없다. 실제 계산부226묶음×보정/복귀452대조의
좌표 차이는0m였다. [품질](../../../config/baselines/evidence/camera_imu_joint_v1/quality.json),
[계산부](../../../config/baselines/evidence/camera_imu_joint_v1/runtime-parity.json),
[CPU 회귀](../../../config/baselines/evidence/camera_imu_joint_v1/cpu-regression.json)를 따른다.
모델 callback 지연은 촬영→표시 지연과 구분한다. 원본 IMU/피팅 trace는 같은 생산 세션·발행 stamp로
대응하며, 원본 report의 과거 lane10Hz 목표 표시는 실제 설정15Hz로 별도 정규화해 보존했다.
10Hz 최소 공급 통과를 모든5초 창의15Hz 목표 달성으로 설명하지 않는다.

## 실행과 남은 과제

기본 전환 판정 전에는 실험 workspace의 `_imu_compare.launch`에서만 새 후보를 실행한다.
M0 대조는 `enabled:=false`, 기존 폭 계산과 IMU만 대조하려면
`enabled:=true corrected_width_prior:=0.35 joint_width_fit:=false`를 사용한다.
일반 `camera.launch`와 동시에 켜지 않는다. 시각화 명령은 기존 `view.launch`를 유지한다.

남은 과제는 3뷰 중첩의 중복 가중/보정 차이, 실제 가림과 출구 소실, 터널에서 사용 가능한
상대 이동 입력을 확보한 뒤의 병진/yaw 정렬이다. 과거 역할 제거5건을 이번 실시간 정상 영상의
통과로 해결했다고 표시하지 않는다. 고정 RGB BEV 배경은 M1 평면으로 warp하지 않았으므로
보정 family의 절대 밀착도 판정에는 원본 RGB와 후보별 재투영을 사용한다.

재개 후 순서는 외부 주행 종료 확인→새 정차 기준/복원 장부 고정→SUNNY/FOGGY 할당 수정 대조→
FOGGY10분 전체 GUI 확인→통과 시 정본 enabled=true·운영 runtime 설치/재빌드·일반 launch 확인이다.
이번 미완료 단위는 로컬 커밋만 보존하며 push·PR을 게시하지 않는다. 이전 과제 worktree와 모델은 유지한다.
