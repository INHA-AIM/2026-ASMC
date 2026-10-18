# 원본 시각 IMU·폭 사전값 완화 구현과 품질 판정

## 결과와 적용 상태

**실험 프로필 구현은 완료했지만 품질 수락은 미통과다. 운영 M0를 유지한다.**
원본 시각 IMU가 세 뷰에 적용된 프레임만 prior0을 사용하며, 근거가 부족하면 원래 고정 평면과
prior0.35로 복귀한다. 매 프레임 읽기 전용 설정을 선택하고 폭2.4–4.6m·역할·support 판정은 유지한다.
모델·TensorRT·해상도·공개 ROS 계약은 변경하지 않았다.

작업 위치는 `2026-ASMC-camera-motion` / `experiment/camera-imu-motion`이다.
`08e4a283`에 정책과 fitter 회귀, `53bdd54d`에 품질 판정·ROS parity·실패 재현 도구를 저장했다.
`440a6d32`에 무효 family의 실패 보존과 source clock 역행 후 새 bracket 요구 검사를 추가했다.
데이터 이관의 Docker 스크립트와 LiDAR 연구 worktree는 포함하지 않았다.

## 동일 입력의 네 후보

[이전 터널 기록](2026-10-18-tunnel-imu-motion-and-fit.md)의 원본 시각226묶음을 재사용했다.
M0+prior0.35, M1+prior0.35, M0+prior0, M1+prior0의 네 후보를 비교한다.
과거 smoothing 대조까지 포함했던5후보 집계와 달리, 이번 시간 지표는 지정한4후보 모두에서
공통으로 유효한 위치·역할에 한정해 재계산했다. 따라서 과거 표와 소수점 값이 조금 다를 수 있다.

다음 값은 같은 지면 위치를 개발용 Ego pose로 SE(2) 정렬한 뒤 계산한 프레임 쌍별 변화 p95의
분포 p95다. **독립 GT 위치 오차나 대회용 Ego motion 입력으로 해석하지 않는다.**

| 구간 | M0 | M1+prior0 | 판정 |
|---|---:|---:|---|
| 첫 가속 | 28.37cm | 6.56cm | 30% 이상 감소 |
| 첫 제동 | 54.47cm | 9.45cm | 30% 이상 감소 |
| 재가속 | 25.14cm | 3.84cm | 30% 이상 감소 |
| 두 번째 제동 | 22.23cm | 3.16cm | 30% 이상 감소 |
| 정속 | 1.93cm | 1.82cm | 악화1cm 이내 |
| 조향 | 1.34cm | 1.43cm | 악화1cm 이내 |

가감속·정속·조향 지표는 모두 통과했다. 새 무효 family·역할 뒤바뀜·폭 범위 초과도 없었다.
그러나 다음 두 항목이 합의한 품질 기준을 넘었다.

1. **오른쪽 카메라 흰선:** 관측 선의 프레임별 segmentation 거리 중앙값 분포 p95가
   M0의0px에서 후보2px로 악화됐다. 허용한1 model-grid pixel을 넘었다.
   전방 흰선0→0px, 전방 황색선1→0px, 좌측 흰선1→1px, 좌측 황색선0→0px였다.
   양쪽 모두 평가 가능한 근거가 없던 뷰/클래스는 검증한 것으로 추가하지 않는다.
2. **0–15m 지원 범위:** 첫 가속의 index18에서 왼쪽 역할의 기존 지원 중0.03133m가 사라졌다.
   다른 위치에 범위가 늘어난 것으로 이 손실을 상쇄하지 않는다. 수치 허용1e-6m를 넘었다.

프레임 간 폭 변화 p95는 왼쪽2.57→1.27cm, 오른쪽1.90→0.82cm로 감소했다.
폭 흔들림 악화가 아니므로 조건부 prior0.10 탐색은 시행하지 않았다. 전체 파라미터 탐색으로 확대하지 않았다.

## 실패 단계 확인과 기존 소실5건

실패 예시는 고정 metric으로 선택한 오른쪽 흰선 악화 상위3묶음(index0/1/2)과 지원 손실 index18이다.
같은 원본 RGB에서 segmentation과 네 후보를 나란히 표시했다. 후보별 자체 평면으로 재투영했고
원본 이미지·그리드 SHA를 검사했다. 고정 평면 BEV 배경으로 M1을 판정하지 않는다.

오른쪽 흰선의 raw 광선과 병합 node는 예시4개에서 재투영 중앙값0px였다. 최종 shared family에서만
후보 중앙값약1.80–2px의 차이가 생겼다. 이는 이번 예시에서 **근거 추출 이후의 공동 피팅에 잔차가
남는다는 증거**다. 고정 폭·공통 형상·카메라 사이 근거 정합의 기여는 추가 분리가 필요하다.
카메라 보정이나 일정한0.36m 지면 가정이 절대적으로 정확하다는 증명은 아니다.

기존 단일 경계 제거 실패5건은 같은 그리드 SHA로 M0에서 모두 재현했다. 동일 제거를 적용한
receive 시각 M1+prior0 대조에서는5건 모두 유효했다. 그러나 구형 bag에는 원본 시각이 없으며,
track 연결 뒤에 역할 근거를 제거한 fitter 스트레스다. 실제 영상 가림·source M1·운영 readiness
해결로 표시하지 않는다. 나머지 전체 가림 조합을 이번에 재검증한 것도 아니다.

## 실제 계산부·회귀 검사

별도 ROS master11366에서 GPU 없이 실제 `LaneVectorizerNode._compute_family_frame`을 호출했다.
원본 시각 보간으로 구한 자세를 계산부에 주입했고, 정확한 wire 연결·세션·freshness는 별도 adapter 시험으로 확인했다.
226묶음의 조합 후보와 M0 복귀를 반복한452대조에서 유효성·역할·클래스·support가 같고 좌표 차이는0m였다.
입력 SHA 검사를 포함한 `production-parity-r2`를 최종 결과로 사용한다.

읽기 전용 프레임 설정·동시 선택·잘못된 prior·관측하지 못한 역할·M0 복귀 동등성을 검사했다.
Noetic에서는 M1→M0→M1, 한 뷰 source 누락, 잘못된 wire/생산자, 계산 중 모드 전환,
이전 세대 발행 차단, timing serialization·재시작·stale·readiness 회귀를 확인했다.
품질 판정기는 빈 자료·누락 지표·무효 family를 통과시키지 않으며 내부 지원 공백을 보존한다.
호스트44개와 Noetic의 runtime24개·정책4개·판정기4개가 통과했다. 문서 이전·실행 명령·Python3.8
구문 검사도 통과했다. [실행 manifest](../../../config/baselines/evidence/camera_imu_width_v1/execution-manifest.json)에
검사 수와 실제 사용한 코드/설정·결과 SHA를 고정했다.

**품질 미통과로 SUNNY/FOGGY60초×3 및 전체 GUI10분 수락은 건너뛰었다.**
해당 후보의 Hz·latency·VRAM 수락을 통과한 것으로 표시하지 않는다. 공개 master의 운영 카메라와
bridge는 재시작하거나 시험 버전으로 교체하지 않았다. rqt·차량 제어·이동·날씨 변경은 수행하지 않았다.
Hosted CI·push·PR도 시행하지 않았다.

## 산출물·재현·다음 과제

- 원본과 결과의 논리 경로: `data/runtime/camera-imu-width-20261018-r1/`.
  데이터 이관 후 실제 위치는 `D:\morai dataset\shahn03\data\runtime\camera-imu-width-20261018-r1`이다.
  기존 workspace 링크와 컨테이너 `/data/runtime`를 유지한다.
- 품질 판정 `quality-r2/report.json`, 계산부 일치 `production-parity-r2/report.json`, 실패 `failures-r1/report.json`.
  이전 r1 결과는 보존하며 최종 결과와 구분한다.
- [고정 evidence](../../../config/baselines/evidence/camera_imu_width_v1/quality.json),
  [계산부 parity](../../../config/baselines/evidence/camera_imu_width_v1/runtime-parity.json),
  [실패 재현](../../../config/baselines/evidence/camera_imu_width_v1/failures.json).
- [E395 실험 갤러리](http://127.0.0.1:8000/camera-imu-width-policy-20261018-r1/index.html).
  E393은 실패 장면, E394는 품질 판정이다. 이번3페이지의 정적 링크13개는 정상이며 HTTP200을 확인했다.
  기존 실험 ID를 유지해 전체 목차를 갱신했다. 이관된 artifacts 전체 탐색/링크 검사는 지연으로 중단했다.
  이전 전체 검사에서 남은 다른 페이지의25개 링크 누락은 이번 작업의 해결·통과 항목이 아니다.

전용 checkout과 Noetic 메시지를 준비한 환경에서 아래 도구의 `--help`로 직접 재현한다.
출력은 새 폴더를 사용하며 운영 launch와 병행해 공개 생산자를 만들지 않는다.

```bash
PYTHONPATH=src/asmc_camera/src:tools/perception python3 tools/perception/evaluate_imu_width_candidate.py --help
python3 tools/perception/check_imu_width_runtime.py --help
PYTHONPATH=src/asmc_camera/src:tools/perception python3 tools/perception/review_imu_width_failures.py --help
```

다음 우선 과제는 **3카메라 중첩 근거와 shared family의 측방 잔차를 분리하는 분석**이다.
같은 선의 뷰별 지면 근거 차이·거리·중복 가중을 확인하고 고정 폭/공통 형상 제약과 비교한다.
현재 문제를 완화하기 위해 임의 자세 평활화·support 확장·추론 재학습을 적용하지 않는다.

| 과제 | 현재 상태 | 다음 조건 |
|---|---|---|
| 원본 시각 보존·M1 투영 | 구현됨 | 기존 exact 연결·freshness 유지 |
| M1 폭 완화·M0 복귀 | 구현됨, 품질 미통과 | 측방 잔차·지원 손실 해결 후 재판정 |
| 3뷰 중첩·중복 가중 | 분석 대기 | 다음 우선 과제 |
| 경계 소실·출구 보간 | 추가 검증 필요 | 소실5건을 운영 해결로 승격하지 않음 |
| 뷰 사이 병진/yaw 정렬 | 조건부 보류 | 터널 유효 속도·상대 이동 입력 검증 |
| 과거 근거200ms 누적 | 후순위 | 현재 관측 정합과 이동 보정의 이득 확인 |
| 높이·도로 법선 정밀화 | 조건부 보류 | 독립 노면 근거·거리별 체계적 오차 확인 |
| ≥10Hz·250ms공백·350msfreshness | 후보 검증 대기 | 품질 통과 후 실시간 대조 |
