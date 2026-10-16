# YOLO/TwinLite15Hz·NMS0.50·confidence 대조 — 2026-10-15

실행 정본은 [카메라 실행·관측 계약](../../contracts/camera-runtime-v1.md)이다.
PR #85에서 E347의 가중치·16종/세 그룹·3뷰 mask·독립 프로세스·원본 header를 유지한다.
학습·수집·LiDAR/판제 연결·공개 차량 제어는 변경하지 않는다.

## 결정과 수락 범위

- 정본 `camera.launch`는 YOLO15/TwinLite15/ResNet10Hz를 요청한다. legacy `perception.launch`의 TwinLite 기본10Hz는 복구·재현용으로 유지한다.
- YOLO/TwinLite는 완전한5초 창에서10Hz 이상 공급을 확인한다. 15Hz 목표의95%인14.25Hz 달성은 별도 표시하며 두 판정을 혼동하지 않는다.
- 기존 모델 callback→발행 p95≤150ms·p99≤200ms·공백≤250ms·전체 VRAM≤90%와 readiness를 유지한다. 입력 중단을 자동 면제하지 않는다.
- 동일 클래스 NMS 기본값은0.50으로 바꾼다. 교차 클래스 억제는 사용하지 않는다.
- Confidence는0.40을 유지한다. 소비자가 각 bbox의 confidence로 더 엄격히 거를 수 있으며 생산자 cutoff를 낮추려면 시작 옵션을 바꿔 재시작한다.

## 15Hz의 연산과 시각

15와10이 정수배 관계일 필요는 없다. 타이머는 실제 경과 시간과 새 source stamp로 동작한다.
다만 기존 코드에 `1e9 / round(1e9/15) = 14.999999925Hz` 비교 경계가 있어 같은15Hz 생산자에
두 번째 source-time 제한을 적용했다. 같은 반올림 규칙의 나노초 주기로 비교하도록 고쳤다.
새 결과만 통과시키며 duplicate·역행 stamp 배제와 더 빠른 생산자에 대한 기존 상한은 유지한다.

원본 센서 설정은 유지했다. 이번 실시간 시험의 RGB 공급은 약20Hz이고, 두 모델이 최신 프레임을 골라 목표15Hz로 처리한다.
TwinLite의 모델 격자와 BEV fusion 상한은 정본에서15Hz로 맞췄다. 같은 목표 Hz여도 두 모델이
같은 RGB를 고르거나 동시에 발행하는 것은 아니다. YOLO↔차선 및 향후 LiDAR 연결에는 source stamp가 필요하다.
15Hz 목표는 일부 처리가 떨어져도10Hz 이상을 공급할 여유를 줄 수 있지만 센서 중단·연산 한도를 해결해 주지는 않는다.

## 차선 후단 최적화와 실패 보존

첫 실시간 대조에서 모델 출력은 약14.8–14.9Hz였으나 동기 입력 callback 안에서 fitting을 실행해
차선 family p95가413ms, pipeline_ready=false였다. 반복을 중단하고 날씨를 복원했다.
실패 원본은 `data/runtime/camera-structure-v1/results/rate-followup/live-final/`에 보존한다.

Fitting을 기존 노드의 독립 worker로 옮기고 처리 중인 한 묶음 외에는 최신 동기 묶음 하나만
보관한다. 오래된 pending 묶음은 대체 횟수를 기록하고 버린다. 입력 callback은 fitting/epoch 잠금을
기다리지 않는다. 입력 때 세대를 기록하고 worker의 epoch 잠금 안에서 세대·cutoff를 다시 검사한다.
STANDBY와 종료에는 pending을 버린다. 차선 알고리즘·튜닝값·ODD·timeout은 유지한다.

최신 worker에서 SUNNY/FOGGY 각3회·60초, 세 모델 ACTIVE·GUI OFF로 반복했다.
모든 회차가 모델 최소 공급·지연·공백·VRAM·readiness 기준을 통과했다. 중복/역행 stamp와 추론 오류는0이었다.
SUNNY-r3의 TwinLite는 한 완전한5초 창14.0Hz로 목표95%인14.25Hz에는 미달했다. 최소10Hz 통과와 구분한다.

| 날씨·회차 | YOLO 뷰별 Hz | TwinLite 뷰별 Hz | 모델 최저5초 Hz | Family Hz / 최저5초 Hz | Family p50 / p95 / p99 ms |
|---|---:|---:|---:|---:|---|
| SUNNY 1 | 14.93 | 14.80 | 14.4 | 13.38 / 12.8 | 150.09 / 195.13 / 220.27 |
| SUNNY 2 | 14.90–14.92 | 14.80 | 14.4 | 13.10 / 12.8 | 150.35 / 196.83 / 219.61 |
| SUNNY 3 | 14.83–14.95 | 14.73 | 14.0 | 13.53 / 13.0 | 145.87 / 193.74 / 221.05 |
| FOGGY 1 | 14.87–14.88 | 14.80 | 14.6 | 13.18 / 12.0 | 146.55 / 198.70 / 219.36 |
| FOGGY 2 | 14.88–14.93 | 14.87 | 14.6 | 13.48 / 12.8 | 145.72 / 187.68 / 211.02 |
| FOGGY 3 | 14.90–14.95 | 14.92 | 14.6 | 13.67 / 13.2 | 146.73 / 192.22 / 212.97 |

Family 지연은 같은 source stamp의 TwinLite callback→family 구독 수신이며 모델 callback→발행 지연과 별도다.
모델용150/200ms 기준을 family에 통과한 것처럼 적용하지 않는다. Family는 기존350ms freshness/readiness를 유지한다.
전체 GPU VRAM 최대는4845/8188MiB(59.17%)였다. CPU는 카메라 프로세스 합산 약303–305%(100%=1core)였다.
FOGGY-r1 ACTIVE의 첫 약12초에는 CPU 회귀 검사가 겹쳤다. 배경 부하로 보존하고 다른 반복과 구분하며,
이를 조용한 PC에서의 대조로 설명하지 않는다. 사용자 문서·브라우저 작업은 종료하지 않았다.

| 세 모델 + 기존 Main preview 추가 부하 | Family Hz | Family p50 / p95 / p99 ms | ACTIVE readiness 이상 |
|---|---:|---|---:|
| SUNNY, GUI OFF | 13.32 | 148.74 / 197.97 / 214.90 | 0 |
| SUNNY, 모든 debug 영상 + rqt | 12.05 | 173.91 / 232.00 / 257.21 | 2회 |
| SUNNY, 객체 영상만 + rqt | 12.27 | 167.73 / 219.66 / 244.09 | 1회 |
| FOGGY, 객체 영상만 + rqt | 12.57 | 163.08 / 213.33 / 242.22 | 0 |

추가 부하에서도 모델 최소10Hz·모델 지연 기준은 통과했지만, GUI ON의 SUNNY 두 실행은
readiness 순간 저하로 전체 회차 수락에 실패했다. 자동 면제하거나 timeout을 완화하지 않았다.
기본15Hz는 일반 GUI OFF 실행의 결과이며, 모든 시각화와 전체 스택 동시 주행 수락은 완료하지 않았다.
시각화의 중복 decode/encode 정리는 다음 과제로 남긴다. 기존 Main 제어는 `/b0/discarded_ctrl_cmd*`로 격리했다.

저장 영상의 batch 대기20→10ms 대조는 지연 이득이 없고 family p95가172→222ms로 늘어20ms를 유지했다.
단일 대조의 배경 부하는 다를 수 있어 인과 효과를 확정하지 않는다. 계산이10Hz를 충분히 넘으면
15Hz 숫자를 채우기 위해 검증되지 않은 GPU/정밀도·차선 알고리즘 변경을 추가하지 않는다.

## Confidence 비교

E8의 실제 runtime packing·FP16·3뷰 mask·동일 클래스 NMS0.50을 고정한 pre-NMS cache로 비교했다.
selection205는 반복 사용한 개발 평가본이며194장 complete·11장 partial이다.
완전 GT만 precision/F1에 사용하고 partial은 recall·사례로 분리한다. 같은 GT에 일대일 IoU≥0.5를 적용한다.

| Confidence | 3그룹 macro F1 | 16종 macro F1 | 차량 recall | 동적 recall | 정적 recall | 중복/이름/배경·위치 FP |
|---|---:|---:|---:|---:|---:|---|
| 0.40 | 72.33% | 74.69% | 63.04% | 84.50% | 82.93% | 1 / 22 / 77 |
| 0.50 | 73.22% | 74.57% | 54.35% | 80.62% | 79.27% | 1 / 16 / 51 |
| 0.60 | 71.00% | 72.70% | 45.65% | 75.19% | 73.17% | 0 / 12 / 35 |

0.50은 그룹 F1을0.89%p 높였으나 차량 recall을8.70%p 낮췄다. 모든 그룹의 recall 하락이
기존1%p 보존 기준을 넘어서 기본으로 선정하지 않는다. 0.60은 F1도 낮아졌다.
Heldout을 보고 confidence를 다시 조정하지 않았다. NMS0.50의 고정 heldout 대조는 E347에 보존돼 있다.

## 결과·재현

- 원본·실패·복구된 trace: `data/runtime/camera-structure-v1/results/rate-followup/`.
- 갤러리·요약: `artifacts/camera-rate-confidence-20261015-r1/`, 기존 ASMC 실험 목록에 연결한다.
- Git 요약 증거: `config/baselines/evidence/camera_rate_confidence_v1/`.
- 측정은 `run_camera_structure_matrix.py`·`compare_camera_runtime.py`, 집계는 `summarize_camera_structure.py --rate-followup`를 사용한다.
- Confidence 원본 cache·GT SHA와 모든 표본의0.40/0.50/0.60 패널을 보존한다.

첫 독립 master 준비 실패와 상대 probe 경로가 ROS_HOME을 가리킨 계측 문제도 보존했다.
상대 경로 trace9개를 원래 실험 폴더로 복사하고 SHA와 원래 보고서를 보존한 뒤 집계를 복구했다.
도구는 출력 경로를 절대 경로로 고정하고 fresh 측정 출력이 없으면 실패한다.

```bash
source /data/runtime/camera-structure-v1/devel/setup.bash
roslaunch asmc_camera camera.launch mode:=active
```

별도 터미널의 `view.launch`와 rqt 명령은 실행 계약을 따른다. CI·로컬 검사는 별도로 기록한다.
CPU CI 단계517개(런타임54개 포함), 추가 계약 회귀35개와 계약/문서 검사, 생성된 ROS 메시지 기반 fixture5개를 통과했다.
새 검사는15Hz 반올림 경계, 최신 pending 한 묶음·종료·구세대 배제, launch 기본값과 실측 집계를 포함한다.
PR은 전체 준비·운영 수락 조건을 충족하기 전까지 Draft로 유지한다.
