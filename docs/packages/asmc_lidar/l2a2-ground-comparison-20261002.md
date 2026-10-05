# L2-A2 — 기존 지면 제거와 Patchwork++ 동일 조건 비교

상태: **격리 비교 완료 · 현재 고정 Patchwork++ 설정 미채택 · 운영 미연결** (2026-10-02).
이 문서는 지면 단계의 점 분류만 평가한다. 객체 검출, 터널 벽 제거 후의 OBB,
추적 및 판제 주행 성능을 승인하지 않는다. 소스·입력·결과의 고정값은
[기계 판독 manifest](../../../config/baselines/lidar_l2a2_ground_20261002.yaml)에 있다.

## 비교 조건과 해석 단위

기준 브랜치 시작점은 `e638fc6`, 손재호 소스는 `6435af1`,
vendored Patchwork++는 공식 `3e6903a1`이다. Autonav v3 Capture
105개 정차 snapshot을 카메라 `missing`과 `valid_empty` 두 조건으로 재생했다.
각 조건은 5회 반복되므로 1,050 replay row, 210 측정 조건이다. 105개도
서로 상관된 반복 관측이며 독립 도로 상황 105개가 아니다.
지면 마스크의 정량 점수는 `missing`의 105개에서만 계산하고, `valid_empty`는
같은 geometry의 불변성 검사에 썼다. 정상 양성 YOLO, 연속 주행, GPS 전환은
이 자료에 없다. 수집 때 보존한 roll/pitch를 사용하지만 당시 live IMU 수신
이력과 필터 내부 상태를 완전히 복원한 실험은 아니다.

네 조건 모두 유한한 동일 ROI와 동일 자세를 쓴다. `reference/voxel`은 기존
10 cm voxel 지면 경로, `reference/raw`는 그 지면 경로에 원본 ROI를 넣는
점 표현 대조군이다. `Patchwork++/voxel`과 `Patchwork++/raw`는 각각 같은
표현을 받는다. 위아래 비교는 알고리즘 효과, 좌우 비교는 표현 효과를 나타낸다.
두 알고리즘 모두 같은 원본 점 인덱스에 결과를 돌려놓아 채점한다. 공통 천장
제외점은 지면 실패나 객체 보존으로 세지 않는다. Patchwork++ 설정은
`sensor_height=1.80 m`, 거리 1–60 m, RNR OFF로 끝까지 고정했다.
1.80 m는 실측 장착 높이 1.45 m와 다른 초기 가설값이며, 장면별 조정은 안 했다.
각 정차 시퀀스는 상태를 재초기화했다. 첫 처리와 5회 반복 결과를 연속 주행
적응이라고 해석하지 않는다. Patchwork++ 지면을 기존 단일 평면 기반 벽 필터에
연결하지 않았다.

## 입력·기준선 회귀

- 같은 고정 입력으로 기존 `reference`를 다시 실행해 측정 210/210의
  voxel 개수·FNV·원본점 계보·label·OBB 개수/순서를 재현했다. 중심 최대
  0.00005275 m, 코너 최대 0.00005885 m 차이는 L2-A1 수치 공차 안이다.
- 네 조건 모두 동일 ROI 원본 인덱스·voxel FNV를 검증한다. 105장면의
  `missing`/정상 빈 카메라 사이 지면 마스크 차이는 **0건**이다. 기준 평면
  유효 105/105, fallback 0이며 원본 ROI에서 미분류점도 0이다.
- 같은 1,050행을 다시 실행한 결과, 관측 stamp·입력 count·voxel FNV·네
  마스크·상태가 모두 일치했다. CPU 처리시간은 결정성 비교에서 제외했다.
- 일반도로 소스에서 분리한 PCL 지면 마스크를 기존 100-frame 고정 benchmark의
  같은 staged raw ROI, pitch 0에 재생했을 때 1,184,389점 중 다른 분류는
  **0점**이었다. 이는 **지면 소스 단계 parity**이지 일반도로 전체
  LiDAR 출력/주행 parity가 아니다.
- 재생 도구는 roll/pitch/geometry-valid 부재와 지원하지 않는 카메라 상태를
  명시적 입력 오류로 거부한다. ROS review는 관측 stamp 이전에 이미 도착한
  유효 IMU 중 최신 것을 고르고, missing·future-only·invalid·stale을 진단한다.

## 정량 결과

Capture의 GT point semantic에서 지면은 설정된 ground class,
대상 객체는 차량 `125`와 일반 장애물 `164`만 집계했다. `StandingOBJ(109)`는
벽 등의 구조물을 포함하므로 문맥 계수로 분리한다. 검증되지 않은 box는 AP
정답으로 사용하지 않는다. 원본 ROI의 대상점은 차량 16,640점, 일반 장애물
3,998점, 지면 291,200점이다.

| 표현·지면 처리 | 지면 GT 제거 | 대상 객체점 보존 | 기준선 대비 추가 완전 소실* | 지면 처리 p95 |
|---|---:|---:|---:|---:|
| voxel + 기존 기준선 | 99.986% | 97.490% | 기준 | 0.468 ms |
| voxel + Patchwork++ | 93.438% | 64.643% | 10관측 (검증 box 6) | 0.845 ms |
| 원본 ROI + 기존 기준선 | 99.975% | 97.587% | 기준 | 0.687 ms |
| 원본 ROI + Patchwork++ | 92.111% | 65.031% | 9관측 (검증 box 6) | 1.594 ms |

* 같은 표현 기준선에는 객체점이 남았고 Patchwork++에는 0점인, ROI에
원본점이 10개 이상 있던 box 관측 수다. voxel 10관측 중 6개, raw 9관측 중
6개만 box geometry가 정량 승인됐고 나머지 4/3개는 참고 사례다. 여러
프레임의 같은 actor를 중복 셀 수 있으므로 서로 다른 차량 수가 아니다.
승인된 box에서도 완전 소실이 있어 현재 설정의 후보 진급을 보류한다.
Patchwork++는 차량 `125` 16,640점 가운데
voxel 10,564점, raw 10,612점을 보존했다(기준선 16,448/16,454점).
장애물 `164`는 3,998점 가운데 2,777/2,809점이었다
(기준선 3,672/3,686점). 지면 잔여도 같은 표현의 기준선보다 많다.
즉 원본점 복원만으로 이 손익을 뒤집지 못했다. 승인된 box·원본 ROI 10점 이상
109관측을 거리/점 수로 나누면 voxel PP의 완전 소실 6관측은 모두
20–40 m, 10–49점 구간이었다. 이는 특히 성긴 원거리 객체의 위험을
보이지만 동일 actor의 반복 관측이며 독립 객체 6개가 아니다.

각 정차 조건의 **첫 처리**를 별도 채점했다. voxel 기준선/PP의 대상점
보존은 97.383%/62.535%, 지면 제거는 99.184%/97.763%였다. 원본
기준선/PP는 대상점 96.972%/64.498%, 지면 99.910%/98.041%였다.
첫 처리와 다섯째 처리 사이에 마스크가 변한 장면은 voxel 기준선/PP
98/86개, 원본 기준선/PP 98/92개다. 반복 예열 결과를 연속 주행에서의
적응 성능으로 대체할 수 없으며, 첫 처리에서도 PP 객체 보존이 낮다.

처리시간은 Release, 동일 컨테이너의 단독 오프라인 측정이며
end-to-end 지연이 아니다. 공통 ROI·voxel 전처리 p95는 3.592 ms다.
Patchwork++ 자체 p95는 voxel에서 수평화 0.045, 입력 복사 0.026,
분할 0.749, 복원 0.013 ms; raw에서 각각 0.065, 0.046, 1.467,
0.024 ms다. 기준선은 수평화+지면 처리 p95가 voxel 0.468,
raw 0.687 ms다. YOLO·bridge·판제와 동시 실행 CPU/GPU 부하는 측정하지 않았다.

대표 점 손실 그림은 `$ASMC_DATA/lidar_assessment/20261002_l2a2_ground_compare/figures_v2/`
의 `...000020_id2.png`, `...000022_id2.png`, `...000023_id2.png`다.
세 장면 모두 box geometry 승인을 받은 같은 객체의 근접 시점이므로 독립 사례 3개는 아니다.
각 그림은 원본 ROI·각 분류의 BEV와 측면을 보여준다. 전체 per-frame·per-box
점수는 같은 데이터 디렉터리의 `score_v3/frames.jsonl`,
`score_v3/objects.jsonl` 및 `summary.json`의 첫 처리·거리/관측량별 표에
있다. 그림과 목록은 실패 진단이지 새로운
정답 주석이 아니다.

## 과거 보조 자료와 이번 자료의 차이

L1-A의 과거 100-frame 실험은 10 Hz/28,800-ray 다른 센서 조건에서
Patchwork++가 낮은 `164` 객체를 423/640점 보존해 PCL 단일 평면의
332/640보다 나았지만, 보행자는 387/427로 PCL 420/427보다 나빴다.
그 실험은 현재 터널 refit 전체와 같은 파이프라인이 아니다.
이번 v3 Capture에는 보행자 수락 데이터가 없으므로 옛 보행자 손익을
v3 성능이라고 옮기지 않는다. AIM 자료는 센서 위치·카메라4·시간/GT 계약
차이가 있어 네 조건 표에 합치지 않았다.

## 판정과 재실행

현재 설정의 Patchwork++를 운영 경로에 **채택하지 않는다**. 같은 입력 표현
두 쌍 모두 객체점 보존과 지면 GT 제거가 악화됐고 충분 관측 box의 추가
완전 소실도 발생했다. 설정을 조정해 통과시킬 때까지 A2를 반복하지 않는다.
속도가 이득이라는 근거도 없다. 일반도로 전체 출력, v3 보행자/낮은 장애물,
연속 주행, 추적 정확도는 미검증으로 남긴다. 새 배치/gRPC 수집은 이번
미채택 판정에 필요하지 않아 수행하지 않았다.

Noetic 컨테이너에서 저장소 루트가 `/root/ws`, `$ASMC_DATA=/data`일 때
재실행 예시는 다음과 같다. 출력 경로는 새 파일을 지정해야 한다.

```bash
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps asmc_lidar -j2 -DCMAKE_BUILD_TYPE=Release
SNAP="$ASMC_DATA/lidar_assessment/20260929/l1b4b_capture_gt/inputs_v4"
OUT="$ASMC_DATA/lidar_assessment/20261002_l2a2_ground_compare"
./devel/lib/asmc_lidar/asmc_lidar_ground_compare "$SNAP/snapshots.jsonl" \
  src/asmc_lidar/config/autonav_v3.yaml src/asmc_lidar/config/reference_tunnel.yaml \
  tunnel "$OUT/new_four_way.jsonl"
python3 -B tools/lidar/l2a2/score.py --comparison "$OUT/new_four_way.jsonl" \
  --admission "$SNAP" --output "$OUT/new_score"
```

기존 고정 결과를 재검증하려면 `tools/lidar/l2a1/verify.py`,
`tools/lidar/l2a2/verify_repeat.py`, `verify_general.py`,
`test_offline_inputs.py`를 사용한다. 정확한 인자와 SHA는
[manifest](../../../config/baselines/lidar_l2a2_ground_20261002.yaml),
[LiDAR package README](../../../src/asmc_lidar/README.md)에 연결했다.

다음은 **L2-A3 군집 분할·누락**이다. 기존 성공 차량 69·장애물
41관측의 추가 실패 없이, 차량 실패 22관측의 원인을 고정 입력에서
개선하는 단위를 먼저 설계한다. Patchwork++를 보류해도 군집 문제의
우선순위는 바뀌지 않는다. 구조물 분리·지면 후단 결합은 그 다음 L2-B이며,
운영 토픽·메시지·판제 전환은 별도 승인과 회귀가 필요하다.
