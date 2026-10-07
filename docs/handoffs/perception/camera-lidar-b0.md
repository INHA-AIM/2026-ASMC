# B0 카메라·LiDAR 통합 부하 기준선

> 역할: 실험 프로토콜과 결과 인수. 운영 프로필·주행 승인 아님.
> 기준: 연구 문서가 반영된 main `3b142b1`, RTX 4060 Ti 8GB.
> 측정 도구와 입력·설정은 실행별 SHA manifest로 고정한다.
> 완료: 고정 51회 + lifecycle trace 2회 + fusion trace 8회 + 별도 C2 확인 1회.
> 실행 조합 수락과 운영 기본값 변경은 없다.

## 측정 결과 (2026-10-07)

고정 프로토콜은 51/51회 완료했고 개별 검사 49회가 통과했다. 카메라 조합 C1~C5는
각 3회 통과, C6는 2/3회 통과했다. C6 debug ON도 2/3회 통과했다.
실패 두 번은 최종 차선 family의 ACTIVE 첫 1초 출력 8개였다. 고정 창을 유지하고
첫 출력부터 시간을 다시 세거나 목표 Hz를 낮추지 않았다.

C6 첫 실행의 아홉 번째 새 semantic 결과는 RPC 시작 후 0.9612초,
같은 source의 family 출력은 1.0018초에 도착했다. 약 40.5ms downstream 지연으로
첫 1초 경계를 넘은 사실은 원본 stamp와 wall 수신 시각으로 확인했다.
내부 단계별 시간은 별도 trace에 연결하며, 이 결과를 모델 예열 실패로 확대하지 않는다.
정상 카메라 구간의 YOLO는 조합별 약 19.8~19.9Hz였고 중복·역행 stamp는 없었다.

| bag | 조합 | 3회 전체 구간 매칭 쌍 | C1 최솟값 대비 절대 수 기준 |
|---|---|---|---|
| 주 | C1 | 536 / 543 / 536 | 기준 |
| 주 | C4 | 530 / 530 / 532 | 미수락 |
| 주 | C5 | 537 / 535 / 534 | 미수락 |
| 주 | C6 | 530 / 531 / 529 | 미수락 |
| 음영 | C1 | 194 / 195 / 193 | 기준 |
| 음영 | C4 | 190 / 190 / 192 | 미수락 |
| 음영 | C5 | 194 / 194 / 194 | 통과 |
| 음영 | C6 | 191 / 191 / 191 | 미수락 |

공통 stamp·상호 유일한 중심 대응·C1 반복의 안정 분류라는 조건 아래 신규 분류 차이는
0건이다. 주 bag C4/C5/C6의 대응 관측은 각각 2,978/2,874/2,802개지만 미대응도
1,099/1,149/1,172개다. 이를 전체 객체 recall이나 추적 품질 통과로 해석하지 않는다.
기존 PR #69 분류 차이 2건의 원본 승인 기록은 보존했고 B0에 자동 적용하지 않았다.

주 bag trace에서 C1/C4/C6은 541/533/533개를 처리하고 539/531/531쌍을 매칭했다.
생략 generation은 107/115/115개, pose 실패는 각 1개, timeout은 모두 0이었다.
각각 dropped 1 + 종료 pending 1까지 처리 공급 집계가 맞는다. 이 진단 실행에서는
C4/C6의 감소 8쌍을 추가 처리 생략 8개로 설명한다. trace OFF 실행의 개별 내부 사유를
trace ON 기록으로 임의 대체하지 않는다. 처리 시간 p95는 약 105~110ms였으며
이는 주 bag의 약 75ms LiDAR 입력 간격보다 길다. GPU 포화만으로 원인을 설명하지 않는다.

GPS 중단 3회에서는 매번 단절 구간 LiDAR 입력 80개에 출력 3개만 남고 pose 실패가
80회 기록됐다. 카메라·IMU·Ego 공급은 계속됐으며 복구 후 0.242/0.249/0.251초에
출력이 재개됐다. 원인은 기존 GPS age ≤0.30초 게이트다. 주 bag에는 `/gps/fix`가
없으므로 실제 단절 증명은 `/gps`에 한정하고 NavSatFix 단절을 검증했다고 하지 않는다.

GPU는 RTX 4060 Ti 8188MiB, CPU는 i5-14600K/20개 사용 가능한 논리 CPU였다.
카메라 측정의 GPU 전체 메모리 표본 최댓값은 3506MiB이며 Windows desktop 사용분도
포함한다. 200ms 표본의 최댓값을 순간 peak나 노드별 VRAM으로 부르지 않는다.
CPU·RSS·source age·같은 source의 wall 지연 p50/p95/p99는 반복별 원본 분석에 남겼다.

[전체 반복 그림](../../../config/baselines/evidence/camera_lidar_b0_v1/profile-comparison.png),
[수치 기준선](../../../config/baselines/perception_camera_lidar_b0_v1.yaml),
[전체 보고서](../../../config/baselines/evidence/camera_lidar_b0_v1/summary.json)를 함께 본다.

## 실패 보존과 정성 사례

초기 측정의 drain/입력 공급 종료 창, Main legacy 제어 발행 격리, XML-RPC 연결 공유 결함을
수정한 이전 실행도 원본 archive에 보존했다. 공유 연결 음성 대조에서는 오류 516개가
재현됐고 수정 후 master 조회·노드 조회 각각 400회에서는 오류가 없었다.
초기 C4 첫 1초 8개와 steady-window 실패도 삭제하지 않았다.
CPU 음성 대조가 겹친 C2-r1은 표시해 보존했고, 별도 clean C2-r4를 추가해 통과를 확인했다.
추가 성공으로 앞선 실패를 면제하거나 유리한 결과만 선택하지 않는다.

원본 JPEG·raw XYZ·raw XY 그림·header·근접 source 대응을 주/음영 각 3개씩 보관했다.
주 bag 차량 옆 구조물과 61.63m unknown OBB, 음영의 52.80m unknown OBB는
벽 혼입·군집 합침의 검토 후보다. 실제 군집 점의 소속과 카메라 투영이 검증되지 않아
합침 정답으로 확정하지 않았다. 음영 영상의 나무 상자는 현재 허용 YOLO 결과가 비지만
독립 GT recall 분모로 사용하지 않았다. C1이 처리하고 C6가 생략한 stamp도 별도로 남겼다.
[정성 검토 목록](../../../config/baselines/evidence/camera_lidar_b0_v1/qualitative-review.json)은
추가 정확도 승인 자료가 아니라 B1의 재현 입력이다.

outer/nested 빌드, 메시지 1·HD맵 160·Main 267·LiDAR core 2·jamming 94개와
측정기 및 관련 시험 38개, 계약 validator 4개는 통과했다. 카메라 278개 중 기존 facade
1건은 `autonomy_no_bridges_v2.launch`의 traffic_light false/true 차이 그대로이며 신규 실패는 없다.
최종 PR HEAD의 CI 상태는 GitHub checks에서 확인한다. 보호 소스 219개와 외부 TwinLite
소스 12개가 측정 시점과 같고 운영 소스·설정의 main diff가 없어 3,514프레임 unmasked
동등성 증거를 재사용했다. mask의 기존 person 손실 1건 예외는 확대하지 않는다.

## 범위와 프로토콜

YOLO11m 전방 20Hz, TwinLite 3뷰 10Hz, camera4 ResNet18 10Hz와 semantic contour
보넷 mask를 유지한다. 모든 조합에서 세 모델을 GPU에 상주시키고 선택 모델만 ACTIVE로
요청한다. 구간과 결과 소비는 판제가 결정하며 차선 피팅의 소비 ODD는 음영구간이다.

| 조합 | ACTIVE |
|---|---|
| C1 | YOLO |
| C2 | TwinLite·BEV·차선 |
| C3 | ResNet18 |
| C4 | TwinLite + YOLO |
| C5 | ResNet18 + YOLO |
| C6 | 세 모델 |

`config/experiments/camera_lidar_b0_v1.yaml`에 고정된 총 51회 프로토콜을 따른다.
각 반복의 조합 순서를 순환하고 실행마다 새 master·노드를 사용한다.

- 카메라 18회: 09-21 bag의 44–86초, STANDBY 2초 → 해당 세대 PREPARE 확인
  (timeout 10초) → ACTIVE 20초. C1~C6 각각 3회.
- fusion 24회: Ego가 있는 주 LiDAR·음영 bag 전체를 C1/C4/C5/C6으로 각각 3회.
  로드·dummy 예열 후 선택 모델은 처음부터 ACTIVE이며 lifecycle 전환 시험과 구분한다.
- GPS 6회: 주 bag에서 정상 중계와 8–14초 GPS 중단·복구를 각각 3회. 원본 header는
  유지하며 IMU·Ego·LiDAR·카메라를 계속 공급한다. trace ON 진단이고 수락 부하 측정이 아니다.
- debug 3회: C6 카메라 구간에서 추가 debug 영상·raw 발행 ON을 별도로 측정한다.
- 실패 조합은 trace ON 진단을 추가한다. 최초 실패·반복·진단 모두 보존한다.

MORAI 종료와 다른 추론 부재를 확인한 뒤 `asmc-camera-fusion-check`의 network-none,
master 11312에서 직렬 측정한다. 공유 master 11311·bridge·GUI·차량은 변경하지 않는다.
GPU 전체 메모리는 Windows desktop 사용분도 포함한다. 4070 SUPER 또는 판제·bridge를
포함한 대회 전체 스택 성능으로 환산하지 않는다.

## 측정 경계와 기존 Main 동작

카메라 전환의 첫 1초는 trace OFF에서 RPC 호출 시작, trace ON 진단에서 서버 요청
수락 시각으로 잡는다. 준비는 요청 세대의 prepared와 pipeline_ready를 함께 확인한다.
source cutoff 이전 결과는 제외하고 처리율과 유효 판정 비율은 따로 계산한다.

전체 bag 처리율의 종료는 해당 카메라 입력의 마지막 수신까지다. 최종 차선은 3뷰가 모두
존재하는 입력 구간을 사용한다. bag publisher 종료 대기나 fusion drain을 최대 공백에 넣지
않는다. drain 중 결과도 원본 관측·지연·fusion 집계에 보존하고 개수를 따로 보고한다.
이 규칙은 ACTIVE 20초 lifecycle 창을 짧게 만들거나 첫 출력부터 측정하는 데 쓰지 않는다.

원본 stamp의 ROS source age와 같은 source 입력→출력의 wall 지연은 다른 지표다.
차선 family header와 여러 카메라의 동일 stamp가 대응해 입력을 확정할 수 없으면
wall 지연을 미확정으로 표시한다. Main의 기존 ms 단위 stamp 절삭만 입력 대조에 반영한다.
trace OFF의 처리 stamp는 처리 성공마다 발행하는 unfiltered tracked_objects_map에서
관측한다. 선택 생략·pose 실패·timeout의 내부 원인 분리는 trace 진단에 한정한다.

Main은 기존 fusion debug 영상을 항상 생성하며 OFF 옵션이 없다. 운영 코드를 변경하지
않으므로 이 비용을 baseline에 포함한다. 카메라 추가 debug OFF와 구분한다.
기존 ParseCAN의 `/ctrl_cmd_0` 반복 발행은 시험 전용 `b0_main.launch`에서
`/b0/discarded_ctrl_cmd_0`으로 격리해 기록한다. `/ctrl_cmd`도 시험 namespace로 격리한다.
운영 Main이나 bridge를 수정하지 않으며 공개 제어 토픽의 메시지는 없어야 한다.

## 도구와 판정

ROS→바깥 workspace→nested workspace 순서로 source한 격리 컨테이너에서 실행한다.
`run_b0_matrix.py`는 전체 프로토콜, `run_camera_lidar_b0.py`는 개별 조합,
`analyze_b0_matrix.py`는 모든 반복을 함께 분석한다. 사용자 실행을 위한 sh wrapper는
만들지 않으며 일반 디버그는 기존 직접 launch/rqt 명령을 유지한다.

```bash
roslaunch asmc_camera perception.launch profile:=debug
```

```bash
rqt_image_view --clear-config /perception/camera/objects/debug/image/compressed
```

TwinLite·ResNet 첫 1초 ≥9개, 5초 창 ≥9.5Hz, 최대 공백 ≤250ms와 YOLO ≥18Hz,
중복·역행 stamp 0을 조합별로 판정한다. 전체 bag의 초기 ACTIVE 실행에는 PREPARE 후
첫 1초 수락을 주장하지 않는다. 매칭 수는 프레임 쌍이며 물체 재검출 수가 아니다.
동일 LiDAR stamp·상호 유일한 3D 중심 1mm 대응으로 안정 분류를 비교하고 미대응도 보고한다.

PR 완료는 프로토콜과 측정기의 검증·실패 보존·병목 설명이다. 모든 부하 조합의 수락이나
운영 기본값 승격을 뜻하지 않는다. 기존 facade 실패와 PR #69의 두 분류 차이 승인 예외는
원본에 남기며, B0 신규 차이를 자동 면제하지 않는다. GT 없는 벽 혼입·객체 합침·누락 후보를
정량 정확도로 계산하지 않는다.

결과와 원본 연결은 `config/baselines/perception_camera_lidar_b0_v1.yaml`에 기록한다.
이전 3,514프레임 동등성은 보호 대상 소스·weight·계약의 SHA가 같은 경우에만 재사용한다.

## 다음 단계

B1은 동일 YOLO11m 결과와 raw LiDAR에서 깊이·연결성 연관을 비교하고 재호의 기존
box 보호와 대조한다. GPS 없는 local 처리와 map 출력 분리는 LiDAR 담당과 별도 구현한다.
전방/3카메라, LiDAR 최대/10Hz × YOLO20/10Hz, YOLO26s detect→s-seg boxes→같은
seg 결과의 masks, 필요시 m-seg를 각각 분리한다. 학습·FP16·batch·공통 decode·TensorRT와
4070 SUPER 인수도 별도 단계이며 jamming 어댑터·21% crop 운영 기본값은 유지한다.
