# B0 카메라·LiDAR 통합 부하 기준선

> 역할: 실험 프로토콜과 결과 인수. 운영 프로필·주행 승인 아님.
> 기준: 연구 문서가 반영된 main `3b142b1`, RTX 4060 Ti 8GB.
> 측정 도구와 입력·설정은 실행별 SHA manifest로 고정한다.

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
