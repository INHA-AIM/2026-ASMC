# 카메라 내부 구조·통합

[빠른 실행](../README.md)과 [공개 계약](../../../docs/contracts/camera-runtime-v1.md)을 먼저 확인합니다.
이 문서는 package 내부 책임과 변경 지점을 다룹니다. 현재 모델별 독립 프로세스이며
YOLO/TwinLite 전방544×960 B1·좌우480×640 B2, TensorRT, 목표15/15/10Hz입니다.

## 책임 경계

| 영역 | 위치 | 책임 |
|---|---|---|
| 진입점·구성 | `launch/camera.launch`, `_runtime.launch`, `_components.launch` | 모델 조합·설정 주입, 중복 producer 없이 실행 |
| 객체 | `src/asmc_camera/detection/`, 객체 node | E8/사전학습·3뷰 batch·Ego mask·class별 NMS·원본좌표·관측 metadata |
| semantic | `src/asmc_camera/camera/` | TwinLite backend·전처리·model-grid·필요 출력만 복원 |
| family/BEV | `src/asmc_camera/bev/` | 지면 투영·동기화·family·유효성; 영상 BEV 수요 기반 |
| 모드·수명 | `src/asmc_camera/runtime/` | 모델별 모드/세대·PREPARE·입력 freshness·bounded 진단 |
| 신호등 | traffic_light node/model | camera4 ResNet·typed+legacy 결과 |
| 시각화 | `launch/view.launch`, visualizer | 객체 단일 합성 렌더러·별도 GUI, 추론 생산자 미기동 |
| 학습·offline | `training/`, `offline/`, 저장소 tools | 실행과 분리; 현재 배포본은 학습 재개 checkpoint가 아님 |

정확한 module 파일 목록은 소스를 기준으로 봅니다. 입력·모드·후처리·발행을 분리하며
같은 private param을 공유하는 새 process를 기본에 끌어오지 않습니다.
공유 입력 후보는 보존하지만 T3 채택 기준을 통과하지 않아 현재 기본은 아닙니다.
YOLO mask는 TwinLite 입력 정책과 다르므로 같은 tensor 크기를 이유로 최종 tensor를 공유하지 않습니다.

## family 필수 경로

```text
TwinLite model-grid class/confidence → 카메라별 지면 투영
  → 최신3뷰 근거 통합 → shared family 피팅/유효성 → boundary_family
```

`config/camera/lane_input.yaml`이 추론·model-grid Geometry의 공통 크기를 소유합니다.
카메라 원본·header·보정·미터 단위 BEV는 변경하지 않습니다.
3뷰 queue4/skew35ms·계산1+최신대기1·세대 발행 재검사와 bounded basis cache를 사용합니다.
BEV 합성에 수요가 없으면 상위 영상 구독/원본 복원도 중단합니다. model-grid family 계산과 구분합니다.
독립 spline 비교는 내부 명시 설정이며 일반 Marker 구독으로 실행하지 않습니다.

STANDBY는 준비 모델을 유지하고 계산/유효 결과를 중단합니다. PREPARE는 새 영상으로 family까지
확인한 뒤 pipeline_ready를 제공합니다. 이전 세대·cutoff·stamp 회귀 결과는 readiness에 사용하지 않습니다.
차선 소비는 음영 ODD이며 구간 판단·제어·fallback은 판제 책임입니다.
GC를 끄거나 freshness350ms를 늘려 순간 공백을 통과시키지 않습니다.

## 객체·소비 경계

16종·3그룹의 현재 표는 [에셋/라벨](../../../docs/competition/obstacle-assets-and-labels.md),
배포 모델은 [모델 안내](../../../docs/operations/camera-deployment.md)에 있습니다.
source_class_name은 모델 이름, class_name은 소비 그룹이며 실제 이동 여부/정확한 API 에셋명은 아닙니다.
관측에는 session/generation·보정/모델/설정 ID가 있습니다. source stamp를 유지하고
빈 정상 결과·무효·미수신·stale을 구분합니다.
Main은 전방 CPU 어댑터만 가지고 YOLO를 실행하지 않습니다. 3뷰 fusion은 [LiDAR 후속](../../../docs/handoffs/camera-to-lidar.md)입니다.

## 검증과 제한

semantic TensorRT 품질 검증·배포 SHA는 [TRT 기록](../../../docs/records/camera/camera-twinlite-tensorrt.md),
family 최적화·사용자 주행 확인·알려진 순간 공백은 [공급 기록](../../../docs/records/camera/camera-family-supply-optimization.md)을 봅니다.
기본 backend·사용자 시각화 승인과 전체 자동 수락을 구분합니다.
bus/truck, 차로 앞 bicycle/dog, dog2/dog10 약세는 추가학습 요청 시 우선 검토합니다.

공개 메시지·서비스·MD5·alias는 유지합니다. source 좌표·stamp·세대, 정상 빈 결과,
중단 뷰, mode 전환, mask, demand 시작/종료를 회귀로 확인합니다.
[config 안내](../config/README.md), [models](../models/README.md), [음영 판제 인계](../../../docs/handoffs/camera-to-planning.md),
[과거 BEV 설계](bev-lane-design.md)를 함께 봅니다. BEV 설계의 후보안은 현재 계약을 덮어쓰지 않습니다.
