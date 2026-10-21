# 신호등 ResNet18 실행과 최적화 검토

2026-10-20 main `2c94bf11` 코드·CPU checkpoint 메타데이터 검토다.
이번에는 TensorRT export·engine 생성·GPU 벤치마크·모델 교체를 실행하지 않았다.
학습은 장원태 담당이며 카메라 패키지는 현재 배포 모델의 단일 추론 생산자를 제공한다.

## 현재 경로

| 항목 | 현재 구현 |
|---|---|
| 모델 | `models/traffic_light/best.pt`, ResNet18 +8출력 fc, SHA `70ddaf222e7bc6ff2b4748c4b07c1b24dec72a2671f39bc18e0b243ddd93c0d5` |
| 입력 | camera4 JPEG, B1 NCHW **1×3×384×512**, RGB·ImageNet 정규화 |
| backend | PyTorch CUDA, BF16 autocast. YOLO/TwinLite의 TensorRT backend와 구분 |
| 전처리 | PIL RGB decode → uint8 H2D → GPU bilinear resize(`align_corners=False`)·정규화 |
| 출력 | FP32 softmax →8확률 CPU 전송 → 기존 결정 규칙 → typed + legacy 결과 |
| 스케줄 | camera4 공급/목표10Hz, 최신 프레임 하나·같은 수신 sequence 재추론 방지·세대 발행 gate |
| 클래스 순서 | red, yellow, green, red_yellow, red_left, green_left, left, unknown |
| 의미 | 영상 전체 신호 상태 분류. 신호등별 bbox·거리·HD 신호 ID를 직접 출력하지 않음 |

[`traffic_light_node.py`](../src/asmc_camera/nodes/traffic_light_node.py)의 `inference_ms`는
JPEG decode·전처리·forward·확률 D2H까지 포함하며 순수 GPU forward가 아니다.
카메라 수신 대기·ROS 발행/소비 지연과도 구분한다. CPU 복구는 PIL/torchvision transform을
사용하므로 CUDA 전처리와 자동 동등하다고 가정하지 않는다.

기존 `decide()`는 unknown을 후보 선택에서 제외하고 가장 높은 알려진 클래스 확률이
**0.40 초과**일 때 해당 이름을 반환한다. 전체 argmax나 unknown 우선 규칙으로 바꾸는 것은
속도 최적화와 별도의 정책 변경이다. 현재 비교에서는 tie 순서·threshold 경계를 유지한다.

## 이미 확인한 성능과 한계

[초기 parity](../../../config/baselines/perception_traffic_light_parity_v1.yaml)는 같은25프레임에서
canonical 처리 median10.61ms·p95 22.124ms였다. 과거 입력과 초기 실행의 수치이며 현재
전체 스택의 새 벤치마크가 아니다. 대부분 not_detected인 표본이므로8종 안전 판단 검증을 대체하지 않는다.
[최종 family 동시 부하](../../../docs/records/camera/2026-10-18-joint-family-stabilization.md)에서는
ResNet의 완전한5초 창 최소9.8Hz를 확인했다. 이 결과만으로 엔진 전환 필요나 효과를 확정하지 않는다.

최신 프레임 처리·GPU resize/정규화·혼합 정밀도·무구독 영상 수요 처리가 이미 있다.
camera4가10Hz이면 backend를 빠르게 해도 새로운 영상 자체를15Hz로 만들 수 없다.

## 조건부 최적화 순서

1. 같은 camera4 저장 영상에서 decode·H2D/전처리·GPU forward·D2H·발행을 분리해 계측한다.
   CUDA event와 CPU wall을 구분하고 YOLO/TwinLite 동시 실행도 확인한다.
2. 고정 B1 384×512의 logits8개를 ONNX로 export하고 같은 TensorRT10.13 환경의 FP16 후보를
   비교한다. RGB·보간·정규화·기존 FP32 softmax·결정 규칙은 유지한다.
3. YOLO/TwinLite의 TensorRT 버퍼·실행 도구를 재사용하되 context·stream·모드·세대는 독립으로
   둔다. 준비 때 버퍼를 할당하고, dtype/layout 때문에 매 프레임 생기는 복사는 계측 후 줄인다.
4. 다른 두 모델과 카메라가 다르므로 최종 tensor 공유나 대기 batch로 묶지 않는다.
   JPEG decoder 교체는 별도 후보로 대조하고 interpolation/색 차이를 속도 향상으로 섞지 않는다.

엔진 후보는8종 순서·유한값·확률 차이·0.40 경계의 label 전환을 기록한다.
적색/황색/직진/좌회전·미검출·입력 중단과 여섯 환경의 대표 영상으로 확인하고,
기존 25프레임 parity만으로 채택하지 않는다. Typed header·candidate·stale·복구·단일 producer도 유지한다.
GUI 포함 동시 부하에서 최소9.5Hz·공백250ms·p95 150ms/p99 200ms 기준과 다른 모델 공급을
유지하면서 CPU/GPU/입력→출력 지연의 실질적인 이득이 있을 때 기본 전환을 검토한다.
오류를 자동 PyTorch 전환으로 숨기지 않으며 현재 모델·backend를 명시적 복구 경로로 보존한다.

현재 판단은 **변환 검토 가치는 있으나 대회 전 필수 변경은 아님**이다.
재학습·해상도 축소·confidence 변경을 동시에 섞지 않고 backend 하나의 효과부터 확인한다.
[NVIDIA TensorRT10 성능 지침](https://docs.nvidia.com/deeplearning/tensorrt/10.x.x/performance/best-practices.html)처럼
전송·전후처리를 포함한 애플리케이션 비용을 함께 측정한다.

## 판제 소비

`/perception/traffic_light/detection`은 source header·confidence·candidate·stale을 제공한다.
Legacy `/traffic_light/state`·`detail`은 latch되므로 재구독 때 받은 문자열만으로 최신 신호라
판단하지 않는다. HD 신호 ID/정지선·접근 방향·구간과 결합하는 책임은 판제에 있다.
[인지→판제 확인표](../../../docs/handoffs/perception-to-planning.md#판제-적용-확인표)를 따른다.
