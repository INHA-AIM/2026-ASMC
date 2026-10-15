# YOLO·TwinLite 공유 입력 설계 — 2026-10-15

상태: **공유 GPU 입력 후보 구현·검증 중**. 기존960 선별 기록과 실패를 보존하며,
수락 전 공유 구조의 기본 승격·PR 병합은 하지 않는다.
실행 시작점은 [카메라 실행 계약](../../contracts/perception/camera-runtime-v1.md)이다.
구현·배포 모델·실패를 포함한 최종 판정은 [B1/B2 검증 기록](camera-shared960-validation.md)을 따른다.

## 현재 코드에서 같아진 것과 다른 것

| 항목 | YOLO | TwinLite |
|---|---|---|
| 전방 tensor H×W |544×960|544×960|
| 좌우 tensor H×W |480×640|480×640 후보;384×640 복구 프로필 보존|
| 원본 전방 영상 |1280×720|동일|
| 원본 좌우 영상 |640×480|동일|
| 축소 보간 |OpenCV linear|OpenCV area|
| Ego mask |ON, 원본 크기에서 적용|기존 원본 입력 유지|
| 독립 CPU→GPU 입력 |BGR uint8, GPU에서 RGB·binding dtype·/255|CPU RGB float32·/255, GPU AMP|
| 현재 실행 |TensorRT FP16|PyTorch AMP, TensorRT 아님|

전방 너비와 tensor shape가 같아도 입력 픽셀·dtype·layout이 다르면 최종 tensor를
그대로 공유할 수 없다. mask를 resize 뒤로 옮기는 것도 경계 픽셀을 바꾸므로 동등한
최적화로 단정하지 않는다. `channels_last`는 FP16 수치 차이와 전체 부하를 별도로 확인한
후보이며, 공통 tensor의 storage layout과 모델별 요구 layout도 구분한다.

## B1/B2를 공통 처리 단위로 사용

| 논리 batch | 뷰 | YOLO 실제 입력 | TwinLite 실제 입력 |
|---|---|---|---|
| B1 |전방1장|1×3×544×960|1×3×544×960|
| B2 |좌·우2장|2×3×480×640|2×3×480×640 후보|

B1/B2는 입력 구성의 공통 이름이며 두 모델을 한 번에 실행하는 batch가 아니다.
모델별 실행은 각각 필요하다. TwinLite를 기존 batch3에서 두 호출로 나누는 비용도 측정한다.
사용자 승인으로 좌우480×640 후보를 구현한다. 네트워크·학습 가중치는 유지하며
기존384 대비 좌우 tensor 픽셀25%·전체 약12% 증가를 품질·연산 대조에서 확인한다.
원본 좌우640×480에 추가 upscale은 없고 기존512×384 축소·좌우 padding이 사라진다.

같은15Hz 목표가 같은 프레임 소비를 보장하지 않는다. 공통 입력을 두 worker에 제공하되
느린 모델을 기다려 빠른 모델의 출력도 늦추는 barrier는 만들지 않는다.
두 모델이 실제로 같은 프레임을 사용한 경우에만 같은 입력으로 연결한다.
YOLO는 중단 뷰를 기다리지 않고 남은 뷰를 처리한다. TwinLite의 기존3뷰 coherence와
차선 family readiness는 유지하며 오래된 뷰를 복제해 완전한 batch로 만들지 않는다.

## 공유 경계

```text
전방 / 좌 / 우 JPEG
        ↓ 카메라별 최신 프레임, source header 보존
FrameBroker → 원본 디코딩 캐시 (읽기 전용)
        ↓ FrameKey + Geometry + 선택한 B1/B2
공통 GPU 원본 uint8 버퍼 (후속 구현 후보)
        ├─ YOLO: mask → resize/padding → RGB/정밀도 변환 → TensorRT
        └─ TwinLite: resize/padding → RGB/정밀도 변환 → PyTorch
                    ↓
             모델별 후처리·모드·발행
```

첫 구현은 **FrameBroker·디코딩·변환 정보 공유**다. 기존 R2의 공유 캐시를 확장하며
새 실행 체계를 만들지 않는다. 원본 JPEG 구독은 뷰당 한 번, 같은 프레임의 디코딩도
한 번으로 제한한다. ResNet은 camera4의 별도 입력과 worker를 유지한다.
현재 단일 시각화 노드는 별도 소비자다. 추론 내부 공유만으로 시각화 프로세스의 디코딩까지
없어졌다고 표시하지 않는다.

그다음 공통 원본 uint8를 GPU에 한 번 전송하고, 모델별 작업 버퍼에서 mask·보간·정규화를
수행하는 경로를 비교한다. 최종 tensor 두 개가 필요하더라도 H2D는 한 번으로 줄일 수 있다.
이는 GPU 전처리 구현과 기존 CPU 전처리의 출력 검증이 필요한 단계다. CPU에서 서로 다른
전처리를 해놓고 기존 `predict(list)`를 그대로 호출하는 것만으로 H2D가 통합되지는 않는다.

최종 resize나 normalized tensor까지 공유하는 것은 **같은 프레임, shape, 보간,
padding 값·위치, mask, RGB 순서, 정규화, dtype, layout, device**가 모두 일치할 때만 허용한다.
현재 mask 차이는 유지한다. 입력을 공유하려고 TwinLite에 YOLO mask를 자동 적용하지 않는다.
동일 조건이 아닌 경로도 공통 Geometry와 원본 GPU 버퍼는 사용할 수 있다.

## 내부 계약과 버퍼 소유권

- `FrameKey`: 입력 epoch·camera ID·source stamp·sequence. bridge 재시작과 stamp 재사용을 구분한다.
- `NativeFrame`: FrameKey, 원본 header·크기, camera calibration ID, 읽기 전용 원본 영상.
- `Geometry`: 원본/resize/tensor H×W, padding LTRB, scale XY. bbox 역변환과 lane grid 투영에 함께 사용한다.
- `PreprocessSpec`: 보간·padding·mask SHA·색 순서·정규화·dtype·layout·device. 다른 spec은 별도 cache다.
- 모델별 결과는 자신의 producer session·generation·model/config ID와 정확한 FrameKey에 연결한다.

이는 내부 Python/buffer 계약이다. 공개 ROS 메시지·서비스·MD5를 바꾸거나 GPU pointer를
ROS 메시지로 보내지 않는다. 원본 영상 header와 모델별 빈 정상 결과·무효 결과 구분을 유지한다.

뷰별 버퍼는 사용 중인 슬롯과 최신 슬롯, 최대2개로 제한한다. worker는 lease를 잡고,
GPU event가 완료된 뒤 반납한다. 입력 callback은 사용 중 슬롯을 기다리지 않고 오래된
미처리 프레임을 교체한다. CPU 버퍼와 GPU 버퍼 모두 종료·세대 변경·예외 반환을 검사한다.
사용하지 않을 모델의 전처리를 미리 만들거나 결과를 재발행해 처리율을 높이지 않는다.

## 실행 프로세스와 동기화

현재 R1의 모델별 프로세스는 기준선·복구 경로로 유지한다. GPU 입력 공유 후보는 기존 R2의
한 프로세스 안에 YOLO/TwinLite 독립 worker를 두는 구조다. 일반 ROS 메시지를 주고받는
별도 프로세스는 CUDA tensor를 자동으로 공유하지 않는다. 첫 구현에서 CUDA IPC까지 확대하지 않는다.

ResNet은 우선 독립 프로세스로 유지한다. YOLO와 TwinLite의 모드·주기·오류·준비 상태는
독립이며 같은 프로세스를 사용한다고 같은 generation을 부여하지 않는다.
동시 GPU 실행이 항상 빠르다고 가정하지 않는다. 모델별 stream/event와 GPU 실행 순서를
비교하되 현재 backend의 전체 device synchronize가 다른 worker를 정지시키는지도 계측한다.
버퍼를 공유해놓고 실제 predictor 내부에서 재전송·cast·layout 복사가 일어나는지도 집계한다.

공유 프로세스는 Python GIL·GC와 치명적인 CUDA 오류의 영향도 공유한다.960 선별 중
YOLO에서 generation2 GC 약2.34초와 같은 시점의 callback 공백 약2.38초가 확인됐다.
다른 프로세스의 TwinLite 입력은 계속됐다. 이 경로를 그대로 합치면 두 모델이 함께 멈출 수 있다.
공유 기본 전환 전에 이 공백과 장시간 RSS 증가를 확인해야 한다. GC를 꺼서 숨기지 않으며
현재는 GC 정책을 변경하지 않았다.

## BEV와 후처리

카메라 보정은 모델별이 아니라 **카메라별 intrinsic/extrinsic**이다. 서로 다른 입력 크기도
동일한 센서 보정과 각각의 Geometry를 이용할 수 있다. 공통 크기는 변환 관리와 검증을 단순하게 한다.
각 모델의 bbox·class map·probability 복원 방식은 결과 형식에 맞게 유지한다.

차선은 지면 평면 투영, 객체는 bbox·접점·깊이/향후 LiDAR 연관을 사용하므로 BEV 알고리즘까지
하나로 합치지 않는다. 같은 영상 변환 정보를 쓸 수 있다는 것이 bbox만으로 정확한3D 위치를
안다는 뜻은 아니다. 이번 카메라 설계는3뷰 LiDAR fusion 구현과 구분한다.

## 구현과 검증 순서

1. 현재960 입력의 mask·보간·dtype·layout과 B1/B2, FrameKey/Geometry를 설정·내부 계약으로 고정한다.
2. 모델 adapter를 분리하고 기존 공유 캐시에 FrameKey/Geometry·lease를 연결한다. 픽셀 입력은 유지한다.
3. 공통 GPU 원본 업로드와 모델별 전처리 후보를 추가한다. 검증 전에는 기존 입력 경로를 기본으로 둔다.
4. 공통 최종 tensor가 가능한 조건만 별도 후보로 비교한다. 좌우 크기·보간·mask 정책을 한꺼번에 바꾸지 않는다.
5. 같은 저장 RGB로 원본 입력·bbox/semantic·차선 투영·head/NMS·stamp·세대 동등성을 검사한다.
6. 통과 후보만 R1 대비 실제 resize·decode·H2D 횟수/bytes·단계별 시간·CPU/RSS·VRAM·GC 공백을 측정한다.
7. 같은960·confidence0.40·NMS0.50·15/15/10Hz로 SUNNY/FOGGY 반복과30분 시험을 진행한다.

최소10Hz 공급(ResNet9.5), 모델 p95≤150ms·p99≤200ms, 공백≤250ms, 전체 VRAM≤90%,
차선 freshness350ms·readiness 저하0은 유지한다.15Hz 목표 달성은 별도로 표시한다.
공유 후보는 기준선 대비 CPU 또는 p95가10% 이상 개선되고 다른 모델 p95 악화≤5ms,
품질·수명·오류 격리 기준을 통과할 때만 채택한다. 이득이 작으면 독립 프로세스를 유지한다.
