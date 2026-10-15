# 카메라 전방960 검증 — 2026-10-15

사용자 요청에 따라 기존 GUI 재시험을 중단하고 전방 입력을 재설정했다.
이전 `camera-visualization-v1` source와 E349 성공·실패 결과는 보존한다.
중단 후 시험 소유 rqt와 Main preview를 종료하고 죽은 시험 ROS 등록을 제거했으며,
공유 master에는 기존 bridge·TF·rosout 네 노드만 남았다. MORAI 날씨도 원래 SUNNY로 복원했다.
실험 외 사용자의 프로세스는 종료하지 않았다.

| 모델 | 이전 tensor | 현재 tensor | 처리 |
|---|---|---|---|
| E8/사전학습 YOLO26l 전방 |736×1280|544×960|기존 dynamic TensorRT FP16 engine 재사용|
| YOLO 좌우 |480×640|480×640|2뷰 batch 유지|
| TwinLite 전방 |384×640|544×960|같은 모델의 별도 shape batch|
| TwinLite 좌우 |384×640|384×640|2뷰 batch 유지|
| ResNet |기존 camera4|동일|전처리·정밀도 유지|

전방 원본1280×720의 유효 영상은960×540이며 위아래2px padding이다.
시야를 crop하지 않고 결과는 원본 영상으로 복원한다. TwinLite front는 확대이므로
YOLO front 축소와 별도로 연산 비용·차선 결과를 확인해야 한다.
해상도가 다른 세 입력을 큰 tensor로 전부 padding하거나 모델을 두 번 로드하지 않는다.
누락된 뷰를 기다리지 않으며, 같은 최신 프레임 묶음의 결과는 기존 coherence/generation
검사를 거쳐 발행한다. model-grid projector는 같은 `lane_input.yaml`을 읽고
뷰별 grid geometry를 사용한다. public 메시지·MD5는 바꾸지 않는다.

새 검증 workspace는 `/data/runtime/camera-front960-v1`, 증거는
`artifacts/camera-front960-20261015-r1`이다. 이전 runtime을 덮어쓰지 않았다.
모델 파일의 SHA를 확인한 뒤 기존 engine의384×640~736×1280 profile 범위 안에서
960을 사용한다. engine export·학습은 수행하지 않는다.

현재 상태: ROS 빌드, 실제 TwinLite960 shape/원본 복원, 관련 CPU/ROS318개 검사와
공유 메시지·topic 계약을 확인했다. 별도 검증 workspace의 Docker 설정 누락으로 난 facade
1건은 해당 파일을 연결해17개 모두 재검사했다. 배포 모델4개 파일의 SHA도 확인했다.
전체 과거 회귀를 모두 새 설정으로 재실행했다고 표시하지 않는다.

960 선별은8회 원본을 보존한다. 첫2회 중 SUNNY는 자체 CPU 검사와 잠시 겹쳐 개발 진단으로만
사용하고, 같은2회를 검사 없이 반복했다. OpenCV 스레드1 대조는 개선이 확인되지 않아 채택하지
않았다. 같은 실제 RGB의 NCHW→channels_last→NCHW 진단에서는 추론 중앙값37.1/32.7/38.1ms였다.
FP16 수치 차이는 confidence 최대0.0110, 차선 class 픽셀 최대0.00174%였다. 완전 일치로 표시하지 않는다.

channels_last 후보의 날씨별60초1회 결과는 다음과 같다. 모든 값은 해당 회차의 원시 프레임으로
계산하며 p50/p95를 실행 중앙값의 평균으로 대체하지 않는다.

|날씨|YOLO 전방 Hz·p50/p95 ms|TwinLite 전방 Hz·p50/p95 ms|TwinLite 세뷰 최대 p95 ms|차선 readiness 저하 snapshot|
|---|---|---|---:|---:|
|SUNNY|14.90 ·42.6/67.7|11.60 ·107.3/149.6|152.9|3|
|FOGGY|14.35 ·46.8/80.1|11.43 ·108.1/151.1|155.9|2|

전체 VRAM 최대58.7%, Qt 개수는 각 실행0→1→0이었다. 그러나 모델 p95·차선 freshness/readiness
기준 미달이고 FOGGY YOLO 최대 공백2.33초도 남았다. 따라서 **선별 통과·운영 승격은 보류**한다.
SUNNY/FOGGY 각3회·30분 수락은 미완료이며 PR #85는 Draft다.
자료는 [E350 갤러리](http://127.0.0.1:8765/camera-front960-20261015-r1/index.html)에 연결한다.

사용자의 후속 요청으로 추가 실험을 중단하고 [공유 입력 설계](camera-shared-input-design.md)를
먼저 정리했다. 공유 GPU 입력/전송은 아직 구현하지 않았다.

시험은 실제 Qt 개수를 시작 전/측정 중/종료 후 기록한다. 독립 process group인
rqt와 Main preview를 ROS launch shutdown보다 먼저 종료해, 느린 worker 종료 때문에
시험 창이 남지 않도록 했다. 시작 전 잔류 시험 viewer가 있으면 다음 실행을 차단한다.
기존30분의 드문3초 차선/신호등 공백은 아직 원인 미확정이다. 새로운 YOLO 선별의 한 공백은
generation2 GC2.34초와 callback 공백2.38초가 일치했고, 같은 카메라의 TwinLite 수신은 계속됐다.
해상도 변경만으로 해결됐다고 주장하지 않으며 GC 정책도 변경하지 않았다.

설정 역할은 [config 안내](../../../src/asmc_camera/config/README.md),
실제 가중치·설치는 [models 안내](../../../src/asmc_camera/models/README.md)를 따른다.
