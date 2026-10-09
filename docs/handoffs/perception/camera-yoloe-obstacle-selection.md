# YOLOE m/l・전방640/960/1280 비교

이 작업은 YOLOE의 추가학습 후보를 고르는 실험이며 운영 기본 모델을 바꾸지 않는다.
현재 기준선은 YOLO26l detect・전방1280・좌우640×480・3뷰15Hz・2+1 batch・confidence0.40이다.
작업 브랜치는 `feat/camera-yoloe-obstacle-selection`, worktree와 공유 컨테이너는 유지한다.
PR #76은 GitHub 계정 billing 제한으로 hosted CI가 시작되지 않아 병합 대기 중이다.
따라서 이 브랜치는 #76의 사용자 확인 기록 커밋 `503a44ac` 위에 쌓는다. main 기준으로
병합됐다고 표시하지 않으며, #76 병합 후 기준을 옮긴다.

## 고정 비교

`config/experiments/camera_yoloe_selection_v1.yaml`의 obstacle17을 m/l 각각의
checkpoint에서 embedding으로 만들고, 너비별 engine을 따로 만든다. 기존89/42종
embedding을 출력에서 필터링한 결과로 대신하지 않는다. head는 one-to-many,
class-aware NMS・IoU0.7・max_det300을 유지한다. 세 뷰 Ego mask는 ON이다.

| 조합 | 전방 tensor | 좌우 tensor | 기본 batch |
|---|---|---|---|
| m/l・640 |384×640|480×640|2+1|
| m/l・960 |544×960|480×640|2+1|
| m/l・1280 |736×1280|480×640|2+1|

640 batch3 대조는 모두480×640으로 padding한다. 종횡비・FOV와 원본 좌표를 유지한다.
TensorRT profile의 최소 높이는384로 확장한다. checkpoint・embedding・engine SHA와
실제 I/O dtype을 기록하며, internal FP16을 모든 I/O가 FP16이라는 뜻으로 쓰지 않는다.

## 평가 경계와 진행 상태

- GPU engine 준비는 network-none 전용 컨테이너에서 직렬 수행한다. 실제 live trial은
  `asmc-ros-noetic`에서 bridge를 유지한 채 실행하며, 공개 차량 제어는 발행하지 않는다.
- 기존 Capture의 검증된 COCO7 세부 라벨만 정량 평가한다. 새 프롬프트 라벨을 기존
  7종 GT의 배경 FP로 세지 않는다. 미확인 Vehicle/Obstacle 영역도 자동 정답으로 바꾸지 않는다.
- 신규10에셋을 SUNNY/FOGGY・전방12/30/50m・좌우7m에서 수집했다.100장면・1500 RGB,
  스폰 오류0이며 시험 actor 정리・날씨 복원을 확인했다. Ego를 텔레포트하지 않았다.
  원본 RGB는 육안 외형 검토를 위한 것이며 actor 상태에서 pixel GT를 추정하지 않는다.
- BabyStroller1에는 밀고 있는 사람이, Scooter1/ElectricScooter1에는 탑승자가 포함된다.
  사람과 장비 검출의 공존은 동일 물체의 유사 이름 중복과 구분한다.
- 동일 RGB 품질 비교 → 6조합 live 선별 → 최대2후보 SUNNY/FOGGY 각3회 확인 →
  필요 이름만 실제 재추론 → 고정 confidence 확인 → 전방 bbox fusion 순서다.
- 목표15Hz 미달・8–9Hz・공급 공백・OOM 등 모든 실패를 남긴다. p95≤150ms,
  p99≤200ms・최대 공백250ms・전체VRAM90% 조건과 다른 모델 공급을 함께 본다.
- engine 준비・실측 결과가 모두 끝나기 전 후보 선정이나 운영 수락을 표시하지 않는다.

원본은 `artifacts/camera-yoloe-selection-v1`과
`artifacts/camera-coco7-live-v1/yoloe-obstacles-{range,sides}-v1`에 보존한다.
[갤러리 목차](http://localhost:8768/)에서 날짜・E번호로 접근한다.
[원본 스폰 장면](http://localhost:8768/camera-yoloe-selection-v1/spawn-source-review/index.html)은
검출 품질 측정 이전의 입력 검토 자료다.

## 직접 실행

준비된 engine과 SHA 고정 설정이 있는 경우에 사용한다. 기본은 YOLO만 ACTIVE이며
TwinLite/ResNet은 아래 옵션으로 명시적으로 함께 켤 수 있다. bridge・GUI는 별도다.

```bash
roslaunch asmc_camera camera_yoloe_experiment.launch model:=m front_width:=960
```

```bash
rqt_image_view --clear-config /perception/camera/objects/debug/mosaic/compressed
```

```bash
roslaunch asmc_camera camera_yoloe_experiment.launch model:=l front_width:=1280 \
  start_camera_lane:=true start_traffic_light:=true
```

모든 결과는 원본 프롬프트 라벨로 발행한다. 네 그룹 매핑은 평가에만 사용하며 기존
LiDAR 어댑터의 COCO7 제한은 유지한다. fusion은 전방 bbox・unique・50ms/200ms다.
3뷰 mask fusion을 구현된 기능으로 표시하지 않는다.

Roboflow SDK/CLI・22개 데이터 다운로드・라벨 감사・추가학습은 이번 범위에서 제외한다.
후속에서는 정리한 bbox로 YOLO26l detect와 선별 YOLOE detect만 짧게 학습하고,
유리한 하나를 본학습한다. 사전학습 seg 비교만으로 학습 후 성능을 단정하지 않는다.
