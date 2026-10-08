# 3뷰 COCO7 통합 실행·모델 선정

이 문서는 #73·#74·#75의 누적 작업을 흡수한 통합 작업의 시작점이다.
작업 worktree는 `asmc-camera-objects`, 브랜치는 `feat/camera-three-view-coco-runtime`이다.
`2026-ASMC`는 main checkout, `asmc-camera-verify`는 rename 재현용이다.
컨테이너 `/root/ws`가 어느 worktree를 읽는지 확인한 뒤 실행한다.
폴더명과 Git 브랜치명은 다르며, build/source만으로 마운트 대상은 바뀌지 않는다.

## 구현과 수락 상태

현재는 모델 선정 실험 중이다. 새 3뷰 후보를 측정하기 전에 운영 기본 모델을
선정 완료로 표시하지 않는다. 과거 YOLOE vocabulary·seg·실패 증거는 유지한다.

- raw 출력: `person, dog, bicycle, car, motorcycle, bus, truck`.
  새 프로필은 `class_name == source_class_name`이며 네 그룹으로 매핑하지 않는다.
  과거 group 프로필은 별도로 보존한다. 메시지 필드·MD5는 유지한다.
- 전방·좌우의 각 Ego polygon mask를 기본 적용한다. 원본 JPEG·화질·좌표·header를
  바꾸지 않고 YOLO 입력 복사본만 가린다. TwinLite·ResNet 입력은 유지한다.
- TensorRT detect/seg 공통 backend는 정확한 engine SHA·task·head·CUDA를 검사한다.
  실패 시 CPU/PyTorch로 조용히 전환하지 않는다. seg mask 복원 비용도 측정한다.
- 원본 전방 1280×720에서 tensor 너비 640/960/1280을 비교한다.
  대응 tensor 높이는 384/544/736이다. 좌우는 640×480을 유지한다.
  batch3에서도 좌우를 확대하지 않고 padding한다.
- `target_hz_by_view`는 뷰별 목표다. 모드 서비스의 `target_hz: 0`은 이를 유지하고,
  양수 요청은 모든 뷰를 해당 주기로 통일한다. 처리 완료 결과를 복제하지 않는다.
- TwinLite 목표는 뷰별 10Hz, 입력 tensor 640×384다. YOLO와 Hz를 강제로 맞추지 않는다.
  차선 결과의 주행 소비는 음영 ODD에 한정한다.

## 시험 도구·증거

실험 조건은 [camera_coco7_live_v1.yaml](../../../config/experiments/camera_coco7_live_v1.yaml)에 둔다.
원본 증거는 상위 workspace `artifacts/camera-coco7-live-v1`, 모델은
`data/models/camera_coco7_v1`이다. 전체 checkpoint·engine을 Git에 넣지 않는다.

1. `prepare_coco7_models.py`: checkpoint/ONNX/engine SHA, 라이브러리·dynamic shape 기록.
2. `capture_coco7_actors.py`: gRPC 실제 목록 대조, owned actor 촬영·정리.
   actor 3D 박스를 검수한 영상 GT로 간주하지 않는다. 요청 z와 실제 생성 z가
   다를 수 있으며 pedestrian SetTransform은 이 버전에서 지면 상대 z다.
   공중 배치 v1/v2 실패도 보존한다. 실제 위치와 영상 검토를 함께 사용한다.
3. `coco7_scene.py`: 7종·3뷰에 객체가 있는 고정 연산 부하 장면.
   기존 actor는 유지하고 registry에 생성 성공이 기록된 시험 actor만 정리한다.
4. `evaluate_coco7_models.py`: 고정 selection/heldout 영상, raw 클래스별 일대일 IoU 0.5.
   세부 종류가 미확인인 Vehicle/Obstacle 영역은 TP·FP로 임의 판정하지 않는다.
   GT가 없는 종류는 평가 불가다. confidence sweep과 원본 예측·mask·갤러리를 보존한다.
5. `run_coco7_live_trials.py`: 모델 3개 상주, PREPARE 실제 영상 준비 확인 후 ACTIVE,
   wall 입력 수신→출력 수신 지연·뷰별 공급률·200ms 리소스 기록.
   모든 실패를 보존하고, 최소 실행 가능성과 목표 Hz 달성을 구분한다.

## 전방 fusion 재사용 실험

`Main main.launch fusion_yolo_policy:=unique|bounded_reuse`로 비교한다.
두 방식 모두 ±50ms와 200ms 대기를 유지한다. 결과를 재사용할 때도 해당 LiDAR
관측으로 다시 투영·기하 연관한다. 같은 source stamp/epoch를 같은 track의
분류 확인·miss에 두 번 반영하지 않는다. 정상 빈 관측 이전의 검출을 되살리지 않는다.
STANDBY·세대 변경·producer 종료·전방 결과 침묵은 버퍼를 무효화한다.

fresh/reused 매칭 수를 분리한다. 총 매칭 수 증가만으로 객체 정확도 개선을 주장하지 않는다.
카메라 ID·source stamp·epoch·보정 식별자는 내부 관측에 보관하고 투영에 보정값을
명시적으로 전달한다. 이번 운영 연결은 전방 bbox뿐이다. 3뷰 mask fusion은 후속 구현이다.
legacy classification의 car 범주와 별도로 `yolo_class`에는 원래 7종 이름을 유지한다.

## Jamming 연결

`lane_boundary_adapter camera_bbox_detector.launch` 기본은 공유 카메라 전방 결과의
Msg/BBox 변환이다. 별도 YOLO를 실행하지 않는다. `detector_source:=legacy`는
과거 21% crop 생산자를 명시적으로 켠다. 기존 주행·차선 변경 튜닝값은 유지한다.
새 소비 경로는 crop 0과 동일 Ego mask를 사용하고, 박스 하단 접점을 차체가 가리면
박스를 보존한 채 UNKNOWN/UNOBSERVED로 보고한다. 이런 박스를 삭제하거나
이전 차로 소속을 근거 없이 유지하지 않는다.

## 직접 실행

컨테이너 진입·bridge와 rqt는 별도다. 모든 ROS 터미널에서 같은 outer/nested 환경과
master를 source한다. 전용 빌드의 devel 경로와 공유 workspace devel을 혼용하지 않는다.

```bash
roslaunch asmc_camera camera_objects_three_view_experiment.launch config:=/선정한/runtime.yaml
```

```bash
rqt_image_view --clear-config /perception/camera/objects/debug/image/compressed
rqt_image_view --clear-config /perception/camera/objects/debug/left/image/compressed
rqt_image_view --clear-config /perception/camera/objects/debug/right/image/compressed
```

위 실험 launch의 debug는 영상 발행이며 rqt 자동 실행이 아니다.
전체 스택 계산 시험의 `coco7_preview_stack.launch`는 명령을 `/evaluation/ctrl_cmd*`로
격리한다. 실제 주행은 별도 gate·저속 시험이며 preview 성공으로 대체하지 않는다.

## 이후

이번은 추가학습 없이 일반 YOLO 모델·해상도·주기를 선정하는 단계다.
다음 YOLOE 장애물 확대에서는 원본 이름+네 소비 그룹을 별도로 비교한다.
단일 모델로 전 구간을 처리하는 구성을 우선하고, 부족할 때만 판제가 구간별
준비·활성화·가중치 전환을 요청한다. LiDAR 구조물·군집 개선은 재호의 인계 과제다.
