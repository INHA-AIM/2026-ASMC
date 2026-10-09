# 3뷰 COCO7 통합 실행·모델 선정

이 문서는 #73·#74·#75의 누적 작업을 흡수한 통합 작업의 시작점이다.
작업 worktree는 `asmc-camera-objects`, 브랜치는 `feat/camera-three-view-coco-runtime`이다.
`2026-ASMC`는 main checkout, `asmc-camera-verify`는 rename 재현용이다.
컨테이너 `/root/ws`가 어느 worktree를 읽는지 확인한 뒤 실행한다.
폴더명과 Git 브랜치명은 다르며, build/source만으로 마운트 대상은 바뀌지 않는다.

## 구현과 수락 상태

구현과 비교 시험은 완료했으며 운영 수락은 보류 중이다. 기존 기본 모델을
새 3뷰 후보로 승격하지 않는다. 과거 YOLOE vocabulary·seg·실패 증거는 유지한다.

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
   최신 측정기는 모델 로드·예열부터 GPU 메모리를 수집하고 90% gate를 적용한다.
   `resources`는 ACTIVE 구간, `resources_startup_and_active`는 로드·준비 포함이다.
   이전 실행의 ACTIVE 메모리로 시작 시점 peak까지 검증했다고 해석하지 않는다.
6. `analyze_coco7_input_gaps.py`: 실패한 출력 공백과 같은 시점의 원본 카메라
   공급 공백을 연결한다. 입력 공백이 설명된 실행도 원래 수락 실패는 유지한다.
7. `analyze_coco7_mask_control.py`: 동일 RGB SHA의 전체 mask ON/OFF 결과를
   같은 GT·confidence로 비교하고 신규 누락 전부를 이미지 갤러리에 남긴다.
8. `analyze_coco7_fusion.py`: 완전한 종료 집계, 고유/재사용 관측, 트랙별
   시간적 분류 확인 중복과 빈 결과 이후의 과거 관측 사용을 검사한다.
9. `summarize_coco7_live_matrix.py`: 실험 버전·모델·해상도·주기·batch별 세 반복을
   함께 판정한다. 실패 반복을 제외하지 않으며 미완성 묶음을 수락하지 않는다.
   잘못된 코드 SHA로 중단한 실행은 `REJECTED_PROVENANCE.json`과 원본을 보존하고
   정상적인 모델 성능 표본에서 제외한다.

`near-object-gallery/index.html`은 같은 RGB의 960/1280과 PyTorch/TensorRT를
비교하는 갤러리다. 파랑은 GT, 초록은 검출이며 slider는 저장된 예측을 필터링한다.
선정 자료의 일부 가까운 차량은 960에서 검출되지만 1280에서 미검출됐다.
해상도 확대를 무조건 품질 개선으로 간주하지 않는다. `summarize_coco7_geometry.py`의
GT 픽셀 높이별 평가는 크기별 진단이며 실제 거리·정량 거리 정확도가 아니다.

heldout 평가는 `evaluate_coco7_models.py --split heldout --frozen-confidence ...`로
선정 자료에서 정한 점수만 사용한다. heldout에서 점수를 다시 탐색하지 않는다.
Capture의 현재 검수 GT에는 car/truck/person/bus만 있으므로 dog/bicycle/motorcycle을
포함한 7종 전체의 정량 수락으로 확대하지 않는다. gRPC actor의 3D 상태와 영상
검토 갤러리는 나머지 종류의 관찰 자료이며 독립적인 픽셀 GT를 대체하지 않는다.

다섯 모델 × 전방 너비 세 가지의 같은 gRPC 촬영 RGB 비교는
`actor-all-models-fixed07/comparison.html`에서 두 모델을 선택해 확인한다.
각 프로필은 지면 배치 검사를 통과한 97개 fixture를 사용한다. 배치·스폰 실패
21개도 원본에 남겨 두었으며, actor의 기대 클래스가 검출됐다는 이유로 같은
actor에 정확하게 대응됐거나 픽셀 GT recall을 통과했다고 계산하지 않는다.
갤러리는 214개 같은 RGB 표본에 박스·원본 라벨·confidence·instance mask를 표시한다.

### Mask 수락 보류

960 입력의 YOLO26m-seg, confidence 0.65, 2+1 batch, 세 뷰 15Hz는 전체 스택
세 반복에서 처리 기준을 통과했다. 그러나 같은 RGB 2,252장의 black mask ON/OFF
대조에서 전방 1·좌측 1·우측 4건의 신규 GT 누락이 나왔다. 윤곽 안쪽 4픽셀과
중성색 96을 적용한 선택적 v2 후보는 기존 사례를 회복했지만 다른 전방 1·우측 1건이
생겼고 오검출 4건으로 같은 너비의 기준선 3건을 초과했다. detect 960/0.7의
black mask에서도 좌측 1·우측 1건이 나왔다. 기존 전방 누락 승인과 별개다.

부분 장면에서 회복됐다는 이유로 v2를 기본값으로 적용하지 않는다. v1의 black
설정과 실패 원본을 유지하며 v2는 명시적 실험 프로필이다. `coco7_masks.py`의
`--mask-profile black-v1|neutral-v2`로 동일 데이터 대조를 재현한다. OFF는 원본을
수정하지 않고 빈 제외 영역을 반환한다. v2의 fill/inset과 실제 raster 윤곽은 진단에
기록된다. 선정되지 않은 설정을 heldout 결과에 맞춰 다시 조정하지 않는다.

`full-mask-controls-selection/index.html`,
`full-neutral-mask-controls-selection/index.html`,
`full-detect-mask-controls-selection/index.html`에서 모든 신규 누락을 직접 확인한다.
`yolo26m_seg_coco7_three_view.yaml`은 처리 성능 후보이며 운영 수락된 기본값이 아니다.

실측에서 GPU의 박스 필드를 하나씩 Python scalar로 읽는 비용이 컸다. 박스를
한 번에 CPU로 옮기고, TwinLite가 구독자 없는 대형 debug 영상을 직렬화하지
않도록 수정했다. 클래스·좌표·순서·source header와 GPU 추론 결과는 유지한다.
이 수정 전후 실행은 별도 증거로 보존하며 수정 전 실패를 삭제하지 않는다.
수정 후 PyTorch–TensorRT 동일 입력 270개 비교도 다시 통과했다.

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

정상 빈 결과는 유효한 시간 관측이다. 최신 빈 결과는 이전의 비어 있지 않은 박스를
막지만, 시간 범위 안의 이전 빈 관측까지 지우지 않는다. 최초 재사용 시험에서는
모든 과거 stamp를 막아 음영 장면의 빈 관측을 대량으로 거부하는 결함이 확인됐다.
`FusionTimeMatcher.hpp::EligibleObservationStamp`와 실제 matcher 시험으로 이를
분리했다. 수정 전 실행은 `fusion-coco7-10hz-v2`에 보존하며 운영 수락으로 세지 않는다.
앞선 `fusion-coco7-10hz`는 뷰별 15Hz가 10Hz 요청을 덮어쓴 설정 오류다. 원본과
`REJECTED_PROTOCOL.json`을 보존한다. 실행기는 run-local YAML에 scalar와 뷰별 주기를
함께 고정하고 ROS의 실제 파라미터도 확인한다.

동일 영상이 여러 관측에 쓰이므로 `analyze_fusion_trace.object_state`는 trace 순서상
이미 적용된 최근 LiDAR 갱신을 연결한다. 미래의 재사용 갱신으로 과거 상태의 원인을
덮어쓰지 않는다. 기존 연속 C++ 재현기는 unique 소비 기준으로 보존하며, 그것의
통과를 reuse 정책 전체의 증명으로 확대하지 않는다. reuse의 실제 기록은 별도
`analyze_coco7_fusion.py`로 고유 증거·카운터 중복·빈 결과 barrier를 검사한다.

수정 후 10Hz 전체 반복은 `fusion-coco7-10hz-v3`에 있다. 주 bag의 unique 매칭은
404/394/400쌍, reuse는 455/455/457쌍(재사용 71/75/72쌍)이었다. 음영 bag은
unique 173/171/169쌍, reuse 173/174/167쌍이며 실제 재사용은 0쌍이었다.
음영 reuse의 세 번째 반복은 약 314ms 출력 공백으로 실패했다. 영상 입력 최대
공백은 약 92ms였으므로 센서 침묵으로 면제하지 않는다. 주 bag의 기존 세 실행에서
분류가 안정된 공통 관측 2,512개 중 새 정책에서 분류 차이 1개가 남았다.
음영의 안정된 공통 관측 1,001개에서는 차이가 없었다. 미대응 관측도 따로 기록한다.
숫자 track ID 대신 동일 LiDAR source stamp와 상호 유일한 1mm 중심 대응으로
비교했으며, 이 통계는 독립 GT의 정확도나 추적 recall이 아니다.

15Hz 대조의 주 bag은 unique 477/466/473쌍, reuse 478/478/484쌍이었다.
음영은 unique 177/178/175쌍, reuse 176/178/178쌍이며 여기서도 실제 재사용은
0쌍이었다. 안정된 공통 분류는 주 2,206개·음영 936개에서 차이가 없었지만,
주 unique 두 번째 실행에서 세 뷰 모두 307–321ms 출력 공백이 발생했다.
같은 시기 GPU 사용률과 계측 소요시간의 변화는 상관 관측이며 원인 확정이 아니다.
10/15Hz의 trace ON 재사용 검사는 총 8회 모두 카운터 중복·정책 위반 없이
완전한 종료 집계를 통과했다. 이를 trace OFF 처리율·분류 수락의 대체로 쓰지 않는다.

전체 실시간 스택의 15Hz reuse 대조 3회 중 첫 실행은 모든 모델의 공급 공백과
판제 객체 공급 deadline에서 실패했다. 나머지 두 실행의 통과로 이 실패를
제외하지 않는다. `live-finalist-reuse-v3`와 입력 공백 분석을 함께 보존한다.

`compare_coco7_fusion_trials.py`는 두 bag의 두 정책 각각 세 반복을 모두 요구한다.
실패 반복·분류 차이·미대응을 보존하고 설정 오류로 거부된 실험은 비교 입력으로
받지 않는다. 결과는 각 실험의 `policy-comparison.json`으로 남긴다.

```bash
python3 tools/perception/compare_coco7_fusion_trials.py \
  ../artifacts/camera-coco7-live-v1/fusion-coco7-10hz-v3 \
  --output /tmp/coco7-fusion-comparison.json
```

재사용은 선택적 실험 기능이며 기본 정책은 `unique`다. 총 매칭 수의 증가와
출력 지속성·분류 상태 수락은 각각 판단한다.

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

공유 `asmc-ros-noetic`은 현재 worktree를 `/root/ws`로 연결한 고정
Ultralytics 8.4.150 / TensorRT 10.13.0.35 이미지다. 바깥 workspace를 먼저
빌드·source하고 nested workspace를 빌드했다. 실제 CUDA 연산·engine 예열·생성
메시지 MD5와 `/vehicle/state` 50Hz, 세 카메라 20Hz·camera4 10Hz 입력을 확인했다.
컨테이너 재생성 전 셸에서는 나와 다시 진입한다.

```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source src/lidar+sensorfusion/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=localhost
```

```bash
roslaunch asmc_camera camera_objects_three_view_experiment.launch config:=/선정한/runtime.yaml
```

현재 후보의 객체 단독 확인은 아래와 같다. 후보 설정의 `target_hz_by_view`와
`front_width`, `conf_thres`를 수정하고 launch를 재시작해 비교한다. 목표 주기 서비스의
0은 파일의 뷰별 설정 유지, 양수는 모든 뷰의 주기를 같은 값으로 바꾼다.

```bash
roslaunch asmc_camera camera_objects_three_view_experiment.launch \
  config:=/root/ws/src/asmc_camera/config/camera/yolo26m_seg_coco7_three_view.yaml
```

TwinLite·신호등까지 화면을 확인하려면 앞선 객체 launch를 종료하고 실행한다.
이때 차선의 주행 소비는 계속 음영 ODD로 제한한다.

```bash
roslaunch asmc_camera perception.launch profile:=debug \
  camera_objects_config:=/root/ws/src/asmc_camera/config/camera/yolo26m_seg_coco7_three_view.yaml \
  objects_views:="[front, left, right]"
```

별도 LiDAR 터미널에서는 후보의 confidence를 어댑터에도 명시한다.

```bash
roslaunch Main main.launch start_yolo:=false rviz:=false \
  camera_objects_conf_thres:=0.65 fusion_yolo_policy:=unique
```

```bash
rqt_image_view --clear-config /perception/camera/objects/debug/image/compressed
rqt_image_view --clear-config /perception/camera/objects/debug/left/image/compressed
rqt_image_view --clear-config /perception/camera/objects/debug/right/image/compressed
```

위 실험 launch의 debug는 영상 발행이며 rqt 자동 실행이 아니다.
일반 `profile:=debug`는 객체 overlay 3개·mosaic·BEV·신호등의 6개 debug 출력이다.
`objects_debug_masks:=true`는 instance mask 영상 3개를 추가한다. 9개 영상 발행
부하는 별도 실험에서 세 실행 모두 실패했다. 6개 출력 대조는 모델 공급은
세 실행 모두 통과했지만 판제 trajectory deadline은 두 실행에서 실패했다.
debug OFF의 headless 공급 수락과 GUI 디버그 성능을 구분한다.
3뷰 raw7 공급 검사는 아래 도구를 사용한다. legacy
`verify_perception_runtime.py --camera-objects`는 전방 18Hz 계약 검사이므로
3뷰 저주기 후보 검사로 대신 사용하지 않는다.

```bash
python3 tools/perception/verify_coco7_runtime.py --seconds 20 --minimum-hz 9.5
```

전체 스택 계산 시험의 `coco7_preview_stack.launch`는 명령을 `/evaluation/ctrl_cmd*`로
격리한다. 실제 주행은 별도 gate·저속 시험이며 preview 성공으로 대체하지 않는다.
`drive_coco7_preview.py`는 이미 준비된 전체 preview를 5km/h 이하에서 확인하는
시험 도구다. preview 조향과 유효 trajectory의 속도를 사용하되 종방향 명령은
type2 속도 상한으로 변환한다. 이는 수정 없는 MPC 구동 성능 검증과 구분한다.
단일 공개 제어 발행자, 모델·영상·판제 입력 신선도, 충돌·경로 이탈·속도 상한을
감시하고 종료 후 Ego pose·gear·제어 모드를 확인한다. 실제 이동이 0.5m 미만인
명령 시도는 주행 성공으로 기록하지 않는다. 기존 actor·시나리오는 변경하지 않는다.

복원은 실제 정지·위치·roll/pitch/yaw·기어·제어 모드를 세 번 확인한다.
현재 MORAI에서는 `SetTransform`이 제어 모드를 AUTO로 바꾸고 약 1.34° yaw
차이를 만들 수 있었다. 관측된 오차를 한 번만 보정하고 모드·기어를 마지막에
복원한다. 임시 Ego pause는 명시적으로 해제하며 원래 pause 상태까지 복원했다고
주장하지 않는다. 복원 실패 시 다음 물리 시험을 중단한다. 최초 복원 실패·추가
회복 RPC는 `ego-restoration-recovery.json`에 남겨 두었다.

UDP 저속 대조에서는 `127.0.0.1` 및 Windows 호스트 목적지 모두 명령과 실제
전진이 맞지 않았다. 세 번의 작은 역방향 이동은 주행 성공으로 세지 않는다.
원래 자세·P 기어·키보드 모드로 복원했으며 기존 actor는 유지했다. UDP 송신 성공
로그는 MORAI의 제어 수신 증명이 아니다. gRPC 제어 진단은 별도 transport로
실행하고 UDP를 동시에 송신하지 않는다. 이 진단의 성공도 UDP/MPC 통합 구동
승인으로 확대하지 않는다.

gRPC API의 proto는 velocity 단위를 명시하지 않는다. 최초 진단에서 ROS의
km/h 숫자를 그대로 전달하자 실제 속도가 약3.6배로 접근했고5.5km/h guard가
중단했다. `drive-grpc-v1` 실패·복원 결과를 보존한다. 시험 gate에서만 m/s로
명시 변환한 `drive-grpc-mps-v2-r1/r2/r3`은 모두 전진·정지·복원과 세 모델의
재진입을 통과했다. 실제 전진거리는1.22–1.25m, 최고 속도1.77–1.88km/h다.
판제 STOP과 더 낮은 속도 명령은 존중했으며 실제5km/h 주행이나 UDP 수신·
원래 MPC actuator의 수락으로 확대하지 않는다. 운영 bridge 단위는 변경하지
않았다. `drive-grpc-mps-summary.json`이 세 실행과 모든 stream의 결과다.

## 이후

### 최종 회귀와 CI

실제 공유 workspace의 바깥·nested build를 사용한 network-none CPU 회귀를
`final-regression`에 보존했다. 카메라·메시지 등563개 중 기존 facade1건,
Main311개 모두 통과, LiDAR 좌표·시간 core3개 통과, 측정 도구131개 통과다.
생성 메시지를 사용한 모드/클래스/seg/batch/호환 어댑터 ROS 검사5개와
jamming fixture4개(내부36시험), 계약·문서 validator6개도 통과했다.

audit 전체51개 중 P1-C metadata authority 상태1건은 실패한다. 해당 코드·
테스트·HD맵 metadata와 facade 관련 코드가 main과 동일함을 SHA로 기록했다.
다른 대회 metadata를 변경하거나 검사 기준을 완화하지 않았다. 최초 audit
실행의 ASMC_DATA 마운트 누락2건과 CMake의 --test-dir 미지원으로 실제 시험이
없었던 실행도 보존하고, 올바른 환경·작업 위치의 수정 실행을 별도 기록했다.
jamming의 --text 종료 코드만으로 판정하지 않고 내부 unittest의 OK를 확인했다.

GitHub CI는 결제/사용 한도 제한으로 job의 첫 step부터 실행되지 못했다.
과거 HEAD의 CI 성공이나 로컬 회귀로 최종 CI 통과를 대체하지 않는다.
PR은 Draft이며 기본 모델·재사용 정책의 수락과 CI 확인이 남아 있다.
Git에 포함한 [최종 요약](../../../config/baselines/evidence/camera_coco7_live_v1/final_acceptance_summary.json)
과 baseline에서 원본 증거 경로·SHA·성공·실패를 확인한다.

### 30분 통합 실행

`live-sustained-30min`은 MORAI 렌더링을 유지한 정지 상태의 전체 스택
1800초 실행 1회다. YOLO26m-seg/전방960/2+1 batch/세 뷰15Hz에서 실제
YOLO 평균은 전방14.94·좌14.89·우14.89Hz, wall p95는91–92ms,
p99는104–106ms였다. TwinLite 뷰별 평균9.99Hz, ResNet 처리율9.94Hz,
최대 전체 VRAM5168MiB, 모델 오류0·중복/역행 stamp0이었다.

수락 결과는 **실패**다. YOLO 최대 공백409–447ms, TwinLite467–471ms,
ResNet400ms, 판제 trajectory/state/objects/control deadline 초과를 보존한다.
YOLO의250ms 초과 출력 공백은 뷰별13/12/12회이며 모두250ms 초과
카메라 수신 공백과 겹친다. 인과관계를 확정하거나 실패를 면제하지 않는다.
TwinLite 최소5초 창9.4Hz, ResNet9.2Hz도 기준에 미달한다.
ResNet의 신호등 부재에 따른 valid=false는 처리 실패와 구분한다.

카메라3종의 시작1–2분 대비 마지막1분 RSS 중앙값 증가는 각각
YOLO2.09·TwinLite5.72·ResNet0.49MiB였다. 30분만으로 메모리 누수가
없다고 확정하지 않는다. Fusion20617쌍·drop2266·종료 pending1/2를 기록했다.
screen 로그는 일부 interval이 끊겨 합계가 맞지 않았으며, master의 온전한
rosout interval358개와 종료 tail을 연결한 합계는 정확히 일치했다.
screen 원본도 보존한다. `sustained-30min-summary.json`과
`sustained-input-gaps.json`이 상세 결과다. 평균 성능만으로 기본 모델을
승격하지 않으며, 센서 전달·스케줄링 공백과 판제 deadline을 후속 해결한다.

이번은 추가학습 없이 일반 YOLO 모델·해상도·주기를 선정하는 단계다.
다음 YOLOE 장애물 확대에서는 원본 이름+네 소비 그룹을 별도로 비교한다.
단일 모델로 전 구간을 처리하는 구성을 우선하고, 부족할 때만 판제가 구간별
준비·활성화·가중치 전환을 요청한다. LiDAR 구조물·군집 개선은 재호의 인계 과제다.
