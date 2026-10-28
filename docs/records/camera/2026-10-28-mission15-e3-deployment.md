# Collect v5 E3 객체 모델 교체 — 2026-10-28

사용자가 선택한 `asmc_yolo26l_collect_v5_mission15`의 완료된 3에포크 EMA를
현재 객체 검출 기본으로 설정한다. 선택 근거는 고정 검증셋의 클래스별 macro F1이며,
`best.pt`의 mAP 선택이나 11에포크 micro F1 선택과 구분한다.

## 모델·환경

- 원본: `/data/training_runs/asmc_yolo26l_collect_v5_mission15/best-f1.pt`
  (`main/fit/weights/epoch2.pt`와 같은 선택).
- 원본 SHA256: `b272d096633553151540866309d854d133a4386c4c8790a5149a65175b8a89cd`.
- 추론용 PT: `src/asmc_camera/models/object_detection/yolo26l-mission15-e3.pt`.
  EMA의 1,092개 tensor/buffer와 dtype을 원본과 대조했다. 학습 상태만 제거했다.
- 추론 PT SHA256: `95cbc4eda62644b6f913c126d784e6b6d99078c20ee628c64b3b878b979429e2`.
- engine: 같은 폴더의 `yolo26l-mission15-e3.engine`. RTX5070 12GB에서 재생성한다.
- 런타임: `asmc-ros-noetic`, Python3.9, Torch2.7.1+cu128, Ultralytics8.4.150,
  TensorRT10.13.0.35 CUDA12. ROS 도구·빌드는 시스템 Python3.8을 사용한다.
- 전방960×544(내용960×540), 좌우640×480, 2+1 batch, FP16 내부 연산,
  confidence0.4·같은 클래스 NMS IoU0.5·차체 마스크를 유지한다.
- [현재 모델 목록](../../../config/models/camera_object_models_v3.json)과 YAML의 engine SHA가
  실제 배포 파일을 고정한다. 기존 E8·사전학습 파일과 [v2 목록](../../../config/models/camera_object_models_v2.json)은 보존한다.

현재 컨테이너에는 TensorRT가 없어 CUDA12 패키지를 설치했다. 같은 구성이 컨테이너
재생성 후에도 적용되도록 `Dockerfile.rtx50`과 고정 requirements를 수정했다.
기존 E8/사전학습 engine은 RTX4060Ti용이므로 RTX5070에서의 복구 실행을 보장하지 않는다.
기본 TwinLite Large 차선 engine의 manifest도 RTX4060Ti를 명시하고, backend가 GPU 이름을
검사한다. 이는 YOLO 교체와 별개의 기존 환경 제한이다. 이 기록의 실행 명령은
`models:=objects`만 대상으로 하며, RTX5070에서 `models:=all`을 사용하려면 차선 engine도
별도로 재생성·검증해야 한다. 차선 가중치·알고리즘은 이 교체에서 변경하지 않았다.

## 최신 메인 통합 후 연결 지점

기준 커밋은 `3b0bc98f26880934d8d1e99cc9248ffb591d402b`다. 기존 브랜치 HEAD는 이 커밋의
조상이므로 fast-forward했고, 작업 중 변경을 다시 적용하며 충돌10개를 해결했다.
메인에서 폐기한 nested Main은 복구하지 않았다. 변경 전 전체 작업은 Git stash
`8d58b4ba019beb1a4a0571fc193637164d4b912e`와 `refs/backup/pre-main-e3-20261028`에 보관한다.

| 지점 | 처리 |
|---|---|
| 카메라 모델·클래스 목록 | 15종 프로필 추가, traffic_cone 제외, stroller 모델 ID14 확인. 모델의 실제 이름 순서·SHA가 다르면 시작 거부 |
| 공개 launch·공유 worker | `detector:=e3` 기본, 선택별 가중치·그룹 설정 일치. `e8`·`pretrained`는 명시적으로 유지 |
| `/perception/camera/objects` | 원본 이름과 기존 세 그룹 전송. 숫자 YOLO ID는 외부로 전달하지 않음 |
| `CameraObjectObservation` | 3뷰 원본 이름·그룹·좌표·stamp·세션·보정/모델/설정 ID를 직접 전달. 구 `/YOLO/BBox` 어댑터는 폐기 |
| `asmc_lidar` 정본 경로 | `mission15_no_cone_v1` 프로필을 허용하고 기본으로 설정. `fusion.views=all`, 15종 허용 목록 지정. 객체 운동·기하와 최신 메인의 3뷰 매칭 유지 |
| MotionClassifier | 위치·속도·이력으로 실제 이동 상태 판단. YOLO ID나 클래스 목록 길이에 의존하지 않으므로 알고리즘 변경 없음 |
| planning_inputs·플래너 | 기존 vehicle/pedestrian/unknown enum 유지. 이름/그룹 소비이므로 stroller ID 변경을 반영할 숫자 수정 없음 |
| 시각화 | 일반 renderer는 수신된 클래스명을 표시. 명시적인 이전 프로필 선택과 기존 진단 renderer 설정도 유지 |
| 로컬 엔진 준비 도구 | E8 고정을 해제하고 선택한 프로필의 checkpoint·head를 검증. RTX50 GPU venv와 ROS Python 환경을 분리 |
| scen_runner 로컬 설정 | E8용 `camera_runtime.json`을 백업·해제. 패키지의 E3 RTX5070 엔진을 사용 |

카메라→라이다 흐름은 다음과 같다.

```text
카메라 3뷰 YOLO → 원본 클래스명·그룹·원본 영상 bbox
  → /perception/camera/objects/observations (CameraObjectObservation)
  → asmc_lidar 전방·좌측·우측 fusion → classification 그룹 + camera_class 원본 이름
  → planning_inputs 및 판제
```

**현재는 최신 메인의 3뷰 융합을 사용한다.** 입력 해상도와 별개로 bbox는 원본 영상 좌표로
복원되므로 LiDAR 카메라 보정값을 960 기준으로 바꾸지 않는다.
`dynamic_obstacle` 그룹이 실제로 움직인다는 뜻은 아니다. car/bus/truck은 vehicle,
person/dog/deer/bicycle/stroller는 dynamic_obstacle, 나머지7종은 static_obstacle이다.
15종 모델 대응을 위해 최신 메인의 메시지 정의·시간 매칭·추적·플래너 의사결정을 변경하지 않았다.

## 메인 통합 전 모델 검증 자료

작업 원본·로그는 `/data/models/camera_mission15_collect_e3_rtx5070`에 보관한다.

- `deployment_source.json`: 원본/추론본 SHA 및 모든 EMA tensor 비교.
- `yolo26l.onnx.provenance.json`, `yolo26l.engine.provenance.json`: 변환 환경·형상·해시.
- `engine-parity.json`: 실제 카메라 backend로 동일 영상의 PT/engine 박스·confidence와 batch 조합 비교.
- `ros-smoke/report.json`: 독립 ROS master에서 기록 영상 입력→3뷰 GPU 추론→Main 전방 어댑터 확인.
- `runtime-evaluation/`, `runtime-quality-comparison.json`: 실제3뷰 검증 영상 전체의 engine 정확도 비교.
- 빌드 로그: `contracts-build.log`, `lidar-library-build.log`, `main-build.log`, `shadow-build.log`.

Main과 shadow·planner 관련 C++ 테스트84개, 관련 Python 테스트58개가 통과했다.
클래스명 전달, stroller ID 이동, cone 제외, 원본 픽셀 복원, 기존 프로필 선택을 확인했다.
독립 ROS 시험은15초 동안 전방·좌측·우측 각각224프레임을 받았고, 전방224프레임을
Main 어댑터가 클래스명·좌표·stamp 오류 없이 전달했다. stroller 출력과 dynamic_obstacle
그룹도 확인했으며 두 출력 토픽의 생산자는 각각1개였다. 시험 노드와 private master는 종료했다.

표본166장의 PT/engine 비교에서는479개 박스가 연결됐다. 엄격한 기준(박스 IoU≥0.98,
confidence 차이≤0.02)은8건이 미통과했고, 별도로 confidence0.4 근처의 검출 차이가1건 있었다.
이를 동일 출력으로 승인하지 않고 전체3뷰4,126장·GT11,464개의 동일 평가로 영향을 확인했다.

| 동일 검증 범위 | Precision | Recall | Macro F1 | Micro F1 |
|---|---:|---:|---:|---:|
| 원본 E3 PT FP16 | 94.6225% | 87.7966% | 91.4746% | 91.0819% |
| 배포 E3 TensorRT | 94.5885% | 87.8227% | 91.4770% | 91.0802% |

GT·이미지·마스크·지도 무시영역·confidence·NMS·정답 매칭 기준을 유지했다.
정확히 동일한 박스는 아니지만 macro F1 차이는+0.0024%p, micro F1 차이는−0.0017%p다.
클래스별 F1 차이는−0.117~+0.096%p 범위다. 별도의 rear/unknown 검증 이미지는
실행 중인3뷰 입력 형상과 다르므로 이 배포 비교에서 제외했다.

실제 MORAI 주행·전체 인지 스택 동시 부하는 별도 확인 대상이다. 기록 영상 시험의
처리 시간이나 통과 결과를 현장 주행 성능으로 대체하지 않는다.

## 최신 메인 통합 검증

[통합 결과](../../../config/baselines/evidence/camera_mission15_e3_deployment_v1/main-integration.json)와
`/data/quality_checks/rebase_main_20261028T174143`의 로그를 따른다.

- RTX50 컨테이너의 전체 루트 ROS workspace 빌드 통과. 소스와 LiDAR 실행파일의 build identity 일치 확인.
- LiDAR 융합·runtime·공용 출력·판제 변환 C++139건 통과. 15종 각각을 세 뷰에 입력하고
  stroller 원본 이름·그룹 보존과 cone 거부를 확인했다. 기존 GPS 손실·복구와 3뷰 교대 시험도 통과했다.
- LiDAR ROS 통합19건 통과. 카메라 detection90건, runtime124건 중123건 통과/1건 skip,
  카메라→LiDAR 설정 계약5건, 로컬 엔진 선택3건 통과.
- 독립 ROS master에서 E3 RTX5070 실제 추론: 전방·좌측·우측 각각223프레임.
  세 뷰 모두 `CameraObjectObservation`과 실제 LiDAR 수신부에 도달했고, 이름·그룹·좌표·stamp
  전달 오류0건. `fusion_views=all`, 합성 LiDAR223scan 처리, invalid/stamp error0건.
- 시험은 저장 RGB와 합성 LiDAR/IMU/CAN을 사용했다. 실제 공간 정합·주행 성능 검증을 뜻하지 않는다.
  시험용 노드와 master는 종료했다. MORAI 차량 제어는 실행하지 않았다.
- 기존 E8용 로컬 엔진 선택 파일은 같은 로그 폴더의 `camera_runtime.pre-e3.json`에 보관한다.
  현재는 패키지의 E3 RTX5070 엔진을 그대로 사용하며 엔진 SHA·confidence0.4·NMS0.5를 유지한다.

## 실행

현재 RTX50 컨테이너에서 바깥 workspace를 source하고 실행한다.

```bash
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash
roslaunch asmc_camera camera.launch models:=objects
```

시각화는 별도 터미널에서 같은 환경을 source한다.

```bash
roslaunch asmc_camera view.launch models:=objects
rosrun rqt_image_view rqt_image_view --clear-config /perception/camera/objects/debug/mosaic/compressed
```

LiDAR도 루트 workspace만 source하고 별도 터미널에서 실행한다.

```bash
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash
roslaunch asmc_lidar lidar.launch
```

기본 `fusion.camera_class_profile`이 새15종과 일치한다. 이미 켜진 카메라/LiDAR는 재시작해야 새
설정·실행파일을 읽는다. 같은 토픽의 detector를 두 개 띄우지 않는다.
이전 E8로 비교할 때는 GPU에 맞는 engine을 준비하고 카메라/시각화 `detector:=e8`,
LiDAR 설정 파일의 `fusion.camera_class_profile: mission16_v1`을 함께 지정한다.
