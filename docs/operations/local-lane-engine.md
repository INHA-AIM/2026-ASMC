# GPU별 TwinLite 차선 엔진

`camera.launch`의 기본 차선 모델은 TwinLite Large v7 E8이다. 저장소의
RTX 4060 Ti용 TensorRT 엔진은 RTX 4060용 엔진과 구분한다. GPU 이름이나
TensorRT 버전 검사를 해제하거나 manifest의 GPU 이름을 수정하지 않는다.

플래너와 카메라를 정지한 뒤, GPU를 사용하는 ROS 컨테이너의 저장소 루트에서
다음을 한 번 실행한다. 노드를 자동으로 재시작하지 않는다.

```bash
python3 -u tools/perception/prepare_local_lane_engine.py
python3 tools/perception/prepare_local_lane_engine.py --check
```

원본 Large v7 E8 checkpoint와 기존 TwinLite/ONNX/TensorRT 변환 환경이 필요하다.
첫 실행은 전방 및 좌우 엔진을 생성하므로 몇 분 걸릴 수 있다. exporter의
ONNX 비교와 엔진 SHA 검사를 통과한 결과만 캐시에 원자적으로 배치한다.
이미 존재하는 캐시는 확인만 하며, 잘못된 캐시를 덮어쓰지 않는다.

이후 평소 명령을 그대로 사용한다.

```bash
roslaunch asmc_camera camera.launch
```

packaged manifest가 현재 GPU/runtime과 일치하면 기존 경로를 유지한다.
일치하지 않으면 같은 checkpoint SHA·GPU·TensorRT에 해당하는 로컬 manifest를
선택하고 기존 checkpoint, 입력 크기, upstream, class, 엔진 SHA 검사를 모두 실행한다.
소스 워크스페이스의 캐시는 `diagnostics/camera_models/lane_engines`에 저장한다.
워크스페이스를 bind mount로 유지하면 컨테이너 재생성 후에도 재사용할 수 있다.
설치형 패키지의 기본 위치는 `~/.cache/asmc_camera/engines`이다. 다른 위치를 쓰려면 생성과
실행 터미널 모두에서 `ASMC_CAMERA_ENGINE_CACHE`를 동일하게 지정한다.
새 checkpoint/GPU/TensorRT 조합은 별도 캐시를 사용한다.
카메라 시작 로그의 `[TwinLite] selected TensorRT manifest`에서 실제 선택된 경로를 확인한다.

확인:

```bash
rosnode info /camera_lane
timeout 5s rostopic echo -n 1 /perception/camera/lane/status
timeout 5s rostopic echo -n 1 /planning/camera_bbox_lane_change/diagnostics
```

상태 토픽 발행과 실제 차선 결과의 신선도·유효성이 모두 확인되어야 한다.
GPU에서 엔진을 생성하고 MORAI 재밍 주행을 확인하는 절차는 CPU 네이티브
테스트로 대체하지 않는다.
