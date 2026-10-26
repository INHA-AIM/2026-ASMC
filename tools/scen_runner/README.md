# scen_runner — 전체 씬·단일 구간 검증

카메라 실행은 현재 `asmc_camera/camera.launch mode:=active`를 사용한다. Main은 외부 전방 검출의 CPU 어댑터만 실행하고 독립 YOLO 옵션은 제거됐다. 카메라 설정은 카메라 패키지가 소유하며 [LiDAR 인계](../../docs/handoffs/camera-to-lidar.md)를 따른다.

MORAI에 객체·환경을 배치하고 ego의 준비 정지, 충돌/도착/정체에 따른 다음 주행과 최종 시작점 복귀를 처리한다. `run_validation.sh`는 기존처럼 키보드 또는 외부 제어기를 사용한다. `run_scenario.sh`는 별도 판제 실행기와 연동하여 리스폰마다 판제·MPC를 재시작하고 준비를 확인한다. 시나리오 자체는 `/ctrl_cmd`를 발행하지 않는다.

## 폴더 역할

```text
tools/scen_runner/
├── run_bridge.sh               # 기존 UDP ↔ ROS 브릿지 실행
├── run_perception.sh           # LiDAR + 카메라 차선/YOLO/신호등
├── run_planning.sh             # 시나리오 요청에 따라 판제·MPC 재시작
├── run_scenario.sh             # 판제 연동 전체 씬·단일 구간·재현
├── run_validation.sh           # 전체 씬·단일 구간·저장 기록 재현
├── run_runtime_trace.sh        # DLL 수정본 빌드·설치·원복
├── config/                     # 기존 YAML 설정, 파일 위치 유지
│   ├── local.example.yaml      # PC별 MORAI 주소·DLL 경로 작성 예시, Git 포함
│   └── local.yaml              # 자신의 PC 설정, Git 제외
├── launch/                     # 팀 launch를 포함하는 인지·판제 구성
├── scen_runner/                # Python 패키지
│   ├── config.py               # 공유 설정/저장소 경로
│   ├── ui/                     # GUI와 실행 진입점
│   ├── runtime/                # 씬 수명주기·객체 관리·환경·종료/정체 판정
│   ├── planning/               # 지도·차선·객체 배치·횡단 계산
│   ├── morai/                  # gRPC 통신·생성점·접촉 조회·DLL 명령
│   ├── orchestration/          # 프로세스 소유권·준비 검사·리스폰 요청/응답
│   └── records/                # 접촉/정체 저장·재현·진단 기록
├── dll/
│   ├── src/                    # 수정 C# 소스
│   ├── scripts/                # 빌드·설치·원복 PowerShell
│   ├── tests/                  # DLL 동작/원복 검사
│   └── builds/                 # 기존 빌드 결과와 원복 확인 해시, Git 제외
├── test/
│   ├── unit/                   # 배치/판정 계산 검사
│   ├── integration/            # 생성·상태 전환·통신·저장/재현 검사
│   └── ui/                     # GUI·CLI·재현 화면 검사
└── save_collision_log/         # 기존 저장 기록 위치, Git 제외
```

기존 검증 명령, YAML 경로, 저장 파일 위치와 배치/주행 설정은 유지한다. 네 실행기의 연동 설정은 `config/stack.yaml`에 추가한다. 설치된 Simulator DLL과 DLL 빌드 결과는 이 실행기 연동으로 변경하지 않는다.

## 처음 사용하는 PC의 설정

저장소 루트에서 예시를 복사하고 `tools/scen_runner/config/local.yaml`을 편집한다. WSL과 컨테이너가 같은 저장소를 공유한다면 한 번만 작성한다. `cp -n`은 이미 작성한 개인 설정을 덮어쓰지 않는다.

```bash
cp -n tools/scen_runner/config/local.example.yaml tools/scen_runner/config/local.yaml
```

| 개인 설정 항목 | 입력할 값 |
|---|---|
| `morai.host` | 컨테이너에서 접속할 MORAI Windows 호스트의 IP 또는 호스트명. PC마다 다를 수 있으므로 직접 입력한다. |
| `morai.port` | MORAI gRPC 서버 포트. 기본값은 `7789`이며 UDP 포트와 다르다. |
| `dll.managed_path` | 자신의 `Simulator_Data/Managed` 전체 **WSL 경로**. 예: `/mnt/c/MORAI/Simulator/Simulator_Data/Managed`. DLL 도구를 사용하지 않으면 `null`로 둬도 된다. |

시나리오는 `static_obstacle.yaml`을 읽은 뒤 개인 설정의 `morai.host`와 `morai.port`를 적용한다. 주소가 비어 있으면 GUI·판제 연동·MORAI 연결을 시작하기 전에 설정 파일과 입력 항목을 안내한다. 맵·객체·속도·날씨·반복 횟수는 기존 공유 YAML에서 관리하며 개인 설정으로 덮어쓰지 않는다. 개인 설정 없이도 `--help`와 오프라인 배치/재현 자료 검사는 사용할 수 있다.

DLL 도구의 경로 우선순위는 **명령의 `--managed` → `local.yaml`의 `dll.managed_path`**다. 두 값이 모두 없으면 안내 후 종료하며, 개발자의 설치 경로를 기본값으로 사용하지 않는다. `--managed`를 직접 지정하면 개인 설정 파일 없이도 DLL 도구를 사용할 수 있다. DLL 경로 변경은 실제 설치본 버전/해시 검사를 우회하지 않는다.

Git에는 `local.example.yaml`만 포함한다. **`local.yaml`은 Git 제외 대상**이며 충돌 기록의 코드·설정 지문 목록에서도 제외한다. GPU 모델별 로컬 설정은 기존 `.runtime/camera_runtime.json`을 계속 사용하며 역시 Git에 포함하지 않는다.

## 네 실행기로 인지·판제·시나리오 연동

기존 판제/인지 launch와 GPU 신호등을 따로 실행해둔 경우 그 터미널에서 종료한 뒤 아래 구성을 사용한다. 실행기는 같은 이름의 노드나 `/ctrl_cmd` 등의 중복 발행자가 있으면 시작을 거부하며, 다른 실행기의 프로세스를 종료하지 않는다. 모든 명령은 같은 `asmc-ros-noetic` 컨테이너의 `/root/ws`에서 실행한다.

WSL에서 `docker start asmc-ros-noetic`을 한 번 실행하고, 각 터미널에서 `docker exec -it asmc-ros-noetic bash`로 들어간다. 네 터미널 모두 `cd /root/ws` 후 아래 명령을 순서대로 실행한다.

| 터미널 | 명령 | 담당 |
|---|---|---|
| 1 | `./tools/scen_runner/run_bridge.sh` | 기존 local UDP ↔ ROS 브릿지, ROS master |
| 2 | `./tools/scen_runner/run_perception.sh` | LiDAR/추적/퓨전, YOLO, TwinLite 차선, 신호등 |
| 3 | `./tools/scen_runner/run_planning.sh` | 판제 실행기; 시나리오 요청 전에는 주행 노드를 시작하지 않음 |
| 4 | `./tools/scen_runner/run_scenario.sh --mode scene --seed 42` | 전체 씬 배치·반복과 판제 재시작 연동 |

구간별 검증은 터미널 4에서 `./tools/scen_runner/run_scenario.sh --mode section --section dynamic`처럼 실행한다. `--section`에는 `static`(정적), `dynamic`(동적), `roundabout`(회전교차로), `high_speed`(고주로), `tunnel`(터널)을 지정한다. 기존 YAML 주행 횟수·객체 배치·속도·날씨 설정과 저장/재현 기능을 사용한다. 저장 기록의 판제 연동 재현은 아래 [충돌·정체 저장과 재현](#충돌정체-저장과-재현)의 `run_scenario.sh --replay-browser` 명령을 사용한다. 키보드/외부 판제 검증은 `run_validation.sh`를 그대로 사용한다.

구간별 검증은 `config/validation.yaml`의 `section_traffic_lights` 설정에 따라 해당 진입 방향의 신호등을 **직진·좌회전 초록불**로 유지한다. 현재 정적구간은 `C1256W000091`/`IntTL7` 단계 0, 동적구간은 `C1256W000016/018`/`IntTL3` 단계 2가 대상이다. 회전교차로·고주로·터널 구간은 대상이 없다. 전체 씬은 신호를 고정하지 않는다. 기능을 끄려면 `enabled: false`로 설정한다.

개별 신호의 `impulse=False` 설정만으로는 교차로 단계가 전환될 때 초록이 풀린다. 따라서 별도 작업 스레드에서 1초마다 대상 교차로의 초록 단계 경과 시간을 초기화하고 대상 신호 색상을 조회한다. 같은 교차로의 다른 진입 방향도 해당 단계의 신호를 따르며, 다른 교차로는 변경하지 않는다. 객체 준비·판제 대기 중에도 갱신하며, 적용/유지 확인이 실패하면 출발을 차단하거나 ego를 정지시킨다.

정적구간의 원래 단계는 직진 초록(16)이므로 단계 갱신과 직진·좌회전 초록(48) 적용 사이에 좌회전 화살표가 잠깐 꺼질 수 있다. 이때도 직진 초록은 유지된다. 동적구간은 원래 단계부터 직진·좌회전 초록(48)이다.

GUI 구간 변경, 다음 반복, 정상 종료 시 갱신 스레드의 종료를 확인하고 진입 전 교차로 단계에서 자동 주기를 재개한다. 해당 단계의 경과 시간은 0부터 재개되며, 저장된 교차로 스케줄/주기 길이는 수정하지 않는다. 강제 종료·통신 단절 시에도 단계 갱신이 끊기면 기존 주기에 따라 다음 신호로 진행한다. 충돌·정체 저장 파일에는 신호 ID/색상과 교차로 단계도 저장한다. 신호 설정이 없던 이전 재현 기록에는 초록불을 추가하지 않으며, 초기 초록불 기록은 해당 구간의 교차로 단계 정보를 보완해 재현한다.

GUI의 `선택 모드로 시작`과 `다음 반복 · 다시 준비`도 같은 판제 종료/재시작 경로를 사용한다. 구간 선택을 바꾼 뒤 다시 준비를 누르면 선택한 구간을 적용하고 해당 구간의 주행 횟수를 새로 시작한다. 같은 구간의 다음 반복은 기존 주행 횟수 안에서 진행한다. GUI의 `판제 연동: 켜짐/꺼짐`과 판제 터미널의 `[판제 종료 확인]`, `[판제 재시작]` PID 로그로 확인한다. 재시작 후 `objects 미수신` 등으로 대기하는 상태는 판제가 시작됐지만 출발 입력이 준비되지 않은 상태다.

리스폰마다 **ego 정지 → 판제 노드 종료 확인 → 시작점 복귀/객체 배치 → 차량 상태 추정·판제·MPC·제어 선택기 재시작 → 새 입력 준비 확인 → 주행 허용** 순서로 처리한다. 종료 요청은 최대 `stop_timeout_s` 동안 확인하며, 실패하면 위치 이동/출발을 진행하지 않는다. 마지막 회차에는 판제를 종료하고 시작점으로 복귀한 뒤 ego 정지를 유지한다. 수동 종료도 소유한 판제를 정리한다. 시나리오가 비정상 종료되거나 응답을 멈추면 판제 실행기가 자신이 띄운 노드를 정리한다.

판제 초기화 중 구간을 바꾸면 강제 종료된 노드의 등록이 ROS master에 남을 수 있다. 새 판제 worker는 실행할 노드 이름 중 로컬 프로세스가 없고, 동일한 로컬 API 주소에서 두 번 연결 거부가 확인된 등록만 해제한다. 정리 시 `[판제 잔류 등록 정리]`를 출력한다. 응답하는 노드·초기화 중인 프로세스·통신 시간 초과·원격 주소·다른 노드 이름은 자동 정리하지 않고 기존 중복 실행 검사를 유지한다.

회전교차로 준비 단계에서는 NPC 1대를 생성한다. 판제 입력 준비와 ego 주행 허용이 완료된 뒤 설정된 주기로 추가 생성한다. `판제 연동 실패` 상태에서 1대만 보인다면 생성 대수 제한이 아니라 판제 시작 오류부터 확인해야 한다.

준비 검사는 새 판제 실행 시각 이후의 `map` 좌표 차량 상태·객체 배열·경로와 제어 출력을 요구한다. 차량 상태와 경로의 `ego_at_plan`은 새 출발 위치/방향 및 정지 조건에 맞아야 하고, 경로의 `failure_reason`은 0이어야 한다. 객체가 0개라는 새 관측과 객체 미수신을 구분한다. 입력이 일정 시간 일치해야 준비 완료로 인정한다. 시간 초과/통신 단절은 ego 정지와 원인 로그로 표시한다. 실제 주행 중의 센서/경로 유효성 판정은 팀 판제·MPC가 계속 담당한다.

연동 설정은 `config/stack.yaml`에 있다. `instance`가 같은 실행기들이 `.runtime/stack/<instance>/`의 회차별 요청/응답과 heartbeat로 연결된다. 기존 회차의 준비 응답을 새 회차에서 재사용하지 않는다. 모든 실행기에 `--stack-config <같은 YAML 경로>`를 전달해 다른 설정을 사용할 수 있다.

인지 실행기는 `asmc_camera/camera.launch`와 선택한 LiDAR를 실행한다. `config/stack.yaml`의 `lidar_backend: main`은 기존 Main을, `asmc_lidar`는 새 typed 판제 연결을 선택한다. 저장소 기본은 `asmc_lidar`이며, 기존 사용자별 설정에 키가 없으면 `main`을 유지한다. 실제 시뮬레이터 출구 수락은 별도다. [전환과 복구](../../docs/operations/lidar-cutover.md)를 따른다. 카메라 차선·YOLO·신호등 세 GPU 노드의 Python만 이미지의 `ASMC_YOLO_PYTHON`으로 선택한다(RTX50: `/opt/yolo-venv/bin/python`, RTX40: 시스템 Python). CPU의 BEV·차선 후처리, LiDAR·추적·퓨전은 기존 인터프리터/실행 파일을 사용한다. 모델/임계값/제어 파라미터는 팀 소스의 YAML/launch를 읽는다. 차선 제어의 BBOX 입력은 카메라 정본 `/perception/camera/objects`의 전방 결과를 직접 받는다. Main 실행 여부와 무관하다.

`config/stack.yaml`의 `camera_runtime: auto`는 로컬에 `.runtime/camera_runtime.json`이 있으면 검증된 GPU용 엔진을 선택한다. RTX 5070용 E8 모델은 같은 학습 체크포인트로 변환한 `/data/models/camera_mission16_v11_e8_rtx5070/yolo26l.engine`을 사용하고, TensorRT·Ultralytics 의존성은 `.runtime/inference_rtx50/python`에서 객체 인식 노드에만 추가한다. 팀 launch의 엔진 경로와 검증 SHA만 바꾸며 클래스·임계값·배치·해상도 설정은 그대로 읽는다. 실행 시 `[카메라 엔진]`에 선택 경로와 SHA가 표시된다.

로컬 프로필과 의존성은 Git에 포함되지 않는다. 프로필이 없는 PC는 팀 기본 엔진을 사용하며, GPU가 다른 PC는 해당 GPU에서 같은 체크포인트를 변환·검증한 뒤 프로필을 준비해야 한다. 프로필에는 원본 배포 엔진 SHA, 변환 엔진 SHA, 통과한 추론 검증 기록이 필요하다. 팀 모델이 바뀌어 원본 SHA가 달라지면 예전 로컬 엔진을 계속 쓰지 않고 시작 오류로 알린다. `camera_runtime: off`로 팀 기본 엔진 선택으로 되돌릴 수 있지만, RTX 4060 Ti용 배포 엔진은 RTX 5070에서 사용할 수 없다.

RTX 4060처럼 배포 엔진의 GPU와 다른 PC에서는 MORAI와 카메라·판제 실행을 종료한 뒤,
같은 컨테이너에서 아래 명령으로 현재 E8 객체·Large E8 차선 모델을 다시 변환할 수 있다.

```bash
cd /root/ws
bash tools/scen_runner/prepare_camera_engines.sh --check  # 변경 없는 사전 점검
bash tools/scen_runner/prepare_camera_engines.sh
```

빌드에 필요한 ONNX Runtime 의존성만 `.runtime/engine_build_python`에 설치하고,
원본 `.pt`와 공용 `.engine`을 보존한 채 `.runtime/camera_engines/`에 새 파일을 만든다.
객체·차선의 기본 추론 검사가 모두 성공하면 `.runtime/camera_runtime.json`을 원자적으로
갱신한다. 기존 프로필은 새 산출물 폴더에 백업한다. `READY` 출력 후 기존
`run_perception.sh`가 두 새 엔진을 선택한다. 중간 실패 시 실행 프로필은 유지된다.
이는 GPU 로딩·기본 추론 검사이며 실제 영상 인식 정확도·처리 속도·주행 검증을 대신하지 않는다.
이 명령은 ROS 노드나 차량 제어를 시작하지 않는다.
엔진 생성은 끝났지만 마지막 검사에서 실패했다면 처음부터 생성하지 않고
`bash tools/scen_runner/prepare_camera_engines.sh --resume <출력된 Artifacts 절대경로>`로
기존 파일의 출처를 확인한 뒤 검사·연결만 다시 실행할 수 있다.

최신 소스를 받은 PC에서는 기존 빌드를 먼저 수행한다(브릿지·주행을 시작하기 전).

```bash
cd /root/ws
./scripts/build_ws.sh -j2 -l2
cd src/lidar+sensorfusion
source /opt/ros/noetic/setup.bash
source /root/ws/devel/setup.bash
catkin_make --only-pkg-with-deps Main --force-cmake -j2 -l2
```

TwinLite의 공식 소스가 `/opt/baselines/TwinLiteNetPlus`에 마운트돼 있으면 사용한다. 소스가 없는 기존 컨테이너는 아래와 같이 Git 제외 폴더에 준비할 수 있다. [공식 저장소](https://github.com/chequanghuy/TwinLiteNetPlus)의 버전은 팀 로더가 지정한 커밋으로 고정하며 기존 모델 가중치를 사용한다.

```bash
cd /root/ws
git clone https://github.com/chequanghuy/TwinLiteNetPlus.git tools/scen_runner/.runtime/TwinLiteNetPlus
git -C tools/scen_runner/.runtime/TwinLiteNetPlus checkout --detach 90f1b8695ae311d5123b05f8534b2e11e42499d2
```

ROS 노드·MORAI 주행을 시작하지 않고 구성과 실행 파일만 검사하려면 다음 명령을 사용한다.

```bash
./tools/scen_runner/run_bridge.sh --check
./tools/scen_runner/run_perception.sh --check
./tools/scen_runner/run_planning.sh --check
```

각 터미널의 Ctrl+C는 그 실행기가 소유한 프로세스만 종료한다. 판제/브릿지 종료나 MORAI 연결 단절 시에도 시나리오의 종료 요청은 우선 처리한다. 실제 MORAI의 리스폰 후 자동 주행은 오프라인 시험과 별도로 검증한다.

## 실행

WSL에서 기존 컨테이너를 시작하고 들어간다.

```bash
docker start asmc-ros-noetic
docker exec -it asmc-ros-noetic bash
cd /root/ws
```

전체 구간을 한 씬으로 실행하거나 선택한 구간만 반복한다. 객체 배치·상태 수신이 완료되기 전에는 ego를 정지시킨다.

```bash
./tools/scen_runner/run_validation.sh --mode scene
./tools/scen_runner/run_validation.sh --mode section --section static
./tools/scen_runner/run_validation.sh --mode section --section dynamic
./tools/scen_runner/run_validation.sh --mode section --section roundabout
./tools/scen_runner/run_validation.sh --mode section --section high_speed
./tools/scen_runner/run_validation.sh --mode section --section tunnel
```

`--seed 42`로 첫 배치를 고정할 수 있다. `--collision-debug`는 접촉 원본·판정 이유·동적 활성화 진단을 추가 출력하며 생략해도 충돌 판정과 저장은 수행한다. `--speed-debug`는 실제 속도·가감속·RPC와 판정 기록을 `.logs/validation_speed_*.jsonl`에 저장한다. GUI에서도 모드/구간 선택, 다음 반복, ego 주변/구간/전체 경로 보기를 사용할 수 있다. GUI 갱신·주행 작업은 200ms, 별도 접촉 조회는 50ms 주기를 유지한다.

기존 브릿지와 판단·제어기는 각각 별도 컨테이너 터미널에서 실행한다. 브릿지와 `/ctrl_cmd` 발행자는 각각 한 개만 둔다. 이 개발 PC에서 확인한 MORAI UDP 포트는 Ego Status Destination 909, Competition Status Destination 911, Ctrl Cmd Host 9111이었다. 다른 PC에서는 MORAI 설정에 맞게 지정한다.

```bash
source devel/setup.bash
roslaunch asmc_bridge morai_udp_bridge.launch ego_port:=909 cmd_port:=9111 competition_port:=911
```

기본 `frenet_planner/keep_morai_no_obstacles.launch`처럼 브릿지를 포함하는 기존 판제 실행은 별도 브릿지와 중복되지 않게 설정한다. 시나리오 자체에는 경로 추종 제어기가 포함되지 않는다. 같은 공유 작업 폴더·같은 MORAI host/port의 통합 검증은 `.runtime/` 잠금으로 중복 실행을 막으며, 다른 객체 생성기도 함께 실행하지 않는다.

### MORAI를 먼저 종료한 경우

Ctrl+C/창 닫기는 재연결이나 새 배치보다 먼저 처리한다. 종료 요청 중 MORAI 연결이 없어지거나 RPC가 응답하지 않으면 원격 객체 정리를 재시도하지 않고 로컬 연결·실행 잠금·저장 기록을 마무리한 뒤 GUI와 코드를 종료한다. `[검증 종료] MORAI 통신 불가 · 원격 객체 정리 미확인`으로 표시하며 삭제 성공으로 기록하지 않는다. 시뮬레이터가 정상 응답할 때의 객체 정리·D 기어/입력 복귀는 기존 방식이다. 통신 문제와 다른 객체 삭제 거부 오류는 화면에 유지하고 다시 정리/종료할 수 있다.

Python 신호 처리기는 종료 플래그만 남기며 Qt 타이머가 Ctrl+C/SIGTERM 요청을 전달한다. 접촉 조회 채널도 닫아 대기 중 RPC를 취소한다. MORAI가 아닌 프로세스나 컨테이너 전체를 종료하지 않는다.

## 배치와 설정

| 구간 | 배치 | 주요 설정 |
|---|---|---|
| 정적 | 접근도로 1개·교차로 1~2개, 진로 방해와 회피 가능성 검사 | `one_scene.yaml`, `obstacle_catalog.yaml`, `static_obstacle.yaml` |
| 동적 | Man1/Woman1/Boy1/Girl1/Deer1/Dog1/Ped_Pet1, 가로·대각선 횡단 | `dynamic_crossing.yaml`, `dynamic_validation.yaml` |
| 회전교차로 | 모델 랜덤·4/5초 씬 고정 주기, 목적지 도착 삭제 | `roundabout.yaml` |
| 고주로 | 생성점 6/8/10, 차마다 목표 50~80km/h, 8/9초 공통 씬 고정 주기 | `high_speed.yaml`, `roundabout.yaml` |
| 터널 | 정적 1개·동적 1개·각각 1개 동시 중 씬마다 선택 | `tunnel.yaml` |

설정 파일은 모두 `config/`에 있다. 기존 모델·난수 선택·횡단 시작/끝·Activation Distance·생성 주기와 안전거리/TTC 조건을 유지한다. 동적 속도는 15/16/17/18km/h에서 선택하고 ego 25km/h 기준의 횡단 타이밍을 계산한다. 회전교차로 속도 상한은 15/17/19/20/21km/h이며 15 다음 차량은 21로 선택한다. 고주로는 MORAI 원생성점의 Custom 목표속도와 차량 물리 상한을 함께 설정한다. 목표와 실측 속도는 GUI 표에서 구분한다.

정적 교차로 후보 ENU Y는 -147~-135m이며 도로·실선과 회피 조건을 검사한다. 터널 정적 후보는 A126 전체 링크에 좌우 같은 계산 폭을 사용한다. 터널은 노란/흰 선 침범을 허용하면서 도로 이탈·객체 간섭은 검사한다. 터널 횡단은 가로 또는 ego 전방 대각선이며 정지 후 ego와 겹치는 횡단 후보는 배제한다.

링크 맵은 최신 main의 `src/asmc_planner/config/behavior/link_set.json`을 사용한다. 예전 충돌 기록의 `src/behavior_planner/src/config/` 경로는 공통 로더에서 새 위치로 변환하며 같은 지도 SHA256 검사를 적용한다. 지도 내용이 달라지면 재현을 계속 거부한다. 외부 절대경로로 지정한 지도는 별도로 유지한다.

맵·차선·링크·전역 경로 위치는 기존 YAML이 지정하는 저장소 파일을 사용한다. GUI는 도로 JSON, 배치 계산은 기존 도로/차선 JSON과 경로로 거리 격자를 만든다. 파일 또는 조건이 바뀌면 캐시를 갱신한다. 별도 H5로 실행 경로를 바꾸지 않는다. `validation.yaml`은 구간 시작/끝과 구간 종료선을 설정한다. 기본 구간 종료선은 끝점에서 진로에 직각이며 좌우 10m, 끝점 접근 범위는 25m다.

전체 씬은 확인된 ego 접촉 또는 **CP14 반경 3m 통과 후 START/END 반경 3m 복귀**로 끝난다. CP1~13의 순서 통과는 완주 조건이 아니다. 단일 구간은 확인된 접촉 또는 끝점 도착/종료선의 전방 통과로 끝난다. 큰 위치 점프나 후진은 통과로 세지 않는다. 충돌은 gRPC 접촉 ID의 실제 상대·위치/BoundingBox·새 진입을 확인해 판정하며, 거리만으로 충돌을 만들지 않는다. 준비 중부터 남은 접촉 ID와 오래된 응답은 새 접촉으로 승격하지 않는다.

## 주행 횟수·정체·환경

`run_counts.yaml`은 **최초 시작을 포함한 총 주행 횟수**다. 1이면 한 번, 10이면 최초 주행과 재시작 9번이다. 준비 실패는 횟수에 포함되지 않는다. 파일을 수정하고 러너를 재실행한다. 다른 파일은 `--run-counts <YAML>`로 지정한다.

```yaml
scene_runs: 10
section_runs:
  static: 10
  dynamic: 10
  roundabout: 10
  high_speed: 10
  tunnel: 10
```

남은 횟수가 있으면 객체 정리·시작점 복귀·새 시드/환경 배치 후 다시 주행한다. 마지막 횟수면 기록 복사 → 객체 정리 → 해당 시작점 복귀/정지 확인 → 저장 마무리 → GUI와 코드 종료로 끝낸다. 마지막에는 외부 MPC가 계속 실행돼도 ego가 움직이지 않도록 속도 0·P 기어·일시 정지를 유지한다. 사용자가 중간에 창 닫기/Ctrl+C로 종료하면 객체 정리 후 D 기어·일시 정지 해제로 기존 조종 입력을 복귀한다. 정리/복귀 실패 시에는 오류와 정지 상태를 유지한다.

`stagnation.yaml`은 주행 허용 후 **150초 동안 기준 위치에서 XY 거리 1m 이내**에 머무르면 해당 구간 설정을 저장하고 주행을 끝내도록 한다. 1m를 넘게 이동하면 새 위치에서 시간을 다시 센다. 준비 대기·기록 재현에는 적용하지 않는다. 관측 공백 5초 초과는 시간에 포함하지 않는다.

```yaml
enabled: true
duration_s: 150.0
position_radius_m: 1.0
max_sample_gap_s: 5.0
```

의도적인 정지·신호 대기도 조건을 만족하면 재시작한다. 정지 원인 판정 기능은 아니다. 필요하면 시간을 늘리거나 `enabled: false`로 끈다.

날씨/시간은 Sunny/Foggy × 11/13/15시의 6가지에서 고르며 직전 **조합 하나만** 다음 선택에서 제외한다. 전체 씬은 시작과 체크포인트 통과 시, 단일 구간은 매 주행 시작 시 변경한다. Foggy Density/Distance는 따로 지정하지 않는다. 객체 난수와 환경 난수는 분리하며 `checkpoint_environment.yaml`의 좌표 보정을 유지한다.

## 충돌·정체 저장과 재현

기본 저장은 항상 켜져 있다. ego/객체를 옮기거나 삭제하기 전에 구간 설정·seed·실제 상태·날씨/시뮬레이터 시간·UTC/KST 감지 시각·생성/삭제 이력·구간 진입 상태를 복사한다. 파일 쓰기는 별도 스레드가 처리한다. 현재 구간 밖의 정체는 다음 생성 구간 또는 가장 가까운 이전 구간을 선택하고 실제 경로 위치·선택 방식을 함께 남긴다.

기본 저장 위치는 **`tools/scen_runner/save_collision_log/<코드 실행 시각>/`**이다. 컨테이너에서는 `/root/ws/tools/scen_runner/save_collision_log/`이며, 저장소를 공유 마운트했다면 WSL에서도 같은 저장소의 `tools/scen_runner/save_collision_log/`에서 확인한다. 실행 시 `[충돌 저장 폴더]`, 저장 완료 시 `[충돌 저장 완료]` 또는 `[정체 저장 완료]`에 실제 경로가 출력된다. 실행 시각 폴더는 한국시간 `YYYYMMDD_HHMMSS` 형식이며 같은 초에 시작한 실행은 숫자 접미사를 붙여 구분한다.

상세 JSON에는 다음 자료를 저장한다.

| 구분 | 저장 내용 |
|---|---|
| 실행·충돌 | 실행 모드·회차·seed, 시작/감지 시각(UTC/KST), 주행 경과 시간, 상대 객체 ID·구간, 접촉 판정 근거 |
| 객체 설정 | 선택된 모델, 배치 위치·방향, 목표속도, 횡단 시작/끝·Activation Distance, 차량 생성 주기·경로 설정 |
| 실제 상태 | 수신된 ego·객체의 위치·방향·속도·크기·가속/제동/조향 값, 충돌 직전 최대 25개 상태 샘플 |
| 환경·재현 | 구간 진입 당시 상태, 이후 생성/삭제·날씨/시간 변경 일정, 구간 신호등 유지 설정과 교차로 단계 |
| 버전 | 코드·설정·지도 SHA256, 조회에 성공한 경우 Git 커밋·변경 여부 |

카메라 영상·LiDAR 원본·판제/MPC 내부 계산 상태는 저장하지 않는다. 신호등 항목은 시나리오가 적용한 유지 설정이며 전체 맵의 자동 신호 변화 기록은 아니다.

`events.jsonl`에는 회차 시작·생성 요청 ID/충돌 ID와 소유 범위·접촉 판정 변경·객체 정리 확인/실패를 기록한다. `--collision-debug`나 `--speed-debug` 없이도 저장하며, 시작할 때 `[검증 진단 파일]`로 경로를 표시한다. 이전 실행 등 이번 회차 관리 대상 밖의 객체와 접촉하면 GUI에 해당 ID와 제외 이유를 표시한다.

정적 객체는 초입에 1개, 교차로에 1~2개로 총 2~3개다. 정리할 때 정적 객체의 생성 별칭을 확인하고, 삭제 응답 후 상태 목록에서도 삭제를 확인한다. 삭제를 확인하지 못한 객체는 관리 목록에 남기고 다음 회차 생성을 중단한다. 종료 시 판제 실행기가 먼저 꺼졌어도 MORAI가 연결돼 있으면 객체 정리는 계속하며, 판제 종료를 확인하지 못한 경우 ego는 정지를 유지한다.

```text
tools/scen_runner/save_collision_log/
└── 20261012_120000/                    # 코드 실행 시각
    ├── run.json                       # 실행 시작·버전 정보
    ├── events.jsonl                   # 준비·접촉·정리 진단 기록
    ├── summary.csv                    # 충돌·정체 목록 요약
    ├── 001_static.json                # 첫 번째 충돌 상세·재현 자료
    └── 002_stagnation_high_speed.json  # 두 번째 기록: 고주로 정체
```

**판제와 연동해서 재현:** 브릿지·인지·`run_planning.sh`가 실행된 터미널 1~3은 유지하고, 터미널 4의 기존 시나리오를 종료한 뒤 컨테이너 `/root/ws`에서 실행한다.

```bash
./tools/scen_runner/run_scenario.sh --replay-browser
```

**키보드로 직접 주행하며 재현:** 자동 제어 입력을 보내는 주행 코드를 종료하고 아래 명령을 실행한다.

```bash
./tools/scen_runner/run_validation.sh --replay-browser
```

재현 브라우저는 **재현 가능한 기록이 있는 가장 최신 실행 폴더의 첫 기록**을 자동 선택한다. 기록이 없거나 `replay_available: false`인 사례만 있는 폴더는 건너뛴다. GUI의 **저장 실행** 목록에서 이전 실행을 고르고, **이전 기록 / 다음 기록**으로 같은 폴더 안의 기록을 감지 시각 순서대로 이동한다. **저장된 구간 다시 재현**은 현재 기록을 처음부터 다시 준비한다. 브라우저 모드는 재현 중 새 충돌 파일을 저장하지 않는다.

판제 연동 상태에서 다음 기록을 선택하면 **ego 정지 → 판제·MPC 종료 → 기존 객체 정리 → 다음 기록의 구간 진입 위치로 ego 재배치 → 저장된 객체·환경 복원 → 판제·MPC 재시작 및 입력 준비 확인 → 주행 허용** 순서로 처리한다. 전체 씬 최초 출발점이 아니라 해당 기록에 저장된 구간 진입 위치에서 시작한다.

JSON 하나를 직접 지정하는 방식도 지원한다. 여러 구간이 담긴 파일이면 `--replay-section`으로 선택한다.

```bash
./tools/scen_runner/run_scenario.sh --replay 저장파일.json --replay-section static
# 키보드/외부 제어기로 재현할 경우
./tools/scen_runner/run_validation.sh --replay 저장파일.json --replay-section static
```

재현은 저장된 모델·위치·횡단/차량 설정과 이후 생성/삭제·환경 변경 일정을 사용한다. ego는 저장된 구간 진입 위치에 **0km/h로 준비**하며, 이후 주행 입력은 연동한 판제 또는 사용자가 제공한다. 기록의 진입 속도나 외부 제어기 내부 상태를 자동 복원하지 않으므로 동일한 충돌 발생까지 보장하지는 않는다. 지도 지문이 다르면 재현을 거부한다. 구간 진입/필요 객체 상태를 확보하지 못한 사례는 설정만 저장하고 `replay_available: false`로 구분한다. 재현 GUI는 충돌/구간 끝에서 정지하고 다음 기록 선택을 기다리며 정체 자동 전환과 횟수 제한을 적용하지 않는다.

`--collision-log-dir <폴더>`로 저장/재현 루트를 바꾸고 `--no-save-collision`으로 충돌·정체 저장을 끈다. 컨테이너 root가 만든 새 기록은 공유 폴더 소유자 UID/GID를 따르므로 WSL에서 수정·삭제할 수 있다.

## MORAI DLL 수정본 빌드·적용·원복

다른 PC에는 수정 DLL을 복사하지 않고 자신의 설치본으로 빌드한다. gRPC SDK가 사용하는 함수 구조와 SHA256을 검사하며 다른 버전·불명확한 설치본은 덮어쓰지 않는다. 원본·설치 직전 DLL과 필요한 helper 백업은 **Simulator_Data/Managed에 보관**하며 삭제하지 않는다.

| 수정 | 이유 | 남긴 소스 |
|---|---|---|
| 자기 접촉 정리 방지·Ped_Pet1 행동 유지 | 내부 자기 collider 접촉으로 객체가 정리되던 문제와 PET 이동 모션 | `dll/src/PatchTrace.cs`, `dll/src/TraceHooks.cs` |
| 삭제/재사용 접촉 참조·카운터 정리 | NPC·보행자·정적 객체의 잔류/누락 접촉 기록 | 같은 최종 접촉 수정본 |
| 빈 경로 이름 삭제 방지 | 생성점 삭제 때 WaypointManager 전체가 삭제돼 오래 기다린 보행자가 출발하지 못하던 문제 | `dll/src/PathCleanupPatch.cs` |

현재 최종 접촉 수정본은 `ScenRunner.RuntimeTrace.dll` helper를 참조한다. **이 helper는 실행에 필요하므로 설치 폴더에서 지우지 않는다.** 기존 최종 수정본의 명령·로그 후크를 보존하려고 `dll/src/SpawnTrace.cs`도 빌드 입력에 유지한다. 별도 활성화 진단 DLL의 신규 빌드·설치는 제거했지만, 이미 설치한 사용자와 전체 원복을 위해 그 진단을 제거하는 호환 원복 코드는 남긴다. 물리·콜라이더·차량 제어를 끄는 수정은 아니다.

WSL 저장소 루트에서 사용한다. `config/local.yaml`의 `dll.managed_path`에 자신의 설치 경로를 입력하거나 각 명령에 **`--managed '<자신의 Simulator_Data/Managed WSL 경로>'`**를 지정한다. 명령 인수가 개인 설정보다 우선한다.

```bash
# 빌드는 사본만 준비한다. 설치 명령 전에는 MORAI를 완전히 종료한다.
./tools/scen_runner/run_runtime_trace.sh build-fix
./tools/scen_runner/run_runtime_trace.sh install-fix
./tools/scen_runner/run_runtime_trace.sh build-path-cleanup-fix
./tools/scen_runner/run_runtime_trace.sh install-path-cleanup-fix
# 적용 후 MORAI를 다시 실행한다.
```

`build-fix`/`install-fix`는 **최종 자기 접촉·PET·차량/보행자·정적 접촉 수정 전체**를 선택한다. 기존 이름 `build-static-contact-fix`/`install-static-contact-fix`도 같은 수정본을 선택한다. 빌드 결과는 `dll/builds/out_static_contact_fix/`와 `dll/builds/out_path_cleanup_fix/`에 있으며 Git에서 제외한다. 이미 경로 수정이 설치돼 있는데 접촉 수정본을 다시 설치하려면 먼저 경로 수정만 원복한 뒤 순서대로 적용한다. 다른 설치본·helper·백업 해시가 확인되지 않으면 거부한다.

Simulator 종료 후 **두 수정 모두 주최측 원본으로 원복**하려면:

```bash
./tools/scen_runner/run_runtime_trace.sh restore
```

활성화 진단이 설치돼 있으면 진단 → 경로 삭제 방지 → 기존 원본 순서로 해제한다. helper는 원본 복원 과정에서 제거한다. 필요한 백업/검증 정보가 없거나 파일이 바뀌면 임의로 복원하지 않는다.

일부만 제거하는 명령도 유지한다.

```bash
./tools/scen_runner/run_runtime_trace.sh restore-path-cleanup-fix  # 직전 접촉 수정본 유지
./tools/scen_runner/run_runtime_trace.sh restore-static-contact-fix
./tools/scen_runner/run_runtime_trace.sh restore-activation-trace  # 과거 진단 설치 사용자용
```

과거 중간 빌드 DLL은 정리했고 로컬에서 빌드했던 해시만 `dll/builds/out_legacy_hashes/`에 보존해 원복 검증에 사용한다. 현재 설치된 Simulator 파일과 Managed 원본/직전 백업은 소스 정리로 변경하지 않는다. 수정 소스·빌드/설치/원복 스크립트·검사는 팀 레포에 포함하고 설치 DLL·빌드 DLL·개인 로그는 포함하지 않는다.

## 황색 정책의 통제된 신호 시험

[정책 계약](../../docs/contracts/yellow-signal-policy.md)과
[명령–궤적 연결](../../docs/contracts/control-binding-v1.md)을 함께 빌드한다.
기본 운영 설정은 관측 모드다. 아래 실행기만 `active`와 추가 연결을 명시적으로
선택하고 자신의 판제·MPC·입력 adapter를 기동한다. 기존 판제 실행을 먼저 종료하고
Bridge·Main·카메라는 유지한다. 실행기는 다른 판제의 ROS 이름을 빼앗지 않는다.

신호는 `SetTrafficLightState`로 바꾸고 `GetTrafficLightInfo`로 실제 값을 확인한다.
종료 시 원래 교차로 phase에서 자동 주기를 재개한다. 원래 phase의 경과 시간은
복원하지 않는다. 강제 황색/적색은 phase 유효 기한 전에 끝내며 초록 갱신으로
덮어쓰지 않는다. 무응답·색상 불일치·시계 변동은 합격 처리하지 않는다.
적색 복구의 신호 유지 구간은 정책 EXITED와 독립 실제 차체의 이탈 경계 여유
0.5m를 모두 확인할 때까지다. 그 뒤 신호를 복원하고 출구 뒤 15m의 끝점까지
주행을 계속 검사한다. 이탈 전 녹색 flash를 만드는 phase 갱신은 하지 않는다.

시험 전 동일 Windows의 MORAI 원본 시계와 ROS 시계를 확인하고 로컬 clock profile을
만든다. 읽기 전용 `scen_runner.morai.clock_contract.calibrate_local_windows()`의
offset/불확실성은 해당 실행 환경의 측정값이다. 원본 JPEG와 SensorFrameTiming의
시계영역·세대·식별자 연결까지 확인한 뒤에만 `source_clock_verified`를 설정한다.
이 도구는 OS 시계를 수정하지 않는다. 다른 PC의 profile을 복사해 사용하지 않는다.

도로 평가 자료는 실제 실행 중인 지도 번들의 물리 road MeshCollider를
`export_signal_trial_surface.py`로 읽어 `.runtime/`에 생성한다. 별도의 Python 3.9
이상 검사 환경에 UnityPy·numpy·scipy·shapely가 필요하며 설치된 번들을 수정하지 않는다.
일반 시험에는 `requirements.txt`의 pyproj도 필요하다. 지도·차량 정답이나 gRPC의
신호 색상은 운영 정책 입력으로 발행하지 않는다.
실행기와 평가기는 MGeo 환경의 shapely도 사용한다. 제공 Docker 환경에서는
`docker/ros-noetic/requirements-mgeo.txt` 및 HD Map 의존성으로 설치한다.

컨테이너 `/root/ws`에서, 검증한 endpoint와 로컬 자료를 지정한다.

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export PYTHONPATH="tools/scen_runner:$PYTHONPATH"
python3 tools/scen_runner/run_yellow_signal_trial.py \
  --host 127.0.0.1 --port 7789 \
  --clock-profile tools/scen_runner/.runtime/yellow/current-clock.yaml \
  --physical-road tools/scen_runner/.runtime/yellow/physical-road-oracle.json \
  --light C1256W000077 --case yellow_stop --start-distance 40 \
  --output tools/scen_runner/.runtime/yellow/trials/new-yellow-stop
```

`--record-bag`는 원본 카메라·센서·frame timing과 실제 명령·궤적·제어를 함께 기록한다.
분할 bag은 정상 종료 후 닫힘·필수 토픽·SHA를 확인한다. 원본 bag은 수정하지 않는다.
`analyze_signal_bag.py <회차 폴더>`는 Controller가 실제 사용한 token·Planner 세대·
sequence를 해당 요청과 궤적에 연결해 검사한다. 생성 시각으로 명령 소속을 추정하지 않는다.
이 bag 검사는 독립 물리 주행 판정을 대신하지 않는다.

`--full-route`는 800m에서 종료하지 않고 CP14를 통과한 뒤 START/END로 복귀해야
완주로 인정한다. 이때 `export_signal_trial_surface.py --full-route`로 만든 전체
경로 자료와 원본 경로 SHA가 필요하다. 기존 800m 자료를 전체 경로 합격에 사용하지 않는다.
`--full-route --case corridor --natural-signals`는 신호를 고정하지 않고 기존 자동
주기를 사용한다. 전체 경로 기능은 시계 수락과 실제 주행 검증을 생략하지 않으며,
옵션이 준비됐다는 이유로 기본 운영 모드를 활성화하지 않는다.

지원 사례는 `green`, `red_stop`, `yellow_stop`, `yellow_close`, `yellow_inside`,
`recovery`, `corridor`다. `corridor`는 공식 출발점부터 s=800m까지이며 회전교차로
시작 약 s=858.5m 전에 끝난다. 나머지는 선택한 신호 접근/출구 구간이다.
`yellow_close --trigger-distance 6` 같은 불가능 조건은 무위반 주행 합격과 구분한다.
`analyze_yellow_signal_trial.py <회차 폴더들> --output <새 분석.json>`은 원본 result를
바꾸지 않고 최대제동 선행·초과 기록 보존·실제 도로 이탈을 따로 평가한다.

`--park-static ID:DX:DY`는 명시한 기존 정적 prop만 임시 이동한다. 원래 상태를 쓰기
전에 기록하고 물리 위치·방향 readback과 원위치 복원을 확인한다. 다른 객체를
삭제하거나 재생성하지 않는다. 복원 실패는 회차 실패로 남기고 snapshot을 보존한다.
이 옵션을 쓰지 않은 장애물 유지 사례와 비어 있는 신호 기본 사례를 구분한다.

`qualify_yellow_blockage.py`는 같은 profile·도로·endpoint와 명시한 `--static-id`로
실제 정적 prop을 사용하는 추가 연결 시험이다. `--case red_recovery_blocked`는
C091 내부에 차량을 정지시킨 상태에서 전체 이탈 경로를 막고 제동 유지를 확인한다.
이는 held preflight 평가이며 이동 중 돌발 장애물 제동 시험으로 집계하지 않는다.
장애물을 검증된 임시 주차 위치로 옮긴 뒤 현재 명령의 새 안전 경로로 실제
주행을 완료해야 합격한다. 원본 prop을 복원하기 전 자차는 공식 출발점에 정지한다.
예를 들어 위 명령과 같은 공통 인자에 `--case red_recovery_blocked --static-id 2`
및 새 `--output`을 지정한다. 이 ID는 이 설치본에서 확인한 정적 prop이므로 다른
scene에 그대로 적용하지 않는다. 다른 차량·보행자 유형이면 이동을 거부한다.

회차마다 소스·실행 바이너리·지도·clock profile 지문, 원본 인식, 정책 사건·원인,
원자적 요청/경로, 실제 제어 출력, MORAI 실제 차량·신호 평가를 저장한다.
소스가 도중에 바뀌면 해당 회차는 합격으로 집계하지 않는다. 정상/오류 종료 모두
ego를 pause·속도 0으로 유지하고 자신의 launch를 종료한 뒤 신호·시험 fixture와
이전 binding 설정을 복원한다. SIGKILL·통신 단절의 원격 복원은 보장하지 않는다.
이 경우 남은 snapshot과 실제 simulator 상태를 확인한 후 재개한다.

이 설치본에서 수행한 회차·실측·실패 기록은
[2026-10-22 검증 기록](../../docs/records/planning/2026-10-22-yellow-light-validation.md)에 있다.

## 검사와 공유 범위

남긴 테스트는 통합 실행의 배치 계산, 생성/정리·ID 재사용, 접촉, 환경, 완주/구간 종료, 주행 횟수·정체, 저장/재현, GUI·진입 명령을 검사한다.

```bash
QT_QPA_PLATFORM=offscreen python3 -m unittest discover -s tools/scen_runner/test
```

DLL 검사는 WSL에서 수행하며 설치/원복 테스트는 임시 폴더를 사용한다.

```bash
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$(wslpath -w tools/scen_runner/dll/tests/test_guard.ps1)"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$(wslpath -w tools/scen_runner/dll/tests/test_restore.ps1)"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$(wslpath -w tools/scen_runner/dll/tests/test_path_cleanup_install.ps1)"
# 경로 수정 IL/동작 검사는 자신의 Managed 경로를 지정한다.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$(wslpath -w tools/scen_runner/dll/tests/test_path_cleanup_fix.ps1)" -ManagedPath '<Windows Managed 경로>'
```

실제 MORAI 주행 성공·충돌 재현은 오프라인 검사와 구분해 확인한다. 의존성은 `requirements.txt`와 기존 저장소 gRPC SDK이며, 컨테이너가 이미 준비됐다면 별도 진입 스크립트를 설치할 필요는 없다.

통합·단일 구간 실행에 불필요한 개별 생성 GUI, 경로 추종/PID 시험기, 벤치마크·범위 PNG 생성기, 과거 진단 실행기는 제거했다. 개인 `.logs/`, `.runtime/`, `save_collision_log/`, 이전 개별 실행의 저장 배치 `layouts/`, 빌드 `dll/builds/`, 소스 정리 전 로컬 백업 `.archive/`는 Git에서 제외한다.
