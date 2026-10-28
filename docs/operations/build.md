# 운영·전체 시험 빌드

저장소 루트에서 ROS Noetic 환경의 `scripts/build_ws.sh`를 사용한다.
운영과 전체 시험은 같은 build/devel 및 컴파일 설정을 재사용하며, 모드 전환 때 clean하지 않는다.

```bash
# 기본값: 대회 운영에 필요한 노드·라이브러리·메시지·native 카메라 모듈
./scripts/build_ws.sh runtime -j2 -l2
# 인자 없이 실행해도 runtime
./scripts/build_ws.sh

# 전체: 모든 구성 패키지의 운영·개발 도구·C++ 시험을 빌드
./scripts/build_ws.sh full -j2 -l2

# 필요한 target만 묶어서 빌드
./scripts/build_ws.sh asmc_planner_node mpc_controller_node -j2 -l2
```

모든 모드는 **빌드만** 한다. `full`은 `EXCLUDE_FROM_ALL`인 C++ 시험도 포함하며 시험을 실행하지 않는다.
Python 시험은 C++ 컴파일이 필요하지 않다. 기존 CTest/rostest/nosetest 등록과 launch는 유지한다.
전체 회귀 실행은 [CI 안내](ci.md)의 수동 workflow 또는 해당 group script를 사용한다.
원래 catkin 명령이 필요하면 `./scripts/build_ws.sh --catkin ...`으로 전달한다.
`clean`, `install`, `run_tests`도 기존 catkin 명령으로 전달되며 실행 의미는 각 명령을 따른다.
`all`은 wrapper에서 `full`의 호환 별칭이다. 직접 `catkin_make`를 쓰면 원래 기본 `all` 동작이 유지된다.

## 중복을 줄이는 방식

- [프로필 정의](../../cmake/AsmcBuildProfiles.cmake)는 선택한 target을 `asmc_requested` 하나에 연결한다.
  여러 make goal을 차례로 처리하며 공통 메시지 검사를 반복하던 비용을 줄인다.
- `runtime`은 기본 camera/LiDAR/bridge/통합 planner·controller와 Python 메시지·native 모듈을 포함한다.
  package whitelist로 필수 항목을 제외하면 구성 단계에서 실패한다. Python association으로 조용히
  바뀌거나 PCL 파이프라인이 생략된 운영 빌드를 성공으로 처리하지 않는다.
- `full`은 구성된 패키지에서 C++ executable/library 및 메시지 target을 자동 수집한다.
  새 시험 target이 추가돼도 별도 고정 목록 누락으로 빌드에서 빠지지 않는다.
  `CATKIN_ENABLE_TESTING=OFF`와 함께 full을 요청하면 실패한다.
- `planning_inputs`의 같은 객체 변환 소스는 `planning_objects_core`에서 한 번 컴파일하고,
  어댑터·bag 도구·시험이 링크한다. 변환 로직과 message contract는 변경하지 않는다.
- 같은 build 디렉토리에서 wrapper를 중복 실행하면 lock 오류로 중단한다.
  다른 worktree는 각각 자신의 build/devel을 사용한다. 직접 catkin/CMake 호출은 이 wrapper lock에 참여하지 않는다.

## 컴파일 설정과 cache

wrapper 기본 build type은 Release다. runtime/full 간에 같은 설정을 유지한다.
`-DCMAKE_BUILD_TYPE=Debug` 등 명시적 인자는 우선하며, 설정을 바꾸면 영향을 받는 파일은 재컴파일된다.
기존 CI는 명시적인 빈 build type으로 이전 컴파일 flags/cache 호환성을 유지한다.

`ccache`가 PATH에 있으면 compiler launcher로 자동 사용한다. 기본 ROS Dockerfile에 포함하며,
기존 실행 중 컨테이너에는 자동 설치하지 않는다. wrapper가 apt·pip·모델 다운로드를 실행하지 않는다.
기본 cache는 worktree의 `.ci-cache/ccache`에 최대 400 MB로 두며 기존 CCACHE 환경변수는 우선한다.
ccache가 없는 이전 이미지로 빌드를 옮겼다면 `ASMC_CCACHE=off`로 cached launcher를 해제한다.

```bash
ASMC_CCACHE=on ./scripts/build_ws.sh runtime  # 없으면 명확히 실패
ASMC_CCACHE=off ./scripts/build_ws.sh full   # launcher 자동 사용 해제
ccache --show-stats
```

명시적인 `CMAKE_C_COMPILER_LAUNCHER`/`CMAKE_CXX_COMPILER_LAUNCHER` 설정은 자동 선택보다 우선한다.
cache는 컴파일 결과만 재사용하며 실제 헤더·compiler 검증, 링크, 시험 실행을 생략하지 않는다.
동일 worktree의 결과가 최신이면 다시 컴파일하지 않는다. cache의 주된 이득은 clean 이후나
브랜치 전환·이전에 빌드한 소스 복원에서 생긴다. build/devel·시험 성공 자체를 cache로 재사용하지 않는다.

카메라 native kernel은 시스템 pybind11 또는 설치된 Torch의 bundled header를 찾는다.
Torch 탐색은 module 위치만 읽고 Torch/CUDA 모델을 import하지 않는다. 명시적으로 지정하려면
`-DASMC_PYBIND11_INCLUDE_DIR=...`를 사용한다.

## 결과 확인

선택한 목록은 `build/asmc-build-targets.txt`에 남는다.
운영 빌드만으로 전체 시험 executable이 준비됐다고 판단하지 않는다.
전체 시험이 필요하면 먼저 `full`을 실행한 뒤 해당 시험을 실행한다.
최초 빌드·변경 없는 재빌드·단일 파일 수정·runtime→full 전환은 소요 시간을 구분해 비교한다.
