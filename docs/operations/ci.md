# CI 실행과 유지

매 PR과 main push에서는 운영 실행파일 빌드와 핵심 계약·회귀만 자동 실행한다.
전체 회귀는 `Full regression (manual)`에서 필요할 때 실행한다. 자동 검증의 목표는
runner 실행 시작부터 종료까지 **7분 이내**다. 빌드 job은 각각 6분, 계약 job은 3분으로
제한하고 병렬 실행해 여유를 둔다. 시간 제한으로 실패한 실행은 목표 달성으로 세지 않는다.
GitHub runner 대기열 지연은 workflow 밖의 대기 시간으로 구분한다.

## 매번 실행하는 검사

자동 workflow는 [ci.yml](../../.github/workflows/ci.yml) 하나이며 세 job을 의존 관계 없이 실행한다.
기존 check 이름을 유지하지만 세 job 모두 실제 검사를 수행한다.

| check | 필수 검사 | 제한 |
|---|---|---|
| `validated-lidar-build` | LiDAR 운영 정본·입력/토픽/저장·bridge·문서 계약, 엔진 선택 정책, dashboard, Python 문법 | 3분 |
| `planning-catkin-build` | 판제·MPC·입력 어댑터 운영 target 빌드, 입력 신선도/유효성, 제동·reference·GPS 모드, 고속도로 초기화 preflight, ROS 입력 연결 | 6분 |
| `catkin-build` | LiDAR/PCL·vehicle state·카메라 C++·native lane association 빌드, 배포 모델 무결성, frame/time·형상·투영, native parity, 카메라 launch/model/로컬 engine 준비 계약 | 6분 |

빌드할 package/target과 실행 script는 [essential.json](../../.github/ci/essential.json)에 명시한다.
모든 시험 실행파일을 컴파일하던 `all` 대신 운영 target과 선택한 작은 회귀 target을 빌드한다.
Python/카메라 native 모듈의 실제 import를 확인하며 모듈 누락을 skip으로 처리하지 않는다.
LiDAR 운영 target은 `ASMC_LIDAR_PIPELINE_REQUIRED=ON`으로 PCL 의존성 누락을 오류로 처리한다.
카메라 GPU 모델 추론·TensorRT 속도나 MORAI 전 구간 주행을 이 CPU CI의 검증 결과로 해석하지 않는다.

## 필요할 때 실행하는 전체 검사

- [Full regression (manual)](../../.github/workflows/full-regression.yml): Actions에서 대상 ref를 선택해
  `Run workflow`로 실행한다. 전체 workspace와 기존 시험 target, CPU Torch/Ultralytics 환경,
  [6개 회귀 그룹](../../.github/ci/groups.json)을 보존한다. LiDAR 그룹을 단독 실행한 뒤
  lane·camera-unit·camera-ros·bridge·planning 그룹을 최대 2개씩 실행한다.
- [Planning integration build](../../.github/workflows/planning-integration.yml): package.xml·CMake·외부
  의존성 변경 시 최소 6패키지 whitelist를 독립 빌드한다. 자동 CI와 중복 실행하지 않는다.

긴 차선 합성 시험·카메라 ROS fixture·전체 bridge·AVOID/로터리/고속도로 회귀가 필요하면 수동 전체
검사를 실행한다. 시험 내부 timeout·freshness·수치 허용 오차는 바꾸지 않았다.
[검사 보존 audit](../../tools/ci/check_coverage.py)은 기존 145개 고유 명령·옵션 조합이 수동 전체
검사에도 남아 있는지 확인한다. 자동 workflow에 전체 회귀나 Torch 설치가 다시 들어오거나,
핵심 운영 target·병렬 job·시간 제한이 사라지면 audit을 실패시킨다.
자동 CI 통과는 수동 전체 회귀까지 통과했다는 뜻이 아니다.

## 시간·cache·실패 기록

인지 build에는 공식 Noetic perception 이미지의 고정 digest를 사용해 PCL/OpenCV 의존성 준비 비용을
줄인다. PR 검사에서는 Torch/torchvision/Ultralytics를 설치하지 않고 필요한 NumPy·SciPy·PyYAML만
[requirements](../../.github/ci/essential-requirements.txt)로 고정한다. 운영 컨테이너의 환경은 바꾸지 않는다.
계약 job은 큰 배포 PT/engine/ONNX/과거 tar.gz를 sparse checkout에서 제외한다.
Noetic의 Git 2.25는 checkout action의 sparse 지원 조건을 충족하지 않아 판제 job은 일반 checkout을 사용한다.
인지 job은 전체 파일을 받아 E3/E8/Large와 신호등 모델 존재·해시를 확인한다.

ccache는 C++/header/CMake/package/message/service 내용으로 key를 만들고 역할별로 저장한다.
호환되는 이전 compiler cache도 복원하되 compiler/header 검사·링크·시험은 매번 실행한다.
통계는 이번 실행 전에 초기화한다. 빌드 성공 직후 cache를 저장하고 build/devel·시험 성공은 저장하지 않는다.

실패·취소를 성공으로 넘기지 않는다. 더 최신 PR revision은 이전 CI를 취소하고 main 검증은 완료한다.
자동 build/test 로그는 `essential-planning-*`, `essential-perception-*` artifact에 7일간 보존한다.
수동 전체 회귀의 `regression-*` artifact에는 그룹별 로그·script SHA·종료 코드·시간·summary가 남는다.
시간 제한 초과·cache miss·이미지 준비 지연은 Actions의 각 step 시간으로 구분해서 조사한다.

이전 측정: [초기 CI 분석](../records/repository/2026-10-26-ci-runtime.md),
[PR #133 판제 초기화·중복 빌드 개선](../records/repository/2026-10-29-planner-startup-ci.md).
PR #133의 전체 CI는 16분 42초였고 그중 회귀가 8분 50초였다. 자동 검사를 핵심 범위로 줄인 것은
후속 변경이며, 수동 전체 회귀의 소요 시간을 7분으로 줄였다는 의미는 아니다.

## 로컬 재현

별도 Noetic workspace에서 역할별로 실행한다. 아래 두 빌드는 서로 다른 whitelist를 사용하므로
같은 build/devel에서 동시에 실행하지 않는다. 기존 운영 노드·ROS master는 유지한다.

```bash
python3 tools/ci/build_essential.py planning
bash .github/ci/essential-planning.sh

python3 tools/ci/build_essential.py perception
bash .github/ci/essential-perception.sh

bash .github/ci/essential-contracts.sh
```

계약 검사는 PyYAML·Node.js가 필요하다. ROS 검사는 role별 빌드가 만든 devel을 사용한다.
전체 회귀 재현은 수동 workflow의 build target과 의존성을 준비한 뒤
`python3 tools/ci/run_checks.py --workers 2 --output /tmp/asmc-ci-local-unique-run`을 실행한다.
출력 경로는 새 이름을 사용한다. 합성 단위시험의 unmanaged 출력은 해당 `/tmp`에만 허용하며
수집·학습·운영 도구의 저장 규칙을 우회하지 않는다.
