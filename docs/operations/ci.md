# CI 실행과 유지

매 PR과 main push에서는 전체 workspace를 한 번 빌드하고 판제 회귀도 그 결과를 사용한다.
최소 패키지 구성의 의존성 점검은 `Planning integration build`의 수동 실행(`workflow_dispatch`)에
남겼다. 이 독립 빌드는 더 이상 매 PR의 자동 gate가 아니며, package.xml·CMake·외부 의존성 변경 때
추가 실행한다. 빌드 결과·devel·시험 성공을 cache로 건너뛰지 않는다.

## 구성

- `CI / catkin-build`: 환경·문서·공용 계약 검사, `all`과 명시적인 시험 target을 한 catkin 호출에서 빌드한 후
  시간 민감 HTTP 검사를 포함한 LiDAR 그룹을 단독 실행한 뒤 나머지
  [회귀 그룹](../../.github/ci/groups.json)을 최대2개씩 실행한다. 같은 build tree에서 병렬 컴파일하지 않는다.
- `planning` 그룹: dashboard·launch·AVOID·로터리·도로 제약·충돌·고속도로 runtime 검사를
  공통 빌드에서 수행한다. 입력 executable 4개와 `lidar_observation.test`는 `lidar` 그룹에서 한 번 실행한다.
- `CI / planning-catkin-build`: 이전 check 이름을 유지하는 결과 gate다. 공통 build/test의 실패·취소도
  실패로 전달한다. workflow 소속이 바뀌므로 workflow까지 제한하는 외부 보호 규칙이 있다면 확인한다.
- `Planning integration build / planning-standalone-build`: 수동 실행에서만 기존 6패키지 whitelist를
  최소 의존성 환경에서 클린 빌드한다. PR에서는 실행하지 않는다.
- `validated-lidar-build`: 구 실행 의존성과 입력 소유권의 정적 검사다. 실제 LiDAR 회귀는 전체 CI에서 필수 실행한다.

같은 job의 LiDAR/음영 어댑터/카메라/bridge/판제 시험은 각각 다른 ROS master와 시험 소유 프로세스를
사용한다. ROS 시험 내부 timeout·freshness·수치 기준은 줄이지 않는다. 같은 workspace에서 반복하던
YOLO baseline/all·segmentation 세 fixture만 한 번씩 실행하고 `--batched` 등 다른 옵션 검사는 유지한다.
[보존 검사](../../tools/ci/check_coverage.py)가 이전141개와 PR #114 신규4개(총145개) 고유 명령·옵션 조합과 필수 그룹을 대조한다. 판제 고유 검사, 중복 build·입력 검사, 결과 gate도 확인한다.
LiDAR 단독 실행 후 lane을 먼저 시작하고 나머지 그룹을 2 worker에 배치한다. 그룹 timeout과
시험 내부 허용 시간·허용 오차는 이전과 같다. 차선 fixture 자체의 병렬 분할은 포함하지 않는다.

## 실패·취소와 로그

한 그룹이 실패하거나 timeout돼도 나머지 그룹을 실행하고 전체는 실패한다. 빈/누락 그룹·기존 출력
덮어쓰기는 오류다. 취소 신호는 시험 소유 process group에 전달한다. 더 최신 PR revision은 이전 실행을
취소하지만 main 검증은 완료한다. 오류를 무시하는 `continue-on-error`는 사용하지 않는다.

Actions의 `regression-<run-id>-<attempt>` artifact에는 각 그룹 전체 로그·script SHA·종료 코드·시간과
`summary.json`을7일간 보존한다. 성공 결과를 다음 빌드에 재사용하지 않는다. 업로드 범위는 생성한
`.ci-results`뿐이며 cache·설정·자격 증명은 포함하지 않는다.

## Cache와 비용

ccache key는 commit SHA 대신 C++/header/CMake/package/message/service 내용으로 결정한다.
동일 job의 cache를 우선하고 다른 job의 호환 cache를 보조로 복원한다. 실제 compiler/header 검사와
링크·시험은 매번 수행한다. 통계는 복원 후0으로 초기화해 누적 hit 수치를 이번 실행으로 오해하지 않는다.
CPU wheel cache는 [실제 설치 requirements](../../.github/ci/cpu-requirements.txt)에 연결한다. 의존성
설치·검증은 생략하지 않으며 일반 workflow 편집만으로 같은 다운로드 cache를 새로 만들지 않는다.
Noetic Python3.8을 지원하는 pip24.3.1 resolver를 CI에서 먼저 준비하고 설치 후 `pip check`를 실행한다.
이전 pip의 dependency conflict 경고를 성공으로 넘기지 않는다. 운영 컨테이너의 pip/모델 환경은 변경하지 않는다. NumPy1.24.4·SciPy1.10.1을 함께 고정하고
설치 후 공간 검색/회전·OpenCV·Torch의 실제 import도 확인한다. C++ cache는 빌드 성공 직후 저장해
나중의 시험 실패로 같은 소스를 다시 컴파일하는 낭비를 줄인다. 시험 성공이나 build/devel은 저장하지 않는다.

소요 시간은 runner·cache 영향을 받으므로 wall time과 병렬 job 시간 합계를 함께 비교한다.
[초기 측정](../records/repository/2026-10-26-ci-runtime.md)과
[중복 빌드 제거·판제 초기화 분석](../records/repository/2026-10-29-planner-startup-ci.md)을 참고한다.
별도 job 제거는 runner 사용량을 줄이지만 기존 lane 그룹의 350초를 제거하지는 않는다.
실제 PR 대기시간 개선은 수정 후 Actions run으로 확인해야 한다.

## 로컬 재현

Noetic에서 전체 workspace와 필요한 시험 executable을 먼저 빌드한다. 기존 운영 노드는 유지하고
서로 다른 master의 합성 시험만 실행한다. 아래 명령은 사용자용 실행 wrapper가 아니라 CI 재현 도구다.

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin_make planning_lidar_observation_test planning_group_conversion_test planning_lidar_companion_test planning_gps_recovery_test ground_projection_test detected_msg_converter_test feature_builder_avoid_test dynamic_object_policy_test highway_runtime_test -j2 -l2
python3 tools/ci/check_coverage.py
python3 -m unittest discover -s tools/ci -p 'test_*.py'
python3 tools/ci/run_checks.py --workers 2 --output /tmp/asmc-ci-local-unique-run
```

출력 디렉토리는 새 이름을 사용한다. 별도 빌드의 재현은 `ASMC_CI_BUILD`와 `ASMC_CI_DEVEL`에 해당
경로를 주입한다. `--workers 1`은 동일 검사를 순차 실행한다. 합성 capture 단위시험의 unmanaged 출력은
그 시험의 `/tmp`에만 허용하며 수집·학습·운영 도구의 저장 규칙을 우회하지 않는다.

GitHub 참고: [cache key·복원 순서](https://docs.github.com/en/actions/reference/workflows-and-actions/dependency-caching),
[artifact 보존·숨김 경로](https://github.com/actions/upload-artifact#uploading-hidden-files).
