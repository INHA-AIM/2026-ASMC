# CI 실행과 유지

전체 workspace 검사와 별도 판제 전용 빌드를 유지한다. 후자는 큰 workspace가 잘못된 의존성을
가려주는 일을 검출하기 위한 독립 구성이다. 빌드 결과·devel·시험 성공을 cache로 건너뛰지 않는다.

## 구성

- `CI / catkin-build`: 환경·문서·공용 계약 검사, 전체 클린 빌드와 명시적인 시험 executable 준비 후
  시간 민감 HTTP 검사를 포함한 LiDAR 그룹을 단독 실행한 뒤 나머지
  [회귀 그룹](../../.github/ci/groups.json)을 최대2개씩 실행한다. 같은 build tree에서 병렬 컴파일하지 않는다.
- `Planning integration build / planning-catkin-build`: 기존 whitelist만 새 환경에 빌드하고 입력 변환,
  dashboard·launch·AVOID·로터리·도로 제약·충돌 회귀를 유지한다.
- `validated-lidar-build`: 구 실행 의존성과 입력 소유권의 정적 검사다. 실제 LiDAR 회귀는 전체 CI에서 필수 실행한다.

같은 job의 LiDAR/음영 어댑터/카메라/bridge 시험은 각각 다른 ROS master와 시험 소유 프로세스를
사용한다. ROS 시험 내부 timeout·freshness·수치 기준은 줄이지 않는다. 같은 workspace에서 반복하던
YOLO baseline/all·segmentation 세 fixture만 한 번씩 실행하고 `--batched` 등 다른 옵션 검사는 유지한다.
[보존 검사](../../tools/ci/check_coverage.py)가 이전141개 고유 명령·옵션 조합과 필수 그룹을 대조한다.

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
이전 pip의 dependency conflict 경고를 성공으로 넘기지 않는다. 운영 컨테이너의 pip/모델 환경은 변경하지 않는다.

소요 시간은 runner·cache 영향을 받으므로 wall time과 병렬 job 시간 합계를 함께 비교한다.
[초기 측정·debug OFF 확인](../records/repository/2026-10-26-ci-runtime.md)을 기준으로 실제 Actions 결과를 확인한다.

## 로컬 재현

Noetic에서 전체 workspace와 필요한 시험 executable을 먼저 빌드한다. 기존 운영 노드는 유지하고
서로 다른 master의 합성 시험만 실행한다. 아래 명령은 사용자용 실행 wrapper가 아니라 CI 재현 도구다.

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin_make planning_lidar_observation_test planning_group_conversion_test ground_projection_test -j2 -l2
python3 tools/ci/check_coverage.py
python3 -m unittest discover -s tools/ci -p 'test_*.py'
python3 tools/ci/run_checks.py --workers 2 --output /tmp/asmc-ci-local-unique-run
```

출력 디렉토리는 새 이름을 사용한다. 별도 빌드의 재현은 `ASMC_CI_BUILD`와 `ASMC_CI_DEVEL`에 해당
경로를 주입한다. `--workers 1`은 동일 검사를 순차 실행한다. 합성 capture 단위시험의 unmanaged 출력은
그 시험의 `/tmp`에만 허용하며 수집·학습·운영 도구의 저장 규칙을 우회하지 않는다.

GitHub 참고: [cache key·복원 순서](https://docs.github.com/en/actions/reference/workflows-and-actions/dependency-caching),
[artifact 보존·숨김 경로](https://github.com/actions/upload-artifact#uploading-hidden-files).
