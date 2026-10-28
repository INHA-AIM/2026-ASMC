# 판제 초기화와 CI 중복 분석 — 2026-10-29

기준 main은 `be20138a`(PR #130)다. `perf/planner-startup-20261029`의 별도 작업트리와
네트워크 없는 2 CPU / 6 GiB 컨테이너에서 검사했다. 병행 LOCAL·AIMDrive·E3 작업트리,
운영 컨테이너, 센서/제어 토픽 및 MORAI는 변경하지 않았다.

## 원인과 변경

사용자 로그의 동일 `Loaded 4430 waypoints` 7회는 7개 planner 노드가 아니다.
`ProductionRuntime` 내부의 mission 1개와 고속도로 target 2개·차선 3개·공통 코어 1개가 각각
`FrenetPlanner::Init`을 호출했다. 모든 코어가 같은 HD-map JSON·raster·전역경로·로터리 영역을
준비한 뒤 dormant 코어의 reference를 다시 교체했다. 생성자가 완료돼야 callback spinner가 시작된다.

`InitFromPrepared`는 최초 검증에서 얻은 **정적 입력 snapshot**만 새 코어에 복사한다.
객체 관측·projection history·속도 profiler·maneuver/transaction state는 복사하지 않는다.
설정과 지도/raster/waypoint의 실제 바이트 및 map 해석 경로가 다르면 기존 로딩/검증을 수행한다.
원본 경로 교체나 로터리 runtime reset은 snapshot에 반영되지 않는다. 일반 `Init`는 항상 새로 읽는다.
config 검증, 도로 footprint, 곡률, 합류 commit 조건과 stale/invalid 정지 조건을 생략하지 않는다.

`autonomy_no_bridges_v2.launch`만 `highway_merge` 기본값을 true로 바꿨다.
개별 `planner_v2.launch`는 false이고 통합 실행의 명시적 false override도 유지한다.
따라서 아래 성능 비교는 **양쪽 모두 고속도로 기능 ON**이다. 이전 기본 OFF와 비교한 값이 아니다.

첨부 로그에는 dashboard 8088 포트 중복 종료도 있다. 정식 dashboard launch에 HTTP 포트를
명시적으로 전달하고 ROS 노드 이름 등록 전에 포트를 확보한다. 기존 포트가 사용 중이면 새 실행만
실패한다. 기존 ROS 노드를 먼저 종료시킨 후 HTTP bind에서 실패하는 순서를 방지한다.
직접 `rosrun ... _port:=...`처럼 `--port`를 전달하지 않는 옛 호출은 이 선점 보호 대상이 아니다.

로그의 GPS/IMU unavailable·INVALID_TIMESTAMP·MPC 입력 대기와 XmlRpc acceptConnection EAGAIN은
계산 초기화와 구분해야 한다. 관측이 유효해질 때까지의 정지 대기는 유지하며, EAGAIN의 발생 원인은
격리 통합 시작에서도 20회 재현됐지만 원인은 확정하지 않았다. 로그 총량은 24 KiB였고
10개 노드가 정상 등록되어 mode 상태가 발행됐다. XML-RPC 라이브러리나 오류 출력을 임의로 숨기지 않았다. 지도 준비 시간 감소를 센서 준비/실제 출발 전체 시간으로 표현하지 않는다.

## 오프라인 측정

저장소 밖 D 정본 run:

- `planner-startup-baseline-20261029-r1`: 변경 전 3회, 14.052 / 13.817 / 14.021초.
- `planner-startup-candidate-20261029-r1`: 같은 컨테이너에서 보존한 기존 바이너리와 수정본을 교대로 실행.
  최초 짝 비교 기존 11.700 / 11.750 / 11.550초, 수정 2.623 / 2.673 / 2.674초.
  중앙값 11.700 → 2.673초(77.2% 감소). 다른 작업의 부하 영향 때문에 별도 시점 14초보다 짝 비교를 기준으로 삼는다.
- 전역경로/구간 JSON 6개 SHA256은 모두
  `bb80f5f669ccb77a8e6d8944e51473e11142db93fb80e72205a998c0b4e75844`로 일치했다.
  reference 로딩 로그는 7 → 1회다. 이 export는 전체 closed-loop trajectory 동등성 검사가 아니다.

실행 프로그램은 `highway_merge_preflight <params.yaml> <새 report.json>`이다.
기존 바이너리의 shared library도 보존하고 `LD_LIBRARY_PATH`로 분리했다.
최종 소스 재빌드 후 짝 비교는 기존 11.201 / 11.253 / 11.254초, 수정 2.523 / 2.574 / 2.574초다.
중앙값 **11.253 → 2.574초(77.1% 감소)**이며 로딩 횟수와 JSON SHA 일치는 유지됐다.
GPU는 사용하지 않았다. `/usr/bin/time`이 없는 환경이라 최대 RSS는 측정하지 않았다.

## 로컬 검증

- 새 prepared-input 회귀: 동일 map/reference·도로 footprint, 독립 execution state,
  원본 reference 교체/zone reset 이후 snapshot 보존, 설정/지도/선/raster/경로의 같은 크기 훼손,
  경로 누락과 다른 config 위치 검증 통과.
- 기존 lane 그룹과 이관한 planning 그룹을 2 worker로 동시에 실행: **345.292 / 108.276초**, 둘 다 통과.
  planning의 CTest 8개와 highway runtime ROS 시험을 포함한다. LiDAR·camera·bridge 전체 묶음은
  이번 로컬 실행에서 재실행하지 않았다. 이전 145개 명령/옵션 보존과 중복 방지 audit는 통과했다.
- highway reference execution, source approach, transition recovery, lane-change execution/recovery,
  readiness, reference feature, asset gate **8개 추가 CTest 통과(45.72초)**.
- `all`+명시적 시험 target을 합친 단일 catkin 호출은 판제/입력/카메라/adapter subset에서 통과했다.
  로컬 컨테이너에는 PCL 개발 패키지가 없어 새 CI의 전체 workspace 빌드는 여기서 재현하지 않았다.
- 실제 ROS dashboard 중복 실행: 새 프로세스 exit 2, 기존 node URI/PID 유지, HTTP 200.
- 네트워크 격리 통합 launch: 노드 10개 모두 등록, 기본 highway gate true,
  첫 mode 상태 수신까지 5.239초(rosmaster 기동 포함). 센서 입력 없음에서 accel=0, velocity=0,
  brake=0.3의 정지 출력을 확인했다. 명시적 highway=false의 roslaunch parameter 전개도 검사했다.
- CI runner/coverage 단위시험, launch/default 계약, dashboard 단위시험, 문서/링크 검사를 통과했다.
  호스트에서 container-built native tool을 바로 호출한 library 경로 오류는 컨테이너 안에서
  `source devel/setup.bash` 후 asset/preflight를 재실행해 확인했다.

원격 CI의 실제 전체 시간은 이 로컬 검증 수치와 별도다. 변경 후 Actions 결과를 비교해야 하며
로컬 병렬 345초를 GitHub 전체 CI 시간으로 사용하지 않는다.

## CI 분석

실제 성공 run [CI 40697240894](https://github.com/INHAautonav/2026-ASMC/actions/runs/40697240894),
[Planning 40697240877](https://github.com/INHAautonav/2026-ASMC/actions/runs/40697240877)를 확인했다.

| 항목 | 관측 |
|---|---:|
| 전체 CI job | 약 16분 36초 |
| CI 회귀 묶음 | 523초 |
| LiDAR 단독 그룹 | 173초 |
| lane 그룹 | 350초 |
| camera-unit / camera-ros / bridge | 46 / 62 / 19초 |
| 별도 Planning job | 약 7분 24초 |
| 공통 CI의 ccache | hit 319 / miss 2 |

전체 CI가 이미 planner/controller를 빌드하는데 Planning job이 같은 소스를 다시 구성·컴파일·링크했다.
입력 executable 4개와 `lidar_observation.test`도 양쪽에서 실행했다. 반면 AVOID/로터리/highway runtime은
고유 검사여서 제거하지 않고 공통 CI의 `planning` 그룹으로 옮겼다. Node 20, 명시적 gtest build 대상도
함께 옮겼다. 공통 CI 안의 전체 빌드/명시적 시험 빌드 두 catkin 호출도 `all`+시험 target
한 호출로 합쳐 재구성·메시지 검사·make 순회를 줄였다. apt의 Node 중복 설치도 제거했다. 시간 민감 LiDAR 단독 실행, worker 2개, 각 fixture의 timeout/허용 오차는 유지한다.

`planning-catkin-build` check 이름은 CI workflow의 결과 gate로 남기고, 실패/취소는 실패로 전달한다.
최소 패키지 의존성 분리 빌드는 Planning workflow의 **수동 실행**으로 보존했다. 이는 자동 PR마다
독립 whitelist 빌드를 하던 검증 정책의 변경이다. package.xml/CMake/외부 의존성 변경 시 수동 실행한다.
GitHub 보호 규칙 API가 계정 기능 제한(403)으로 조회되지 않아 workflow 자체를 고정한 외부 규칙은 미확인이다.
기존 145개 명령/옵션 조합과 이전 판제 고유 검사의 보존, 중복 build/입력 실행 재유입을 자동 검사한다.

별도 job의 runner 사용량은 줄지만 기존 lane 350초는 그대로 남는다. 전체 PR 대기시간이 7분 줄었다고
해석하면 안 된다. 차선 fixture는 다른 작업에서 수정 중이므로 이번 변경에서 건드리지 않았다.
향후 추가 개선은 그 작업이 합쳐진 뒤 가장 긴 fixture를 재측정하고 테스트별 독립 ROS master를
유지하는 분할을 검토한다. 정지·상태 전이 검사의 대기 시간을 일괄 줄이는 방식은 제외한다.

## 실제 MORAI 확인과 복구

현재 검증은 네트워크 격리 환경이며 전 구간 주행 검증은 미실시다. 다른 작업의 시뮬 시험이 끝난 후
사용자가 선택한 작업트리를 컨테이너에 연결하고 전체 빌드/source 후 다음을 확인한다.

1. bridge·camera·LiDAR가 준비된 상태에서 통합 판제를 한 번 실행한다. 기본 ON에서 지도 로딩 1회,
   재사용 6회와 planner runtime 초기화 시각을 확인한다.
2. 초기화 완료와 최초 valid state/trajectory/selector NORMAL 시각을 각각 기록한다. GPS 음영에서
   시작했다면 센서 복귀 대기는 별도 수치로 기록한다. timeout/invalid 제동을 해제하지 않는다.
3. 일반 구간·로터리·고속도로 합류/차선변경·터널 진입/복귀를 확인한다. 실행 CPU가 빨라진 것만으로
   고속도로 기능 기본 ON의 전 구간 수락을 대신하지 않는다.
4. 이미 띄운 dashboard가 있다면 `dashboard:=false`를 사용한다. 중복 확인 시 기존 콘솔이 남고
   새 dashboard만 포트 점유 메시지로 종료하는지 확인한다.

문제가 있으면 고속도로 기능은 명시적 `highway_merge:=false`로 이전 기본 동작을 선택한다.
초기화 최적화만 되돌릴 때는 `HighwayReferenceExecution`의 세 `InitFromPrepared` 호출을 기존 `Init`로
복구해 별도 빌드한다. 운영 중 라이브러리를 교체하지 않는다. CI 변경은 런타임 수정과 분리해 되돌릴 수 있다.


## 사용자 후속 요청 — 통합 판제 패널

통합 launch 한 번으로 판제 패널 서버도 사용하도록 `dashboard=true`를 유지한다.
브라우저 자동 열기는 사용자 요청대로 **`open_dashboard=false`**를 유지한다.
별도 dashboard launch는 기본 운용 절차에서 제외하고 터미널의 HTTP 주소를 눌러 패널을 연다.
서버 default/forwarding과 브라우저 OFF를 launch 계약 회귀로 검사한다. 기존 10개 노드 통합
smoke와 동일한 기본 구성이다. 공용 컨테이너 마운트·호스트 실행 서비스는 변경하지 않았다.


## 원격 실행에서 추가 확인한 다운로드 비용

첫 PR run `40725154074`는 사용자 후속 기본값 반영으로 취소됐다. 그 실행의 checkout은
16:23:54~16:30:33 UTC, **399초**였고 fetch가 대부분이었다. 이전 성공 run의 약 57초와 달라
다운로드 환경 편차가 크며, 빌드 중복만으로 모든 지연을 설명할 수 없다. Git 추적 바이트는
1,558,508,776이다. 대형 배포 모델과 51개 tar.gz 보존 근거(718,253,777 bytes)가 포함된다.

`validated-lidar-build`는 코드·launch·의존성만 검사하므로 해당 job에만 non-cone sparse checkout을
적용했다. PT/engine/ONNX/tar.gz를 받지 않되 전체 소스, 기존 퇴역 경로 탐지와 보존 참조 문서는
유지한다. 로컬 독립 임시 clone에서 실제 materialized bytes **332,776,604(78.6% 감소)**,
일반 감사 통과 및 퇴역 Main/package.xml을 심었을 때의 탐지까지 확인했다. 이는 전송 압축 바이트나
확정 소요 시간이 아니라 working-tree 바이트 비교다. 전체 회귀 CI checkout은 변경하지 않았다.

[actions/checkout v4 구현](https://github.com/actions/checkout/blob/v4/src/git-source-provider.ts)은
sparse-checkout을 지정하면 fetch에 blob:none을 적용한다. 별도 filter 인자로 덮어쓰지 않았다.


## 사용자 승인 자산 정리

현재 tree에서 미사용 자산을 제외하고 Git 이력으로 보존하라는 후속 요청을 적용했다.
[파일별 이력 목록](../../../config/artifacts/git-history-assets-v1.json)에 65개 경로·크기·SHA·blob을 기록했다.
기준 main `af6df9e66ecb439bc44be94ef4370e2d97e010e0`에서 모든 원래 바이트를 검증했다.
기록 목록은 현재 tree에 남으며 과거 대형 blob을 CI가 다시 받도록 하지 않는다.

현재 모델 선택은 객체 E3·E8, 차선 Large E8·같은 가중치의 PyTorch 복구다.
Medium 기본값이 남아 있던 단독 camera_lane/facade도 Large로 맞췄다.
COCO pretrained의 공개 선택은 거부한다. BDD/YOLO11m 실험 설정은 역사 가중치를 복구한
별도 checkout의 재현 자료이며, 현재 모델이 패키지에 있다고 표시하지 않는다.

로컬 camera-unit 31.8초 통과, E3/E8 및 Large 패키지 launch/manifest 검사 통과,
native association 회귀 6개 통과, camera-ros 60.1초 통과했다.
초기 camera-ros 두 실패는 최소 시험 컨테이너에 native module/HD-map 메시지가 준비되지 않은
문제였으며, 해당 빌드 산출물을 준비한 후 동일 fixture 전부를 통과했다.
D의 원본·reference·full checkpoint와 병행 작업트리는 변경하지 않았다.
