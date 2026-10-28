# LOCAL-2D 준비: main 리베이스와 실행 소유권

2026-10-29 기준. 기존 LOCAL 결과는 `3bb14526`이며
`archive/local-before-rebase-20261029`로 보존했다. 원격 작업 브랜치의
`1125582c`도 `archive/local-remote-before-rebase-20261029`에 보존했다.
main `be20138a` 위로 이력을 리베이스했고 E3 실행 수정 `51c9672a`는
출처를 포함해 별도 적용했다. 원격 push/강제 push/병합은 하지 않는다.

## 운영과 역사적 실험의 경계

PR127의 BBOX classifier/follower와 CameraBBoxLaneResult 정의는 main과 동일하게
유지한다. S1/S2/S3의 옛 접점 실행·메시지 확장을 이 노드에 재적용하지 않았다.
옛 소스·bag·fixture는 백업 revision에서 함께 재현한다. 현재 기록 분석기는 옛
bag의 저장 메시지를 계속 읽을 수 있다. 과거 ROS wire 전용 시험 세 개와
contact_path_probe는 현행 실행/CI에서 분리하고 원본 파일을 보존한다.
순수 접점/관측 코어 회귀와 최신 main의 ROS 시험은 계속 실행한다.
시나리오 planning에 옛 ASMC_SHADOW_* 실험 옵션을 주면 명시적으로 거부한다.

LOCAL은 `asmc_planner/src/local`, `asmc_controller/src/local`과
LocalTrajectory를 유지하며 옛 follower를 시작하지 않는다. semantic_space_trial은
현재 LOCAL 입력 생산자로 계속 사용한다. NORMAL 실행 기본값을 LOCAL로 바꾸지 않았다.

## 컨테이너

운영 `asmc-ros-noetic`은 `2026-ASMC` / `51c9672a`에 그대로 연결한다.
AIMDrive worktree와 복구 이미지는 변경하지 않는다.
LOCAL 전용 이름은 `asmc-local2c-20261028`을 유지한다.
직접 worktree bind 요청이 Docker에서 옛 repro 이름으로 등록되는 현상이 재현되어,
해당 미시작 컨테이너를 제거하고 canonical 부모를 `/workspace`에 읽기 전용으로
마운트했다. `/root/ws`는 `/workspace/2026-ASMC-work`로 연결한다.
우리 build/devel/install과 tools/scen_runner/.runtime만 별도 쓰기 마운트다.
다른 worktree와 Git 공통 저장소는 컨테이너에서 읽기 전용이다.

시뮬 lease는 `/validation-lease`로 공유하고 ASMC_VALIDATION_LEASE_DIR을 사용한다.
엔진 선택 pointer는 LOCAL의 .runtime에 따로 두며 검증된 D 엔진만 읽는다.
운영 pointer를 덮어쓰거나 모델을 재생성하지 않는다.
옛 devel은 devel-before-local2d에 보존했다. 새 빌드는 baseline/devel overlay를
사용하지 않는다. 복구 이미지의 옛 /tmp/local2c-env.sh를 실행하지 않는다.

```bash
docker exec -it asmc-local2c-20261028 bash
cd /root/ws
source /root/local-env.sh  # 기본 독립 master 11348
```

실제 시뮬 시험에는 공유 lease뿐 아니라11311의 제어/인지 생산자 확인이 필요하다.
현재 운영 노드를 종료해 시험을 강제로 시작하지 않는다. 새로운 등록 run에서만
결과를 기록한다. build 중 실주행 성능을 비교하지 않는다.

## 검증 근거와 다음 단계

D의 `artifacts/local2d-20261029-rebase02`에 range-diff, 충돌 처리 목록,
클린 build 및 회귀 결과를 기록한다. 첫 등록 시 입력 manifest 형식 오류로 거부된
rebase01의 준비 로그는 별도 보존하며 성공 run으로 계산하지 않는다.
현재 준비 결과와 이후 MPC 공백/지면0.10m/회피 변경은 별도 커밋으로 구분한다.
실제 폐루프10km/h 회피·복귀 성공 여부는 이후 기록에만 표시한다.

## 준비 완료 검사

- 전체 workspace를 새 build/devel에서 빌드했다. `/baseline` overlay는 없으며
  MPC·planner·LiDAR 실행파일과 링크된 라이브러리는 모두 이 worktree 산출물이다.
- 복구 이미지에는 pybind11 헤더가 없어 처음에는 Python fallback으로 configure됐다.
  전용 컨테이너에 pybind11 2.10.4를 설치하고 재configure했다. 실제
  `_lane_association.so` import, native/Python 6개 대조(300곡선 포함), 시각10개,
  실제 ROS family geometry SHA·추론 lifecycle 계약 검사를 통과했다.
- LiDAR41개 CTest 중 최초1개는 LocalTrajectory의 Odometry를 정적 MD5 검사기가
  모르던 실패였다. 실 wire는 바꾸지 않고 외부 MD5·메시지 registry를 등록했다.
  해당 검사 재실행과 계약11개가 통과했다. 나머지40개는 최초 통과했다.
- NORMAL MPC의 추종·가감속·시각·정지 재개 검사, 판제의 회피·동적 정책,
  LiDAR→판제 변환, LOCAL planner와 독립 ROS 입력/제어 소유권 검사가 통과했다.
- main의 BBOX classifier/follower·CameraBBoxLaneResult·MPCProcess는 바이트가 동일하다.
  과거 S1/S2/S3 flag가 새 follower에 조용히 적용되지 않는 거부 검사도 추가했다.
- 작업 도중 다른 대화에서 기본 checkout이 `57d48792`, AIMDrive가 `e7fc3185`로
  갱신됐다. 이쪽에서는 두 worktree에 쓰지 않았다. E3 선택의 추가 opt-out/검증 수정
  `4d3bd82d`만 출처를 남겨 우리 브랜치에 별도 적용했다. 원격 main은 종료 확인에도
  `be20138a`다. 연구/기본 branch의 이후 변화는 이 고정 기록과 구분한다.

로컬 실행 명령은 위 `docker exec`와 `/root/local-env.sh`를 따른다. 일반
`asmc-ros-noetic`에서 실행하면 이 LOCAL 빌드가 아닌 사용자 기본 checkout을 실행한다.
실제 시험 준비는 끝났지만 공유 시뮬의 다른 실행을 종료하거나 자동 제어를 시작하지 않았다.

## LOCAL-2D 오프라인 구현·검증

준비 이후 변경은 별도 커밋이다. 구현 범위:

- `asmc_controller/src/local/execution_history.hpp`, `node.cpp`: 실제 명령 시각의
  zero-order hold 이력을 MPC50ms 격자에 대입한다. 한두 solve가 빠져도 지연 이력이
  정지하지 않는다. 재개 시 warm start만 비우고 마지막 실제 조향·가속을 보존한다.
  미래 **관측**을 쓰는 것과 MPC가 미래 **상태**를 예측하는 것은 다르다. 67µs 미래
  관측은 계산·재발행 없이 기존150ms 한계 안에서 기다린다. 잘못된 quaternion/경로는
  이 대기를 통과하지 않는다. 350ms freshness·100ms 상태 전개·selector250ms는 유지한다.
  시간 이력6개와 실제 ROS 미래/만료/중복 생산자 검사가 통과했다.
- `asmc_lidar/config/lidar_pipeline.yaml`: 이 worktree의 fitting 합의 폭만0.10m로
  분리했다. 제거 폭0.30/0.40m, 최신 main 군집 설정은 유지한다. 길이/높이로 객체를
  지우지 않는다. 복구는 새 `fit_distance_threshold_m` 키를 제거하는 것이다.
- `asmc_planner/src/local/planner.{cpp,hpp}`: 곡선·격자·이전 경로의 최종 점수를 거리
  적분으로 통일했다. 목표점 유도 비용은 탐색에만 사용한다. 이전 경로를 현재 OBB·
  노면·곡률·조향 변화율로 재검사한다. 분할0.2/0.5m 비용 동등성, 새 장애물 유입과
  기존 local core 회귀가 통과했다. NORMAL 플래너 정책은 변경하지 않았다.
- `tools/scen_runner/scen_runner/records/local_preflight_report.py`: native replay 결과와
  실제 source stamp의 RGB·노면·경로를 연결한다. 기존 결과는 수정하지 않았다.

[LOCAL-2D 실험 검수](http://localhost:8000/local2d-20261029-offline01/index.html)
와 D `artifacts/local2d-20261029-offline01`이 결과 정본이다.

| 검사 | 결과 | 해석 한계 |
|---|---|---|
| 최신 main 설정의 barrier117scan | 근처 검출71→78, 신규 누락0, map무효37동일 | 1m 동일 연결 게이트이며 전체 재현율/독립 GT 아님 |
| 원본 긴 노면 군집 | 262점 제거, 양의 높이1711점 보존 | 한 실패 scan의 인과 대조 |
| local 객체 합계 | 123→131, 길이4m초과2→0 | 길이는 진단 수량, 벽/장애물 정답이나 삭제 조건 아님 |
| LiDAR 처리 p95 | 11.67→14.68ms | 공유 PC의 단일 offline 재생, 실시간 수락 아님 |
| 빈 터널30초 replay | 유효213/235, 계획p95 27.61ms, MPCp95 12.37ms | 입력 대기20·미완성1·객체무효1 포함, 최대 공백282.88ms |
| barrier replay | 유효95/114, 계획p95 40.96ms, MPCp95 13.67ms | 입력 대기19, assisted36 포함, 공백136.20ms |

계획 budget·clock·입력 무효를 감추지 않았다. 두 planner replay는 원래 기록의 LiDAR
관측을 그대로 사용하며, 새 fitting 결과를 과거 bag에 덮어씌우지 않았다. 경로가 바뀌어도
기록된 ego는 바뀌지 않으므로 충돌 없는 회피·복귀 성공으로 집계하지 않는다.

**남은 수락은 실제10km/h 무장애 추종과 좌/우 단일 장애물 각3회 회피·복귀다.**
최종 확인 시 다른 연구·시작 점검 컨테이너와 공용 camera/LiDAR 생산자가 실행 중이다.
11311의 제어 발행자는 당시 없었지만 새 fitting 생산자로 교체하려면 운영 LiDAR와
충돌하므로 그대로 유지했다. 현재 준비/오프라인 결과만으로 LOCAL-2D 전체 완료,
250ms 공급 수락 또는 운영 기본 전환을 선언하지 않는다. 15km/h는 추가 확인이고
25km/h 달성은 이 작업의 목표가 아니다.
