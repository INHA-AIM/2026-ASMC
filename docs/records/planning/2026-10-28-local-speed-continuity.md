# LOCAL-2C: 속도 연속성·입력 시각 경계·노면 OBB 원인

**local 속도 기준 생성과 두 입력 교체 경계를 수정했다. 노면의 긴 OBB는 지면
fitting 후보까지 대조했지만 기본 승격하지 않았다. 반복 중 급제동이 남았으므로
음영구간 무정지 주행 완료는 아니다.** 사용자의 최신 결정에 따라25km/h 달성을
완료 조건으로 삼지 않고10·15km/h의 추종과 실패 원인 분리에 집중했다.

- 시작 revision `656f8a7b`, 브랜치 `feat/shadow-local-footprint`, 개발 위치
  `2026-ASMC-work`. 속도 수정 첫 커밋 `7fe9b7d2`.
- [실험 목록](http://localhost:8000/) → **LOCAL-2C 속도·노면 OBB 검수**.
- [전체 결과](http://localhost:8000/local2c-20261028-audit01/review-v2/index.html),
  [주행별 경로·제동·실패 RGB](http://localhost:8000/local2c-20261028-audit01/campaign/index.html),
  [지면 평면과 원본 점](http://localhost:8000/local2c-20261028-audit01/ground/index.html),
  [실제 barrier 보존 대조](http://localhost:8000/local2c-20261028-ground02/index.html).
- D 정본의 `artifacts/local2c-20261028-*`에 source/입력/설정/바이너리 SHA,
  원본 bag·판단·native 접촉·복구 결과를 기록했다. 기존 LOCAL-1/2A/2B run은 읽기만 했다.
- 별도 `asmc-local2c-20261028` 컨테이너와 독립 master로 회귀를 실행했다. 실제 시험은
  공유 simulator lease를 얻고 중복 제어/인지 생산자를 검사한 뒤 시작했다.
  직접 시작한 camera/LiDAR/local/selector/recording만 정리하고 scene·Ego를 복원했다.
  IMU localization·lane-perf worktree, 기존 bridge와 semantic 노드, SensorData는 수정하지 않았다.

## 속도 흔들림: scalar cap과 물리 궤적 끝점의 혼용

기존 NORMAL의 `SpeedProfiler::Update`는 내려간 속도 상한을 scalar 목표에 바로 적용한다.
이는 NORMAL 정책의 의도된 동작이다. LOCAL은 그 값을 시간 궤적으로 쓰면서 Δv를
가속/jerk로 다시 제한하고 **구간 평균 가속을 다음 구간 끝점 가속으로 Commit**했다.
그 결과 cap보다 조금 빨라도 필요 이상으로 감속한 뒤 재가속하는 기준이 만들어졌다.

LOCAL은 기존 분석적 S-curve 적분 코어를 `AdvancePhysicalSpeed`로 호출해 속도와
가속의 물리적 끝점 상태를 그대로 다음 궤적점에 전달한다. 기존 NORMAL Update/Preview
분기·정책·제동거리·가속 상한·충돌 검사는 유지했다. 또 다른 속도 조절기를 만들지 않았다.

동일 free raster·초기4.3m/s·가속0·cap15km/h의 native Plan 대조에서,0.5초 궤적점은
기존4.054m/s·수정 후4.1667m/s였다. 이후 약1.5초부터는 경로 끝의 정지거리 제약에
따른 감속이 필요하므로 계속15km/h로 유지하지 않는다. 별도 일정 cap 회귀에서는
초기 가속−1/0/+1m/s² 모두 목표속도·가속0으로 수렴했다. 이는 **고정 입력의
기준 궤적 대조**이며 실제 차량의 속도 정확도나 폐루프 수락으로 표현하지 않는다.
제어 trace에는 기준 가속·MPC 가속·가속 잔차를 추가해 원인 구분이 가능하다.

## 순간 입력 교체와 미래 상태

1. drive02: 계획 age200.9ms지만 직전 유효 solve가150ms 이내인 tick에서 급제동했다.
   새 계획 교체 대기 한계를200→250ms로 정리했다. **350ms가 지난 인지로 다시
   계산하거나 명령을 재발행하지 않고**, 마지막 유효 solve 이후150ms 이내만 기다린다.
   selector250ms watchdog·지속 중단 처리·source timestamp는 유지한다.
2. drive09: local 상태 stamp가 제어 시각보다 **67µs 앞서** `local_source_stale`이 발생했다.
   같은 session/frame의 직전 유효 solve가 있고 입력 자체가 최신이면, 최대5ms의 미래
   상태는 `motion_clock_pending`으로 발행 없이 기다린다. 미래 상태로 계산·음수 dt
   적분·timestamp 보정은 하지 않는다.150ms 이상 지속되거나5ms보다 큰 미래 값은
   정상으로 수용하지 않는다. 독립 ROS clock으로67µs·회복·지속 중단·7ms 거부를 검증했다.

시계가 앞선 근본 원인이 NTP인지 송신 clock인지까지 확정한 것은 아니다. 수신된 상태와
제어 tick의 실제 차이, 잘못된 급제동, bounded 대기 동작을 재현한 결과다.

## 노면의 긴 OBB: 크기/나이 필터보다 지면 추정 수정 우선

기존 [재호 인계](http://localhost:8000/local2b-20261028-perception-handoff01/index.html)의
scan `1793189383321630716`을 native pipeline으로 재생했다. 이 scan에서는 터널 필터가
실행되지 않았다. 따라서 이 사례를 벽 RANSAC의 오류로 분류하지 않는다.

- 네 군집3/4/5/6은65/56/56/85점, 합계262점이다. 실제 base 좌표의 Z범위는
  0.131/0.048/0.103/0.159m다. 수평화 OBB는 길이4.34–7.11m, 높이0.129–0.204m였다.
- 기존 평면은 `[-0.01313186,-0.01492088,0.99980244,1.60304358]`.
  이 점들은 평면 **아래 약0.316–0.414m**에 있었다.
- 현재 fitting consensus도 far 제거 band0.40m를 사용한다. 낮은 벽 등까지 넓은
  합의에 포함되면서 이 프레임의 평면이 노면 위로 치우쳤다. 제거는 **절댓값 거리**라
  평면 아래라도 band 바깥 점은 비지면으로 남아 긴 OBB가 됐다.
- fitting만0.10m로 분리한 후보 평면은
  `[0.00063576,-0.03486037,0.99939199,1.81236056]`였다. 제거 near/far0.30/0.40m는
  유지했다. 네 군집262점은 모두 제거되고 같은 scan의 양의 높이 군집7–12의1,711점은
  보존됐다. 두 추정값은 독립 노면 GT가 아니다.

`ground.fit_distance_threshold_m`은 **명시적으로 설정할 때만 사용하는 실험 선택지**다.
미설정은 기존 fitting과 동일하다. 운영 `lidar_pipeline.yaml`은 변경하지 않았다.
실험 설정은 audit01의 `ground-tight.yaml`과 SHA로 고정했다. 아래 점을 무조건 삭제하는
새 부호 규칙·차체 높이 변경·HD맵 임계값 변경도 없다.

### 실제 장애물 보존과 남은 제한

기존 LOCAL-1 drive13/15에는 원시 LiDAR 점군이 없어 replay가0scan이었다.
이를 성공으로 세지 않고 ground01에 자료 부족으로 남겼다. 같은 left barrier 배치를
새로 만들고3km/h 상한에서5.04m 접근했다. native 접촉0, 회피·통과 시험은 아니다.

원시117scan을 두 설정으로 비교했다. 시나리오에 고정한 `PE_Firewall_Orange` 근처
발행 map 객체를 양쪽 모두 같은1m gate로 찾았다.37scan은 map 무효이므로 비교 불가다.
나머지80scan에서 기존71·후보78회 확인됐고 **기존에 있던 객체가 후보에서 새로 사라진
경우는0**이었다. 특정 위치 근처의 대응 검사이며 전체 검출 재현율이나 분류 수락이 아니다.

이 실제 barrier는 기존 OBB 높이 중앙값이 약4.5cm에 불과했다. 한두 LiDAR ring의
반사만으로도 실제 장애물이 검출되므로, “낮다/가늘다/새 객체다”만으로 버리는 정책은
부적절하다. 이름을 모르거나 군집이 합쳐질 수 있어 에셋 최대 길이도 단독 삭제 기준으로
사용하지 않는다. 길이·나이는 진단/의심 근거로 기록하고 실제 점·평면 잔차·관측 품질과
결합하는 방향이 적절하다.

후보는 다른 벽면 점도 더 보존했다. ground-kept 중앙값1426→1990점,
ground 계산 p95 0.65→2.25ms, 전체 처리 p95 8.51→9.51ms였다. 단일 replay의 비용이며
엄격한 부하 benchmark는 아니다. 원래 floor 실패 scan도 총 군집13→16으로 늘었다.
**노면 실패를 개선했다고 모든 벽·천장·낮은 물체에 대한 승격 근거가 확보된 것은 아니다.**

## 전체 반복 결과와 실패

모두65m 내외 빈 구간 시험이다. 속도 편차는 native 주행 시작10초 이후의 고정 구간으로
집계했으며 실패/회복을 빼지 않았다. 횡오차는 family 상대값이지 독립 GT가 아니다.

| run | cap km/h / fitting | 속도 편차 p95 km/h | family 횡오차 p95 cm | 급제동 명령 tick | 명령 최대 공백 ms |
|---|---|---:|---:|---:|---:|
| drive02 | 10 / 기존 | 0.512 | 4.45 | 5 | 149.7 |
| drive03 | 15 / 기존 | 0.290 | 5.80 | 0 | 100.0 |
| drive04 | 10 / 기존 | 0.067 | 4.24 | 0 | 50.6 |
| drive05 | 15 / 기존 | 0.254 | 5.53 | 0 | 50.4 |
| drive06 | 15 / 기존 | 0.345 | 6.43 | 2 | 100.0 |
| drive07 | 10 / 기존 | 0.068 | 4.70 | 0 | 100.1 |
| drive08 | 10 / 후보0.10m | 0.067 | 3.88 | 0 | 100.0 |
| drive09 | 15 / 후보0.10m | 0.292 | 7.68 | 1 | 199.9 |
| drive10 | 15 / 기존·시각 보완 후 | 12.813 | 4.45 | 29 | 200.0 |
| drive11 | 15 / 기존·시각 보완 후 | 0.294 | 7.83 | 2 | 50.7 |

- drive01은 selector 실행파일 준비 실패로 출발하지 않았다. 별도 실패 목록에 유지했다.
- drive02의 입력/교체 경계, drive06/11의 긴 노면 OBB, drive09의67µs 미래 상태는
  원본 판단·제어·RGB로 분리했다. 실제 접촉은 전체0이다.
- drive10은 입력 지연으로 정지/작은 후진이 반복됐다. 같은 호스트의 runtime 회귀가
  초반과 겹친 부하 교란도 있다. 이를 일반 주행 성능 수락에 쓰거나 원인을 그 부하로
  단정하지 않는다. 실패 기록은 그대로 보존했다. drive11은 해당 부하 없이 재실행했지만
  노면 OBB 급제동2tick이 남았다.
- 계산 p95는 실제 주행 구간에서 약29.8–34.9ms. MPC·명령 공백·누락은 각 run에
  별도 집계했다. 초기3km/h 근거리 보조를 완전 관측 고속 통과로 세지 않는다.
- 갤러리 첫 생성에서 drive03 bag 읽기 메모리 오류가 있었다. 별도 새 생성 결과에서
  정확한 stamp의 bundle/RGB를 모두 읽었고 누락0이었다. 첫 시도 오류 로그를 보존했다.

속도 기준 자체의 불필요한 왕복은 개선됐지만 **모든 원인의 급제동이 없어지지는 않았다.**
운영 승격·전체 LOCAL-1 완료·25km/h 수락은 하지 않았다.

## 수정 위치·실행·회귀

```text
src/asmc_planner/src/Frenet/Speed/SpeedProfiler.*   기존 물리 적분 함수 공개
src/asmc_planner/src/local/planner.cpp             LOCAL 궤적 생성에서 재사용
src/asmc_controller/src/local/node.cpp             bounded 교체/시계 대기·가속 진단
src/asmc_lidar/{include,src}/.../preprocessing/ground*
src/asmc_lidar/src/ros/runtime_config.cpp           fitting 실험값, 기본값 유지
src/asmc_lidar/tools/ground_fit_audit.cpp           native 동일 점군 비교
tools/scen_runner/                                소유권·기록·replay·갤러리
```

공용 메시지/MD5·주행 토픽·NORMAL 행동 정책·YOLO/family 모델·추적/fusion·충돌 형상은
변경하지 않았다. local planner가 별도 벽 RANSAC이나 길이/나이 삭제 필터를 갖지 않는다.
기존 map 경로/이강태 음영 제어기를 같이 실행하지 않는다.

검사: LOCAL 코어, 기존 SpeedProfiler 회귀, 실제 ROS 경계/중복 제어 검사,
ground18개·runtime33개 검사, 기록/갤러리 Python 검사, 실제 브라우저 링크/이미지/JS
검사를 수행했다. 변경 target을 별도 컨테이너에서 재빌드했다. 기존 generated 메시지와
변경 없는 의존 라이브러리는 baseline에서 읽었으며 전체 clean install이나 hosted CI를
실행했다고 표시하지 않는다.

현재 세션의 준비된 컨테이너에서 코드/비교를 확인할 수 있다.

```bash
docker exec -it asmc-local2c-20261028 bash
source /tmp/local2c-env.sh
export ROS_MASTER_URI=http://127.0.0.1:11311
# 다른 local/제어 실험이 없을 때만. 기본은 경로 preview이며 제어하지 않는다.
roslaunch asmc_planner local.launch
```

실제 반복시험은 `run_local_trial.py --start-perception`이 공유 lease와 기존 생산자를
검사한 뒤 자기 프로세스만 관리한다. `--lidar-config`는 그때만 허용되며 설정 SHA를
기록한다. 새로운 run은 저장 관리 도구로 별도 등록해야 한다. 기존 trial을 덮어쓰지 않는다.

native plane 대조는 ROS master 없이 다음처럼 수행한다. CSV는 기존 observer의
수평화 점군이며 fitting 외 설정은 현재 GroundConfig 기본값이다.

```bash
rosrun asmc_lidar lidar_ground_fit_audit \
  /artifacts/local2b-20261028-perception-handoff01/stages16/1793189383321630716.csv 0.10
```

복구는 local 속도/시각 수정 커밋을 별도로 되돌리고 재빌드한다. 실험 fitting은
`pipeline_config`를 기존 `lidar_pipeline.yaml`로 돌리면 원래 동작이다. 다른 worktree의
컨테이너 source/mount를 바꾸지 않는다. 시험 종료 후 자동 제어는 남기지 않았다.

## 다음 한 단위와 다른 localization 작업과의 중간 평가

우선 **지면 추정과 낮은 실제 장애물 보존을 묶은 LiDAR 품질 대조**를 권장한다.
재호에게 기존 handoff와 이번 plane/117scan 결과를 같이 전달한다. 후보0.10m의
노면/벽/낮은 barrier/box/drum/가감속·GPS 무효 사례를 같은 기준으로 검수하고,
수락되면 생산자에서 반영한다. planner는 검증된 품질 사유가 전달될 때만 순간 오검출의
위험/회복을 구분하고 길이·출현 횟수만으로 실물을 지우지 않는다.

그다음 단일 장애물 회피·복귀의 반복성과 입력 지연 실패를 이어서 본다. 현재 barrier
접근은5m에 그쳐 회피 완료가 아니다. IMU localization 경로와는 동일 배치·시작 위치·
속도 상한으로 충돌/정체/주행 진전/입력 나이/계산/복구를 비교한다. 이 채팅에서는 그
worktree를 실행·수정하거나 어느 쪽이 더 우수하다고 결론내리지 않았다.
