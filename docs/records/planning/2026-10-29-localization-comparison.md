# Localization 1차 비교 체크포인트 — 2026-10-29

현재 실행 안내는 [planning-control](../../operations/planning-control.md)이며 과거 실험·SHA는
[개선 기록](2026-10-29-localization-integration.md)에 보존한다. 이번 통합은 추가 튜닝보다
재현 가능한 비교와 최신 main 계약 유지가 목적이다.

## 기준과 이관

- 원격 main 기준: `fad73f82068eee3869e91c92bc6bef4b2e713752`.
- 실험 선정 소스: `fb6981971e8b3e07a0dc0cdfb2f1e36052666ab9`.
- 통합 코어: `c6c6956f`. 비교 브랜치: `integrate/localization-compare-20261029`.
- 원본 전체 이력은 `research/shadow-localization`,
  `archive/localization-validated-20261029-fb698197`,
  `archive/localization-wall-wip-20261029`와 검증한 incremental bundle로 보존한다.
- 172개 연구 이력의 과거 merge 재연은 최신 main 파일을 반복 복원하므로 중단했다.
  최신 main에서 3-way squash 통합 후 충돌과 최종 diff를 검토했다. main 자체와 다른 worktree는 수정하지 않았다.
- main의 카메라·LiDAR·기본 재밍 패키지 전체, 기존 CameraBBoxLaneResult와 CI 공통 빌드는 유지했다.
  Localization에는 새 메시지 LocalTrajectory/ControlHandoff, 공용 core의 opt-in 확장만 연결했다.
- 현재 public main에는 개발 중인 별도 LOCAL 선택이 없다. 다른 채팅의 브랜치를 합치지 않는다.
  LOCAL 병합 후 해당 PR의 공식 옵션으로 같은 조건을 재현해야 3방식 비교가 성립한다.

## 실행과 책임

`asmc-localization-compare`는 이 worktree를 `/root/ws`로, 새 전용
`.runtime/localization-compare-20261029/{build,devel,install}`을 사용한다.
GPU·host network·2GiB shm·12GiB RAM, 빌드는 별도 CPU2/6GiB 컨테이너다.
옛 campaign 실행 파일은 새 소스 실행에 재사용하지 않는다.

새 대화형 Bash에서는 ROS와 `/devel`, 공용 master 11311이 자동 적용된다.
공용 master와 raw bridge가 준비된 상태에서 같은 컨테이너의 각 터미널에 입력한다.

```bash
docker exec -it asmc-localization-compare bash
roslaunch asmc_camera camera.launch
roslaunch asmc_lidar lidar.launch
roslaunch asmc_lidar view.launch dashboard_port:=8097
roslaunch asmc_planner autonomy_no_bridges_v2.launch jamming_planner:=localization
```

판제: <http://localhost:8089/web/index_v2.html>의 Auto 또는 Localization map 후보·선택.
LiDAR: <http://localhost:8097>. main 판제 기본 포트는 8088로 유지한다.
전체 파이프라인을 한 worktree씩 실행한다. 기존 autonomy를 Ctrl-C로 종료한 다음
`roslaunch asmc_planner autonomy_no_bridges_v2.launch jamming_planner:=camera`으로 복귀한다.
선택하지 않은 재밍 제어기는 실행하지 않으며 `/ctrl_cmd`는 selector 한 개가 소유한다.

자동 시험은 환경을 명시한 private master 11371을 사용한다. 대시보드는 같은 master의
읽기 전용 관측자이며 별도 master·인지·제어를 시작하지 않는다. 시험마다 own-perception으로
카메라·LiDAR를 실행하고 lease 안에서 종료한다. observer 5Hz/후보24개/64점과 HTTP10Hz를
ON으로 기록한다. 렌더·인코딩 비용과 planner processing_ms를 구분하며 hard real-time 보장으로 해석하지 않는다.
자동 observer가 종료된 후 수동 launch가 8089를 사용한다. 다른 master의 서버가 있으면 덮어쓰지 않고 오류로 중단한다.
map 위치와 base_link 관측은 변환 근거가 있을 때만 겹쳐 표시하며 stale/무효 경로를 현재 선택으로 남기지 않는다.

## 개선 범위와 선정 근거

- GPS/IMU/Ego 정렬, 한 적분기, anchor 검증과 GPS 복귀 검증·blend를 유지한다.
- LiDAR geometry의 시간 일관성으로 정지 근거를 확인한다. 의미 클래스와 실제 운동은 분리하고 미확정 속도를 0으로 만들지 않는다.
- 최대 0.6초 직전 경로를 새 Ego 위치로 잘라 현재 지도·객체·차체 검사를 통과할 때만 재사용한다.
  과거 명령·solver 상태를 그대로 재생하지 않으며 무효 입력·epoch 변경·실제 충돌 근거에서는 정지한다.
- 비교 MPC의 양의 가속도 상한1m/s², 가속도 증가/브레이크 해제 변화율2m/s³를 제한한다.
  긴급 감속·정지는 즉시 반영한다. 물리 차체 jerk가 항상 이 값이라는 뜻은 아니다.
- 추종 목표3m는 범퍼 간 거리다. 차량 기준점→앞 범퍼와 객체 footprint 보정을 더하며 충돌 여유를 삭제하지 않는다.
  서행 차량은 사용자 요청으로 새 비교에서 제외했고 실제3m 추종 수락을 주장하지 않는다.
- 벽 기반 지도 확장·5개 유도선 저속 대안은 `f52e6a2b`에 보존한 미검증 WIP로 통합하지 않았다.

## 보존된 실제 결과

MolitComp03, Foggy13시, 15km/h, 자체 카메라·LiDAR, GPS mask1880→1985m 조건이다.
원시 JPEG4개·LiDAR·motion·control bag과 native 평가 기록을 보존했다.

| 회차 | 소스 | 결과 | 중간 정지 최대 |
|---|---|---|---|
| static-k5 baseline | a77a8a23 | v2 수락 | 2.608s |
| static-k5 continuity | dc4f125d | v2 수락, 경로 재사용16회 | 0s |
| static-truck | dc4f125d | v2 수락 | 1.747s |
| static-k5 comfort-rise | fb698197 | v2 수락, 선정 버전 | 0s |
| crossing-left r02 | 5c431f60 | v2 수락 | 0.857s |
| standing-person | a77a8a23 | 완주했으나 정지 기준 실패 | 3.564s |
| angled-k5 | fb698197 | station1898에서 정지12s로 실패 | 12s |
| crossing-right | fb698197 | v2 수락 | 0.627s |

v2 수락은 해당 회차의 map/clearance/입력/제어/복구/기록 gates다. 모든 장면의 성공을 뜻하지 않는다.
vehicle/pedestrian v2 gates에는 터널 탈출3초 내 본차로 복귀를 독립 판정하는 gate가 없으므로
해당 조건은 추가 비교 항목으로 남긴다. OBB clearance는 실제 mesh 접촉과 동일하지 않다.
UDP type=-1/id=0은 시간·위치가 자차 상태와 일치하는 경우만 중립 상태로 분류했고 raw는 보존했다.
불일치 이벤트는 유리하게 제외하지 않는다. 20km/h, 모든 동적 장면, 장시간 반복, 26.R1은 미검증이다.

## LOCAL과 동일하게 고정할 비교 조건

station1845→2008, Foggy13시, 같은 센서셋·모델/engine SHA·출발 자세, 속도15km/h부터 비교한다.
무장애, static OBJ_Kia_K5, static OBJ_Hyundai_HD65, 좌/우 횡단 보행자,
정지 보행자와25도 정차 K5를 대표 장면으로 유지한다. 정차 차량은 NPC0km/h가 아닌 정적 에셋이다.
시나리오 정의는 `tools/scen_runner/localization_fixtures.py`, 자동 재현은
`run_localization_trial.py --own-perception --gps-outage --cap 15 --start 1845 --end 2008`에
고유 trial ID·fixture·세 연구 플래그를 함께 기록한다.

완주·시간·정지 횟수/시간/사유·접촉 raw/native/기하 근거·추종·계산 비용과
GPS 진입/복귀, 탈출3초 내 복귀를 동일 전체 시간축으로 비교한다.
Localization은 IMU/Ego 적분과 HD-map을 사용하고 LOCAL의 입력 차이는 해당 배포 후 명시한다.
인지 요청: 부분 가림·roll/pitch 때 OBB의 크기/방향 안정성과 fused confidence/association provenance.
이 PR은 인지 알고리즘·배포 engine을 바꾸지 않는다.

새 통합 빌드/실행 결과는 아래에 추가하며 과거 수락으로 대신하지 않는다.

## 최신 main 통합 검증

`86c270ca`의 깨끗한 소스/launch와 새 build/devel에서 확인했다.
전체 runtime 및 evaluator·회귀 타깃 빌드, 입력/epoch·정적 core·제어 바인딩25개,
GPS mode/recovery/selector/handoff/seed/reorder/approach ROS7개, 최신 main Frenet3개와
LiDAR 벽·천장·지면5개 회귀가 통과했다. 새 ROS 메시지2개는 정식 additive registry에
등록하고 실제 Noetic 생성 MD5와 독립 계산을 대조했다. 기존42개 메시지는 유지한다.

통합 후 `integration-localization-k5-15-r01`: 51.36초 완주, 중간 정지0초,
최소 native OBB 여유0.939m, 유효 이동 판단98.73%, 제어 최대 간격70.5ms,
planner p95 7.72ms. GPS/DR/복구 및 양방향 인계를 포함해 v2 gates 모두 통과했다.
진입/복귀 시 brake최대0/0.0039, 해당 인계 근처 속도최저7.94/14.86km/h다.
이 수치는 ROS 수신 기준이며 actuator 적용 latency 보장이 아니다.

일반 GPS station1700→1740에서 main r01과 localization r02는 같은15km/h 시험 정책으로
모든 normal gates를 통과했다. 횡오차p95는2.14/2.16cm, 제어 최대 간격62.8/54.8ms다.
Localization r01은 두 프레임의 공용 Frenet `all candidates blocked by collision`에 따른
brake0.3 이후 steady-speed gate만 실패했다. 해당 bag과 실패를 보존했고 무해한 노이즈로 확정하지 않았다.
재시험 성공을 최초 실패의 삭제나 전체 일반 구간 수락으로 해석하지 않는다.

기본 팀원 재밍 방식은 기존 target20km/h를 유지한다. 따라서15km/h 시험은 조건이 맞지 않아
16.36km/h에서 중단됐다.20km/h 시험도21.17km/h로 목표+1 제한을 넘어 중단됐다.
두 회차는 접촉/복구오류 없이 정리됐지만 진입 이후 실제 전체 복귀는 수락하지 않았다.
기본 팀원 제어를 이 PR에서 튜닝하거나 guard를 느슨하게 하지는 않았다.
양방향 기본 selector/GPS 복귀의 ROS 회귀 통과와 실제 주행 미완료를 구분한다.

독립 native 참조 추가 분석에서 선정 K5·트럭·우측 횡단·통합 K5는 station1996.73 첫
관측 시점에 이미 횡오차25cm/방향3도 이내다. 이는 첫 표본 결과이며 전체 차체의
차선 접촉 규칙이나3초간 유지 수락을 대신하지 않는다.

통합 비교 갤러리 [E744](http://localhost:8000/localization-obstacles-20261029-r01/scenario-diversity-review-r2/index.html)는
기존 E730을 보존한 새 페이지다. 여덟 회차의 성공/실패·후보·선택·native 위치·정지·OBB 여유를 비교한다.
모든 원본은 등록된 `localization-obstacles-20261029-r01` 아래 append-only로 보존한다.

LiDAR의 기본 `lidar.launch`는 debug stage가 꺼져 있어 콘솔의 진단/객체 상태만 보인다.
점군·전처리 단계를 보려면 `roslaunch asmc_lidar lidar.launch debug:=true`로 시작한다.
자동 runner의 `--lidar-debug`도 같은 선택을 기록하며 planner 알고리즘은 바꾸지 않는다.

## 선택 체계와 미병합 LOCAL

LOCAL PR [#138](https://github.com/INHAautonav/2026-ASMC/pull/138)의 공개 head는
`c73f10f6`이며 확인 시점에는 OPEN, main에는 미포함이다. 해당 코드는 가져오지 않았다.
그 PR의 명칭을 존중해 정본 선택은 `jamming_planner:=camera` / `jamming_planner:=localization`으로 맞췄다.
선택 인자는 `jamming_planner` 하나로 통일했고 호환 별칭은 제거했다.
아직 `jamming_planner:=local`은 실행 전 오류로 거절한다.
#138 병합 후에는 최신 main을 다시 통합해 local 분기와 공용 foundation의 양쪽 의도를 검토해야 한다.
별도 브랜치의 구현을 미리 합쳤거나 세 방식 통합을 이미 수락한 것으로 표현하지 않는다.

자동 시험 도구도 `--jamming-planner camera|localization`으로 선택하며 기존 동결 run의 명령·기록은 보존한다.
