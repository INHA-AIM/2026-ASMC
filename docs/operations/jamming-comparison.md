# 세 재밍 플래너 비교 실행

이 안내는 `integrate/jamming-three-way-20261029` 비교 브랜치용이다. 기존 main·LOCAL·localization 작업트리와 PR은 별도로 유지한다.

## 선택과 실행

`asmc-jamming-compare` 컨테이너에 세 구현이 함께 빌드된다. 새 대화형 Bash는 이 워크트리의 `/devel`과 전용 master11373을 적용한다. 각 터미널에서 추가 env source 없이 실행한다.
공식 지도 정본은 `ASMC_DATA=/workspace/data` 병합 뷰로 읽는다. `/data`의 과거 상대 링크를 지도 정본으로 사용하지 않는다. main #140의 정적 구간 UNKNOWN 정책도 이 정본의 SHA를 검증한다.

```bash
docker exec -it asmc-jamming-compare bash
```

각각의 터미널에서 다음을 실행한다. MORAI 월드는 하나이므로 다른 물리 시험의 bridge/control이 없어야 한다.
컨테이너가 정지돼 있으면 먼저 `docker start asmc-jamming-compare`를 실행한다. roscore는 해당 master가 없을 때 한 번만 시작한다. 읽기 전용 콘솔을 먼저 켠 경우 같은 컨테이너·master·revision의 콘솔은 이후 `view.launch`와 autonomy에서 재사용한다. 다른 소유자의 포트는 종료하지 않는다.

```bash
roscore
roslaunch asmc_bridge bridge.launch
roslaunch asmc_camera camera.launch
roslaunch asmc_lidar lidar.launch debug:=true
roslaunch asmc_lidar view.launch
```

마지막 판제 터미널에서는 다음 중 하나만 실행한다. 기본값은 camera다. 방식 교체는 autonomy를 종료한 뒤 다른 옵션으로 재실행한다. 인지·bridge는 유지할 수 있다.

```bash
roslaunch asmc_planner autonomy_no_bridges_v2.launch jamming_planner:=camera
roslaunch asmc_planner autonomy_no_bridges_v2.launch jamming_planner:=local
roslaunch asmc_planner autonomy_no_bridges_v2.launch jamming_planner:=localization
```

전체 스택을 한 번에 소유하려면 기존 수동 master/노드가 없는 상태에서 `./scripts/drive.sh jamming_planner:=local`을 사용할 수 있다. 이 실행기는 자신의 master·lease·bridge·인지·제어·콘솔을 소유하고 종료한다.

- 판제: http://localhost:8089/web/index_v2.html — 자동 보기 또는 LOCAL/Localization map 보기.
- LiDAR: http://localhost:8097 — 점군 단계는 위의 `debug:=true`가 필요하다.
- 자동 시험은 별도 master11374와 같은 physical lease를 사용한다. 대화형 master를 재사용하지 않는다.
- 다른 작업이 canonical 포트를 사용하던 오프라인 회차는 판제8087·LiDAR8098을 사용했다. 이는 수동11373 콘솔과 다른 replay 그래프이며 완료 후 종료된다. 녹화 재생은 `BAG 재생 · 고정 자차 · 주행 수락 아님`으로 표시한다.
- 포트 점유 시 다른 실행을 종료하지 않고 시작을 거절한다. 다른 ROS master도 같은 MORAI 차량을 움직일 수 있다.

## 공통 전환과 책임

일반 구간 상한57.5km/h는 유지한다. 현재 속도·가속도와 공통 jerk/감속 전이를 사용해 A448 접근에 필요한 거리에서 목표25km/h를 적용한다. 속도 제한은 기존 Frenet 속도 계획이 한 번 적용한다.

A448=`A2256W000448`, 시작 노드=`A1256W000456`, 정본 경로 station1859.641578240m에서 재밍 제어를 시작한다. 경로와 링크 파일 SHA는 시작 시 확인한다. 실제 터널 시작1877.744798966m와 제어 진입점은 별도다.

| 옵션 | 기본 목표 | 입력·경로 | 제어 |
|---|---:|---|---|
| camera |10km/h|main의 camera 기준선과 LiDAR 물체·벽|기존 곡률 추종·페달 제어|
| local |10km/h|관측 노면/family/로컬 OBB와 상대이동|공통 LOCAL MPC|
| localization |15km/h|GPS anchor·IMU 상대이동으로 map Frenet 회피|공통 LOCAL MPC|

접근 중 선택한 방식만 준비한다. 인계 직전의 제한된 준비 시간에 현재 경로와 후보 명령을 만들고, 같은 epoch·생산자·시각·frame의 명령으로 넘긴다. 실제 속도25에서10/15 목표로 감속하므로 속도를 순간적으로 잘라 넣지 않는다. 후보가 준비된 정상 인계에 비상제동을 삽입하지 않는다. 선택된 명령 수신 watchdog250ms는 유지한다.

`/planning/mode`와 최종 `/ctrl_cmd`의 발행자는 각각 하나다. 공통 준비·인계는 `/planning/jamming/*`, 방식별 geometry/trace는 기존 `/planning/local/*`, `/planning/localization/*`, camera 진단을 사용한다. `LocalTrajectory`와 `ControlHandoff`의 필드·MD5는 유지한다.

GPS 건강 상태와 제어 소유권은 다르다. A448 안에서는 GPS가 정상이어도 선택 방식이 유지된다. Localization은 유효 GPS 기준점을 유지하다 실제 소실 시 DR로 이어간다. 구간 이탈·GPS 회복·새 정상 경로를 확인하고 돌아오며, Localization은 기존 anchor 복구 검증도 통과해야 한다.

## 회복·지도 경계

의미 그룹은 운동 상태가 아니다. 공통 운동 근거 코드는 자차 보상·연속 고유 관측·현재 footprint로 정지 근거를 판단한다. 이동/횡단은 양보하고, 앞에서 정지하면 새 관측으로 우회 경로를 다시 찾는다. Localization의 person/dog 고정 제외는 제거하며 재이동 관측은 정지 근거를 취소한다. 미확정 속도를 유효0으로 바꾸지 않는다.

LOCAL의 관측 공간 기반 복구와 Localization의 현재 OBB로 재검증한 경로 기억을 보존한다.5km/h는 검증된 복구 경로의 목표이며 입력 무효·통과 공간 부재를 무시하는 최저속도가 아니다. 입력 누락이나 실제 폐쇄 통로는 정지할 수 있다.

Localization은 페인트 횡단 허용과 주행가능영역 확장을 분리한다. 측정된 벽·바닥의57개 구간 폴리곤을 원본 HD맵 위에 추가하고, 지도 SHA·map frame·MolitComp03·0.65m 벽 여유·station을 검사한다. 벽 확장은 실제 터널 범위에만 적용한다. 페인트 허용의3초 복귀 유예가 벽 권한을 늘리지 않는다. 전체 차체와 sample 사이 sweep이 지도·현재 OBB 검사를 통과해야 한다. NORMAL이나 LOCAL은 이 확장으로 자유공간을 얻지 않는다.

LOCAL의 관측 노면/제한된 과거 관측과 Localization의 지도 사전정보는 서로 다른 입력이다. 기본10/10/15 비교와 모두10km/h 비교를 구분한다.

## 재현과 수락 기록

FOGGY13시, 동일 출발·배치·센서로 무장애 진입/복귀, 정차 차량, 기울어진 정차 차량, 좁은 정적 통로, 좌우 횡단을 비교한다. 서행차량은 제외한다. 정차 차량은 정적 에셋을 사용한다.

완주, native/UDP/영상 충돌 근거, 정지 시간·사유, 인계 공백, 추종 상태와 처리 비용을 원시 bag과 함께 기록한다. 열린 새 경로가 있는데3초 이상 정지가 남으면 실패다. 입력 무효와 실제 폐쇄 통로는 별도 실패/제약으로 남긴다. 관측 부재를 무충돌로 확정하지 않는다.

현재 구현의 검증 결과는 `docs/records/planning/2026-10-29-jamming-three-way.md`에 회차·source SHA와 함께 갱신한다. 과거 두 브랜치의 주행 성공은 이 통합본의 수락을 대신하지 않는다.

## 재부팅 후 콘솔 점검

화면이 열리지 않으면 컨테이너 재생성 전에 HTTP 리스너와 실제 master를 확인한다. 이번 복구에서는 서버 종료 상태와 header 밖의 상태바가 본문을 밀어내는 표시 오류를 각각 수정했다. Windows 브라우저의 지도·패널·LiDAR canvas를 확인했다. 입력이 없을 때 WAIT/STALE과 데이터 대기는 정상이며 다른 master의 데이터를 대신 표시하지 않는다.

재사용 검사는 같은 master, 컨테이너 내부의 살아 있는 ROS 노드, 포트, dashboard 소스 경로·revision을 확인한다. autonomy가 동일한 기존 dashboard를 재사용할 때는 셸 실행기가 `dashboard:=false`를 전달해 두 번째 서버만 생략한다. 플래너 선택·제어에는 영향을 주지 않는다. 새 대화형 Bash에서 적용된다.
