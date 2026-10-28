# main 기반 고주로·회전교차로 통합 — 2026-10-26

기준 main은 `0dab4adf643470ddcff2edb26c3e8ca91928a196`, 주행 검증 기능의
출처는 `fix/roundabout-confidence-v6`의 `29dcc1063333bad511648511f2ee380ea6f07da9`다.
기존 검증 브랜치는 그대로 보존하고 main 위의 별도 통합 브랜치에서 작업한다.

## 통합 정책

- main의 단일 `asmc_lidar` runtime, typed observation, 품질·속도 validity,
  재밍·카메라·신호등·Competition 속도 입력과 일반 정지 개선을 유지한다.
- 검증한 고주로 간격 판단·좌우 차선변경·실패 복구·참조선 인계와
  회전교차로 관측 신뢰도·회전 예측·서행 진입을 이식한다.
- 관측 이력은 원본 시각으로 적합한다. 계획 복사본만 자차 시각으로 한 번
  진행시키고 미래 질의는 원본 모델의 누적 시간에서 계산한다. 차로 곡선,
  측정된 차체 corner, 유한 회전 예측과 속도 구간을 직선 모델로 바꾸지 않는다.
- `SpatialStopOptions`에서 `braking_only`와 `allow_partial_horizons`를 분리한다.
  일반 신호 정지의 재가속 금지와 고주로 부분 접근을 독립적으로 적용한다.
- 내부 MERGE 대기 STOP이 외부 MERGE 완료로 처리되지 않도록 요청 소유권을
  확인한다. 실제 고주로 후보 검사와 별도의 고정 대기점 KEEP 간격 gate를
  중복 실행하지 않는다. 현재 차체와 실행 구간의 충돌 검사는 유지한다.
- 일반 구간 절대 속도 제한과 동적 구간·신호 제약은 main을 유지한다.
  활성 고주로의 상한은 기존 검증 설정을 사용하고 더 낮은 제약을 우선한다.

새 군집 프로파일은 `src/asmc_lidar/config/lidar_pipeline.yaml`의
`lidar.Clustering.Adaptive`에 있다. 앞 1.30→1.32 m, 뒤 1.60→1.62 m,
10→50 m 선형 보간이며 지면 제거·수평화 후 sensor-local x 부호를 사용한다.
x=0은 앞, 누락된 rear 키는 앞 설정을 상속한다. 두 점의 연결 조건은
각 점 tolerance의 최솟값이며 장착 위치 이동은 추가하지 않는다.

지도는 main의 공식 v7 벡터·격자로 통일했다. 원본 파일은 수정하지 않으며
reader 호환 처리의 범위는 [지도 안내](../config/frenet/hd_map/README.md)를 참고한다.

## 빌드·실행

주행을 종료한 다음 사용자 checkout에서 새 통합 브랜치를 받는다.
컨테이너 안에서는 outer workspace만 빌드하며 구 Main/nested setup을 source하지 않는다.

```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
./scripts/build_ws.sh -j2
source devel/setup.bash
python3 tools/planning/sync_hd_map.py --check
python3 tools/planning/check_highway_merge.py
```

각 터미널의 공통 환경은 `/opt/ros/noetic/setup.bash`와 `/root/ws/devel/setup.bash`다.
다음 네 항목을 각각 별도 터미널에서 실행한다.

```bash
./scripts/udp_bridge.sh
```

```bash
roslaunch asmc_camera camera.launch
```

```bash
roslaunch asmc_lidar lidar.launch
```

```bash
roslaunch asmc_planner autonomy_no_bridges_v2.launch highway_merge:=true
```

고주로는 opt-in을 유지한다. 대시보드는 planner launch에 포함된다.
기본 상태 입력은 GPS/IMU 위치 + Competition 종방향 속도다.
`objects_source`, `lidar_source_policy`, `traffic_light` 구 인자는 제거된 main 계약을 따른다.
구 `Main main.launch`, 별도 BBox 변환기, 별도 dashboard를 중복 실행하지 않는다.

## 검증 범위

실제 production C++를 이용한 native 회귀와 기존 bag 고정 입력 재생은 통합
로직을 검사한다. ROS 메시지 전송·catkin 전체 빌드와 새 LiDAR가 생성하는
객체 변화, MPC/MORAI 폐루프 주행은 이 환경에서 확인할 수 없다.
따라서 이 결과만으로 전체 미션 인수나 기존과 동일한 주행을 보장하지 않는다.
통합 브랜치에서 ROS 빌드와 센서 연결을 확인한 다음 회전교차로·고주로·정지
구간을 주행 검증해야 한다. main 병합과 PR 생성은 수행하지 않는다.

실행한 오프라인 검증:

| 항목 | 결과 |
|---|---|
| 실제 Frenet/참조선/신호 core 48개 C++ 단위 | 빌드·분리 library `--no-undefined` 링크 통과 |
| 순수 C++ Frenet 회귀 | 65개 suite 통과; ROS transport suite는 제외 |
| 관측 시간·예측 모델 | 194개 검사, 0/50/100/200 ms, 누적 보정·곡선·불확실성 보존 |
| 정지 옵션 독립성 | 일반 신호 braking-only와 고주로 부분 horizon 조합 통과 |
| Behavior 속도 중재 | 12개 검사; 고주로 상한과 main 일반/동적/신호/긴급 제약 |
| 군집 | 실제 PCL KD-tree 743개 검사; 앞뒤 10/30/50 m, x=0, 혼합 점·순열 |
| 지도 | 공식 v7 벡터와 모든 raster cell 일치; 원본 제품 역순·hole·차체 검사 |
| 실행 연결 | 변경 launch의 include 인자 35개, 문서 launch 27개·명령 92개, 재밍 hash 검사 |
| 고주로 native preflight | 합류 2개 구간·주행 참조선 3개 준비 통과 |
| 기존 회전교차로 bag | 아래 3개 고정 입력에서 READY 확인 |

| bag | 최초 READY (bag 시작 기준) |
|---|---:|
| `highway_merge_diag(20261026-072038).bag` | 15.840 s |
| `highway_merge_diag(20261026-072244).bag` | 27.141 s |
| `highway_merge_diag(20261026-072451).bag` | 21.121 s |

이 재생은 기록된 ego/객체 입력을 유지한 PREPARE/READY 계산이다. 실제 차량이
새 궤적을 추종한 반사실적 폐루프 결과나 새 LiDAR 파이프라인 재처리 결과가 아니다.
Behavior의 순수 계산 빌드에서는 사용하지 않는 v1 payload와 ROS 로그 경계만
대체했으며, 이를 ROS adapter 컴파일·전송 시험 통과로 세지 않는다.
