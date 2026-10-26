# 웹에서 Adaptive PCL 연결 거리 변경

이 기능은 실제 군집화 입력 설정을 바꾼다. 화면 필터가 아니다. UNIT은 사용하지 않는다.
기존 Main, UDP 브릿지, MORAI 설정, Hungarian/CA-KF 계산식 및 가중치는 변경하지 않는다.
추적 상태를 재생성하지 않고 단일 처리 worker가 다음 scan 경계에서 여섯 값을 한 번에 교체한다.
분할이 달라지면 기존 추적기의 연결 결과와 ID는 영향을 받을 수 있다.

## 정확한 파일과 키

컨테이너: `/root/ws/src/asmc_lidar/config/lidar_pipeline.yaml`

WSL: `/home/autonav/2026AIM/Yangtest_lidar_web_monitor/src/asmc_lidar/config/lidar_pipeline.yaml`

```yaml
lidar:
  Clustering:
    Eps: 0.5          # Adaptive 비활성 시 고정 연결 거리
    MinPoints: 5      # 군집 최소 점 수 (웹 변경 대상 아님)
    Adaptive:
      Enabled: true
      NearRange: 10.0
      FarRange: 50.0
      NearEps: 1.30       # 앞 x >= 0
      FarEps: 1.32
      RearNearEps: 1.60   # 뒤 x < 0
      RearFarEps: 1.62
```

정식 키는 `lidar.Clustering.Adaptive.NearRange`, `.FarRange`, `.NearEps`, `.FarEps`,
`.RearNearEps`, `.RearFarEps`이다. 기존 NearEps/FarEps는 앞쪽 연결 거리이며 추가한 Rear 키는 뒤쪽 값이다.
`src/detection/clustering.cpp`의 `AdaptiveClusterTolerance()`가 이 설정을 사용한다.
거리 r은 수평화된 LiDAR 점의 `hypot(x,y)`이며, LiDAR 원점 기준이다. base_link 원점과 다르다.
x 부호도 지면 제거 후 수평화 LiDAR 좌표에서 판단한다. sensor→body 변환이나 mount 이동을 추가하지 않는다.
x=0 및 -0.0은 앞쪽이다. 클래스에 관계없이 모든 해당 방향의 점에 같은 함수를 적용한다.

| 구간 | 앞 x≥0 | 뒤 x<0 |
|---|---|---|
| r ≤ 10m | 1.30m | 1.60m |
| r = 30m (중간 예시) | 1.31m | 1.61m |
| r ≥ 50m | 1.32m | 1.62m |

중간 구간은 각 방향에서 `near + (far−near) × clamp((r−10)/40,0,1)`로 선형 보간한다.
경계 두 개와 앞·뒤 연결 거리 네 개를 웹에서 조절한다.
군집의 두 점을 연결하려면 두 점 각각의 거리별 tolerance 중 작은 값 이내여야 한다.
연결된 점들을 따라 확장하므로 군집 전체 직경을 제한하는 값은 아니다.
x=0을 가로지르는 두 점도 작은 쪽 tolerance 이내면 연결된다. 앞·뒤를 별도 점군으로 강제 절단하지 않는다.
Rear 키가 없는 기존 YAML은 rear=front로 읽어 기존 대칭 동작을 유지한다.

## 실행

기존 처리와 모니터 터미널에서 각각 Ctrl+C 후 재실행한다. UDP 브릿지는 유지한다.
사용자 요청에 따라 이 변경 작업은 실행 중인 두 프로세스를 임의로 재시작하지 않았다.

```bash
# 컨테이너 터미널 1
source /tmp/asmc_container_env.sh
roslaunch /root/ws/src/asmc_lidar/launch/lidar.launch can_topic:=/Competition_topic lidar_level_roll:=true fusion:=false experimental_tunnel_filter:=false debug:=true clustering_tuning:=true
```

```bash
# 컨테이너 터미널 2
source /tmp/asmc_container_env.sh
roslaunch /root/ws/src/asmc_lidar/launch/view.launch open_browser:=false dashboard_port:=8091 clustering_tuning:=true
```

Windows 브라우저 `http://localhost:8091/`에서 Ctrl+F5를 누른다.
왼쪽 위의 “군집 연결 거리 · 실시간 설정”에서 수정하고 “실시간 적용”을 누른다.
“시작값 복원 준비”는 입력창만 시작값으로 복원한다. 실제 복원에는 다시 적용 버튼을 누른다.

## 상태와 보존

- 기본 `clustering_tuning:=false`: 읽기 전용. 두 launch 모두 true여야 적용된다.
- `pending`: HTTP가 요청을 전송했지만 아직 처리에 반영되지 않았다.
- `waiting_for_valid_scan`: 설정은 worker에 반영됐지만 유효한 LiDAR 처리 프레임을 기다린다.
- `applied`: 유효 프레임 처리 완료. 최초 적용 stamp_ns와 revision을 표시한다.
- 화면의 장면 revision은 노드의 적용 revision과 별도로 표시된다. Pause/전송 지연 시 다를 수 있다.
- 노드 재시작, 연결 해제, 상태 stale, 거부를 성공으로 표시하지 않는다. 자동 재전송하지 않는다.
- 숫자 NaN/Inf, FarRange < NearRange+1, FarEps < NearEps 등 기존 함수의 fallback/clamp를 유발하는 입력은 거부한다.
- 입력 범위: 0.1 ≤ NearRange, NearRange+1 ≤ FarRange ≤ 200m; 0.05 ≤ NearEps ≤ FarEps ≤ 5m.
- YAML/ROS 파라미터는 덮어쓰지 않는다. 프로세스를 재시작하면 YAML 값으로 돌아간다.
- 영구 보존은 위 YAML 키를 사용자가 직접 편집한 뒤 재시작한다. 실행 중 YAML 파일만 수정하면 자동 적용되지 않는다.
- 새 설정은 observation config_id에 반영한다. 각 debug scan에도 여섯 값/revision/config_id를 함께 저장한다.
- 이전 네 값 API와 새 여섯 값 API를 혼용하지 않는다. 처리·웹 노드를 모두 재시작해야 한다. 이전 처리 상태를 받으면 웹 적용을 잠그고 재시작을 안내한다.
- HTTP 설정은 loopback bind에서만 허용한다. 세션 토큰, Host/Origin 검사, 2KB 요청 제한을 적용한다.
- private ROS JSON 요청/상태는 `.../lidar_pipeline/clustering_request`, `.../lidar_pipeline/clustering_status`이다.

## 검증

실제 설치된 ROS Noetic/PCL 환경에서 `lidar_pipeline_node` 빌드 통과.
새 runtime 검사: 같은 점군이 연결 거리 변경 전 1개, 변경 후 2개 군집이 되는 것을 확인했고,
근·중·원거리 tolerance 값(0.3/0.6/0.9m), config_id 변경, tracker 인스턴스 유지도 검사했다.
이는 합성 입력의 기능 검사이며 실제 물리 객체 분할 성능 향상을 주장하지 않는다.

| 검사 | 결과 |
|---|---|
| C++ runtime (앞·뒤 YAML/기존 YAML 호환, config_id 및 runtime 전달 포함) | 31/31 PASS |
| C++ clustering (앞·뒤 반사 점군, x=0 경계와 순서 독립성 포함) | 18/18 PASS |
| Python 설정 전달/입력 검증/HTTP 보호/재연결 | 6/6 PASS |
| 기존 Python 웹 코어 | 33/33 PASS |
| 기존 JavaScript 좌표/OBB/표시/재연결 | 67/67 PASS |
| JavaScript 구문 검사 | PASS |
| package isolation contract | 9 PASS, 기존 Git mode 검사 1 SKIP |

현재 운영 중인 웹·처리 노드는 사용자 재시작을 기다리므로 이 문서 작성 시점의
새 UI 실제 브라우저 조작 및 MORAI live 적용은 미검증이다. 기능 검사와 실사용 검증을 구분한다.
