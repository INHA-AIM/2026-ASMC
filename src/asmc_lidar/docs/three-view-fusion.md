# M1 3뷰 fusion·전달 표시·정적 클래스 보조

기준선은 PR #119·#121 병합본 `52017fe2`다. 구현은 `feat/lidar-three-view-fusion`에서
진행한다. 2026-10-28 사용자 승인에 따라 **최적화한 3뷰 fusion을 기본으로 승격**했다.
정본 YAML과 코드 기본값은 `fusion.views: all`이다. `motion.semantic_static_enabled: false`는 유지한다.
기본 선택과 전체 대회 주행 수락은 구분한다. 초기 입력 공백·얇은 물체 소실은 미해결이며,
[승격 검증과 후속 복구 설계](fusion-recovery.md)를 따른다.
[시험 결과](../../../docs/records/lidar/2026-10-27-m1-three-view-fusion.md)에서 구현·검증·미수락을 구분한다.

## 실행 경계와 복구

운영 명령은 `roslaunch asmc_lidar lidar.launch`와 `roslaunch asmc_lidar view.launch` 그대로다.
점군 단계와 상세 근거를 보려면 처리 쪽에 `debug:=true`가 필요하다. 판제 입력은 debug와 독립적이다.
공개 ROS 타입·MD5·토픽·판제 행동 코드는 바꾸지 않는다.

다음 키는 내부 YAML에서 관리한다. 다른 값·보정·필터 임계값을 함께 바꾸지 않는다.

| 설정 | 현재 기본 | 내부 복구/후보 |
|---|---|---|
| `fusion.views` | `all` — 전방·좌·우 matching와 터널 보호 | `front` — 전방 단독 복구 |
| `motion.semantic_static_enabled` | `false` | `true` — 확인된 static 클래스 + 연속 기하 근거 |

시험은 `lidar.launch namespace:=/migration/lidar pipeline_config:=<시험 YAML> debug:=true`와
`view.launch namespace:=/migration/lidar dashboard_port:=8093 pipeline_config:=<같은 YAML>`로 실행한다.
이미 같은 입력의 운영 생산자가 있으면 처리 부하를 먼저 확인하며, 동일 정식 출력의 생산자를 추가하지 않는다.
전방 복구는 정본 YAML을 별도 파일로 복사해 `fusion.views`만 `front`로 바꾸고
`pipeline_config:=<복구 YAML>`로 실행한다. 다른 임계값·보정은 유지하며 새 config ID로 시작한다. launch 인자에 모델/운동 정책 선택지를 추가하지 않는다.

### 빌드와 실제 실행파일 확인

PR #119의 GPS 무효 시 터널 분기와 이번 3뷰 코드는 같은 실행파일에 포함돼야 한다.
`pipeline_config`는 YAML만 선택한다. 다른 checkout의 YAML을 지정해도 이미 빌드된
실행파일이 바뀌지 않으며, `debug:=true`도 빌드 선택과 관계없다.

PR #122 병합 후 정본은 `/root/ws`의 빌드다. 이전 개인 `/root/asmc-m1` overlay는
재현용으로 보존하지만 기본 셸에서 자동 source하지 않는다. 새 터미널과 LiDAR 웹도
같은 `/root/ws/devel/setup.bash`를 사용한다.

```bash
cd /root/ws
source /opt/ros/noetic/setup.bash
./scripts/build_ws.sh -j2 -l2 -DCMAKE_BUILD_TYPE=Release
source /root/ws/devel/setup.bash
rospack find asmc_lidar
rosrun --prefix echo asmc_lidar lidar_pipeline_node
python3 src/asmc_lidar/tools/check_runtime_build.py
```

예상 경로는 `/root/ws/src/asmc_lidar`와
`/root/ws/devel/lib/asmc_lidar/lidar_pipeline_node`다. 점검 도구는 노드를 가동하지 않고
선택된 실행파일의 `--build-info`와 현재 패키지 C++ 소스 SHA를 대조한다.
`source_matches: true`를 확인한다. 기존 빌드는 식별 기능이 없어 실패하므로 재빌드한다.
실행 중인 노드의 교체 여부나 의존 패키지 전체의 동일성까지 증명하는 검사는 아니다.
YAML은 별도 `config_id/config_sha256`로 확인하며, 정책 변경을 C++ 코드 변경으로 취급하지 않는다.

시작 로그와 기존 1Hz diagnostics에서 `build_package_source_sha256`, `fusion_views=all`,
`original_tunnel_pipeline=true`를 확인한다. `all`은 선택한 모드이고 세 뷰의 실제 수신/매칭 성공은
기존 `fusion_view_*` 진단으로 확인한다. 상세 debug를 켜지 않아도 빌드 진단은 제공한다.
시작 로그의 실제 실행파일 경로와 config 경로도 함께 남긴다.

빌드·source만으로 기존 프로세스는 교체되지 않는다. 기존 launch를 종료한 후 기본 명령
`roslaunch asmc_lidar lidar.launch`로 다시 실행한다. 같은 namespace에 두 생산자를 띄우지 않는다.
ROS master를 다시 켠 뒤에는 `rosnode list`에 없는 옛 프로세스도 남을 수 있으므로
`pgrep -af lidar_pipeline_node`로 함께 확인한다. 다른 사용자의 프로세스를 일괄 종료하지 않는다.

## 관측 선택과 확인 단위

`MultiCameraFusion`은 각 scan의 공용 Context를 사용해 카메라별 투영 사각형을 한 번 계산한다.
대기열에는 사각형·track ID·수신부의 불변 관측 token만 보관하며, 세 벌의 점군을 복사하지 않는다.
카메라별 대기와 unique 소비는 독립적이다. ±50ms, 최대200ms, IoU0.25,
초기5회/변경3회/미스유예3회와 전방 원본1280×720·좌우640×480을 유지한다.
실제 receive header를 사용하며 서로 다른 source clock을 직접 차감하지 않는다.

확인 단위는 **track + scan**이다. 같은 단위의 세 양성 관측도 한 번만 확인한다.
늦은 뷰는 그 단위 시작 상태에서 재집계하고, 더 새로운 단위가 이미 적용됐다면 과거 결과로 덮어쓰지 않는다.
가시 범위 밖·무효·중단 뷰를 음성 관측으로 세지 않으며, 양성 근거는 다른 뷰의 빈 결과보다 우선한다.
그룹 충돌은 기존 확정 분류와 확인 횟수를 유지한다. vehicle/dynamic 충돌은 공통 보호 자격만 유지하고,
static과 보호 그룹 충돌에서는 새 보호를 보류한다. 같은 그룹에서 원본 이름이 충돌하면 현재 근거에
기존 이름이 있을 때만 유지하고 그렇지 않으면 unknown을 사용한다.

세션/세대 전환 시 대기·소비·현재 양성 근거를 비운다. 보정 identity는 `all`에서
세 카메라의 정렬된 frame/보정 ID를 모두 포함한다. 원본 크기와 보정 ID가 맞지 않으면 해당 관측은 사용하지 않는다.
상세 trace는 실제 matching scan과 발행 scan을 구분한다. 진단 제한은 별도 수량이며,
확정 분류·현재 양성 관측·HD맵 보호 자격은 서로 다른 값이다.
모든 기여 관측원이 무효/중단되면 분류와 확인 상태를 초기화한다.
과거 매칭 token만 만료되고 스트림은 정상인 경우 기존 분류는 유지하되 새 보호 근거는 해제한다. 한 뷰만 무효일 때는
다른 유효 근거를 유지한다. 무효를 미검출 횟수로 세지 않고 LiDAR track·기하는 보존한다.

최종 검토에서는 정본 `cameras_v3.yaml`의 측방 yaw±70도·130도 시야각·원본 해상도로
같은 local 군집이 좌→전방→우로 이동하는 합성 회귀를 추가했다. 뷰가 바뀌어도 같은 track에
scan당 한 번만 확인하며, 늦은 영상은 현재 자세가 아닌 저장된 scan 투영을 사용한다.
대기 scan들이 재사용하는 영상 순서는 호출당 뷰별 한 번만 정렬하고, 소비 여부는 매칭 직전에
다시 확인한다. 보관 token에는 식별자와 불변 record만 복사해 검출 박스를 복사 후 비우는 비용을 없앴다.
이는 ego motion 보정이나 실제 모든 가림·오검출의 무중단 추적을 보장하는 시험은 아니다.

## 터널 연결

이미 도착한 car/bus/truck/person/dog/deer/bicycle/stroller 관측의 실제 점 지지를 합친다.
후단 unique token을 소모하거나 확인 횟수를 올리지 않는다. 보호 표식을 확정 카메라 분류로 복사하지 않는다.
기존 bbox 여유·최근접 깊이 기준·차량6m/사람류1.5m 범위를 유지한다. 새4종은 사람류 범위의 후보다.

M1은 `camera_projection`의 센서 장착·optical 축 변환을 사용한다. 기존 전방 터널 경로는
optical 변환이 없는 별도 수식이었으므로 복구 경로와 구분한다. 이는 새 벽 제거 알고리즘을
도입한 것이 아니다. 벽/천장 판단과 #119의 확인 횟수는 유지한다.

가용성은 뷰별로 계산한다. 관측이 없는 뷰가 보는 점은 ambiguity로 남기되, 다른 유효 뷰가
실제로 보는 점은 전체 카메라 미수신으로 취급하지 않는다. 한 뷰의 수신으로 모든 방향을
검증 완료로 표시하지 않는다. 양성 보호와 무관측 ambiguity는 다르다.

## 터널 연산 최적화

[같은 입력의 전후48회 대조](../../../docs/records/lidar/2026-10-27-m1-tunnel-optimization.md)에서
3뷰 터널 pipeline p95는51.637→8.519ms, 전방 대비 증가4.605ms였다. 카메라 투영보다
두 번의 조밀한 벽 군집화가 병목이었다. 정확한 radius 연결 성분 탐색과 같은 scan의
변경 없는 군집/OBB 재사용을 적용했다. 벽/천장 판단·카메라 미수신 보호·추적·전달 결과는
유지하며, 다음 scan이나 제거된 점 집합에 cache를 재사용하지 않는다.

판제 family 어댑터는3경계에서2차로 중앙선을 유도한다. LiDAR 터널의 입력이 아니며,
이번 replay에는 판제/family 노드가 없었다. 계산 기준 통과는 실제 객체 보존·전체 공급
수락과 별개다. 계산 개선과 사용자 승인으로3뷰만 기본 승격했으며 semantic 보조는 OFF다.

## 웹 표시

- 현재 관측은 실제 `delivery_excluded`와 최종 사유로 표시한다. 지도 SHA 경고가 실제 제외를 되살리지 않는다.
- 예측에는 마지막 실제 관측 판정·시각을 연결한다. 예측 표시를 켜도 해당 제외 숨김을 따른다.
  전달 필터가 켜져 있는데 이력이 없는 예측은 제외 숨김 중 표시하지 않는다. 숨김을 모두 해제하면
  회색 `예측·현재 전달 대상 아님`으로 검수할 수 있다. 현재 위치의 새 지도 판정으로 취급하지 않는다.
  새 제외 사유를 웹이 아직 모르는 경우에도 명시된 `delivery_excluded`를 우선한다.
- 이력은 살아 있는 track만 보관하고 session/generation/config 변경·시간 역행 때 비운다.
- 점군의 기본 색은 **원본 도로 안 초록/밖 회색/지도·pose 불명 노랑**이다. 안쪽10cm 판정은 별도 범례다.
  YOLO 보호 객체의 도로 밖 점도 회색이다. 객체의 보호·전달 색과 점의 위치 색을 혼합하지 않는다.
- 터널 제거점은 터널 체크박스를 따른다. RANSAC 이후 보기에서 초록색으로 우회시키지 않는다.
- 생산자가 보내는 최대20,000개 sampled leveled 점에만 도로 코드를 붙인다. 공용 지도 인스턴스와
  scan pose를 재사용하며 브라우저가 전체 지도와 매 프레임 재계산하지 않는다.
- 현재 제외 수에 예측 이력을 더하지 않는다. audit 전용에서는 비교 후보/발행 유지라고 구분한다.
- 구 생산자가 `delivery_history` 또는 `leveled.road`를 보내지 않으면 화면에 재빌드·재시작 안내를
  표시한다. 없는 점 위치를 초록/회색으로 추정하지 않고 노랑으로 표시한다. 웹 새로고침만으로
  이미 실행 중인 C++ 생산자나 다른 checkout의 정적 웹 파일이 갱신되지는 않는다.

최신 소스의 터널 경로 선택은 `TunnelRoute::Select`의 **map pose 무효** 조건이다.
터널 링크 이름을 확인하는 방식은 아니므로 일반 구간에서도 pose가 무효이면 선택될 수 있다.
`tunnel_filter=true`는 기능 설정이고, 해당 scan의 실제 실행은 `tunnel.ran`으로 확인한다.
같은 scan에서 map 유효인데 터널 단계가 실행된 기록은 실행파일·설정 일치를 먼저 검사한다.
RANSAC 비지면은 객체·가드레일·벽을 모두 포함하며, 점의 초록색은 벽 검출의 정답 표식이 아니다.

## semantic 정적 보조와 판제 인수

`static_obstacle`은 대회 의미 그룹이다. motorcycle도 포함되므로 물리적으로 절대 정적이라고 가정하지 않는다.
정적 보조는 기존 `MotionClassifier` 한 곳에서만 처리하고 원시 KF·속도·OBB·ID는 보존한다.
map pose/속도/형상 유효, 실제5관측, 시간폭0.25–0.6초, 최근350ms 이내 확정 static 양성·무충돌일 때
기하 변위≤2.5km/h가3회 연속이면 LOW_THREAT를 보조한다.

양쪽 OBB 면의 일관된 이동, 누적>0.35m, 보정 변위>2.5km/h, 방향 일치도≥0.7이면
**10km/h를 기다리지 않고 첫 근거 scan에서 보조를 해제**한다.3회 확인하면 DYNAMIC이다.
이미 DYNAMIC인 객체는 static 라벨로 덮어쓰지 않으며 기존8회 정적 복귀를 유지한다.
한쪽 면 확대·형상 불일치·시간/pose 불량은 정적 확인으로 사용하지 않는다.
GPS 무효에서 새 절대 정적 상태를 만들지 않는다. 예측·미수신 프레임은 확인 횟수에서 제외한다.

동일 scan fusion을 사용할 수 있도록 기하 입력 수집과 최종 motion 결정을 나누되 이력은 scan당 한 번 갱신한다.
판제의 기존 LOW_THREAT 예측 속도0 정책은 그대로다. 무효 속도를 유효0으로 만들지 않는다.
웹은 원시 map 속도·기하 변위·적용 상태·판제 적용 속도·보조 사유를 함께 보여준다.
이 후보는 재호의 M3 위치/OBB·M4 연관/KF/속도 개선을 대체하지 않는다.

## 참고 구현과 적용 범위

외부 코드는 workspace `external/baselines`에 보존하며 빌드 의존성이 아니다.

| 참고 | 고정 revision·라이선스 | 읽은 경계와 적용 판단 |
|---|---|---|
| [EagerMOT](https://github.com/aleksandrkim61/EagerMOT) | `2792a76a044250e55e7f25b3e75cd5961366bd33`, MIT | `tracking/data_association.py`: 2D 연결·3D 추적의 분리. 전체 tracker/대표 면적 뷰 선택은 이식하지 않음 |
| [Autoware Universe](https://github.com/autowarefoundation/autoware_universe) | `bafa06574d97e6255bc7f5d8c36683574fa07ebb`, Apache-2.0 | image_projection_based_fusion의 `camera_projection.cpp`, `fusion_collector.cpp`, `fusion_matching_strategy.cpp`; multi_object_tracker의 processor 경계. ROS2 의존성·모든 뷰 대기 구조는 이식하지 않음 |

M1 코드는 기존 ASMC 투영·greedy IoU·분류 정책을 확장했다. 외부 소스의 수학을 새 Python
운영 구현으로 복제하지 않았다. 논문 benchmark를 우리 시스템의 성능으로 보고하지 않는다.

## 재현 도구

- `tools/perception/audit_three_view_fusion.py`: 등록 run의 원본 SHA 검증, 기존/전방/3뷰/semantic 교차3회 native replay.
- `tools/perception/m1/match_gallery.py`: native bbox와 정확한 RGB header/sequence만 연결. 이미지 크기도 검증.
- `tools/perception/m1/ros_replay_check.py`: 독립 master의 실제 ROS 발행·수신·debug·브라우저. 차량 제어 없음.

시험 실행파일은 대응하는 공유 라이브러리와 함께 실행한다. 별도 빌드에서
`LD_LIBRARY_PATH=<candidate-build>/devel/lib:$LD_LIBRARY_PATH`가 빠지면 기존 overlay의 옛 라이브러리를
읽을 수 있다. 원본 bag·완료 결과는 덮어쓰지 않는다. 저장 bag 결과는 새 MORAI 동시 부하 수락과 다르다.

- `tools/perception/m1/tunnel_benchmark.py`: 군집화 전후48회 strict trace 동등성·세부 시간·RSS 대조.
