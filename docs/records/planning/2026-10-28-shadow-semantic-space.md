# S3 — 공유 IMU 주행가능영역과 family 기반 local 경로 실험

2026-10-28, PR #126의 opt-in 후속 실험이다. PR #125는 접점/local 연결과 반복 실험
기반이고, #126은 그 위에 쌓인 회전 footprint·BEV 검사와 이번 S3다. 두 PR은 병합하지
않으며 운영 기본은 변경하지 않는다. [S2 당시 결과](2026-10-28-shadow-footprint-bev.md)의
실패·제어 미확인 기록은 보존한다.

## 구현 범위

- 기존 `CameraGroundProjection`의 세 뷰 원본 stamp·보정 ID·IMU 평면을 그대로 사용한다.
  별도 IMU 보간·family 피팅·추론·RGB/JPEG 경로를 만들지 않는다. 원본 확률 영상만 투영한다.
- `ComparisonProjection`의 기하 코어를 공유한다. 원본 전방1280×720·측방640×480에
  family가 실제 적용한 평면을 한 번 적용한다. M0 fallback 프레임은 이 실험에서 사용하지 않는다.
- 세 확률 영상의 정확한 시각·세대·설정·보정을 확인한다. 강한 중첩 뷰의 비노면 관측을
  평균으로 지우지 않는다. 확률166/255, 뷰 품질0.25는 이번 비교 설정이며 보정된 확률이 아니다.
- private `/experimental/shadow/semantic_space_trial/{drivable_probability,coverage,context}`만
  추가한다. 공개 메시지/MD5와 기본 fixed-plane BEV는 유지한다. ROS publisher별 `header.seq`는
  독립적이므로 서로 같아야 한다고 검사하지 않는다. exact stamp와 생산자 epoch로 대응한다.
- family·영상 확률·local LiDAR의 **가장 최신 완성된 묶음**을 선택한다. 새로운 부분 묶음이
  아직350ms 이내인 완성 묶음을 지우지 않는다. family–BEV는 exact, local과는50ms 이내다.
  source/receipt350ms와 세대 cutoff는 유지하며 과거 XY 경로를 현재 경로로 재발행하지 않는다.

`bev_mode=semantic_trial`에서는 YOLO bbox의 차로 분류 성공이 주행 전제가 아니다.
family의 실제 연속 support 안에서 두 차로 중심 주변의 lateral offset·quintic 경로를
생성하고, **회전된 차체 전체+기존 여유**를 확률 grid 및 실제 관측 local footprint와 검사한다.
기존 제어기와 control selector 하나를 사용한다. 이강태의 별도 YOLO 개선 경로를 변경하지 않는다.

- 기준 차로를 기억하되 좌표는 매 프레임 다시 계산한다. 현재 관측에서 본차로 복귀 경로가
  **전체 현재 지원 지평**에서3번 확인되면 복귀 후보를 선택한다. 장애물 앞의 짧은 빈 prefix만으로
  복귀하지 않는다. 가림 뒤의 복귀점을 미리 만들어야 출발하는 구조는 아니다.
- 최대 rear-axle 목표15m이며 실제 family/grid 경계에서 차체 전방·횡 여유를 뺀 범위만 쓴다.
  유효 support20m가 rear-axle 경로20m라는 뜻은 아니다.25m 외삽은 하지 않는다.
- 긴 경로가 없으면8/5/3m의 관측된 prefix를 검사한다. 본차로의 긴 경로가 실제 비노면이나
  local 객체로 막혔다면 옆 차로 prefix를 먼저 검사한다. unknown만으로 차로를 바꾸지 않는다.
  명령5km/h 상한·실제 속도 확인을 유지한다. 내장 속도 제어의 순간 초과까지 없앤 것은 아니다.
  제동거리 계산의1m/s²는 실험 가정이며 실제 정지거리 인증으로 해석하지 않는다.
- 관측된 local OBB는 class와 무관하게 경로를 거부할 수 있다. 예측 OBB는 새 측정으로 세지 않는다.
  GPS 무효를 map 빈 배열로 바꾸지 않으며 map pose를 만들지 않는다.

## 관측 불가와 근거리 보조의 한계

`coverage`는 private 실험에서255=관측,128=원본 차체 mask 가림,0=유효 카메라 지지 없음이다.
확률이 낮은 관측255는 장애물/비노면/가림 그림자를 포함한다. 이를 정확한 장애물 길이로
해석하지 않으며, LiDAR 빈 결과로 지우지 않는다.

실제 영상에서 본네트·시야가 전방약4–6m를 가린다. 순수 semantic은 이 구간을 새로 사용할 수
없어 추가 장애물 없는 기준 시험도 출발하지 못했다. 원본 RGB에 역투영한 r04의 실패 셀은
차체 mask 바깥2–3pixel의 추론/보간 경계였다. 실험 visibility에만4pixel 제외 여유를 적용했다.
YOLO mask 정본·모델 출력·차선 피팅은 바꾸지 않았다.

`ASMC_SHADOW_NEAR_EGO_LIDAR=true`는 별도 미수락 후보다. fresh local 입력과 모든 실제 OBB
검사를 전제로, 제한된 차량 근거리 가림을 LiDAR 보조에 맡긴다. r06의946장 가림 투영 전방
범위는 중앙6.0m·p95 6.275m·최대6.3m였다. 후보 상한은6.5m, 실제 ego-mask128의 좌우는
2.5m(기록된2.3m+2cell), 일반 시야 부재0은1.6m다. 후방은 차체+기존 여유 범위뿐이다.
가림 셀 방문 횟수와 적용 여부를 diagnostics에 명시한다.

**이는 LiDAR 빈 배열이 공간을 무장애로 증명한다는 뜻이 아니다.** 얇은 물체·누락 OBB가
근거리 가림에 있으면 놓칠 위험이 남는다. 따라서 이 보조와 S3 전체는 운영 기본으로 승격하지
않는다. 추가 근거리 ground/ray visibility 근거 또는 검증된 관측 정렬 없이 unknown을
전역적으로 free로 바꾸지 않는다. 관측255의 낮은 확률, 먼 unknown, 잘못된 pose/시각은 계속 거부한다.

## 시험 및 재현

원태의 시나리오 생성/재생기와 기존 ROS recorder를 사용한다. 센서·family·local tracks·
제어 명령 외에 experimental grid/context도 저장한다. 각 run은 고유 ID로 등록했으며,
소스·설정·장면 SHA와 실제 actuator readback·native pose·contact 조회를 보존한다.
단순 완료된 bag이나 출발 준비를 충돌 없는 통과로 계산하지 않는다.

객체가 없는 기준 장면은 `asmc.section_trial`의 명시적 `allow_empty_scene: true`에서만
허용한다. 기존 collision archive의 baseline 누락은 계속 오류다. 생성·release 양쪽 준비
판정을 일치시켰다. 사용자 기존 actors는 보존하고 시험 fixture29만 임시 이동 후 복구한다.
따라서 '추가 장애물 없음'은 터널 전체에 사용자 객체가 없다는 뜻이 아니다.

제어 적용은 simulator UI의 실제 UDP9093 연결을 확인하고 전체/actor pause와 AUTO/P/D를
정리한 뒤 실제 brake1·steering readback으로 확인했다. 네트워크 JSON·bridge 코드는 변경하지
않았다. 이전 문제의 단일 원인을 UDP 부호나 LiDAR로 확정하지 않는다.

결과 정본은 D의 `artifacts/shadow-semantic-s3-20261028-audit01/index.html` 및 각 run이다.
**실제 회피·통과 수락은 실패, S3와 근거리 보조는 기본 OFF를 유지한다.** 모든 신규 주행은
FOGGY13시다. 약6분30초의 `driving_ready → stop_requested` 시험창이며, 준비 실패·정차 검사·
복구 시간은 별도다. 준비 실패까지 포함해 r01–r11을 보존한다. 반복별 정책과 장면이 다르므로
동일 구성 다회 통계나 전체 터널 완주 성공률로 계산하지 않는다.

| run 끝번호 | 구성·장면 | native 시작→끝 변위 | 결과 | 경로 계산 p95 / p99 |
|---|---|---:|---|---:|
| r01–r02 | 명시적 빈 장면 준비 | 주행 없음 | 준비 조건 결함 수정 후 재시험 | 해당 없음 |
| r03–r04 | S3 strict / 초기 근거리 보조, 추가 장애물 없음 | 출발 없음 | 근거리 시야·mask 경계 거부 | 3.42 / 5.55, 3.48 / 5.79ms |
| r05 | S2 접점+local, 추가 장애물 없음 | 64.30m | 실제 전진 확인; 기존 사용자 객체 부근 정지 | 4.65 / 6.69ms |
| r06 | S3 4px mask·6m 보조, 추가 장애물 없음 | 48.06m | 시작/정지 반복, 가림 상한 측정 | 5.30 / 8.62ms |
| r07 | S3 6.5m 보조, plastic barrier | 17.29m | 옆 mask wing 거부·정지 | 6.25 / 9.07ms |
| r08 | S2 접점+local, 같은 plastic barrier | 9.66m | 장애물 전 정지 | 1.45 / 2.38ms |
| r09 | S3 가림 wing 분리 | 16.69m | 차로 변경 시작, 최종 통과 실패 | 5.30 / 7.17ms |
| r10 | S3 빠른 prefix 회피·전체 지평 복귀, plastic barrier | 17.84m | 차로 변경 시작, 최종 통과 실패 | 5.27 / 8.05ms |
| r11 | r10 구성, drum | 17.18m | 차로 변경 시작, 최종 통과 실패 | 10.09 / 12.76ms |

변위는 약1Hz native 표본의 시작·끝 거리다. 실제 경로 길이나 장애물 통과 거리가 아니다.
S2의 관측 최고 속도는 r05 6.90·r08 7.13km/h로 명령5km/h를 초과했고, S3 최종 r10/r11은
5.11/5.23km/h였다. 따라서 기존 속도 제어·제동 수락을 완료로 표시하지 않는다.
표본 native contact 조회에서 충돌 ID는 없었지만 연속 접촉 검증은 아니며, ROS CollisionData의
해석 불가 표본도 결과에서 보존한다. 충돌 없는 완주로 보고하지 않는다.

최종 r10/r11의 bag 전체 수신 공급은 다음과 같다. 준비·복구를 포함한 통계이며 완전한5초
정상 주행창의 수락 판정과 구분한다.

| 입력/출력 | r10 Hz / 최대 공백 | r11 Hz / 최대 공백 |
|---|---:|---:|
| family | 14.01 / 132ms | 13.90 / 157ms |
| local tracks | 12.10 / 138ms | 12.19 / 133ms |
| experimental semantic grid | 9.99 / 128ms | 9.99 / 125ms |
| 최종 ctrl_cmd | 20.02 / 58ms | 20.03 / 61ms |

### 마지막 정지의 입력별 재현

r10 `semantic-stop-fixture`의 정확한 family/grid/plane과50ms 이내 local 관측을 native
`contact_path_probe`에 넣었다. 후보336개 중 도로/곡률193, semantic109, local OBB34개가
거부됐다. 이는 정지 표본의 상태를 초기화해 수행한 결정 재현이며 전체 제어 이력 재생은 아니다.

- semantic만 free로 바꾼 **오프라인 원인 분리**: local OBB가 나머지143개 후보를 거부.
- local 객체만 비운 원인 분리: 원래 semantic으로9m 경로 후보가 나옴.
- 두 입력을 비운 결과는 온라인에 적용하거나 발행하지 않는다.
- semantic에 통과한 대표 후보의 local 거부 객체는 track9, `plastic_barrier`,
  중심 `(4.917, -2.258, 0.356)m`, 관측50점, OBB `1.305×0.106×0.353m`였다.
  원래8코너의 XY convex hull·0.25m 불확실 여유와 차체 sweep가 겹친다.
- 따라서 이 장면을 단순 벽 오검출, YOLO class 실패,3뷰 지연 탓으로 설명할 근거는 없다.
  관측 OBB가 물체 전체를 정확히 복원했다는 독립 GT 주장도 하지 않는다.

실패 직전 RGB·semantic·family와 거부 지점은 r09/r10/r11 `semantic-analysis/index.html`에서
같은 source로 확인한다. 과거 marker 선이나 가장 가까운 영상으로 대응을 만들어내지 않았다.

### 수정한 재현 결함

1. 별도 publisher의 seq 일치 요구를 없애고 exact stamp·epoch 계약으로 대응.
2. 새로운 부분 입력 때문에 아직 fresh한 완성 입력을 버리던 선택 경계 수정.
3. ego mask와 시야 밖을 구분하고 원본2–3px mask 경계의 모델 투영 오염을 실험4px 여유로 제외.
4. 명시적 빈 장면의 준비/release 조건을 일치시켜 baseline 재생 가능.
5. 장애물 앞 짧은 직진 prefix를 반복 선택하거나 이를 본차로 복귀 근거로 쓰던 정책 수정.
6. 실패 diagnostics에 현재 사용 입력 stamp를 기록해 마지막 성공 프레임과 혼동하지 않도록 수정.

카메라 비교 영상·추론·family 피팅 정본, LiDAR 검출·추적·필터, 공용 메시지와 운영 launch
기본값은 유지한다. 실행 시험은 `cde4166a`, 이후 커밋은 probe·회귀·결과 정리다.

## 최종 코드 검사

- 컨테이너 `scripts/build_ws.sh -j4 -l4`: 전체 workspace 빌드 통과.
- 독립 master11331에서 `.github/ci/lane.sh`: C++26개·ROS61개 통과.
  semantic 무YOLO 입력, exact source/epoch, partial 회피/전체 지평 복귀 회귀를 포함한다.
- 카메라 BEV216개·runtime123개 통과. 기존 comparison/render 경로 회귀 포함.
- 시나리오 unit232개·collision archive/replay29개·replay UI9개 통과.
- 문서 이전 검사 및26launch/87개 명령 검사 통과. `git diff --check` 통과.
- hosted CI 결과는 push 이후 별도 확인한다. 로컬 통과를 hosted 통과로 표시하지 않는다.

시험 코드와 컨테이너 바이너리를 맞췄다. 시험 planner/LiDAR/semantic producer와 독립 master는
종료했고 기존 bridge/camera/rqt는 보존했다. 기존 actor·Ego 위치·날씨/시간과 초기 전체 pause를
복구했다. native reset 후 물리 settle에 따른 cm 수준 위치 차이는 복구 기록에 남는다.
시뮬을 직접 사용할 때는 pause를 해제해야 한다.

## 실행·복구

사용자 운영 bridge/camera/view는 유지한다. 동일 차량의 다른 autonomy를 중복 시작하지 않는다.
등록한 local 시작 run은 `readiness_profile: shadow_local`을 사용하며 실제 GPS0·fresh family/local·
정차·검증된 음영 제어 명령을 요구한다. 기본 준비 프로필은 여전히map이다.

```bash
ASMC_SHADOW_SEMANTIC=true ASMC_SHADOW_NEAR_EGO_LIDAR=true \
ASMC_SHADOW_CONTACT_CLEARANCE=true ASMC_SHADOW_LOCAL_CONTACT=true \
ASMC_SHADOW_APPROACH_CAP=true ./tools/scen_runner/run_planning.sh
```

위 환경 없이 시험 supervisor를 다시 시작하면 기존 정책으로 복구한다. producer만 살펴볼 때는
`roslaunch tools/scen_runner/launch/semantic_space.launch`이며 이 노드는 제어하지 않는다.
확률/coverage를 보기 위해 별도 비교 영상을 인코딩하지 않는다.

```bash
python3 -m scen_runner.records.analyze_shadow /artifacts/<run-id> --allow-incomplete
python3 -m scen_runner.records.analyze_semantic /artifacts/<run-id>
```

분석 출력은 새 하위 디렉토리만 만든다. 기존 결과를 덮어쓰지 않는다. semantic gallery는 새
`semantic_input_stamp_ns`와 정확히 일치하는 family/grid/context/RGB만 보여준다. 이 지표가
없는 과거 revision의 실패를 가장 가까운 영상으로 대체하지 않는다. frame 그림의 family는
실행 궤적이 아니며, 선택된 경로와 제어 결과는 diagnostics/native 기록으로 별도 검수한다.

## 설계 참고와 다음 판단

[Autoware Path Sampler](https://autowarefoundation.github.io/autoware_universe/main/planning/sampling_based_planner/autoware_path_sampler/)의
후보 생성→충돌/곡률/주행영역 제약→선택 분리와
[Autoware occupancy grid](https://autowarefoundation.github.io/autoware_universe/latest/perception/autoware_probabilistic_occupancy_grid_map/)의
관측 불가 구분을 참고했다. ROS2 의존성이나 전체 알고리즘을 이식하지 않았고 성능을 재현했다고
표시하지 않는다. lane family가 도로 기하, semantic이 관측된 공간, LiDAR가 실제 기하 근거를
제공하도록 책임을 나눈 실험이다. 실제 회피·복귀, 가까운 얇은 물체, 동적 객체와 제동거리 수락이
남으면 기존 정책을 유지한다.


다음 작은 단위는 **기록된 r10/r11을 이용한 회피 시작 시점·유한 후보와 폐루프 추종의 대조**다.
semantic 구멍과 장애물 뒤 unknown을 분리하면서, 실제 OBB 여유를 통과할 수 있는 경로를
충분히 일찍 만드는지 확인한다. 매 프레임 동일한 변경 완료 거리를 다시 설정하는 영향,
짧은 prefix의 추종 오차, 현재 차체/곡률 제약을 각각 비교한다. 단순히 OBB veto나 안전 여유를
끄거나 unknown을 free로 바꾸지 않는다. 근거리에는 별도 ground/ray 관측 근거가 필요하며,
그 전까지 LiDAR-negative 보조는 미수락 상태다. IMU-only 적분·25m 외삽·새 네트워크를 먼저
추가하는 단위로 확대하지 않는다.
