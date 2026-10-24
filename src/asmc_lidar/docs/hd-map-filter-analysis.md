# A1 — OBB 기반 HD맵 전달 필터 분석

2026-10-24. 담당 안승현. PR #104 병합 main `6927fe9d`를 기준으로 한다.
A1은 기존 필터 감사와 **삭제 없는 오프라인 판정 비교**다. 아래 A1.1은 실행 경로의 판정·진단을 다룬다. 운영 Main·판제 입력,
기본 ROI·검출·OBB·tracker·fusion을 변경하지 않는다. 점군 HD맵 ROI는 보류한다.

## 실제 호출 경로와 소유권

| 경로/기능 | 실제 동작·기본값 | 다음 이관의 소유권 |
|---|---|---|
| Main `PlannerObjectRosPublisher::Publish` | map 객체 → road/known-static/structure → relevance → 필요 시 OBB clipping → `/last_lidar` | 지도/측정 품질 판정은 인지. 원본 추적 상태와 최종 전달을 분리 |
| `planning_inputs` 기본 `validated_lidar` | `/last_lidar` + 같은 source stamp의 `/lidar/motion_debug`에서 ID/최종 keep/운동 상태를 대조한 뒤 변환. road 필터를 다시 실행하지 않음 | 현행 wire gate 유지. 다음 공용 출력에서는 진단 JSON 의존 제거를 별도 검토 |
| `planning_inputs` 선택 `main_raw` | private motion classifier → road/known-static/structure → clipping → `/planning/objects` | Main과 일부 구현 중복. 선택 입력에 따라 정책·기하가 달라짐 |
| road footprint | threshold 0.15, margin 0m, supersample 5. 보행자 보존. motion이면 0.5초×6회 map 속도 예측으로 진입 검사 | 인지 관련성. 새 후보는 8코너 footprint 사용·OBB clipping 금지 |
| known-static | UNKNOWN·확정 클래스 예외, cluster center와 catalog 거리 ≤2.5m. 검증 영역 `[-35,15]×[108,180]`, `[-107,15]×[175,345]`. UNKNOWN이면 motion label이 있어도 catalog 일치가 우선 | 인지. catalog ID는 tracker ID의 영구 정체성으로 해석하지 않음 |
| structure exclusion | UNKNOWN·비이동 근거, 지정 zone과 높이≥2.5m 또는 하단 z≥2m. 일부 zone은 형상 조건 없음 | 인지. 지도 전체의 일반 장애물 삭제 규칙으로 확대하지 않음 |
| Main relevance | 높이 0.10m·지면 근거·3회 확인·5회 release·20frame 이력 정리·camera 의미 예외, 자차 속도 기반 emergency 거리 override | 측정 근거와 행동 정책이 섞임. emergency/TTC·추종·회피를 통째로 인지에 옮기지 않음 |

호출 근거: Main `src/PlannerInterface/PlannerObjectRosPublisher.cpp`, `main.launch`;
`planning_inputs/src/objects_node.cpp`, `src/filters/PlanningObjectFilter.cpp`,
`scripts/tracked_wire_bridge.py`, `launch/objects.launch`.
`main_raw`는 필터 전 데이터가 자동으로 생긴다는 뜻이 아니다. 같은 `/last_lidar`를 입력하면
이미 Main이 삭제한 객체는 복원되지 않고 추가 필터가 적용될 수 있다.
road/known-static/structure 세 구현은 include 경로를 정규화하면 Main과 planning_inputs가
동일하다. catalog 두 사본도 동일 SHA `2d8fe6e5b47215e85fbe05f03b552e06096499f0bba43e1a54166245ab4242a1`
(5,912관측)다. 그러나 Main의 relevance와 입력의 motion 상태가 달라 전체 실행 경로가
동일하다고 해석하지 않는다.

## 지도·예외 구간·위치 유효성

- 기존 PNG는 흰색이 도로, v6 파생 OccupancyGrid는 0이 도로다. 원점/map ID가 같아도
  형상은 다르며 v6 vector가 정본이다. 비교 도구의 겹침 수치는 파생 grid 기반 근사로 표시한다.
- 정적 장애물 `A213/A222/A220`, 동적 장애물 `A202/A728/A599/A207/A205/A315`,
  로터리 `A148/A146/A151`은 현재 팀 metadata의 예외 링크다. v6의 maneuver/support
  link도 함께 보호한다. metadata의 mission polygon은 비어 있으므로 외부 접근 영역의
  완전한 검수가 끝났다고 표시하지 않는다. 판제 설정과 차이는 결과에서 별도 보고한다.
- 일반 구간 밖에서 객체가 들어오지 않는다는 것은 사용자 제공 대회 운영 전제다.
  Ego 링크만으로 예외를 켜지 않고, 객체 footprint가 예외 공간에 접근하면 보존한다.
- GPS 5Hz/pose age≤300ms는 위치 정확도 보장이 아니다. 새 후보의 제외에는 명시적으로
  검증된 위치 오차 범위와 구간 검토 근거가 필요하다. 근거가 없는 기존 trace는 판단 불가다.
  IMU 자세만으로 병진 이동이 해결됐다고 가정하지 않는다. GPS 무효 시 local은 보존한다.

## 비교 도구와 근거

**상태: A1 조사·오프라인 검증 완료, 실제 전달 필터 적용 보류.** 실행 코드는 `845a0cda`,
최종 run은 `lidar-hd-map-filter-audit-20261024-r2`다. D 정본의
`artifacts/lidar-hd-map-filter-audit-20261024-r2/`에 `summary.json`, `decisions.jsonl`,
`index.html`·SVG 갤러리, `coverage.json`, `verification.json`, `evidence-manifest.json`을
작성 후 동결했다. 갤러리의 파란 영역은 v6 vector, 실선은 원본 8코너 footprint,
파란 점선은 기존 필터가 clipping한 형상이다. 지도 +X가 오른쪽, +Y가 위다.
원본 RGB나 독립 GT 영상으로 오인하지 않는다.

전체 혼합 갤러리는 대표48개다. **세 미션 예외·GPS·경계 합성 사례 전체**는 별도 동결 run
`artifacts/lidar-hd-map-filter-cases-20261024-r1/index.html`에35개 SVG로 제공한다.
36사례 중 잘못된 형상1개는 판정 JSON에만 남고 그리지 않는다. `case-labels.json`은
SVG와 fixture 사례 이름의 대응표다. 이 갤러리는 합성임을 표시하며 실제 미션 자료를 보강한
것으로 세지 않는다. 저장소의 작은 [결과 포인터](../../../config/baselines/lidar_hd_map_filter_audit_result_v1.json)에
두 run ID와 동결 manifest SHA를 기록했다.

| 입력/결과 | 수량·해석 |
|---|---|
| 실제 자료 | 기존 동결 bag 두 개의 857scan을 pitch/roll-pitch 및 GPS normal/invalid/missing 조건에 재사용한 trace 8개, 3,428scan. 반복 재생을 독립 장면으로 세지 않음 |
| 실제 객체 관측 | 50,635개: 유지 38,101 / 판단 불가 12,534 / 제외 후보 **0**. 모든 객체를 결과 JSONL에 보존 |
| 합성 회귀 | 36개: 유지 21 / 판단 불가 8 / 제외 후보 7. 필요한 객체로 지정한 합성 사례의 제외 후보 0 |
| GPS 정합 범위 | 실제 normal 자료의 GPS 표본 간격 중앙값 200.4/201.5ms, scan 시각 GPS age p95 191.7/196.4ms. 최대 12.96m/s·조향 포함. 이를 위치 오차 측정으로 해석하지 않음 |
| 실제 구간 한계 | 최근접 v6 centerline은 A448/A126/A128. 정적·동적 장애물·로터리의 실제 외부 진입은 미검증. 이 최근접 조회는 판제의 정식 mission matching이 아님 |
| 지도 차이 | 공통 drivable 352,857셀 / 기존 PNG만 2,325,594셀 / v6만 20,851셀. 해상도 0.2m·grid 정합 확인. 전체 도로와 대회 경로 제품의 범위 차이도 포함하므로 어느 쪽이 더 정확하다는 지표가 아님 |
| 새 후보 오프라인 비용 | 객체당 p50 0.543ms / p95 1.823ms / p99 2.877ms. 별도 legacy 실행기·파일 처리까지 포함한 scan 비용과 분리. 실시간 공급 수락/군집 계산량 절감으로 표현하지 않음 |

과거 trace에는 최종 Main motion state·적용 속도와 위치 오차 보장이 없다. 실제 keep/drop을
확정 복원했다고 표시하지 않는다. **실제 자료 중 42,921관측의 legacy 비교는 속도 0·비이동을
가정한 counterfactual static probe**이며 출력에 그 scope를 기록했다. 나머지는 비교 입력 부족이다.
합성 DTO가 완전한 경우에만 기존 Main의 relevance까지 실행한다. 속도 validity 미기록을
측정된 0/static으로 바꾼 새 후보는 없다.

새 후보는 위치 오차 범위·구간 검토가 검증되지 않으면 제외 후보를 내지 않는다.
설정의 `candidate_guard_m=1.0`, `exception_protection_m=10.0`은 **오프라인 보존 후보**이며
측정으로 승인된 운영 여유가 아니다. metadata의 비어 있는 mission polygon을 검수 완료로
채우지 않았고, 미션 전체 링크·검토된 maneuver 링크를 공간상 넓게 보호한다.
동적 구간의 판제 behavior 설정은 A728/A599/A207/A205, metadata는 준비·복귀를 포함해
A202 중간–A315 30%다. 이 차이는 역할에 따른 범위 차이이며 임의로 동일하게 줄이지 않는다.
로터리는 판제 source A148 / target A146 / conflict A134·A151을 확인했다.

### 재현과 실패 보존

`r1`은 판정·summary 생성 후 legacy clipping 형상을 그릴 때 OpenCV의 비연속 XY 배열
오류로 실패했다. 원본 결과와 `failure.json`을 동결해 보존했다. 렌더링 수정·회귀 검사 후
새 ID `r2`로 전체를 재실행했다. 두 실행의 수치 판정 SHA는 동일하다.

```text
stable_decisions_sha256:
f9def17297a6b1d7d962c8016561b48f71f1106ca1bffd67e9b44f3fbbe26c42
```

SHA 계산은 측정 시간·서로 다른 run ID를 제외하며, 객체·좌표·판정·사유·비교 scope를 포함한다.
입력 파일의 실행 전후 SHA와 메모리 내 객체 불변성을 확인했다. 원본 객체 SHA도 두 실행이
동일하다. source stamp·session/generation/sequence·파일 SHA·행 번호로 원본 trace에 대응한다.
normal A126 trace는 유효 GPS로 재생한 개발 자료이며, invalid/missing은 명시적 fault 변형이다.
이를 실제 대회 음영구간의 GPS 가용성으로 설명하지 않는다.

### 직접 검사·실행

저장소의 단위검사와 legacy 실행기만 사용하며 ROS master·센서·GPU·GUI를 실행하지 않는다.
Main 필터의 adapter include만 build 사본에서 portable DTO include로 좁혔고 계산 본문은
원본 그대로 컴파일했다. 이 실행기는 저장소 비교 도구이며 catkin 설치·export 의존성은 없다.

```bash
# Noetic 컨테이너의 /root/ws
cmake -S tools/perception/lidar_filter_audit -B /tmp/lidar-a1-probe-build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/lidar-a1-probe-build -j2
python3 src/asmc_lidar/test/test_hd_map_filter_audit.py
python3 tools/perception/lidar_filter_audit/test_legacy_probes.py /tmp/lidar-a1-probe-build
```

새 실험은 PC의 저장 규약으로 고유 run을 먼저 등록하고, 컨테이너에서 **같은 ID**를
`ASMC_AUDIT_RUN_ID`에 지정한다. host env.sh를 컨테이너에서 source하지 않는다.
아래 합성 입력을 실제 자료의 검증으로 표현하지 않는다. 실제 자료는 `--input`을 반복해 추가한다.
기존 출력 디렉토리가 있으면 도구가 거부하므로 완료 run을 덮어쓰지 않는다.

```bash
python3 src/asmc_lidar/tools/compare_hd_map_filters.py \
  --run-id "$ASMC_AUDIT_RUN_ID" \
  --config src/asmc_lidar/tools/filter_audit/audit.yaml \
  --input src/asmc_lidar/test/fixtures/hd_map_filter_audit/cases.jsonl \
  --legacy-bin-dir /tmp/lidar-a1-probe-build \
  --output "/artifacts/$ASMC_AUDIT_RUN_ID"
```

입력은 기존 `asmc_lidar_scan_trace_v1` JSONL 또는 offline `asmc.lidar.hd_map_filter_audit/v1`
fixture다. 후자는 frame/status/map 원점·ID·scan 시각의 pose·선택 근거·객체 목록과
8코너를 제공한다. `position_error_verified`, `position_error_bound_m`, `mission_scope_verified`
등은 **오프라인 검토 근거**이며 공개 ROS 메시지에 추가한 필드가 아니다.
기존 trace에 없는 유효성을 생성하지 않고, map 무효일 때는 local 객체를 판단 불가로 보존한다.

검사: Python core 18개, 합성 36개×2회 수치 재현·두 C++ 체인 비교,
패키지 독립 계약 10개·카메라 연결 계약 5개, 문서/실행 unittest 17개를 통과했다.
실시간 부하·ROS 운영 전환·새 hosted CI는 이 단위에서 수행하지 않았다.

## 다음 구현 경계

필터는 OBB·추적·camera fusion 뒤의 전달 판정에만 연결한다. tracker에 삭제를 피드백하지
않고 원본 중심·8코너·속도·ID를 보존한다. 정상 빈 결과와 센서/pose 무효를 구분한다.
실제 제거·새 공개 객체 출력·판제 전환·기존 중복 코드 삭제는 A1/A1.1 결과 검토 이후 별도 작업이다.

**A1 이후 연결:** 아래 A1.1에서 `/migration/lidar`의 전달 **판정만** 계산하도록 연결한다.
원본 객체/추적은 보존하고 실제 제외 모드는 두지 않는다. 동시에 scan 시각 pose의 검증된
위치 오차와 세 예외 공간의 외부 접근 범위를 확보한다. 현재 자료만으로 여유를 확정하거나
메타데이터의 `verified`를 자동 승격하지 않는다. 이 두 근거가 부족하면 제거 활성화를 보류한다.


## A1.1 실행 연결과 진단

구현 소스 `ca9ac202`. runtime의 **fusion → 최종 local/map 배열 구성 직후** C++ 코어를
호출하고 원본 배열을 그대로 발행한다. 계산은 epoch 잠금 밖에서 수행하며 기존 세대 검사
안에서 발행한다. tracker에 제거를 피드백하거나 OBB를 자르지 않는다. 새 공개 타입·노드·
launch 옵션은 없다. 기본 `hd_map_audit.mode: "off"`, 내부 시험만 `audit`다.
기존 schema1 설정에 새 블록이 없으면 OFF로 읽어 과거 재현 설정을 유지한다.

`prepare_hd_map_audit.py`는 SHA 검증한 v6 NPZ·미션 공간을 row-major int8/uint8 배열과
manifest로 내보낸다. vector가 정본이며 이 bundle은 판정 계산용 파생 입력이다. 원본 SHA,
원점·해상도·크기, 정책 canonical bytes·SHA와 각 배열 SHA를 기록한다. C++는 manifest의
기대한 SHA와 **실제로 메모리에 읽은 배열**을 검증한다. 준비 완료 후에는 읽기 전용으로
공유해 reset 때 재로딩하지 않는다. 프레임마다 파일·Python을 호출하지 않는다.
지도 누락·해시 오류는 audit만 사용 불가로 표시하며 local/map 기존 발행을 유지한다.
`off/audit` 이외의 모드는 설정 오류다. 실제 제외 모드는 제공하지 않는다.

파생 bundle은 동결 run `lidar-hd-map-audit-bundle-20261024-r1`이다.
manifest SHA는 `1c250f4192ea21babefd71c64eaacaa7f0917b4a9251ba447759b66ed26d4c97`다.
다른 PC에서는 해당 지도 정본으로 새 등록 run에 다시 export하고 새 manifest SHA를 설정한다.
개인 D 경로를 패키지 기본값으로 고정하거나 이전 run을 덮어쓰지 않는다.

```bash
# 저장 규약으로 AUDIT_RUN_ID/출력 위치를 등록한 뒤, 컨테이너 /root/ws에서 준비.
python3 src/asmc_lidar/tools/prepare_hd_map_audit.py \
  --config src/asmc_lidar/tools/filter_audit/audit.yaml \
  --output "/artifacts/$AUDIT_RUN_ID/bundle"
```

실험용 runtime YAML은 기존 `lidar_pipeline.yaml` 전체를 복사해 아래 블록만 바꾼다.
실험 경로의 YAML에 쓴 manifest SHA와 실제 적용 설정 SHA를 run 근거에 보존한다.

```yaml
hd_map_audit:
  mode: audit
  manifest: /artifacts/<등록한-run-id>/bundle/manifest.json
  manifest_sha256: <준비 도구가 출력한 SHA>
```

```bash
# 기존 처리와 대안으로 실행하며 동일 namespace의 두 생산자를 켜지 않는다.
roslaunch asmc_lidar lidar.launch pipeline_config:="$AUDIT_CONFIG"
rostopic echo /migration/lidar/diagnostics
```

1Hz diagnostics는 최근100scan의 판정·사유별 수량, 계산 p95, 지도 오류를 제공한다.
기존 trace는 수요가 있을 때만 선택 블록을 직렬화한다. session/generation/scan sequence와
객체 ID로 원본 배열에 대응하고 제품/정책 SHA를 남긴다. 지도 오류·정상 빈 결과·입력/pose
무효를 구분한다. 진단과 갤러리는 판제 입력이나 실제 제외로 해석하지 않는다.
[trace 형식](scan-trace-schema.md#선택적인-hd맵-audit와-발행-지연)을 함께 참고한다.

A1 Python과 C++의 합성36사례와 실제 trace31,801관측에서 판정·사유·격자 비율이 정확히
일치했다. 실제 자료는 GPS normal/invalid/missing 변형을 포함하며 독립 장면 수가 아니다.
runtime 검사는 OFF/ON 객체 배열의 ROS 직렬화 바이트, GPS 장애/복귀·빈 결과·지도 오류·
reset 후 map 공유와 상태 초기화를 확인한다. 공개 속도 validity나 위치 오차 근거는 승격하지 않았다.

OFF/ON 비교 도구 `tools/filter_audit/check_runtime.py`는 고정 bag을 각각3회 교차 replay하고
공개 객체·tracker digest, 계산 p95·RSS를 기록한다. unpaced replay 처리율은 실시간 공급률이 아니다.
`check_runtime_ros.py`는 독립 master에서 네 입력 토픽만 실제 속도로 재생하고 callback→publish,
소비자 수신 공백·5초 창 공급·RSS를 기록한다. 자기 master/node/player만 종료하며 차량 제어를
발행하지 않는다. 새 run ID를 전달하며 입력 bag·설정·소스 SHA를 등록해야 한다.

실제 제외를 검토하기 전 **검증된 scan 시각 위치 오차와 세 예외 공간의 외부 접근 범위**가
추가로 필요하다. 현재 GPS age와 비어 있는 mission polygon만으로 승인하지 않는다.


### A1.1 검증 완료 — 2026-10-24

근거 run `lidar-hd-map-runtime-20261024-r1`의 `verification.json`, `evidence-manifest.json`,
`paired/`, `paced/`, `validation/`, `gallery/index.html`을 보존한다.
[작은 결과 포인터](../../../config/baselines/lidar_hd_map_runtime_audit_result_v1.json)에 SHA와 경로를 기록했다.

| 비교 | 결과 |
|---|---|
| 필수 빌드/회귀 | CTest32개·필수 gate 통과. audit ON 별도 ROS17사례 통과 |
| 같은 bag OFF/ON 각각3회 replay | 각649scan, 원본 공개 객체·track digest가6회 모두 동일 |
| replay audit 계산 p95 | 0.720–0.744ms/scan, 전체 runtime p95 증가0.153–0.491ms |
| 실제 속도 ROS OFF/ON 각각3회 | 입력/출력 매회649개 일치. 중복/역행 stamp·덮어쓴 scan0 |
| audit ON callback→publish p95 / p99 | p95 17.15–17.37ms / p99 18.45–18.71ms |
| OFF 대비 callback→publish p95 증가 | 0.209–0.649ms, 모든 반복5ms 이내 |
| ROS audit 계산 p95 | 0.767–0.781ms/scan, 모든 반복5ms 이내 |
| 실제 신규 공급/공백 | 완전5초 창 최소13.0Hz.6회 전체 최대 수신 공백97.8ms≤250ms |
| RSS | audit 최대106.31–106.66MiB, OFF62.38–62.57MiB. 두 지도 배열의 고정42.55MiB 추가 + 기존 계산/allocator 변동 |
| 실제 runtime trace 갤러리 | 649scan의9,572객체 판정과 Python 재계산 일치. 합성36사례를 함께 표시, 실패0 |

원본 객체·track digest:
`830e76a5debdad6f6db4c35b3e02a8179d07f46a200a31900d17ffc5e982b115`.
실제 runtime 자료는 유지8,450 / 판단 불가1,122 / 제외 후보0이다. 합성 제외 후보7개는
위치 오차·범위 검증 근거를 명시한 합성 입력이며 실제 객체 제거·검증 성공률로 합치지 않는다.

RSS는 약49초 bag을 반복한 결과이며 장기간 leak 부재의 증명은 아니다. 초기/후반의 수MiB
변화는 OFF에도 있고 지도 배열 증가와 분리해 전체 표본을 보존했다. audit 최대값은3회간
0.36MiB 범위였고 큐·진단 창 상한을 확인했다. 카메라 모델·MORAI 동시 부하나3뷰 fusion·
판제 공동 주행 수락으로 확대하지 않는다. 초기 isolated overlay의 Python `morai_msgs` 경로
누락은 기존 생성 경로를 source/PYTHONPATH에 포함한 후 전체 검사로 재확인했다.

**완료 판정:** 시험 경로에서 HD맵 판정과 비용 검증 완료. 기본 audit OFF·실제 제외 미구현·
기존 판제 입력 유지. 다음 우선 과제는 scan 시각 위치 오차와 세 예외 공간의 외부 접근 범위
검증이며, 실제 전달 제외와 판제의 중복 처리 정리는 별도 PR이다.
