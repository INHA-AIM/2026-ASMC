# A1 — OBB 기반 HD맵 전달 필터 분석


현재 실행 기본값은 [A1.9](#a19-10cm-경계와-실제-발행-필터--2026-10-25)의 **10cm·실제 발행 필터**다.
아래 A1–A1.8의 원본 발행 유지·제외 보류 설명은 당시 설정의 기록이다.
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

## 점군·높이·카메라 보호 미리보기

2026-10-24 후속 A1.2. 사용자가 지적한 A304의 큰 OBB, 머리 위 구조물, 지도 밖 카메라
동적 그룹을 함께 검수한다. **이번 PR도 원본 ROS 배열·tracker를 수정하거나 객체를 제거하지 않는다.**
웹에서 미리보기 제외 후보만 숨겨 비교하며, 최종 전달 판정과 검수 후보를 별도 필드로 남긴다.

| 근거/순서 | 구현과 보류 조건 |
|---|---|
| 카메라 양성 보호 | 원본16종→`dynamic_obstacle`에 해당하고 같은 fusion consumer epoch의 최신 연결이면 **모든 구간**에서 keep. scan 기준 과거500ms·미래50ms 이내, class miss=0. `dynamic_object`라는 새 그룹은 만들지 않음 |
| 점군 support | 해당 객체의 전체 `pcl_ClusterPoints`를 scan context로 실제 `base_link`와 map에 한 번 변환. voxel·지면 제거·군집 이후의 점이며 raw UDP 전체나 웹 샘플이 아님 |
| 최소 근거 | 기본10점 이상·전부 유한값. 부족하면 미리보기 보류. 별도 cloud를 복사하지 않고 수량과 최저 높이만 누적 |
| 머리 위 후보 | 알려진 카메라 객체는 보호. 최저 점과 OBB 하단 **둘 다** base 지붕1.284m+여유0.60m를 넘을 때만 후보. 하단 가림은 확인되지 않아 최종 판정은 `overhead_lower_surface_unverified`로 보류 |
| 지도/예외 보호 | 유효 map pose·geometry 필요. 세 미션 예외 공간 또는 unknown 셀이면 보호/보류. GPS age 통과만으로 제외 근거를 승인하지 않음 |
| 도로 support | 도로 점3개 이상 **또는** 비율5% 이상이면 keep. 기존1m 비교용 경계 여유 근처 점도 보존. 격자 dilation은 보수적으로 올림하므로 정확한1m 거리 측정으로 해석하지 않음 |
| 도로 밖 support | 모든 점이 도로/경계 밖이면 `cluster_support_outside` 미리보기 후보. OBB 겹침은 원래 판정으로 보존. 위치 오차·범위 검증이 없으므로 최종 제외 허가로 승격하지 않음 |

이는 **비교용 정책**이다. 경계 점 하나도 보호하므로 A304 가드레일이 도로에 매우 가까우면
keep이 나올 수 있다. 실제 원본 support와 지도 정합을 검수한 뒤 조정하며, 합성 oversized OBB
검사가 통과했다는 이유로 A304 실차 장면의 문제가 해결됐다고 표시하지 않는다.
상부만 스캔된 미분류 물체는 여전히 머리 위 후보가 될 수 있으므로 임의 자동 삭제를 하지 않는다.

카메라 buffer는3뷰지만 현재 객체 matching은 **전방만 구현**돼 있다. 첫 양성 관측도 보호할 수
있으나 정식 class 확인 횟수나 tracker 분류를 바꾸지는 않는다. 카메라-only bbox를 임의3D 객체로
생성하지 않는다. 좌우 또는 미연결 dynamic 객체의3D matching은 재호의 후속 fusion 작업이다.

### 웹 디버거와 지도 확인

[웹 모니터](web-monitor.md)는 상단 수량, 객체별 점/OBB 비율, 최저 높이, 카메라 보호와 보류
사유를 기본 화면에 모은다. OBB/audit metadata를 먼저 표시하고 점군은 같은 stamp로만 붙인다.
브라우저 조회100ms·단일 요청·단일 예약 그리기, 숨긴 측면 그리기 중단, 지도 polygon bounds
재사용으로 대기/반복 작업을 줄인다. producer session/generation 전환은 캐시를 비우며 경계
scan의 모호한 점군은 생략한다. metadata와 점군 완료를 별도 관측 두 개로 Hz 집계하지 않는다.

배경 지도는 planner의 v5 사본에서 `asmc_hd_map` **v6 정본**으로 변경했다. 실제 파일 SHA와
audit map SHA를 화면에서 대조한다. 지도 파일이 없으면 명확히 사용 불가로 표시한다.
독립 GT가 없는 웹 표시를 지도/위치 정확도의 증명으로 사용하지 않는다.

사용자 요청 **A029·A175·A406**의 주행영역/차선 추가는3-source 비교 후 별도 지도 작업이다.
이번 PR에서 지도 JSON·NPZ·원본 선택 설정은 변경하지 않았다. audit는 tracker를 삭제하지
않으므로 고주로 객체가 사라진 경우 기존 Main 필터·예측 숨김·matching도 따로 확인한다.

```bash
# 컨테이너 /root/ws: 최신/기존 주행영역과3개 차선 source 비교
source /root/ws/devel/setup.bash
roslaunch asmc_hd_map hd_map_source_comparison.launch repo_root:=/root/ws
# 이미 RViz가 실행 중이면 위 명령에 rviz:=false 추가
```

### 실행과 검증 기록

현재 PC의 새 시험 설정은 `/artifacts/lidar-delivery-console-20261024-r2/runtime.yaml`이다.
동결 설정을 수정하지 않고 재사용하며, 새 비교 결과를 만들 때는 새 run을 등록한다.

```bash
# 컨테이너 /root/ws, bridge/camera는 기존 한 세트 유지
roslaunch asmc_lidar lidar.launch debug:=true \
  pipeline_config:=/artifacts/lidar-delivery-console-20261024-r2/runtime.yaml
# 별도 터미널, http://127.0.0.1:8091/
roslaunch asmc_lidar view.launch \
  pipeline_config:=/artifacts/lidar-delivery-console-20261024-r2/runtime.yaml
```

첫 run `lidar-delivery-console-20261024-r1`은 브라우저 시험기가 접힌 측면 메뉴를 열지 않고
조작해 중단됐다. baseline 수치와 두 metadata 스트림을 보존하고 실패로 동결했다. 메뉴를
실제 사용자 순서로 펼치도록 고친 새 run r2에서 전체를 재검증한다. 테스트용 headless 브라우저의
한글 글꼴도 PC의 기존 맑은 고딕으로 연결했으며 배포 웹에 폰트 파일을 추가하지 않았다.

### A1.2 대조 결과 — 2026-10-24

근거 run `lidar-delivery-console-20261024-r2`. 알고리즘/웹 대조 소스 `6fa5f05b`, 이전 구현
`c883e549`. 고정49초 bag을 같은 설정·headless Chromium·1440×900·debug5Hz에서 각각3회
교차 재생했다. GPU 추론·MORAI·사용자 ROS는 실행/종료하지 않고 별도 master11435를 썼다.
최종 문구와 표 줄바꿈 정리는 성능 대조 이후의 표시 변경이며 계산/정책을 바꾸지 않았다.
run은 동결했으며 [결과 포인터](../../../config/baselines/lidar_delivery_console_result_v1.json)에
전체 갤러리 `index.html`, `verification.json`, 파일 manifest SHA를 연결했다.

| 항목 | 이전 / 신규 |
|---|---|
| 표시 장면 경과시간 p50 | 212.5–220.6ms / 164.8–168.0ms |
| 표시 장면 경과시간 p95 | 365.0–368.0ms / 279.2–287.3ms. 반복 p95 평균 약22.6% 감소 |
| 브라우저 그리기 p95 | 6.9–7.4ms / 4.5–5.0ms |
| 실제 ROS 객체 수신 | 6회 모두649개. 중복/역행 stamp·브라우저 오류0 |
| ROS 객체 수신 최대 공백 | 이전104.1–116.7ms / 신규97.6–102.7ms |
| 점군 미완성 표시 | 신규 샘플의0.87–1.30%. 같은 stamp 점군이 아직 준비되지 않은 OBB 우선 장면이며 다른 scan 점군으로 채우지 않음 |
| OFF/ON 각각3회 replay | 각649scan의 객체·track digest가6회 모두 기존 A1.1과 동일 |
| 점군 포함 audit 계산 p95 | 1.071–1.306ms/scan, 최대4.709ms |
| 전체 runtime p95 증가 | 0.279–1.000ms. 단독 unpaced 계산 비교이며 실시간 공급 수락으로 확대하지 않음 |
| replay RSS | OFF66.18–67.10MiB / audit134.69–134.96MiB. occupancy/exception/경계 mask 총63.83MiB 고정 + 기존 allocator 변동. 장기 leak 증명 아님 |

표시 경과시간은 **모니터에 해당 scan의 첫 부분이 도착한 때부터 브라우저 샘플 때까지**다.
센서 취득→ROS→화면 전체 지연이 아니다. metadata 우선 표시와 polling100ms의 효과를
함께 비교했으며 순수 그리기 시간과 분리했다. 점군 없이 먼저 표시한 장면 비율도 숨기지 않았다.
지도/pose 근거를 승인하지 않았으므로 최종 전달 판정의 제외 후보는 여전히0이다.

실제 bag은 주로 A448/A126/A128 자료로 카메라 fusion을 껐다. 카메라 양성 보호·오래된 양성
배제·상부만 보인 알려진 물체 보호는 코어 회귀로 확인했고, 실제 A304 가드레일·동적 미션
카메라 matching·새 지도 범위·하부 가림에 대한 운영 수락은 남아 있다. 웹의 분홍 후보 수를
실제 제거 성공률로 해석하지 않는다. 이 제한을 유지하면서 다음 검수에 사용할 수 있는 상태다.
이번 bag에는 **도로 OBB 겹침이 양수인 제외 미리보기 후보가0**이었다. 따라서 큰 OBB만 도로에
걸친 반례의 동작은 합성 회귀 근거이며 실제 A304 가드레일 해결 근거로 바꾸지 않는다.

## A1.3 인접 도로와 차선 검수 반영 — 2026-10-25

사용자가3-source 화면과 MORAI를 대조해 승인한 **A029·A175·A406**을 v7 주행가능영역에
추가했다. H5 전체를 가져오지 않고 세 링크 envelope 안에서만 표면을 사용한다. 기존102개
도로 면과 ID, 기존121개 paint의 형상은 보존 검사로 확인한다. 최종 제품은73차로·115개 면·
126차선·7정지선이다. 위 A1/A1.1/A1.2의 v6 SHA·수치는 당시 결과로 유지한다.

추가5개 선은 legacy origin-aligned 참조4개 묶음과 A029 왼쪽 누락 보완1개다. 점선은
최종3m/3m 규약을 사용하고 기존 endpoint와 phase를 맞춘다. A029 왼쪽은 관측 원본이 없어
현재 A029 중심선의 왼쪽1.75m로 생성하고 기존 최종 선 endpoint에 연결했다. inference/provenance를
명시했으며 독립 측정 정확도나 RGB GT 검증 완료로 바꾸지 않는다. 비교 그림의 청록색이 이 보완선이다.

RViz 비교 노드가 빈 `LINE_LIST`를 발행할 수 있던 경로를 수정했다. source 레이어를 지우는
`DELETEALL`은 유지하고 주변에 실제 선이 있는 레이어만 발행한다. 스크린샷의 상세 오류 문구는
접혀 있었고 비교 노드는 종료된 상태여서 그 에러 문구와 직접 대응시키지는 못했다. 설치 RViz의
빈 점 배열 오류와 해당 코드 경로를 확인했고, 실시간 세 source에서도 빈/홀수 Marker0을 확인했다.

웹에는 도로 면을 한 번에 채워 내부 링크 경계를 제거하고, 같은 product가 고정한 실제 차선만
그린다. RViz와 동일한 점선 생성 함수를 쓰며 파일 SHA가 다르면 차선을 숨긴다. 실제 객체 제외·
tracker 변경·판제 사본 교체는 하지 않는다. 현재 PC의 실행 설정은 다음과 같다.

```bash
# 컨테이너 /root/ws: 기존 같은 namespace의 lidar/view를 먼저 종료한 뒤 각각 실행
roslaunch asmc_lidar lidar.launch debug:=true \
  pipeline_config:=/artifacts/hd-map-adjacent-review-20261025-r1/runtime.yaml
# 별도 터미널
roslaunch asmc_lidar view.launch \
  pipeline_config:=/artifacts/hd-map-adjacent-review-20261025-r1/runtime.yaml
```

새 지도 번들은 `hd-map-adjacent-bundle-20261025-r1`이며 manifest SHA는
`f2ea9bc0619788291b0d9e7bcb4e578b021db15b238c0ad6bd782e102a9a8934`다. 기존 v6 프로필을
덮어쓰지 않는다. 새 그림·웹 화면·검증은 `hd-map-adjacent-review-20261025-r1`에 보존한다.
현재 설정으로 두 노드를 재시작하고 브라우저를 새로고침해야 기존 캐시와 v6 audit를 혼용하지 않는다.

### 검증 결과

- HD맵182개·LiDAR CTest33개·웹 JavaScript62개 통과. 실제 HD맵 토픽에서도133개 marking·
  73차로·115개 면과 geometry SHA를 확인했다. 과거 동결 보고서5개는 바이트를 보존한다.
- 같은649scan replay의 원본 객체·track digest는 기존 A1.1/A1.2와 동일하다. 새 v7 audit
  계산 p95는1.019ms/scan이다. 지도 추가가 tracker의 탐지·삭제 정책을 바꾸지는 않는다.
- 웹 한 쌍 smoke에서 오류·중복/역행 stamp0, 그리기 p95는6.8→6.1ms였다. 병행 단위시험이
  있던 저장 bag 검사이므로 기존3회 성능 측정이나 실제 주행 수락을 대체하지 않는다.
- [결과 포인터](../../../config/baselines/hd_map_adjacent_review_result_v1.json)의 동결 run에서
  `index.html`, `adjacent-map.png`, `browser/candidate-1-detail.png`를 검토한다. A029 보완선의
  실제 노면 밀착도는 사용자 확인이 남아 있다. 최초 기대 수량·색 집계 오류의 실패 로그도 보존했다.

## A1.4 기본 HD맵·높이 ROI와 제외 근거 — 2026-10-25

**현재 기본은 패키지 HD맵 audit와 평지 높이 ROI다.** 위 A1–A1.3의 개인 프로필·OFF 기본값·
v6/초기 v7 SHA와 수치는 당시 근거로 보존한다. 기존 Main·판제 입력을 전환하거나 map 판정으로
ROS 객체를 실제 제거하지는 않는다. 높이 ROI는 새 패키지의 실제 군집 입력을 변경한다.

### 지도 정본과 실행

A029·A175·A406은 별도 화면 overlay가 아니라 `asmc_hd_map`의 route curation 설정·최종
paint·compact vector·grid·runtime pin에 통합했다. 기존121개 paint와102개 면/ID를 보존하고,
세 링크 중심과 기존 A411/A418이 같은 주행가능영역 연결 성분임을 검사한다.
A029 왼쪽 추정선은 유지하되 사용자 표시 단면 부근(y≈314m)부터 북쪽 합류 끝까지 실선이다.
구 참조 `B219BS011115/B219BS011118`의 Solid 도색을 확인했으며 점선·실선은 같은 좌표에서
연속 연결된다. 최종127차선+7정지선·73차로·115개 면이다. 형상 추정과 도색 확인은 구분한다.

LiDAR의 `config/hd_map_audit/`는 이 정본에서 내보낸 약70KB의 gzip 배포 사본이다. 실행 때
한 번 풀고 배열/manifest/현재 product SHA를 검사한다. 새 지도 생성기의 마지막 단계에서
동반 export를 갱신하며, `--check`로 stale 여부를 확인한다. 개인 D 경로나 Python의 프레임별
지도 조회는 없다. 누락·불일치는 audit만 사용 불가로 표시하고 local 객체를 보존한다.

```bash
# 컨테이너 /root/ws. 같은 namespace의 기존 lidar/view 실행을 종료한 뒤 각각 실행
source /root/ws/devel/setup.bash
roslaunch asmc_lidar lidar.launch debug:=true
# 별도 터미널
roslaunch asmc_lidar view.launch
```

웹은 `http://localhost:8091/`다. 두 실행의 기본 설정 파일은 같다. 이전 D 시험 프로필을
넘기면 그 파일 값이 우선하므로 현재 기본값 확인에는 `pipeline_config`를 생략한다.
복구는 내부 YAML에서 `lidar.height_roi.enabled: false`, `hd_map_audit.mode: "off"`다.
기존 설정에서 height 블록이 없으면 OFF로 읽어 과거 replay를 유지한다.

### 가드레일 원인과 판정

현재 정차 위치의 큰 가드레일은 지원 점736개 중 도로 점0, 보호 여유 점736개였고 OBB는
도로와 약61% 겹쳤다. 곡선을 따라 연결된 긴 군집을 직사각형 하나로 감싸는 문제와 기존1m
경계 보호가 겹친 사례다. 정차 중에도 나타나므로 GPS5Hz 지연만으로 설명하지 않는다.

도로 점이 없는 경계 여유 내 군집은 이제 `cluster_outside_boundary_uncertain` **미리보기 후보**다.
최종 판단은 위치 오차 미검증으로 보류하고 ROS 배열·tracker는 유지한다. 기존 road support
기준(도로 점3개 이상 또는5% 이상)은 유지한다. 당시 미션 예외 보호는 아래 A1.5에서 변경됐으며,
현재 미리보기는 YOLO 양성 매칭만 영역 밖 보호에 사용한다.
도로 안 점이 소수라도 나타나는 주행 사례의 경계 안정성은 후속 검수 대상이며, 이를 전체
가드레일 제거 수락으로 표시하지 않는다. 웹 후보 숨김은 OBB와 정확한 소속 표시 점에만 적용한다.

### 처리 순서·좌표·높이

1. 센서 프레임의3D 거리0.5–100m → X[-30,50]/Y[-20,20]m → Z[-2,3]m → 자차 column 제외.
2. Voxel0.1m → 기존 IMU roll/pitch 수평화 → **평지 지면 위2.5m 높이 ROI** → RANSAC → 군집/OBB/추적/fusion.
3. 최종 객체의 지원 점과 OBB에 같은 scan pose를 적용해 HD맵 점군 판정과 OBB 참조 판정을 계산.

높이는 map 고도가 아니다. 검증한 명목 후륜축 높이0.36m와 장착 위치를 같은 수평화 회전으로
처리해 중력을 기준으로 계산한다. 평지·고정 높이 가정이며 차체 heave를 측정한 값은 아니다.
자차 높이1.644m보다0.856m 높은 cap으로, 트럭 상부가 잘려도 아래쪽 충돌 근거는 남기는
구성이다. 출력 OBB 높이는 남은 점의 높이이므로 실제 차량 전고로 해석하지 않는다.
큰 차량의 지붕만2.5m 위에서 관측되는 가림 상황은 점이 모두 빠질 수 있어 추가 검수가 필요하다.
따라서 이 설정을 기존 Main의 운영 경로로 승격한 것으로 표시하지 않는다.
원본과 잘린 높은 점은 debug로 보존한다. IMU가 무효이면 기존 local 준비 게이트를 따른다.

map grid는 읽기 전용으로 재사용하고 `map_from_level`을 scan당 한 번 조합한다. 점별로
matrix 합성이나 지도를 재생성하지 않는다. 각 점은 한 번 변환·상수 시간 셀 조회하고,
OBB raster는 작은 국소 bounding box만 평가한다. 높이 cap을 군집 전에 두어 비싼 이웃 탐색을
줄이고, 지도 제외는 fusion 뒤에서 카메라 보호와 함께 판단한다. 지도 기반 원본 점 삭제는 하지 않는다.

HD맵을 local로 warp해도 같은 전역 pose가 필요하며 GPS 무효 문제를 해결하지 못한다.
전체 local raster를 매 scan 재생성하기보다 현재의 정적 grid+합성 transform이 이 점 수에서는
간결하다. 화면은 같은 pose의 역변환으로 local에 지도를 그린다. GPS 무효·stale에서는 지도
판정을 보류하고 local 지면/높이/거리 처리와 객체는 유지한다. 마지막 GPS를 고정해 재사용하지 않는다.

기존 map pose는 scan 시각의 IMU 방향·속도로 마지막 GPS에서 최대300ms 제한 예측한다.
5Hz GPS를 단순 반복해10Hz로 만들지 않는다. 이 age 제한은 위치 정확도 보장이 아니며 웹에
GPS age·`gps_bounded_prediction`을 표시한다. 주행 중 남는 오차는 시간 정합·회전 중 예측·
LiDAR 한 바퀴 안의 운동 왜곡을 분리해야 한다. 현재 점에는 개별 취득 시각이 없어 정확한
deskew가 구현됐다고 표시하지 않는다. 이번에는 새로운 IMU 위치 적분이나 GPS 무효 구간 외삽을 추가하지 않았다.

### 웹에서 확인

- `제외점 다시 표시`: 거리/XY/센서 Z/자차/수평 높이 제거점을 따로 켠다. 지면 제거점과
  TunnelFilter 제거점은 전처리 상세에서 켠다. 각 기준은 현재 scan 설정에서 표시한다.
- 객체 후보 숨김은 점군/OBB/상부 높이별로 분리한다. 기본은 점군·높이 숨김 ON/OBB 비교 숨김 OFF이며 체크를 끄면
  분홍/보라/주황 OBB와 소속 점을 볼 수 있다. 초록은 유지, 노랑은 보류다.
- 소속 점 색은 샘플 XYZ와 실제 track support의 정확한 대응이다. OBB 안의 모든 점에
  그 객체 색을 칠하거나 전체 raw를 다시 지도 판정하지 않는다. 모호한 점은 연결하지 않는다.
- 원본 목록·ROS 출력은 유지한다. 선택 객체는 다시 표시한다. 배경과 판정 지도 SHA가
  다르면 경고하고 후보 숨김을 중단한다. 표시 체크는 알고리즘을 재실행하지 않는다.

### A1.4 검증 결과

등록 run `lidar-map-support-20261025-r1`을 동결했다. [결과 포인터](../../../config/baselines/lidar_map_support_result_v1.json)가
갤러리·최종 manifest·원본 bag·전체 압축 trace를 가리킨다. 실행기 원본은 workspace의 작은
코드 동반 기록에 SHA와 함께 보존했다. Docker cp 실패 뒤 동결 결과를 수정하지 않았다.

| 동일649scan 재생 | 높이 OFF·audit ON | 높이 ON·audit ON |
|---|---:|---:|
| 처리 scan | 649/649 | 649/649 |
| 전체 runtime p95 | 20.327ms | 11.780ms |
| 군집 p95 | 14.188ms | 7.031ms |
| HD맵 audit p95 | 1.039ms | 0.966ms |
| 높이 검사 p95 | OFF | 0.052ms |
| 상부 제거 점/scan 평균 | 0 | 1,996.18 |
| 상부 후보 객체-프레임 | 1,316 | 106 |

각 구성1회 비실시간 재생으로, GPU/MORAI 공동 주행 수락이나 장기 안정성 검사를 대신하지 않는다.
상부 후보는 중복 객체-프레임 수이며 고유 천장 수나 제거 recall이 아니다. 남은106건이 있어
모든 천장 제거 완료로 표시하지 않는다. 높이 OFF에서 기존 객체/track/기존 counts digest는
`830e76a5...`로 동일하다. 비교 시 추가된 높이 진단 count만 제외한다. 높이 ON에서는 의도적으로
군집·OBB·추적 결과가 달라지며, 원본 객체 불변성을 주장하지 않는다.

현재 정차 가드레일의52개 신규 화면에서 점군 후보3개가 일관됐다. 전체 표시6개 / 점군 숨김3개 /
OBB 비교 숨김6개로 두 기준의 차이를 확인했고 브라우저 오류0이었다. 상부 점17개/scan을 실제
제거했다. GPS source age가 약2–209ms로 바뀌어도 정차 장면의 후보는 같았다. 이것은 주행 중
독립 위치 정확도 검증이 아니다.

HD맵 전체183개와 추가 국소 연결성 회귀(10개 묶음, 고유 총184개), LiDAR CTest33개·gate,
웹JS64개·문서/공용 계약·공개 지도 토픽 검사를 통과했다. 실제 토픽에서134markings·73lanes·
115areas와 현재 geometry SHA를 확인했다. 기본 workspace도 재빌드했다. 최초 고정 SHA 기대값
실패와 접힌 UI를 검사한 도구 실패는 별도로 보존했다. 시험 노드·브라우저만 정리했고 기존 실행은
유지했으므로 새 기본값을 보려면 기존 lidar/view 및 지도 비교 생산자를 재시작해야 한다.

## A1.5 YOLO 양성 매칭과 미션 영역 보호 구분 — 2026-10-25

사용자는 도로 밖 보호를 **YOLO가 dynamic_obstacle로 분류한 객체와 LiDAR의 실제 matching**으로
명확히 했다. LiDAR 속도 추정의 동적 상태나 미션 영역 위치를 카메라 분류로 취급하지 않는다.

현재 실행에서 사용자 화면의 `#4059`를 조회했다. 원본·유지 클래스 unknown, 도로 점0/259개,
OBB 도로 겹침0%, `camera_dynamic_protected=false`, `camera_known=false`였다.
사유는 `exception_space_protected`, 예외 유형 dynamic_obstacle이었다. 이는 지도에 둔 미션
주변10m 공간에 unknown 군집도 일괄 보호하던 규칙이며 LiDAR 속도 판정에서 유래한 것이 아니다.
이 조회는 실행 중 metadata의 직접 점검이며 새로운 주행 성능·위치 오차 수락이 아니다.

당시 미리보기 정책 `yolo_positive_only_v2`는 다음과 같다. 최신 vehicle·경계 기준은 A1.6을 따른다.

- 미션 근접만으로 point/OBB 후보를 keep으로 덮지 않는다. 기존 OBB audit의 미션 판단은
  `obb_reference_*`와 exception metadata로 보존하되 최종 제외를 승인한 것으로 해석하지 않는다.
- person/dog/deer/bicycle/stroller의 최신 실제 양성 매칭만 전 구간 보호한다.
  같은 fusion consumer epoch, 기존500ms 나이 한계·미래50ms 한계, 이후 miss0을 확인한다.
- 확인 횟수·class 유지 정책은 그대로다. 보호 근거는 `observed_class/observed_raw_class`로 별도 보존한다.
  최신 unknown 또는 miss가 도착했는데 과거 유지된 dog 등으로 다시 보호하지 않는다.
- vehicle·static_obstacle·unknown은 LiDAR 속도/운동 진단이 dynamic이어도 영역 밖 보호를 받지 않는다.
  road support·높이·지도/위치 근거에 따라 평가한다. 근거가 부족하면 보류다.
- 웹은 `YOLO 동적 보호`, `도로 근거 유지`, `구형 미션 보호`를 구분하고 보호 정책·최신 YOLO 원본/그룹을
  상세에 표시한다. 구형 표시는 이전 C++ 프로세스이며 최신 바이너리로 재시작해야 한다.
- 원본 ROS 객체·tracker·운영 Main·판제 입력을 제거하거나 전환하지 않는다.

공개 ROS 메시지·MD5는 바꾸지 않는다. 유효 config ID에는 이 preview 정책 버전을 포함한다.
현재 matching은 전방이며3뷰 matching 완료나 실제 운동 상태 측정으로 표현하지 않는다.

검증에서는 세 미션 종류·영역 밖0/259점·카메라 양성/vehicle·unknown/miss/세대 변경·분류 유지와
최신 매칭의 분리·고속 LiDAR 진단만 있는 unknown을 포함했다. CTest33개는 모두 통과했다.
처음 ROS 환경을 source하지 않은 실행에서 Python 메시지 import로3개가 실행 실패했고,
outer/독립 devel을 source한 뒤 해당3개를 명시 선택해 모두 통과했다. 알고리즘 실패로 면제한 것이 아니다.
웹JS66개·Chrome 합성 UI12개·문서/history 검사도 통과했다. 합성 검사는 실제 주행 품질 수락과 구분한다.

기본 workspace를 재빌드하고 새 바이너리의 정책 문자열을 확인했다. 사용자가 시험 LiDAR만 재시작을
승인한 뒤 점검했을 때 LiDAR·카메라·웹은 이미 종료돼 있었다. bridge/master는 유지하고
`/migration/lidar/lidar_pipeline` 한 개만 `lidar.launch debug:=true`로 실행했다.
새 session은 `79cb9b7c-2295-44f9-b3f0-c63fca11e7c0`, 유효 config ID는
`2ecf16d4b3c5ceac8a3bb1b5b8f175ca65e2aabe50c644b0c528be5082d0ccc6`이었다.

정차 상태 신규10scan(seq512–521)의 객체50건 모두 정책 v2를 사용했다. 도로 점0개인 관측5개가
모두 점군 제외 후보였고, 미션 공간만으로 적용된 preview keep은0건이었다. 이전 `#4059`와 동일한
수평화 중심(8.4157, -11.3543)m·support259개/도로0개인 객체는 재시작 후 ID2로 대응했고
`cluster_support_outside`였다. 비교용 `obb_reference_reason=exception_space_protected`는 남지만
현재 숨김 정책에는 적용되지 않는다. 독립 메시지 발행자는 하나였다.

이10scan의 audit 최대1.525ms·callback→발행 최대4.478ms는 짧은 정차 확인값이며 p95 성능 수락이나
주행 위치 오차 검증이 아니다. 카메라가 꺼져 `status_missing`이었으므로 실제 YOLO 양성 보호는 이번
정차 확인에서 검증했다고 주장하지 않는다. 양성/unknown/miss/epoch gate는 위 회귀 검사로 확인했다.
웹도 꺼져 있어 실제 HTTP 화면 재검수는 하지 않았으며 다시 실행한 뒤 새로고침해야 한다.
최초 임시 trace 점검기는 nullable `spatial`을 dict로 취급해 실패했다. 점검기의 null 처리를 수정한 뒤
위10scan을 별도로 확인했으며 생산자 오류로 집계하거나 최초 실패를 숨기지 않는다.


## A1.6 차량 보호·주행 중 경계 근거 — 2026-10-25

정본16종의 dynamic_obstacle은 person/dog/deer/bicycle/stroller5종이고 vehicle은 car/bus/truck3종이다.
사용자 요청에 따라 현재 정책 `yolo_vehicle_dynamic_interior_v3`는 두 그룹의 최신 실제 matching을
전 구간 보호한다. 기존 consumer epoch·나이500ms·미래50ms·후속 miss0 gate를 유지한다.
LiDAR 속도·motion이나 미션 위치는 계속 보호 근거가 아니다. 이후 unknown/miss/세대 전환 시
보호를 비우고, public 확정/유지 분류와 최신 관측 분류를 별도로 기록한다.

A304 정차 조회에서 작은 군집은2/29점(6.9%)만으로 유지되고 큰 군집은2/726점으로 후보였다.
기존3점 OR5% 조건은 큰 군집의 소수 경계점과 작은 군집의 소수 점 비율에 모두 민감했다.
당시 후보는 도로 면 합집합의 **바깥 경계20cm 안쪽**에서3점 AND5%를 요구했다. 현행10%는 A1.7을 따른다. 0.2m grid의
1셀 직사각 침식이며 adjacent lane 내부 seam은 줄이지 않는다. 원본 occupancy·도로 점/비율·OBB
비교는 그대로 보존한다. grid 조회 때 안쪽 mask를1회 더 읽으며 시작 시1byte/cell(약21.3MiB)을
추가 준비한다. 프레임마다 mask를 만들거나 점군을 복사하지 않는다. 지도/pose 불명은 계속 보류다.

### 동일 입력 대조

등록 run `lidar-vehicle-guardrail-20261025-r1`에서 기존 Capture의 LegacyPurePursuit·P 속도 제어를
재사용했다. Capture·JPEG·새 actor·날씨 변경·공개 ROS 차량 제어는 사용하지 않았다. SDK Ego는
시험 경로 추종·복구에만 쓰고 LiDAR가 쓰는 GPS/IMU pose를 대체하지 않는다. 사용자가 D 여유
49.955GiB에서 소규모 진행을 승인해 해당 등록 호출만49GiB·총512MiB 예산으로 제한했다.
전역 저장 정책은 변경하지 않았다.

기존 코드로10/20/20km/h 목표·각15초 세 번 주행했다. 실제 전진11.46/22.78/22.87m,
최대 속도11.49/21.46/20.87km/h, 최대 횡오차0.190m였다. 정차·원래 위치/기어/모드 복구를
확인했다. 목표 속도로15초 내내 주행했다는 의미는 아니다. 동일639scan을 새 replay에서 대조했다.

| 기준 | 기존 기준이 유지하던 경계 사례10건 중 유지 | 전체 도로 근거 유지 | audit p95 |
|---|---:|---:|---:|
| 기존 원본 도로3점 OR5% |10|10(실행 trace)|별도 기준선|
| 내부 여유0cm·3점 AND5% |1|1|0.437ms|
| 내부 여유20cm·3점 AND5% |0|0|0.481ms|
| 내부 여유40cm·3점 AND5% |0|0|0.424ms|

10건은 고유 객체 수가 아닌 객체-프레임 수다. 해당 OBB는 길이8.1–14.2m·폭0.42–0.88m의
가드레일 형태였고 원본 도로 점은2–4개/23–283개였다. 모든 사례의20cm 내부 점은0개였다.
40cm 추가 이득이 없어 더 작은20cm를 선택했다. 세 replay의 tracks·typed local/map·counts는
모든 stamp에서 동일했다. 지도·위치 보류127객체-프레임과 sparse281객체-프레임도 숨기지 않는다.
이는 실제 장애물 GT의 recall 수락이 아니며 차량/보행자의 경계 보존은 별도 검수 대상이다.

한 프레임은 person 최신 양성 매칭으로7점 군집을 보호했다. 당시 SDK actor 목록은 Ego 하나였고,
이 얇은 군집의 실제 하부 관측/카메라 matching 오분류 가능성을 남긴다. 사용자 요청의 양성 보호를
무시해 강제로 숨기지 않는다. 원본 확정 분류는 unknown일 수 있어 최신 camera_raw_class와 보호
사유를 함께 확인해야 한다. 이는 재호의 matching 신뢰도 보완과 YOLO 오탐 검토에 인계한다.

회귀 CTest33개·웹JS66개·Chrome 합성 UI12개를 통과했다. build 중 추가된 inset 검사가 최초
바이너리에는 없어 필터 조회가0개로 끝난 것을 확인했고, 해당 두 target을 다시 빌드해 runtime/
delivery 검사를 명시 재실행했다. 최초 browser 검사는 sandbox loopback 소켓 제한으로 실행되지
못했고 소켓 권한으로 재실행해 통과했다. 이 실패를 알고리즘 통과로 계산하지 않는다.

### 실시간 적용과 제한

시험 LiDAR만 새 기본 workspace 바이너리로 교체했고 bridge·카메라·웹·master PID는 유지했다.
동일 위치·목표 속도로 새 후보를 세 번 더 주행했다. 전진11.45/22.84/22.81m, 최대 횡오차0.191m,
실제 최대 속도11.50/21.61/21.35km/h였다. 각 구성은 별도 주행이라 scan 수가 다르다.

| 실제 실행 | 기존3회 | 새 후보3회 |
|---|---:|---:|
| 신규 scan |639|638|
| 도로 근거로 유지한 경계 객체-프레임 |10|0|
| 최신 person 매칭으로 보호한 객체-프레임 |1|2|
| audit p95 |0.387ms|0.402ms|
| callback→발행 p95/p99 |3.683/5.558ms|3.625/4.780ms|
| 원래 Ego 위치·기어·모드 복구 |통과|통과|

단시간·한 위치의 반복이며 지연 수치 차이를 일반적인 속도 개선으로 표현하지 않는다. GPS는
기존5Hz·scan 시각 제한 예측을 유지했다. 위치 오차 자체를 새로 추정/보정한 것이 아니라, 소수
경계점이 판정을 바꾸는 민감도를 줄였다. SDK Ego 좌표를 운영 map pose로 넣지 않았다.

남은2건은 person 양성 matching이었고 support6/7점·도로 내부0점이었다. 요청한 카메라 보호를
따른 결과다. 모든 도로 밖 객체를 제거했다고 표현하지 않으며 카메라 오탐/3D 연결은 별도 과제로
남긴다. 실제 차량·보행자를 두고 보호 recall을 검증하지 않았고, vehicle+GPS 무효 보호는 회귀로
확인했다. 새 캡처·추가학습·운영 Main 변경·판제 전달 제외는 하지 않았다.

최종 웹에서 v7 SHA·`road_inset_m=0.2`·`count_and_ratio`, 정상 pipeline/live debug를 확인했다.
조회 순간 입력/처리12.8Hz·debug4.2Hz였으며15Hz 달성·전체 공급 수락으로 해석하지 않는다.
run은 약286MiB로512MiB 예산 안에서 동결했다. 최초 비교 그림의 뒤쪽 박스가 설명 영역에 겹쳐
원본 그림/manifest는 보존하고 v2 그림과 최종 manifest를 새 파일로 남겼다.
[결과 포인터](../../../config/baselines/lidar_guardrail_boundary_result_v1.json)가 고정 결과·갤러리·
재현 소스 SHA를 연결한다. 현재 노드는 새 설정으로 실행 중이며 웹은 Ctrl+F5로 갱신한다.


## A1.7 사용자 지정10%와 OBB 비교 경계 — 2026-10-25

사용자 요청으로 현행 점군 기준을 **20cm 안쪽 도로 점3개 이상 AND 전체 cluster support의10% 이상**으로
변경했다. C++ 기본값·정본 YAML·현행 안내를 함께 변경한다. 0.2m grid1셀 침식·YOLO vehicle/
dynamic_obstacle 양성 보호·지도/pose 보류·원본 ROS 객체 보존은 유지한다. 설정 SHA와 유효
config ID가 새 비율을 반영하며, 보호 정책의 종류는 같아 버전 이름은 v3를 유지한다.

새 비율의 미만/이상 경계(100점 중9/10점), 작은 군집의3점 조건, 큰 군집의 비율 조건을 회귀로
확인한다. 기존 A1.6의5% 반복 주행 수치·동결 run·포인터는 당시 결과로 보존한다. 이 변경을10%의
새 주행 수락이나 실제 장애물 recall 검증으로 표시하지 않는다. 과거 YAML의 명시적0.05는 재생 시
그대로 사용할 수 있다.

A1.7 당시 OBB 비교는 `road_ratio_grid>0`이면 keep이고0이면 후보였다. 현행 AND/10%는 A1.8을 따른다. 고정된15% 전달 조건이 아니며
점군/높이 판정과 별도 checkbox다. OBB 비교 체크를 켜면 그 독립 후보 숨김도 적용하므로 점군으로
유지된 객체가 OBB 비교 때문에 숨겨질 수 있다. 기본 OBB 비교 숨김은 OFF다. 새 패키지는 아직
HD맵 사유로 ROS 객체를 제외하지 않는다. 기존 Main/planning_inputs의 별도 road filter 기본값
0.15와 구분한다. 해당 기존 경로에는 motion 진입 예측·기타 필터도 있어15%가 유일한 전달 조건은
아니다. 이번 작업에서 기존 경로의 임계값·행동 정책·운영 전환은 변경하지 않는다.

관련 runtime/delivery CTest2개와 문서/history·실행 안내 검사가 통과했다. 기본 workspace의
`lidar_pipeline_node`를 재빌드하고 시험 LiDAR만 재시작했다. 기존 bridge·카메라·웹·master PID를
유지했고 웹 metadata에서20cm·3점·0.10·count_and_ratio와 단일 발행자를 확인했다. 새 유효
config ID는 `77b30bdd5177baabb9a1c6b2235454fd3a8eb266ce995ead6075b166a22c72e7`이다.
새 주행·수집은 수행하지 않았으며 기존 동결 실험 파일은 수정하지 않았다.


## A1.8 공통 도로 영역의 점군·OBB AND — 2026-10-25

사용자는 동일 영역·점군 조건에 OBB10%를 더하고 OBB가 통과해도 점군 미달을 보완하지 않도록
요청했다. 현재 `yolo_points_and_obb_v4`의 일반 유효 관측 표시는 다음을 모두 요구한다.

- 영역: 기존 결정과 같은 도로 합집합 바깥 경계20cm 안쪽(0.2m grid1셀 직사각 침식).
- 점군: 전체 cluster support의3점 이상 AND10% 이상이 이 영역 안에 있음. 전체 근거10점 이상 품질 gate 유지.
- OBB: 원본8코너의 XY convex footprint raster 셀 중 같은 영역과 겹치는 비율10% 이상.
- 높이 후보는 별도 적용한다. camera vehicle/dynamic의 유효 최신 양성 보호는 유지하며 속도·미션 위치는 보호 근거가 아니다.

원본 도로 점/OBB 겹침과 기존 A1 `obb_reference_*`를 보존한다. 새 OBB 내부 겹침은 같은 footprint
루프에서 mask를 한 번 더 읽어 계산한다. 추가 영상·지도·점군을 복사하거나 scan마다 mask를 만들지
않는다. 격자 비율은 연속 면적의 정확한 기하 교차값이 아닌0.2m 근사임을 유지한다. 독립 근거는
각각 기록하지만 종합 preview keep은 AND다. 정상 YOLO 보호 외에 한쪽 근거만으로 keep하지 않는다.

기본 웹의 점군·OBB·높이 후보 숨김을 모두 ON으로 맞췄다. 선택 track·typed box·소속 점은 같은
표시 함수를 쓴다. 원인별 숨김을 해제하면 사유 색으로 제외 후보를 다시 검수할 수 있다. 지도 SHA
불일치·map/pose 불명·근거 부족은 후보로 확정하지 않으며 기존 보류를 유지한다.

공개 메시지·MD5·ROS 객체·tracker·기존 Main·판제 입력은 바꾸지 않는다. 실제 HD맵 전달 제외는
여전히 미적용이다. A1.6의 반복 주행 수치는 당시5% 정책이며 현행10%+OBB AND의 새 주행 수락으로
표기하지 않는다. 기존 frozen run과 결과 포인터는 수정하지 않았다.

다음 PR의 기존 필터 편입 목록은 [별도 정리](filter-migration.md)를 따른다. road/known-static/
structure/ground 품질·추적 확인과 속도는 인지 후보이고 긴급 거리·행동·센서 장애 대응은 판제에 남긴다.
OBB clipping은 새 인지 정본에 편입하지 않는다. 기존 코드 삭제와 실제 소비 전환은 기능별 검증 이후다.

검증은 CTest33개·웹JS67개·Chrome 합성 UI17개를 통과했다. OBB9.9/10/10.1%와 점군 통과/실패의
조합, 같은 마스크의 footprint 침식, unknown/무효·보호·typed/선택 객체 우회·검수용 해제를 포함한다.
OBB 미계산을 정상0%로 표시하지 않도록 null을 보완한 뒤 관련 runtime/delivery·기존 A1 공간 대조
3개를 다시 통과했다. 마지막 추가 검사 명령의 probe target 이름 오류는 바로잡아 재실행했으며
실행되지 않은 검사를 통과로 집계하지 않았다.

기본 workspace를 재빌드하고 시험 LiDAR 한 개를 적용했다. 종료된 launcher의 zombie 기록 때문에
첫 재시작 guard가 중단됐고, 실제 처리 노드·ROS 발행자가 없음을 확인한 뒤 새 노드 하나만 시작했다.
bridge·카메라·웹·master PID는 유지했다. 웹 신규5프레임·81객체에서 v4·20cm·점군3/10%·OBB10%·
points_and_obb를 확인했다. 새 유효 config ID는
`30456322794a6c4facdbfe4dac69abff021064dfa34cf5ac0892fc23293a2c96`이다. 이는 적용 확인이며
새 반복 주행·실제 장애물 recall·전체 연산 수락으로 표시하지 않는다. 새 수집이나 학습은 하지 않았다.

## A1.9 10cm 경계와 실제 발행 필터 — 2026-10-25

사용자 요청에 따라 **10cm·점군3점/10% AND OBB10%**와 상부 객체 판정을 새 패키지의
발행 배열에 적용했다. 기존 Main·판제 입력 전환은 하지 않는다. A1–A1.8의 실제 제외 보류는
당시 결과이며, 이번 적용과 구분한다. 구현 커밋은 `049f9589`, 기본 전환은 `bf65d658`이다.

### 10cm·15cm 구분과 선정

이전 구현은 `ceil(inset / 0.2m)`로 올림해10cm·15cm도 실제20cm였다. 현재는 기존 도로 셀의
**합집합을 기준으로 직사각 거리(L∞)의 안쪽 여유**를 적용한다. 점은 셀 내부 실제 위치로 검사한다.
OBB는 기존0.2m footprint raster의 경계 셀을4×4(5cm 간격) 중심 샘플로 면적 가중한다.
지도 정본을 편집하거나 매 scan 고해상도 지도를 만들지 않는다. 읽기 전용 mask는 기존과 같은
셀당1byte이며 경계 coverage를0–16으로 저장한다. 도로 내부 차로 seam은 침식하지 않는다.
이 값은 원본 vector 경계에 대한 정확한 수직 거리나 GPS 위치 정확도 보장이 아니다.

등록 run `lidar-delivery-inset-20261025-r1`에서 기존 A304 PP/P 주행 bag639scan을 같은
점군·OBB·fusion 설정으로 재생했다. 원래 기록의3회 통과는10/20/20km/h 목표이며 새 주행은 하지 않았다.

| 설정 | 일반 객체의 점군·OBB 동시 유지 | 카메라 보호 | audit 계산 p95 / p99 |
|---|---:|---:|---:|
| 20cm·audit | 0 | 1 | 0.410 / 0.582ms |
| 15cm·audit | 0 | 1 | 0.438 / 0.666ms |
| **10cm·audit** | **0** | **1** | **0.426 / 0.814ms** |
| **10cm·실제 filter** | **0** | **1** | **0.480 / 0.934ms** |

가장 덜 좁히는10cm에서도 이 기록의 가드레일 유지가 재발하지 않아10cm를 채택했다.
5,722객체 관측 중5,467건은 발행 제외,255건은 보존했다. 보존은 위치 불가127·근거 부족127·
최신 카메라 보호1건이다. 이는 객체 관측 횟수이며 서로 다른 장애물 수가 아니다.
10cm는15/20cm보다 sparse OBB를 덜 제외했다. 모든 구성의 원본 track·점 개수·좌표는 같고,
audit 모드의 typed 배열도 동일하다. filter 결과는 원본 배열에서 해당 ID만 안정적으로 제거한 것과
같다. 최종 기본 YAML로639scan을 한 번 더 재생해 raw/typed 결과 동등성을 확인했다.

### 발행·상부 객체·실패 경계

- `hd_map_audit.mode`는 `off / audit / filter`이며 기본은 **filter**다. `audit`은 이전처럼 판정만 한다.
- 최종 fusion 뒤에 판정하고 `/migration/lidar/objects_local/map` 및
  `observations_local/map`에 같은 제외 ID를 적용한다. tracker·검출·OBB·속도는 수정하지 않는다.
- 모두 제외되면 `OBSERVED_EMPTY`, reason `delivery_filtered_empty`다. 센서 무효와 pose 무효 상태를
  정상 빈 결과로 바꾸지 않는다. map 불가 시 local 객체는 지도 사유로 제거하지 않는다.
- 최신 실제 YOLO vehicle/dynamic 매칭은 모든 구간에서 보호한다. 속도·운동 상태·미션 위치는 보호 근거가 아니다.
- 상부 판정은 support10점 이상·유한값 조건에서 최저 지지점과 OBB 하단이 **모두 base_link z1.884m 초과**일 때다.
  이는 지붕1.284m+여유0.60m다. 알려진 카메라 객체는 보호하며, 이 규칙은 GPS·지도 없이도 적용한다.
  앞단의 지면 위2.5m height ROI도 유지한다. 실제 하부 가림 완전성과 전체 장애물 recall은 독립 GT로 미수락이다.
- trace의 `delivery_applied`, 입력/제외/출력 수와 객체별 `delivery_excluded`가 실제 발행 근거다.
  기존 `decision/reason`은 위치 오차 등을 검증하지 않은 A1 audit 비교로 남긴다.
  웹 체크박스는 원본 tracker의 검수 표시만 바꾸며 ROS 배열에 객체를 되살리지 않는다.

### 실행 검증·근거·복구

CPU/ROS 회귀33항목을 확인했다. 최초 전체 실행의 기존 ROS fixture는 임의 도로 밖 좌표 때문에
실패해, 정상 fixture를 현행 지도 내부로 옮기고 별도 도로 밖 제외·GPS 무효 복귀 단계를 추가했다.
최종 ROS18시험은 통과했고 원본 tracker가 같은 ID로 유지됨을 확인했다. 웹JS67·Chrome UI19개도 통과했다.
최초 live checker는 publisher가 붙인 `header.seq`와 nested producer seq를 같은 값으로 비교해 실패했다.
이 전송 필드만 정규화한 뒤 나머지 직렬화 bytes·시각·상태·session/generation/scan ID를 비교했다.

기본 workspace를 재빌드하고 시험 LiDAR만 재시작했다. bridge·카메라·웹·master는 유지했다.
실시간 첫 비교는 GPS 위경도0,0에서255scan·1,675객체 관측을 보존했고 약12.75Hz·최대 공백123ms였다.
이는 지도 제외 보류 검증이다. 최종 바이너리 재시작 후 GPS가 정상으로 복귀한 비교에서는
**253scan·506객체 관측을 실제 제외**했고 약12.64Hz·최대 수신 공백114.3ms였다.
네 토픽은 단일 생산자이며 배열/관측/trace가 일치했다. 실제 제외는 bag·합성 ROS·후자의 live로 확인했고,
이를 독립 장애물 recall·전체 주행 성능 수락으로 확대하지 않는다.
새 config ID는 `47861fc1a320dce0d4bc4c73dacae6c7db2b281bc7ba0cef901b3bc3588b409e`다.

결과·입력/설정/소스 SHA·로그는 [결과 포인터](../../../config/baselines/lidar_delivery_inset_result_v1.json)에서
등록 run의 `index.html`, `result.json`, `final-replay.json`, `live-result-v3.json`으로 연결한다.
이전 frozen run을 수정하지 않았다. 소규모 검증 계속 승인 범위에서 결과256MiB 상한을 두었고
새 bag 수집·학습·차량 조작은 하지 않았다.

복구는 내부 YAML의 `hd_map_audit.mode: audit`로 바꾸고 시험 LiDAR만 재시작한다.
이는 같은 판정을 표시하며 배열을 보존한다. 점군 HD맵 ROI·기존 Main 필터 삭제·판제 소비 전환은 후속이다.

## A1.10 톨게이트 섬의 지도 제외 — 2026-10-25

사용자 화면에서 톨게이트 기둥이 있는 섬이 주행가능영역으로 채워진 것을 확인했다.
원인은 검수된 흰 실선과 별개로 생성한 lane-width·인접 차로 corridor-fill 면이 섬 내부까지
덮는 것이었다. 객체 필터의 임계값을 조정하지 않고 HD맵 정본에서 수정했다.

`asmc_hd_map/config/kcity2025_compact_drivable_source.yaml`의 `non_drivable_islands`는
기존 검수된 A153/A451·A432의 실선 체인을 지정한다. 공통 분기점과 합류점이 닫힌 두 섬만
구성하며, 임의 좌표나 추정 폭으로 영역을 넓히지 않는다. 모든 corridor-fill 생성 **후에**
Lane과 DrivableArea에서 동일하게 차감한다. 단순 polygon 계약을 깨는 분리/구멍은 자동으로
버리거나 메우지 않고 빌드 오류로 돌려 명시적 토폴로지 검토를 요구한다.

- 두 섬의 전체 면적은 약43.98/45.09m²다. 이미 지도 밖이던 부분을 제외하고 최종 도로
  합집합에서 약32.92m²를 추가 차감했다. 0.2m grid는686셀(27.44m²)을 주행불가로 바꿨고
  새 주행가능 셀은0이다. 벡터와 grid 면적 차이는 좁은 경계의 raster 근사 때문이다.
- Lane3개와 도로 면4개만 수정했다. 기존73차로·115도로 면의 ID/선택·토폴로지는 유지한다.
  이전102개 도로 면 중 수정 대상4개를 제외한98개는 이전 제품의 SHA 기준으로 그대로다.
  실제 차선 paint는 변경하지 않았다. 섬 사이/옆의 통행로도 보존했다.
- 정본 벡터·grid·runtime pin 검사·LiDAR 배포 bundle을 같은 빌더 체인으로 갱신했다.
  배포 도구가 모드를 audit로 강제하던 동작을 고쳐 filter/audit/off 선택을 보존한다.
  10cm·점군3점/10%·OBB10%와 상부 정책은 유지한다. 이전 frozen run·결과 포인터는 수정하지 않는다.

관련 지도·ROS 소비78검사를 생략 없이 통과했고, LiDAR runtime·기존 공간 판정 동등성·배포 계약
3검사와 전체 artifact 재생성 check도 통과했다. 기존 geometry 고정 검사는 승인된4개 면을
구분해98개 보존 검사로 바꿨다. 초기 전체 연구용 suite는 환경 생략 및 이와 같은 예전 고정값
실패가 있어 통과로 집계하지 않았으며, 올바른 catkin 환경에서 관련78개를 다시 확인했다.

시작 시 지도 캐시를 읽는 시험 LiDAR와 디버그 웹만 재시작했다. bridge·카메라·master·기존 Main은
유지했다. 웹 배경과 실제 판정의 파일 SHA가 일치하고, 두 섬 내부는 비주행·통행로는 주행 상태임을
확인했다. 신규10프레임의140개 발행 제외 객체 관측은 typed 배열에 없었다. 전체 주행 recall
검증으로 확대하지 않는다. 새 수집·학습·차량 조작은 하지 않았다.

현재 제품 파일 SHA는 `16b8f5f811ec08ef0a7827546f4450302b69533be44952a030bf8c9c79825b7d`,
bundle manifest SHA는 `13c517e3fffee3bfd64fabdb6f473c6c668d3d87e2142c38ff756e3650b98f2e`,
실제 config ID는 `cfe908b7c17612517e2ee5cc68c869b48801689462c1048e9bb918b7cb335506`이다.

시각화는 보존한 원본 tracker와 필터 후 typed 배열을 함께 전달한다. 실제 처리 순서는
판정→배열 필터→객체/관측 발행→debug snapshot이며, 발행 제외 객체가 원본 레이어에 보이는 것은
검수용이다. 화면을 먼저 발행해서 제외가 늦게 적용되는 것이 아니다.

병합 전 hosted CI에서 모니터 상태 시험의 시작 순서 경합1건이 확인됐다
([실행 근거](https://github.com/INHAautonav/2026-ASMC/actions/runs/40198658906/job/121184924872)).
같은 master snapshot에 점군 발행자와 pipeline이 모두 나타날 때까지 기존10초 한계 안에서
기다리도록 시험만 수정했다. 운영 코드·timeout은 바꾸지 않았고 독립 ROS 모니터7시험을 다시 통과했다.
