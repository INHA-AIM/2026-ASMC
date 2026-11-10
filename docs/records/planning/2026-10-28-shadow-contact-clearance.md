# S1 정지 수정 실험: 영상 접점과 local 관측의 제한된 경로 검사

2026-10-28 KST, PR #125, `fix/shadow-stop-reproduction`.
사용자가 기존 6회 재현 이후 **4시간의 자율 수정·반복 주행과 S1의 D 공간 예외**를 승인했다.
[첫 6회 기록](2026-10-28-shadow-scenario-reproduction.md)은 수정 전 결과로 보존한다.
이 문서는 후속 수정 실험이다. 일반 `autonomy.launch`의 운영 기본값은 그대로이고,
새 동작은 시나리오의 내부 실험 설정으로만 켠다. 전체 음영 주행 수락과 동일하지 않다.

**결론: 재현된 입력·기하 결함은 수정했지만, plastic/concrete 회피 정지는 아직 해결되지 않았다.**
4시간 작업창에서 r07–r50의44회 실행 시도(준비 실패 포함)를 기록했고, 앞선6회와 함께50개를 보존한다.
저속 drum 통과는 기존 방식에서도 재현됐으므로 새 알고리즘의 단독 개선 근거가 아니다.
운영 기본은 유지하고 PR은 Draft로 둔다. 다음 단위는 공통 local 입력과 회피 중 재계획·추종의 결합 검증이다.

전체 목록은 D 정본 `artifacts/shadow-stop-fix-20261028-audit01/matrix-final-v2/index.html`이다.
각 행에서 개별 RGB/판정 갤러리, 실제 소스 SHA, 배치 변경·배경 분리·기록 누락을 확인한다.
초기6회의 `running/runs_done`은 종료 직전 driver 상태이며 현재 실행 여부나 주행 수락을 뜻하지 않는다.
후속 event가 없는 옛 실행에 `section_end`를 추정하여 추가하지 않았다.

## 수정한 원인과 경계

| 원인 | 변경 | 유지한 제한 |
|---|---|---|
| 가까운 BOTH 접점과 먼 DEFERRED_FAR가 함께 있으면 전역 정지 | 정확히 대응한 family 안에서 차량 footprint와 접점 점유를 검사 | far/UNKNOWN을 삭제하지 않음. 모르는 객체 깊이는 관측 도로를 따라 하류 점유로 유지 |
| family의 observed/inferred가 연속인데 첫 조각의 끝만 지평으로 사용 | support 구간을 연결하되 INVALID gap은 넘지 않음 | 실제 두 차로·소비자 가용 길이의 교집합 사용 |
| 분류기가 사용한 family가 소비자의 최신값으로 교체됨 | stamp로 대응하는 최대8개 family cache와 queue8 | 같은 frame을 못 찾으면 정지. 최신 family를 대신 사용하지 않음 |
| 변경 길이를 재설정하며 끝점 방향이 부정확 | 기존 quintic 경로의 시작 기울기 Hermite basis 수정, 전체 shift 후보 분리 | 기존 기본 경로의 partial-shift 계산 유지. 도로·곡률·차체 여유 검사 유지 |
| 차로 중앙선을 걸친 객체를 반드시 전체 차로 변경으로 회피 | 충돌 검사된 현재 차로 내 경로를 우선 | 현재 차로가 막히면 기존 상태 머신의 인접 차로 확인·변경 사용 |
| 가까운 drum의 접점이 ego mask에 가려짐 | 동일 시각의 실제 발행 local OBB를 유일한 기존 클래스/투영 대응일 때 사용 | 빈 결과·예측 track·중복 대응·오래된 관측은 영상 불확실성을 해소하지 못함 |
| 실험 감속값이 KEEP에만 적용돼 회피 중 약25km/h 진입 | 기존 Behavior의 최종 `maximum_speed_kph`를 시험에서5로 제한, 실제 CAN 속도도 검사 | GPS가0이어도 CAN 속도만 사용. 무효 위치를 복원하지 않음 |
| 60초 이후 bag이 닫히기 전에 첫 split만 분석 | recorder 자식의 종료/전체 inventory 검증 후 분석 | 이미 동결한 원본·부분 분석은 덮어쓰지 않고 별도 재분석으로 교정 |

모델, LiDAR 검출·추적·fusion·HD맵·터널 임계값은 수정하지 않았다.
차로 변경 pause/진행 timeout과 selector watchdog250ms를 늘리지 않았다. family의 공용 readiness350ms,
기존 follower의 source/bbox source 상한0.5초와 receive 검사값도 유지했다. 새 local·실제 속도 입력은
source/수신350ms를 별도로 적용한다. 이 서로 다른 계약을 모두350ms라고 요약하지 않는다.
새 제어 발행자나 두 번째 tracker·운동 분류기를 만들지 않았다.

## 입력과 경로의 의미

- 이미지 하단 접점은 실제 선택된 영상 시각 IMU 평면으로 base_link에 역투영한다.
  객체의 전체 3D 크기나 독립 깊이 GT가 아니다. 가림·잘린 하단·잘못된 projection은 무효다.
- local은 `/perception/lidar/tracks_local`의 기존 atomic 관측을 읽는다. base_link·session·generation·
  config/calibration ID, source/수신 freshness350ms, 미래50ms, 영상–scan 차이±50ms를 검사한다.
  뷰별 카메라 inference를 새로 실행하거나 fusion 확인 횟수를 증가시키지 않는다.
- 관측 OBB8코너의 XY envelope에25cm를 더한다. 이것은 현재 관측 점의 extent이며 가려진 전체 물체의
  정답 형상이 아니다. `OBSERVED`, miss0, 실제 지지점3개 이상과 유한 크기/코너를 요구한다.
  발행된 다른 local 객체도 클래스와 무관하게 충돌 검사에 포함한다.
- 이미지↔기존 local 클래스 이름 일치, 현재 카메라 보정의 원본1280×720 투영 IoU≥0.25,
  양방향 유일 대응일 때만 이미지 접점을 측정 local footprint로 대체한다.
  모든 local 객체를 새 클래스 근거로 사용하지 않는다. 빈 local이 unresolved bbox를 지우지 않는다.
- 그림상 같은 시각처럼 보이더라도 ego motion/deskew를 구현한 것은 아니다. family–image≤150ms와
  image–local≤50ms의 제한된 시간차를 저속 실험에서 허용한다. 물리적 정합 정확도 보장은 별도다.
- 같은 기존 Pure Pursuit·차로 변경 상태 머신을 사용한다. 기존 차체 폭1.892m, 전방3.845m,
  후방0.790m, 횡 여유0.30m, 종 여유0.50m와 sampling 여유를 검사한다.
  영상 접점의 미관측 하류 영역은 도로 reference를 따라 점유로 둔다. 측정 local extent에는 이
  무한 하류 가정을 붙이지 않고, 현재 실제 발행 객체 전체를 함께 검사한다.
- 영상 접점의 네 투영점을 볼록 사각형으로 보존한다. 원거리 꼭짓점의 횡 범위를 근거리까지
  축 정렬 사각형으로 늘리지 않는다. 가장 먼 접점 이후의 미관측 깊이는 종전 보수적 점유를 유지한다.
  순서가 잘못되거나 퇴화한 사각형은 무효다. 이는 전체 물체 깊이를 복원한 것이 아니다.
- 경로는 가용 family support 안에서 차체 앞부분까지 검사한다. 도로 내부 보간 공백을 메우거나
  부족한15–20m를 임의 외삽하지 않는다. 근거리 후방 도로 검사는 기존 접선 근사를 유지한다.
- 현재 차로의 중심 offset0/±0.05…±0.50m와 길이3/4/5/6/8/10/12m, 최대147개를 차로별로 검사한다.
  이147개가 모두 실패할 때만15m 완결 길이를 같은 offset21개에서 추가로 검사한다(최대168개/차로).
  이전에 통과하던 후보의 우선순위를 바꾸지 않는다. 실제 경로의 끝은 여전히 관측 support에 제한되며,
  15m 차로 변경 완료를 관측 없이 확정하지 않는다.
  후보가 통과한 뒤 기존 Pure Pursuit의 추종점을 해당 길이의 절반(1.5–4m)으로 선택한다.
  짧은3m 후보 뒤를 운영 기본8m 추종점으로 바라보는 문제를 줄이는 실험 정책이며, 새 제어기는 아니다.
- 늦게 도착한 local 관측은 기존 최대8개 영상 대기 큐에서 source±50ms로 대응한다. local 콜백이
  기존 작업을 깨우며 고정 sleep을 하지 않는다. 기존250ms 대기 제한·350ms freshness는 늘리지 않는다.
  빈 관측도 유효 상태와 정확한 시각 대응이 있어야 하며 unresolved 이미지 bbox를 지우지 않는다.
- 후보 수·탈락 원인·실제 새 계획 계산 wall time을 diagnostics로 제공한다. 이전 계산값을
  입력 정지 동안 반복 샘플링하여 계산 p95를 부풀리지 않는다.

## 재현 자료와 결과 해석

후속 run 접두사는 `shadow-stop-fix-20261028-`이고, D의 `artifacts/<run-id>`가 정본이다.
각 run은 실행 전 소스 revision·scene/config SHA·용량 상한을 등록했다. 기록 용량 상한은3GiB,
수집 중 비상 잔여 공간은2GiB다. 예외는 사용자 승인 S1에만 적용했고 저장 정책 파일은 바꾸지 않았다.

- r07–r17: 접점 지평·정확한 family 대응·경로 수학·차체 전체 검사를 대조한 실패 자료.
- r12: SUNNY plastic을 지나갔지만 배경 차량29 앞에서 정지. 전체 구간 통과가 아니다.
- r18: FOGGY concrete를 지나갔지만 배경 차량29 앞에서 정지. 전체 구간 통과가 아니다.
- r19/r20: 가려진 drum과 현재 local OBB의 유일 대응을 확인했으나 중간 경로가 막혔다.
- r13–r21의 일부 진입 실제 속도는 약6.8m/s였다. KEEP-only 감속을5km/h 진입으로 잘못 해석하면 안 된다.
- r22: 전 행동 최종5km/h cap을 적용해 실제 약1.36m/s로 진입, drum을 지나간 뒤 배경 차량29 앞 정지.
- r23: 배경29를 일시 이동한 분리 장면에서 불필요한 전체 차로 변경 후 경로 막힘/30초 pause watchdog.
  같은 실패 geometry에서 짧은3/4/5m 후보도 통과하지 못했다. 이 실패를 해결된 것으로 표시하지 않는다.
- r24: 현재 차로 우선에서도 좁은 경로가 후보 격자 사이에 빠졌다. 같은 native C++ 검사에서
  offset−0.35m/길이3m는 기존 안전 여유로 통과했다. 작은 Git fixture로 보존했다.

- r25/r27: 터널 진입 전 NORMAL의 도로 경계 정지. JAMMING 명령 미수신을 녹화 실패로 남겼다.
- r26: 세밀한 후보만 추가해도 실제 추종이 부족해 drum 앞 정지. 계획 p95=5.765ms, 조회한 native 접촉0.
- r28: 짧은 경로 추종점 적용 후 분리 FOGGY11 drum 구간 종료(100.912초). native 조회110회에서 접촉0,
  새 계획1,586표본 p95=2.411ms. local 미수신77·실제 속도 제한33 등 중간 정지 표본은 남았으므로
  무중단·독립 무충돌 수락으로 표시하지 않는다.
- r29: SUNNY plastic의 NORMAL 진입 정지. 첫36초에 전체 CPU 단위시험이 겹쳐 부하 대조에서 제외한다.
- r31: SUNNY plastic의 진입점 분리 시험에서 `contact_swept_horizon_missing` 정지.
  실제 native strip은 왼쪽1.11392–19.4756m, 오른쪽0.925828–19.5185m인데, 실험 코드의 시작점≤1m
  가정 때문에 지원 범위를0으로 계산했다. 기존1m 근거리 접선 기준을 실제 첫 점에 맞추되 최대 한
  샘플 간격25cm까지만 허용하도록 보완했다. 중간 INVALID gap/전방 외삽은 허용하지 않는다.
  같은 raw fixture의 원본0 값은 보존하고 공통 지원 범위 함수를 재실행하니19.4756m 및 유효 후보가 나온다.
- r32: FOGGY11 drum 반복도 구간 끝 도달(97.801초), native 접촉0/106조회. local 미수신 정지 표본은
  r28의77에서0으로 줄었고, 새 계획 p95=1.480ms였다. projection 만료4·ego mask10·속도 제한8 등
  나머지 중간 정지 표본은 여전히 별도 기록한다.
- r30: 시작점을 터널 쪽으로 옮겼으나 이미 GPS0인 곳이어서 기존 판제 준비 검사가 실패했다.
  가짜 위치나 과거 GPS로 준비를 통과시키지 않았다. 주행·회피 검증 표본이 아니다.

- r33: near-start 수정 후 SUNNY 입구 plastic은 지원 범위17.91m가 있어도 모든 후보가 막혔다.
  음영 제어 진입 시 접점은 약7.30–8.08m 앞이었다. 같은 결과의 local 배열에는 전방 plastic이 없고
  뒤쪽 관측만 있었다. 영상 접점만으로 전체 물체 깊이가 해결된 것처럼 처리하지 않았다.
- r34: FOGGY11 concrete의 실제 연속 support14.764m를 고정15m 요구 때문에 전부 버렸다.
  `short-end-*` native fixture에서 수정 전 후보0개/road reference 누락, 수정 후 같은 hard margin으로
  후보가 만들어진다. 실제 지원 범위에서 차체 전체 길이를 뺀 기존 sampling grid까지만 생성한다.
- r35: NORMAL의 low-speed fallback이 터널 전에 거부됐다. JAMMING 개선의 실패/성공과 분리한다.
- r36: valid-entry concrete는 고정15m 실패가 사라졌지만 가까워진 뒤 모든 경로가 막혔다.
  계산 p95=3.146ms, native 접촉0/70조회. 이 경계는 여전히 미해결이다.
- r37: 추가15cm 여유를 선호하는 후보는 concrete 정지를 해결하지 못했다(p95=3.305ms).
- r38: 인접 차로를 일찍 우선하는 후보도 concrete에서 실행 가능한 인접 경로를 찾지 못했다.
  두 선택 정책은 결과를 보존하고 최종 실행 코드·설정에서 제거했다. 원본 커밋과 run SHA로만 재현한다.
- r39: 네 투영점 보존 후에도 SUNNY 입구 plastic은 약8m 앞에서 전체 경로가 막혔다.
  p95=3.348ms, native 접촉0/49조회. 투영 표현 수정이 이 주행 실패까지 해결했다는 의미가 아니다.
- r40: 불채택 선택 정책을 제거한 최종 후보도 분리 FOGGY11 drum의 `section_end`에 도달했다.
  생성기 구간 시간98.873초, driver의 주행 허용/정리 창106.808초, native 접촉0/107조회.
  새 계획1,647표본 p95=1.436ms·p99=1.856ms·최대2.383ms. local 미수신 정지 표본0이지만
  projection 만료3·support 부족3·ego-mask 접점17표본은 남았다. 무정지 수락은 아니다.
- r41: 같은 최종 후보의 FOGGY13 입구 plastic은30초 정체로 종료했다. `contact_all_paths_blocked`
  611표본, 계산 p95=2.749ms, native 접촉0/47조회다. SUNNY만의 실패가 아니다.
- r42: 입구 위치·FOGGY13·5km/h를 맞춘 **기존 bbox 방식**도 plastic 앞에서 정지했다.
  `adjacent_lane_detected_or_path_invalid`586·`no_continuous_own_lane_to_lookahead`24표본,
  native 접촉0/49조회다. 새 충돌 검사만의 회귀라고 단정하지 않되, 새 정책도 해결하지 못했다.
- r43: **기존 bbox 방식도5km/h에서는** 같은 분리 FOGGY11 drum 구간 끝에 도달했다.
  생성기 구간98.628초, native 접촉0/107조회다. 따라서 r40의98.873초 통과나 초반 고속 실패와의
  차이를 새 알고리즘의 순수 성능 개선으로 주장하지 않는다. 감속·배경 분리의 영향을 구분해야 한다.

같은 입력 fixture로 확인한 support 계산·시각 대응·보간 비용 개선과, 실제 시나리오 전체 통과는
서로 다른 근거다. 기존 방식과 후보 모두 성공한 drum 한 장면으로 전체 회피를 수락하지 않는다.

- r44: plastic을 drum 위치로 옮긴 배치 대조는 NORMAL에서 정지해 음영 후보 비교에 쓰지 않았다.
  JAMMING 토픽 미수신을 녹화 불완전으로 유지했다.
- r45: 같은 파생 배치를 valid-entry에서 시작했다. 실제 local `plastic_barrier` 관측과 연결됐고
  차로 변경을 시작했으나 회전 중 `contact_target_path_blocked`로 정지했다(native 접촉0/94조회).
  `turn-blocked-*` fixture는 해당 family/bbox를 보존한다. 이 결과는 입구 전환이 늦은 것만으로
  모든 실패를 설명할 수 없다는 근거다. 기존 후보 생성·재계획·추종의 결합도 비교해야 한다.
  물체 위치·도로 방향에 맞춘 yaw를 바꾼 파생 시험이며 원래 입구 장면의 해결로 집계하지 않는다.
- r46: 동일한 안쪽 plastic 배치의 저속 기존 방식도 `adjacent_lane_detected_or_path_invalid`648표본
  이후 정지했다(native 접촉0/66조회). 클래스/위치 하나만의 문제가 아니라 결합된 경로 검사가 남아 있다.
- r45 native 검사에서12m 후보의 차체 여유 모서리는 도로 경계를 약3.7cm 넘었다. 길이를0.5m 간격으로
  오프라인 대조하니15m 후보가 **동일한 여유·도로·장애물 검사**를 통과했다. 길이 전체 탐색은 실시간에
  넣지 않고, 기존147개가 실패한 뒤15m 후보만 추가했다. `turn-blocked-*` 회귀는148번째 후보의 통과를 확인한다.
- r47: 새 fallback 재시험은 NORMAL 진입에서 멈췄다. 음영 변경의 성공/실패로 집계하지 않았다.
- r48: 같은 장면 반복은 음영 제어에 진입해 이전 정지 위치보다 더 진행했으나 변경 중 다시 막혔다.
  새 계획 p95=1.315ms·최대3.380ms, native 접촉0/77조회다. 기존30초 변경 pause timeout도 유지했다.
  오프라인 가용 후보 복구는 확인했지만 이 장면의 전체 주행 해결은 아니다.
- r49: 최종 `6eda2523`의 분리 FOGGY11 drum 회귀는 구간 끝에 도달했다(생성기100.964초).
  계획1,652표본 p95=1.330ms·p99=1.790ms·최대3.453ms, native 접촉0/109조회다.
  projection 만료4·support2·ego-mask15개의 정지 표본은 남았다. 전체 bag의 bbox 수신 공백도
  262.3ms여서250ms 목표를 통과하지 못했다. r43의 저속 기존 방식 통과와 함께 해석한다.
- r50: 마지막 원래 입구 plastic 재시험은 NORMAL에서30초 정체 후 종료했다. JAMMING 토픽 누락을
  녹화 불완전으로 남겼으며, 새 음영 경로의 성공/실패 통계에 포함하지 않는다.

5km/h는 두 경로에 적용한 **목표 속도**다. NORMAL 제어의 과도 응답까지 실제5km/h로 고정한 것은
아니다. 후보는 fresh CAN 실제 속도의 시작/유지 제한을 별도로 검사하며, 초과 정지도 결과에 남긴다.

r28/r32의100.912/97.801초는 private driver의 주행 허용/정리 창이다. 정확한 생성기 구간 시간은
각 `events.jsonl`의 `run_ended.elapsed_s`로 확인한다. 준비·정리 시간을 처리 성능이나 순수 주행 시간에 섞지 않는다.

r23부터 분리 장면은 원래 정적 차량29를300m 옆으로 임시 이동하고, pose readback으로 위치·yaw를
확인해 매번 복원한다. 원래 다중 객체 장면의 성공으로 집계하지 않는다. `fixture-events.jsonl`이 근거다.
Ego·날씨·시간도 복원하고, 시험 소유 actor·프로세스만 종료한다. 원래 bridge/master는 유지한다.

원태 생성기의 접촉 판정은 소유 actor를 대상으로 한다. 추가 native query 기록에는 배경 객체 접촉도
담지만 저빈도/변경 시 샘플이며 연속 무충돌 GT가 아니다. `/CollisionData`의 type−1/id0 상시 sentinel은
충돌로 세지 않고 별도 미확정 수량으로 남긴다. 종료 시 `Channel closed!`도 기록에서 삭제하지 않는다.
GPS 무효 때 `/Ego_topic.position`의0 좌표로 주행 진척을 계산하지 않는다.

r10의 동결 manifest는 두 번째 split을 빠뜨렸고 r07–r17의 일부 첫 요약은 첫 split만 읽었다.
원본은 보존했다. `shadow-stop-fix-20261028-audit01/reanalysis`의 별도 분석과 r17의
`analysis-complete`를 사용한다. r18부터 종료를 기다리는 소유 프로세스 실행기로 재검사했다.
토픽 하나에 세 카메라가 교차 발행한 stamp는 뷰/session/generation별로 검사하며,
왼쪽 stamp를 앞 카메라 stamp와 비교해 역행으로 세지 않는다.

## 계산 비용과 회귀 근거

r26의 동일 native 입력에서 차선 구간 검색을 대조했다. 이어 붙인 차선의 x 정렬을 한 번 검증하고,
정렬된 경우 이진 검색을 쓴다. epsilon 선택·보간 산술은 그대로이며 비정렬 자료는 기존 선형 검색을 쓴다.
500회×3에서 선형 검색 p95=4.008/3.606/3.941ms, 변경 후2.004/2.205/2.041ms였다.
이는 단일 probe 비용이고 전체 주행 지연의 실측 개선으로 바꿔 말하지 않는다.
24,220개 보간 표본을 비교했으며 r24 좁은 경로의 전체 후보 출력도 바이트가 같았다
(SHA `dfa3947ea74db420c6783a3ee48cdd4b5672e286feaef3a537bbdd8740c64082`).
근거는 `shadow-stop-fix-20261028-audit01/interpolation`에 저장했다.

### 실제 JAMMING 시간창의 공급

`active-windows-final`은 기록된 `/planning/mode=1` 수신부터 생성기 종료/정지 요청까지를 따로
집계한다. 시작·끝의 무수신 시간도 포함하며 준비 구간을 섞지 않는다. mode 수신은 개별 명령의
selector 선택 증거를 대신하지 않는다.5초 창은 mode 전환 시각에 정렬하고 짧은 마지막 창은 제외한다.

| r40 최종 후보 | 완전한5초 창 최소Hz | 최대 수신 공백 |
|---|---:|---:|
| family |14.8|182.4ms|
| LiDAR local |11.8|124.7ms|
| bbox 차로 판정 |9.2|267.8ms|
| 음영 제어 명령 |20.0|89.9ms|

16개 창에서 bbox 판정은10Hz/250ms 목표를 만족하지 못했다. 전체 bag의335.9ms 공백과도 구분한다.
같은 저속 기존 방식 r43의 bbox는 최소10.2Hz·254.8ms였으므로 기존 경로도 공백 기준은 실패다.
빠른 계획 계산만 보고 입력 공급 문제를 해결했다고 표시하지 않는다. r41/42의 plastic 정지 동안에도
local은 최소11.8Hz, family는 최소15.0/14.8Hz, 제어 명령은20Hz였다. 정지의 직접 판단은 입력 전체
중단보다 경로/점유 거부였으며, 별도의 순간 공급 공백은 여전히 보완 대상이다.

`source-pairing-final`에서 r40의 음영 이후 bbox frame1,251개 중 정확히 같은 front projection이
기록된 것은929개, 없었던 것은322개다. 기존 방식 r43도1,246개 중929개만 일치했다.
각각 유효 결과는839/915개이고, projection에 없는 stamp를 유효하게 사용한 사례는0이다.
이 수량에는 마지막 부분 창이 포함된다. 독립 추론 경로의 원본 frame 선택 교집합과 후속 처리/무효
탈락을 구분한다. 약15Hz씩 나오는 두 결과가 자동으로15Hz의 동일 입력 묶음을 만드는 것은 아니다.
이를 개선하려면 공통 입력/투영 context의 생산 경계를 검토해야 하며, 가까운 다른 stamp를 정확한
원본인 것처럼 대신 사용하거나 freshness를 늘리는 방법은 적용하지 않는다.

## 참고한 기준과 사용하지 않은 것

[Coulter, CMU-RI-TR-92-01, §4.1](https://publications.ri.cmu.edu/storage/publications/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf)의
lookahead와 경로 수렴 관계를 대조했다. 짧은 후보 밖을 바라보던 추종점 선택을 수정했지만,
1.5–4m 범위는 우리 저속 실험값이다. 논문이 이 차량·값의 안전성이나 성능을 입증한 것으로 인용하지 않는다.


[Werling et al., ICRA2010](https://publikationen.bibliothek.kit.edu/1000022393)의 경로/시간 후보·
끝점 조건과, [Autoware obstacle avoidance](https://autowarefoundation.github.io/autoware.universe_planning/pr-5562/planning/obstacle_avoidance_planner/)의
reference/객체/차체 제한 분리를 대조했다. 현재 구현은 기존 경로 생성기의 제한된 후보 검사이며,
논문의 전체 최적 궤적 알고리즘을 이식하거나 그 논문의 성능을 재현했다는 의미가 아니다.
[Frustum PointNets](https://stanford.edu/~rqi/frustum-pointnets/)와
[Autoware projection fusion](https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_image_projection_based_fusion/)은
2D 검출과 실제3D 관측을 연결할 때의 깊이·대응 한계를 확인하는 참고다. 새 학습 모델은 사용하지 않았다.

## 배포와 남은 범위

최종 동작 소스 `6eda2523`에서 `.github/ci/lane.sh`를 독립 master11331로 실행했다.
순수 C++22개와 ROS59개, 총81개가 통과했다. 시나리오 전체 suite는379개가 통과했다.
전체 suite에서 드러난 기존 fixture import 오류는 `76ccc890`에서 수정했으며 수정 전 실패 로그도 보존한다.
문서 검사(26 launch·87 literal command), 역사 보존 검사, 기존145개 CI 검사 유지 확인도 통과했다.
실주행 중에는 전체 회귀·컴파일을 병행하지 않았다(r29의 초기 혼입은 별도 실패 기록).

컨테이너 전체 workspace와 변경 노드/시험 바이너리를 재빌드했다. 실행 근거는
`shadow-stop-fix-20261028-audit01/validation-final`에 보존한다. PR #125의 base는 아직 미병합인
PR #124 브랜치다. 현재 hosted workflow는 main 대상 PR만 실행하므로 **hosted CI 통과로 표현하지 않는다**.
최신 원격 main `54f6bb9a`는 이번 정리 중에도 같았다. 이 PR에서 기반 PR을 임의 병합·retarget하지 않는다.

시험 소유 프로세스만 종료하고 원래11311 master·bridge는 유지했다. 최종 readback에서 기존 actor
2/3/29의 pose와 날씨·시간은 처음 기록과 같고 추가 actor는 없다. Ego는 정지했으며 반복 reset 후
물리 안정화에 따른 약8cm·0.22도 차이를 `restoration-final`에 기록했다. 정확한 바이트 복원으로 주장하지 않는다.

- 새 실험 동작은 기본 OFF다. [실행 안내](../../../tools/scen_runner/docs/shadow-recording.md)의 실험
  환경값을 scenario planning supervisor 시작 전에 지정한다. 일반 autonomy 명령은 종전 동작이다.
- **private `lane_boundary_adapter/CameraBBoxLaneResult` MD5가 바뀐다.** classifier·follower·dashboard와
  관련 소비자를 같은 소스로 재빌드·재시작한다. 공용 `asmc_msgs`는 변경하지 않았다.
  옛 bag은 rosbag의 내장 정의로 오프라인 분석하거나 당시 revision에서 재생한다.
- `contact_clearance_enabled=false`, `local_contact_support_enabled=false`, `contact_local_required=false`가
  복구 경로다. scenario supervisor를 종료하고 세 실험 환경값을 해제한 뒤 다시 시작한다.
- `asmc_planner/src/jamming` 흡수, 일반 Frenet/MPC의 local 입력 경계, 벽 통로·출구 복귀,
  ego motion, 얇은 객체 복구와 정적 semantic 보조는 이번 실험과 분리된 후속이다.
- 좁은 후보가 존재한다는 오프라인 결과만으로 제어기가 그 경로를 추종하거나 전체 구간을 통과한다고
  인정하지 않는다. 실제 반복 주행·충돌 근거·진척·제어 공급을 함께 보고 기본 승격을 판단한다.
