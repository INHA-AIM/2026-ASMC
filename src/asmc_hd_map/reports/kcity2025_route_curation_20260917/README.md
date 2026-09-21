# K-City 2025 route curation stages 1-14

이 report는 전체 HD맵이 아니라 출발점부터 `A2256W000751` finish까지 검수된 차선 선택만 담은
부분 후보이다. 정본 결정과 후속 backlog는
[`docs/hd-map-route-curation-review-20260917.md`](../../../../docs/hd-map-route-curation-review-20260917.md)에
있다.

- implemented: stage01~`stage14_return_332_to_finish`
- route station: `[0.0, 2184.6117233360696]` m; 교차로와 중앙선의 실제 paint gap은 비워 둠
- stage01 links/source: `748→182→329→236`, origin-only legacy + A182 current-link curve correction
- stage02 links/source: `219→215→213`, Won-tae H5 vectors `5/6/21/22/23`
- stage03 links/source: `222→220`, white broken guide + Won-tae H5 vectors `7/8`
- stage04 links/source: `202(+009)→728→599→207`, Won-tae H5 `7/8/10/17` + legacy
  `B219BS010398` A009 outer solid
- stage05 links/source: `205→315`, Won-tae H5 vectors `10~15/17/18` + legacy
  `B219BS010382` lane-1 right broken divider; A207/A205 같은 solid 사이 짧은 수집 단절만 연결
- stage06 links/source: `318/319`, current-route 좌·우 white broken guide + H5-anchored A319 outer solid;
  mismatched legacy curve rejected
- stage07 links/source: `308(+309)`, Won-tae H5 vectors `19/20` + legacy `B219BS011450`
  lane-1 right broken divider + A309 current-link outer solid transition
- stage08 links/source: `083`, current-route pre-plan geometry를 white broken route guide로 교정
- stage09 links/source: `304(+307)`, Won-tae H5 vectors `25/28` 양 외곽 + 두 관측선의
  route-normal midpoint white broken 공유 divider
- stage10 links/source: `148→146→151`, Won-tae H5 vector 25/29 계열의 robust station profile +
  H5 vector 26 원형 yellow의 endpoint 고정 호장 평탄화; 정지선 뒤 noisy yellow tail은 맞은편
  H5 white에 평행하되 legacy `B219BS011472` 종점에서 끝나는 4.250m 선분으로 교체
- stage11 links/source: `866→054→846`, origin-only legacy corridor에서 비경로 `A057` guide 제외
- stage12 links/source: `411→420→408→445→153`, pre-plan white corridor 기반 +
  A445 원태 H5 횡오프셋 보정; far-left bus-only yellow는 제외
- stage13 links/source: `451→446→448→126→128→333→154`, A451 초반은 구맵
  톨게이트 섬 두 개, 합류 뒤~A446 중간은 Won-tae H5, 이후 A448까지는 pre-plan,
  음영구간은 검수된 legacy/pre-plan 경계 사용;
  bus-only yellow와 비도색 `A1256W000616` 정지선 제외
- stage14 links/source: `332→751`, origin-only legacy `B010411/472/474` finish corridor
- explicit corrections:
  - `B219BS010474` on `A2256W000748`, broken → solid
  - 곡률이 틀린 `B219BS010180`을 A182 current-link 곡률로 교체하고 legacy 종점 오프셋 유지
  - `B219BS011301` endpoint `0.618m` extension to the `B219BS011297` route cross-section
  - H5 vector 6 endpoint `0.220m` extension to vector 5 route cross-section
  - H5 yellow vectors `21/22` 사이 `2.278m` 실제 도색 공백 보존
  - A222 pre-plan left solid를 white broken intersection guide로 제한 변환
  - H5 vector 7 내부 되짚기와 vector 9/16 중복 조각 제외
  - A202의 H5 vector 8을 white broken 공유선으로 교정하고 A009 outer legacy solid 추가
  - A207 vector 17은 최대 `0.142m` gate 안에서 median route-normal offset으로 직선화
  - stage05 vector 10의 retrace를 직선화하고 vector 11/13/14의 짧은 retrace run 제외
  - H5 vector 11~14의 source yellow를 실제 white broken으로 교정하고 vector 10과 평행 정렬
  - A207→A205 yellow/white solid 수집 단절 `0.351m/0.152m`만 연결하고 solid→broken 연결 금지
  - `B219BS010382`를 `11.474m` 연장해 C123 정지선 단면까지 연결
  - A318에서 route를 가로지르는 `B219BS011450`을 제거하고 current 곡률의 좌·우 guide 사용
  - A318 right=A319 left 공유 점선은 하나의 geometry에 두 link ID를 귀속
  - A319 outer solid를 A310 H5 vector 18 종점의 실제 `1.465m` 오프셋에 연결
  - `B219BS011450`은 위치가 맞는 A308 구간에서만 solid → broken 내부 divider로 교정
  - A319 outer 종점부터 A309를 따라 H5 vector 20 시작까지 varying-offset white solid 연결
  - A308/A309 divider에 dash phase `3.746m`를 적용해 이전 guide와 점선 간격 연속성 유지
  - A304/A307의 H5 외곽선 간 관측 폭 median `6.964m`의 midpoint를 새 white broken으로 생성
  - H5 vector 25의 지글거리는 짧은 되짚기 run은 제외하되 첫 yellow anchor는 `0.606m`
    직선 연결로 보존
  - A148 출력 폭 p05/median/p95 `3.433/3.524/3.587m`; 폭이 큰 legacy right line 배제
  - 로터리 inner yellow vector 26은 최대 `0.137m`만 호장 방향으로 평탄화하고 endpoint 고정
  - 로터리 right white는 H5 `29/31/51/36/37/43`의 중복을 robust station median으로 통합
  - `A148→146` 정지선 뒤 vector 25 되짚기 꼬리를 제거하고 맞은편 white 기준 `3.434m`
    평행 offset의 `4.250m` yellow solid로 교체. 종점은 legacy `B219BS011472`의
    실제 도색 종점 `route_s=889.351m`에 고정
  - H5 중앙선 조각 사이 실제 공백 보존; legacy 중앙선은 적용하지 않음
  - stage11에서는 `B219BS010215` (`A057` 비경로 진입 guide)와 A150 yellow fork를
    제거하고, A866 yellow `B011499`를 white broken 시작 단면까지 `11.829m` 연장
  - A846 짧은 left `B011113`은 제거하고 아래 `B011490` anchor 차로폭 `3.530m`로 right와 평행하게 재생성해 노드까지 유지
  - stage12의 usable 1/2/3차로 경계와 A418 lane-1 left solid 5개를 `A411` 시작
    단면에 `0.147~0.328m` 이내로 정렬. 잘못 추가한 A411 constant-width solid은 제거하고
    원래 pre-plan `B011097` broken 및 다른 divider와 같은 `B011106` 시작 단면을 복원
  - A445 broken divider는 legacy `B010518`을 merge taper 전까지만 사용하고, right는
    `hi_right.csv` 폭 profile을 current link에 적용. A153 오른쪽 실선과의 동일 속성 공백만 연결
  - A153 legacy `B219BS010432` broken을 제거하고 current-link offset `+1.797m`로
    재생성. 첫 과잉 dash를 제거해 station `36.5m`부터 시작하고 right solid
    `-1.710m`와 같은 곡률을 유지하며 차로폭은 `3.507m`
  - A846→A411 right solid은 `0.0054m`만 연결하고 left solid→broken은 `0.162m` 횡단 단차 유지
  - A423+A425 공유선을 white solid로 복원하고 앞 solid와 `0.293m` handoff를 연결;
    B010447/518의 0.4m 미만 terminal dash 파편 제거
  - A179의 노드 초과 dash를 제거하고 A423 우측 `B010509`는 노드까지 연장
  - A445 right solid에 겹치던 `hi_right.csv+hi_right3.csv` broken trace 제거
  - 고주로 far-left bus-only yellow source는 denylist로 배제하여 stage12 출력 31개는
    모두 white(`broken=12`, `solid=19`)로 유지
  - 두 번째 톨게이트 섬을 관통하던 전체 재구성 solid를 제거하고, 노드부에만 pre-plan
    `A424+A432`를 사용해 이전 solid와 `0.024m`를 연결한 뒤 `4.140m` 접선 연속 구간으로
    legacy `B010446`에 혼합; `B010435/436→B010455/456`은 원본 공통 endpoint로 연결
  - A451 초반은 stage12 `B010433/434`, `B010435/436`과 stage13 `B010453/454`,
    `B010455/456`을 결합한 섬 두 개 및 `B010512` right를 유지;
    합류 뒤 left는 `B010438` 시작 anchor에서 H5 vector 53 right와 평행하게 재생성하고,
    right source 단차는 접선 연속 handoff로 연결
  - A446 초반 left는 H5 vector 62를 사용;
    `route_s=1820.599m`부터 A448까지 pre-plan 양 경계로 전환하고 음영구간 연결은 접선 연속 handoff 사용
  - A448 뒤 A126 양 경계와 A131 외곽 solid를 포함하고 A128 후반의 0.25m 내 단절 연결
  - A333/A154 왼쪽은 pre-plan `B011535` segment `01/00/02`의 점선 3개를 원본 길이로 유지
  - A333 다음 비도색 current stopline `A1256W000616`은 allowlist에 추가하지 않음
  - stage13→14 `B010472` handoff를 연결하고 `B010411/474`를 finish 단면까지 연장
- stopline: Won-tae H5 `A1256W000682/807/208/582/757/709`과 결합
  `A1256W000593+A1256W000742` 일곱 개만 포함
- superseded decision: 최신 주행 확인에 따라 `A308/A309` 끝 정지선의 이전 기각은 폐기
- surface: intentionally absent
- state: `partial_candidate`, `operational=false`
- inventory: lane marking 121개(`white=94`, `yellow=27`, `solid=90`, `broken=31`), stopline 7개

산출물:

- `route_curated_candidate_map.json`: 승인된 1~14단계 차선·정지선 vector
- `selected_markings.csv`: source, link, route station과 override 감사표
- `selected_stoplines.csv`: 선택된 정지선 allowlist와 route station
- `summary.json`: 개수와 상태
- `overlays/stage01_start_legacy.png`: 1단계 정적 검수 화면
- `overlays/stage02_new_road_219_to_213.png`: 2단계 정적 검수 화면
- `overlays/stage03_static_obstacle_intersection_222.png`: 3단계 정적 검수 화면
- `overlays/stage04_link_202_and_following_intersection.png`: 4단계 정적 검수 화면
- `overlays/stage05_dynamic_obstacle_205_to_315.png`: 5단계 정적 검수 화면
- `overlays/stage06_right_turn_318.png`: 6단계 정적 검수 화면
- `overlays/stage07_post_turn_308.png`: 7단계 정적 검수 화면
- `overlays/stage08_guide_083.png`: 8단계 정적 검수 화면
- `overlays/stage09_two_lane_expansion_304.png`: 9단계 정적 검수 화면
- `overlays/stage10_roundabout_approach_148.png`: 10단계 정적 검수 화면
- `overlays/stage11_roundabout_exit_to_high_speed.png`: 11단계 정적 검수 화면
- `overlays/stage12_high_speed_entry_and_body.png`: 12단계 정적 검수 화면
- `overlays/stage13_high_speed_tail_to_333.png`: 13단계 정적 검수 화면
- `overlays/stage14_return_332_to_finish.png`: 14단계 정적 검수 화면

RViz에서는 다음을 실행한다.

```bash
roslaunch asmc_hd_map route_curation_review.launch repo_root:=/root/ws
```

`A2256W000215` 교차로의 빈 구간과 `A2256W000213` 노란선 중간 공백은 의도된 결과다.
1~14단계 geometry는 모두 출력되지만 정식 operational map 승격 전의 partial candidate다.
