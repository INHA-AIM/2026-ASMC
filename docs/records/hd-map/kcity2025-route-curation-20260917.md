# K-City 2025 경로별 HD맵 선택 검수 — 2026-09-17

> **문서 역할:** Dated HD Map Audit Record — stage01~14 선택·보정 근거
>
> **기록 허브:** [records index](../README.md)
>
> **현재 구현:** [asmc_hd_map guide](../../packages/asmc_hd_map/README.md)와
> `src/asmc_hd_map/config/kcity2025_route_curation.yaml`을 우선한다.

## 목적과 고정 원칙

세 source를 자동으로 섞지 않고 실제 MORAI 2025 주행 화면을 기준으로 link 구간마다 선택한다.

- `legacy_origin_aligned`: 구 KATRI geometry에 origin translation만 적용
- `won_tae_h5`: 원태 커밋 `e67f5da`의 관측 lane/stopline vector
- `preplan`: 원태 관측 기준 정본화 계획 직전 hybrid candidate
- current Node/Link: 연결 topology와 route station 정본
- 공식 전역경로: 출력 구간과 교차로 guide 선택 기준

선택하지 않은 source를 평균하거나 spline으로 섞지 않는다. 위치 보정, pattern 변경, midpoint
생성은 아래에 명시된 link에서만 수행하고 provenance를 남긴다. 단계별 산출물은 계속
`operational=false`이며, 미구현 구간은 geometry를 발행하지 않는다.

사용자 문장의 `A2256W00219`는 존재하는 route link `A2256W000219`로 정규화했다. 첫 문장의
불완전한 `A2256W000`은 이어지는 설명에 따라 `A2256W000219` 진입 전으로 해석했다. 따라서
1단계는 `route_s=[0,186.286m)`, link `748→182→329→236`이고 `219`부터는 2단계다.

## 스크린샷 근거 등록

스크린샷은 2026-09-17 대화에 첨부된 원본을 육안 검수한 기록이다. 대화 이미지에는 저장소
파일 경로가 제공되지 않아 bitmap을 복제하지 않고, 재현 가능한 link ID와 관찰 내용을
evidence ID로 고정한다. 원본 파일을 별도로 전달받으면 동일 ID로
`reports/kcity2025_route_curation_20260917/evidence/`에 보존한다.

| ID | 화면에서 확인한 내용 | 적용 구간 |
|---|---|---|
| S01 | `A2256W000748`: 실제 route-right 흰선은 solid지만 origin-only legacy RViz는 broken으로 표시 | stage01 |
| S02 | 초기 교차로: 연속 실선보다 진행 방향 guide broken marking이 실제 도색과 일치 | stage01~03 |
| S03 | `A2256W000205/315` 주변: H5 중앙선·외곽선 위치가 정확하고 내부 흰 broken marking이 누락 | stage05 |
| S04 | `A2256W000318` 우회전: 교차로 내부 선은 broken guide | stage06 |
| S05 | S04와 같은 우회전 화면의 중복 캡처; 별도 독립 근거로 세지 않음 | stage06 |
| S06 | 고주로 다차로: usable 1/2/3차로와 far-left bus-only 경계, boundary 시작 station 확인 | stage12 |
| S07 | 고주로 합류부: `A2256W000411` 시작점과 left merge broken marking 필요 | stage12 |
| S08 | `A2256W000148` 로터리 접근: legacy 우측 폭 과대, H5 anchor는 정확하나 polyline wiggle 존재 | stage10 |
| S09 | `A2256W000304`: legacy 1차로와 달리 2025는 2차로이며 H5 양 외곽선 midpoint가 내부 divider 후보 | stage09 |
| S10 | `A2256W000329` 끝: route-right 흰선이 노란선보다 약 0.7m 짧아 같은 횡단 단면까지 연장 필요 | stage01 |
| S11 | `A2256W000219` 끝: H5 route-right 흰선이 노란선보다 짧아 같은 횡단 단면까지 연장 필요 | stage02 |
| S12 | `A2256W000213` 중간: H5 yellow 21/22 사이 공백은 데이터 누락이 아니라 실제 도색 공백으로 확인 | stage02 |
| S13 | `A2256W000222` 교차로: 중앙 유도선은 yellow가 아니라 white broken 도색 | stage03 |
| S14 | `A2256W000207` 끝: H5 route-right 흰선 위치는 맞지만 작은 횡방향 굴곡을 제거해야 함 | stage04 |
| S15 | `A2256W000205/315` U턴 도색은 흰 broken이며 중앙선과 평행, `C1256W000123` 아래 두 차로 정지선과 A315 divider 종점 확인 | stage05 |
| S16 | `A2256W000318`: legacy guide가 route를 가로질러 실제 우회전 곡률과 불일치하며, 실제 좌측 유도선은 current turn과 평행한 white broken | stage06 |
| S17 | `A2256W000207→205` 경계: U턴 broken 도색 전 yellow/white solid에 각각 짧은 수집 단절이 있으나 뒤의 broken 공백은 실제 도색 | stage05 |
| S18 | `A2256W000319→309`: 오른쪽 외곽 white solid는 연속이고, 내부 divider의 broken 간격은 이전 링크의 dash 위상을 이어야 함 | stage07 |
| S19 | `A2256W000418` 1차로 왼쪽 white solid 누락. `A2256W000153` left broken은 초반·중후반 기울기 변화를 right solid과 같이 따라야 함 | stage12 |
| S20 | `A846` 끝 legacy solid 단절, `A423` 경계 속성 오류와 두 terminal dash 파편, `A153` 첫 과잉 dash, 톨게이트 `A427` 좌측 solid 단절을 확인 | stage11~13 |
| S21 | `A148` 시작 양 경계 단절, `A866` 좌측 solid 단절과 비경로 `A150` yellow 파편, `A846` 종점 속성, `A423/427` tollgate solid, `A445` 중복 dash, `A451→446` 노드 전환, `A448→126/131` 음영 회피 차로, `A128` 후반 경계 단절을 확인 | stage10~14 |
| S22 | `A866` yellow 종점 단절, `A411` 끝 차로폭 축소, `A179` 노드 초과 dash, `A423` 우측 dash 조기 종료, 톨게이트 1차로의 `A427` 과잉 출력, `A451→446` 연결 불연속을 확인 | stage11~13 |
| S23 | `A451→A446` 초반 우측 외곽은 2025와 구맵 차이가 있어 H5가 적합하고, A446 중반부터 A448은 pre-plan이 적합함. A333 로터리 접근 왼쪽에는 pre-plan 점선 조각 3개가 필요 | stage13 |
| S24 | A451 초반은 구맵의 톨게이트 섬 두 개를 보존하고, 합류 뒤 left broken/right solid부터 H5·pre-plan으로 전환해야 함. 직전 A411 폭 유지 보정은 오지정이므로 롤백 | stage11~13 |
| S25 | 두 번째 톨게이트 섬 자체는 보존하되 `A432` 왼쪽 접근선은 white solid. A451의 섬 뒤 broken은 구맵 방향이 아니라 2025 right solid과 평행해야 하며, A451 right·A448 source handoff의 횡단 단차를 제거해야 함 | stage12~13 |
| S26 | 중간 검수에서는 A846를 legacy-origin-aligned 원본으로 롤백했으며, 이 판단은 다음 S27의 구체적인 차로폭·종점 지시로 대체됨 | stage11~12 |
| S27 | A846의 짧은 왼쪽 실선은 아래 `B011490` 차로폭에 맞춰 오른쪽으로 이동해 오른쪽 실선과 평행해야 함. 오른쪽 실선은 A411 right solid에 연결하되 왼쪽은 A846 끝 노드까지만 연장하고 A411 left broken과 횡단 단차를 보존 | stage11~12 |
| S28 | `A432` 왼쪽 white solid가 시작 노드에서 이전 링크와 횡단 단차 없이 이어져야 함. 노드부는 pre-plan, 이후 톨게이트 섬까지는 legacy를 혼합하되 섬 형상은 보존 | stage12 |

텍스트 주행 검수 결정도 별도 ID로 고정한다.

| ID | 결정 | 적용 구간 |
|---|---|---|
| R01C | `A182`의 legacy white broken guide는 실제 회전 곡률과 불일치. current A182 곡률을 사용하되 legacy의 시작·끝 횡오프셋과 종료 station은 유지 | stage01 |
| R04 | `A202` 계열은 H5 위치가 정확하고, 다음 정지선 뒤 교차로는 H5가 끝난 뒤 비워야 하며 `A207`에서만 다시 경계를 시작 | stage04 |
| R04C | `A202` 1차로 오른쪽=`A009` 왼쪽 공유선은 white broken. 구맵 `B219BS010398`의 `A009` 우측 white solid를 A202 끝까지만 추가 | stage04 |
| R06 | `A318` 교차로에는 좌·우 white broken guide가 모두 있고, 오른쪽 guide는 `A319`의 왼쪽 경계와 공유. `A319`의 바깥쪽 흰 실선도 유지 | stage06 |
| R06A | `A319` 바깥선은 1.75m 명목 폭이 아니라 앞선 `A310` H5 vector 18 종점의 1.465m 법선 오프셋으로 연결 | stage06 |
| R07 | `A308`은 H5 19/20으로 좌·외곽을 고정하고 legacy `B219BS011450`은 이 link의 내부 divider 구간에만 사용 | stage07 |
| R07S | 최신 주행 확인에서 `A308/A309` 끝의 두 차로 정지선을 확인. 이전 비도색 판단을 폐기하고 H5 결합 선분 `A1256W000593+A1256W000742`를 사용 | stage07 |
| R08 | `A083` 교차로에는 전역경로 방향의 white broken guide만 필요. pre-plan의 곡률은 사용하되 yellow solid 속성은 폐기 | stage08 |
| R05C | A207/A205의 같은 색·같은 solid 사이 `0.351m` yellow, `0.152m` white 단절만 연결. solid→broken 경계는 어떤 경우에도 bridge하지 않음 | stage05 |
| R07C | A319 외곽 실선을 A309 current 경계로 H5 vector 20 시작까지 잇고, A308/A309 공유 divider에는 앞선 guide의 dash 위상을 계승 | stage07 |
| R09 | `A304/A307`의 H5 vector 25/28을 양 외곽 정본으로 쓰고 그 route-normal midpoint만 2025 내부 white broken divider로 생성 | stage09 |
| R10 | `A148`은 H5 vector 25/29의 robust station profile로 폭을 유지하고, 로터리 원형 yellow vector 26은 route-normal이 아닌 원 polyline 호장 평탄화 적용. H5 정지선 709만 허용 | stage10 |
| R10C | `A148→146` 정지선 뒤 vector 25의 되짚기 꼬리는 폐기. 정지선까지 직접 관측선을 유지하고 이후는 맞은편 H5 white solid에 평행하게 유지하되 legacy `B219BS011472` 중앙선의 실제 종점(`route_s=889.351m`)에서 끝냄 | stage10 |
| R10D | stage09→10 source handoff에서 A148 yellow/white profile을 `route_s=858.307m` 단면까지 각각 `0.445/0.695m` 연장해 시작점 단절을 제거 | stage10 |
| R11 | `A866→054→846`은 origin-only legacy corridor를 그대로 사용하되 route로 진입하지 않는 `A057` guide만 제거 | stage11 |
| R11A | `A150` 비경로 fork source `B011489/497`을 제거하고 A151→A866 left solid의 `3.35m` 수집 공백만 연결. A846 right `B011112`는 끝 노드까지 연장하고, 짧은 left `B011113`은 제거한 뒤 아래 `B011490` anchor 차로폭 `3.530m`로 right와 평행하게 재생성 | stage11 |
| R11B | A866 왼쪽 yellow `B011499`는 기존 법선 오프셋을 유지한 채 white broken 시작 단면까지 `11.829m` 연장 | stage11 |
| R12 | `A411→420→408→445→153` 고주로는 pre-plan white corridor를 기반으로 하고 far-left bus-only yellow를 제외. A411 usable 1/2/3차로 경계를 같은 시작 단면에 맞추고 left merge `B011097`은 원래의 white broken으로 유지. A445 양쪽은 H5 관측에 맞추며 right는 보정된 A153 right와 꼭지점에서 연결 | stage12 |
| R12A | `A418+A429` pre-plan 공유 white solid을 1차로 왼쪽 경계로 복원. A153 legacy broken `B219BS010432`의 초반 파고듦(0.02→1.80m offset)을 폐기하고, 기존 중앙 위치 `+1.797m`는 유지하면서 right solid과 평행한 current-link broken으로 교체 | stage12 |
| R12B | A846 right solid과 A411 right solid 사이 `0.0054m`만 연결. A846 left solid은 노드에서 끝내고 A411 left `B011097`은 white broken으로 유지해 `0.162m` 횡단 단차를 보존. `B010447/518` terminal dash 파편도 제거 | stage11~12 |
| R12C | `A423+A425` 공유 경계를 white solid로 복원하고 앞 solid와 `0.293m` handoff를 연결. A445 right solid에 겹친 H5 broken trace는 제거해 `B010518`이 merge taper 전에 끝나도록 제한 | stage12 |
| R12D | `B011100`은 A179 끝 노드에서 잘라 초과 dash를 제거하고, A423 우측 `B010509`는 끝 노드까지 유지. A432 terminal로 잘라 두었던 `B010454/455`는 stage13의 완전한 톨게이트 섬 형상으로 승격하되 A427 topology는 제외 | stage12~13 |
| R12E | 두 번째 섬을 관통하던 별도 재구성 solid는 제거. `A432` 왼쪽 `B010446`은 2025 white solid로 교정해 분기점까지 유지하고, `B010435/436→B010455/456` 실선 네 개는 원본 공통 endpoint로 정확히 연결 | stage12~13 |
| R12F | 이전 `A423+A425` white solid에서 pre-plan `A424+A432` 시작점까지 `0.024m`만 닫고, 노드 뒤 pre-plan 형상을 짧게 사용한 다음 `4.140m` 접선 연속 구간으로 legacy `B010446`에 혼합. 이후 섬 분기점까지 legacy와 `B010435/436→B010455/456` 원본 형상을 보존 | stage12 |
| R13 | `A451→446→448→126→128→333→154`는 pre-plan route/usable-lane 경계만 선별. bus-only yellow는 제외하고 `A333` 뒤 2025 비도색 정지선 `A1256W000616`은 출력하지 않음 | stage13 |
| R13A | A451 초반은 구맵 `B010453/454`와 `B010455/456` 두 섬 및 route-right `B010512`를 유지하되 A427 topology는 부여하지 않음. 섬 뒤 left broken은 `B010438` 시작 횡오프셋만 anchor로 삼아 H5 vector 53 right solid과 평행하게 재생성 | stage13 |
| R13B | H5 vector 53은 A451의 구맵 합류점부터 A446 중간(`route_s=1820.599m`)까지 사용하고 A446 초반 왼쪽은 H5 vector 62를 사용. 그 뒤 A448 끝까지는 pre-plan 양 경계로 전환하며, A451 right와 A448 양 경계 source handoff는 2m+2m 접선 연속 cubic으로 연결 | stage13 |
| R13C | A448 뒤 A126 route 양 경계와 A131 반대차로 outer solid를 포함하고 A128 후반의 0.25m 내 source handoff를 연결. A333/A154 왼쪽은 pre-plan `B011535` segment 01/00/02의 원본 점선 조각 3개를 모두 유지 | stage13 |
| R13D | A451 left는 `B010438` anchor와 H5 right의 검수 폭 `3.832m`를 유지한 white broken parallel profile. A153 right→`B010512`는 `0.355m`만 연결하고 `B010512`→H5의 `0.203m` 횡단 단차는 접선 연속 blend로 대체 | stage13 |
| R14 | `A332→751`은 origin 정합 legacy `B010411/472/474`를 사용하고 yellow-left와 white-broken-right를 finish 단면까지 0.5m gate 안에서 연장 | stage14 |

## 구간별 결정과 작업 순서

| 단계 | current route link | 결정 | 상태 |
|---:|---|---|---|
| 01 | `748→182→329→236` | origin-only legacy 기반. `A182` guide만 current 곡률로 교체, `748` solid·`329` 종점 교정 | **구현·검수 반영** |
| 02 | `219→215→213` | 원태 H5 5/6/21/22/23. `219` 끝점 정렬, `213` yellow 실제 공백 보존, H5 정지선 682/807 허용 | **구현·검수 반영** |
| 03 | `222→220` | `222`에는 white broken guide 하나만, `220`부터 H5 7/8의 직접 관측 solid 경계 사용 | **구현·검수 반영** |
| 04 | `202(+009)→728→599→207` | A202/A009 shared broken·A009 legacy outer solid 추가. H5 7/8·10/17과 정지선 208 유지 | **구현·검수 반영** |
| 05 | `205→315` | H5 center/outer + legacy lane-1 right broken. U-turn은 white broken 평행 보정, 같은 solid 수집 단절만 연결 | **구현·검수 반영** |
| 06 | `318(+319)` | 좌·우 white broken guide와 H5 vector 18에 anchor한 `A319` 바깥 white solid 사용 | **구현·검수 반영** |
| 07 | `308(+309)` | H5 19/20 + lane-1 right legacy broken + A309 outer 연결 + H5 두 차로 정지선 | **구현·검수 반영** |
| 08 | `083` | pre-plan route-left 곡률을 white broken guide로 교정 | **구현·부분 후보** |
| 09 | `304(+307)` | H5 외곽 기준 midpoint divider 생성. legacy의 옛 1차로 white solid 사용 금지 | **구현 후보** |
| 10 | `148→146→151` | H5 anchor robust profile. 정지선 뒤 noisy tail은 맞은편 white와 평행하고 legacy 중앙선 종점에서 끝나는 4.250m yellow로 교체. H5 정지선 709만 포함 | **구현·검수 보정** |
| 11 | `866→054→846` | legacy 사용, 비경로 guide/fork 제거, A846 종점 차로폭 3.530m의 평행 실선으로 보정 | **구현·검수 보정** |
| 12 | `411→420→408→445→153` | pre-plan 기반 고주로. bus-only yellow 제거, A411 오지정 롤백, A179/A423 종점 보정, A445 H5 정렬 | **구현·검수 보정** |
| 13 | `451→446→448→126→128→333→154` | A451 구맵 톨게이트 섬 2개 뒤 H5, A446 중간 이후 pre-plan, A427 제외, A333 왼쪽 점선 3개 복원 | **구현·검수 보정** |
| 14 | `332→751` | origin-only legacy로 finish 복귀, 양 끝 paint를 finish 단면까지 정렬 | **구현 후보** |

세부 source policy와 모든 사용자 주의사항은
[`kcity2025_route_curation.yaml`](../../../src/asmc_hd_map/config/kcity2025_route_curation.yaml)이
기계 판독 가능한 정본이다. 계획 상태의 항목은 builder가 읽더라도 geometry를 만들지 않는다.

## 1~14단계 구현 계약

- 전체 구현 route interval: `[0.0, 2184.6117233360696]` m
- 1단계 route interval: `[0.0, 186.28573174794127)` m
- 1단계 포함 link: `A2256W000748`, `A2256W000182`, `A2256W000329`, `A2256W000236`
- geometry source: origin-only legacy A/B reference, 단 `A182` left guide는 current link 곡률 사용
- explicit override: `B219BS010474`, `A2256W000748`, `broken→solid`
- endpoint correction: `B219BS011301` 흰선을 `B219BS011297` 노란선의 route 단면까지
  `0.618m` 연장. 기존 횡 offset은 유지하고 노란선을 줄이지 않음
- `A182`: 곡률이 맞지 않는 legacy `B219BS010180`은 제외하고 current link left guide로 교체.
  legacy guide가 실제 도색과 맞았던 시작·끝 법선 오프셋 `2.090→1.366m`를 선형 보간하고
  link station `18.250m`에서 끝내 다음 legacy yellow와 약 0.1m 안에서 접속
- stopline: stage02의 `A1256W000682`, `A1256W000807`, stage04의
  `A1256W000208`, stage05의 `A1256W000582/757`, stage07의
  `A1256W000593+A1256W000742`, stage10의 `A1256W000709`만 포함
- drivable surface/lane area: 이번 단계에서 생성하지 않음
- output status: reviewed partial candidate, `operational=false`

2단계는 `route_s=[186.286,315.906)m`, link `219→215→213`이다.

- geometry source: 원태 push H5의 저장 vector만 사용
- 선택 vector: `third_point.csv`의 5(yellow), 6(white),
  `fourth_point.csv`의 21+22(yellow), 23(white)
- `A2256W000219` 끝 H5 white vector 6을 yellow vector 5와 같은 route 단면까지
  `0.220m` 연장하며 기존 횡 offset 유지
- `A2256W000213` yellow vector 21/22 사이 `2.278m`는 실제 도색 공백이므로 연결하지 않음
- H5에 없는 `A2256W000215` 교차로 구간은 비워 두며 pre-plan의 연속 실선을 가져오지 않음
- guide 도색은 현재 H5에 직접 관측된 geometry가 없으므로 이번 단위에서 추정 생성하지 않음
- 원태 H5의 collection `LEFT/RIGHT`는 canonical 좌우 소유관계로 복사하지 않음
- `A2256W000213` 끝은 원태 H5의 `A1256W000807` 정지선이 route station
  `315.906m`에 정확히 일치하므로 원태 geometry 그대로 사용
- `A2256W000219` 끝도 원태 H5의 `A1256W000682`가 route station
  `208.459m`에 일치하므로 추가
- 다른 H5/구맵 정지선과 반대차로 차선은 이 단계 출력에 없음

3단계는 `route_s=[315.906,365.776)m`, link `222→220`이다.

- `A2256W000222`: pre-plan의 좌·우 연속 실선을 모두 제거
- current topology 기반 pre-plan left 곡선만 geometry reference로 잘라
  `3m/3m` white broken guide로 변경
- `A2256W000220`: 원태 H5 `5.csv`의 vector 7(yellow), 8(white)을 직접 사용
- A222에 white solid를 만들지 않으며, A220 이후 vector가 stage04로 넘어가지 않게 link 끝에서 자름
- stage02가 진입 정지선을 소유하므로 stage03에서 정지선을 중복 생성하지 않음

4단계는 `route_s=[365.776,471.092)m`, link `202→728→599→207`이다.

- `A2256W000202`: 원태 H5 `5.csv` vector 7은 yellow solid 좌측 경계로 사용
- H5 vector 8은 `A202` 구간에서 white broken으로 교정하고 `A202` 오른쪽=`A009` 왼쪽
  공유 경계로 두 link ID에 귀속
- `A009` 오른쪽 외곽은 origin 정합 구맵 `B219BS010398` white solid를 사용하되
  `route_s<427.852m`, 즉 `A728` 분기 시작 전까지만 clip
- H5 vector 8의 `A728` 이후 직접 관측 조각은 기존 white solid로 유지
- `A2256W000728` 끝 route station `436.798m`: 원태 H5 정지선 `A1256W000208` 사용
- 정지선 뒤에도 직접 관측된 vector 7/8 끝(`route_s≈441m`)까지만 유지
- 그 뒤 `A2256W000599` 교차로에는 pre-plan 연속 실선을 만들지 않음
- `A2256W000207`: H5 `6.csv` vector 10(yellow), 17(white)만 다시 시작
- vector 17 흰선은 관측 station 범위를 유지한 채 median route-normal offset으로만 직선화
  (`p95=0.114m`, 최대 보정 `0.142m`, 허용 gate `0.20m`)
- vector 7 내부의 짧은 되짚기 run과 vector 9/16의 중복 retrace 조각은 제외
- stage05 `A2256W000205`로 넘어가는 geometry는 link 경계에서 자름

5단계는 `route_s=[471.092,550.619)m`, link `205→315`이다.

- 중앙선과 U턴부: 원태 H5 `6.csv` vector `10~15`를 각각 독립 선분으로 사용
- vector 10의 수집 순서 되짚기는 station 정렬 후 median route-normal offset으로 직선화하며
  최대 보정은 `0.119m`로 `0.25m` gate 안이다
- source에서 yellow로 저장된 vector `11~14`는 실제 U턴 흰 broken paint로 속성을 교정하고,
  vector 10과 같은 route-normal offset(`1.917m`)을 사용해 중앙선과 평행하게 맞춘다.
  최대 geometry 보정은 `0.191m`이며 원 관측 색과 출력 색을 감사 필드에 모두 보존한다
- vector `11/13/14`의 짧은 retrace run은 버리고 가장 긴 전진 관측 run만 남김
- vector `10~15` 사이의 물리적 공백은 연결하지 않음
- A207 끝과 A205 시작의 같은 paint 사이 수집 단절만 별도 feature로 닫는다. yellow solid
  `0.351m`, white solid `0.152m`이며, 두 bridge 모두 같은 color/pattern을 강제한다.
  뒤따르는 U턴 white broken이나 legacy divider로 pattern 경계를 넘는 연결은 금지한다
- 2차로 외곽: 원태 H5 vector `18` white solid 사용
- 1차로 우측 divider: origin-only legacy `B219BS010382` white broken만 보완하고
  횡 offset을 유지한 채 `11.474m` 연장해 `route_s=550.619m` 정지선 단면까지 닫음
- 구맵 yellow `B219BS010393/011448/011449`는 위치가 다른 옛 중앙선이므로 사용하지 않음
- H5 vector 17의 A205 진입 조각은 stage04와 같은 제한 직선화를 적용한 뒤 legacy divider
  시작 전까지만 사용하며, 어느 source도 관측 범위 밖으로 extrapolation하지 않음
- `C1256W000123` 아래 정지선은 원태 H5의 `A1256W000757`(route lane)과
  `A1256W000582`(인접 lane) 두 선분을 geometry 변경 없이 포함한다

6단계는 `route_s=[550.619,580.868)m`, route link `A2256W000318`과 인접
`A2256W000319`이다.

- legacy `B219BS011450`은 이 구간에서 법선 오프셋이 `+0.97m→-1.78m`로 바뀌어
  route를 가로지르므로 geometry reference에서 제외
- current route reconstruction의 좌·우 guide를 A318 구간에만 clip한다. A318 중심선 대비
  법선 오프셋 중앙값은 각각 `+1.75m`, `-1.75m`이며 전체 변화폭은 `0.053m` 이하다
- 양쪽 source와 출력 모두 white broken(`3m/3m`)이며 pattern 변환 없이 current 곡률만 채택
- 오른쪽 guide는 물리적으로 `A318` 오른쪽=`A319` 왼쪽 공유 경계이므로 하나의 geometry에
  두 current link ID를 귀속한다
- `A319` 바깥 경계는 명목 `1.75m`를 사용하지 않는다. 앞선 `A310`의 직접 관측 H5
  vector 18 종점을 A319 시작 단면에 투영해 얻은 오른쪽 `1.465m`를 anchor로 사용한다.
  anchor의 종방향 차이는 `0.035m`이며 current link와 H5 hash를 감사 필드에 기록한다
- 세 선 모두 A318/A319 끝 이후로 연장하지 않음

7단계는 `route_s=[580.868,609.877)m`, link `A2256W000308`과 인접
`A2256W000309`이다.

- 원태 H5 `7.csv` vector 19: yellow solid 좌측 경계
- 원태 H5 `7.csv` vector 20: white solid 2차로 외곽 경계
- origin 정합 legacy `B219BS011450`은 A308에 대응하는 구간만 잘라 누락된 1차로 우측
  white broken divider로 사용하고, source solid→broken 변경을 override ID로 감사한다.
  A309 구간까지 같은 공유 경계로 귀속하고 앞선 A318 guide 길이와 링크 사이 간격을 합친
  dash phase `3.746m`를 적용해 시작부의 어색한 짧은 dash를 제거한다
- A319 바깥 white solid 종점과 H5 vector 20 시작 사이 `1.987m`는 A309 current link의
  오른쪽 법선 오프셋 `1.465→1.527m`를 사용한 별도 white solid로 닫는다. 양 끝 잔차는
  각각 `0.10m` 이하다
- H5 직접 관측 시작 전의 짧은 공백은 extrapolation하지 않음
- 최신 실주행 검수에서 두 차로를 가로지르는 실제 정지선을 재확인했으므로 이전 기각을
  명시적으로 폐기한다
- 원태 H5 결합 선분 `A1256W000593+A1256W000742`(`7.554m`)를 geometry 변경 없이 사용하고
  `A308/A309` 두 current link에 공동 귀속한다

8단계는 `route_s=[609.877,635.113)m`, link `A2256W000083`이다.

- pre-plan `roundabout_exit_to_high_speed_2025:left`의 current-route 곡률만 reference로 사용
- source의 yellow solid 속성은 실제 교차로 도색과 맞지 않으므로 명시적 override ID를 남기고
  white broken(`3m/3m`) guide로 변경
- 전역경로 방향 guide 하나만 출력하며, 반대·비주행 유도선이나 연속 solid 경계는 넣지 않음
- A083 current link 대비 법선 오프셋 중앙값은 `1.750m`, 변화폭은 `0.041m`이므로 도색과
  시각적 곡률 차이는 있어도 link topology 곡률과는 일치하는 것으로 판정해 유지

9단계는 `route_s=[635.113,858.502)m`, route link `A2256W000304`와 인접
`A2256W000307`이다.

- 2025 확장 도로이므로 legacy의 옛 단일차로 white solid는 사용하지 않음
- 원태 H5 `ro_merge3.csv` vector 25는 A304 왼쪽 yellow solid, vector 28은 A307 오른쪽
  white solid로 각각 귀속
- vector 25 시작부의 `3.755m` 되짚기/지글거림 run은 geometry로 사용하지 않되 원태의
  첫 관측 anchor `[1.0440, 116.4134]`는 보존한다. 이 anchor에서 가장 긴 정상 run까지
  `0.606m` 직선 연결해 노란선 시작점과 매끈함을 함께 유지한다
- 두 외곽 관측선을 같은 route station의 공통 법선에 투영하고 midpoint를 1차로 오른쪽=
  2차로 왼쪽 white broken(`3m/3m`) 공유 경계로 생성한다
- 관측된 전체 2차로 폭은 median `6.964m`(p05 `6.901m`, p95 `7.184m`)이고 midpoint의
  route offset median은 `-1.734m`다. 생성 divider는 A304/A307에 공동 귀속한다
- stage09에는 legacy geometry, 반대차로 또는 불필요한 정지선을 넣지 않음

10단계는 `route_s=[858.502,914.837)m`, route link `A2256W000148→146→151`이다.

- A148 접근부 yellow left는 H5 vector 25, white right는 vector 29부터 이어지는 직접 관측을
  0.5m station bin robust median으로 축약한 뒤 각각 5m/3m 길이로 평탄화한다
- 정지선 전 첫·마지막 유효 observation bin의 offset은 고정한다. 내부 smoothing 최대 보정은
  0.25m gate보다 작게 제한하고, 마지막 관측 bin에서 정지선 단면까지의 짧은 연장량도
  0.50m gate로 별도 감사한다
- 출력 A148 차로폭은 p05/median/p95 `3.433/3.524/3.587m`다. 우측선을 더 밖으로 두는
  legacy geometry는 출력하지 않는다
- 로터리 내부 원형 yellow는 H5 vector 26을 사용한다. 이 선은 route와 평행하지 않으므로
  route-normal 평균을 금지하고, 원 polyline의 호장을 따라 1m triangular moving average만
  적용한다. 최대 이동은 `0.137m`이고 양 endpoint는 고정한다
- 로터리 오른쪽 white solid는 H5 vector `29/31/51/36/37/43`의 중복 trace를 station-bin
  median으로 통합한다. H5 vector 38~42의 짧은 되짚기 조각과 vector 27의 A151 비대응
  조각은 사용하지 않는다
- 접근 yellow와 원형 yellow 사이, 원형 yellow와 출구 사이의 H5 미관측 구간은 자동으로
  연결하지 않는다
- vector 25의 정지선 뒤 되짚기 관측은 중앙선을 안쪽으로 휘게 하므로 geometry에서 제외한다.
  직접 관측 yellow는 `A1256W000709` 정지선 단면에서 끝내고, 그 뒤 `4.250m`만 맞은편
  H5 white solid의 route-normal profile을 `3.434m` 평행 이동한 별도 yellow solid로 둔다.
  이 선은 구맵 중앙선 `B219BS011472`의 실제 도색 종점인
  `route_s=889.351m`에서 끝나며 inner circular yellow까지 자동 연결하지 않는다
- stage09에서 넘어오는 A148 시작 단면은 yellow/white robust profile을
  `route_s=858.307308m`까지 각각 `0.445/0.695m` 연장해 두 경계의 source handoff를 닫는다
- 정지선은 원태 H5 allowlist의 `A1256W000709`(`3.5m`, route_s=`885.101m`)만 사용한다.
  pre-plan/current의 다른 로터리 정지선과 비주행 선은 포함하지 않는다

11단계는 `route_s=[914.837,1118.742)m`, route link `A2256W000866→054→846`이다.

- geometry source: origin translation만 적용한 legacy corridor
- route에 대응하는 yellow/white solid와 white broken marking은 legacy pattern/geometry 그대로 유지
- `B219BS010215`는 `A2256W000057`에서 들어와 `A2256W000054`를 가로지르는 비경로 guide이므로
  명시적 denylist로 제외한다
- 비경로 `A2256W000150`의 yellow fork인 `B219BS011489/011497`도 제외한다
- A151 H5 white solid와 A866 legacy `B219BS011494` 사이 `3.35m`만 같은 solid로 연결한다
- A866 왼쪽 yellow `B219BS011499`는 고정 횡오프셋을 유지해 white broken 시작 단면까지
  `11.829m` 연장한다
- A846 아래쪽 왼쪽 `B219BS011490` 종점을 anchor로 사용한다. 짧은 `B219BS011113`은
  제거하고, 오른쪽 `B219BS011112`와 평행하면서 anchor 차로폭 `3.529540m`를 유지하는
  white solid을 생성한다. 시작점은 `B011490` 종점에 정확히 일치하고 A846 끝 노드에서 끝난다
- 오른쪽 `B219BS011112`는 A846 끝 노드까지 연장한다. A411 right `B219BS011096`과 남는
  `0.005407m`만 같은 solid connector로 닫는다
- 다음 고주로 시작 link `A2256W000411`의 paint는 stage12에서 별도 승격한다

12단계는 `route_s=[1118.742,1741.721)m`, route link
`A2256W000411→420→408→445→153`이다.

- pre-plan의 고주로 곡선과 solid/broken 표현을 기본으로 사용하되 far-left
  bus-only yellow source는 출력하지 않는다. 검수 보정 feature를 포함한 최종 31개 marking은
  전부 white이며 `broken=12`, `solid=19`이다
- A846 left solid은 노드 단면에서 끝내고 A411 left `B219BS011097`은 white broken으로
  유지한다. 둘 사이 `0.162400m` 횡단 단차는 의도적으로 연결하지 않는다
- `A411` 진입 단면의 usable 1/2/3차로 경계와 `A418` lane-1 left solid 등
  5개는 같은
  `route_s=1118.741751m`에서 시작한다. 원본 시작점으로부터의 연장은
  `0.147/0.328/0.172/0.253/0.321m`로 0.50m gate 안이다
- `A411` 오지정 보정은 롤백한다. left merge는 pre-plan `B219BS011097` white broken을
  사용하고, 다음 구간의 `B219BS011106`도 다른 divider와 같은 원래 시작 단면
  `route_s=1281.157m`부터 시작한다
- `A445`의 legacy broken divider `B010518`은 solid merge taper 전에 끝내고, 그 위에
  겹치던 원태 `hi_right.csv+hi_right3.csv` broken trace는 제거한다. right solid은 current A445에 `hi_right.csv` 관측의
  station별 offset profile을 적용하고, 종점 offset을 1.710m로 맞춰 이전에
  검수한 A153 right solid와 하나의 꼭지점으로 수렴시킨다
- 두 white solid 사이의 남은 `0.333m` 공백은 속성이 같은 경우에만 연결하며,
  solid/broken 전환을 bridge하지 않는다
- `A153` legacy broken `B219BS010432`는 초반 offset이 `0.02m`에서 `1.80m`로
  바뀌며 right solid과 기울기가 달라지므로 제거한다. 대신 A153 current link
  station `36.5~123.0m`에 left offset `1.797m`를 적용해 right solid offset
  `-1.710m`와 동일한 곡률의 white broken을 생성한다. 검수 차로폭은 `3.507m`다
- `A423+A425` 공유 경계는 white solid로 복원하고 앞선 `A421+A435` solid와 남은
  `0.293m` source handoff만 연결한다. `B219BS010447/518`은 길이를 각각
  `39.922/39.895m`로 제한해 다음 dash cycle에서 생기던 `0.383/0.390m` 점 파편을 출력하지 않는다
- `B219BS011100`은 A179 끝 노드에서 잘라 노드를 넘는 마지막 dash를 제거하고,
  A423 우측 `B219BS010509`는 A423 끝 노드까지 유지한다
- 두 번째 섬을 관통하던 전체 `A2256W000424+A2256W000432` 재구성 solid는 제거한다.
  이전 `A423+A425` white solid에서 pre-plan `A424+A432` 시작점까지 남는 `0.024m`를
  같은 solid로 닫고, 노드 뒤 pre-plan은 짧게 유지한다. 이어지는 `4.140m` 구간에서
  접선 연속 cubic으로 legacy `B219BS010446`에 혼합한 뒤 2025 white solid로 교정한
  legacy 형상을 분기점까지 유지한다.
  `B219BS010435/436` solid fork를 원본 전체 길이로 복원한다. 이 두 선의 종점은 stage13
  `B219BS010455/456` 시작점과 정확히 일치한다. `A2256W000427` topology는 출력하지 않는다
- stage12에는 yellow source와 drivable surface를 포함하지 않는다

13단계는 `route_s=[1741.721,2132.231)m`, route link
`A2256W000451→446→448→126→128→333→154`이다.

- route 양쪽, usable adjacent lane, safety-zone outer/inner boundary만 명시적으로
  선택하며, source 연결을 포함한 출력은 28개다. `A427` topology와 고주로 far-left
  bus-only yellow `B219BS011265`, `B219BS010519`, `LM000002`는 제외한다
- 톨게이트 섬은 stage12 `B219BS010433/434`, `B219BS010435/436`의 upstream fork와
  stage13 `B219BS010453/454`, `B219BS010455/456`의 downstream close를 원본 공통
  endpoint로 결합해 두 개 모두 완전한 형상으로 유지한다. route-right는 구맵
  `B219BS010512`를 사용한다
- 두 섬이 합류하는 `route_s≈1759.2m` 뒤에는 raw `B219BS010438`의 시작 위치만 anchor로
  사용하고 H5 vector 53 right solid과 `3.832m` 평행한 white broken을 생성한다. 오른쪽
  B010512→H5 사이 `0.203m` 횡단 단차는 양쪽 2m 접선 연속 blend로 바꾼다.
  A446 초반 왼쪽은 H5 vector 62 yellow다
- A446 중간부터 A448 끝(`route_s=1877.745m`)까지는 pre-plan current reconstruction의
  yellow-left/white-right를 사용한다. H5와 pre-plan 사이 같은 속성 공백은 각각
  `0.164/0.166m`로 연결한다. A448→음영구간의 양쪽 횡단 단차는 각각 양쪽 2m를
  사용한 접선 연속 cubic으로 치환한다. 이후는 검수된
  legacy `B219BS011264/010443/011267/010473/011260/010472`와 `B219BS011535`
  white broken guide를 사용하며 `LM000001` A131 외곽 solid도 포함한다. A333/A154
  왼쪽은 `B219BS011535`의 pre-plan segment `01/00/02` 세 조각을 원본 길이로 모두
  유지한다. A128 후반의
  `0.245/0.250m` source handoff는 같은 색·pattern끼리만 연결한다
- `A2256W000333` 다음 current stopline `A1256W000616`은 2025에 도색되지
  않았으므로 stopline allowlist에 추가하지 않는다

14단계는 `route_s=[2132.231,2184.612]m`, route link `A2256W000332→751`이다.

- origin 정합 legacy `B219BS010411` yellow solid, `B219BS010472` white solid,
  `B219BS010474` white broken만 사용한다
- stage13 `B010472`와 stage14 같은 source 사이 `0.243m` solid handoff를 연결한다
- yellow left와 white broken right는 각각 `0.117/0.240m`만 연장해 finish route 단면에서 끝낸다

`A2256W000076`은 현재 공식 route curation corridor 밖이고, 원태 H5 direct vector 중
해당 link에 20m 이내로 대응되는 선분도 없다. 따라서 이번 후보에 임의 U턴선을 생성하지
않는다. 별도 비경로 maneuver feature로 넣으려면 그 위치의 `HDMAP_REVIEW x/y`, 화면 또는
원태 source vector ID를 먼저 고정한 뒤 `review_required` 단위로 구현한다.

차선 source 선택과 정지선 allowlist는 독립 검수한다. 따라서 승인된 일곱 정지선 외에
이전 후보의 13개 current stopline이 조용히 재유입되지 않는다.

빌드:

```bash
python3 src/asmc_hd_map/scripts/build_route_curation.py
```

RViz 주행 검수:

```bash
roslaunch asmc_hd_map route_curation_review.launch repo_root:=/root/ws
```

정적 오버레이는 stage별로 `stage01_start_legacy.png`,
`stage02_new_road_219_to_213.png`, `stage03_static_obstacle_intersection_222.png`,
`stage04_link_202_and_following_intersection.png`,
`stage05_dynamic_obstacle_205_to_315.png`, `stage06_right_turn_318.png`,
`stage07_post_turn_308.png`, `stage08_guide_083.png`,
`stage09_two_lane_expansion_304.png`, `stage10_roundabout_approach_148.png`,
`stage11_roundabout_exit_to_high_speed.png`, `stage12_high_speed_entry_and_body.png`,
`stage13_high_speed_tail_to_333.png`, `stage14_return_332_to_finish.png`로 분리했다.
1~14단계 geometry가 모두 구현됐으며, 한 단계의 실패가 다음 구간
source를 바꾸지 않도록 각 단계는 별도 overlay, inventory와 regression test를 유지한다.
