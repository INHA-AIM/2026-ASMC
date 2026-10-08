# 대회 환경·규정·공개 자산

> **문서 역할:** Canonical Summary — 팀 개발에 필요한 본선 v1.2 요약과 자산 목록
>
> **기준:** 규정집 v1.2, 2026-09-30 (v1.0 07-11, v1.1 09-08) 및 교차로 신호준수 채점 후속 공지
> (팀 반영일 2026-10-06, 공지 게시일 미제공). 주최 측 최신 규정집과 공식 공지가
> 항상 우선한다.

v1.2 변경: 일정 추가, 체크포인트 15개로 정리, 네트워크 스펙(판정 전용 채널·CVS 미제공 필드) 명확화,
시뮬레이터 성능 고정, 신호·차로 판정 기준 보완.

이 문서는 팀 저장소에 필요한 결론만 둔다. 규정 원문·Q&A·배포 링크는 저장소 상위의
`docs/competition/`에 보존하고, 구현 설정은 [운영 가이드](../operations/README.md)와
[HD Map 계약](../contracts/hd-map/topic-v1.md)을 따른다.

장애물 출제 에셋·시나리오 높이·Object Info 속도는
[2026-10-06 전달 Q&A와 분류 설계](../handoffs/competition/obstacle-assets-and-labels.md)를 참고한다.
F2 전체 객체가 범위이며 `OBJ_G80_Open_Door`는 제외라는 답변과,
미검수 102개 집계·후속 학습 제안을 구분한다.

## 1. 본선 환경과 금지 경계

```text
[주최 Client PC: MORAI·판정·녹화] <-- Ethernet LAN2LAN --> [Autonav 팀 PC]
```

![본선 진행 환경](./images/스크린샷%202026-07-10%20124338.png)

| 항목 | 기준 |
|---|---|
| 주최 PC | i5-13600KF, RTX 4060 Ti, 32 GB, Windows 11 |
| 참가 PC | 데스크톱·서버·워크스테이션·노트북, OS 자유 |
| 외부 통신 | 2-8의 허용 UDP만 사용, 그 외 사용 시 실격 |
| 팀 PC 내부 | ROS1 Noetic 사용 가능 |
| 판정 | Unity + UDP, 판정·녹화를 시뮬과 동시에 실행 |
| 주행 | 2회, 제한시간 15분 |
| 제출 후 | 코드·파라미터·주석과 실행 명령 인자 수정 불가, 네트워크 IP·Port만 예외 |
| 지연 제출 | 등록·PC 제출 시간 초과 시 각 주행 차시마다 30초 패널티 |
| 현장 장비 | 휴대용 모니터·LAN cable 제공; keyboard·mouse 지참 |

각 팀은 독립 single-player 환경에서 주행한다. 10/30 당일은 08:30~09:30 등록·PC 제출,
오전조 14팀 09:30~12:30, 오후조 20팀 13:30~17:30, 17:30~18:30 결과 집계이며, 앞 팀이 끝나 공석이
생기면 다음 팀이 준비한다.

| 일정 | 내용 |
|---|---|
| 09-30 | 규정 설명회·본선 순번 추첨 (온라인) |
| 10-01 ~ 10-16 | 중간 점검 2차 (온라인) |
| 10월 4주차 | 센서 파일 제출 및 검수 |
| 10-29 / 10-30 / 10-31 | 현장 세팅 / 개최식·본선 / 시상식 |

본선 profile에서 rosbridge, gRPC, Semantic/Instance/Object GT와 기타 privileged 입력을
실행하지 않는다. 센서 지연과 packet 유실은 팀 책임이므로 판정·녹화 부하를 포함해 rate,
staleness, latency와 자원 사용량을 측정한다.

## 2. 차량·센서·맵

차량은 `2023_Hyundai_ioniq5`다.

| 항목 | 값 |
|---|---:|
| wheelbase | 3.000 m |
| 최대 wheel angle | 40° |
| 최소 회전반경 | 5.87 m |
| 전/후 overhang | 0.845 / 0.790 m |
| L × W × H | 4.635 × 1.892 × 2.434 m |

| 센서 | 수량·규정 | 현재 팀 개발 기준 |
|---|---|---|
| GPS | 1, 최대 30 Hz | 5 Hz |
| IMU | 1, 최대 50 Hz | 50 Hz |
| 3D LiDAR | VLP16 Intensity 1, 최대 15 Hz | v3 설정·손재호 소비 loop 15 Hz; 실제 scan 약 12 Hz 허용 |
| Camera | 고정 3 + 자유 1, 최대 30 Hz, GT None | 20/20/20/10 Hz |

고정 카메라:

| 카메라 | 위치 xyz (m) | 회전 rpy (deg) | 해상도 | FOV |
|---|---|---|---|---:|
| Front | 1.90, 0.00, 1.20 | 0, 2, 0 | 1280×720 | 90° |
| Left | 1.15, 0.65, 1.20 | 0, 10, 70 | 640×480 | 130° |
| Right | 1.15, -0.65, 1.20 | 0, 10, 290 | 640×480 | 130° |

- 맵: `R_KR_PG_K-City_2025`
- 대회 SIM: `25.S4.MolitComp03`, System Time Scale 1·Target FPS 60, Rendering Quality High 고정
- 차량 제어: Ego Ctrl Cmd, `cmd type 1`(accel/brake), `ctrl mode 2`만 허용
- 상태: Competition Vehicle Status와 CollisionData
- **판정 전용:** Ego Detail Status, Ego Vehicle Status, Get TrafficLight Status는 판정 프로그램만
  쓰며 본선에서 참가팀은 사용할 수 없다. 팀 bridge는 26r1 호환을 위해 수신할 수 있지만 runtime
  노드가 소비하면 안 된다 ([v1.2 영향](../handoffs/competition/rule-v1.2-impacts.md)).
- Competition Vehicle Status 미제공: `pos_x/y/z`, `vel_y/z`, `accel_x/y/z`, 타이어 횡력·side slip·
  cornering stiffness
- 사용하지 않는 센서 네트워크는 Disconnect할 수 있다.

현장 동일 포트·IP 파생 계약과 v3 15 Hz UI 저장본은 적용됐다. 다만 v3는 30초
실제 완성 scan은 약 12 Hz로 관측됐고 팀 요구상 허용된다. 없는 scan을 복제하지 않으며
packet/latency/resource와 손재호 tracking 품질을 계속 관찰한다.
규정의 10 Hz 이하 권장은 의무 상한이 아니며 공식 최대 15 Hz 안에서 팀 목표를 검증한다.

## 3. 공개 자산과 로컬 배치

대용량 원본은 Git에 넣지 않고 `$ASMC_DATA/R_KR_PG_K-city_2025/`에 둔다.

| 파일 | 의미 |
|---|---|
| `2026_molit_comp_cam_set.json` | 주최 공개 카메라 센서셋 |
| `2026_molit_comp_global_path.txt` | 전역경로 xyz 4,429점 |
| `2026_molit_comp_sample_scene.json` | 샘플 시나리오와 Ego spawn |
| `2026_molit_comp_checkpoints.json` | 규정집 v1.2 START/END + 1~14 pose·yaw (이전 16점은 `_20260803_16pt.json`) |
| `link_set.json`, `node_set.json`, `global_info.json` | 기존 MGeo 원본 |

대회 당일 새 맵은 제시되지 않지만 시나리오는 달라질 수 있다. 샘플 Ego spawn은 대략
`(-131.49, -427.96, 28.88)`, yaw `62.5°`이며 본선 pose는 당일 시나리오를 따른다.

```bash
ls "$ASMC_DATA/R_KR_PG_K-city_2025/2026_molit_comp_"*
```

| 저장소 센서셋 | 용도 |
|---|---|
| `config/morai/sensors/ASMC_sensor_set_v2.json` | 26.R1.H3 현재 10 Hz 회귀 기준 |
| `config/morai/sensors/ASMC_sensor_set_v3.json` | 설정·소비 15 Hz 본선본, 실제 scan 약 12 Hz 허용 |
| `config/morai/sensors/ASMC_sensor_set_GT.json` | Semantic×3 + Instance LiDAR 개발 정답 전용 |
| `config/morai/sensors/ASMC_sensor_set_v1.json` | 과거 재현 전용 |

현장 자료의 `Autonav_sensor_submitted.json`은 이전 제출본 provenance이고 다른 팀 센서셋은 사용하지
않는다. v2와 이전 Autonav의 의미 있는 차이는 자유 카메라 pitch `328°` 대 `340°`다. 현재
본선 정본은 같은 pose에서 LiDAR 설정을 15 Hz로 저장한 v3이며, v2는 10 Hz 회귀용이다. GT
센서셋은 본선 profile에서 사용할 수 없다.

## 4. 전역경로·체크포인트·HD Map

- 전역경로 길이: **2,184.61 m**
- 출발 성공 5% station: **109.23 m**
- 체크포인트: START/END와 1~14, 총 **15개** (v1.2에서 2026-08-03 공지의 11번 삭제, 12~15 재번호)
- 통과 판정: 각 공식 pose의 반경 **3 m** 이내
- 별도 MGeo 경로 생성은 허용되지만 체크포인트를 순서대로 경유해야 한다.
- 실제 본선 시나리오는 당일 운영 요원이 load하며 2회에 같은 시나리오를 사용한다.

![공식 체크포인트](./images/스크린샷%202026-07-13%20162644.png)

체크포인트 좌표·원본 yaw·route station·link와 미션 상태는
`src/asmc_hd_map/config/kcity2025_competition_metadata.yaml`과 `/hd_map/global/*` 토픽이
팀 구현 정본이다. 공개 JSON과 생성 bundle의 좌표·개수 parity를 유지한다.

## 5. 미션·채점 요약

![대회 주행 경로](./images/스크린샷%202026-06-30%20111239.png)

| 항목 | 규칙 요약 |
|---|---|
| 출발 | 1분 안에 경로 5%를 통과하지 못하면 주행 실패 |
| 속도 | 기본 60 kph. `A2256W000411` 시작→`A2256W000153` 끝은 고속 예외 |
| 과속 | 초과 즉시 15초, 이후 3초 지속마다 15초 추가 |
| 신호 | 초록 또는 좌회전 신호에 맞게 주행; 앞바퀴가 정지선을 넘는 시점의 신호로 판정. 미준수 시 **30초**; 판정 대상 **4개 교차로 모두** 미준수하면 해당 차시 실패. 동일 교차로는 1회만 집계(체크포인트 복귀 후 재통과 포함) |
| 차로 | 흰·노란 실선과 중앙선; 바퀴 1개가 3초 지속 접촉할 때마다 5초 (Ego Detail Status 바퀴 위치); 차선변경 시간제한 없음 |
| 장애물 | CollisionData 충돌마다 15초 |
| GPS 음영 | GPS 전체 blackout; 해당 구역은 차로 준수 패널티 미적용; 구역 내 랜덤 미션은 장애물 회피 |
| NPC | 회전교차로·고속도로 외 구간에도 추가될 수 있음 |

하이패스 30 kph와 Foggy 감속은 별도 강제 규칙이 아니다. 팀의 안전 감속 정책과 공식
속도 채점 규칙을 구분한다.

올해 한시적으로 GPS·IMU noise는 인가하지 않지만 GPS 음영 미션의 blackout은 유지된다.
카메라 Compression Ratio는 자유다. 완주팀은 실제
주행시간+패널티의 2회 중 최단 기록으로 순위를 정한다. 미완주팀은 체크포인트 달성도와
총 주행시간 순이며 완주팀이 항상 우선한다.

주행 불능은 바퀴 네 개가 중앙선·실선 등 주행 영역을 벗어나 경로를 완전히 이탈하거나(GPS 음영
구역 예외), 충돌 후 주행할 수 없거나, 체크포인트를 불연속 통과한 경우다. 체크포인트는 각 위치
반경 3 m 안으로 통과해야 한다. 승인된 체크포인트 복귀에는 15초 패널티가 붙는다. 시뮬레이터 또는
주최 하드웨어 문제는 해당 차시를 무효로 하고 재주행한다.

### 5.1 교차로 신호준수 후속 공지 — 2026-10-06 반영

근거는 팀원이 전달한 `[교차로 신호준수 채점 기준 업데이트 공지]`와 업데이트된 규정집 v1.2
붙여넣기 원문이다. 별도 버전 번호나 공지 게시일은 제공되지 않았으므로 v1.3으로 표기하지 않는다.
첨부 원문 SHA-256: `aa67ad550f7fa21ecf7ac046b974921f9d4986125c185c94485589e21bf07b7b`.

| 항목 | 변경 전(폐기된 신호 채점 기준) | 현재 적용 기준 |
|---|---|---|
| 미준수 패널티 | 15초 | **30초** |
| 해당 차시 주행 실패 | 빨간불임에도 감속 없이 임의로 미션을 건너뛰는 주행 | 3-1 신호등 구간의 판정 대상 **교차로 4개소 모두** 신호 미준수 |
| 중복 집계 | 이번 공지에서 명확화 | 동일 교차로의 미준수는 1회만 집계; **체크포인트 복귀 후 재통과도 포함** |

업데이트된 첨부 규정의 해당 문구:

> 미준수 시, 30초 패널티
>
> 판정 대상 교차로 4개소 (3-1 중 신호등 구간)의 신호를 모두 미준수한 경우, 해당 차시는 주행 실패
>
> 동일 교차로의 미준수는 1회만 집계
> (체크포인트 복귀 후 재통과하는 경우 포함)

앞바퀴가 정지선을 넘는 순간의 신호로 판정하며 초록 또는 좌회전 신호에 맞게 주행한다는 조건은
유지된다. 변경은 신호 채점 기준에 한정되며 과속·충돌·승인된 체크포인트 복귀의 15초 패널티는
변경하지 않는다. 미션·체크포인트를 임의로 건너뛰면 실패한다는 일반 규정도 유지된다.
새 채점 기준이 신호를 무시하는 주행을 허용한다는 뜻은 아니다.

실패 조건은 총 위반 이벤트 4회가 아니라 **서로 다른 판정 대상 교차로 4개소의 위반 여부**다.
공식 판정 교차로 4개소와 현재 HD맵의 신호 경계 5개를 동일시하지 않는다. 전달된 텍스트에는
4개소의 링크/정지선 ID 대응이 없으므로 그림·공식 근거 확인 전 임의 매핑하지 않는다.
중복 위반의 패널티 누적 세부 구현 역시 단순 이벤트마다 30초로 추정하지 않고 공식 판정과 대조한다.

PR #64 후속 구현은 HD맵 source·생성 metadata·발행용 payload에 새 규칙을 반영한다. 다만
공식 4개소의 대응은 `unverified`, 교차로 ID는 `null`, `intersection_failure_evaluation_available=false`다.
**규칙 제공과 자동 채점 구현은 다르다.** 판제의 실제 위반 집계기는 이번 변경에 포함하지 않는다.
신호 이외 규칙의 점검 결과와 파트별 확인 항목은
[신호 채점 인계](../handoffs/competition/rule-v1.2-impacts.md#6-신호-채점-후속-공지--2026-10-06-반영)를 따른다.

## 6. 당일 절차와 배포 전 gate

1. Operator 1명이 PC를 수령해 주최 Client PC와 LAN으로 연결한다.
2. 운영 요원이 미션 시나리오를 load하고 Ego를 keyboard 모드·P기어로 둔다.
3. 운영 요원이 센서 세팅 파일을 load하고 네트워크를 연결한 뒤 Operator와 함께 확인한다.
4. 운영 요원이 판정 프로그램을 준비하고 Operator는 알고리즘 실행을 준비한다.
5. Operator가 코드를 실행하면 이후 PC를 조작할 수 없다. 판정 프로그램은 주행 시작에 맞춰 시작한다.
6. 주행 종료 후 판정 데이터를 저장하고 2번부터 다음 차시를 준비한다.

- [ ] 허용 UDP 채널만 competition profile에서 기동
- [ ] Ctrl mode 2와 packet 목적지 확인
- [ ] Competition Vehicle Status·CollisionData 수신
- [ ] 카메라·LiDAR·GPS·IMU 포트와 rate 확인
- [ ] 체크포인트 15개, 3 m 반경과 순서 검증
- [ ] runtime 노드가 Ego Vehicle Status 파생 토픽(`/Ego_topic`)을 소비하지 않음
- [ ] 모든 `/ctrl_cmd` 송신 노드가 `longlCmdType=1`만 사용
- [ ] planning이 `route_speed_limits`의 effective 상한(60 구간 최대 60 km/h)을 지킴
- [ ] 속도 예외 링크 경계와 15분 timeout 검증
- [ ] 신호 30초·서로 다른 4개소 모두 미준수 시 실패·복귀 후 동일 교차로 중복 제외와 HD맵 판정 교차로 대응 검증
- [ ] GPS blackout에서 localization fallback 검증
- [ ] 판정·녹화 동시 부하에서 drop·latency·CPU/GPU 확인
- [ ] 인터넷 없이 image load→기동→복구 수행

실행 명령과 현재/후속 네트워크 값은 [운영 가이드](../operations/README.md), 전체 현장 배포
순서는 [통합 로드맵](../architecture/consolidation-roadmap.md)을 따른다.
