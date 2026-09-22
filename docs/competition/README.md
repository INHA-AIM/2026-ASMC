# 대회 환경·규정·공개 자산

> **문서 역할:** Canonical Summary — 팀 개발에 필요한 본선 v1.1 요약과 자산 목록
>
> **기준:** 규정집 v1.1, 2026-09-08. 주최 측 최신 규정집과 공식 공지가 항상 우선한다.

이 문서는 팀 저장소에 필요한 결론만 둔다. 규정 원문·Q&A·배포 링크는 저장소 상위의
`docs/competition/`에 보존하고, 구현 설정은 [운영 가이드](../operations/README.md)와
[HD Map 계약](../contracts/hd-map/topic-v1.md)을 따른다.

## 1. 본선 환경과 금지 경계

```text
[주최 Client PC: MORAI·판정·녹화] <-- Ethernet LAN2LAN --> [Autonav 팀 PC]
```

![본선 진행 환경](./images/스크린샷%202026-07-10%20124338.png)

| 항목 | 기준 |
|---|---|
| 주최 PC | i5-13600KF, RTX 4060 Ti, 32 GB, Windows 11 |
| 참가 PC | 팀 알고리즘 PC, OS 자유 |
| 외부 통신 | 주최가 허용한 UDP만 사용 |
| 팀 PC 내부 | ROS1 Noetic 사용 가능 |
| 판정 | Unity + UDP, 판정·녹화를 시뮬과 동시에 실행 |
| 주행 | 2회, 제한시간 15분 |
| 제출 후 | 코드·파라미터·주석 수정 불가 |
| 지연 제출 | 패널티 시간 부과, 구체치는 후속 공지 기준 |
| 현장 장비 | 휴대용 모니터·LAN cable 제공; keyboard·mouse 지참 |

한 조는 5팀의 독립 single-player 환경이며 전체 7개 조다. 당일 시간표는 공식 공지를 따른다.

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
| 3D LiDAR | VLP16 Intensity 1, 최대 15 Hz | v2는 10 Hz; 3단계 승인 목표 15 Hz |
| Camera | 고정 3 + 자유 1, 최대 30 Hz, GT None | 20/20/20/10 Hz |

고정 카메라:

| 카메라 | 위치 xyz (m) | 회전 rpy (deg) | 해상도 | FOV |
|---|---|---|---|---:|
| Front | 1.90, 0.00, 1.20 | 0, 2, 0 | 1280×720 | 90° |
| Left | 1.15, 0.65, 1.20 | 0, 10, 70 | 640×480 | 130° |
| Right | 1.15, -0.65, 1.20 | 0, 10, 290 | 640×480 | 130° |

- 맵: `R_KR_PG_K-City_2025`
- 대회 SIM: `25.S4.MolitComp03`
- 차량 제어: Ego Ctrl Cmd, `cmd type 1`, `ctrl mode 2`
- 상태: Competition Vehicle Status와 CollisionData

LiDAR 15 Hz와 현장 동일 포트는 승인된 후속 요구지만 아직 설정에 적용하지 않았다. 숫자를
JSON에서 추측해 바꾸지 않고 3단계 `asmc_bridge`에서 MORAI UI 저장본과 30초 이상 실측으로
검증한다.
규정의 10 Hz 이하 권장은 의무 상한이 아니며 공식 최대 15 Hz 안에서 팀 목표를 검증한다.

## 3. 공개 자산과 로컬 배치

대용량 원본은 Git에 넣지 않고 `$ASMC_DATA/R_KR_PG_K-city_2025/`에 둔다.

| 파일 | 의미 |
|---|---|
| `2026_molit_comp_cam_set.json` | 주최 공개 카메라 센서셋 |
| `2026_molit_comp_global_path.txt` | 전역경로 xyz 4,429점 |
| `2026_molit_comp_sample_scene.json` | 샘플 시나리오와 Ego spawn |
| `2026_molit_comp_checkpoints.json` | START/END + 1~15 pose·yaw |
| `link_set.json`, `node_set.json`, `global_info.json` | 기존 MGeo 원본 |

대회 당일 새 맵은 제시되지 않지만 시나리오는 달라질 수 있다. 샘플 Ego spawn은 대략
`(-131.49, -427.96, 28.88)`, yaw `62.5°`이며 본선 pose는 당일 시나리오를 따른다.

```bash
ls "$ASMC_DATA/R_KR_PG_K-city_2025/2026_molit_comp_"*
```

| 저장소 센서셋 | 용도 |
|---|---|
| `config/morai/sensors/ASMC_sensor_set_v2.json` | 26.R1.H3 현재 10 Hz 회귀 기준 |
| `config/morai/sensors/ASMC_sensor_set_v3.json` | 15 Hz 현장 후보, live acceptance 전 |
| `config/morai/sensors/ASMC_sensor_set_GT.json` | Semantic×3 + Instance LiDAR 개발 정답 전용 |
| `config/morai/sensors/ASMC_sensor_set_v1.json` | 과거 재현 전용 |

현장 자료의 `Autonav_sensor.json`은 이전 제출본 provenance이고 다른 팀 센서셋은 사용하지
않는다. v2와 이전 Autonav의 의미 있는 차이는 자유 카메라 pitch `328°` 대 `340°`이며 현재
실행 정본은 v2다. GT 센서셋은 본선 profile에서 사용할 수 없다.

## 4. 전역경로·체크포인트·HD Map

- 전역경로 길이: **2,184.61 m**
- 출발 성공 5% station: **109.23 m**
- 체크포인트: START/END와 1~15, 총 **16개**
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
| 신호 | 앞바퀴가 정지선을 넘는 시점의 신호 기준, 위반 15초 |
| 차로 | 바퀴 1개가 차선에 3초 지속 접촉할 때마다 5초; 차선변경 시간제한 없음 |
| 장애물 | CollisionData 충돌마다 15초 |
| GPS 음영 | GPS 전체 blackout; 해당 구역은 차로 준수 패널티 미적용 |
| NPC | 회전교차로·고속도로 외 구간에도 추가될 수 있음 |

하이패스 30 kph와 Foggy 감속은 별도 강제 규칙이 아니다. 팀의 안전 감속 정책과 공식
속도 채점 규칙을 구분한다.

일반 센서 noise는 인가하지 않지만 GPS 음영 미션의 blackout은 유지된다. 완주팀은 실제
주행시간+패널티의 2회 중 최단 기록으로 순위를 정한다. 미완주팀은 체크포인트 달성도와
총 주행시간 순이며 완주팀이 항상 우선한다.

주행 불능은 바퀴 네 개가 주행 영역을 벗어나거나, 충돌 후 복귀할 수 없거나, 체크포인트를
불연속 통과한 경우다. 승인된 체크포인트 복귀에는 15초 패널티가 붙는다. 시뮬레이터 또는
주최 하드웨어 문제는 해당 차시를 무효로 하고 재주행한다.

## 6. 당일 절차와 배포 전 gate

1. 주최 Client PC와 팀 PC를 LAN으로 연결한다.
2. 운영 요원이 scenario·sensor set을 load하고 시뮬레이션을 준비한다.
3. IP, route, UDP packet rate와 Ctrl 응답을 확인한다.
4. 알고리즘을 시작한 뒤 제출 규칙에 따라 수정·조작을 중단한다.
5. 판정 reset·시작, 주행과 점수 저장을 2회 수행한다.

- [ ] 허용 UDP 채널만 competition profile에서 기동
- [ ] Ctrl mode 2와 packet 목적지 확인
- [ ] Competition Vehicle Status·CollisionData 수신
- [ ] 카메라·LiDAR·GPS·IMU 포트와 rate 확인
- [ ] 체크포인트 16개, 3 m 반경과 순서 검증
- [ ] 속도 예외 링크 경계와 15분 timeout 검증
- [ ] GPS blackout에서 localization fallback 검증
- [ ] 판정·녹화 동시 부하에서 drop·latency·CPU/GPU 확인
- [ ] 인터넷 없이 image load→기동→복구 수행

실행 명령과 현재/후속 네트워크 값은 [운영 가이드](../operations/README.md), 전체 현장 배포
순서는 [통합 로드맵](../architecture/consolidation-roadmap.md)을 따른다.
