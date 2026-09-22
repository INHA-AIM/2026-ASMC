# 대회 환경·규정 (본선 v1.1)

> **문서 역할:** Canonical Summary — 팀 개발용 규정 요약. 주최 측 최신 원문·공지가 최우선
> **문서 허브:** [README.md](../README.md)
> **규정집 v1.1** (2026.09.08) · 팀원 전원 **필독**
> 본선 데이터: [competition-data.md](./assets.md)

## 1. 참가 PC 요건

| 항목 | 요구사항 |
|------|----------|
| 형태 | **데스크톱 PC 및 노트북** |
| OS | **자유** |
| 지참 | 알고리즘 개발된 PC 직접 지참 |

### 팀 개발 vs 대회

| 구분 | 대회 당일 | 팀 일상 개발 |
|------|-----------|--------------|
| OS | **자유** | WSL2 + Docker **또는** Ubuntu 20.04 |
| 통신 | **UDP only** (허용 외 실격) | UDP + ROS (내부 변환) |
| Python | 팀 PC 기준 | **3.8** (Noetic·ML 공통) |

**팀 정책**
- **WSL + Docker:** 코드·catkin build
- **네이티브 / 대회 PC:** UDP 통합, 판정 프로그램 연동, 최종 성능 측정
- **중간점검 8/18~9/4(온라인)**, **현장점검 9/14~9/18**에서 연습용 판정 프로그램 LAN 테스트

## 2. 대회 당일 제약

| 규칙 | 내용 |
|------|------|
| 판정 프로그램 | **당일 제공** (현장 점검 시 **연습용**) |
| 주행 기회 | **2회** |
| PC 제출 후 | **코드·파라미터·주석 수정 불가** |
| PC 지연 제출 | **패널티 시간** (구체치 추후 공지) |
| 현장 장비 | 휴대용 모니터·**이더넷 LAN선** 제공, **키보드·마우스** 지참 |

### 성능·지연

- Client PC에서 시뮬·판정·**녹화 프로그램을 함께 실행**하므로 센서 수신 지연 가능
- 센서 사양 하향 프로파일과 코드 경량화를 준비하고 drop·age·p95를 부하 상태에서 확인
- 센서/네트워크 **지연은 운영측에서 고려하지 않음** — **팀 책임**
- 판정: **Unity + UDP**

## 3. 본선 진행 환경

```
[Client PC] ←── 이더넷 (LAN to LAN) ──→ [참가팀 PC]
```

| PC | 사양/역할 |
|----|-----------|
| **Client PC** (주최) | i5-13600KF, RTX 4060 Ti, 32GB, Win11 — 시뮬 + **판정·녹화 프로그램** |
| **참가팀 PC** | 자율주행 알고리즘 |

- 조별 **5팀**, 각자 **싱글플레이** 독립 환경
- **7개조** (당일 타임테이블은 로컬 `docs/competition/regulations/02-competition-environment.md` 참고)

![본선 진행 환경](./images/스크린샷%202026-07-10%20124338.png)

## 4. 차량·센서·맵

### 차량: 2023_Hyundai_ioniq5

| 항목 | 값 |
|------|-----|
| Min Turning Radius | 5.87 m |
| Max Wheel Angle | 40° |
| Wheelbase | 3.000 m |
| Front / Rear Overhang | 0.845 / 0.790 m |
| L × W × H | 4.635 × 1.892 × 2.434 m |

### 센서 (본선 v1.1)

| 센서 | 개수 | 비고 |
|------|------|------|
| GPS | 1 | UDP, max 30 Hz |
| IMU | 1 | UDP, max 50 Hz |
| 3D LiDAR | 1 | **VLP16**, Intensity, max 15 Hz (10 Hz 이하 권장), UDP |
| Camera | 4 | **고정 3 + 자유 1**, max 30 Hz, GT None, UDP |

#### 고정 카메라 3대 (변경 불가)

| | pos (m) | rot (deg) | 해상도 | FOV |
|---|---------|-----------|--------|-----|
| Front | 1.9, 0, 1.2 | 0, 2, 0 | 1280×720 | 90º |
| Left | 1.15, 0.65, 1.2 | 0, 10, 70 | 640×480 | 130º |
| Right | 1.15, −0.65, 1.2 | 0, 10, 290 | 640×480 | 130º |

센서셋 JSON: [competition-data.md](./assets.md) — `2026_molit_comp_cam_set.json`

### 맵

**R-KR_PG_K-City_2025** · 시뮬 **25.S4.MolitComp03**

### 제어·네트워크 (UDP)

| 항목 | 값 |
|------|-----|
| 제어 | `Ctrl_cmd`, **cmd type 1** (accel, brake) + **ctrl mode 2** |
| 상태 | CollisionData, Competition Vehicle Status (자세·x방향 속도·각속도·제어 입력 등) |
| **실격** | 허용 UDP 외 사용, 센서 스펙 위반 |

## 5. 미션·채점 요약

![주행 경로](./images/스크린샷%202026-06-30%20111239.png)

| 미션 | 핵심 |
|------|------|
| 출발 | 1분 내 경로 **5%** 미통과 → **주행 실패** |
| 속도 | 60 kph. 예외는 **`A2256W000411` 시작점→`A2256W000153` 끝점**. 초과 즉시 15초, 지속 3초마다 +15초. 하이패스 30·Foggy 감속 **별도 의무 없음** |
| 신호 | **앞바퀴**가 정지선을 넘는 시점의 신호, 미준수 15초 |
| 차로 | 바퀴 1개 기준 차선 접촉이 3초 지속될 때마다 5초. 차선변경 **시간제한 없음**. **GPS 음영 구역은 채점 미적용** |
| 장애물 | 충돌마다 15초 (CollisionData) |
| 끼어들기 / NPC | 회전교차로·고속도로 외에도 NPC **추가 가능** |
| GPS 음영 | GPS 전체 Blackout, 랜덤 미션, 해당 구역 **차로 준수 패널티 미적용** |
| 제한시간 | **15분** |

**순위:** 총 주행시간 = 실제 + 패널티. 완주팀은 **2회 중 최단**. 미완주팀은
2회 중 **체크포인트 달성도 → 총 주행시간** 순이며, 완주팀이 항상 우선한다.

**센서 교란 구분:** 2026년에는 GPS·IMU를 포함해 일반 센서 Noise를 인가하지 않는다.
단, 미션으로 명시된 GPS 음영 구역의 GPS 전체 Blackout은 그대로 적용된다. 일반 Noise
미인가를 Blackout 폐지로 해석하지 않는다. 상세 Q&A는 로컬
`docs/competition/faq-official.md`.

전역 경로: `2026_molit_comp_global_path.txt` — [competition-data.md](./assets.md)

## 6. 주행 불능·체크포인트

![체크포인트](./images/스크린샷%202026-07-13%20162644.png)

| 상황 | 처리 |
|------|------|
| 주행 불능 | CP 복귀 (승인, **15초 패널티**) |
| 시뮬/하드웨어 이슈 | 차시 무효, 재주행 |

**불능 기준:** 바퀴 4개가 중앙선·실선 등 주행 영역을 벗어나거나 경로를 이탈한 경우,
정적 장애물·지형지물 충돌로 복귀할 수 없는 경우, CP를 불연속 통과한 경우다. 차로
접촉 패널티의 바퀴 1개 기준과 구분한다.

## 7. 당일 운영 절차 (요약)

1. Operator PC 수령·세팅
2. 운영: 시나리오 Load, 센서셋 Load, keyboard+P
3. 네트워크 확인 → 알고리즘 실행 (**이후 조작 불가**)
4. 판정 Reset·시작 → 주행 → 점수 저장
5. 2회차 반복

## 8. 개발 체크리스트

- [ ] UDP 센서·제어 (본선 허용 채널만)
- [ ] Ego Ctrl UDP **cmd type 1 + ctrl mode 2** packet capture 검증
- [ ] 속도 예외 링크 `A2256W000411`→`A2256W000153` 경계 테스트
- [ ] 체크포인트 순차 통과·미완주 `last_checkpoint`/`total_time` 로그 보존
- [ ] 판정+녹화 부하 가정 센서 저부하 프로파일·drop/latency 검증
- [ ] `2026_molit_comp_cam_set.json` Load·포트 매칭
- [ ] `2026_molit_comp_global_path.txt` → planner 입력
- [ ] `2026_molit_comp_sample_scene.json` 시뮬 리허설
- [ ] 연습용 판정 프로그램 LAN 검증 (**9/14~9/18 현장점검 전**)
- [ ] morai_msgs **`beta_drive`** (개발용 ROS) — [morai-msgs-beta-policy.md](../contracts/simulator/morai-msgs-beta-policy.md)

## 9. MORAI 참고

| 용도 | GitHub | 브랜치 |
|------|--------|--------|
| morai_msgs | [MORAI-ROS_morai_msgs](https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs) | `beta_drive` |
| UDP | [MORAI-NetworkModule](https://github.com/MORAI-Autonomous/MORAI-NetworkModule) | `24.R2.0` |
| Python | [beginner_tutorials_answer](https://github.com/MORAI-EDU/beginner_tutorials_answer) | `main` |

> © 2026 MORAI Inc. · 문의: 대회 카카오톡 / 디스코드
