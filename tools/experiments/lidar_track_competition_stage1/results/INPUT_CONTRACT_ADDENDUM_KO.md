# 입력 계약 보강: Competition 채널 식별 정정 (2026-10-22 23:30 KST 추가)

이 문서는 `INPUT_CONTRACT.md` 2절·7절, `INPUT_ALLOWLIST.json`의 `sources_of_rules`·`source_competition`, `FIRST_STAGE_REPORT_KO.md` 1절 3항(P1)과 2절 미검증 항목("105 B 디코드")을 정정한다. 원본 파일은 1차 보고(16:01) 당시 기록으로 그대로 둔다. 기계 판독 사본은 `results/COMPETITION_CHANNEL_IDENTITY.json`이다.

## 1. 정정 내용

| PUBSUB | 이름 | 패킷 | 저장소 template(molitcomp03) 목적지 | bridge 포트·kind | ROS topic | 계약상 지위 |
|---|---|---|---|---|---|---|
| 295 | **Competition Vehicle Status** | `#MoraiInfo$` 152 B body, 181 B datagram, 확장 필드 없음 | 127.0.0.1:**9099** (hostPort 9088) | ego_port 9099, kind=ego | `/Ego_topic` | **허용 입력(vel_x)** |
| 770 | Ego Vehicle Status | `#MoraiInfo$` 229 B, 확장 12개(타이어 횡력·슬립각·코너링 강성) | 127.0.0.1:911 (hostPort 910) | 없음 | 없음 | 판정 전용 |
| 787 | Ego Detail Status | `#DetailEgoInfo$` 105 B (stamp 8 + 12×float64 + lamp 1) | 192.168.0.10:**9798** (hostPort 9797) | competition_port 9798, kind=competition | `/Competition_topic` (디코드 안 됨, 0건) | 판정 전용 |

1차 보고는 787을 Competition Vehicle Status, 295를 Ego Vehicle Status로 기록했다(저장소 문서 `config/morai/README.md` 57·91·95행과 같은 역전). 위 표가 맞다.

## 2. 근거와 수준

| 근거 | 수준 | 내용 |
|---|---|---|
| `2026-ASMC-nogps/config/morai/network/templates/ASMC_network_template_molitcomp03.json` | [설정 확인, 이번 세션] | 항목 3: PUBSUB 295 → 9099. 항목 4: 770 → 911. 항목 7: 787 → 9798(hostPort 9797) |
| `2026-ASMC-nogps/src/asmc_bridge/launch/morai_udp_bridge.launch:12-13`, `scripts/morai_udp_bridge.py:197-198, 272-273` | [코드 확인, 이번 세션] | ego_port 9099 → kind ego → `/Ego_topic`, competition_port 9798 → kind competition → `/Competition_topic` |
| `morai_udp_bridge.py:56-75` (`EGO_VARIANTS`, `EXT_NAMES`) | [코드 확인, 이번 세션] | `#MoraiInfo$` 181/229/245 B만 디코드. 확장 12개(tire_lateral_force, side_slip_angle, tire_cornering_stiffness)는 229/245 B에만 있다. 이 필드들이 바로 지시문의 "타이어 횡력·슬립각 등 미제공 정보"이며 **295(181 B)에는 없다** |
| `morai_udp_bridge.py:95-125` | [코드 주석] | "Static inspection of the competition simulator establishes the 152-byte vehicle-status payload (181 bytes including header/aux/tail)". 105 B `#DetailEgoInfo$`는 "observed local envelope, do not decode its body" |
| 1차 보고 `INPUT_CONTRACT.md` 2절의 과거 관측 | [정합성] | 2026-10-17 local에서 9798 포트에 들어온 패킷은 `#DetailEgoInfo$` 105 B였다. 이는 787 = Ego Detail Status(105 B) 정의와 일치하고 Competition Vehicle Status(152 B)와 불일치한다. 즉 과거 관측 자체가 역전 기록을 반증한다 |
| 2026-10-22 05:13 감사(설치된 MolitComp03 시뮬레이터의 네트워크 기본 항목 자산과 Assembly-CSharp IL) | [과거 세션 확인, 이번 세션 재확인 실패] | 295 = Competition Vehicle Status(pos·vel_y·vel_z·accel 상수 0, 타이어·슬립 블록 없음), 770 = Ego Vehicle Status, 787 = Ego Detail Status, velocity.x = 차체 종방향 km/h. 이번 세션에서는 `C:\2026_MORAI\MoraiLauncher_Win\MoraiLauncher_Win_Data\Bundle\v.S4.251001.MolitComp03\ntdata`(16,582 B)를 복사했으나 UnityPy가 Python ≥ 3.9를 요구해(WSL 3.8) 디코드를 재확인하지 못했다. 시뮬레이터 본체 디렉터리는 현재 `C:\2026_MORAI`에 없다(런처만 있음) |

반대 기록(수정 대상, 제품 저장소 변경은 이번 단계 범위 밖): `config/morai/README.md` 57·91·95행, `docs/competition/rule-v1.2-impacts.md`, `validate_bridge_profiles.py:239-247`, `bridge_profiles_v1.json`. `asmc_planner competition_speed_state.yaml` 주석만 맞다.

## 3. 판정 변경

1. **P1(remap 위반) 철회.** `main.launch:126-128`의 `/Competition_topic → /Ego_topic` remap은 molitcomp03 template(295 → 9099) 아래에서 Competition Vehicle Status를 읽는 **올바른** 경로다. 1차 보고 4절 3항의 "remap 제거" 권고는 철회한다. 제거하면 Main은 `/Competition_topic`(787, 디코드 불가, 0건)을 구독해 모든 LiDAR scan이 not-ready로 skip된다(`SensorPoseHistory.cpp:415-418`).
2. **P2(속도 노름) 유지.** `Parser.cpp:532-533`은 `sqrt(vx²+vy²+vz²)`를 쓴다. 295에서는 vel_y = vel_z = 0 상수라서 수치는 |vel_x|와 같지만 **부호를 잃는다**(후진, 정차 잡음). 계약상 vel_y·vel_z를 읽는 코드 자체가 위반이므로 `velocity.x`만 읽도록 바꾸고, 음수 vel_x 처리(AddCan 거부 → scan skip)를 함께 정한다.
3. **새 위험 H3(이 PC의 MORAI 로컬 저장본).** `C:\2026_MORAI\MoraiLauncher_Win\MoraiLauncher_Win_Data\SaveFile\Network\25.S4.MolitComp03\NetworkInfo_2023_Hyundai_Ioniq5.json`은 **770 → 9099, 295 → 909**로 저장되어 있다(이번 세션 확인). 이 상태로 live 실행하면 bridge가 229 B(770, 판정 전용, 타이어·슬립 확장 포함)를 ego kind로 디코드해 `/Ego_topic`으로 내보내고 Main이 소비한다. 입력 계약 위반이 **경고 없이** 일어난다. 대책:
   - live·MORAI 재생 전 MORAI 네트워크 설정을 template(295 → 9099)으로 교체하고 bridge 진단에서 payload 길이 181 B를 확인한다.
   - 다음 단계 과제: bridge ego_port에 Competition 모드 가드(181 B만 허용, 229/245 B는 fault로 거부)를 추가한다.
4. **1단계 수치에는 영향 없음.** 하네스 competition 모드는 `vel_x_mps` 열만 공급했고, 실시간 측정의 `/Ego_topic velocity.x`는 Competition vel_x 채널로 라벨링해 velocity.x만 채웠다. 채널 이름 정정은 결론 수치를 바꾸지 않는다.

## 4. 허용 목록 갱신 요약 (INPUT_ALLOWLIST.json 대비)

- `vel_x.source_competition`: "Competition Vehicle Status (PUBSUB 295, `#MoraiInfo$` 181 B) → morai_udp_bridge ego_port 9099 → `/Ego_topic` morai_msgs/EgoVehicleStatus.velocity.x" 로 정정.
- `vel_x.unit`: 패킷은 km/h, bridge `ego_velocity_in_ms=true`가 m/s로 변환(변경 없음).
- `vel_x.status_in_product_P0`: "VIOLATION: velocity norm (P2)" 만 유지, remap 항목 삭제.
- 금지 필드 보강: 229/245 B 확장 12개(tire_lateral_force_*, side_slip_angle_*, tire_cornering_stiffness_*)는 770/Ego Vehicle Status에만 있으며 bridge가 디코드하므로, Competition 모드에서는 bridge 단에서 거부해야 한다.
