# P3-G bridge local closeout baseline

> **식별자:** `p3g_bridge_local_closeout_v1`
> **날짜:** 2026-09-23
> **범위:** 참가팀 로컬 MORAI 수락 기준과 현장 전용 Competition Vehicle Status gate 분리

## 결론

`asmc_bridge`의 로컬 기본 실행은 RGB 4대, VLP16, GPS, IMU, Ego Vehicle Status와 유효한
`/vehicle/state`를 필수로 검사한다. `/Competition_topic` decoder와 publisher는 계속 기동하지만,
참가팀용 MolitComp03가 type 787 패킷을 실제로 내보내지 않는 조건에서는 누락을 정상으로
표시한다. 주최 현장판에서 type 787을 수락할 때만 `--require-competition-status`를 명시한다.

이 구분은 simulator release 이름으로 추측하지 않는다. `--simulator-release`는 측정 provenance
label일 뿐 필수 토픽을 바꾸지 않는다.

## Network export 대조

| 원본 | SHA-256 | entry | Ego 계열 | Competition type 787 |
|---|---|---:|---|---|
| 주최 MolitComp03 `2026_molit_comp_network.json` | `68b5bd089b38449819ec064a67e9f821e3280068c1171e751c77e13282b57577` | 10 | type 295 `9088→9099` | 포함, `9797→9798` |
| 팀 MolitComp03 직접 저장본 | `182fdaa802d59a41a343fbff7344d233031153b0435402ccbaa8ea1fbd7f4b51` | 12 | type 295 `9088→9099` | 포함, `9797→9798` |
| 팀 26.R1 직접 저장본 | `5b57d2c95b57548f6fda2d574f8db77e1748239cb39056c705516dd79fe93006` | 23 | type 770 `908→909` 원본 | 없음 |

MolitComp 두 JSON에 type 787 항목이 있다는 것은 UI가 그 채널을 정의할 수 있다는 뜻이지,
참가팀 배포 시나리오가 packet을 발행한다는 뜻은 아니다. P3-E 로컬 계측에서는 UDP 9798에
8초간 packet이 없었다. 반면 `Ego Vehicle Status`는 참가팀 local에서 실제 수신됐고, bridge
생성본은 두 release의 Ego port를 `9088→9099`로 정규화한다.

`/vehicle/state`는 Competition packet 변환값이 아니다. GPS와 IMU를 합성하는 팀 내부
`asmc_msgs/VehicleState`이므로 type 787이 없어도 독립적으로 유효해진다.

## 실행 계약

```bash
# 참가팀 로컬 회귀: Competition packet은 선택 사항
./scripts/udp_bridge.sh
./scripts/docker_ros_up.sh verify 8 molitcomp03

# 주최 현장판 수락: type 787을 명시적으로 필수화
./scripts/udp_bridge.sh --transport lan2lan --require-competition-status
./scripts/docker_ros_up.sh verify-field 30 molitcomp03
```

기본 diagnostics에서 packet이 없는 Competition 입력은 level `OK`, message
`optional; no data`, key `required=false`다. 명시적 field gate에서는 같은 상태가 `WARN`,
`no data`, `required=true`가 되며 verifier도 실패한다. 다른 활성 입력은 계속 필수다.

## 유지되는 경계

- Ctrl 기본 `9093/9094`, mode 2와 정지 제동 closed loop 결과는 바꾸지 않았다.
- sensor set v3의 UI/소비 주기는 15 Hz이고 실제 완성 cloud 약 12 Hz는 승인된 입력이다.
- Object/Semantic/Instance는 local `development`에만 허용한다.
- LAN2LAN IP·firewall·route·재시작 복구와 type 787 실제 packet capture는 현장 리허설 gate다.
- release별 MORAI import JSON은 schema가 달라 계속 분리하지만 ROS bridge binary는 하나다.

## 검증 결과

- Noetic 전체 workspace build: 12개 catkin package, 성공
- 전체 현재 graph: `674 tests, 0 errors, 0 failures, 0 skipped`
- `asmc_bridge`: 14 tests 통과(UDP 7, sensor 4, verifier policy 3)
- vehicle-state direct test: 정속 `10.0174 m/s`, 200 ms braking `8.36229 m/s`, PASS
- 참가팀 MolitComp03 local 8.02초: 필수 9 topic 모두 PASS, 유효 VehicleState 394개
- 관측률: front/left/right 약 20 Hz, extra 9.99 Hz, VLP16 11.23 Hz, GPS 5.02 Hz,
  IMU 44.29 Hz, VehicleState 50.00 Hz
- Competition diagnostics: level 0, `optional; no data`, `required=false`, packet 0
- 같은 연결에서 `verify-field 3 molitcomp03`: `/Competition_topic`만 0건, 예상 exit code 2

마지막 실패는 결함이 아니라 아직 참가팀 local에서 제공되지 않는 현장 전용 capability가
명시적 gate에서 차단되는지를 확인한 음성 시험이다.

## 다음 단계

Phase 3의 local 개발 범위와 물리 패키지 통합은 닫는다. 다음 구현은 Phase 4의
`P4-A perception runtime baseline`이며, 먼저 현재 camera/LiDAR/traffic-light/BEV/fusion의
topic·frame·timestamp·model artifact와 oracle 금지선을 이동 없이 동결한다. 실제 두 PC
LAN2LAN 리허설은 perception 통합과 최소 단일 실행 진입점 이후에 수행한다.
