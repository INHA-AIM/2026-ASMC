# 두 Windows PC의 MORAI LAN2LAN 준비·채점 채널 확인

> **2026-10-27 현재 실행 기준:** CompetitionInfoPublisher는9088→9099→`/Ego_topic`이다.
> `--require-competition-status`/`verify-field`도 이9099 입력을 검사하도록 정정했다.
> 9798은 로컬 Detail 진단이며 LAN 운영·수락에서 제외한다. 아래 과거 P3 설명과 현재 역할은
> [정정 기록](../records/bridge/2026-10-27-competition-9099-correction.md)을 기준으로 구분한다.


안승현 PC는 MORAI 송신/제어 수신, 정윤태 PC는 Windows·WSL2·Docker 추론을 담당한다.
현장점검의 채점·녹화 프로그램도 **추론 PC에서 실행**했다는 사용자 확인을 2026-10-20 반영했다.
프로그램이 Windows native인지 WSL/컨테이너인지와 실제 listen endpoint는 별도 확인한다.
이 문서는 설정·확인 절차이며 시스템 변경이나 실제 두 PC/채점 수락을 수행한 기록이 아니다.
기존 [직접 명령](commands.md#6-lan2lan-연습-사전-준비)과
[bridge 계약](../../config/bridge/README.md)을 함께 사용한다.

## 주소·프로파일

| 역할 | 연습용 IPv4 | 담당 |
|---|---|---|
| Windows MORAI PC | `192.168.0.1/24` | 안승현 |
| Windows 추론 PC | `192.168.0.10/24` | 정윤태 |

주최측 현장 NIC/IP/포트가 우선이다. 전용 Ethernet NIC/케이블 또는 스위치를 사용하고
연구실·인터넷 NIC의 기존 주소·route를 먼저 기록한다. 단독 연습 링크는 gateway/DNS를 비우고,
Wi-Fi/VPN/가상 NIC에 같은 subnet route가 중복되지 않는지 검사한다.
현재 연결 중인 연구실 NIC를 문서만 보고 재설정하지 않는다.

본선 시뮬레이터/맵/차량과 제출 센서셋·카메라 장착/해상도/FOV를 맞춘다.
`*_molitcomp03_lan2lan.json`과 `ASMC_sensor_set_v3_lan2lan.json`이 팀 통신 연습용이다.
26.R1에서는 그 release의 import JSON을 사용하며 동일 IP/포트만으로 대회판 packet까지
동일하게 재현했다고 표시하지 않는다. 센서·network 두 설정 모두 destination을 추론 PC로 바꾼다.

## 방화벽·장치관리자

MORAI PC는 추론 PC에서 들어오는 **Ctrl UDP9093**을 해당 Ethernet 프로필/원격 주소에
허용한다. 송신 차단 정책이 있는지도 검사한다. 추론 PC는 아래 팀 수신 포트를 MORAI
주소에서 오는 UDP에 허용한다. Windows Defender와 WSL Hyper-V 방화벽을 각각 확인한다.

| 방향 | 팀 실행의 포트·역할 |
|---|---|
| MORAI → 추론 | RGB `9291/9293/9295/9297`, LiDAR `9299`, GPS `9301`, IMU `9303` |
| MORAI → 추론 | Collision `9092`, 현재 CompetitionInfoPublisher `9099` → `/Ego_topic` |
| 추론 → MORAI | Ctrl destination `9093`, cmd type1·ctrl mode2 |
| 별도 상태 경로 | `9798`은 현재 Detail 형식으로 decoder 거부. 채점용 endpoint와 역할을 별도 확인 |

**이 표는 채점 프로그램 전체의 방화벽 목록이 아니다.** 아래 채점 채널 검토 없이 표의
포트만 전체 PC allowlist로 적용하면 채점 연결을 누락할 수 있다. 방화벽 전체 OFF 대신 실제
프로그램·방향·프로필·상대 IP·포트를 확인한다. 새 규칙과 기존 explicit block 충돌도 검사한다.

장치관리자에서는 실제 연결 NIC의 driver·협상 link speed·전원 관리부터 확인한다.
연속 실행 중 절전/링크 끊김이 있다면 NIC 절전·Energy Efficient/Green Ethernet을 대조한다.
Rx/Tx buffer·RSS·interrupt moderation은 지원 여부와 drop/CPU/지연 근거가 있을 때 한 항목씩
비교한다. Offload 전체 OFF, 강제 duplex, jumbo frame을 기본 일괄 적용하지 않는다.
송신 PC 설정과 고속 수신 PC의 buffer 병목을 구분한다.
[Microsoft NIC 성능 지침](https://learn.microsoft.com/en-us/windows-server/networking/technologies/network-subsystem/net-sub-performance-tuning-nics)은
드라이버/부하별 대조 근거이며 모든 Windows 데스크톱에 같은 값을 요구하지 않는다.

## 추론 PC의 WSL2·Docker

Windows11 22H2 이상에서 다음 mirrored 구성을 연습 기준으로 삼는다.

```ini
[wsl2]
networkingMode=mirrored
firewall=true
```

적용 후 WSL/Docker 재시작은 추론 PC 담당자가 작업 중단 가능한 때 수행한다.
시뮬만 실행하는 PC에는 추론용 WSL 설정이 필수인 것은 아니다.
[WSL 공식 안내](https://learn.microsoft.com/en-us/windows/wsl/networking)에 따라
LAN 수신과 Hyper-V firewall을 확인한다. NAT 환경에 TCP용 `netsh interface portproxy`만
추가해서 UDP 전달을 완료했다고 보지 않는다.

Docker Desktop Linux containers·WSL Integration·Enable host networking을 켜고
현재 compose의 `network_mode: host`를 유지한다. `ports:`를 추가해 다시 NAT하지 않는다.
Docker Desktop host mode는 TCP/UDP 계층을 제공하며 컨테이너가 Windows NIC IP를 직접
소유하지 않는다. Bridge 수신은 wildcard bind이고 Ctrl 원격 destination은 transport에서 지정한다.
Engine/ROS 소켓이 실제로 열렸는지 확인하며 host mode만으로 Windows 방화벽 통과를 보장하지 않는다.
[Docker 공식 안내](https://docs.docker.com/engine/network/drivers/host/)를 따른다.

추론 PC에서 실행한다. 네트워크 확인 단계에서는 차량 제어 노드를 자동 실행하지 않는다.

```bash
./scripts/docker_ros_up.sh status
docker inspect asmc-ros-noetic --format '{{.HostConfig.NetworkMode}} {{json .HostConfig.PortBindings}}'
ip -br addr
ip route get 192.168.0.1
ss -lunp
# 컨테이너의 같은 workspace/ROS 환경에서 bridge만 실행
./scripts/udp_bridge.sh --transport lan2lan
```

현재 UI의 CompetitionInfoPublisher는9099이며 `/Ego_topic`으로 전달된다.
[2026-10-23 loopback 점검](../records/bridge/2026-10-23-competition-channel-audit.md)은
실제 packet 해석까지 확인했으며 두 PC LAN 시험을 대신하지 않는다.
`--require-competition-status`/`verify-field`는2026-10-27에9099 `/Ego_topic` 검사로 정정했다.
9798의 유무는 이 수락 결과에 사용하지 않는다. UI publisher·목적지·실제 소비를 함께 대조한다.

## Wireshark와 단계별 관측

현장에서 들은 “shark”는 **Wireshark일 가능성이 높지만 실제 변경 기록은 확인되지 않았다.**
Wireshark/Npcap은 packet 관측 도구이며 MORAI나 TensorRT의 필수 추론 설정이 아니다.
[Windows 설치 안내](https://www.wireshark.org/docs/wsug_html_chunked/ChBuildInstallWinInstall.html)처럼
Npcap을 함께 준비하고 두 PC의 실제 Ethernet interface를 선택한다.

처음에는 capture filter `host 192.168.0.1 and host 192.168.0.10`으로 양방향을 본다.
Display filter는 `ip.addr == 192.168.0.1 && ip.addr == 192.168.0.10`을 사용한다.
처음부터 팀 포트 몇 개만 capture하면 미확정 채점 채널이나 IP fragment를 놓칠 수 있다.
프레임 조립 문제에서는 packet 길이·방향·IP fragmentation·MORAI 자체 JPEG 조각을 구분한다.
송신 capture의 checksum 표시는 NIC offload 영향을 받을 수 있으므로 즉시 손상/유실로 세지 않는다.
[Wireshark 설명](https://wiki.wireshark.org/CaptureSetup/Offloading)을 따른다.

| 관측 위치 | 확인할 것 | 이 단계만으로 알 수 없는 것 |
|---|---|---|
| MORAI PC Ethernet | 올바른 목적지로 실제 packet 송신, Ctrl 수신 | 추론 PC 방화벽 이후 도착 |
| 추론 PC Ethernet | 같은 채널 도착·rate·길이 | WSL/Docker socket 전달·decode |
| WSL/컨테이너 | bind·UDP 도착·socket drop·bridge diagnostics | 채점 프로그램의 실제 소비 |
| ROS 소비자 | 새 stamp·완성 cloud/영상·stale·parse 오류 | frame Hz를 packet 유실률로 직접 환산 불가 |
| 채점/녹화 프로그램 | 올바른 수신 endpoint와 이벤트·기록 갱신 | 별도 프로그램 수신을 bridge 정상으로 대체 불가 |

가능하면 Windows/WSL 시간을 맞추고 서로 다른 PC wall timestamp를 보정 없이 빼서
네트워크 지연으로 발표하지 않는다. 먼저 packet 수·방향·연속성으로 구간을 좁힌다.
캡처는 수신용 UDP socket을 추가하지 않으므로 기존 producer/consumer와 포트 소유권을 경합하지 않는다.

## 채점 전용 채널: 아직 닫지 않은 경계

2026-10-20 대조에서 팀 generated LAN JSON은 주최 원본의 type258·770·772·259를
MORAI loopback으로 돌린다. 포트는 유지하지만 목적지 IP가 달라 **주최 원본과 동등한
채점 연결로 수락할 수 없다.** [원본/생성본 대조 기록](../records/bridge/2026-10-20-lan-channel-review.md)을 따른다.

현재 정적 팀 설정과 Docker host mode가 설정된 상태/Collision/센서를 의도적으로 차단하는
증거는 없지만 실제 두 PC·채점 동시 시험은 남아 있다. 채점·녹화의 실행 위치는 사용자 확인으로
**추론 PC**다. 따라서 필요한 채점 packet을 MORAI loopback으로 보내면 추론 PC에 도착하지
않는다. 프로그램의 실행 환경과 실제 bind를 확인하기 전에는 채점 통신 정상이나 현재
generated JSON의 현장 적용을 보장하지 않는다.

1. 주최 원본 network JSON과 채점 프로그램 설정을 보존한다. 추론 PC의 채점·녹화 프로그램이
   Windows/WSL/컨테이너 중 어디에서 실행되는지와 listen IP·포트·프로토콜·필요 채널을
   확인한다. packet 목적지는 연습 기준 `192.168.0.10`이며 현장 제공 주소가 우선이다.
2. 팀 알고리즘 입력과 채점 전용 소비를 나누어 endpoint 표를 확정한다. 채점 전용 packet이
   PC에 도착하는 것과 인지/판제가 이를 사용하도록 허용하는 것은 별개다. 필요한 judge
   channel은 LAN을 통과시키고 알고리즘의 구독/입력 제한은 별도로 유지한다.
3. 주최판에서는 팀 generated 파일로 전체 설정을 덮지 않는다. 합의한 sensor/팀 UDP endpoint와
   채점 endpoint를 함께 유지하며 수정 범위와 실제 packet을 대조한다.
4. 채점/녹화가 Windows에서 실행되면 Docker가 중계한다고 가정하지 않는다. 컨테이너에서
   실행되면 해당 프로그램의 포트 bind와 Windows/Hyper-V 경로도 확인한다.
5. 같은 host/port에 judge·bridge·시험 수신기를 중복 bind하지 않는다. 한 packet이 모든
   프로그램에 전달될 것으로 기대하지 않고 수신 소유자를 확정한다.
6. 알고리즘에는 GT/Object/신호 oracle 등을 연결하지 않은 채 실제 주행·제동·충돌/신호 등
   합의한 이벤트가 채점 기록에 갱신되는지 주최 프로그램으로 확인한다.

채점 프로그램이 없는 집 연습에서는 센서·Ctrl·상태·pipeline까지만 통과로 기록한다.
가짜 ROS 토픽이나 단순 UDP 송신을 실제 채점 수락으로 대체하지 않는다.
기본 단독 연결 후 세 모델·LiDAR·판제와 녹화 동시 부하로 최소 한 차시15분을 재현하고,
Windows/NIC·bridge·MORAI 재시작 후 재연결을 확인한다.
