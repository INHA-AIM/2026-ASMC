# LAN2LAN·채점 채널 코드 대조 — 2026-10-20

main `2c94bf11`과 개인 PC의 읽기 전용 점검이다. 두 PC 연결·packet capture·제어 명령·
GPU 추론·채점 프로그램 실행은 하지 않았다. IP·방화벽·장치 옵션·컨테이너·실행 노드를
변경하지 않았다. 이 기록은 실제 현장 수락이나 주최 설정 변경 승인이 아니다.
실행 절차는 [LAN2LAN 안내](../../operations/lan2lan-rehearsal.md)를 따른다.

## 근거와 원본

| 입력 | 식별 |
|---|---|
| 주최 원본 | 보유한 On-site inspection의 `2026_molit_comp_network.json`, SHA `68b5bd089b38449819ec064a67e9f821e3280068c1171e751c77e13282b57577` |
| 팀 생성본 | `config/morai/network/generated/ASMC_network_set_molitcomp03_lan2lan.json`, SHA `bf8eeb355222a923bacd046f9d46c0e8951ddda6b78df32407ef8ace85830507` |
| 생성 규칙 | `tools/bridge/render_morai_network_profiles.py`: 명시한 팀 채널만 LAN으로 보내고 나머지 UDP endpoint는 loopback |
| 계약 | `config/bridge/bridge_profiles_v1.json`: local/lan2lan 공용 bridge, type787 현장 live 수락 대기 |

주최 원본은10개, 팀 template/generated는12개 network entry다. 원본은 repository에 새로
복사하거나 수정하지 않았다. 위 SHA는 이번 대조 때 실제 읽은 두 파일의 값이다.

## 목적지가 달라지는 채널

아래 이름은 보유 계약/네트워크 항목의 식별이다. 최초 검토 때 실행 위치는 미확인이었으며,
같은 날 후속 사용자 확인으로 **채점·녹화는 추론 PC에서 실행**한 사실을 반영했다.
각 채널의 실제 소비와 Windows/WSL/컨테이너 실행 형태·bind는 아직 미확인이다.
판정 전용/개발용 데이터를 팀 알고리즘 입력으로 추가하라는 의미가 아니다.

| PUBSUB type | 식별 | 주최 원본 Host → Destination | 팀 generated Host → Destination |
|---|---|---|---|
| 258 | TrafficLight status oracle | `192.168.0.1:7603 → 192.168.0.10:7502` | `127.0.0.1:7603 → 127.0.0.1:7502` |
| 770 | Ego Detail 계열 | `192.168.0.1:910 → 192.168.0.10:911` | `127.0.0.1:910 → 127.0.0.1:911` |
| 772 | Object Info | `192.168.0.1:7605 → 192.168.0.10:7505` | `127.0.0.1:7605 → 127.0.0.1:7505` |
| 259 | Ego 설정 입력 계열 | `192.168.0.1:7607 → 192.168.0.10:7503` | `127.0.0.1:7607 → 127.0.0.1:7503` |

Ctrl(type771), Collision(type261), Ego(type295), Competition(type787)의 위 LAN endpoint는
주최 원본과 같다. type266/783은 팀 template에 추가로 있는 항목이며 주최 원본10개에는 없다.
생성 검사의 통과는 팀 allowlist 규칙과의 일치이지 주최 채점 설정과의 동일성이 아니다.

특히 type259는 Division_TYPE=2인 설정 입력이다. `destinationPort`가 쓰여 있다고 해서
이를 팀이 반드시 수신해야 하는 새 상태 토픽으로 해석하지 않는다. 각 타입의 실제 송수신
방향·소유 프로그램을 확인한 뒤 필요한 방화벽을 정한다.

## 현재 PC에서 읽은 상태

- 물리 NIC는 Realtek2.5GbE이고 현재 협상 link는1Gbps다. 현재 주소는 연구실 네트워크이며
  `192.168.0.1/24` 연습 설정이 아니다. 이 점검에서 주소를 바꾸지 않았다.
- 현재 WSL은 `172.24.208.0/20` 가상 네트워크를 사용하며 사용자 `.wslconfig`가 없었다.
  따라서 이 PC의 mirrored 설정을 완료했다고 표시하지 않는다. 이 PC가 시뮬 전용이면
  추론용 WSL mirrored 설정은 정윤태 PC에서 확인한다.
- 실제 `asmc-ros-noetic`은 `network=host`, port bindings/published 모두 `{}`다.
  Docker Desktop 설정의 `HostNetworkingEnabled=True`도 읽었다. compose의 개별 포트
  누락으로 type787을 막고 있다는 증거는 없다.
- Domain/Private/Public Windows firewall은 켜져 있고 MORAI/Docker 이름의 기존 규칙이 있다.
  규칙 존재만으로 새 Ethernet 프로필·원격 IP·포트를 허용한다고 판정하지 않았다.
- 기본 `C:\Program Files\Wireshark\Wireshark.exe`는 없었고 `npcap` 서비스도 관측되지 않았다.
  다른 설치 경로까지 전수 검색한 결과는 아니다. 현장 연구원이 무엇을 바꿨는지의 증거도 아니다.

시뮬 PC에 보내는 packet과 WSL/컨테이너 socket은 다른 계층이다. 채점 프로그램이 추론 PC의
Windows에서 직접 실행되면 팀 Docker가 모든 채점 packet을 중계하는 구조가 아니다.
반대로 같은 PC에서 judge와 bridge가 같은 포트를 bind하면 충돌 가능성을 따로 확인해야 한다.

## 미완료와 담당

| 항목 | 상태·담당 |
|---|---|
| 본인 PC의 MORAI LAN IP·sensor/network import | 준비 절차 제공, 실제 적용 전 — 안승현 |
| 추론 PC Windows/WSL2/Hyper-V/Docker | 원격 PC 미검사 — 정윤태 |
| 채점·녹화 실행 PC | **추론 PC — 후속 사용자 확인** |
| 채점 프로그램 이름·실행 형태·listen 포트 | Windows/WSL/컨테이너 및 실제 bind 확인 대기. 주최 담당자/팀 공동 확인 |
| type787 실제 packet·decode·Main 소비 | fixture/코드 준비, 현장 live 대기 |
| 센서·Ctrl 양방향·채점/녹화 동시 부하 | 두 PC/실제 프로그램 수락 미실행 |

팀 generated JSON을 주최 PC에 통째로 덮어쓰거나 oracle을 알고리즘에 연결해 이 차이를
해결하지 않는다. 주최 원본과 실제 judge 설정을 기준으로 별도 소비 경로를 확정한다.
사용자 기억의 “shark”는 Wireshark일 가능성이 있다는 추정으로만 남긴다.
추론 PC의 프로그램에 필요한 채널이라면 위 loopback 목적지는 실제 누락 원인이다.
필요한 judge channel을 LAN으로 전달하는 것과 알고리즘의 oracle 사용 금지는 별도 계층에서
유지한다. 이 후속 확인으로 실제 network JSON이나 방화벽을 변경하지는 않았다.

## 이번 변경의 로컬 검증

- 문서 링크/anchor·중앙 구조·기존 동결5개 보존 검사 통과.
- 실제 launch25개·명령 예시63개 검사와 문서 CPU 회귀17개 통과.
- network/sensor generated 결정성·bridge 계약 정적 검사 통과. validator가 출력하는
  과거 full-graph674회 필드는 이번에 ROS674회를 다시 실행했다는 의미가 아니다.
- 소비 코드3개는 주석만 추가됐고 Compose의 YAML 해석 결과도 동일함을 검사했다.
- `git diff --check` 통과. 운영 모델·메시지·네트워크 생성물·추론/판제 로직은 변경하지 않았다.

전체 빌드·노드 재시작·새 성능 측정·hosted CI는 이번 문서/주석 점검에 포함하지 않는다.
