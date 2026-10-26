# 팀원 PC의 LAN2LAN 연결 순서

Windows+WSL2+Docker 실행 PC용이다. MORAI는 시뮬레이터 PC에서 실행한다.
프로젝트·Docker 이미지·모델은 기존 설치본을 사용하며 전달 ZIP은 정정 패치·설정·문서 묶음이다.

## 1. 현재 채널과 전용 NIC

- 시뮬레이터 `.1/24` ↔ 실행 PC `.10/24`, gateway/DNS 없이 연결한다. 인터넷 NIC는 별도다.
- CompetitionInfoPublisher의 UI Host9088 / Destination9099를 확인한다.
  시뮬레이터 IP `.1`에서 실행 PC `.10:9099`로 보낸다. ROS 호환 이름은 `/Ego_topic`이다.
- 판정용 Ego910/911과 Detail9798은 운영 수락에 사용하지 않는다.
- 어댑터 이름·ifIndex는 USB 포트 변경으로 달라질 수 있다. MAC으로 실제 전용 NIC를 확인한다.

## 2. Windows 설정

전달 ZIP을 Windows 폴더에 풀고 관리자 PowerShell에서 실행한다.
`<USB_MAC>`은 실제 실행 PC USB 랜의 MAC으로 바꾼다.

```powershell
Get-NetAdapter -Physical | Format-Table Name,ifIndex,Status,MacAddress,InterfaceDescription
powershell -NoProfile -ExecutionPolicy Bypass -File .\setup_lan2lan_windows.ps1 -Role Autonav -ExpectedMacAddress '<USB_MAC>' -ConfigureWsl
```

미리보기가 맞으면 같은 명령에 `-Apply`를 추가한다. 연결이 Down이거나 다른 NIC에 같은
192.168.0.x 주소가 있으면 적용 전에 원인을 해결한다. 이미 올바른 NIC에 .10이 있으면
IP 삭제·추가를 반복하지 않는다. 기존 ASMC 규칙도 현재 NIC·포트로 갱신하며 다른 규칙은 유지한다.
Windows/Hyper-V 허용 UDP는9099,9092,9291,9293,9295,9297,9299,9301,9303이다.
Host9088은 UI 값이며 실제 UDP source port는 동적일 수 있으므로 수신 규칙에 강제하지 않는다.
설정 변경 전 기록과 `.wslconfig` 백업은 `%LOCALAPPDATA%\ASMC\lan2lan`에 남는다.

WSL 설정을 바꿨다면 실행 PC 작업을 저장한 뒤 `wsl --shutdown`으로 반영하고 Docker Desktop을
재시작한다. 시뮬레이터 PC의 WSL/Docker를 종료하지 않는다. WSL에서는 실제 실행 결과를 확인한다.

```bash
wslinfo --networking-mode
docker inspect asmc-ros-noetic --format '{{.HostConfig.NetworkMode}}'
```

기준은 mirrored 및 host다. Docker Desktop의 Resources → Network → Enable host networking도
별도로 켜져 있어야 한다. 컨테이너의 host 표시만으로 이 옵션이 켜졌다고 판단하지 않는다.
근거: [Microsoft WSL networking](https://learn.microsoft.com/en-us/windows/wsl/networking),
[Docker Desktop host networking](https://docs.docker.com/engine/network/drivers/host/).

## 3. 코드 반영

ZIP의 manifest에 있는 기준 revision·수정 commit·SHA를 확인한다.
기존 r2가9099를 차단한 상태인지, 현재 revision과 미커밋 변경이 무엇인지 먼저 확인한다.
패치는 `git apply --check`를 통과할 때만 적용한다. 충돌 시 강제 reset/덮어쓰기하지 않는다.
Git 사용 시 전달된 수정 branch/commit을 해당 작업 코드에 병합한다.

## 4. 실제 수신

실행 중인 같은 포트의 임시 Windows UDP 수신기와 중복 bridge가 없는지 확인한다.
ROS 컨테이너(`/root/ws`)에서 bridge 한 개만 실행한다.

```bash
./scripts/udp_bridge.sh --transport lan2lan --require-competition-status
```

두 번째 ROS 컨테이너 터미널에서 다음을 실행한다.

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 scripts/verify_morai_topics.py --profile competition --transport lan2lan --require-competition-status --duration 30
```

9099의 실제 수신·decode·`/Ego_topic` stamp 갱신과 센서 토픽을 확인한다.
9798만 들어오거나 토픽 이름만 선언된 것은 성공이 아니다. source timestamp와 서로 다른 PC의
wall time을 보정 없이 빼서 지연을 계산하지 않는다. 위 검사는 새 차량 제어를 시작하지 않는다.

## 5. 문서와 기록

[commands.md 6절](commands.md#6-lan2lan-연습-사전-준비)은 상세 계약,
[운영 README](README.md#3-docker-기동)는 Docker 준비,
[rosbag 안내](rosbag-evaluation.md)는 기록할 때 참고한다.
`competition_raw`는9099의 `/Ego_topic`을 기록한다. 새 bag/학습은 별도 저장소 여유·run 등록을
통과해야 하며, 이번 연결 수락에서는 bag/학습을 시작하지 않는다.
MORAI Network JSON은 시뮬레이터 담당자가 사용하고 실행 PC에 import하지 않는다.
