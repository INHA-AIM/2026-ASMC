# MolitComp03 Competition 9099 LAN 정정

사용자 기준2026-10-27 UI에서 CompetitionInfoPublisher Host9088/Destination9099와
MoraiInfoPublisher Host910/Destination911을 확인했다.
[2026-10-23 실측](2026-10-23-competition-channel-audit.md)과 일치한다.
이전 work 전달본의9099 제외·9798 Competition 필수화는 잘못된 해석으로 철회한다.

현재 wire 호환 이름과 실제 UI 역할은 다르다. `ego_port=9099`/`/Ego_topic`은 이 환경의
허용 Competition 입력이며 `competition_port=9798`은 과거 호환 이름의 로컬 Detail 진단이다.
기존 decoder·단위 변환·ROS 메시지·Main/shadow 소비 계약은 바꾸지 않는다.
LAN에서9099를 필수로 받고9798·판정용 Ego·Object/GT를 운영 fallback으로 사용하지 않는다.
26.R1의 기존 설정·제어 보정과 local/GT 동작은 보존하며 이번 UI 판정을 일반화하지 않는다.

MolitComp03의 verified_status_bindings를 추가했고 generated LAN JSON의 type295는 LAN,
type787/770은 loopback으로 둔다. 원본 template, 과거674개 시험 수와 P3 관측은 동결 이력이다.

## 회귀 근거

측정 fixture는 보존된 competition-channel-audit-20261023-r1/packets.json
(SHA256 33eca123d9e547d95dcf22fc6ff633d9dd052426ae92688772139bac8bb11c8b)의
9099 181B 및9798 138B 표본이다. 작은 회귀 fixture만 Git에 보관한다.
정상9099 +9798 부재는 통과하고, Detail만 있거나9099의 header/길이가 잘못되면 실패한다.

## 실제 수락 범위

기존 USB NIC 송신 캡처의9798 78개는 모두 #DetailEgoInfo$ 138B였으며 Competition 수락이 아니다.
활성 Competition의 목적지 .10:9099 적용 후 별도 수동 캡처와 실행 PC의 decode·`/Ego_topic`
갱신·센서 수신을 확인한다. 정적 시험과 송신 관측만으로 실행 PC 수신·주행·채점 수락을 선언하지 않는다.
시뮬레이션 재생·센서 송신을 유지하며 새 제어 송신, 학습, bag 수집은 시작하지 않는다.

## 구현 검증 결과

- 계약·CLI/토픽 정책·문서: 호스트 unittest35개 통과. bag 지원6개 통과.
- ROS Noetic의 격리된 임시 소스: node15개·transport3개 통과. 실제9099 fixture의
  decode·기존 단위 변환·mock ROS 발행, Detail138B 거부를 포함한다.
- network/sensor 재생성 일치, 계약 SHA, shell syntax, diff whitespace 검사 통과.
- Windows PowerShell5.1에서 native cmdlet 인자·MAC 선택·규칙 갱신/재적용·미리보기·
  오선택/충돌 거부·WSL CPU/메모리 보존을 mock 기반으로 확인했다.
- 26.R1 network·모든 sensor JSON·local network·제어 encoder는 변경하지 않았다.

## 시뮬레이터 송신 확인

사용자가 보존된 활성 설정을 기준으로 Competition 행 IP만 수정한 Network를 load/connect했다.
활성 센서7개의 목적지는 이미 `.10`이므로 sensor reload는 하지 않았다.
우리 USB NIC의 ASMC 전용 방화벽을 갱신했고 WSL/Docker/MORAI는 재시작하지 않았다.

수동30초 캡처 `ASMC9099-20261026T173920845Z`에서 `.1→.10:9099`의 전체181B UDP를
64개 보존했고 기존 decoder로64/64 decode했다. source timestamp도 진행했다.
PCAPNG SHA256: `8d43f826b136370342dc9e7b868fd5b7d4007d371a16a5ccc161c23f583ff0a2`.
관측 구간은 UTC17:39:33.036947~17:40:02.344180이다. 이번 source port는9088이었으나
방화벽은 이를 강제하지 않는다. pktmon counter1274와 export64의 집계 범위가 달라
완전한 패킷 수·손실률·송신률은 확정하지 않았다. 영상 frame 성능의 근거도 아니다.

실행 PC의 최신 revision/r2/dirty 상태와 실제 decode·`/Ego_topic` stamp 갱신·센서 검사는
아직 외부 확인 대기다. 송신 성공과 전체 수락 완료를 구분하여 이 PR은 Draft로 유지한다.
