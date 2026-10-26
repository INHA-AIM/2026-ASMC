# 버전별 MORAI 정본과 주행 모드 정정

사용자 지정 이름은 `YYMMDD_ASMC_LAN2LAN/Local_Network/Sensor_모라이버전.json`이다.
[정본 목록](../../../config/morai/catalog.json)은26.R1.H3·25.S4.MolitComp03 각각
Network/Sensor의Local/LAN2LAN8개와 별도Local SensorGT2개다.
정본은 공통 계약·생성기를 공유하고 모드·버전 메타데이터와 모든 JSON byte를 검사한다.
config/morai의 network/·sensors/에는 불러올4+6개 파일만 둔다. 생성 입력4개는
tools/bridge/templates, 필요한10Hz 입력은 테스트fixture로 이동하고 원본 SHA를 유지한다.
미사용v1·9111 archive와 중복 generated 파일은 Git 이력2c691304에만 보존한다.
런처·보정·감사 경로를 함께 바꾸되 과거 bag 식별자와 동결 근거는 치환하지 않는다.

## 센서 동일성

사용자 `8_Autonav_sensor.json`은 v3 LAN2LAN과 parsed JSON 차이0이다.
카메라4대·VLP16·GPS·IMU이며 GroundTruthList는 비어 있다.
Local은 UDP host/destination IP만127.0.0.1로 바뀐다. 설치 버전별 Sensor 내용은 같지만
오선택을 줄이기 위해 버전이 다른 별도 배포 파일로 만든다. 센서 위치·FOV·주기·채널을 바꾸지 않는다.

## 제어 모드 오류와 근거

이전 MolitComp 생성기는 UDP AutoMode 바이트2를 JSON egoCtrlMode에도 적용했다.
이것은 GameWheel 값이며 자율주행용 파일의 의도와 달랐다. 두 설치 버전의
Assembly-CSharp.dll을 실행하지 않고 .NET metadata로 확인했다.

| 필드 | Keyboard | GameWheel | Auto |
|---|---|---|---|
| JSON EgoCtrlConfig.egoCtrlMode |1|2|3|
| UDP #MoraiCtrlCmd$ payload CtrlMode |1|해당 표의 값 없음|2|

[근거와 DLL SHA](../../../config/morai/control-mode-enums-20261027.json)를 보존한다.
JSON mode3은 주행 상태이고 controller 선택은 PUBSUB_TYPE771=MoraiCmdController다.
이전 R1 설명 중 mode3을 controller 선택 숫자로 표현한 부분도 분리했다.
MolitComp 정본의 JSON mode를3으로, `_PUBSUB_TYPE`을771로 명시한다. UDP encoder는 바꾸지 않는다.
과거 template·주최/직접 저장 snapshot·동결 실측 기록은 수정하지 않는다.

파일 load가 실제 Auto 상태나 차량 제어 성공을 입증하지 않는다. UI의 실제 Control Mode,
연결과 소비를 확인해야 한다. 이 작업은 설정 파일 배포이며 새 control 송신을 시작하지 않는다.

## 9798과 type267

현재 MolitComp03의9099는 CompetitionInfoPublisher이며 별도9798에서는
#DetailEgoInfo$138B를 관측했다. 26.R1 원본에는 type787/9798이 없다.
팀 LAN bridge는 이 Detail을 수신·발행하지 않아 기본 정본에서는 loopback을 유지한다.
진단이나 별도 주최 소비자에게 보내야 한다면 그 소비자·허용 역할·endpoint를 확인해 별도 구성한다.
LAN transport가 모든 publisher를 상대 PC로 보낸다는 뜻은 아니다.

Type267은 TF2이다. 팀 입력이 아니므로 loopback과 버퍼 스레드OFF를 유지한다.
이 채널 플래그를 센서·9099 전체 큐 설정으로 해석하지 않는다.
주최 채점 프로그램의 전체 network는 팀 생성본으로 일괄 덮어쓰지 않는다.

## 배포와 검증

해당 버전의 SaveFile/Network 또는 SaveFile/Sensor 하위에 정확한 Git 생성 byte를 배포한다.
바뀌는 ASMC UI 이름의 원본은 로더 밖 복구 폴더와 원경로/SHA manifest로 보존한다.
자동저장 NetworkInfo/SensorInfo, NPCGhost, 비ASMC 원본, 데이터·bag·학습 라벨은 이 작업의 대상이 아니다.
실행 중 시뮬레이터·WSL·Docker·다른 채팅의main/work checkout은 유지한다.

실행한 검사:

- 계약·CLI/토픽/transport·문서 unittest38개 통과
- 세 생성기 재생성 일치, canonical8+GT2, JSON/UDP mode 분리 및 metadata 오선택 거부 통과
- 문서 migration·런처 참조·기존CI145개 검사 보존·shell syntax·diff whitespace 통과

실제 두 PC 자율주행·채점 수락은 이 파일 생성 시험과 별도이며 재호 담당 검증을 대체하지 않는다.

## 경로 정리의 검증

Network4개·Sensor6개의 JSON byte는 기존 SaveFile 배포본과 동일하다.
생성 원본4개와10Hz 회귀fixture도 이동 전 SHA와 같다.
현재 camera/BEV/LiDAR 설정은 입력 경로 문자열만 바꿨고 수치·원본 sensor SHA를 유지했다.
현재 parity 목록에는 metadata 경로 수정의 previous SHA를 남겼다. 과거 run·동결 기록은 바꾸지 않았다.
config 하위에 중복 presets/archive/generated/templates 폴더가 다시 생기면 계약 검사가 실패한다.

## 병합 전 확인

정본·공용bridge README, 치트시트, 규정 해설과 실행 로그에서 JSON 주행모드3과
UDP CtrlMode2를 구분한다. GT catalog의release/sensor_set 오표기 거부와 누락 출력 디렉터리의
정확한 재생성도 검사한다. 원본10개 SaveFile·8_Autonav 센서의 내용은 유지한다.
실제 주행·채점 검증은 수행하지 않았으며 팀 통신 정본을 주최 전체 설정으로 승인하지 않는다.
