# MORAI 설정 정본

MORAI에서 불러올 파일은 `network/`와 `sensors/`에만 둔다.
[목록과 버전 매핑](catalog.json)을 기준으로 같은 버전·transport의 Network와 Sensor를 고른다.
이름은 `YYMMDD_ASMC_LAN2LAN/Local_Network/Sensor_모라이버전.json`이다.
날짜는 프리셋 개정일이며 GT 개발용은 `SensorGT`·Local로 구분한다.

| 버전 | 모드 | Network | Sensor |
|---|---|---|---|
|26.R1.H3|Local|[Network](network/261027_ASMC_Local_Network_26.R1.H3.json)|[Sensor](sensors/261027_ASMC_Local_Sensor_26.R1.H3.json)|
|26.R1.H3|LAN2LAN|[Network](network/261027_ASMC_LAN2LAN_Network_26.R1.H3.json)|[Sensor](sensors/261027_ASMC_LAN2LAN_Sensor_26.R1.H3.json)|
|25.S4.MolitComp03|Local|[Network](network/261027_ASMC_Local_Network_25.S4.MolitComp03.json)|[Sensor](sensors/261027_ASMC_Local_Sensor_25.S4.MolitComp03.json)|
|25.S4.MolitComp03|LAN2LAN|[Network](network/261027_ASMC_LAN2LAN_Network_25.S4.MolitComp03.json)|[Sensor](sensors/261027_ASMC_LAN2LAN_Sensor_25.S4.MolitComp03.json)|

Network4개·Sensor6개(GT2개 포함)가 현재 배포 정본 전부다. 해당 버전의
`SaveFile/Network/<버전>`·`SaveFile/Sensor/<버전>`에 배포한다.
자동저장 NetworkInfo/SensorInfo와 NPCGhost는 MORAI 소유 파일이다.

## 모드와 채널

[설치본 enum 근거](control-mode-enums-20261027.json): JSON `egoCtrlMode`는1=Keyboard,
2=GameWheel,3=AutoMode다. 정본은3으로 저장한다. UDP 제어 패킷의 `CtrlMode=2`는 별도 enum의
Auto이며 controller 선택은 `PUBSUB_TYPE=771`=MoraiCmdController다. 파일 load 후 실제 UI의
Control Mode와 연결 상태는 별도로 확인한다. 이 파일 생성·배포는 제어 송신을 시작하지 않는다.
26.R1의 controller buffer는59, MolitComp03은55이며 wire encoder는 기존값을 유지한다.

MolitComp03의 CompetitionInfoPublisher9088→9099는 `/Ego_topic`으로 전달한다.
9798은 별도 #DetailEgoInfo$138B 관측이며26.R1 원본에는 없다. 팀 bridge는 LAN에서 Detail을
수신하지 않아 이 팀 runtime 파일에서는 loopback이다. **채점용 소비 여부는 미확인**이며,
알고리즘에서 사용하지 않는다는 이유로 주최 채점 송신을 차단해도 된다는 뜻이 아니다.
Type267 TF2는 팀 입력에 사용하지 않아 loopback·버퍼 스레드OFF를 유지한다.
이는 센서·9099 전체 큐 설정이 아니다.

**이 파일들은 팀 통신용이며 채점까지 포함한 최종 현장 설정이 아니다.**
주최 전체 설정을 덮어쓰지 말고 주최 채점 프로그램의 endpoint를 보존한다.
[채점 경계](../../docs/operations/lan2lan-rehearsal.md#채점-전용-채널-아직-닫지-않은-경계)를 따른다.

일반 Sensor는 v3: 카메라4대·VLP16·GPS·IMU이며 LAN 내용은 `8_Autonav_sensor.json`과 JSON 차이0이다.
Local은 UDP IP만127.0.0.1이다. 센서 위치·FOV·주기·포트는 이 디렉터리 정리로 바꾸지 않았다.
GT는 Semantic Camera3대와 Instance LiDAR를 추가한 Local 개발 전용이다.

## 생성과 보존

```bash
python3 tools/bridge/render_morai_release_presets.py --write
python3 tools/bridge/render_morai_release_presets.py --check
python3 tools/audit/validate_bridge_profiles.py
```

생성기는 [공용 계약](../bridge/bridge_profiles_v1.json)과
[필요한 원본4개](../../tools/bridge/templates/README.md)를 사용한다.
Network의 버전별 import schema와 센서 원본은 생성·보정에 실제 필요하므로 tools에 두며
바이트·SHA를 유지한다. 10Hz 비교 입력은
[회귀 fixture](../../src/asmc_bridge/test/fixtures/sensor_v2_10hz.json)로 유지한다.
`capture_asset_ground_*`와 `object_spawn_catalog_v1.yaml`은 현재 capture workflow 입력이므로 유지한다.

미사용9111 archive, v1 센서 및 중복 generated 배포본은 현재 트리에서 제거했다.
원본 복원은 Git 이력 `2c691304`의 원경로를 사용한다. 예:

```bash
git show 2c691304:config/morai/sensors/ASMC_sensor_set_v1.json
```

과거 bag·데이터셋의 sensor_set 식별자와 동결 provenance는 이름을 치환하지 않는다.
실제 Windows SaveFile에 배치된 파일명·내용은 이 경로 정리로 바뀌지 않는다.
