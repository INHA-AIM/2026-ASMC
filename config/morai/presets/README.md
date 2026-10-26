# MORAI 불러오기 정본

날짜는 프리셋 개정일YYMMDD다. 파일 이름은 다음 순서로 고정한다.

`261027_ASMC_LAN2LAN_Network_25.S4.MolitComp03.json`

- Transport: `Local` 또는 `LAN2LAN`
- Kind: `Network`, `Sensor`; GT 개발용만 `SensorGT`
- Version: `26.R1.H3`, `25.S4.MolitComp03`

[전체 목록](catalog.json)에서 같은 Transport/Version의 Network와 Sensor를 한 쌍으로 고른다.
해당 버전의 SaveFile/Network·SaveFile/Sensor에 배포한다. SensorGT는Local에서만 쓴다.
자동저장·주최 채점 설정은 이 폴더의 파일로 대체하지 않는다.

```bash
python3 tools/bridge/render_morai_release_presets.py --write
python3 tools/bridge/render_morai_release_presets.py --check
```

Network의 JSON egoCtrlMode3(Auto)과 UDP 제어 패킷 CtrlMode2(Auto)는 별개다.
파일을 불러온 뒤 실제 UI 주행 모드는 별도로 확인한다.
[전체 계약과 역할](../README.md), [정정 근거](../../../docs/records/bridge/2026-10-27-versioned-morai-presets.md)를 따른다.
