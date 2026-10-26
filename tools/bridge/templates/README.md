# 생성·보정 입력

이곳은 MORAI UI 배포 폴더가 아니다. 불러올 정본은 `config/morai/network`·`sensors`에 있다.

- `network_26r1.json`, `network_molitcomp03.json`: release별 전체 import schema의 입력. 과거 export의 잘못된 제어 모드는 생성기가 현재 계약에 따라 보정한다.
- `sensor_v3.json`: 현재 센서 생성·카메라/LiDAR 보정의 정확한 입력 바이트.
- `sensor_gt.json`: Local GT 생성 입력.

이동 전 원본과 바이트·SHA가 같다. 내용 변경에는 공용 계약과 보정 근거의 갱신이 필요하다.
10Hz v2는 `src/asmc_bridge/test/fixtures/sensor_v2_10hz.json`에 있다.
현재 사용하지 않는 v1/9111 archive는 Git 이력 `2c691304`에만 보존한다.
