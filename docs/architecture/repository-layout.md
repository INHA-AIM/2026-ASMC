# 저장소 지도

```text
2026-ASMC/
├── docs/                 # 온보딩·구조·운영·계약·인계·대회·기록
├── src/                  # 팀 ROS 패키지, nested LiDAR workspace
├── config/               # simulator·기계 계약·모델/데이터 규칙·보존 증거
├── scripts/              # 빌드·Docker·bridge 진입점
├── tools/                # scenario·gRPC·오프라인 감사/평가
├── docker/               # ROS Noetic, 격리 학습 이미지
├── mgeo_toolkit/          # 지도 도구
└── R_KR_PG_KATRI/         # 기존 맵 자산
```

## 패키지 경계

| ROS 패키지 | 현재 경로 | 책임 |
|---|---|---|
| asmc_msgs | `src/asmc_msgs` | 팀 공용 메시지·서비스 |
| asmc_bridge | `src/asmc_bridge` | UDP 송수신, 기본 GPS/IMU 상태 |
| asmc_camera | `src/asmc_camera` | YOLO·TwinLite·ResNet, 차선 family·시각화 |
| asmc_hd_map | `src/asmc_hd_map` | 지도 제품·global/local 발행·mission metadata |
| asmc_planner | `src/asmc_planner` | Behavior·Execution·Frenet, 최종 제어 선택기 |
| asmc_controller | `src/asmc_controller` | MPC 추종, `/ctrl_cmd/normal` |
| planning_inputs | `src/planning_inputs` | 판제 전용 GPS/IMU 상태와 LiDAR wire 변환 |
| lane_boundary_adapter | `src/jamming_lane_boundary_adapter` | 음영 family 검토·BBox 차로/재밍 제어; launch별 권한 다름 |
| data_collection | `src/data_collection` | Capture·bag 기록/재생·dataset lineage |
| asmc_lidar | `src/asmc_lidar` | 좌표/시간 코어와 인계 설정·시험; 실행 노드 미구현 |
| Main, Msg | `src/lidar+sensorfusion/src/{Main,Msg}` | 현재 LiDAR·전방 fusion 실행과 구형 wire |

이름 변경이 완료된 `src/asmc_perception` 폴더와 이전 안내는 제거했습니다. 카메라 정본은 `src/asmc_camera`입니다.
삭제된 `behavior_planner`·`frenet_planner` 패키지명은 내부 C++/파라미터 namespace와 구분합니다.
`lane_boundary_adapter`의 실제 폴더명은 `jamming_lane_boundary_adapter`입니다.

nested LiDAR에는 `CATKIN_IGNORE`가 있어 바깥 빌드와 별도로 빌드합니다.
바깥 setup → nested setup 순서와 [직접 명령](../operations/commands.md)을 따릅니다.
공용 `morai_msgs` submodule과 nested vendor/SDK 코드는 팀 소스 주석 정리에서 제외합니다.
외부 원본을 편의상 수정·복제하지 않습니다.

## 데이터·모델·문서

이미지·bag·대형 manifest·학습 checkpoint는 `$ASMC_DATA`와 별도 artifacts에 둡니다.
예외로 검증된 YOLO/TwinLite 배포 파일은 `asmc_camera/models`의 실제 Git 파일입니다.
버전·SHA·지원 GPU/runtime은 [모델 배포](../operations/camera-deployment.md)를 따릅니다.
package-local docs는 내부 설계, 중앙 contracts/handoffs는 다른 파트의 소비 규칙을 소유합니다.

`tools/scen_runner`는 최신 통합/구간 검증 실행기이며 `tools/aim_scenario_runner`와 구분합니다.
개인 local.yaml·DLL 설치본·주행 기록은 Git에 올리지 않습니다.
이전 경로·실험은 [기록](../records/README.md), 현재 목표·담당은
[통합 로드맵](consolidation-roadmap.md)과 [ownership](../onboarding/ownership.md)을 봅니다.
