# perception

> **역할:** 인지 파트 코드·문서 진입점
> **현재 패키지 경계:** [Perception 개발·통합 가이드](../../docs/packages/asmc_perception/README.md)
> **설계·개발 이력:** [카메라·MGeo·LiDAR 인지 융합 로드맵](../../docs/packages/asmc_perception/roadmap.md)
> **팀 문서 허브:** [docs/README.md](../../docs/README.md)

## 현재 상태

- 안승현: Capture GT 기반 4-class lane/drivable 모델과 모델 독립 3-camera live 추론 완료
- 안승현: raw/result bag, 구조화 diagnostics와 3-camera fused metric BEV 구현
- 안승현: GPS 음영 왕복 2차로용 shared lane-family v3와 판제 후보 interface를 성능 bag으로 동결·인수 완료
- HD Map: Phase 2 통합 완료. 정적 lane·drivable·mission context는 별도 `asmc_hd_map`에서 제공
- 손재호: LiDAR/3D 장애물 인지 코드 유지
- 공통 BEV와 planning 메시지는 camera/map pilot 이후 합의

## 코드 지도

| 경로 | 역할 | 상태 |
|---|---|---|
| `twinlite_morai/` | MORAI dataset, 2-head loss, TwinLiteNet+ loader | 구현·학습 완료 |
| `camera_semantic_perception/` | 모델 독립 ROS live 추론·시각화·계측 | Stage C1·C2 구현·live 검증 완료 |
| `drivable_bev/` | 3-camera metric fusion, direct evidence, 음영구간 shared lane-family | v3 동결·판제 인수 완료; 전역 범위는 provisional |
| `../asmc_hd_map/` | map-frame lane·drivable·checkpoint·mission context | Phase 2 운영 map 완성; 별도 package 유지 |
| `morai_3d_detection/` | 카메라 3D 인지 연구 | 손재호 |
| `../lidar+sensorfusion/` | LiDAR perception/fusion | LiDAR 파트 |
| `../data_collection/` | Capture GT, raw/result rosbag, replay/audit, mask/dataset | 구현·UDP smoke 완료 |

단일 `asmc_perception`은 아직 Phase 4 목표다. 현재 package와 제안 topic을 구분하고, 공용
interface 변경은 [`src/interfaces/`](../interfaces/README.md)와 consumer 합의를 거쳐 승격한다.

## 문서 지도

| 문서 | 내용 |
|---|---|
| [Perception package-group guide](../../docs/packages/asmc_perception/README.md) | 현재 코드 경계, 문서 권위, runtime interface와 Phase 4 gate |
| [융합 로드맵](../../docs/packages/asmc_perception/roadmap.md) | 전체 우선순위, 코드 후보, topic과 대회판 이식 |
| [데이터 계약](../../docs/contracts/perception/dataset-labeling-split.md) | 원본, mask, ignore, manifest와 split 정본 |
| [BEV/lane 설계 노트](../../docs/packages/asmc_perception/bev-lane-interface-plan.md) | 정책 선택 근거와 상세 후보 |
| [v4 학습 평가](../../docs/records/perception/twinlitenet-v4-evaluation.md) | 현재 18 epoch 후보 성능 |
| [rosbag 재현 평가](../../docs/operations/rosbag-evaluation.md) | Real Time UDP raw/result bag과 종료된 privileged GT 실험 경계 |
| [시뮬 체크리스트](../../docs/operations/README.md) | live 및 full-stack 검증 항목 |

## 실행 환경과 데이터

| 용도 | 환경 |
|---|---|
| ROS sensor bridge | `docker/ros-noetic/`, `scripts/udp_bridge.sh` |
| PyTorch 학습·GPU 추론 | `docker/perception-train/`, `scripts/docker_perception_up.sh` |
| 외부 TwinLiteNet+ | `/opt/baselines/TwinLiteNetPlus` read-only mount |
| 원본·파생 데이터 | `$ASMC_DATA` (`/data`) |
| checkpoint·평가 artifact | `artifacts/perception_eval/`, Git 제외 |

MORAI와의 본선 경계는 허용 UDP만 사용한다. ROS는 팀 PC 내부 모듈 연결에 사용하며
GT/gRPC/Object oracle은 배포 추론 입력에 포함하지 않는다.
