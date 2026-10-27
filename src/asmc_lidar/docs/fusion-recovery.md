# 3뷰 기본 승격과 얇은 장애물·벽 처리 후속 설계

2026-10-28. PR #122는 **3뷰 확장·동등한 연산 최적화·local/map 전달 검증**으로 병합됐다.
사용자가 계산 개선 후 기본 승격을 승인했다. `fusion.views: all`이며, 전방 단독 `front`는
명시적인 `namespace:=/test/lidar` 비교 경로다. 정본 namespace는 최적화 설정을 검사한다.
semantic 정적 운동 보조는 계속 OFF다. 전체 장면 수락이나 모든
터널 장애물 보존 완료를 의미하지 않는다. [M1 실행 정본](three-view-fusion.md)을 함께 본다.

## 이번에 확인한 실행 경계

- 같은 입력48회 대조에서3뷰 터널 p95는51.637→8.519ms(83.5% 감소), 최적화 전방 대비
  +4.605ms다. 군집 연결과 동일 scan의 불변 군집/OBB 재사용으로 줄였으며 결과는 같았다.
- GPS 무효에서도 센서→수평화→실제 `base_link`와3뷰 투영은 작동한다. 카메라 관측의
  시각·보정·유효성 검사는 계속 필요하다. map이 없다는 이유로 local fusion을 중단하지 않는다.
- `tracks_local`의 기하·분류를 유지하고, `tracks_map`은 GPS/pose 무효 시 객체 없는
  `POSE_UNAVAILABLE`을 보낸다. 이 상태는 정상 빈 검출이나 센서 전체 무효와 다르다.
  map 기반 절대속도·운동 상태를 유효한0이나 정적으로 바꾸지 않는다.
- 전방1280×720·측방640×480 원본 보정, ±50ms/최대200ms/unique/IoU0.25/확인5·변경3·유예3을 유지한다.
  기존 camera inference는 이미3뷰였으므로 LiDAR fusion 확대가 모델 추론을3배 추가하지 않는다.
- 새 회귀는 각 뷰를 단독 양성 근거로 두고 GPS 정상→무효→복귀 중 실제 runtime의 fusion→typed
  출력을 검사한다. 합성 카메라/정지 장면이며 터널 물체 보존의 실측 GT 대체물이 아니다.
- 벽/천장 제거 결과는 내부 점 label·trace·debug에 있다. 벽 평면·통로 폭을 판제에 전달하는
  새 공용 메시지는 없다. 장애물 local 출력 검증을 벽 정보 API 제공 완료라고 부르지 않는다.
- 최신 main PR #123의 ‘최종 필터 후’ 화면·전후 수량을 M1 예측 숨김·점별 도로 색상과 통합했다.
  점별 도로 색상과 객체 전달 판정은 별개다. ‘처리·제외 사유’ 색상에서 구조물 제거점은 주황색이다.

고정 자료·ROS 공급 결과는 [10월27일 최적화 기록](../../../docs/records/lidar/2026-10-27-m1-tunnel-optimization.md)에
보존한다. F1 공급473.47ms 실패·초기 입력 공백, 전 클래스 물체 보존 검수는 아직 남는다.

## 승격 과정에서 수정한 연결 결함

실제 ROS 회귀에서 카메라 invalid 후 클래스가 unknown으로 복귀하지 않는 실패가 재현됐다.
3뷰 구현이 새 보호 근거만 취소하고 확정 이름은 무기한 유지한 것이 원인이었다.
모든 기여 관측원이 무효/중단이면 카메라 상태와 대기 묶음의 옛 시작 상태를 비운다.
과거 매칭 token만 만료된 정상 스트림은 기존 분류를 유지하고 보호 자격만 해제한다.
중간 후보에서 token 만료만으로 초기화했을 때 정차 bag의6scan 발행 차이가 나서 채택하지 않았다. 다른 뷰의
유효 근거는 유지하고 LiDAR 객체·미검출 횟수는 바꾸지 않는다. 복귀 후 새5회 확인을 요구한다.
수정 전 실패 fixture와 수정 후 단위/실제 ROS 통과를 함께 보존한다.

## Steel barricade: 확인한 구조와 미확정 원인

현재 후단 fusion은 기존 LiDAR track에 카메라 분류 근거를 연결한다. 미연결 YOLO 검출만으로
새3D 장애물을 생성하지 않는다. 따라서3뷰 확대만으로 군집이 없는 물체를 복구하지 못한다.
움직일 때 사용자가 관찰한 소실의 원인은 아직 해당 객체의 동시 점군/RGB/단계 추적으로 확정하지 않았다.

| 소실 가능 단계 | 현재 코드 근거 | 다음 대조 |
|---|---|---|
| 반사점 자체 부족·voxel/지면에서 손실 | 0.1m voxel, RANSAC·높이/ROI | raw→ROI→voxel→ground의 동일 원본 지지점 수·높이 |
| 군집·OBB 미생성/배경 합침 | 일반 군집 최소5점, 터널 최소10점·후속 기하 판정 | 군집 소속, 벽/지면 label, OBB 전후 support와 사유 |
| track miss·ID 교체 | 관측 기하와 추적 중심은 별개, 예측은 실제 관측 발행 대상 아님 | detection 존재 여부→association→miss 순서 |
| 전달 제외 | 관측 품질, 지도10cm·3점/10% AND OBB10%, 상부 판정 | 최종 `delivery_excluded`와 소유 필터의 사유 |

`delivery_preview.minimum_points=10`은 점군 판정 근거 요구다. 10점 미만을 모두 즉시 삭제하는
규칙은 아니며 OBB 판단·판단 불가 분기가 있다. ‘최소10점’ 하나로 소실 원인을 단정하지 않는다.
`steel_barricade`는 `static_obstacle`이다. 터널의8종 vehicle/dynamic 사전 보호와는 다르다.
static 전체를 벽 제거에서 보호하면 시설물·오검출 배경이 다시 남으므로 일괄 보호하지 않는다.

## 참고 논문·공식 코드와 적용 판단

외부 코드는 workspace `external/baselines`에 읽기용으로 두며 운영 빌드·학습 의존성을 추가하지 않았다.

| 기준선 | 고정 코드·읽은 부분 | 우리에게 적용할 부분 / 제외할 부분 |
|---|---|---|
| [LCF3D, Pattern Recognition2026](https://arxiv.org/html/2601.09812v1) · [공식 코드](https://github.com/CarloSgaravatti/LCF3D) | `a288fdf1d60366b5f4fe4a8d4e93fd1be48b6080`, MIT(내장 mmdetection3d Apache-2.0). `src/multi_view_inference.py:detection_recovery_multi_view`, `base_inference.py`, KITTI Frustum PointNet config | 미연결2D→실제 LiDAR frustum→3D 복구의 분리. 공식 구현은 학습 Frustum Localizer와 별도 checkpoint를 사용한다. 예제 최소점10이며 코드의 통과는 `>10`이다. KITTI 모델3종은 barricade를 포함하지 않아 즉시 탑재 불가. 미연결 LiDAR/시야 밖 검출을 버리는 논문 정책은 우리 unknown 보존 계약에 이식하지 않음 |
| [EagerMOT, ICRA2021](https://arxiv.org/abs/2104.14682) · [공식 코드](https://github.com/aleksandrkim61/EagerMOT) | `2792a76a044250e55e7f25b3e75cd5961366bd33`, MIT. `tracking/data_association.py` | 2D 관측의 존재와3D 위치 근거를 분리해 관리. 2D만으로 관측된3D 속도/거리 생성이나 과거 OBB를 신규 측정으로 재발행하지 않음 |
| [Autoware ROI pointcloud fusion](https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_image_projection_based_fusion/docs/roi-pointcloud-fusion/) | `bafa06574d97e6255bc7f5d8c36683574fa07ebb`, Apache-2.0. `roi_pointcloud_fusion/node.cpp`, `roi_cluster_fusion/node.cpp` | 영상 ROI의 실제 점을 추출하고 깊이로 배경을 분리하는 비학습 후보. 가까운 배경이 곧 물체라는 가정·클래스/크기 gate·ROS2 의존성은 가져오지 않음 |

LCF3D의 복구는 별도3D 학습 모델이며 BEV 그림에 박스를 투영하는 방식이 아니다. 우리에게는
**local3D 점–영상 투영을 유지하고 미연결 검출만 선택적으로 복구**하는 구조가 우선이다.
카메라 BEV의 지면 가정은 수직 barricade의 높이/깊이를 복구하지 못한다. seg mask는 배경 혼입을
줄일 후보지만, LiDAR에 없는 깊이·가려진 면을 생성하지 못한다. detect 추가학습은 계속 진행할 수 있다.
논문 benchmark 속도/정확도를 우리 파이프라인 성능으로 인용하지 않는다.

## 후속 후보: 제한된 점 지지 복구

현재 우선순위는 [음영 정지 재현과 local 판제 기반](../../../docs/architecture/shadow-planning-refactoring.md)이다.
아래 복구 설계는 보존하되 다음 PR의 확정 착수 범위로 해석하지 않는다.
상태·인수 조건은 [R1을 포함한 보류 작업표](improvement-milestones.md#보류-과제-인수-단위)에서 관리한다.

1. M0 사례 장부에 이동 중 barricade의 raw·중간 점군·OBB·track·fusion·전달 사유를 연결한다.
   어느 단계에서 사라졌는지 먼저 구분한다. 단순 추적 miss라면 새 검출 생성보다 M3/M4를 우선한다.
2. 실제 새 YOLO 관측 중 미연결 검출만 처리한다. 같은 scan context·세 뷰 투영 cache·원본 index를
   재사용하고, bbox 수/점 수/후보 수/시간 예산을 제한한다. 전체 점군의 bbox별 깊은 복사를 피한다.
3. 지면·높이 ROI를 무조건 우회하지 않고, 실제 점의 깊이 분리·연결성과 반복 관측으로 작은
   3D 후보를 만든다. 벽에 붙은 물체와 배경면 혼입·가림을 대조한다. camera-only 거리 추정은 하지 않는다.
4. 기존 LiDAR detection과 점 index·공간 중복으로 통합하고 tracker에는 같은 scan의 관측으로
   한 번만 전달한다. 과거 카메라·미도착 관측은 복구 근거로 쓰지 않는다. GPS가 없어도 local에서 계산한다.
5. 처음에는 오프라인/진단 후보만 만든다. 필요한 장애물 보존·벽 오복구·거짓 OBB·ID 안정성·p95와
   공급 공백을 대조한 뒤 별도 수락한다. 점 부족이면 unknown/복구 불가로 남기고 크기를 지어내지 않는다.

이 계획은 LCF3D의 학습 모듈을 이식했다는 뜻이 아니다. 새로운3D 관측 생성이라 현재3뷰
분류 연결과 위험·검증 범위가 다르므로 이번 PR에 섞지 않는다. 고정 사례를 확인한 뒤 하나의 후보만 구현한다.

## 재호와 대조할 벽 폭 사전정보

현재 `TunnelFilterCore::FitWalls`에는 반대쪽 확정 벽과의 평행성10도·폭4–20m를 이용해
분절된 현재 지지를 확인하는 경로가 이미 있다. 이는 코스의 실제 폭을 정밀하게 고정한 값이 아니다.
`RemoveResidualWalls`와 `ClusterAndFilter`의 반복 군집화는 이번 PR에서 줄였고, 벽/천장 판단은 유지했다.

후속에서는 양쪽 현재 벽 평면의 법선 방향 거리로 통로 폭·잔차·관측 길이·불확실성을 측정한다.
도로 횡축의 단순y차는 곡선/비스듬한 진입에서 실제 폭이 아니며, lane family 폭도 벽 사이 폭이 아니다.
알려진 폭은 평면쌍 점수·오검출 거부·제한된 탐색 범위를 돕는 **부드러운 사전정보**로만 제안한다.
한쪽 벽 미관측·입출구·곡선·폭 변화·이동 입력 무효에서는 불확실성을 키우거나 사전정보를 해제한다.

점 삭제에는 현재 지지 범위와 표면 거리, 돌출부·장애물 근거가 필요하다. family 바깥 흰선부터 벽까지의
여유 공간에 물체가 있을 수 있으므로 흰선/명목 폭 바깥을 통째로 자르지 않는다. 추정한 반대 벽을
실제 관측으로 표시하지 않는다. 벽 출력 API가 필요하면 객체와 구조물의 계약을 별도로 정하고,
현재 diagnostics JSON을 판제의 제어 입력으로 사용하지 않는다.

승현은 위 복구·카메라 연결을, 재호는 M3 관측/OBB·M4 추적과 벽 처리 검토를 이어가는 안이다.
새 벽 폭 정책·학습 모델·2D-only track 정책의 기본 적용은 각 담당의 후속 검증으로 결정한다.
