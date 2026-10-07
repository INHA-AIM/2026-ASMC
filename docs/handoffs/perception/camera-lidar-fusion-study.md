# 카메라와 LiDAR 장애물 융합 분석 및 개발 계획

> 상태: Working Proposal / 참고 구현 검수 기록. 운영 적용이나 성능 수락 아님.
> 기준일: 2026-10-07. 코드 검토 기준 main `7aed612`, 재호 구조물 처리 브랜치 `8dde9b6`.
> 대상: 카메라 안승현, LiDAR 손재호, 음영 회피 이강태 및 판제 소비자.
> 연결: [현재 객체 생산자](camera-objects-p2a.md), [객체 검출 인계](camera-object-detection.md),
> [LiDAR 인계](../../packages/asmc_lidar/README.md), [인지 융합 로드맵](../../packages/asmc_perception/roadmap.md).

일반 정적·동적 장애물 구간은 기존 LiDAR와 YOLO 융합을 유지한다. 음영구간에서는
터널 벽·천장과 장애물이 섞이는 문제를 카메라로 보완할 수 있는지 먼저 확인한다.
우선 후보는 **기존 카메라 검출과 LiDAR 점의 기하 연결**, 그다음은 **YOLO26-seg로
객체 검출기를 교체하여 마스크까지 이용하는 방법**이다. 단안 depth 모델 추가는 후순위다.
목표는 깨끗한 점군 화면이 아니라 장애물 누락·구조물 혼합을 줄인 위치와 점유 범위를
판제에 전달하는 것이다. 회피 정책과 안전 정지는 판제가 소유한다.

이 기록은 소스 검토와 공식 모델·데이터 제공 여부에 근거한다. 새 모델의 가중치 로드,
실행 환경 호환, 우리 GPU 벤치마크, 실제 주행 성능은 아직 이 비교로 검증하지 않았다.
다른 개발 작업의 브랜치·컨테이너·실행 환경을 변경하지 않고 격리 실험부터 진행한다.

## 현재 구현과 문제를 구분하는 방법

| 경로 | 검토된 상태 | 후속 개발에서 주의할 점 |
|---|---|---|
| `asmc_camera` 객체 생산자 | 전방 YOLO11m, `conf=0.7`, `bus/truck/person/car`, box 공용 메시지 | opt-in 생산자이며 신규 mask 필드는 없음. 현재 class 그룹에 `obstacle`이 있어도 임의 장애물을 검출한다는 뜻은 아님 |
| 기존 `Main` 융합 | LiDAR 군집·OBB 이후 영상으로 투영하여 YOLO box와 연관·분류 | 벽과 차량이 이미 합쳐진 군집은 클래스만 붙여도 분리되지 않음 |
| 재호 `8dde9b6` 터널 보호 | 이미 도착한 YOLO box와 깊이 조건으로 점 보호 | box 안의 배경 혼합, 50ms stamp 매칭, 미수신 정책을 별도 평가. 새로운 box 보호 아이디어인 것처럼 중복 구현하지 않음 |
| 음영 카메라 어댑터 | 영상으로 투영한 차로와 box 하단의 겹침을 이용한 소속 판단 | 현재 거리를 추정하지 않음. 차로 소속과 미터 단위 위치·속도는 다른 정보 |
| 목표 `asmc_lidar` | 공통 좌표·시간 골격과 인계 계약 | 이 기준 main에는 완성된 검출 실행기가 없음. 실제 비교 대상은 nested `Main` |

확인할 소스는 [객체 설정](../../../src/asmc_camera/config/camera/yolo11m_coco.yaml),
[FusionCamera.cpp](../../../src/lidar+sensorfusion/src/Main/src/SensorFusion/FusionCamera.cpp),
[LiDAR 처리](../../../src/lidar+sensorfusion/src/Main/src/Lidar/LidarProcess/LidarProcess.cpp),
[음영 box 차로 판단](../../../src/jamming_lane_boundary_adapter/src/camera_bbox_lane_node.cpp)이다.

현재 main의 [main.cpp](../../../src/lidar+sensorfusion/src/Main/src/main.cpp)는
`pose_ready && LidarProcess(...)`를 사용하며,
[SensorPoseHistory.cpp](../../../src/lidar+sensorfusion/src/Main/src/Parser/SensorPoseHistory.cpp)는
stale GPS를 거부한다. 따라서 카메라 모델만 바꿔도 GPS 중단 중 LiDAR local 결과가 계속
나온다고 가정할 수 없다. **local 인지 지속과 map 출력 가용성을 분리하는 작업**이 선행 조건이다.
이것은 소스상의 의존성 지적이며 이번 검토에서 live 장애를 새로 재현한 결과는 아니다.

main 처리에는 pitch 수평화된 점을 `velodyne`으로 표기하는 경로도 있다. 해당 debug 점을
원본 센서 좌표로 가정하여 영상에 바로 투영하면 안 된다. 첫 격리 비교는 raw
`/velodyne_points`에서 시작하거나, 필터 점의 수평화 행렬과 좌표 계약을 먼저 확인한다.

객체 추론을 띄울 수 있는 경로는 `asmc_camera`, 기존 `Main`, 음영 전용 detector에 각각 있다.
동시에 세 개가 실행 중이라는 뜻은 아니다. 향후 공유 생산자는 하나를 목표로 하지만
전방 원본 영상과 음영 hood crop의 범위·해상도·threshold가 다르므로 단순 remap으로
동등성을 선언하지 않는다. 음영 어댑터의 차로 변경 중 box 무시 정책도 새로운 거리 정보를
연결하기 전에 판제 담당과 검토한다. 회피 기동 유지와 충돌 감시는 별개여야 한다.

## 공개 모델과 데이터의 적용 가능성

공개 데이터, 사전학습 가중치, 우리 시스템에서의 즉시 적용 가능성을 구분한다.
가중치로 추론만 할 때는 전체 학습 데이터셋을 내려받을 필요가 없다. 공개 데이터도
이용 조건·접근 절차가 있고, 코드 라이선스와 가중치·데이터 라이선스는 별도로 확인한다.

| 방법 | 데이터와 공개 자산 | 우리에게 필요한 추가 작업 | 현재 우선순위 |
|---|---|---|---|
| YOLO11 및 YOLO26 instance segmentation | COCO 80-class box·mask, 공식 사전학습 가중치 | 원본 영상 좌표의 mask 복원, 모델 호환 및 MORAI 평가 | 높음. 추가 네트워크보다 기존 객체 모델 교체 |
| PIDNet | Cityscapes·CamVid 데이터와 학습 가중치 | 터널 구조물 일반화 검증. 기존 white/yellow/stopline 계약과 다름 | 구조물 semantic이 꼭 필요할 때 |
| Frustum PointNets | KITTI·SUN RGB-D 평가, KITTI 모델과 전처리 자료 링크 | 오래된 TF 환경, 센서 보정·점 분포·클래스 적합성 | 구조 참고. 전체 모델 도입은 후순위 |
| PointPainting | 클론한 제3자 구현은 KITTI, segmentation과 OpenPCDet 활용 | semantic score가 추가된 점을 받는 3D detector 학습·연결 | 투영·점 계보 참고 |
| CLOCs | KITTI와 2D/3D detector·융합 가중치 링크 | detector별 후보·점수 분포 및 실행 환경 적합성 | 이미 형성된 객체 후보의 점수 융합용 |
| ByteTrack | MOT17/20·CrowdHuman 등 공개 자료, detector 모델 제공 | 우리 box에 추적 연관 연결, 다중 클래스·관측 시간 검증 | 선택적 추적. 연관 부분에 새 neural model 불필요 |
| Depth Anything V2 | 공개 상대 depth 가중치, 별도 metric depth 모델 | 상대값과 미터 구분, 카메라·터널 거리 오차 검증 | LiDAR 지지가 부족할 때 |
| UniK3D | 공식 monocular 3D 코드·가중치 | 입력 카메라 조건·거리·좌표 검증과 부하 측정 | 단안 3D 비교가 필요할 때 |

자료: [YOLO segmentation](https://docs.ultralytics.com/tasks/segment/),
[COCO masks](https://docs.ultralytics.com/datasets/segment/coco/),
[PIDNet](https://github.com/XuJiacong/PIDNet),
[Frustum PointNets](https://github.com/charlesq34/frustum-pointnets),
[PointPainting 제3자 구현](https://github.com/Song-Jingyu/PointPainting),
[CLOCs](https://github.com/pangsu0613/CLOCs),
[ByteTrack](https://github.com/FoundationVision/ByteTrack),
[Depth Anything V2](https://github.com/DepthAnything/Depth-Anything-V2),
[metric depth](https://github.com/DepthAnything/Depth-Anything-V2/tree/main/metric_depth),
[UniK3D](https://github.com/lpiccinelli-eth/UniK3D).
각 저장소의 모델 제공 안내를 확인한 것이며 모든 weight 다운로드 링크의 가용성과
전체 학습 데이터의 동일 조건 재현 가능성을 보장하지 않는다.

### YOLO26 segmentation 적용 범위

공식 `yolo26n/s/m/l/x-seg.pt`는 box·클래스·confidence·객체별 mask를 함께 제공한다.
YOLO11m-detect와 seg를 모두 상시 실행하기보다, seg 하나로 기존 검출 출력을 유지하면서
mask를 추가하는 후보를 우선한다. [YOLO26 모델 안내](https://docs.ultralytics.com/models/yolo26/)

COCO의 `person/car/truck/bus`는 현재 허용 라벨과 연결 가능하지만 confidence 분포와
검출 결과가 같지는 않다. 터널 벽·천장·가드레일이나 임의 장애물 전체를 분류하는 모델도
아니다. **mask 바깥은 배경 또는 안전 공간이라는 뜻이 아니다.** mask는 영상 영역일 뿐
거리나 3D 객체 전체 형상을 직접 제공하지 않는다.

seg 결과의 boxes만 사용해도 mask head 계산 자체가 사라지는 것은 아니다. boxes-only
비교는 mask의 후단 사용 효과를 분리하는 실험이다. 모델 크기·GFLOPs·공식 T4 표만으로
4060 Ti/4070 SUPER의 동시 실행 FPS나 VRAM을 환산하지 않는다.

YOLO semantic/depth 등 다른 task 모델이 존재하더라도 하나의 seg checkpoint가 자동으로
그 출력을 모두 내는 것은 아니다. 기존 TwinLiteNet+ 차선 모델과 신호등 ResNet18은 유지한다.
COCO traffic light 검출은 적색·녹색 상태 인식의 대체가 아니다. shared backbone으로
여러 task를 통합하려면 라벨·head·학습·소비 계약을 다시 검증해야 한다.

## 논문과 참고 구현의 차이

### Frustum PointNets와 PointPainting

Frustum PointNets는 2D box가 만드는 공간 안에서 학습된 PointNet으로 점을 분할하고
3D box를 회귀한다. box 내부 점의 깊이·연결성으로만 분리하는 우리의 경량 후보는 이
아이디어를 참고하지만 **논문 모델의 재현이나 논문 정확도 계승은 아니다**.
검토 구현은 점 재표본화와 KITTI 클래스 조건, TF 1.x/custom ops 의존성이 있다.
[CVPR 2018 논문](https://openaccess.thecvf.com/content_cvpr_2018/html/Qi_Frustum_PointNets_for_CVPR_2018_paper.html)

PointPainting은 영상 semantic score를 점에 붙여 학습된 3D detector가 소비하는 방식이다.
객체 instance mask로 점을 선택하는 기하 어댑터와 동일하지 않다. 검토한 제3자 구현의
`painting/painting.py`는 카메라 시야 밖 점을 제외하는 경로가 있으므로 이를 우리
360도 LiDAR 안전 필터로 복사하지 않는다.
[CVPR 2020 논문](https://openaccess.thecvf.com/content_CVPR_2020/html/Vora_PointPainting_Sequential_Fusion_for_3D_Object_Detection_CVPR_2020_paper.html)

### CLOCs와 ByteTrack

CLOCs는 이미 생성된 2D/3D 후보의 점수를 융합한다. 벽과 차량이 하나로 합쳐진 점군을
직접 분리하는 해결책은 아니다. 작은 fusion head 비용만 보고 SECOND 등 전체 의존 비용을
제외하지 않는다. 외부 레포 요약에 적힌 지연은 우리 장비 측정값이 아니다.

ByteTrack은 높은 confidence 연관 뒤 낮은 confidence 관측도 이용한다. 앞단 `conf=0.7`에서
전부 제거하면 그 복구 근거를 활용할 수 없다. threshold 변경은 별도 실험으로 분리한다.
원본 tracker의 프레임 간격·다중 클래스 가정을 점검해야 하며, 영상 ID나 픽셀 속도를
미터 단위 속도·동정적 판정으로 바로 넘기지 않는다.

## 권장 융합 구조와 안전 계약

`공유 카메라 검출 또는 mask + raw LiDAR → 시간·좌표 정합 → 가시성·깊이·연결성 연관
→ 객체·구조물 경계·미확정 → 차로 및 경로 관련성 → 판제`를 격리 후보로 비교한다.

1. 영상 원본 stamp와 LiDAR 관측 stamp, 각 결과의 도착 시각을 구분한다. 늦게 도착한
   결과를 미래 정보로 소급 적용하거나 오래된 stamp를 최신으로 바꾸지 않는다.
2. 원본 영상 크기·camera ID·crop·resize·letterbox 역변환과 보정 SHA를 고정한다.
   마스크 배열 크기가 원본 영상과 같다고 가정하지 않는다. 픽셀 경계·부분 가림을 검증한다.
3. 센서 장착 변환과 실제 수평화 행렬을 한 번씩 적용한다. 공개 local geometry는 후륜축
   `base_link` 계약에 맞춘다. map 결과는 유효한 관측 시점 pose가 있을 때만 만든다.
4. mask/box 안에도 배경 벽과 여러 깊이의 점이 있을 수 있다. 같은 관측의 깊이·연결성으로
   분리하고 지지점이 부족하면 위치·범위를 미확정으로 남긴다. 관측 표면만으로 물체 전체
   크기를 정확히 안다고 하지 않는다.
5. 카메라 FOV 밖, 미지원 클래스, 가림, 미수신·stale·빈 검출은 서로 구분한다. 이 조건을
   LiDAR 객체 삭제 강화로 바꾸지 않는다. 보이지 않는 일반 장애물도 보존할 경로가 필요하다.
6. 벽·가드레일은 추적 차량이 아니더라도 충돌 경계다. 삭제해 자유 공간으로 만들지 않는다.
   카메라 drivable 영역에서 장애물 때문에 생긴 구멍도 삭제 근거가 아니다.
7. 현재 경로 밖이라는 이유로 인접 차로 차량이나 횡단 가능 객체를 버리지 않는다.
   객체 footprint와 차로·주행가능영역의 겹침 및 모호성을 따로 제공한다.

현재 [CameraObjectBoxArray](../../../src/asmc_msgs/msg/CameraObjectBoxArray.msg)와
[CameraObjectBox](../../../src/asmc_msgs/msg/CameraObjectBox.msg)는 box 계약이다.
새 mask는 먼저 격리 산출물로 검증하고, 운영 연결 때 additive 계약을 합의한다.
기존 메시지 MD5와 `/YOLO/BBox` 호환 출력은 조용히 바꾸지 않는다. 프레임 검출 번호와
track ID, 상대 속도와 절대 속도, 미계산과 유효한 0속도도 구분해야 한다.

카메라 단독 fallback을 비교한다면 차로 평면과 box 접지점의 교차로 거리를 추정하는
가벼운 기준선부터 둔다. 경사·가림·box 잘림에서는 오차가 크므로 유효성·불확실성과 함께
평가한다. 단안 depth 추가 여부는 이 기준선과 LiDAR 지지 부족 사례를 보고 결정한다.

## 단계별 실험과 선택 기준

아래는 실행 예정안이며 구현 완료 목록이 아니다. 이전 external 검수 기록의 YOLO11-seg
우선안은 보존하고, **사용자의 YOLO26 업그레이드 의도를 반영해 26을 첫 seg 후보로 변경**한다.
기존 객체 인계의 긴급 YOLO11m 경로와 운영 기본값은 그대로다.

| 단계 | 고정하거나 바꾸는 것 | 분리해서 확인할 효과 |
|---|---|---|
| B0 | 현재 YOLO11m과 LiDAR 설정 고정 | 기준 출력, 실패 사례와 전체 부하 |
| B1 | 같은 YOLO11m 결과와 같은 점군에 기하 연관만 추가 | 새 학습 모델 없이 가능한 분리 효과. 기존 재호 box 보호와 대조 |
| B2 | YOLO26s detect와 B1의 같은 후단 | 세대·크기 변경 효과. 11m 대비 순수 세대 효과로 부르지 않음 |
| B3a | YOLO26s-seg로 교체하되 후단은 boxes만 소비 | seg 모델 자체의 검출 변화와 비용 |
| B3b | B3a와 동일 추론 결과에 masks도 소비 | mask 사용만으로 객체·벽 분리가 개선되는지 |
| B4 | s-seg의 원거리·가림 성능이 부족할 때만 m-seg | 자원 증가를 정당화할 정확도 이득 |

YOLO11s-seg는 26 환경 호환이 막히거나 이전 가설과의 대조가 필요한 경우의 보조 후보다.
n-seg는 측정된 자원 부족이 있을 때만 추가한다. depth·PIDNet·SAM·tracker 교체를 동시에
섞지 않는다. SAM은 필요하면 오프라인 annotation 보조로 검토한다.

### 입력과 평가

- raw bag·동기 자료의 센서 프로필, 영상·LiDAR·IMU timestamp 출처와 calibration을 동결한다.
  Capture는 단일 프레임 기하·검출 평가에 쓰고 연속 속도·추적 정답으로 쓰지 않는다.
- 기존 자료를 다시 쓰면 독립 시험 세트라고 하지 않는다. 추가학습은 run 단위 분리와
  미사용 검증 구간을 준비한 뒤 진행한다. AIM은 센서 위치·카메라4 차이를 별도 계약으로 둔다.
- GT semantic/instance·box의 대응을 확인한 항목만 정량 정답으로 쓴다. instance 0이나
  Standing OBJ를 하나의 물체 또는 터널 벽 정답으로 취급하지 않는다. GT는 추론 입력 금지다.
- 일반도로·정적/동적 장애물·터널 진입/내부/복귀, 인접·횡단·가림·미지원 장애물을 구분한다.
  객체 누락, 벽 혼입, 객체 합침/분할, 위치·관측 범위 오차와 미확정 비율을 기록한다.
- 같은 입력의 B0 대비 새로운 중요 객체 소실이 없어야 한다. 카메라 지연·빈 검출·누락,
  GPS 중단·복귀에도 local 객체가 무조건 사라지지 않아야 한다. 더 깔끔한 화면만으로 채택하지 않는다.

### 하드웨어와 통합 부하

목표는 RTX 4070 SUPER 12GB 또는 4060 Ti다. 4060 Ti는 8GB/16GB가 있으므로 실제 장비
확인 전에는 8GB를 용량 하한으로 잡는다. 시뮬레이터는 다른 PC에서 실행하지만 코드 PC의
네트워크 수신·영상 decode·CPU 복사·LiDAR·판제 부하는 남는다.

TwinLiteNet+ 전/좌/우와 BEV, 신호등 ResNet18, 공유 객체 모델 한 개, LiDAR, 판제,
bridge를 함께 측정하고 recorder/RViz 유무도 별도 기록한다. 카메라 세 뷰 합산 FPS를
전방 단일 카메라 Hz로 쓰지 않는다. 객체 모델 단독 결과로 full-stack을 승인하지 않는다.

- 원본 입력부터 최종 출력까지 age/지연 p50·p95·p99, 관측 매칭 성공률과 원인별 누락.
- 전처리·추론·mask 복원·점 투영·연관 비용과 전체 처리율, 큐 크기·대기시간.
- GPU peak VRAM·사용률과 CPU 사용량, 계속 증가하는 메모리·큐가 없는지.
- 15Hz LiDAR의 66.7ms는 입력 간격이지 무조건적인 전체 파이프라인 deadline이 아니다.
  판제와 합의한 최대 source age·유효거리·주행속도별 여유로 최종 수락한다.
- 동일 해상도·batch 1·정밀도·threshold로 먼저 비교한다. FP16/TensorRT, 입력 크기,
  threshold 변경은 이후 독립 변수다. YOLO26의 NMS/end-to-end·export 설정도 기록한다.

실험별 코드·weight SHA, 설치 버전, 보정·설정·입력 manifest와 실패 사례를 남긴다.
조건을 통과한 후보만 별도 opt-in 연결을 검토하며, 실패하면 기존 YOLO11m 경로로 복귀한다.
모델 비교·mask 계약·판제 closed-loop 승인은 각각 독립 단계다.

## 참고 구현 고정 기록

워크스페이스의 `external/baselines/` 아래를 읽기 전용 참고로 사용한다. 팀원이 같은 개인
디렉터리를 갖고 있다고 가정하지 않도록 URL과 전체 revision을 여기에 보존한다.
기존 external checkout·학습 자산을 삭제하거나 runtime 의존성으로 추가하지 않았다.

| 디렉터리와 출처 | 검수 SHA | 확인 범위 및 라이선스 |
|---|---|---|
| [frustum-pointnets](https://github.com/charlesq34/frustum-pointnets) | `2ffdd345e1fce4775ecb508d207e0ad465bcca80` | 공식. README·model v1·provider. Apache-2.0 |
| [PointPainting-community](https://github.com/Song-Jingyu/PointPainting) | `ec732f1ae103234d60040194df3bf4067bc37b92` | 제3자 재구현. README·painting.py. root MIT, 포함 구성요소 별도 확인 |
| [ultralytics-reference](https://github.com/ultralytics/ultralytics) | `ce1399439b93c490b648b55dedd0a6334f0199da` | 공식. YOLO11 detect/seg YAML·seg predictor. AGPL-3.0, vendor 별도 라이선스 선택지 |
| [ByteTrack](https://github.com/FoundationVision/ByteTrack) | `d1bf0191adff59bc8fcfeaa0b33d3d1642552a99` | 공식. byte_tracker.py·LICENSE. MIT |
| [CLOCs](https://github.com/pangsu0613/CLOCs) | `d7d78f69fcd2b30147f0448f0606df3551975472` | 기존 checkout. README·fusion.py. MIT |
| [PIDNet](https://github.com/XuJiacong/PIDNet) | `4c158cf24ce432f0a8cb43364fae38d93cee0dc3` | 기존 checkout. README·LICENSE만 검수. MIT |

새로 확보한 것은 첫 네 checkout이며 나머지는 기존 자료를 유지했다. 각 항목은
빌드·실험 완료가 아니라 위 범위의 소스 검수다. YOLO26의 공식 제공 여부는 링크된
문서로 확인했으며, **이 표의 YOLO11 코드 검수를 YOLO26 실행 검증으로 확대하지 않는다**.
fresh Ultralytics checkout은 현재 설치 버전이 아니다. 운영 컨테이너에서 즉시 pip upgrade하지 않는다.

개인 external의 `fusion-reference-audit-20261007.json`은 최초 검수 snapshot으로 남긴다.
그 파일의 B2 YOLO11-seg 제안보다 이 문서의 최신 YOLO26 비교 순서가 우선하며,
모델 채택과 메시지 변경에는 여전히 별도 검증·승인이 필요하다.
