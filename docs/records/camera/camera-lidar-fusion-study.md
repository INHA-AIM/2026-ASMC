# 카메라와 LiDAR 장애물 융합 분석 및 개발 계획

> 상태: Working Proposal / 참고 구현 검수 기록. 운영 적용이나 성능 수락 아님.
> 기준일: 2026-10-07. 코드 검토 기준 main `7aed612`, 재호 구조물 처리 브랜치 `8dde9b6`.
> 대상: 카메라 안승현, LiDAR 손재호, 음영 회피 이강태 및 판제 소비자.
> 연결: [현재 객체 생산자](camera-objects-p2a.md), [객체 검출 인계](camera-object-detection.md),
> [LiDAR 인계](../../handoffs/lidar-migration.md), [인지 융합 로드맵](../../architecture/perception-roadmap.md).

## 2026-10-18 추가학습 자산과 HARP·CenterPoint 후속 실행

**작업 격리:** `research/lidar-perception-20261018` / `2026-ASMC-lidar-research`에서 진행한다.
이전 음영 비교 도구·문서·portable 증거는 `4da2a5b0`에 커밋해 옮겼다. 공유 worktree에서는
그 커밋과 바이트가 같은 우리 신규 파일과 우리 문서 patch만 제거했다. 다른 채팅의 데이터
이관·IMU·운영 source, 기존 컨테이너와 데이터 경로 설정은 수정하지 않았다.
아래 결과는 [후속 비교 갤러리](../../../config/baselines/evidence/lidar_learned_candidates_v1/index.html),
[기계 판독 결과](../../../config/baselines/evidence/lidar_learned_candidates_v1/summary.json),
[mask 재사용 감사](../../../config/baselines/evidence/lidar_learned_candidates_v1/segmentation-reuse-summary.json)에 있다.

### TwinLite의 노면 출력과 객체 seg를 같이 사용할 수 있는가

가능하다. **기존 TwinLite의 노면/노면표식과 E8 검출을 유지하면서 객체 instance mask를
학습하는 경로를 먼저 비교**한다. 이전 절의 공개 COCO seg 저순위는 무학습 즉시 교체에
대한 판정이다. MORAI native mask로 추가학습할 수 있는 현재 조건에서는 YOLO26-seg도
다시 우선 후보이며, zero-shot 결과만으로 YOLOE가 학습 후에도 우수하다고 결론내리지 않는다.

TwinLite drivable은 정적 노면 의미이며 객체 픽셀을 loss에서 ignore하는 정책이다.
따라서 높은 drivable 확률은 장애물 부재가 아니고 낮은 확률은 벽/천장이라는 뜻도 아니다.
기존 지면과 실제 지지 높이를 함께 확인하는 노면 문맥으로 사용한다. 카메라 영상의 mask와
3D 표면은 다르므로 같은 시각·보정·가시성·깊이·점 연결을 확인하고 LiDAR FOV/카메라 FOV
밖과 미검출/미확정 점은 보존한다. 객체 mask는 box 안 벽/바닥 픽셀을 덜 포함할 수 있으나
실제 지지점이 없는 scan112 같은 경우의 3D 위치를 새로 만들어주지는 않는다.

공유 backbone으로 클래스 확장도 가능하다. 다만 노면과 그 위의 차선은 중첩되므로 기존
drivable/road-marking head와 새 object/structure head를 분리하는 구성이 자연스럽다.
semantic의 car class만 늘려도 차량별 instance/ID·3D 크기·속도가 생기지는 않는다.
기존 LiDAR 군집·연관·④ 추적 경로는 계속 필요하다.

| 학습 후보 | 지금 데이터로 가능한 일 | 추가 작업과 판단 |
|---|---|---|
| **YOLO26-seg** | native instance mask와 확정된16class를 이용한 box+mask 학습 | **우선 학습 기준선**. E8와 같은 크기의 seg에 backbone/neck 및 호환 head 가중치를 옮기는 안과 s/m-seg 추가학습을 구분. E8 detection-only checkpoint 자체에는 mask head가 없음 |
| **E8 검출 고정+mask branch 학습** | 부분 라벨 중 mask가 확인된 positive만 mask loss에 사용 | 검출 보존을 겨냥한 실험안. 대응되는 가중치/클래스 순서 확인, bbox/cls branch와 BN 통계 고정, 전후 검출 parity가 필요. 이번에 구현/학습하지 않음 |
| **YOLOE26s-seg** |16종의 고정 클래스/prompt와 native mask로 domain 적응 | 비교 후보. 무학습89 prompt의 성능을16class fine-tune 성능으로 간주하지 않음. prompt/클래스 export와 partial-label loss 검증 필요 |
| **TwinLite 보조 semantic head** | 노면/표식 head를 유지하고 object/structure/unknown의 보조 분류 학습 | backbone 공유의 계산 이점은 가설. 현재 lane/drivable 회귀, 희소 작은 물체와 head 용량, decoder 비용을 확인. 우선 coarse 보조 head가 전체16class instance 통합보다 범위가 작음 |
| **PIDNet-S** | Capture semantic과 actor mask를 이용한 도로/객체/구조물 semantic 학습 | domain 적응 기준선으로 가능. 기존 TwinLite와 병렬이면 추가 backbone 비용, 교체하면 노면표식·소비 계약 재검증. 자체 출력은 instance 분리가 아님 |

### 실제 E8 학습 데이터의 mask 재사용 감사

`camera_objects_bbox_v11-preflight-r4/manifest.jsonl`의 **41,666 image row**를 읽었다.
train **41,345**, selection205, heldout116이다. 원본 manifest SHA는
`9b42a32cb11e23324a5e1f13c114cc01d734cc5539ab0980293b6a9e2fb9d3ed`다.

- train positive annotation 중 **64,966개는 native RGBA**, **5,162개는 저장된 packed mask**
  연결 근거를 가진다. 합70,128개는 반복 관측을 포함한 주석 수이며 독립 객체 수가 아니다.
- train **80,485개 주석은 box-only**, 3,924개는 검수가 필요한 polygon 필드다. box/teacher
  사각형을 그대로 instance mask 정답으로 만들지 않는다.
-16개 클래스마다 최대3개씩 **train48개 mask**를 실제 열어 RGB 크기, 가시 mask bbox와
  기존 box의 최대2pixel 대응을 확인했고48개 모두 통과했다(RGBA45, packed3).
  모든 주석의 파일 존재·정확도를 전수 수락한 결과는 아니다.
- train image의 supervision은 complete1,473 / partial39,731 / reviewed_negative141이다.
  positive 전부에 mask 연결 근거가 있고 complete인 프레임은 **train186**이다.
  해당 작은 seed의 클래스는13종이며 deer/motorcycle/bollard를 포함한16종 전체 학습/평가를
  충족하지 않는다. selection151·heldout74는 가용성만 조사했으며 학습에 넣지 않았다.
- 즉 표준 seg 학습에 모든41,345장을 즉시 넣는 안과, 약7만 positive의 mask 근거를 활용할
  수 있다는 판단은 다르다. 기존 bbox trainer의 partial/ignore 처리를 seg loss에도 보존하거나
  native manifest+spawn ledger로 완전 GT subset을 다시 내보내야 한다.

새 대규모 수집을 먼저 전제할 필요는 없다. 우선 기존 native mask·actor ID·spawn ledger의
클래스 연결을 재사용하고, 누락된 완전 라벨/희소 클래스 및 터널 벽 근처·부분 가림·높은 차량·
낮은 보 사례를 보충한다. run/layout 단위 split을 유지하며 기존 개발 bag을 독립 holdout으로
부르지 않는다. 1Hz Capture도 프레임별 mask 학습에는 사용할 수 있다.

**벽/천장 라벨의 한계:** 현재 palette의 `standing_object=(113,178,37)`와 building은
벽·천장 전용 클래스가 아니다. 기존 drum 주석도 standing_object의 고립 component와
검수 근거를 이용했다. 따라서 standing_object를 일괄 구조물 삭제 정답으로 만들면 안 된다.
벽/천장을 따로 학습하려면 클래스 정의와 추가 GT가 필요하다. Depth·자세·노면·법선으로
만든 벽/상부 표면 후보는 actor mask를 보호하고 검수한 pseudo-label로 구분한다.
저신뢰도/겹침은 ignore로 두며, 통과 공간 안의 보/돌출물은 삭제 대상에 넣지 않는다.
현재 조사한 Capture 학습 manifest에는 카메라 intensity/semantic/instance/depth/bbox가
연결돼 있지만 LiDAR point semantic 또는3D box 학습 manifest는 연결돼 있지 않았다.
이는 MORAI가 해당 GT를 제공할 수 없다는 뜻이 아니다.

### HARP-NeXt와 CenterPoint-Pillar 기본 가중치 실행

원본 음영 bag SHA `9bcb0e134a08c9b21cb9e8d29ca0e62710173cf9bebcc091d5dfc82454006b34`의
**208 scan**을 각 후보로 실행했다. 원본 XYZI를 다시 읽어 이전 비교의 XYZ와208개 전부
일치함을 확인했다. 지면 제거·군집·누적을 적용하지 않았고, 기본 가중치는 모든 state key를
strict하게 로드했다. 별도 컨테이너에만 라이브러리를 설치했다.

| 후보 | 실제 실행 구성 | 관측된 p95 비용 | Torch peak reserved | 판정 |
|---|---|---:|---:|---|
| **HARP-NeXt nuScenes** | 공식 CPU 전처리,32×480, FP16 autocast | 전처리/H2D57.2ms＋forward/output99.7ms; **합148.5ms** |814MiB | 실행 성공. 도로/구조물 근거 후보지만 VLP16/MORAI 재학습 필요 |
| **HARP-NeXt SemanticKITTI** | 공식 GPU 전처리,64×512, FP16 autocast | 전처리/H2D96.3ms＋forward/output109.3ms; **합189.5ms** |572MiB | 실행 성공. 이번 입력에서 nuScenes보다 우선할 근거 없음 |
| **CenterPoint-Pillar** | MMDetection3D1.4,0.2m pillar, FP32, circle NMS | 입력 pipeline/forward/NMS 호출 **168.1ms** |242MiB |3D box 출력 성공. 즉시 기존 검출 대체/속도 채택은 보류 |

**속도 표는 GPU를 다른 채팅의 workload와 공유한 관측값**이다. 실행 중 GPU 사용률
약78%·점유 약5GB를 관측했고 단독 GPU를 확보하기 위해 다른 작업을 멈추지 않았다.
CPU/H2D와 forward를 따로 재서 합한 p95는 각 p95의 합과 다르다. Torch allocator 수치는
driver·다른 프로세스까지 포함한 전체 VRAM peak가 아니다. 모두8GB에서 추론은 끝났지만
위 표로 단독 FPS 상한이나 MORAI 실시간 주행 수락/불가를 확정하지 않는다. 초기 smoke의
git 없는 metadata 오류, nuScenes GPU 전처리 변형의208개 출력 후 summary 저장 오류는
raw log를 보존했다. 후자는 정식 비교 표에서 제외했고 GPU metadata 분기는 이후 수정했다.

HARP는 공식 코드 `69c52ddc54e1fefaa7e64ac8e3fffea33e814eda`와 저자
[Hugging Face](https://huggingface.co/samir-abh/HARP-NeXt)의 고정 revision
`c2cd3ce5940be23bfbc2d1cf22131fd2fd3dc149`를 사용했다.
nuScenes weight SHA는 `6f7fc5a1c03e9123541458358b1c703babf37ffc0b8b4ea6beac6c9f8478a147`,
SemanticKITTI는 `5b5db652540a179b56e044d09c792d9f8332c6e4af78801293af604ce1d309e3`다.

- nuScenes intensity는 원본 그대로, SemanticKITTI는 학습 입력 범위 차이를 반영해 **/255**를
  명시했다. 후자는 보정된 reflectance 변환이 아닌 byte→unit의 입력 적응 가정이다.
- VLP16의 관측 각도 중 모델 기본 FoV 밖으로 clamp되는 점은 nuScenes **14.4%**,
  SemanticKITTI **30.5%**다. 그림에서는 회색으로 표시하며 이 점들을 삭제하지 않았다.
- scan64의 E8 상자 box 안38개 점은 nuScenes에서 모두 `manmade`, SemanticKITTI에서는
  fence33/motorcyclist3/vegetation2로 나왔다. 이38개는 배경도 섞인 box 지지점으로 객체점
  GT가 아니다. 다만 `manmade/fence`를 제거하면 객체 보호를 보장할 수 없다는 구체 근거다.
  기본 클래스에는 box나 터널 천장 전용 클래스가 없으므로 목적에 맞는 재학습이 필요하다.
- semantic은 같은 클래스의 여러 차량/장애물을 instance로 나누거나 ID·속도를 직접 내지 않는다.
  현재 기하 군집과④를 유지한 점별 보조 분류부터 검토한다.

CenterPoint는 [OpenMMLab 공식 pillar checkpoint](https://download.openmmlab.com/mmdetection3d/v1.0.0_models/centerpoint/centerpoint_02pillar_second_secfpn_circlenms_4x8_cyclic_20e_nus/centerpoint_02pillar_second_secfpn_circlenms_4x8_cyclic_20e_nus_20220811_031844-191a3822.pth),
SHA `191a38226d2ce990f26cc3f78c454310cc562d81d29e86d8ca8fa39c0a44215d`를 사용했다.
nuScenes10-sweep 학습 모델에 VLP16 **단일 scan**,5번째 time-offset0을 넣었다.
현재 scan을 여러 번 복제해 누적으로 가장하지 않았다. 기본10종에는 box/임의 장애물 class가
없고, 대표 scan64/112의 전방 상자는 score0.4 이상의 대응 box로 나타나지 않았다.
이는 class·점밀도·단일/다중 sweep 도메인이 다른 시험이며 정답 없는 mAP 평가가 아니다.
학습된 velocity 숫자가 출력돼도 이번 단일 scan에서 유효한 속도로 수락하지 않는다.
일반 차량과 box 형상 검출에 투자할 때3D box GT·클래스 추가학습·좌표/추적 연결이 필요하다.

**현재 권장:** TwinLite 노면/표식 유지＋E8 기반 객체 seg 학습을 첫 학습 비교로 두고,
필요하면 TwinLite 보조 semantic head 또는 PIDNet을 같은 coarse label로 비교한다.
카메라 FOV 밖까지 점 분류가 필요하면 HARP nuScenes를②의 domain 적응 기준선으로,
분리 후에도 형상/검출 자체가 부족하고3D GT가 준비되면 CenterPoint-Pillar를③의 기준선으로
유지한다. 모든 모델을 동시에 추가하는 계획은 아니며, 이번에는 재학습·운영 전환을 하지 않았다.

## 2026-10-18 현재 파이프라인 유지와 음영 bag 후속 비교

이 절이 대회 전 우선순위를 갱신한다. 아래의 2026-10-07 YOLO11m/B0/B1a 서술과 고정
소스 결과는 당시 기록으로 보존한다. 현재 카메라 기준선은 YOLO26l E8·3뷰·TensorRT다.
센서 융합 인코더·VLA·E2E는 사용자 결정으로 대회 이후로 미룬다.

| 순위 | 이번 비교 후보 | 확인할 질문 |
|---|---|---|
| 1 | 기존 YOLO26 E8 box + 점군 깊이/연결성 | 모델을 추가하지 않고 객체와 배경의 점 혼합을 줄일 수 있는가? |
| 2 | 공개 YOLO26s-seg | 같은 결과의 box와 mask가 선택하는 LiDAR 지지점과 추가 비용은 어떻게 다른가? |
| 3 | 공개 YOLOE26s-seg + 고정 prompt embedding | 미지원 장애물의 관측 근거가 늘어나는가? prompt가 실제 검출을 보장하지는 않음 |
| 4 | 공개 PIDNet-S Cityscapes | 벽/건물/도로 경계의 보조 의미가 있는가? Cityscapes에는 터널 천장 전용 클래스가 없음 |
| 별도 기하 비교 | PCL 평면 지지+연결성, Conditional Euclidean Clustering, depth_clustering | 구조물 표면 근거와 군집 연결을 개선할 수 있는가? 학습 가중치가 없는 구현 후보 |
| 추적 비교 | 현재 tracker의 관측/연관/초기화/재관측 진단 | 점군 변화와 무관하게 남는 ID·속도 실패는 무엇인가? 새 neural model은 필수 아님 |

목표 장비는 **RTX 4060 Ti 8GB**, 시뮬레이터 별도 PC다. 기존 지면 제거·객체 소비 계약을
기준선으로 두고 공개 모델은 하나씩 로드한다. 앞선 세대/크기/학습 데이터가 다른 모델의
비교를 mask 하나의 효과로 해석하지 않는다. 각 seg 모델 안에서 boxes-only와 mask 사용을
짝지어 비교한다. 신규 모델은 운영 설치 버전이나 기본 launch를 변경하지 않고 실행한다.

터널은 전역 최대 평면/고정 높이를 통째로 삭제하기보다 중력 대비 법선, 실제 관측된 유한한
표면 지지 범위, 돌출부와 깊이 단절을 먼저 기록한다. 천장에는 지면 대비 높이와 통과 공간이
필요하고, 트럭 측면·벽 근처 장애물·낮은 보를 보존한다. 구조물 근거와 억제 마스크는 분리한다.
객체 추적에서 제외해도 충돌 경계에서 지우지 않으며 현재 객체 메시지를 구조물 메시지로
재해석하지 않는다. [기하 후보 검수](../../handoffs/lidar-migration.md#참고-구현-검수와-현실적인-첫-제안--2026-10-05)를 따른다.

② LiDAR semantic/instance와 ③ 학습 객체 검출은 예비 경로다. 다른 클래스 혼합에는 semantic,
같은 클래스 합침에는 instance/검출, 분리 후 형상 불안정에는 형상 추정/검출을 비교한다.
시간·자차 보상·연관·초기화 원인의 속도 급변은 ④에서 먼저 해결한다. GT로 점을 정확히
나눈 입력을 기존 후단에 넣어 분리 자체의 효과를 확인하면 ②/③ 투자 판단에 도움이 된다.

실행은 기존 B1a 음영 원본 `20260912_003803_kcity2025_shadow_static_avoid_v1_sunny_11am_wheel`
bag을 첫 비교 입력으로 고정한다. 24.8초, LiDAR 208 scan, 전방 영상 494개이며 이미 여러
개발에 사용한 자료다. 이 bag에는 독립 semantic/instance/3D box/객체 속도 GT 토픽이 없다.
지지점 수 감소는 객체 보존율이나 벽 제거 정확도가 아니고, 사후 최근접 timestamp 연결은
실시간 도착/소비 정책 검증이 아니다. 연속 ID·속도는 시간열로 진단하되 GT 오차로 표시하지 않는다.

실행 산출물은 개인 workspace `artifacts/lidar-fusion-shadow-20261018-r1/`에 모으고 입력·가중치·
도구 SHA, 실행 명령, 관측 JSONL, 처리 비용, 대표 원본/투영 그림을 남긴다. 이 절에는
실제로 끝난 후보와 막힌 후보를 구분해 결과를 추가한다. 운영 기본값 승격과 closed-loop
실시간 주행 수락은 이번 오프라인 비교에 포함하지 않는다.

### 같은 날 실행 결과 — 공개 가중치 3개와 기존 E8, PCL 관측 근거

[비교 그림과 조건](../../../config/baselines/evidence/lidar_fusion_shadow_v1/index.html),
[전체 수치·가중치/소스 SHA](../../../config/baselines/evidence/lidar_fusion_shadow_v1/summary.json)를
저장했다. 같은 bag의 LiDAR **208 scan**, 전/좌/우 각 **208 대응 영상**, 모델당 **624개**를
E8 → YOLO26s-seg → YOLOE26s-seg → PIDNet-S 순서로 별도 프로세스에서 끝까지 실행했다.
현재 v3 camera4는 전상방 카메라이며 후방이 아니므로 이번 3뷰 비교에 포함하지 않았다.
추론 중 운영 launch·모델·ROS 메시지 기본값은 바꾸지 않았다.

| 후보 | 측정한 단일 뷰 호출 p95 | 이 bag에서 확인한 결과 | 다음 개발 판단 |
|---|---:|---|---|
| 기존 YOLO26l E8 TensorRT FP16 | 31.8ms | 전방 box 출력 228개 관측: `box` 200, 나머지 barrier/drum/dog 등. 대표 상자 검출 유지 | **1순위: 유지하고 기하 분리/유효성 보완**. 출력 수는 정답 검출 수가 아님 |
| 공개 YOLO26s-seg, FP32·imgsz640 | 64.4ms | 전방 208 영상에서 출력 1개(`train`). 동일 상자 장면의 검출 부재를 그림에서 확인 | COCO 가중치 그대로 E8를 교체하는 안은 후순위 |
| 공개 YOLOE26s-seg, FP32·imgsz640 | 66.9ms | 전방 `wooden crate` 67개 관측. 전체 뷰에는 airplane/train/bench 등 배경 혼동 출력도 존재 | **학습 모델 중 조건부 후속 후보**. MORAI 적응과 배경 혼동·점 처리량 제한을 먼저 검증 |
| PIDNet-S Cityscapes, FP32 | 33.6ms* | 전체 inference checkpoint를 빠짐없이 로드. 터널 바닥/구조물 분류가 대표 그림에서 불안정 | 터널 semantic 라벨로 적응하기 전 자동 억제 근거로 채택하지 않음 |

모든 detector는 `conf=0.4`로 실행하고 저장된 출력에서 `0.7`도 별도 집계했다. E8의 학습
클래스·입력 크기·FP16 TensorRT와 공개 seg의 COCO/prompt·640·FP32가 다르므로 같은
조건의 네트워크 속도 순위가 아니다. E8는 운영 backend를 사용했지만 이번 호출은 뷰별
순차 처리이며 운영 `2plus1`의 전체 공급 Hz가 아니다. *PIDNet 수치는 forward·출력 resize·
argmax·CPU 전송이며 입력 resize/정규화/H2D는 제외했다. GPU 모델별 1회 전체 bag 실행으로,
다른 CPU 진단/호스트 부하와 반복 편차를 통제한 성능 수락 시험이 아니다.

공개 가중치 출처:
[YOLO26s-seg](https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26s-seg.pt),
[YOLOE26s-seg](https://github.com/ultralytics/assets/releases/download/v8.4.0/yoloe-26s-seg.pt).
PIDNet 원저자의 대체 Drive 하위 폴더는 로그인으로 연결됐다. 대신 모델 제공자인
[Qualcomm 공개 로더](https://github.com/qualcomm/ai-hub-models/blob/v0.64.0/src/qai_hub_models/models/pidnet/model.py)가
명시한 [원본 PyTorch checkpoint](https://qaihub-public-assets.s3.us-west-2.amazonaws.com/qai-hub-models/models/pidnet/v2/PIDNet_S_Cityscapes_val.pt)를
확보했다. SHA는 `b51aa935bdb64a0779d776f38267fd49f7cce59413910abbbf0a74934b3d7c01`,
공식 PIDNet 코드 `4c158cf`의 inference state 전체 shape/key 대응을 검사했다.
새 모델 설치나 운영 Python 패키지 업그레이드는 하지 않았다.

**점 선택·분리의 실제 한계:** 고정 반경0.55m·깊이차0.75m 연결과 box 중앙을 지지하는
가까운 성분 선택을 관측 전용으로 비교했다. 이는 Frustum PointNets/PCL CEC/depth_clustering
논문의 재현이 아닌 실험용 기하 어댑터다. E8의 추가 연결 계산은 전체 프레임 p95 **3.9ms**,
box가 있는 프레임만 보면 p95 **7.2ms**였다. 하지만 raw 점 지지가 있는 242개 검출 중
42개는 조건을 만족하는 성분을 선택하지 못했다. 미선택 점은 모두 미확정으로 보존했다.
전방 scan112의 E8 상자(conf0.93)는 box 안 raw LiDAR 점이 0개다. 점 분리만으로 복구할
수 없는 관측 부족이며 검출/거리의 유효성을 구분해야 한다.

YOLOE의 큰 배경 box에는 이 어댑터의 계산량이 커졌다. box 존재 프레임 p95 **45.1ms**,
최대 **886.5ms**였다. production 지면/voxel을 먼저 거친 입력이 아닌 raw 점 실험이므로
운영 추가 지연으로 그대로 환산하지 않는다. 다만 CPU 이웃 그래프도 무제한으로 만들 수
없다는 근거다. 후보 점 수/ROI/voxel과 처리 예산을 제한하고 초과는 unknown으로 보존하는
정책이 필요하다. box→mask 지지점 감소 역시 객체 보존이나 벽 제거 정답률이 아니다.

**PCL 표면 근거:** 같은 208 scan에 raw 센서 ROI x[-5,35]·y[-15,15]·z[-4,10]m,
0.15m voxel의 첫 원본점을 사용했다. K12 법선과 RANSAC 최대4평면(잔차0.12m·150회)을
추출하고 원본점 인덱스·관측 min/max·평면 잔차를 저장했다.
법선 p95 **12.6ms**, 평면 p95 **28.4ms**, 둘의 합 p95 **38.5ms**였다(입력 준비 제외).
이는 **구조물 근거의 관측 시험**이며 기존 지면/군집을 교체하지 않았다. 아직 중력 정렬,
벽/트럭 의미 판정, 곡면 연결, RegionGrowing/CEC, 구조물 억제는 적용하지 않았다.
[평면 지지점 그림](../../../config/baselines/evidence/lidar_fusion_shadow_v1/structure-evidence.png)을
보고 IMU·지면·객체 보호를 연결할 가치와 비용을 판단한다. 추가 GPU가 없어도 CPU 비용은 있다.

**④ 추적 진단:** 2026-10-14 배포 snapshot의 `lidar_continuous_replay`를 별도 master11339에서
`common_pitch`로 실행해 208 scan 모두 local-ready·plane-valid인 출력을 보존했다.
새 2026-10-18 Main 빌드나 tracker 교체가 아니며 camera box는 재생하지 않았다. 출력 ID는
329개로, 독립 객체 수나 ID switch 정답이 아니다. 내부 상대 KF 속도 크기 최대 **58.3m/s**는
ID76·scan118·LiDAR miss24 상태였다. 관측과 예측의 분리, miss 중 CA 외삽·유효성/유지 기간을
먼저 조사할 근거이며, 실제 차량 속도나 GT 속도 오차로 해석하지 않는다.
[관측/예측 상태 그림](../../../config/baselines/evidence/lidar_fusion_shadow_v1/tracking-diagnostic.png)을
남겼다. 점 분리 개선만으로 이 예측 상태의 문제까지 해결됐다고 판단하지 않는다.

실행 후 우선순위는 **E8 유지+처리량을 제한한 기하 분리/④ 진단 → PCL 관측 근거 검증 →
필요 시 YOLOE MORAI 적응 → PIDNet 터널 적응**이다. COCO YOLO26s-seg 즉시 교체는 보류한다.
HARP-NeXt/FRNet·CenterPoint-Pillar와 official depth_clustering·PCL CEC는 이번에 실행하지
않았다. 전자는 ②·③ 예비 경로, 후자는 구조물 근거/계보 검증 뒤의 별도 비교다.
모델 단독 로드·추론은 모두 8GB에서 끝났지만 전체 GPU peak는 측정하지 않았다.
JSON의 Torch peak는 TensorRT 외부 할당을 포함하지 않는다. 차선·신호등·판제 동시 실행,
추론 결과 도착 지연, 다른 날/날씨 bag과 실시간 주행 수락은 남아 있다.

도구는 `tools/perception/benchmark_shadow_point_support.py`, `benchmark_shadow_structure.cpp`,
`summarize_shadow_point_support.py`, `plot_shadow_support_evidence.py`다. 실제 실행 카메라 코드는
`2026-ASMC-repro`의 `8de5f65` snapshot 및 결과 JSON의 파일별 SHA로 고정했다. timestamp
경계·원본점 계보·미확정 보존 회귀 3개와 모델당624/PCL208/tracker208 출력 완결성을 확인했다.

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

2026-10-08 B1a 검증 후의 좌표·GPS·출력·추적 경계와 담당별 실행 과제는
[LiDAR B1a 개선 과제](../../../src/asmc_lidar/docs/migration-tasks.md)에 정리했다.
카메라 클래스 fusion의 raw 투영 행렬과 수평화 군집 입력 불일치를 코드에서 확인했으며,
터널의 수평화 전 YOLO 보호와 구분한다. 실제 투영 오차와 수정 수락은 후속 구현에서 측정한다.

[외부 좌표 구현과 보정 검토](camera-lidar-coordinate-reference-review.md)는 6개 고정 레포의
센서/ego 기준·센서별 시각 처리와 우리 Python/C++의 54조건 수치 대조를 연결한다.
합성 행렬 일치, 실제 센서 시간 정합과 객체 보존 수락은 별개다.

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

### 2026-10-07 B0 병합 후 실행 순서

B0 PR #70은 `07cc51a`에 병합했다. 결과는 [B0 기준선](camera-lidar-b0.md)에 있으며,
미수락 부하 조합을 운영 승인한 병합이 아니다. B1은 먼저 **B1a 재호 터널 필터 검증**과
**B1b 잔여 기하 개선**으로 나눈다. [B1a 프로토콜과 직접 실행법](camera-lidar-b1a.md)을 따른다.
기존 `8dde9b6` 브랜치의 실행 루프가 현재 main과 달라 B0 GPS 중단 실패를 그 브랜치의
실패로 대신 판정하지 않는다. 필터를 SHA로 고정하고 원본 코드를 수정하지 않고 비교한다.

local/map 출력 역할은 B1a 결과와 소비자 요구를 확인한 후 LiDAR·판제 공동 설계로 분리한다.
local-only + 판제의 관측 시점 pose 변환과 local + 독립 map 어댑터를 비교하되 local 인지와
추적을 GPS/map 가용성에 종속시키지 않는다. 수평화 센서 좌표를 `base_link`로 간주하거나
두 곳에서 중복 변환하지 않는다. 이번 검증에서 운영 GPS 처리·좌표 계약을 수정하지 않는다.

COCO 클래스 확대는 YOLO26 이전의 별도 YOLO11m 시험 프로필이다. 원본 라벨을 보존하고
person/vehicle/obstacle 대응을 검토하며 신호등·표지판 등 인프라를 바로 회피 장애물로
취급하지 않는다. 기존 네 클래스 LiDAR 호환 경로와 분리하고 미지원 물체도 평가한다.
카메라 only 음영 회피는 필터 결과·클래스 확대·판제 연결 검증 전에는 확정하지 않는다.
기존 jamming 어댑터·21% crop·음영 차선 소비·판제의 구간 활성화 권한을 유지한다.

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
