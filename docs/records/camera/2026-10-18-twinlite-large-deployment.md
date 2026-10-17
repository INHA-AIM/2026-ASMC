# TwinLite Large v7 E8 선정·배포 검증

2026-10-18 사용자 요청으로 완료된1–8epoch에서 배포 후보를 선정하고 검증을 완료했다.
현재 상태는 **사용자 승인으로 Large v7 E8 TensorRT 기본 전환 완료**다.
앞선 보류 판정과 SUNNY freshness·공급 지연 실패는 보존하며 전체 자동 수락으로 표시하지 않는다.

## 선정과 학습 종료

`twinlite_large_v7_20261017_r1`은8epoch 전체 val 평가·저장 후 정상 종료했다.
`stop-policy.json`의 상태는 `stopped_at_target`, 목표 이후 optimizer update는0이다.
원래40epoch polynomial LR 일정과24시간 절대 상한을 바꾸지 않았다.

| 완료 epoch | 6조건 균형 두 head IoU | drivable | 노면 표시 |
|---|---:|---:|---:|
| 1 | 86.9432% | 98.0620% | 75.8244% |
| 2 | 90.6459% | 98.2320% | 83.0598% |
| 3 | 90.5147% | 98.2158% | 82.8137% |
| 4 | 90.3894% | 98.2127% | 82.5661% |
| 5 | 90.5339% | 98.2116% | 82.8562% |
| 6 | 90.7472% | 98.2342% | 83.2602% |
| 7 | 90.9483% | 98.2566% | 83.6401% |
| **8** | **91.0968%** | **98.2664%** | **83.9271%** |

수치는 학습 중 AMP 전체val1,404장의 결과다. SUNNY/FOGGY×11/13/15시를 같은 비중으로
계산하며, 노면 표시는 background를 제외한 흰차선·황색차선·정지선 macro IoU다.
기존 Medium의 CPU FP32 동일val 지표86.4446%와 정밀도가 다른 참고 비교를 구분한다.
이번 배포 평가에서는 두 모델 모두 같은 GPU FP32 설정으로 val/test를 다시 확인한다.
Test를 보기 전에E8 및SHA를 고정했으며 test/17시 결과로 epoch나 threshold를 재선정하지 않는다.
Test는 이번 Large 학습에서 미사용이다. 데이터셋 `split_policy.historical_usage`에 명시된
과거 Medium 개발 사용 이력은 유지하며, 새 비공개·지리적으로 독립된 시험이라고 설명하지 않는다.

동일GPU FP32 재평가에서 val 조건 균형 IoU는 Medium86.4449%→Large91.1009%,
test의 대회6조건은87.6788%→92.7560%다. Test전체14,556장의pooled와6조건 macro는 구분한다.
17시4,098장의 별도pooled 두head 평균은82.1884%→87.4420%다.
17시 집계는 전체test 정수 confusion에서11/13/15시의 합을 빼 정확히 분리했다.
전체test manifest가 오직6조건과17시로만 구성됨과SHA를 확인했고 추가 추론이나 라벨 변경은 없다.

동일GPU FP32의 전방 황색차선은 val85.0039%→83.8019%, test86.2164%→84.2801%다.
전체 개선으로 이 회귀를 숨기지 않고 같은 RGB/pose의 음영 family와 원본 사례를 확인한다.
FOGGY15 정지선은val432장에GT444픽셀로 희소하므로 그 한계를 계속 표시한다.

## 배포 파일과 변환

- 원본: `E:\asmc_data\twinlite\training_runs\twinlite_large_v7_20261017_r1\epoch008.pt`.
- 원본SHA: `4029a2892966d5f1b3e0e2991211214395935015e5062ec9e497b4ff70c142c0`.
- 패키지후보: `src/asmc_camera/models/lane_segmentation/twinlite-large-v7-e8.pt`.
- 배포SHA: `3a37e3e4708374e8b26dcea889b3f132f76bffa8586213f2c171b75504484ce6`.
- 31,775,612B 원본에서 EMA408개 tensor/buffer를 모두 보존한7,955,842B 추론본이다.
  optimizer·scheduler·중복 model 상태는 제외한다. 원본은 덮어쓰지 않는다.
- 전방544×960 B1, 좌우480×640 B1–2를 유지한다. 두head와 클래스 순서·전처리·보정·
  family 피팅·15/15/10Hz 목표·YOLO/ResNet 및 공개 계약은 유지한다.

`package_twinlite_checkpoint.py`는 미완료epoch·검증/EMA 누락·잘못된 클래스 순서를 거부한다.
실제 패키징에서408개 tensor/buffer가 dtype까지 동일함을 재로딩 검사했다.
CPU ONNX는 opset17이며 전방B1·측방B1/B2 모두 대응을 통과했다.
확률 최대 절대차는 약6.38e-6, class ID 차이는0이다.

## 검증 기록과 현재 경계

- 최초 카메라CPU26개·runtime93개·패키징3개 회귀 통과.
  최종 Medium 기본/ Large 후보 설정은 runtime94개·패키징3개·수락4개·문서17개를 통과했다.
- 별도 `/deployment/ros_ws`에서 최신 `asmc_msgs`·`asmc_camera` 빌드 통과.
- 최초 평가의 Capture source root 미지정, ONNX Runtime 미설치, 시험의 잘못된
  Python namespace 순서는 원본 로그를 남기고 수정했다. 모델·데이터 오류로 집계하지 않는다.
- 전체val/test GPU FP32 평가를 완료했다. 같은6조건 음영fixture에서 입구·내부·출구별
  연속8프레임씩144묶음을CPU 비교했다. 양쪽144/144유효, validity/role 변경0,
  support 변경142묶음이며 프레임별횡방향차p95의중앙값2.60cm·최대25.83cm다.
  전체CPU2,356묶음 재생은 장시간 예상으로 중단하고 원본 로그/일부 이미지를 보존했다.
- 사용자 MORAI 종료 확인 후 TensorRT10.13.0.35·RTX4060Ti에서 FP16 engine을 생성했다.
  전방8,937,212B·측방8,391,276B이며 manifest SHA는
  `10f880579ca23d88898f1233d8cdb5868c7f188dd2c811235a84cbdbdb90a404`다.
- Native GT1,404장에서 GPU FP32→TensorRT 비교: 확률 차이p99 최대0.005307,
  class ID 최소 일치99.9896%, 클래스/뷰별 IoU 최대 하락0.0977%p다.
  측방B1/B2 및 이전 결과 buffer 보존도 통과했다.
- 전체 음영fixture2,356묶음에서 Large FP32/TRT는 모두 유효하며 validity/role/color 변경0,
  support 변경441묶음이다. 프레임별 횡방향차p95의 최대는1.765cm다.
  양자화에 따른 support 변화를 숨기지 않으며 직접 GT 기하 정확도 시험으로 설명하지 않는다.
- 실제 ROS 새 overlay에서 저장 영상30초 재생: 세 모델15/15/10Hz,
  family449/449유효·최소5초창14.8Hz·최대공백126.8ms,
  family 입력→수신p95 86.7ms, VRAM3,871MiB다. 이 시험은 MORAI 수락과 구분한다.
- 초기 MORAI 시험 직후에는 SUNNY source freshness 실패로 Medium 기본값을 유지했다.
  후속 사용자 승인에 따른 현재 기본 전환은 아래 결정 기록을 따른다.
  기존 순간family 공백/freshness 제한도 자동 해결된 것으로 표시하지 않는다.
- 재현 checkout과 기존컨테이너 마운트는 유지한다. 오프라인 family의 시험ROS master는11358, 저장 영상/실시간은11359이며
  시험기가 생성한 master만 종료한다. 실시간에는 rqt 한 세트를 사용하고 공개 ROS 차량 제어는 격리한다.
- 실시간 시험 준비 실패는 로그에 보존했다. 키보드 제어에서 브레이크가 유지되지 않아
  정차용 AUTO/P 전환을 명시했고, 시간 API의 정수 형식과 Main의 LiDAR 설정 참조를 수정했다.
  각 실패 후 원래 pose·gear·날씨·시간·제어 모드를 복원했다. 모델 성능 실패와 구분한다.

로컬 결과: `artifacts/camera-seg-large-deploy-20261018-r1/`.
[실험 페이지](http://127.0.0.1:8000/camera-seg-large-deploy-20261018-r1/index.html),
[전환 절차](../../../src/asmc_camera/docs/large-deployment-plan.md).


## MORAI 정차 통합 결과와 승격 보류

SUNNY/FOGGY13시에서 같은 터널 위치로 Medium/Large 각3회·60초를 교차 실행하고,
Large FOGGY·전체 GUI를600초 실행했다. Main/LiDAR 프로세스 부하를 포함했다.
터널의 GPS 무효로 Main의 일부 pose/fusion 처리가 차단되는 로그도 있으므로
완전한 fusion 처리율이나 공동 자율주행 수락으로 설명하지 않는다.

| 조건 | 결과 |
|---|---|
| Medium 교차6회 | 신규 수치·상태 수락 검사 통과 |
| Large 교차6회 | 공급·모델 지연은 통과. SUNNY1회의 정상 ACTIVE에서 readiness false3건 |
| Large FOGGY600초 | 유효 family8,957/8,957·14.928Hz·최소5초14.2Hz, readiness 저하0 |
| 600초 모델 지연 | TwinLite model-grid p95최대128.72ms·p99최대139.30ms |
| 600초 family | 발행 최대공백188.4ms·수신189.1ms, 입력→수신p50/p95/p99=131.8/182.2/201.1ms |
| 600초 자원 | 전체VRAM5,014MiB(61.24%), camera RSS최대3,850MiB |

Family 계산p95는65.03ms로 기존 내부 목표60ms에는 못 미친다. 모델 지연과 family 전체
지연은 구분한다. 모든 조건에서15Hz 목표를 만족했다고 표시하지 않는다.

SUNNY 첫 Large 실행의7.05/7.27/7.67초에 `pipeline_ready=false`가 관측됐다.
`family_valid=true`, `prepared=true`, 모델 오류0이지만 `family_source_age_ms`가
357.96/399.11/357.74ms로350ms를 초과한 `source_stale`이다. Family 발행 공백은145ms
이내여서 단순한 출력 중단과 다르다. 이3초 구간에 모델·family·시험 소비자 GC 사건은 없다.
연산·입력/결과 전달의 누적 지연은 확인됐지만 세부 CPU/GPU/스케줄링 인과는 미확정이다.
Freshness를 늘리거나 오래된 결과를 재발행하지 않는다.

초기 브리핑은 원래 비교 도구의 `failures=[]`를 전체 수락으로 잘못 해석했다.
이는 실행 오류 목록이며 readiness·주기·지연의 최종 수락 목록이 아니었다.
`assess_twinlite_deployment.py`를 추가해 원시 상태의 정상 ACTIVE 창과 모든 수치 기준을
다시 검사했다. `live-acceptance.json`의 최종 결과는12회 중11회 통과+SUNNY1회 실패,
600초는 통과다. 원본 보고서와 실패 이미지는 유지하고 HTML에는 새 판정을 사용한다.
실행 성공/성능 실패, 준비 구간 제외, readiness 누락을 CPU4개 회귀로 고정했다.

기본 연결 확인용10초 저장 영상 재생2회에서는 checkpoint/engine/모드·클래스는 정상이나,
복원된 CLOUDY 환경에서 측방 포함 모델 공급최소9.2Hz·p95약163–169ms가 관측됐다.
첫 실행은 CPU 회귀와 겹쳤고 두 번째도 낮은 공급이 지속됐다. 별도 조건의 성능 실패로
보존하며 터널 FOGGY600초 통과로 숨기지 않는다. 원인을 특정 모델/동시 CPU 검사로 확정하지 않는다.

**초기 검증 판정에서는 Medium을 유지했다. 아래 후속 사용자 결정으로 현재 기본은 Large E8이다.**
원래 pose(위치 차이4.85mm)·gear·CLOUDY13시·키보드 제어 모드를 복원했고 시험 ROS·rqt를 정리했다.
신규 학습·수집·YOLO 변경·PR 병합은 하지 않았다. Hosted CI는 이번 로컬 변경을 게시하지 않아 미실행이다.

## 직접 실행

`asmc-ros-noetic`의 각 터미널에서 현재 검증 overlay를 source한다. 기존 카메라 launch를
종료한 뒤 둘 중 하나만 실행한다. 원래 컨테이너 마운트·재현 checkout은 바꾸지 않았다.

```bash
source /opt/ros/noetic/setup.bash
source /data/runtime/camera-twinlite-large-v7-e8-r1/devel/setup.bash --extend
# Large v7 E8 기본: 세 모델 ACTIVE
roslaunch asmc_camera camera.launch
# 위 실행을 종료한 뒤 기존 Medium으로 복구할 때만
roslaunch asmc_camera _runtime.launch mode:=active lane_runtime_config:=$(rospack find asmc_camera)/config/camera/lane_runtime_medium.yaml
```

별도 터미널에서 같은 환경을 source한 뒤:

```bash
roslaunch asmc_camera view.launch models:=lane
# 또 다른 터미널: rqt는 한 개만 유지
rosrun rqt_image_view rqt_image_view --clear-config /perception/lane/debug/shared_family_bev/compressed
```

다음 단계는 새 Large 출력을 기준으로 family의 투영 grid 범위·support·보간/피팅 품질을 검토하는 것이다.
SUNNY source age·복원 환경 공급 저하는 알려진 후속 성능 과제로 유지한다.
역할·전처리·피팅·freshness350ms·15/15/10Hz 조건은 이번 기본 전환에서 바꾸지 않는다.


## 2026-10-18 후속 사용자 결정 — 기본 전환

사용자가 “이정도면 일단 그냥 기본으로 전환해줘. 그리고 다음단계로 넘어가자”라고 지시했다.
이에 따라 `lane_runtime.yaml`은 Large v7 E8 TensorRT, `lane_runtime_pytorch.yaml`은 같은
Large E8 PyTorch 복구로 변경했다. Medium v6 TensorRT는 `lane_runtime_medium.yaml`에 보존한다.
전방544×960 B1·좌우480×640 B2, 15/15/10Hz·YOLO E8/confidence0.40/NMS0.50은 유지한다.
자동 수락의 `all_pass=false`, SUNNY3건, 별도 재생 최소9.2Hz와 전방 황색차선 회귀는 수정하지 않는다.
이 결정은 기본 모델 선택 승인이다. 판제의 ODD·valid/stale 처리와 공동 자율주행 검증을 대신하지 않는다.
재현 checkout·기존 컨테이너 마운트를 유지하고 로컬 운영 overlay에 새 기본 설정을 반영한다.
현재 실행 중인 사용자 노드를 강제로 바꾸지 않으며 새 셸/노드 재시작부터 적용한다.
이번 변경에서 재학습·새로운 MORAI 부하 시험·PR 병합은 수행하지 않는다.


## Medium→Large 시각화 지연 재집계

두 모델의 SUNNY/FOGGY13시·같은 터널 위치·전체 GUI/Main/LiDAR 조건에서 각6회60초의
원시 monotonic 기록을 합산했다. 실행별 percentile을 평균하지 않았고, Large600초 시험은
비교에서 제외했다. 아래 단위는ms이며 입력 시작점은 카메라 모델의 압축 영상 callback이다.

| 경로 | Medium p50 | Large p50 | 증가 | Medium p95 | Large p95 | 증가 |
|---|---:|---:|---:|---:|---:|---:|
| 전방 모델 결과 | 48.5 | 73.8 | 25.3 | 93.0 | 124.4 | 31.5 |
| semantic 합성 JPEG 수신 | 77.7 | 101.7 | 24.0 | 122.5 | 155.4 | 32.8 |
| family 정보 수신 | 107.7 | 135.8 | 28.0 | 162.8 | 187.7 | 24.9 |
| family JPEG 수신 | 148.6 | 174.5 | 25.8 | 212.9 | 239.4 | 26.5 |

Semantic JPEG2,742/2,718장과 family JPEG각3,600장은 원본 입력stamp에 모두 대응했다.
JPEG 수신은 시험 소비자까지이며 rqt의 화면 표시·모니터 지연은 측정하지 않았다.
Semantic 합성 header는 가장 새 패널의 stamp이므로 다른 패널은 더 오래됐을 수 있다.
Family JPEG는 같은 결과의10Hz 표시와 STALE 화면을 포함하므로 신규 검출 공급률로 쓰지 않는다.
Simulator 렌더링·bridge 이전 지연도 이 수치에 포함되지 않는다.

Semantic 시각화는 목표10Hz지만 실측은 Medium7.43–7.73Hz, Large7.15–7.70Hz다.
Family 시각화는 양쪽10Hz다. Semantic의 현재 callback은 직전 렌더링 후100ms 미만을
건너뛰므로15Hz(약66.7ms 간격) 입력을 두 장 중 한 장 처리하면 약7.5Hz가 된다.
정수 연산 오류가 아니라 이벤트 기반 throttle의 주기 선택 결과이며 두 모델에 같은 코드가 적용된다.
Timer 기반 최신 입력 선택 등으로 별도 대조할 개선 후보이며 이번에 계산 방식을 변경하지 않았다.
큰 변화는 모델 결과가 준비되는 시간이며,
같은 stamp의 원본 크기 class 출력 이후 semantic JPEG 수신까지 p50은23.37→23.74ms,
p95는30.58→31.50ms다. 이 값만으로 병목의 인과를 확정하지 않으며 정확한3뷰 대응·렌더링·
JPEG·ROS 전달과 rqt 화면 지연을 분리한다. 새 실시간/GPU 시험이나 설정 변경은 수행하지 않았다.

재현 자료: `artifacts/camera-seg-large-deploy-20261018-r1/view-latency-audit.py`,
`view-latency-audit.json`. 현재 기본 Large 및 기존 수락 실패 기록은 유지한다.
