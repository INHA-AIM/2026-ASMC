# 카메라 배포 이후 개발 계획 — 2026-10-14

목표는 K-City 대회 에셋을 SUNNY/FOGGY × 11/13/15시에서 단일 모델로 강건하게 인지하는 것이다.
현재 E8은 팀 개발에 사용할 수 있지만 터널·범퍼 앞·일부 차량과 시간대에서 누락이 남아 있다.
라벨 불일치는 가능한 원인이다. RGB/GT와 소스별 실패를 비교하기 전에는 원인으로 확정하지 않는다.

## 1. 팀 개발용 기본 배포 — 이번 반영

- YOLO26l E8을 3카메라 기본으로, 공식 사전학습 모델을 실행 시 선택 가능한 대조군으로 제공한다.
- 두 모델 모두 같은 세 그룹과 원본 이름·confidence·source stamp를 전달한다. 사전학습은 7종만 지원한다.
- 판제와 LiDAR는 [소비 계약](../../operations/camera-deployment.md)을 따른다. 숫자 타입이 unknown인
  장애물을 버리지 않고, 이동 가능 그룹과 실측 운동을 구분한다. 전방 fusion과 3뷰 출력의 범위를 구분한다.
- Release에 pt/engine과 SHA를 고정한다. 모델 변경은 재시작으로 수행하며 두 producer를 동시에 실행하지 않는다.
- 기존 데이터·수집 PR #81/#82는 자동 병합하지 않는다. 이번 배포가 전체 조건의 최종 성능 수락은 아니다.

## 2. 연산 경로 정리와 전체 스택 수락

현재 3뷰 YOLO 15Hz·TwinLite 10Hz·ResNet 10Hz를 먼저 측정한 뒤 TwinLite 15Hz를 대조한다.
공급률 숫자를 통일하려고 timeout·freshness 기준을 완화하지 않는다. 신호등은 별도 camera4와
필요 공급률을 유지하며 GPU 경합·deadline 악화를 함께 확인한다.

1. 입력 수신 → JPEG decode → resize/mask → H2D → 추론 → 후처리 → 발행의 p50/p95/p99와
   프레임 대체·입력 공백을 모델/뷰별로 분리한다. GUI/debug OFF와 ON 비용도 따로 측정한다.
2. 같은 source frame의 중복 decode, 중간 JPEG 재인코딩, ndarray/tensor 복사,
   CUDA 동기화와 CPU NMS 시간을 확인한다. 측정한 병목부터 수정한다.
3. decode 공유·tensor packing·pinned memory·비동기 복사를 적용한다면 원본 header/좌표·세대
   경계·최신 프레임·최대 10ms 대기·중단 뷰 격리를 유지한다. 같은 영상으로 변경 전후를 대조한다.
4. 전방 1280을 우선 유지한다. 부하가 문제인 후보에서만 960을 비교하며 원거리·터널·근거리
   품질 손실도 확인한다. 신호등 인식을 희생해 YOLO 처리율만 높이지 않는다.
5. MORAI 렌더링 + 세 모델 + LiDAR/fusion + 판제 preview에서 반복하고 30분 연속 확인한다.
   목표 15Hz와 최소 9.5Hz를 구분한다. p95≤150ms, p99≤200ms, 공백≤250ms, VRAM≤90%와
   다른 모델 공급·판제 deadline 기준을 함께 확인한다.

리팩토링은 데이터/가중치 변경과 분리해 원인을 추적한다. TwinLite BEV 자세 보정·3뷰 LiDAR
fusion·mask fusion은 필수 배포 마무리와 분리된 후속 과제다.

## 3. 현재 문서 정리와 LiDAR 이전 인계

현재 source와 실제 launch를 기준으로 카메라와 nested Main/`asmc_lidar` 상태를 대조한다.
현재 `asmc_lidar`는 좌표·시간 코어이고 검출/추적 실행은 nested Main이다. 기능 이전을 임의로 완료 처리하지 않는다.

- 시작점: `docs/handoffs/README.md`, 현재 배포 계약, 카메라 README, LiDAR README.
- 갱신 대상: `docs/architecture/{system-overview,repository-layout,consolidation-roadmap}.md`,
  `docs/packages/asmc_lidar/`, `docs/contracts/perception/`, 기존 카메라 객체/모드/3뷰 인계.
- YOLO11m/COCO7·4그룹·단일 전방·0.7 cutoff를 현재 값처럼 쓴 문장을 수정한다. 실험 원본은
  날짜와 당시 설정을 붙여 보존한다. 실행 안내·계약·과거 증거를 구분하고 중복 본문은 정본 링크로 축소한다.
- 구 문서는 내용과 incoming link를 대조한 뒤 archive 또는 짧은 redirect로 정리한다.
  증거·실패 기록을 삭제해 수락 상태를 바꾸지 않는다. 문서 migration·링크·메시지 계약 검사를 수행한다.

이번에는 배포에 필요한 시작 문서와 소비 계약을 갱신한다. 전체 문서 정리와 LiDAR 기능 이전은 후속 작업이다.

## 4. E드라이브 Capture 수집과 E8 후속 학습

새 원본은 **`E:\asmc_data` (`/mnt/e/asmc_data`)**에 둔다. WSL ext4가 C드라이브 VHD 안에 있다면
WSL에 저장해도 C 공간이 줄어든다. 실제 Capture의 Windows 저장 경로, 동기화 복사·임시 파일·로그·
학습 cache가 C로 돌아오지 않는지 짧은 파일럿으로 검증한다. 기존 원본과 symlink 데이터 버전은
일괄 이동하지 않는다. 수집/학습 컨테이너에는 E를 별도로 mount한다.

```text
E:\asmc_data/
  capture_raw/<run_id>/             # RGB·Semantic·Instance·BBox 원본
  capture_runs/<run_id>/            # 조건·actor 장부·높이/중단 기록
  annotations/<dataset_version>/    # 라벨·positive/ignore·complete/partial
  dataset_versions/<version>/      # 고정16종 export·manifest·split
  models/<training_run>/           # checkpoint·optimizer·평가
```

### 수집 계획과 배치

1. gRPC 실제 목록을 대회 차량/사람/장애물 목록과 대조하고 에셋별 하위 유형 coverage 표를 고정한다.
   표시 이름을 API 이름으로 추정하지 않는다. DEFAULT/Random은 실제 생성 모델명을 기록한다.
2. 차량 목록의 모든 차종과 장애물 목록의 차량도 포함한다. 사람은 Man1/2 등 번호·복장·어린이·
   자전거·유모차 복합 에셋을 포함한다. 표지판·나무·가로등·홀커버·불·목표 외 구조물은 제외한다.
   각 에셋을 16종 학습 클래스에 연결하되 복합 에셋은 보이는 인스턴스별로 처리한다.
3. 현재 train의 클래스별 **유효 양성 등장 이미지 × 독립 배치 × 6조건 × 거리/뷰**를 집계해
   부족한 조합에 더 배분한다. 같은 물체의 연속 사진 수로 균형을 판단하지 않고 차종/하위 유형도 보장한다.
4. 전역경로 주변에는 객체 전체와 여유폭이 주행 영역 밖에 있도록 배치한다. 각 지점의 지면 높이·경사·
   에셋 pivot을 확인해 떠 있음/관통을 제외한다. 터널 객체를 벽 바깥에 두지 않는다.
5. SUNNY/FOGGY × 11/13/15시마다 위치·자세를 바꾸고 장부를 저장한다. 회전은 60도 변형을
   기본으로 하되 벽·차선 침범·형상 겹침을 재검사한다. 차량/사람 에셋의 지원 회전도 확인한다.
6. 일부 차량은 주변 링크 또는 Ego 앞에서 주행하도록 해 후면·추종 관측을 확보한다. 팀 MPC/PP 추종과
   NPC 운동 지원 여부를 확인하며, 주행 중 차량 앞으로 actor를 갑자기 재배치하지 않는다.

### 정차 근거리·터널 보강은 별도 세트

전역경로 옆 사진만으로 정면·범퍼 앞 관측을 충분히 채웠다고 보지 않는다. 정/동적 미션 교차로 앞과
터널 내 정차 지점에서 모든 종류를 전방에 번갈아 배치한다. 카메라에서 실제로 보이는 최소 거리부터
약 1/2/3/5/10m를 파일럿으로 확인하고 객체 크기에 따라 조정한다. 화면 잘림·측면 진입·큰 객체·
근접 오버랩을 포함한다. Ego 차체 안이나 충돌 영역에 배치하지 않고 mask와 실제 가시 영역을 구분한다.

서로 가리는 장면은 필요하지만 물리 점유 영역을 관통시키지는 않는다. 단독·다중·부분 가림을 함께
확보하고 완전 가림 GT를 양성으로 만들지 않는다. 터널 입구/내부/출구를 각각 6조건에서 촬영한다.
Capture 1Hz·동시 요청 1건·GT 파일 완전성 검사를 유지한다. RGB 비정상·파일 누락·actor ID 불일치가
발생한 묶음은 보존하고 격리한다. 정지/재개·20–30km/h 추종과 단일 제어 생산자를 먼저 확인한다.
기존 actor·날씨 상태를 보존하고 시험 actor만 정리한다.

### 일관된 라벨·배경·split

- Capture instance/native bbox와 actor 장부로 세부 종류를 확정한다. 원본 출력·에셋 하위 유형·
  목표 16종을 별도로 보존한다. semantic의 포괄 Obstacle만으로 세부 종류를 결정하지 않는다.
- 현재 4px/64px² 양성 선별 정책과 원본/학습 tensor 좌표 변환을 먼저 재검증하고 train/val/test에
  같은 규칙을 적용한다. 극소·완전 가림·애매한 종류를 자동 negative로 바꾸지 않는다.
- 가시 instance로 부분 가림과 Ego mask 후 박스를 확인한다. 유모차 안 아기는 제외하고,
  미는 사람/탑승자와 탈것의 복합 박스 기준을 통일한다.
- 확실한 Capture 배경만 complete/reviewed_negative로 사용하며 맵 고정 traffic_drum 등도 확인한다.
  제외 클래스가 있다는 이유만으로 같은 이미지의 다른 목표 객체를 배경으로 만들지 않는다.
- Roboflow는 목표 클래스에 직접 대응하고 양성/누락 기준이 맞는 자료를 유지한다. 불일치/부분 주석은
  별도 표시하고 E8 실패 조건에서 영향을 확인한다. COCO 자료는 추가 다운로드하지 않는다.
- 새 배치/연속 세션/날씨 변형은 같은 split 그룹이다. 기존 E8 train 자료를 새 heldout으로 옮기지 않는다.
  새 독립 배치 일부를 수집 전에 selection/test로 예약해 16종·6조건·근거리·터널을 고르게 검증한다.

### E8 후속 학습과 교체

E8을 초기화 가중치로 사용한다. 데이터가 바뀌므로 기존 optimizer/scheduler를 그대로 resume하는 것과
새 fine-tuning 단계를 구분한다. 권장은 **E8 가중치 + 새 학습 단계**이며 LR/warmup·loss·ignore 전달을
먼저 확인한다. 검증 없이 기존 LR 0.001을 반복하지 않는다.

전체 유효 데이터를 활용하되 클래스별 등장 이미지·노출 수·하위 유형을 집계하고 반복 상한을 유지한다.
E8과 새 모델을 먼저 confidence 0.40·NMS 0.70에서 비교한다. 16종과 3그룹 F1/recall, 버스·트럭/
사람·이륜차 회귀, 중복 FP, 최초 안정 검출·공백을 중간 평가한다. 소표본/partial GT를 완전한 정밀도
평가로 설명하지 않는다. 개선 체크포인트만 confidence를 selection에서 고른 뒤 고정 heldout과
3뷰 TensorRT 전체 스택으로 확인한다. 충분한 성능이면 epoch 상한 전에 배포 검증으로 넘어가며
기존 Release는 복구용으로 보존한다.
