# Family와 comparison BEV의 IMU 보정 재사용

## 적용 내용

[기본 IMU·공동 폭 피팅](2026-10-18-joint-family-stabilization.md)과 같은 보정을
comparison RGB/semantic BEV에서도 사용한다. IMU 자세 조회·M1/M0 선택은 family worker에서
한 번이다. 선택 직후, 피팅을 기다리지 않고 작은 `CameraGroundProjection` metadata를 전달한다.
Family 선 화면은 실제 발행된 family를 그린다. Comparison은 그 프레임의 원 RGB와 원본 크기
semantic을 뷰별 같은 raster geometry·coverage·quality로 투영한다. 추론·피팅을 다시 하지 않으며
이미 보정된 BEV에 보정을 중복 적용하지 않는다.

명목 지면 법선 거리는0.36m다. 모델·engine·입력 해상도·Joint 피팅·M0 prior0.35와 기존 공개
메시지 MD5는 그대로다. 추가 metadata는 [내부 계약](../../../config/contracts/camera_ground_projection_v1.yaml)에
등록했다. 원 JPEG는 frame/stamp/wire sequence, native head는 현재 추론 epoch의 유일한 frame/stamp로
연결한다. Native head sequence를 JPEG sequence로 간주하지 않는다. 모호한 중복 stamp나 누락 입력은 건너뛴다.

기본 comparison만 구독할 때 고정 BEV 생성·합성 체인의 입력은 해제한다. Recorder·legacy 소비자가
명시적으로 요청하면 기존 데이터 토픽은 계속 동작한다. 구형 BEV 데이터·판제 계약을 IMU 보정으로
몰래 바꾸지 않는다. 결과 녹화 프로필에 작은 projection metadata를 optional로 추가했다.

Comparison은 도착 알림 기반 최신 렌더 worker·최대10Hz, 뷰별8개/500ms 캐시·대기1개다.
JPEG는 마지막에 한 번만 인코딩한다. 무구독 시 원 영상과 metadata 구독·변환을 중단한다.
세대 전환 시 캐시를 비우고, 이미 실행 중인 native callback과 렌더 결과도 이전 epoch이면 버린다.
작은 warp/행렬 작업은 해당 시각화 프로세스만 OpenCV/BLAS thread1로 실행한다.

## 검증

Noetic CPU에서 BEV210개·runtime119개가 통과했다. Suite는 일부 중복된다.
M0 출력의 기존 static 경로와의 정확한 일치, 보정 geometry를 RGB/semantic이 뷰별 한 번 공유함,
원본 영상 불변성, 정확한 JPEG 식별·native 중복 거부·상한/만료·재시작과 계산 중 epoch 변경을 검사했다.
기존 메시지는 변경하지 않고 additive 타입만 등록했으며 공용38종 계약 검사를 통과했다.
[CPU 검사 원본](../../../config/baselines/evidence/camera_shared_bev_v1/cpu-regression.json)을 보존한다.

차량 위치·날씨·시간·제어를 변경하지 않은 읽기 전용 실시간 참고 측정이다. 경로/자세를 통제하지
않았으므로 서로 같은 장면의 품질 대조나 10분 전체 GUI 재수락으로 설명하지 않는다.
rqt 창을 새로 만들지 않고 실제 객체·semantic·family·comparison 토픽 구독으로 렌더링을 실행했다.

| 구성 | 길이 | Comparison 평균/최소5초 Hz | Header→표시 p95 | 최대 공백 | BEV 체인 CPU |
|---|---:|---:|---:|---:|---:|
| 기존 고정 지면 | 60초 | 9.99/10.0 | 216.5ms | 121.8ms | 76.06% |
| 공유 첫 구현·피팅 후 metadata | 60초 | 9.98/9.8 | 241.5ms | 128.6ms | PID 수집 불완전 |
| 피팅 전 metadata | 60초 | 9.97/9.8 | 238.2ms | 168.2ms | 45.90% |
| 최종 epoch guard 연결 확인 | 20초 | 9.96/9.8 | 218.3ms | 128.9ms | 43.90% |

CPU는 core100% 기준 `camera_bev`·`camera_bev_fusion`·`camera_bev_visualizer`의 합이다.
서로 다른 참고 구간이므로 정확한 동일 pose의 인과 개선율로 해석하지 않는다.
Header는 bridge 정책을 유지하므로 위 수치는 카메라 노출→화면의 광자 표시 지연이 아니다.
최종60초에서도 보정 raster 비용이 남았음을 기록하며, 20초 최솟값만으로 지연 개선을 주장하지 않는다.

앞선 구간은 기존 실행부터 family 무효가 많았고, 보정 공유60초에는843개 중780개가 무효였다.
이 구간을 유효 family·readiness 통과로 사용하지 않는다. 마지막20초는300/300 유효 family,
평균14.94Hz·최소5초14.8Hz·최대 공백141.8ms, 세 모델 readiness 저하0·오류0이었다.
Comparison과 family 선의 중복/역행 stamp는 모든 측정에서0이었다.
보정 metadata는 최종 확인에서 family보다 중앙값41.4ms 먼저 도착해 피팅 완료를 기다리지 않는다.
[4개 실행 원본](../../../config/baselines/evidence/camera_shared_bev_v1/runtime.json)에 한계와 집계를 함께 보존한다.

## 설치·실행 상태

기존 운영 runtime `camera-twinlite-large-v7-e8-r1`에 파일별 별도 백업·SHA 장부를 만든 뒤 반영하고
`asmc_msgs;asmc_camera`를 재빌드했다. 카메라 노드는 재시작했으며 마지막 영상 guard는 시각화만
재시작해 확인했다. 기존 모델 파일·bridge·사용자 제어 프로세스는 변경하지 않았다.
원본·중간·최종 실행은 `data/runtime/camera-bev-shared-20261018-r1/`에 보존했다.

시험 view를 종료한 뒤 세 모델 ACTIVE·loaded/warmed/pipeline_ready·오류0,
family config SHA `2d17cf63adbf2ba049d70656233ee2f34e9a4bfb43373ee76b3371aab1e4dc2a`와
단일 생산자를 확인했다. ROS 등록11개 모두 응답하고 죽은 등록·rqt는0이다.
Metadata·native semantic·고정 BEV에 영상 소비 구독자가 없음을 확인했다.
[최종 상태](../../../config/baselines/evidence/camera_shared_bev_v1/final-health.json)를 따른다.

기존 `view.launch`를 실행하고 rqt 한 창에서
`/perception/bev/debug/comparison/compressed` 또는
`/perception/lane/debug/shared_family_bev/compressed`를 선택한다. `IMU shared`/`M0: 사유`가
실제 선택을 표시한다. 실험 목록 **E398**, `camera-shared-imu-bev-20261018-r1/index.html`에
실제 수신 이미지·원본 측정·CPU 표를 연결했다. 예시 이미지는 같은 장면의 전후 비교가 아니다.

차량 이동/yaw에 따른 Ego 기준 BEV 이동은 정상이다. 병진/yaw 시간 정렬·시간 누적·3뷰 중첩 근거
정합은 후속 과제로 유지한다. 이번 표시 변경으로 그 과제나 공동 자율주행 수락을 완료했다고 표시하지 않는다.

## 병합 전 계약 검사

CI와 같은 공용 메시지·인지 계약·topic policy·수락 조건·실행 문서·이전 검사54개를 추가 확인했다.
초기 검사에서 새 metadata 반영 전의 메시지37개/토픽 family33개·84토픽 기대값과 이전 표시 설정
SHA가 남아 있었다. 메시지38개/family34개·85토픽으로 등록 수를 맞추고, 새 메시지의 MD5/필드도 검사한다.
표시 설정 SHA는 검증한 현재 파일로 갱신하되 이전 SHA·개정 사유·이력과 변경 검출 guard를 유지했다.
수정 후54개가 모두 통과했다. 이번 정리는 추론·피팅 실행 코드나 운영 파라미터를 바꾸지 않는다.

사용자 직접 주행 확인에 따라 M2 이동/yaw 정렬·과거 근거 누적은 보류한다.
[현행 보정 설계](../../architecture/camera-imu-projection.md)에 재개 조건을 남겼으며 PR 통합의 필수 조건으로 두지 않는다.

## 판제 별도 인수 경계

사용자 요청에 따라 [판제 인계](../../handoffs/camera-to-planning.md#71-판제-담당의-별도-인수-작업)에
현행 IMU family와 초기 SHA·고정 평면 BBox 소비 경로의 차이, xyz/영상 시각 재투영,
classifier·follower의 동일 설정 승인, read-only/preview 이후 폐루프 검증 항목을 명시했다.
해당 C++ hash gate·고정 평면 투영과 세 launch에도 주석을 남겼다. 카메라 기본값과 기존 판제
승인 SHA·보정값·제어 설정은 변경하지 않았다. 시각화용 평면 metadata를 판제 정본으로 승격하지 않는다.
54개 계약·문서 시험과 문서 검사, planning launch/input 계약이 통과했고,
주석 변경 전후 세 launch XML tree와 두 C++ 파일의 비주석 token이 같음을 확인했다.

Main 대상 hosted CI의 첫 실행은 catkin 빌드·계약 검사를 통과한 뒤 facade 소유권 검사에서
BEV 모듈 수의 이전 기대값20과 실제23이 달라 실패했다. 추가한 IMU 투영·프레임별 피팅 정책·공유
comparison 기하 모듈을 포함하도록 기대 목록을 갱신했다. 소유권·금지 의존성 검사는 유지한다.
격리된 Noetic 복사본에서 CI CPU 회귀58개 명령을 확인했다. 최초 복사본의 생성 ROS 타입·rospack
경로 문제로 실패한 네 suite는 경로를 맞춰 재확인했으며 검출89개·영상37개·BEV210개·runtime119개가
통과했다. 나머지54개 명령도 통과했다. 운영 노드·GPU·학습 가중치는 변경하지 않았다.
