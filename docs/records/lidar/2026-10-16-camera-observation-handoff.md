# 공용 추론 제어와 LiDAR 카메라 입력 지원 — 2026-10-16

기준 main은 `ff7ce1e7667e1e7d81b3334620d8b471c9b29c14`다. 이번 작업은
공용 제어 전환과 수신·버퍼·투영 라이브러리이며, 운영 Main의 전방 fusion을 유지한다.
3뷰 객체 매칭·fusion·추적 이전과 공동 주행 수락은 손재호의 후속 작업이다.

## 구현 경계

- 정본 상태/서비스: `asmc_msgs/InferenceStatus`, `asmc_msgs/SetInferenceMode`.
- 정본 상태의 UUID는 관측과 일치한다. 구형 camera 타입과 노드 루트는 같은 내부 상태를 변환한다.
- Main은 공용 상태를 구독하고 카메라 빌드 의존성을 제거했다. 재시작 UUID 변경에도 입력 epoch를 무효화한다.
- `asmc_lidar_camera_core`: 뷰별8관측·350ms, ROS/monotonic 시간 분리, 세션·세대·보정·모델/설정 검사.
- `asmc_lidar_camera_ros`: 관측/상태 구독, `camera_observation_probe`: 진단만 발행.
- 투영은 원본 영상 크기와 카메라 정본 보정을 사용한다. 시간 보정 근거가 없으면 미보정 시간차를 표시한다.
- 군집·OBB·관측 소비·분류 유지·최종 추적은 구현하지 않았다. `/YOLO/BBox`, `/last_lidar` 생산자를 추가하지 않았다.

## 로컬 검증

운영 마운트와 분리한 `/tmp` workspace에서 실행했다. 모델·GPU·MORAI 조건은 변경하지 않았다.

| 검사 | 결과 |
|---|---|
| 공용 메시지 + LiDAR만 독립 build/install | 통과, CTest9개 suite 통과 |
| 전체 저장소 문맥의 LiDAR 시험 | CTest8개 suite 통과, 실제 보정3뷰 해시 일치 |
| 설치된 라이브러리의 별도 C++ 소비자 | 컴파일·링크·보정 로딩 통과 |
| nested Main build 및 회귀 | catkin 집계324개·오류/실패0 |
| 카메라 runtime / detection | 83 / 89개 통과 |
| 공용 메시지 직렬화·MD5 | 2개 통과 |
| 계약·문서 검사 단위 시험 | 52개 통과 |
| 독립 ROS 제어 / 시각화 / 모드 예제 | 16 / 13 / 9항목 통과 |
| 실제 전방 어댑터: baseline / expanded / seg fixture | 9 / 10 / 10항목 통과 |

독립 checkout은 카메라 원본 파일과 중앙 계약을 포함하지 않으므로 해당 교차 검사를 skip한다.
전체 문맥의 LiDAR 시험에서는 같은 항목을 skip 없이 확인했다. ROS fixture는 재현 가능한
CPU 예측으로 전송·모드·좌표·빈 결과·무효 결과를 검사하며 모델 검출 품질을 측정한 것이 아니다.

기존 상태 MD5 `e106e808a882b2b0140346746e164145`는 유지된다.
새 상태는 `8b2caf96bbf9a9eaea848fcb9551b79f`, 공용/구형 서비스 MD5는
`94a789aab65be7cc8d09e8f777708430`이다. 타입명이 다르므로 정본 소비자는 재빌드해야 한다.

## 시험 환경 정정과 증거

초기 임시 복사본에 MORAI submodule 및 기존 launch/script fixture가 누락됐다.
고정 원본을 보충한 뒤 영향받은 빌드·시험을 다시 실행했다. 동일 ROS stamp로 발행된 상태를
잘못 교차 대응하던 시험도 수정하고, 매 발행별 구형/공용 필드 일치 시험을 추가했다.
이 정정으로 생산 코드의 상태 복사나 fusion 알고리즘을 변경하지 않았다.

[요약·소스 SHA·로그 SHA](../../../config/baselines/evidence/lidar_camera_input_v1/summary.json)를 Git에 남긴다.
원본 로그는 `data/runtime/lidar-camera-input-v1/logs.tgz`에 보존한다. 초기 실패 로그도 포함한다.
Hosted CI는 제출 PR의 실제 head 결과로 확인하며 로컬 통과와 구분한다.
이번 시험이 생성한 ROS master/시험 노드는 종료했고 기존11311 master는 유지했다. rqt는 실행하지 않았다.

## 다음 연결

[최소 API 예제와 직접 명령](../../../src/asmc_lidar/docs/camera-input.md),
[카메라→LiDAR 인계](../../handoffs/camera-to-lidar.md)를 시작점으로 사용한다.
손재호는 scan 처리 루프에서 조회한 후보를 검증한 뒤 객체 매칭·분류/추적에 연결한다.
카메라 미검출은 LiDAR 객체 삭제의 근거가 아니며 소비 그룹은 실제 운동 상태가 아니다.
