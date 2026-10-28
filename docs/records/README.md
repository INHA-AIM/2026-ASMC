# 검증·실험 기록

기록은 당시 데이터·commit·모델·설정·환경에서 관측한 결과입니다.
본문의 현재/정본/다음 단계도 기록 시점의 표현이며 오늘의 실행은 [문서 허브](../README.md)를 따릅니다.
새 측정은 새 기록으로 추가하고 이전 실패/실험 SHA를 고쳐 쓰지 않습니다.

| 분야 | 최근 결정·근거 | 보존 목록 |
|---|---|---|
| 카메라 엔진 | [로컬 GPU 선택·D/WSL 등록·실행 안내 검증](camera/2026-10-29-local-engine-selection.md), [E3 배포 근거](camera/2026-10-28-mission15-e3-deployment.md) | [camera](camera/) |
| 카메라 | [두 시각화의 IMU 재사용](camera/2026-10-18-shared-imu-bev.md), [IMU·공동 폭 계산](camera/2026-10-18-joint-family-stabilization.md), [상시 추론 기본값](camera/2026-10-16-continuous-inference.md), [family 공급·사용자 확인](camera/camera-family-supply-optimization.md), [TwinLite TRT](camera/camera-twinlite-tensorrt.md), [객체 배포 비교](../../config/baselines/evidence/camera_mission16_deployment_v1/comparison.json) | [camera](camera/) |
| LiDAR | [10월 27일 연구 선별 통합](lidar/2026-10-27-research-integration.md), [F1 전방 연결 검증](lidar/2026-10-27-front-fusion-verification.md), [10월 26일 실제 주행·재밍 탈출 bag](lidar/2026-10-26-full-drive-bag-validation.md), [전환·재밍 자동 검증](lidar/2026-10-26-cutover-validation.md), [좌표/시간 경계](lidar/lidar-camera-frame-boundaries.md), [연속 회귀](lidar/lidar-continuous-regression.md) | [lidar](lidar/) |
| Bridge | [콘솔 줄바꿈·센서별 Hz](bridge/2026-10-17-bridge-console-layout.md), [로그·선택 형식 구분](bridge/2026-10-17-bridge-log-readability.md), [LiDAR 회전율·진단 정리](bridge/2026-10-17-lidar-cadence-diagnostics.md), [P3-G local closeout](bridge/bridge-p3g-local-closeout-v1.md) | [bridge](bridge/) |
| HD Map | [compact 제품](hd-map/kcity2025-compact-drivable-v1.md), [입력 감사](hd-map/kcity2025-drivable-input-v1.md) | [hd-map](hd-map/) |
| 저장소 | [구조 이전](repository/documentation-restructure-v2.md), [현행화·소스 검토](repository/2026-10-16-documentation-refresh.md) | [repository](repository/) |
| 종료된 설계 | [과거 실행/연구 목록](archive/README.md) | [archive](archive/) |

## 긴 현행 문서에서 분리한 과거 근거

- [2026-10-17 Large 학습 준비 조사](camera/2026-10-17-twinlite-large-training-preparation.md)
- [Roboflow 확보·Teacher·v7 채택](camera/data-preparation-through-v7.md)
- [v8~v11 카메라 학습 후속 도구 통합](camera/2026-10-19-camera-training-followup.md)
- [인지 융합/BEV 후보와 단계별 실험](camera/perception-roadmap-through-pr85.md)
- [통합·필드 배포의 이전 단계별 계획](repository/consolidation-roadmap-through-pr87.md)
- [v6 segmentation 정책·팔레트·학습 결과](camera/segmentation-dataset-policy-v6.md)
- [객체 Q&A·이전 네 그룹 후보](camera/object-label-proposals-through-pr85.md)
- [v3 음영 인수 실험/과거 명령](camera/shadow-handoff-v3.md)
- [TwinLite 이전 학습 명령](camera/twinlite-training-through-v6.md)

이 보존본은 모델/config의 구버전 선택지와 다릅니다. 당시 명령을 현재 checkout에 그대로 실행하지 않습니다.
동결5개 snapshot과 원래 artifact/hash는 유지합니다. 일반 문서 본문을 이사 당시 SHA에 영구 고정하지 않습니다.
이전 검사 `--verify-relocation`은 #88 시점의 일회성 증명이며 이후 정상적인 내용 현행화에는 실행하지 않습니다.
