# v8~v11 카메라 학습 후속 도구 통합 — 2026-10-19

보존 브랜치 `work/camera-training-followup`의 v8~v11 전처리·학습·평가 도구를
최신 main에 통합한다. v7까지의 [자료 확보 기록](data-preparation-through-v7.md)과
[당시 통합 경계](camera-data-integration-prep.md)는 과거 근거로 유지한다.
원래 WIP의 상세 기록은 [보존 revision의 문서](https://github.com/INHAautonav/2026-ASMC/blob/f4db7c50826407c06a58c542be493383193517b8/docs/handoffs/perception/camera-training-data-audit.md)에 남아 있다.

## 포함한 도구와 근거

- `camera_pretrain_policy.py`, `camera_pretrain_torch.py`: 고정 16종, 작은 GT·ignore·부분 라벨,
  공간별 배경 supervision, 원본 SHA와 동결 데이터 계약 검사.
- `prepare_camera_pretrain*.py`, `prepare_camera_spatial_background.py`,
  `prepare_camera_capture_restoration.py`, `prepare_camera_preflight_revision.py`:
  v8~v11 전처리·차량 라벨 복원·검수 반영과 평가본 격리.
- `train_camera_bbox_pilot.py`, `run_camera_bbox_main.py`: ignore-aware loader/loss,
  전체 epoch 경계 checkpoint·optimizer/EMA/scheduler 재개 확인, 예산·회귀 진단.
- 출처/작은 GT/배경 감사, 지도 드럼 검수 export, 고정 16종 평가·보고·체크포인트 비교,
  live 학습 화면과 당시 `config/baselines/evidence/` 요약.
- `config/datasets/camera_pretrain_policy_v1.json`~`v3.json`과 v8~v11 실험 설정.

`evaluate_camera_bbox_pilot.py`는 현재 패키지의 16종 정의를 유지하면서 동결 pretrain
입력 경로 복원·Ego mask SHA 검사를 연결한다. `--selection-only`는 중간 checkpoint에서
heldout·temporal 입력을 읽지 않는다. 최종 평가는 기존 selection/heldout 구분을 유지한다.
오프라인 평가의 추론 지연이나 당시 비교값을 현재 통합 실행의 성능으로 해석하지 않는다.

## 충돌 해결과 실행 경계

WIP의 구형 runtime 변경은 최신 main의 카메라 launch, 배포 engine 경로·해상도·NMS,
GPU 입력 처리, ROS 메시지, Main/fusion·판제 및 IMU 설정을 덮어쓰지 않도록 해결했다.
원래 WIP의 상태는 위 revision에서 계속 조회할 수 있다.

이번 통합에서는 원본 이미지·라벨·split·checkpoint를 이동하거나 다시 생성하지 않는다.
수집·학습·export·배포 실행은 하지 않는다. 새 실행은 [자료 준비 안내](../../operations/data-preparation.md)의
저장 계약을 따르고, 관리형 워크스페이스에서는 저장 점검·새 run 등록 후 승인된 출력 경로를
도구의 `--output` 등에 명시한다. 기존 run ID나 결과 폴더는 재사용하지 않는다.
옛 증거의 경로·실험 정책·SHA는 당시 provenance이며 현재 출력 기본값이 아니다.

## 검증

실제 운영 이미지의 Ultralytics 8.4.150 / Torch 2.1.2에서 학습·전처리 CPU 회귀를 확인한다.
시험은 임시 fixture를 사용하고 GPU·수집·실데이터 학습을 요구하지 않는다.
추가한 입력 격리 시험은 selection-only의 heldout 제외, 최종 평가의 heldout 유지,
드라이브 이동 후 selection 경로 복원을 검증한다. 해당 시험과 기존 관련 회귀를 CI에 연결했다.
