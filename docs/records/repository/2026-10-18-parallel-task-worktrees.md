# 동시 작업의 브랜치·실행 경계

2026-10-18 사용자 요청에 따라 IMU 운동 분석을 별도 worktree로 분리했다.
각 작업은 로컬 커밋으로 보존하고, 완료 후 검토 가능한 단위로 PR을 게시한다.
중간 push·PR 생성으로 hosted Actions를 실행하지 않는다.

| 작업 경로 | 확인한 브랜치·기준 | 이번 작업의 책임 |
|---|---|---|
| `2026-ASMC-camera-motion` | `experiment/camera-imu-motion`, 시작점 `2af672d1` | 차체 자세·시각 정합·터널 bag·seg와 family의 차이 분석 |
| `2026-ASMC-work` | `train/twinlite-large-capture-v7`, `2af672d1` | 기존 Large 학습·배포와 선행 IMU 구현 이력 보존 |
| `2026-ASMC-lidar-research` | `research/lidar-perception-20261018`, 확인 시 `4da2a5b0` | 별도 인지 개선 연구. 이 worktree의 미추적 연구 도구를 가져오지 않음 |
| `2026-ASMC` | `review/twinlite-bdd-large`, 확인 시 `4585e9ab` | 기존 checkout과 데이터 경로 정리 작업 보존 |
| `2026-ASMC-repro` | `repro/camera-runtime-20261014` | 기존 재현 기준·컨테이너 마운트 보존 |

표는 확인 시점의 소유 경계이며 다른 담당자의 브랜치나 변경을 고정하지 않는다.
`2026-ASMC-work`와 `2026-ASMC`의 `scripts/docker_ros_up.sh`, `scripts/docker_perception_up.sh`
미커밋 변경은 저장소 밖 `.storage` 설정을 읽는 데이터 경로 작업으로 확인했다.
이번 IMU 작업에서 stage·stash·reset·commit하거나 다른 worktree로 복사하지 않는다.
LiDAR 연구 worktree의 미추적 도구도 그대로 둔다.

새 IMU 브랜치는 선행 Large·bridge 변경을 사용하는 `2af672d1`에서 분기했다.
최종 PR을 main에 올리기 전에는 이 기준점의 선행 변경과 새 변경을 구분해 diff·의존성을 검토한다.
다른 작업의 미완료 커밋을 통째로 합치거나 작업 중인 checkout을 전환하지 않는다.

컨테이너 mount와 live runtime overlay는 Git worktree와 별개다.
오프라인 IMU 분석은 `data/runtime/camera-imu-motion-20261018-r1/`에 별도 입력·코드 SHA·결과를 둔다.
다른 연구의 실행 파일·기존 원본 bag·모델·평가 결과를 덮어쓰지 않는다.
실시간 시험이 필요하면 한 bridge·한 camera 생산자만 사용하고, 시험별 입력/실행 ID와 변경 설정을 기록한다.
시험 기록기의 종료와 정상 프로필 복원을 확인한 뒤 결과를 커밋한다.
