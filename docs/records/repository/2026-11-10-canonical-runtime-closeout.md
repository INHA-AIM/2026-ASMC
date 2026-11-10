# 정본 재빌드와 연구 실행 환경 정리 — 2026-11-10

`2026-ASMC`의 main을 `asmc-ros-noetic:/root/ws`에서 재빌드하고, 보존을 확인한 연구 컨테이너와
worktree를 정리했다. 검증 소스는 `2a1a4e218eddce03529c56770f333be70d179e18`이며,
이번 정리로 주행 알고리즘·임계값·메시지·모델·launch 기본값을 변경하지 않았다.

## 실제 확인 범위

- `./scripts/build_ws.sh runtime -j2 -l2` 성공. 기존 정본 build/devel에서 최신 소스를 빌드했으며
  다른 worktree의 devel/install을 복사하지 않았다. 새 환경의 clean build 검증은 아니다.
- LiDAR 실행파일과 소스 SHA 일치: `a8c6f483a46f58e40ac0a702cd9f32b36906a6acf963e6d85cfab7c0a11dbb82`.
  `check_runtime_build.py`의 `source_matches=true` 및 실제 시작 로그를 확인했다.
- `camera.launch`, `lidar.launch`, `autonomy_no_bridges_v2.launch`, `view.launch`를
  같은 master11311에서 기동했다. LiDAR는 `lidar_wall_vehicle.yaml`,3뷰 fusion, original tunnel 경로를 사용했다.
- YOLO E3의 검증된 로컬 TensorRT engine SHA는
  `96694250e0f358f1b0d5ebfdd136b86fecf13761d2f995bbabd49fc371e17e73`이다.
  TwinLite Large E8·신호등 모델도 로딩 완료했다. 엔진을 새로 생성하거나 모델 선택을 변경하지 않았다.
- localization+LOCAL 복구와 단일 localization MPC를 확인했다. 별도 LOCAL 제어기나 bridge는 시작하지 않았다.
- 세 방식 launch graph, LiDAR 정본 의존성 검사, 문서 실행 검사와 관련 Python 회귀28개가 통과했다.
- 판제8088·LiDAR8091의 HTTP200을 컨테이너와 Windows localhost에서 확인했다.
  테스트가 생성한 master·노드는 모두 종료했다. 정본 컨테이너 자체는 실행 상태로 유지했다.

센서 입력·MORAI bridge·실제 주행을 연결하지 않은 **기동 확인**이다. 센서 공급률·판제 성능·충돌 회피나
전 구간 주행을 재수락한 것이 아니다. 이전 PR #145의 hosted CI run42834895770은3개 job 실패 상태다.
이번 운영 빌드·기동 및 선별 검사 통과를 전체 CI 통과로 대체하지 않는다.

## 현재 실행과 웹

정본은 `2026-ASMC` / `asmc-ros-noetic`, ROS11311·판제8088·LiDAR8091이다.
기존 셸을 닫고 새 터미널로 들어오면 현재 main의 단독 실행 보호 스크립트와 `/root/ws/devel/setup.bash`를 사용한다.
브릿지·카메라·LiDAR·자율주행 순서는 [명령 모음](../../operations/commands.md)을 따른다.
기본 음영 선택은 localization+LOCAL 복구이며 이전 camera 방식은 명시적 선택으로 남아 있다.

```bash
docker exec -it asmc-ros-noetic bash
```

LiDAR 웹의 기본 loopback 바인딩은 이 Docker Desktop 환경에서 Windows까지 연결되지 않았다.
다음 명령은 포트를 유지하고 HTTP 수신 주소만 명시한다.

```bash
roslaunch asmc_lidar view.launch bind_address:=0.0.0.0
```

- 판제: <http://localhost:8088/web/index_v2.html>
- LiDAR: <http://localhost:8091/>

`0.0.0.0`은 외부 인터페이스에도 수신하므로 호스트 방화벽의 접근 범위를 유지한다.
이 웹 설정을 위해 주행 정책이나 센서 네트워크를 바꾸지 않았다. 연구용11373/11374·8089/8097은 현행 실행 안내가 아니다.

## 제거한 실행 환경과 보존 범위

사용자 승인 후 실제 삭제 목록을 먼저 안내하고, SHA·마운트·중지 상태와 작성 프로세스를 재확인했다.

| 구분 | 정리한 대상 |
|---|---|
| 중지 연구 컨테이너5개 | `asmc-jamming-compare`, `asmc-jamming-compare-build`, `asmc-jamming-pr141-sync-build`, `asmc-shadow-localization-previous`, `asmc-aimdrive-rebase-20261029` |
| Git worktree4개 | `2026-ASMC-work`, `2026-ASMC-shadow-localization`, `2026-ASMC-jamming-compare`, `2026-ASMC-jamming-pr141-sync-20261030` |
| 비Git 잔여 폴더 | `2026-ASMC-aimdrive`의 보존된 commands 문서와 빈 마운트 폴더 |
| 로컬 브랜치2개 | `feat/shadow-local-footprint`, `integrate/localization-compare-20261029` — 같은 SHA의 archive 태그 유지 |

현재 등록된 ASMC worktree·로컬 개발 브랜치는 `2026-ASMC / main` 하나다.
Docker 이미지·volume prune이나 D/E 데이터 삭제는 하지 않았다. Git worktree 제거 시 링크 대상은 따라가지 않았다.
학습·실험 갤러리 컨테이너, ERC/VIP3의 별도 환경 및 `2026-ASMC-repro`는 이 정리에서 제외했다.

복구 근거는 [보존 기록](2026-11-10-postcompetition-preservation.md)의 원격 archive 태그3개,
대회 최종 태그, Docker 복구 이미지5개 및 run `postcompetition-runtime-preservation-20261110-r2`다.
삭제 전 전체412개 아카이브 SHA, 대상 host 항목과 컨테이너 ID·종료 시각·변경 목록을 대조했다.
Git 밖 엔진 선택·시나리오 설정6개도 별도 개인 복구 기록으로 보존했다.

개인 워크스페이스의 `.storage/runtime-consolidation-current.json`이 정확한 삭제 계획 SHA,
실제 결과, 빌드·기동 로그와 복구 manifest를 연결한다. 과거 동결 run과 기록은 덮어쓰지 않는다.
소스는 태그에서 별도 worktree로 복원할 수 있으나, 옛 devel은 절대 경로가 포함돼 현재 정본에 복사하지 않는다.
연구 런타임 전체 복구에는 해당 run의 `RESTORE.md`, Docker 이미지, host 아카이브와 원본 데이터가 함께 필요하다.

## 후속 문서 작업

이번에는 실행 환경·포트·정리 상태만 최신화했다. 판제 운영, LiDAR 인계, 음영 구조 분석 등
[남은 소스 대조 목록](2026-11-10-postcompetition-preservation.md#다음-문서-대조-단위)은 별도 단위로 진행한다.
과거 실패 수치·SHA·실험 명령은 당시 근거로 보존한다.
