# LOCAL-2D 준비: main 리베이스와 실행 소유권

2026-10-29 기준. 기존 LOCAL 결과는 `3bb14526`이며
`archive/local-before-rebase-20261029`로 보존했다. 원격 작업 브랜치의
`1125582c`도 `archive/local-remote-before-rebase-20261029`에 보존했다.
main `be20138a` 위로 이력을 리베이스했고 E3 실행 수정 `51c9672a`는
출처를 포함해 별도 적용했다. 원격 push/강제 push/병합은 하지 않는다.

## 운영과 역사적 실험의 경계

PR127의 BBOX classifier/follower와 CameraBBoxLaneResult 정의는 main과 동일하게
유지한다. S1/S2/S3의 옛 접점 실행·메시지 확장을 이 노드에 재적용하지 않았다.
옛 소스·bag·fixture는 백업 revision에서 함께 재현한다. 현재 기록 분석기는 옛
bag의 저장 메시지를 계속 읽을 수 있다. 과거 ROS wire 전용 시험 세 개와
contact_path_probe는 현행 실행/CI에서 분리하고 원본 파일을 보존한다.
순수 접점/관측 코어 회귀와 최신 main의 ROS 시험은 계속 실행한다.
시나리오 planning에 옛 ASMC_SHADOW_* 실험 옵션을 주면 명시적으로 거부한다.

LOCAL은 `asmc_planner/src/local`, `asmc_controller/src/local`과
LocalTrajectory를 유지하며 옛 follower를 시작하지 않는다. semantic_space_trial은
현재 LOCAL 입력 생산자로 계속 사용한다. NORMAL 실행 기본값을 LOCAL로 바꾸지 않았다.

## 컨테이너

운영 `asmc-ros-noetic`은 `2026-ASMC` / `51c9672a`에 그대로 연결한다.
AIMDrive worktree와 복구 이미지는 변경하지 않는다.
LOCAL 전용 이름은 `asmc-local2c-20261028`을 유지한다.
직접 worktree bind 요청이 Docker에서 옛 repro 이름으로 등록되는 현상이 재현되어,
해당 미시작 컨테이너를 제거하고 canonical 부모를 `/workspace`에 읽기 전용으로
마운트했다. `/root/ws`는 `/workspace/2026-ASMC-work`로 연결한다.
우리 build/devel/install과 tools/scen_runner/.runtime만 별도 쓰기 마운트다.
다른 worktree와 Git 공통 저장소는 컨테이너에서 읽기 전용이다.

시뮬 lease는 `/validation-lease`로 공유하고 ASMC_VALIDATION_LEASE_DIR을 사용한다.
엔진 선택 pointer는 LOCAL의 .runtime에 따로 두며 검증된 D 엔진만 읽는다.
운영 pointer를 덮어쓰거나 모델을 재생성하지 않는다.
옛 devel은 devel-before-local2d에 보존했다. 새 빌드는 baseline/devel overlay를
사용하지 않는다. 복구 이미지의 옛 /tmp/local2c-env.sh를 실행하지 않는다.

```bash
docker exec -it asmc-local2c-20261028 bash
cd /root/ws
source /root/local-env.sh  # 기본 독립 master 11348
```

실제 시뮬 시험에는 공유 lease뿐 아니라11311의 제어/인지 생산자 확인이 필요하다.
현재 운영 노드를 종료해 시험을 강제로 시작하지 않는다. 새로운 등록 run에서만
결과를 기록한다. build 중 실주행 성능을 비교하지 않는다.

## 검증 근거와 다음 단계

D의 `artifacts/local2d-20261029-rebase02`에 range-diff, 충돌 처리 목록,
클린 build 및 회귀 결과를 기록한다. 첫 등록 시 입력 manifest 형식 오류로 거부된
rebase01의 준비 로그는 별도 보존하며 성공 run으로 계산하지 않는다.
현재 준비 결과와 이후 MPC 공백/지면0.10m/회피 변경은 별도 커밋으로 구분한다.
실제 폐루프10km/h 회피·복귀 성공 여부는 이후 기록에만 표시한다.
