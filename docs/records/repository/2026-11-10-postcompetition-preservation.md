# 대회 종료 후 소스·연구 런타임 보존 — 2026-11-10

사용자가 최종 대회 주행 브랜치로 확인한 `integrate/jamming-three-way-20261029`는
[PR #141](https://github.com/INHA-AIM/2026-ASMC/pull/141)로 main에 병합됐다.
대회 코드와 실험 자료를 보존하면서 실행 정본을 하나로 줄이기 위한 기록이다.
현재 실행 명령은 [commands](../../operations/commands.md), 방식·전환은
[jamming comparison](../../operations/jamming-comparison.md)을 따른다.

## 소스 기준선

| 참조 | SHA / 의미 |
|---|---|
| 병합 전 main | `af911727d2001703ea43a7d6c394d0650e74c489` |
| 대회 최종 소스 | `a62360ec43fb7cd543e07d1fab03c9d2f577c7ee` |
| 대회 보존 태그 | `competition/2026-final` |
| #141 병합 main | `d2b18f745ed334ab809836be7f718aa7c125a00c` — 대회 최종 소스와 tree 동일 |

#142 camera, #143 로터리, #144 LiDAR 커밋은 #141을 통해 main에 포함됐다.
#125·126·138·139는 예전에 병합 없이 닫힌 비교/연구 PR이다.
이번 문서 정비는 대회 주행 소스의 알고리즘·임계값·모델·메시지를 변경하지 않는다.

## 미커밋 연구 보존

아래 커밋은 **미검증 연구 보관용**이다. main에 자동 병합하거나 현재 실행 안내로 사용하지 않는다.
각 원본 워크트리의 변경·미추적 파일을 확인해 선택 파일만 커밋했고, 보존 태그를 원격에도 저장했다.

| 보존 대상 | 커밋 | 태그 |
|---|---|---|
| LOCAL의 CI 등록과 과거 카메라 문서 | `6976f51834da5d3720422fc51443662788186d00` | `archive/20261110/local-ci-wip` |
| localization의 이전 진입/벽 확장·시험 WIP | `0f81ff3599b3fe753215715e2bc384cdd0cc8012` | `archive/20261110/localization-transition-wip` |
| 비Git AIMDrive 폴더의 commands 원본 | `0f1023035b128acf16092956958afb435d6549a3` | `archive/20261110/aimdrive-commands` |

LOCAL은 tracked5개+문서1개, localization은 tracked10개+미추적3개를 보존했다.
이전 커밋 이력은 이미 main에 포함돼 있으며, 이번 보존은 남은 미커밋 변경을 잃지 않기 위한 것이다.

LOCAL CI 등록은 대부분 main에 이미 존재한다. 과거 카메라 문서는 현재
`docs/records/camera/camera-family-supply-optimization.md`와 경로·안내가 다르다.
localization WIP에는 GPS-loss-only 선택과 paint 예외에 연결한 벽 권한 등 최종 코드와 다른 정책이 있다.
현재 main의 wall_permission·공통 전환을 이전 WIP로 덮어쓰지 않는다.
미추적 벽 통로 JSON은 main과 바이트 동일하지만, 새 speed-stage 시험은 당시 정책의 미검증 fixture다.

AIMDrive commands 보존 커밋은 역사적 문서 한 개를 담은 별도 archive 이력이다.
현재 실행 문서로 복사하거나 main에 병합하지 않는다. 필요하면 다음처럼 내용을 읽는다.

```bash
git show archive/20261110/aimdrive-commands:docs/operations/commands.md
git show archive/20261110/local-ci-wip --stat
git show archive/20261110/localization-transition-wip --stat
```

## 연구 런타임 보존 범위

새 보존 run은 `postcompetition-runtime-preservation-20261110-r2`, 저장 위치는
D 정본의 `data/runtime/postcompetition-runtime-preservation-20261110-r2`다.
큰 자료를 팀 Git 저장소에 넣지 않으며, 입력/config/script SHA와 원본 경로는 해당 run의 manifest로 확인한다.

대상은 중지된 `asmc-jamming-compare`, `asmc-jamming-compare-build`,
`asmc-jamming-pr141-sync-build`, `asmc-shadow-localization-previous`,
`asmc-aimdrive-rebase-20261029`의5개 연구 컨테이너다.

- 컨테이너의 추가·변경된 `/tmp`, ROS 로그, env/실행 스크립트를 별도 아카이브로 보존한다.
- 각 컨테이너의 파일시스템은 `asmc/archive:postcompetition-20261110-r2-<기존 이름의 asmc- 이후 부분>` 복구 이미지로 고정한다.
- 생성 설정·이미지 ID·마운트·변경 목록·확인 결과를 함께 보존한다. 인증 환경 변수는 별도 보존 목록에서 제외한다.
- host의 `.runtime`·이전 build/devel·배포 rollback snapshot은 별도 아카이브와 파일별 SHA로 보존한다.
- **복구 이미지에는 bind mount가 포함되지 않는다.** Git 태그, host 아카이브와 원본 D/E 데이터를 함께 참조해야 한다.
- 이미지 표시 크기는 공유 layer를 포함한다. 크기 합계를 삭제 후 회수량으로 사용하지 않는다.

수집기의 최초 r1은 기반 이미지의 `/tmp`까지 중복 수집해 중단했다. 그 수집기만 만든 미완성 `.part`를 정리하고,
실패 메타데이터를 남긴 채 r1을 종료했다. 원본 로그·컨테이너·기반 이미지는 유지했다.
r1을 검증 완료 보존본으로 사용하지 않는다. r2의 최종 정본은 `preservation-manifest.json`이며,
원시 수집 기록인 `result.json`과 구분한다. Docker commit 출력에 섞인 CLI 안내 문구는 원시 기록에 보존하고,
최종 manifest의 이미지 ID는 `docker image inspect`로 독립 확인했다.

r2에서 컨테이너403개+host9개 아카이브, 합계404,270,428B(약385.5MiB)를 보존했다.
host102,416개 항목은 백업 시 파일별 SHA/링크를 대조했고, 전체412개 아카이브 SHA와 복구 이미지5개,
원본 컨테이너5개의 ID·종료 시각·변경 파일 목록을 다시 확인했다.
이미지는 Docker 로컬 저장소에 있고 host/로그 아카이브는 D에 있다. 어느 쪽도 단독으로 전체 복구본이라고 간주하지 않는다.

이번 범위는 보존까지다. 컨테이너·worktree·이미지·D/E 자료를 정리했다고 해석하지 않는다.
실행 중인 정본 `asmc-ros-noetic`, 별도 `vip3-ros-noetic`과 그 마운트, ERC 환경은 유지한다.
특히 개인 `2026-ASMC-repro` 경로는 VIP3 컨테이너가 사용하므로 ASMC 잔여 폴더라는 이유로 지우지 않는다.

## 남은 실행 준비·정리

main checkout은 최신이어도 남아 있는 devel/install은 별도다.
보존 전 정본 컨테이너의 LiDAR build-info 검사에서 source_matches=false였다.
소스 정리나 Git 태그만으로 새 바이너리를 검증했다고 표시하지 않는다.

다음 단위에서 정본 main을 재빌드하고 source/build/config/모델 선택을 확인한다.
연구 컨테이너의 devel을 그대로 복사하면 이전 절대 경로가 남을 수 있다.
그 뒤에야 보존 manifest와 작성 프로세스/마운트를 다시 확인해 컨테이너와 worktree를 해제한다.
현재 단위는 재빌드·주행·삭제를 포함하지 않는다.

## 문서·검증의 경계

현재 문서에서 코드로 확인한 값은 localization+LOCAL 기본 선택, 방식별 속도와
master11311·판제8088·LiDAR8091, 기본 debug=false,3뷰 fusion 및 wall/vehicle YAML 선택이다.
과거 비교 프로필·원본 명령·실패 근거는 [정리 전 안내](../planning/2026-10-30-jamming-comparison-before-closeout.md)와 개별 기록에 남긴다.

기존 CI run40966116951의 생산 빌드는 성공했지만 전체 CI는 실패였다.
문서 reference 미갱신과 별개로, perception 시험의 설정 파일 읽기 오류와 planning의 과거 기본값 기대가 남아 있었다.
이번 문서 검사 통과를 그 실행 시험이나 새 실주행 수락으로 대체하지 않는다.

## 다음 문서 대조 단위

이번에는 실행 시작점·기본값·포트·보존/정리 경계를 먼저 맞췄다.
아래 문서는 후속 단위에서 각 소스와 대조하며, 제목의 현재/완료만으로 내용을 수락하지 않는다.

| 대상 | 대조할 내용 |
|---|---|
| [판제 운영](../../operations/planning-control.md) | 세 방식·소유권·입력·전환 계약과 현재 launch |
| [LiDAR 이관](../../handoffs/lidar-migration.md) | 폐기된 Main 경로와 현재 구현/후속 과제의 상태 |
| [음영 구조 분석](../../architecture/shadow-planning-refactoring.md) | 대회 이전 제안과 최종 구현의 구분 |
| [LiDAR 판제 연결](../../../src/asmc_lidar/docs/planning-runtime.md) | 최종 객체/속도/필터·local/map·실행 경계 |

역사적 실험 결과의 수치·SHA·실패를 최신 코드 설명으로 바꾸지 않는다.
현행 안내를 고칠 때는 원문 고정 revision과 재현 근거를 남기고 작은 문서 커밋으로 반영한다.

## 이번 문서 단위 검사

- 소스 기반 launch reference 재생성: 27개 launch·문서의109개 literal command 검사 통과.
- 문서 migration/link/동결 snapshot 검사 통과.
- runtime documentation 및 migration 회귀18개 통과.
- 변경 경로는 docs에 한정하며 실행 코드·launch·YAML·메시지는 변경하지 않았다.
- 큰 백업과 호스트 개인 생성 설정은 Git에 올리지 않았다. 새 실주행·운영 재빌드는 실행하지 않았다.
