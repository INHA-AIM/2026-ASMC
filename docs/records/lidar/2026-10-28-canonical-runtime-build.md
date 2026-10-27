# 병합된 3뷰·터널 최적화의 기본 실행 일치

2026-10-28. 기준선은 PR #122 병합본 `54f6bb9a`다. 이 작업은 알고리즘 변경 없이
기본 workspace와 실제 실행파일을 일치시키고, 다음 진단에서 같은 혼용을 구분하도록 한다.

## 원인과 적용

마지막 사용자 실행 기록은 `/root/ws/devel/lib/asmc_lidar/lidar_pipeline_node`였다.
기존 파일에는 GPS 무효 터널 분기는 있었으나 `MultiCameraFusion`과 최적화한
`EuclideanComponents` 구현은 없었다. 해당 구현은 개인 `/root/asmc-m1` 빌드에 있었다.
YAML의 `fusion.views: all`만으로 실행파일이 갱신되지 않으며, 두 빌드의 차이를
3뷰 알고리즘의 실시간 성능 차이로 해석할 수 없다.

- `/root/ws`에 작업 commit `80c50466`을 적용해 **전체 outer workspace를 Release 재빌드**했다.
  호스트 개발은 `2026-ASMC-work`의 `fix/lidar-runtime-build-alignment` 브랜치에서 계속한다.
  컨테이너 마운트는 유지하고 마운트된 `2026-ASMC` checkout만 해당 commit으로 전환했다.
  main branch ref와 별도 연구 브랜치는 변경하지 않았다.
- 기본 실행파일에서 `TunnelRoute::Select`, `MultiCameraFusion::Process`,
  `EuclideanComponents`가 모두 포함된 것을 확인했다. 이는 코드 포함 확인이며 수신 성공/성능 수락과 다르다.
- 컨테이너 `.bashrc`의 개인 M1 자동 overlay를 제거했다. 백업은
  `/root/.bashrc.before-lidar-canonical-20261028`이고 M1 폴더 자체는 재현용으로 보존했다.
  새 interactive bash와 명시적 `/root/ws/devel/setup.bash` 모두 기본 실행파일을 선택한다.
- 이전 PR #114 시험 로그를 stdout으로 사용하던 미등록 camera/LiDAR launch 잔재만
  PID·명령·로그 소유 확인 후 SIGINT로 정리했다. 사용자 bridge·웹·master는 유지했다.
  새 운영 LiDAR·카메라·차량 제어는 자동으로 시작하지 않았다.

## 재발 확인 장치

`lidar_pipeline_node --build-info`는 ROS 초기화 전에 빌드 정보를 출력하고 끝난다.
CMake가 `src/`, `include/`의 C++ 소스/헤더와 `CMakeLists.txt`, `package.xml`,
빌드 식별 템플릿의 내용을 SHA로 묶는다. 파일 수정/추가/제거 시 다시 configure한다.
`tools/check_runtime_build.py`는 ROS 환경에서 선택되는 패키지·실행파일과 현재 소스를 대조하고,
불일치/구형 실행파일이면 실패한다. 비교만 하며 모델·센서·제어 노드를 시작하지 않는다.

확인된 package source SHA:

```text
0b4da091b7c9c25676c9cf7bfd97eb10e676071eca671d8870b8132032bdc28d
```

호스트 work, 컨테이너 `/root/ws` 소스와 바이너리 내 SHA가 일치했다.
이는 **asmc_lidar C++ 패키지 소스**의 동일성이다. 의존 라이브러리 전체·실행 중인 다른
프로세스·모델·지도·YAML까지 동일하다는 증명이 아니다. YAML은 기존 config ID/SHA를 사용한다.

기존 1Hz diagnostics에 빌드 SHA/type와 `fusion_views`, `original_tunnel_pipeline`을 추가했다.
시작 로그에는 실제 실행파일·config 경로도 남긴다. 선택 모드와 각 뷰의 유효 수신·matching은 구분한다.
메시지 정의/MD5·토픽·모델·필터·추적·판제 정책은 변경하지 않았다.
소스 해시는 빌드/명시적 점검 때만 계산하며 scan 처리에 파일 해시나 점군 복사를 추가하지 않는다.

## 검증

| 검사 | 결과 |
|---|---|
| 전체 outer workspace Release 빌드 | 통과; 기존 devel의 LiDAR만 바꿔치기하지 않음 |
| 새 interactive 셸의 package/executable 선택 | `/root/ws/src/asmc_lidar`, `/root/ws/devel/lib/asmc_lidar/lidar_pipeline_node` |
| 빌드 식별 회귀 | 같은 소스/다른 위치 일치, YAML만 변경 시 C++ 동일, 소스 변경 시 불일치 검출 |
| LiDAR 전체 CTest | **41/41**, 55.79초; 터널/GPS 전환·3뷰·관측·웹·실제 ROS 시험 포함 |
| 필수 pipeline CI gate | **36/36** 필수 항목 통과; 새 빌드 일치 시험도 필수 등록 |
| 공용 LiDAR→판제/전체 autonomy 경계 | 독립 master 합성 입력 통과. GPS/IMU 무효·복귀·reset·재시작·정상 빈 결과와 단일 생산자 확인 |
| 웹 표시 | **72 passed, 0 failed** |
| 실행 문서/보존/CI coverage | launch26개·literal command89개, 동결자료5개 및 기존145개 검사 유지 |

로컬 실행은 `.github/ci/lidar.sh`와 기존 문서 검사를 사용했다. ROS 시험은 별도 master와
`/tmp` 단위시험 출력만 사용했다. 새 bag 수집·MORAI 주행·GPU 추론 시험은 하지 않았다.
Hosted CI 결과는 PR의 실제 상태를 따르며 위 로컬 결과와 혼동하지 않는다.

## 실행과 남은 범위

사용자는 새 컨테이너 터미널에서 [기본 실행 명령](../../operations/commands.md)을 사용한다.
현재 컨테이너는 이미 재빌드했으므로 기존 터미널에서는 `/root/ws/devel/setup.bash`를 다시 적용한다.
빌드 점검의 `source_matches: true`를 확인하고 `roslaunch asmc_lidar lidar.launch`를 실행한다.
M1 overlay나 별도 YAML 선택은 필요 없다. 변경 전 셸 설정으로 복구할 수 있지만,
운영 복구는 빌드/설정이 일치하는 revision을 재빌드하는 방식으로 한다.

RANSAC 실패 단계와 음영 제어기의 차로 판정/30초 정지 고착은 **미해결**이다.
기본 실행 일치가 이를 해결했다고 주장하지 않는다. 다음 작업은
[음영 정지 경계 우선 검토](../../../src/asmc_lidar/docs/improvement-milestones.md#pr-122-이후-실행-기반과-다음-단위-제안--2026-10-28)를 제안한다.
얇은 장애물 복구와 새 통로 기반 local planner는 별도 단위다.
