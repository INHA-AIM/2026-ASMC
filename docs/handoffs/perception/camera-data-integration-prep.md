# PR #81/#82 도구 통합·데이터 구분 — 2026-10-14

**2026-10-14 사용자 승인에 따른 main 통합 범위와 데이터 상태를 정리한다.**
기존 PR 번호와 이력을 유지하고 #81 → #82 순서로 통합한다. #81은 자료 정리 기반이고,
#82는 이를 사용하는 수집 도구 확장이다. #81은 `46592620`으로 병합했다.
#82의 최종 병합 커밋·CI 상태는 [PR #82](https://github.com/INHAautonav/2026-ASMC/pull/82)에서 확인한다.
배포 기반은 #84의 `c70e5d7a`이며 아래 도구 통합으로 운영 모델을 교체하지 않는다.

## main 통합 범위

| 범위 | #81 | #82 |
|---|---|---|
| 자료 도구 | YOLOE-X/Grounding DINO B/L의 로컬 비교·예측 저장, 원본 의미 검수, v6/v7 채택·export·누출 검사 | Capture GT와 actor 연결, 가시 bbox 생성·재검증, 손상 RGB/누락 파일 격리 |
| 수집 도구 | 초기 파일럿 설계 기록 | 경로 배치·회전·접지 교정, MPC/PP+P 수집 coordinator, 1Hz 단일 Capture, 메모리 감시·소유 actor 정리 |
| Git 산출물 | 도구·설정·시험·고정된 실험 요약 | 도구·접지/주석 정책·시험·수집 성공/실패/종료 요약 |
| 원본 데이터 | 이미지·모델·대형 manifest는 기존 로컬 데이터 영역에 보존 | 동일. PR 병합이나 빌드만으로 원본을 복사·변환하지 않음 |

일반 perception/Main/판제 launch에 수집기를 자동 연결하지 않는다. 수집은 별도 수집 launch 또는
CLI를 명시 실행할 때만 동작한다. 해당 도구는 스폰·날씨·차량 제어를 수행할 수 있으므로 실제
수집 전에 센서 Capture 설정, 제어 생산자, actor 소유권과 저장 경로를 확인해야 한다.

## 데이터와 모델의 상태를 구분한다

| 기록 | 의미 | 수치 또는 상태 |
|---|---|---|
| v7-adoption-r2 | #81 당시 검수·채택한 고정 데이터 버전 | 전체23,264장·129,630객체; train22,023 / selection113 / heldout1,128 |
| 추가 Capture export | #82의 정상 RGB·필수 GT·actor 검증을 통과한 수집 부분집합들 | 31,388장·403,814개 주석 |
| v7 + 추가 Capture 후보 재고 | #82 종료 당시 합산 inventory; 단일 통합 학습 export가 아님 | train 후보53,411장·530,009개 주석; 평가1,241장은 별도 |
| v8~v11 전처리·본학습 | 이후 작은 GT/ignore/배경/차량 복원·평가 재구성 작업 | 관련 도구·증거의 로컬 미커밋 변경은 다음 작업 단위에서 별도 PR |
| 현재 main 모델 | PR #84에서 게시한 v11 E8 배포 | [현재 배포 계약](camera-mission16-deployment.md), [모델 파일/SHA](../../../config/models/camera_object_models_v1.json) |

53,411/530,009를 현행 v11의 유효 양성 수로 설명하지 않는다. #82는 당시 작은 GT를 일괄 제거하지
않았으며 후속 4px/64px²·ignore·배경 손실 정책의 구현을 포함하지 않는다. 이미지 수는 독립 장면 수가
아니며, 동일 배치의 여러 카메라·날씨·연속 관측은 계보를 보존한다.

학습 클래스16종 ID 순서는 유지한다. v2/v7 manifest의 `consumer_group`에는 당시4그룹 분석 의미가
남아 있다. 이를 runtime으로 그대로 전달하지 않는다. 현재3그룹은
`object_classes_mission16_v1.yaml`과 [배포 계약](camera-mission16-deployment.md)이 정본이다.
원본 라벨·MORAI 에셋·teacher phrase·학습 클래스·소비 그룹은 서로 다른 정보다.

## 성공·미완료·과거 기록

#81은 직접 대응 주석과 개별 혼합 라벨 검수로 v7을 생성했다. teacher의 자동 승인 상태는 false다.
과거 teacher recall에는 당시 GT 범위·극소 객체가 포함돼 있으며 현재 평가 정책의 성능으로 전용하지 않는다.
희소 클래스의 독립 평가·장면 다양성은 데이터 지원 범위의 한계다. 도구 병합이 이를 해결했다는 뜻은 아니다.

#82의 초기 실패 이후 FOGGY11·SUNNY15·FOGGY13 세 회차를 추가 완료했다.
마지막 종료에서는 차량 정지·날씨/시간/기어/모드 복원과 소유 actor 정리(Ego만 남음)를 확인했다.
과거 메모리 고갈·RGB 손상·파일 누락 실행은 그대로 보존한다. 모든 중단 원인의 복원과 새 PC 환경,
E드라이브 경로 및 전체 에셋을 포괄하는 새 수집 계획을 수락한 것으로 확대하지 않는다.

수집 도구의 기본 배분 수치는 초기 v7의 예시다. 차량 보강과 재생성은 이후 별도 수집 경로로 수행했다.
새 수집에서는 [현재 후속 계획](camera-deployment-roadmap-20261014.md)에 따라 최신 유효 양성·하위 유형·
조건별 재고와 API 목록을 다시 계산한다. 기존 스크립트가 이미 모든 차종·사람 번호·E드라이브 수집을
지원한다고 가정하지 않는다. 현행 저장 기본은 `/data`이며 E드라이브 전환은 다음 구현이다.

## 이번에 포함하지 않는 변경

- #84의 E8/사전학습 가중치·모델 선택·3그룹·공개 ROS 메시지·confidence·fusion 정책을 바꾸지 않는다.
- shared pip 설치·컨테이너 마운트·bridge 센서 프로필을 변경하지 않는다.
- 데이터를 새로 다운로드/수집/재라벨링하거나 학습을 실행하지 않는다. 데이터 버전을 자동 승격하지 않는다.
- 로컬 v8~v11 전처리·본학습·보고·갤러리 변경을 한꺼번에 가져오지 않는다.
  원래 작업 worktree의 미커밋 파일을 전후 fingerprint로 확인한다.

## 통합 검증과 이력

관련 CPU 시험73개와 `asmc-ros-noetic`의 격리 수집 패키지 빌드·시험45개가 통과했다.
두 시험 집계에는 중복이 있다. P1B·perception·topic policy·문서 링크·diff 검사와 수집 launch
노드 해석도 통과했다. 카메라·Main/fusion·판제·제어·모델·계약 경로는 #84 배포 기반과 같다.
#81의 hosted CI는 통과했고, #82는 최종 갱신 커밋의 hosted CI를 별도로 확인해 병합한다.

[기존 준비 증거](../../../config/baselines/evidence/camera_data_integration_preparation_v1/pr81.json)와
[#82 준비 증거](../../../config/baselines/evidence/camera_data_integration_preparation_v1/pr82.json)의
Draft·pending은 당시 상태를 보존한 기록이다. 최신 병합 상태와 혼동하지 않는다.
원래 작업 worktree의 미커밋 파일151개는 내용·Git 상태를 유지했으며 후속 작업 단위로 남긴다.
이번 승인 범위는 도구·문서 통합이다. 새 스폰·차량 제어·수집·학습은 실행하지 않는다.

## #82 실행 경로

- [Capture coordinator](../../../src/data_collection/scripts/capture_route_collector_node.py): 단일 요청·freshness·제어 소유권·중단/종료 기록.
- [수집 launch](../../../src/data_collection/launch/capture_route_collection.launch): 명시 실행 시만 동작. 기본 MPC·gRPC·20km/h·30 Capture이며 PP 대체는 별도 새 run이다.
- [배치/실행 정책](../../../src/data_collection/src/data_collection/capture_collection.py), [반복 실행기](../../../tools/perception/run_capture_route_collection.py): 목표 달성·중단·최대 두 바퀴·기한과 실제 지원 에셋 검사.
- [GT export](../../../tools/perception/export_capture_route_bbox.py), [독립 재검증](../../../tools/perception/verify_capture_route_dataset.py): native ID·가시 instance·원본 SHA·고정16종·평가본 중복 검사.
- [종료 시점 요약](../../../config/baselines/evidence/camera_capture_collection_v1/summary.json)의 `final_verified_stock`·`collection_end`·`last_two_hour_collection`을 사용한다.
  상위 `remaining`은 초기 장애 발생 시 기록이며 최신 실행 지시가 아니다.

전체6조건의 주행 통과 기록과 정상 영상/GT의 채택은 별개다. 마지막 세 회차만도10,133장·145,474개
주석을 보강했지만 수집량을 클래스별 독립 장면 수나 후속 학습 성능 보장으로 사용하지 않는다.
