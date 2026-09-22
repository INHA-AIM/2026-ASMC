# D3 정보 보존·경로 호환 감사

> **기준 원문:** `main@577231b` · 2026-09-22
>
> **기계 정본:** [`documentation_migration_v1.json`](../../config/docs/documentation_migration_v1.json)
>
> **결론:** 원문 행선지가 없는 문서는 발견되지 않았다. 호환 안내와 historical 자료는 아직
> 삭제 gate를 충족하지 않아 유지한다.

## 1. 감사 범위와 방법

D1 이전 `docs/` 트리와 D2 완료 트리를 다시 대조했다. 단순히 현재 문서의 총 줄 수만 비교하지
않고 다음 네 층으로 확인했다.

1. `577231b`의 Markdown 37개와 generated text 2개를 파일 단위로 전수 등록한다.
2. 각 원문의 현재 정본·기록·archive 행선지와 처리 방식을 명시한다.
3. byte 보존 대상은 SHA-256과 Git blob을 비교하고, 장문 본문은 heading과 원문 행 대응을
   D2 묶음 보고서와 다시 대조한다.
4. 현재 문서는 호환 stub을 경유하지 않고 정본을 직접 링크하게 검사한다.

재검증 명령은 다음과 같다.

```bash
# shallow CI에서도 실행되는 현재 tree 검사
python3 tools/audit/check_documentation_migration.py

# 577231b 이력이 있는 clone에서 원문 inventory와 byte 이동까지 검사
python3 tools/audit/check_documentation_migration.py --with-history
```

## 2. 원문 보존 결과

| 처리 | 파일 | 원문 줄 | 확인 결과 |
|---|---:|---:|---|
| Frozen snapshot byte 동일 | 3 | 256 | 현재 SHA와 원문 blob 동일 |
| Generated message export byte 동일 | 2 | 508 | wrapper·CRLF를 포함한 archive 파일 그대로 유지 |
| 상세 본문 유지 후 상태·링크 현재화 | 21 | 8,130 | 원문 heading 410개 중 399개가 동일, 나머지는 아래처럼 의도적으로 현재화 |
| 절 단위 통합 | 5 | 798 | D2 대응표에서 고유 규정·환경·실행·검증 정보를 확인 |
| 권위에 따라 guide/record/archive로 분리 | 3 | 906 | 현재 설정과 날짜별 실측 또는 historical 실행을 분리 |
| 오래된 정책만 제외하며 통합 | 4 | 1,131 | 제외 항목과 이유를 D2 parity 감사에 명시 |
| 문서 허브 진화 | 1 | 206 | 정본 우선순위와 모든 새 행선지를 포함 |

Markdown 기준은 37개, 11,427줄, 640,681 bytes이고 generated text까지 포함하면 39개,
11,935줄이다. 이미지 5개도 경로와 byte content가 그대로다. 감사 착수 시점의 현재 문서는
호환 stub을 포함해 74개, 12,147줄이므로 총량은 줄지 않았지만, 이 수치는 보존의 근거가
아니라 위 파일별 행선지·byte·절 대응 검사의 보조 지표로만 사용했다.

장문 21개에서 이름이 그대로 남지 않은 heading 11개는 정보 손실이 아니었다.

- 완료 전 범위를 뜻하던 HD Map `이 단계에서 하지 않는 일`은 구현 완료 상태로 교정했다.
- FSM/Frenet의 `D0`, `D3`, `D0~D10` 단계명은 현재 production/rollout 상태로 교정했다.
- `morai_msgs`의 “추후 submodule 등록” 절과 옛 담당 표는 이미 등록된 pinned submodule의
  현재 사용·ownership으로 교체했다. 코드 블록의 `#` 주석 2개도 heading으로 집계된 값이다.
- `repository-layout`의 과거 `PR #3` 상태 표시는 현재 package 상태로 바뀌었다.

원문 37개와 모든 행선지의 전체 표는 machine manifest의 `retention` 배열이 정본이다. 사람이
검토할 절 대응은 [D2 축약 보존 감사](./d2-content-parity-audit.md)와 각 D2 묶음 보고서에 있다.

## 3. 유실 위험을 다시 확인한 항목

다음처럼 구현·계약에 영향을 주는 정보는 현재 정본 또는 날짜별 기록에서 다시 확인했다.

| 정보 | 현재 위치 |
|---|---|
| 현재 UDP/IP/Hz와 아직 미적용인 현장 목표 | [운영 가이드](../operations/README.md), [bridge guide](../packages/asmc_bridge/README.md) |
| 차량·센서 pose, 허용 입력, 공개 자산과 채점 수치 | [대회 환경](../competition/README.md) |
| raw/result bag lineage와 종료된 Streaming GT 결과 | [rosbag guide](../operations/rosbag-evaluation.md), [transport record](../records/simulator/runtime-transport-2026-09.md) |
| ROS1 MD5, HD Map topic/frame/scope | [topic contract](../contracts/hd-map/topic-v1.md), [asmc_msgs guide](../packages/asmc_msgs/README.md) |
| 체크포인트·미션 권위와 provisional 금지 경계 | [competition metadata](../contracts/competition/kcity2025-metadata-v1.md) |
| HD Map 후보 선택과 geometry product 근거 | [HD Map records](../records/README.md), [HD Map guide](../packages/asmc_hd_map/README.md) |
| 데이터 palette·mask·split과 날짜별 모델 평가 | [dataset contract](../contracts/perception/dataset-labeling-split.md), [perception records](../records/README.md) |
| FSM/Frenet transaction과 legacy/production 경계 | [FSM/Frenet contract](../contracts/planning/fsm-frenet-v2.md), [Planning guide](../packages/planning-control/README.md) |

의도적으로 현재 가이드에 복원하지 않은 것은 개인 IDE 선호, 이미 지난 주간 일정, 보호 정책과
충돌하는 `main` 직푸시 Path A, 현재 tree에서 실행되지 않는 명령, 빠르게 낡는 파일 단위 담당
목록이다. 당시 문장 자체는 `577231b` Git blob과 archive에서 확인할 수 있다. 수치·계약·실험
실패 근거를 “오래됐다”는 이유만으로 버린 항목은 없다.

## 4. 외부 링크 호환 정책

현재 호환 stub은 17개다. D1의 주요 옛 경로 12개와 D2 통합 과정에서 생긴 중간 경로 5개를
포함한다. 각 stub은 본문을 복제하지 않고 새 정본 한 곳만 링크한다.

이 migration commit은 아직 `main`이나 후속 팀 배포에 포함되지 않았다. 따라서 “한 릴리스
호환” 유예가 시작되지도 않았고, D3에서 stub을 지우면 과거 PR·채팅·북마크가 즉시 깨진다.
다음 조건을 모두 만족할 때만 별도 PR에서 제거한다.

1. 문서 migration이 `main`에 병합된다.
2. 그 뒤 최소 한 번의 팀 release 또는 대회 배포가 끝난다.
3. 팀 저장소와 관리 중인 상위 workspace가 모두 새 정본을 직접 링크한다.
4. 제거할 옛 경로와 대체 경로를 PR에 명시한다.
5. 제거 뒤 migration checker와 Markdown link 검사가 통과한다.

관리 중인 상위 workspace `docs/`를 검사하면서 호환 stub 경유 링크와 이미 404였던 이동 경로를
발견했다. 28개 로컬 문서의 ASMC 링크를 새 정본으로 직접 교정했고, 옛 network 문서의 사라진
`§6.4` anchor는 [2026-09 transport record §2](../records/simulator/runtime-transport-2026-09.md#2-2026-09-09-v2-udpros-60초-기준선)로 연결했다. 이 작업은 팀 문서 본문을 로컬에 복제하지 않고
외부 참고 문서가 정확한 권위를 따라가게 한 것이다.

인터넷·메신저·과거 PR 본문처럼 저장소 밖의 모든 링크를 전수 검색할 수는 없다. 이 때문에
현재 17개 stub을 보수적으로 유지하며, machine manifest와 checker가 누락·오연결·내부 역참조를
막는다.

## 5. D3 삭제 판단

이번 단계에서 삭제한 문서는 없다.

- 17개 stub: 호환 유예 미충족으로 유지
- `archive/generated-msg/*.txt`: 현재 `.msg`와 semantic duplicate지만 외부 검토 export 소비가
  없다는 확인 전까지 유지
- `records/`와 `archive/`: 현재 실행 정본은 아니어도 실측·실패·결정 근거이므로 유지
- 빈 `docs/` 디렉터리: 0개

즉 D3의 결과는 “삭제할 수 있는 것처럼 보이는 파일을 바로 삭제”가 아니라, 중복 본문은 이미
stub으로 축소됐음을 확인하고 남은 삭제 후보의 gate를 자동화한 것이다. 다음 제거 PR 전까지
manifest의 `redirect_policy.state`는 `keep`으로 둔다.
