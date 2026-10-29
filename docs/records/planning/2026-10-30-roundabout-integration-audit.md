# 로터리 후속 개선 복원 및 LiDAR 적용 범위 감사

## 결정

2026-10-30 04:30 KST 체크포인트. 사용자 요청에 따라 PR #143 롤백 `530a43b0`을 취소하고, 윤태 후속 개선 `28a7246c` 및 원격 통합 `e2b2c6f0`을 수입했다. 생산 코드는 e2b2c6f0과 동일하다. 추가 변경은 합성 A/B 회귀와 이 감사 기록이다. 실제 시뮬레이터 주행·텔레포트·환경·배우 변경·LAN2LAN 설정 변경은 수행하지 않았다.

LiDAR는 **터널 범위 한정을 유지**한다. 이는 전구간 적용보다 주행 성능이 우수하다는 수락이 아니라, 일반 구간의 검증되지 않은 변경을 최소화하는 현재 운영 선택이다. tunnel_only_updates=true, 일반 MinPoints=5와 near ground=.30을 유지한다. 터널 sparse 및 벽/차량 처리는 현재 gate 정책대로 유지한다. 공통 LiDAR 코드 전체를 과거 버전으로 되돌렸다는 뜻은 아니다.

## 통합 감사

| 항목 | 확인 결과 | 해석 |
|---|---|---|
| PR143 원본 a7831212 → 최초 통합 f0a60047 | RoundaboutExecution/Merge, BehaviorInput 구현은 원본과 일치 | 확인한 경로에 잘못된 충돌 해결이나 후속 코드 누락 없음 |
| V2Cycle 차이 | 기존 #141 preparation_limit include와 제한 호출만 추가 | enabled/preparing/freshness 조건을 통과해야 작동. A448 경계는 s=1859.6416, 로터리 conflict s=885.76과 다름. 현장 잘못된 상태 신호를 과거 로그만으로 완전히 배제하지는 않음 |
| 차체 차이 | 팀원 원본1.900m 대비 #141 정본1.892m 및 boundary reserve.1345m | 앞선 명시적 제원 계약 보존. 이번에 임의 변경하지 않음 |
| 후속 윤태 수정 | 누락 객체를 후보별 현재 점유 모델로 검증, 인증 후보 우선, 단순 nominal 후보 때문에 대안 탐색 중단하지 않음 | 로컬 롤백본에는 없던 기능. 이번에 원격 구현 그대로 반영 |
| 처음 LiDAR 통합 | 표준 launch를 wall_vehicle profile로 선택하며 GENERAL MinPoints 5→3, near ground .30→.23, history/cleanup 설정까지 적용 | 터널 전용 변경으로만 이해하면 안 되는 범위 확장. 정상 구간 출력 변동 가능성을 확인 |
| 최신 터널 한정 수정 | 정상 구간 baseline 등가성 + 지도 구간 진입/GPS 소실/이탈 gate 회귀 통과 | GPS 소실 시 기존 arm 유지; GPS 없는 임의 시작은 arm하지 않음. 유효 GPS 복귀 전 위치 오차를 독립적으로 해결하는 기능은 아님 |
| 콘솔 배포 | 기존 observer 시작 revision이 남아 guard 거절 발생 | 앞선 배포 검증 누락. 이번에는 observer 재시작 후 실제 guard reuse 확인을 포함 |

### 시간 간격 설명 정정

`planner.merge.min_front_time_gap`은 PR143의 .6, 롤백 시1.0이지만 실제 로터리 후보 검사에는 **`roundabout_merge.min_front_time_gap_s=.7`이 우선**한다. 따라서 앞선 “로터리 전방1.0으로 복원” 안내는 실효값 설명으로 부정확했다. 실제 후방 값은 공통 merge 설정에서 가져와 현재1.0, 롤백 때2.0이다. 이번에는 윤태 원본 설정을 유지하며 추가 튜닝하지 않는다. 공통 merge 값 변경은 다른 merge 소비자에도 영향을 줄 수 있으므로 로터리 전용 파라미터로 설명하지 않는다.

## 동일 입력 오프라인 실험

새 `Pipeline.GlobalTunnelTuningChangesNormalRoadCandidateAdmission`은 센서 좌표계 평지와 동일 점군을 두 설정에 입력한다. 일반 경로이며 tunnel.ran=false를 검사한다.

| 합성 장면 | 기존 일반5점/.30m | 초기 전구간3점/.23m |
|---|---:|---:|
| 지면에서 충분히 떨어진 3점 성분 | ground kept3, cluster0 | ground kept3, cluster1 |
| 지면 위27cm, 6점 성분 | ground kept0, cluster0 | ground kept6, cluster1 |

이는 클러스터 생성 단계 변화이며 실제 차량/노이즈 분류 정확도나 최종 planner 입력 수락을 뜻하지 않는다. 기존 OBB·시간 확인·추적 필터가 남는다. 이 자료는 합성 분석 fixture이며 독립 실차 시험 세트가 아니다. 튀는 실제 OBB가 로터리 정지 원인이었는지는 충돌 직전 시간열이 없어 확정하지 못했다.

## 검증 결과와 한계

- Release production planner/LOCAL/localization 빌드 성공.
- 로터리 transaction/candidate/execution/prediction/departure_bounds 5종 통과. 후속 누락 차량·부분 관측·후보별 재검증 포함.
- LiDAR pipeline19개(기존18+새 A/B1), launch 계약6개 및 세 옵션 graph 통과.
- target_speed_rate_limit, jamming_policy, jamming_wall_scope, local_planner, shadow_localization15 통과.
- 주행 시뮬/새 bag 재생/전구간 수락/원격 최신 CI는 수행하지 않음. 기존 충돌 직전 객체/계획 시간열이 부족하여 후방 추돌의 단일 원인을 확정하지 않음.
- 기록: D `data/runtime/roundabout-integration-audit-20261030-r01`. 소스·실행파일·프로필·observer identity는 manifest에 고정한다.
- rollback: 직전530a43b0과 별도 보존 overlay를 함께 사용해야 한다. 소스만 변경한 채 기존 실행파일을 검증본으로 표시하지 않는다. 원격 PR141의 후속 구현을 삭제하지 않는다.
