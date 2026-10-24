# 연결 보류를 허용하는 매칭 후보

**기본 OFF / 운영 미채택 / ROS 전체 빌드와 폐루프 재생 미검증.**
기준: handoff `6c5dba83333c908a8d0d265d64d0f79754523291`, P0 유래 A+B+EMA ON.

## 원인과 변경

baseline scan1149에서 차량을 따라가던 track375는 차량 점군 det14에 비용
0.61019522로 잘 맞았다. 그런데 기존 Hungarian은 353→14(2.21594644),
375→13(2.36760306)을 선택했다. det13은 작은 조각이었다.
전체 34×16 비용 행렬에서도 재현된다. 솔버 계산 오류가 아니라
"이번에는 연결하지 않기"의 비용을 목적함수에 포함하지 않은 설계 문제다.

이번 후보의 목적함수:

    sum(연결 비용) + λ/2 × (미연결 트랙 수 + 미연결 검출 수)

λ=기존 COST_THRESHOLD=3을 사용한다. 개별 연결의 기존 게이트는 바꾸지 않는다.
위 두 연결 비용은 4.58354950. 375→14 하나와 양쪽 미연결 비용 합계는
3.61019522이므로, 새 방식은 375→14와 353 미연결을 고른다.
이는 해당 프레임에서 기준선 상태를 고정한 계산이다. 이후 결과의 보장은 아니다.
**scan1090의 최초 가짜 속도는 고정 상태 비교에서 해결하지 않는다.**

정사각 확장 행렬: 좌상단 기존 비용, 우상단·좌하단 λ/2, 우하단 0.
기존 C++ Hungarian/Munkres 솔버를 재사용한다. λ를 기존 게이트에서 가져오는 것은
이번 설계 가정이며, 비용은 보정된 확률/우도가 아니다. 추가 sweep은 하지 않았다.

외부 원문 구현과 대조한 근거:

- [gatagat/lap, `_lapjv_cpp/_lapjv.pyx`, `lapjv`](https://github.com/gatagat/lap/blob/e29341d9e0ec8c3a1f11f239eb65a31973f4a0ab/_lapjv_cpp/_lapjv.pyx)
  commit `e29341d9e0ec8c3a1f11f239eb65a31973f4a0ab`.
  finite cost_limit에서 N+M 확장 및 cost_limit/2 미연결 비용을 사용한다.
- [ByteTrack 공식 `yolox/tracker/matching.py`, `linear_assignment`](https://github.com/FoundationVision/ByteTrack/blob/d1bf0191adff59bc8fcfeaa0b33d3d1642552a99/yolox/tracker/matching.py)
  commit `d1bf0191adff59bc8fcfeaa0b33d3d1642552a99`.
  `lap.lapjv(..., extend_cost=True, cost_limit=thresh)`를 사용한다.
  논문: [ByteTrack: Multi-Object Tracking by Associating Every Detection Box](https://arxiv.org/abs/2110.06864).

외부 코드를 복사하거나 의존성을 추가하지 않았다. ByteTrack 전체나 신경망을
이식한 것이 아니라 같은 미연결 목적함수를 기존 솔버로 구현했다.

## 범위와 위험

`lidar_assignment_allow_unmatched=false`가 기본값이다.
A+B, EMA, Q/R, 점 평균 KF 관측, 클러스터/병합, 기존 연결 비용·게이트,
coast/retire/birth 및 발행 정책은 유지한다. GT/클래스/크기 prior는 없다.
미연결 검출은 삭제하지 않고 기존 신규 ID 경로로 간다. 미연결 이전 track에는
기존 retire/coast 정책이 적용된다. 따라서 ghost·신규 ID·끊김이 늘 가능성이 있다.
부분 관측의 전체 치수, 원시 yaw 오류, 최초 가짜 속도까지 해결했다고 주장하지 않는다.

## 이번 환경에서 실행한 검증

- 실제 C++ HungarianAlg.cpp와 후보 헤더를 컴파일했다. Global.hpp 테스트 shim은
  vector/cstdlib/cfloat include만 제공한다. ROS 전체 빌드 시험은 아니다.
- 무작위·경계 607행렬: SciPy 독립 솔버의 목적함수와 비교. 이 중 107개는
  모든 가능한 부분 매칭을 열거한 해와도 비교했다.
- 저장된 71프레임: 이전 track 행의 누락이 없는지 출력과 대조했고,
  기존 accepted assignment를 71/71 재현했다. 후보는 6프레임에서 변경된다.
- matched edge 총수 1,054→1,048은 속도/IDSW/recall 개선 수치가 아니다.
- ASan/UBSan 같은 검사를 통과했다. `ASAN_OPTIONS=detect_leaks=0`으로 실행했다.
  LeakSanitizer는 앞선 환경에서 /proc 접근 실패가 있었으며 이번 결과도 leak 검증이 아니다.
- 비교 집계 시험 6개 통과: 표본 누락, 중복, GT 불일치, NaN, 초기화, 동일 출력.
- 기존 솔버의 indentation/unused-path 관련 compiler warning은 로그에 보존했다.
  이를 새 후보 오류로 세거나, 기존 코드를 임의 수정하지 않았다.

증거: `evidence/portable/RESULT.json`, `evidence/asan/RESULT.json`,
`evidence/comparison_tests.log`. trace 비용은 9자리 유효숫자이며 완전한 원래
float 행렬이라고 주장하지 않는다. 이후 KF/ID 상태에 되먹임하지 않은 **고정 상태 진단**이다.

## 실행

ROS 없이 후보 계산만 확인:

```bash
python3 -B handoff/lidar/optional_assignment/verify.py --out /tmp/optional-check
python3 -B handoff/lidar/optional_assignment/test_compare.py
```

사용자 PC의 별도 worktree와 새 build prefix에서 REPRODUCE.md대로 빌드한 후:

```bash
export LIDAR_SETUP="/새로_빌드한/devel/setup.bash"
python3 -B handoff/lidar/optional_assignment/compare.py \
  --data "$HOME/asmc-repro/actual_prefix_0_1155_v1" \
  --out "$HOME/asmc-repro/optional-$(date +%s)"
```

새 폴더를 사용한다. OFF는 기존 1,156출력과 timing 제외 tolerance=0으로 같아야
ON을 실행한다. 같은 binary/library/data hash를 검사한다. trace의 assignment_policy로
실제 옵션을 확인하므로 예전 binary로 실행하면 중단한다. 각 arm은 자신의 points.bin으로
평가한다. 전체 표본과 공통 GT·시각 표본, 객체별 꼬리 오차, 기존 평가 계약 지표를 저장한다.
COMPLETED_NOT_ACCEPTED는 실행 완료이지 채택 판정이 아니다.

run.py의 과거 고정 build SHA는 제거했다. 실제 checkout SHA/dirty 여부와 실행 파일 hash를
기록한다. checkout SHA만으로 build 출처가 증명되지는 않으므로 clean build 로그를 보존한다.
전체 ROS 빌드/재생은 이 환경에 의존성이 없어 실행하지 못했다.
후속 작업은 LOCAL_VALIDATION_TASK.md를 따른다. 삭제한 static_bag는 사용하지 않는다.
