# 로컬 Codex 작업: 후보 한 개의 실제 재생 검증

목적: `lidar_assignment_allow_unmatched`만 바꿔, 잘 맞는 차량 연결을 보존하는 효과가
전체 ID/속도/박스/누락의 개선으로 이어지는지 판별한다. 이번 작업에서 새 알고리즘,
threshold sweep, 기각 옵션 결합, 임의 gate 완화를 하지 않는다.

1. 사용자 작업 상태를 먼저 기록한다. 원래 checkout은 전환/초기화하지 않는다.
   원격 `exp/lidar-optional-assignment-20261024`를 fetch하고 별도 worktree에서 고정 SHA를
   기록한다. AGENTS.md, PROJECT_STATUS.md, REPRODUCE.md, 이 폴더 README.md를 읽는다.
   기준은 A+B+EMA ON이며 Yangtest/main으로 바꾸지 않는다.
2. 해당 SHA로 새 build prefix에 -O2 빌드한다. 명령·dependency·binary/library/config hash와
   build 로그를 보존한다. 기존 실패 시험은 숨기지 말고 기존 결함/후보 회귀를 구분한다.
   portable 검증과 비교 집계 시험도 실행한다. 컴파일 연결 오류만 있으면 최소 수정 후
   새 SHA를 기록하고 다시 빌드한다. 연구 로직 변경은 하지 않는다.
3. README의 compare.py로 워밍업 포함 scan0..1155 OFF/ON을 실행한다.
   OFF 1156/1156 정확 일치가 깨지면 ON을 해석하지 말고 먼저 원인을 보고한다.
   old build SHA를 실제 실행 SHA처럼 보고하지 않는다.
4. 1149만 보지 말고 최초로 ON/OFF가 달라진 scan부터 association→KF→lifecycle→발행을
   추적한다. 새 birth 때문에 숫자 ID가 달라질 수 있으므로 ON에서도 반드시 ID375여야
   한다는 검사를 만들지 않는다. GT는 평가에서만 동일 객체 대응을 확인하는 데 사용한다.
   scan1090 초기 가짜 속도와 이후 차량으로의 상태 인계를 별개로 보고한다.
5. prefix는 1155에서 끝나므로 이것만으로 승격/채택하지 않는다. 이미 보유한 전체 actual
   lap(2630scan)을 같은 aligned GT와 인과 입력으로 OFF/ON 재생한다. 기존 전체 재생기를
   재사용하되 새 옵션 하나만 추가하고 실제 적용 trace를 확인한다. 입력/GT/계약을 바꾸지 않는다.
   실제 데이터와 emulated scen93/21/102 회귀 수치를 섞지 않는다.
6. 전체·공통 속도 MAE/p95/p99/max, GT별 꼬리, IDSW/fragmentation, box 중심/크기/π-yaw,
   동일 ID 및 ID변경 경계 흔들림, TP/FN/FP와 coverage, update/coast/birth/retire 비율,
   overmerge/split을 함께 보고한다. 속도 유효 표본이 줄어서 좋아지는 결과는 성공이 아니다.
   반응 지연은 기존 v2.2 고정 support/동일 timestamp evaluator를 사용한다.
   partial GT 때문에 FP를 완전한 precision 근거로 삼지 않는다.
7. 회귀가 분명하면 기본 OFF 유지·기각 이유를 보고하고 임의 대안을 추가하지 않는다.
   유망하면 기존 84개/stress와 -O2 실제 Main 15Hz 비교로 진행하되, LiDAR-only와
   camera/GPS/map/planner 전체 부하를 구분한다. 새 유효 정지 run은 별도 사용자 PC 검증이다.
   삭제된 static_bag와 파생 지표는 복원/사용하지 않는다.
8. 결과는 별도 results branch에 REPORT_KO.md, 실행 manifest, 고정 계약 결과표, 공통 표본,
   객체별 꼬리, 최초 차이 trace 요약, 시험 로그를 커밋한다. 대형 bag/점군/비밀값/개인 절대경로는
   커밋하지 않는다. 원본 파일과 실행 코드 hash를 기록한다. main merge/운영 설정 변경은 하지 않는다.

최종 보고: 실제 실행 SHA, OFF 동등성, 원인 시험과 폐루프의 차이, 전체/공통 분모,
개선/회귀/미검증, 결과 branch와 commit. 채택을 미리 선언하지 않는다.
