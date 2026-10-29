# 로터리 native / 일반 emergency 경계 수정

기준 cf5feccc, 구현9baadf38, 테스트 등록0745f6ef. 일반 FOLLOW의8m 선호 간격을 뺀 요구감속만으로 검증된 로터리 MERGE를 취소하는 경로를 한정했다. FeatureBuilder가 기존 TTC/예측TTC 조건을 별도 내부 근거로 제공하고, BehaviorInput은 아래 조건을 모두 만족할 때만 요구감속 surrogate의 emergency를 native 경로 검사에 맡긴다.

- 현재 요청이 ROUNDABOUT MERGE, 동일 command ID·수락된 COMMITTED/EXECUTING.
- native plan_valid 및 LOCKED 경로, feedback 나이0–150ms.
- 객체 freshness·유효 ego, feedback stamp가 현재 객체 stamp 이후.
- 실제 MPC binding token 일치·linked·planning_checked, 실행 상태 나이0–150ms.
- 기존 TTC/예측TTC 위험이 없음.

조건 하나라도 실패하면 기존 판단으로 돌아간다. native의 실제 차체 swept collision, pedestrian/dynamic 위험, 긴급 TTC와 watchdog, 일반 주행·합류는 변경하지 않았다. 기존 emergency latch를 강제로 해제하지 않는다. 원래 required_deceleration 수치는 진단에 보존하며 실제 억제 시 native swept path 소유권 로그를 출력한다.

검증: planner 재빌드, Behavior66개 통과, standalone 경계 검사(정상/다른 모드/명령불일치/실패경로/stale/역행시각/TTC/실행 미연결) 통과. 경계 검사는 CMake/CTest에도 등록했으나 이번 마감에서는 직접 g++로 실행한 결과다.

같은 A304 s636·NPC seed656에서 새 바이너리 실주행:41.35초,s964.75까지 통과. 로터리 내부s888–935 최저9.11km/h,1km/h 미만0표본,명시적 타 객체 접촉0표본. 이번 주행에서는 native 소유권의 실제 emergency 억제 로그가0건이므로 이전 실패 조건 자체가 재발하지 않은 것이다. **통과가 수정 효과의 실주행 인과 증명이나 후방 충돌 완전 해결을 의미하지 않는다.** 다음에는 동일 실패 bag/실행 binding 근거로 개입 시점 대조가 필요하다. A304 객체 입력 공백은 별도 문제로 남는다.

자료: D:/morai dataset/shahn03/artifacts/roundabout-native-gap-20261030-r01/ (runner·bag·원본JSON·분석·속도HTML·검사로그·바이너리SHA). 실제 실행은9baadf38이고 이후는 테스트 등록/문서 변경이다. 자체 autonomy/NPC/bag 종료, 차체hold/P; master11373·bridge·인지·웹8089/8097 유지.

복구: git revert 9baadf38 후 planner 재빌드. main 병합/강제 push 없음. 비교 컨테이너에서 기존 기본 localization+LOCAL 실행 명령을 그대로 사용한다.
