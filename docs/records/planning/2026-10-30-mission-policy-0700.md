# 07:00 정적 미션·로터리 의미 정책

기준 `eb30fb18`, 구현 `c4b8ccbd`. 사용자 승인한 대회 미션 전제를 판제 소비 경계에 한정했다. LiDAR 출력·추적·속도·센서·네트워크·HD맵 정본은 변경하지 않았다.

## 정적 장애물 미션

이미 있는 `StaticMissionPolicy`가 HD맵 competition metadata와 SHA 검증된 공식 경로를 로딩한다. 중복 지도 구독 노드는 추가하지 않았다. A213 중간부터 A220 끝, 공식 station276.664444175–365.775740557 안에 **유효한 자차와 객체 중심이 모두** 있어야 한다. 자차 방향·횡거리·map frame 검사도 유지한다.

이전에는 vehicle/person의 type만 UNKNOWN으로 바꾸고 DYNAMIC 상태를 유지했으므로 속도 튐이 동적 예측으로 이어질 수 있었다. 이제 Behavior와 Frenet이 공유하는 private copy에서 모든 해당 객체를 STATIC_OBSTACLE 정책으로 소비하고, 유효 속도에는 LOW_THREAT 정적 가정을 적용한다. vx/vy·카메라 클래스/그룹·ID·OBB·원본 메시지는 보존한다. 이 LOW_THREAT는 **미션 기반 계획 가정이며 인지가 실제 정지를 확인했다는 뜻이 아니다**. 속도 무효는 UNKNOWN/무효를 유지한다. 잘못된 motion/velocity 조합은 정적 가정으로 유효화하지 않는다. 구간을 나오면 같은 원본 관측도 다시 원래 정책으로 복구한다.

`/planning/static_mission_policy`에 original/effective type과 motion, 객체별 mission_prior를 기록한다. 기존 launch의 `static_obstacle_unknown` 이름은 호환 유지했지만 활성화 시 위 정책을 포함한다. 전역 속도 유효성 계약이나 메시지 MD5를 변경하지 않았다. 실험 전제가 다른 대회·일반 도로에서는 이 정책을 그대로 사용하면 안 된다.

## 로터리

검증된 ROUNDABOUT 명령의 native 경로·점유 예측 입력에서 확정 `static_obstacle`/`dynamic_obstacle` 그룹을 제외한다. vehicle·unknown·빈 분류·새로운 미인식 문자열은 유지한다. 판제 enum UNKNOWN과 퓨전 그룹 unknown을 혼동하지 않는다. 선택 범위는 로터리 경로 검사이며 일반 구간·LOCAL·LiDAR tracker에는 적용하지 않는다. 기존 Behavior의 독립 emergency 판단을 무조건 해제하지 않는다. 따라서 이 변경이 모든 정지 조건을 제거한다는 의미는 아니다.

## 가드레일 앞 감속 조사

06:55 trial1의 s845–875에서 Behavior 목표25km/h, 일반 상한50→30km/h, avoidance_object_id=-1이었다. 실제 저속과 함께 Frenet `departure_path_rejected`가 기록됐다. A153 선행57.5 상한 변경 때문이라고 볼 근거는 없다. 다만 경로 탈락이 도로 형상·운동학·가드레일 점유 중 무엇 때문인지는 현재 기록으로 확정하지 못했다. 회전교차로 접근 제한/경로 검사를 없애거나 속도만 올리지 않았다. s845 이전 가드레일 구간은 이번 재시험 범위 밖이다.

## 검증과 실행

- 정적 미션 synthetic 회귀: 범위 안/밖·양 경계·반대 heading·pose 무효·base frame·속도 무효/잘못된 조합·원본 보존·구간 이탈 시 복원 통과.
- 로터리 그룹 회귀: vehicle/unknown/빈 문자열/미인식 문자열 보존, 두 확정 비차량 그룹 제외 통과.
- 기존 roundabout execution102, candidate38, transaction33 PASS, 재빌드한 실행파일 사용.
- 동일 seed656·출발s860로 새 실주행: s962.25까지25.26초, 내부s888–935 최저9.88km/h,1km/h 미만 표본0·접촉 표본0. NPC는5대. ROS/NPC 시각이 이전 시험과 같지는 않아 속도 개선 A/B라고 해석하지 않는다.
- 정적 미션에서 속도 튐을 주입한 실주행은 하지 않았다. 합성 계약 통과와 로터리 대표 주행만 검증했다. 표본약0.45초로 전 구간 안전 수락이 아니다.

자료: `D:/morai dataset/shahn03/artifacts/mission-policy-0700-20261030-r01/` (컨테이너 `/validation/mission-policy-0700-20261030-r01/`). runner·배치·실주행JSONL·summary·빌드/검사 로그·바이너리SHA를 보존했다.

compare 컨테이너에서 `source /root/jamming-env.sh`, `export ROS_MASTER_URI=http://127.0.0.1:11373` 후 `roslaunch asmc_planner autonomy_no_bridges_v2.launch dashboard:=false`. localization+LOCAL 기본은 유지한다. 판제8089, LiDAR8097. 시험 autonomy/NPC만 종료했고 차체hold/P, 입력과 웹은 유지한다. 새 설정은 다음 planner 실행부터 적용한다.

복구는 `git revert c4b8ccbd` 후 planner 재빌드. 팀원 이력을 reset하지 않는다. main 병합은 하지 않는다.
