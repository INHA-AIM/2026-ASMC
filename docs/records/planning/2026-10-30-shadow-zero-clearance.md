# 음영구간 추가 충돌 여유 0 실험

사용자 요청에 따라 LOCAL/localization 전용 고정 객체·노면 여유를 0m로 설정한다. 시작 revision은 `398c09a6`이며 NORMAL·로터리의 `params.yaml`은 변경하지 않는다.

- `jamming_motion.yaml`과 LOCAL 설정의 clearance/road clearance는 0이다.
- localization은 같은 설정을 읽고, private native Frenet 인스턴스의 탐색과 최종 sweep에 동일 여유를 적용한다. 기존 native 0.45m와 LOCAL 0.30m 차이를 제거한다.
- 0 설정에서는 LOCAL 속도 비례 추가 이격 및 native 시간 비례 margin growth도 적용하지 않는다.
- 실제 차체와 OBB의 겹침, 도로 외곽 검사, 격자 이산화 보정, 관측 age/위치 불확실성, 움직이는 객체의 예상 점유 공간은 유지한다. 따라서 모든 장애물을 무시하거나 항상 전진하는 기능은 아니다.
- preferred/maneuver clearance는 비용·속도 선호이며 제거하지 않는다. 이는 고정 충돌 불허 여유와 다르다.

실행 중 프로세스는 설정을 시작 시 읽는다. 주행 중 프로세스는 재시작하지 않았으며, 현재 주행은 새 설정 검증으로 계산하지 않는다. 새 빌드 후 autonomy를 다음에 시작할 때 적용된다.

검증: 별도 회귀에서 zero clearance의 분리된 차체 통과·실제 겹침 거부, NORMAL 인스턴스 변경 차단을 검사한다. 최종 빌드·시험 결과는 아래에 기록한다.

복구: 이 변경 커밋을 revert하고 planner를 재빌드한 다음 autonomy를 재시작한다. ROS param만 바꿔 실행 중 native 설정이 바뀐 것으로 해석하지 않는다.

## 검사 결과

- container `/build`에서 LOCAL/localization 노드와 두 회귀 실행파일 빌드 성공.
- `local_planner_test`: 성공. 0m 설정의 실제 차체 겹침 거부 포함.
- `shadow_localization_test`: 성공. NORMAL override 차단·0 설정·잘못된 음수 거부 포함.
- 최초 후보는 기존 nonzero 프로필까지 native margin을 맞추면서 ideal tracking 복귀 검사가 실패했다. override를 명시적인 **0 설정에만** 제한한 뒤 기존 회귀가 통과했다. 해당 최초 후보는 채택하지 않았다.
- 일반 Frenet 설정 파일 diff 없음. 주행 중인 노드 재시작 및 0m 폐루프 시험은 수행하지 않았다.
- 빌드/시험 로그: 컨테이너 `/tmp/shadow-clearance-{build,rebuild,native-test,local-test}.log` (임시 검사 로그).
