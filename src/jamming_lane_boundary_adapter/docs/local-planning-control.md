# 음영 Planning/Control 연결

ROS 패키지 이름은 `lane_boundary_adapter`이며 현재 autonomy launch가 이 패키지의
family→영상 경계→전방 bbox 차로 분류→차선변경/추종 경로를 실행한다.
일반 MPC는 `/ctrl_cmd/normal`, 음영 추종기는 `/ctrl_cmd/jamming`을 제공하고,
ControlSelector만 `/ctrl_cmd`를 발행한다. 별도 차선변경 제어 launch를 추가하지 않는다.

```bash
./scripts/build_ws.sh -j2 -l2 -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch asmc_planner autonomy_no_bridges_v2.launch
```

bridge·camera·LiDAR는 각각 실행한다. 현재 일반 객체 입력은
`/perception/lidar/planning_observations_map`이며 개발 GT `/Object_topic` adapter가 아니다.
GPS 무효 map 입력은 보류한다. local LiDAR 객체는 제공되지만 현재 음영 제어기의 회피 입력은 아니다.

읽기 전용 boundary monitor와 실제 제어 노드를 구별한다. 기존 private 메시지와 내부
영상 투영은 현재 실행에 필요하다. 폴더를 `asmc_planner/src/jamming`으로 흡수하는 변경과
local 통로·기존 planner/controller 재사용은 **다음 단위 제안**이며 아직 실행을 바꾸지 않았다.

[현재 구현·정지 원인·통합 계획](../../../docs/architecture/shadow-planning-refactoring.md),
[카메라·BBOX 차선변경](bbox-lane-change.md), [판제 실행](../../../docs/operations/commands.md)을 따른다.
판제 웹 `dashboard.launch`의 음영 local 화면은 실제 선택 경로와 입력을 관측할 뿐,
새 경로를 계산하거나 차량을 제어하지 않는다.
