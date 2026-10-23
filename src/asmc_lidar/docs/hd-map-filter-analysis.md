# A1 — OBB 기반 HD맵 전달 필터 분석

2026-10-24. 담당 안승현. PR #104 병합 main `6927fe9d`를 기준으로 한다.
이번 단위는 기존 필터 감사와 **삭제 없는 오프라인 판정 비교**다. 운영 Main·판제 입력,
기본 ROI·검출·OBB·tracker·fusion을 변경하지 않는다. 점군 HD맵 ROI는 보류한다.

## 실제 호출 경로와 소유권

| 경로/기능 | 실제 동작·기본값 | 다음 이관의 소유권 |
|---|---|---|
| Main `PlannerObjectRosPublisher::Publish` | map 객체 → road/known-static/structure → relevance → 필요 시 OBB clipping → `/last_lidar` | 지도/측정 품질 판정은 인지. 원본 추적 상태와 최종 전달을 분리 |
| `planning_inputs` 기본 `validated_lidar` | `/last_lidar` + 같은 source stamp의 `/lidar/motion_debug`에서 ID/최종 keep/운동 상태를 대조한 뒤 변환. road 필터를 다시 실행하지 않음 | 현행 wire gate 유지. 다음 공용 출력에서는 진단 JSON 의존 제거를 별도 검토 |
| `planning_inputs` 선택 `main_raw` | private motion classifier → road/known-static/structure → clipping → `/planning/objects` | Main과 일부 구현 중복. 선택 입력에 따라 정책·기하가 달라짐 |
| road footprint | threshold 0.15, margin 0m, supersample 5. 보행자 보존. motion이면 0.5초×6회 map 속도 예측으로 진입 검사 | 인지 관련성. 새 후보는 8코너 footprint 사용·OBB clipping 금지 |
| known-static | UNKNOWN·확정 클래스 예외, cluster center와 catalog 거리 ≤2.5m. 검증 영역 `[-35,15]×[108,180]`, `[-107,15]×[175,345]` | 인지. catalog ID는 tracker ID의 영구 정체성으로 해석하지 않음 |
| structure exclusion | UNKNOWN·비이동 근거, 지정 zone과 높이≥2.5m 또는 하단 z≥2m. 일부 zone은 형상 조건 없음 | 인지. 지도 전체의 일반 장애물 삭제 규칙으로 확대하지 않음 |
| Main relevance | 높이 0.10m·지면 근거·3회 확인·5회 release·20frame 이력 정리·camera 의미 예외, 자차 속도 기반 emergency 거리 override | 측정 근거와 행동 정책이 섞임. emergency/TTC·추종·회피를 통째로 인지에 옮기지 않음 |

호출 근거: Main `src/PlannerInterface/PlannerObjectRosPublisher.cpp`, `main.launch`;
`planning_inputs/src/objects_node.cpp`, `src/filters/PlanningObjectFilter.cpp`,
`scripts/tracked_wire_bridge.py`, `launch/objects.launch`.
`main_raw`는 필터 전 데이터가 자동으로 생긴다는 뜻이 아니다. 같은 `/last_lidar`를 입력하면
이미 Main이 삭제한 객체는 복원되지 않고 추가 필터가 적용될 수 있다.

## 지도·예외 구간·위치 유효성

- 기존 PNG는 흰색이 도로, v6 파생 OccupancyGrid는 0이 도로다. 원점/map ID가 같아도
  형상은 다르며 v6 vector가 정본이다. 비교 도구의 겹침 수치는 파생 grid 기반 근사로 표시한다.
- 정적 장애물 `A213/A222/A220`, 동적 장애물 `A202/A728/A599/A207/A205/A315`,
  로터리 `A148/A146/A151`은 현재 팀 metadata의 예외 링크다. v6의 maneuver/support
  link도 함께 보호한다. metadata의 mission polygon은 비어 있으므로 외부 접근 영역의
  완전한 검수가 끝났다고 표시하지 않는다. 판제 설정과 차이는 결과에서 별도 보고한다.
- 일반 구간 밖에서 객체가 들어오지 않는다는 것은 사용자 제공 대회 운영 전제다.
  Ego 링크만으로 예외를 켜지 않고, 객체 footprint가 예외 공간에 접근하면 보존한다.
- GPS 5Hz/pose age≤300ms는 위치 정확도 보장이 아니다. 새 후보의 제외에는 명시적으로
  검증된 위치 오차 범위와 구간 검토 근거가 필요하다. 근거가 없는 기존 trace는 판단 불가다.
  IMU 자세만으로 병진 이동이 해결됐다고 가정하지 않는다. GPS 무효 시 local은 보존한다.

## 비교 도구와 근거

구현·실행·검증 결과를 이 절에 추가한다. 기존 실험 trace의 원본 SHA와 당시 설정을
보존하고, 재사용 자료를 새 실시간 시험 또는 독립 GT 평가로 표시하지 않는다.

## 다음 구현 경계

필터는 OBB·추적·camera fusion 뒤의 전달 판정에만 연결한다. tracker에 삭제를 피드백하지
않고 원본 중심·8코너·속도·ID를 보존한다. 정상 빈 결과와 센서/pose 무효를 구분한다.
실제 제거·새 ROS 출력·판제 전환·기존 중복 코드 삭제는 A1 결과 검토 이후 별도 작업이다.
