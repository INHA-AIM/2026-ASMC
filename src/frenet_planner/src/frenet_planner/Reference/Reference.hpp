#ifndef FRENET_REF_LINE_HPP
#define FRENET_REF_LINE_HPP

#include <string>
#include <vector>

// =========================================================
// Reference line 구축/조회 파라미터. params.yaml의 planner.reference에서
// 로드하며, C++ fallback default를 두지 않는다(Parameter Single-Source).
// =========================================================
struct ReferenceConfig {
    // [m] 폐루프(닫힌 트랙) 판정 허용오차 - 원본 waypoint 간격보다 훨씬
    // 작게 잡아야 우연히 가까운 open path를 closed로 오판정하지 않는다.
    double closure_tolerance_m;
    // [m] 국소 급커브 완화 1단계(이동평균 스무딩)의 전체 윈도우 폭.
    double smoothing_window_m;
    // [m] 스무딩 뒤 균일 재샘플링 간격(원본 waypoint 간격과 동일하게).
    double resample_ds;
    // 잔여 곡률 초과분을 국소적으로 이완(라플라시안 완화)시키는 최대 반복 횟수.
    int relax_max_iter;
    // 초과 지점을 양옆 중점 쪽으로 당기는 비율(0-1).
    double relax_step;
    // [m] FindClosestS의 ego 위치추정용 기본 탐색창. 정지/회피 목표 투영처럼
    // ego보다 훨씬 앞선 지점을 찾는 호출은 각자의 전용 탐색창을 명시적으로
    // 넘긴다 - 이 값은 ego localization 전용이다.
    double ego_projection_search_window;
};

// =========================================================
// Center line 한 점의 정보
// Global path Waypoint를 arc length로 매개화한 결과
// =========================================================

struct RefPoint {
    double x, y;
    double s;
    double theta;       // heading
    double kappa;       // 곡률
    double d_kappa;     // 곡률 미분한 거
};

// =========================================================
// Center line 전체
//
// closed_loop / length는 Reference 계층이 소유하는 topology 계약이다.
// FindClosestS/Interpolate/CurveSpeed 등 참조선을 조회하는 모든 코드는
// points.back().s를 직접 재추정하지 않고 이 두 필드를 통해 topology를
// 판단한다("one policy, one authoritative owner").
//
// closed_loop == true (닫힌 트랙):
//   points.front() = 물리적 loop 시작점, s = 0
//   points.back()  = 그 시작점의 복제(닫힘점), s = length
//   조회(Interpolate/FindClosestS/CurveSpeed lookahead)는 [0, length)로
//   순환(wrap)한다.
//
// closed_loop == false (열린 경로):
//   points.back()에 points.front()를 복제하지 않는다.
//   조회는 [points.front().s, points.back().s] 범위로 clamp된다 - wrap하지
//   않는다.
// =========================================================

struct RefLine {

    std::vector<RefPoint> points;

    bool closed_loop = false;
    double length = 0.0;  // 사용 가능한 arc length. closed_loop이면 닫힘 구간 포함.

};

// =========================================================
// 생성
// Waypoints (x, y Array) 받아 RefLine 구성
// heading, kappa, d_kappa 수치 미분으로 계산
// =========================================================

// max_curvature는 ReferenceConfig의 필드가 아니라 별도 인자로 받는다 - 이는
// planner.kinematic_limits.max_curvature와 동일한 물리적 차량 능력이며(같은
// 값을 YAML에 중복 정의하지 않는다), FrenetPlanner::Init()이 로드된
// KinematicLimits에서 그대로 넘겨준다.
RefLine BuildRefLine(const std::vector<double>& wx,
                     const std::vector<double>& wy,
                     const ReferenceConfig& config,
                     double max_curvature);

// =========================================================
// s* 탐색 (Cartesian -> Frenet First Step)
// 차량 위치 (x, y)에서 Center Line 위 가장 가까운 s 반환
// Newton Iteration으로 수치 풀이
// =========================================================

double FindClosestS(const RefLine& ref, double x, double y,
                     const double* s_hint, double window);

// =========================================================
// 보간, 임의의 s에서 RefPoint 반환(선형 보간)
// frenet_converter가 heading_r, kappa_r, d_kappa_r을 가져갈 때 사용
// =========================================================

RefPoint Interpolate(const RefLine& ref, double s);

// =========================================================
// waypoint 파일("x y" 또는 "x,y") -> RefLine
// =========================================================
bool LoadReferenceLine(const std::string& path, RefLine& out_ref,
                       const ReferenceConfig& config, double max_curvature);

#endif