#include "frenet_planner/Reference/Reference.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace {

double NormalizeAngleDiff(double dtheta) {
    while (dtheta > M_PI)  dtheta -= 2.0 * M_PI;
    while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
    return dtheta;
}

std::vector<double> CumulativeArcLength(const std::vector<double>& x, const std::vector<double>& y) {
    std::vector<double> s(x.size());
    s[0] = 0.0;
    for (size_t i = 1; i < x.size(); i++) {
        double dx = x[i] - x[i - 1];
        double dy = y[i] - y[i - 1];
        s[i] = s[i - 1] + std::sqrt(dx * dx + dy * dy);
    }
    return s;
}

// 원본 waypoint의 물리적 첫점/끝점 거리로 폐루프 여부를 판정한다. 대회
// global path처럼 마지막 점이 첫점을 그대로 복제해 명시적으로 닫혀 있는
// 입력을 감지하는 것이 목적이므로, 허용 오차(config.closure_tolerance_m)는
// 원본 waypoint 간격(대회 경로 기준 약 0.4~0.5m)보다 훨씬 작게 잡아 우연히
// 가까운 open path를 closed로 오판정하지 않는다.
bool DetectClosedLoop(const std::vector<double>& x, const std::vector<double>& y,
                       double closure_tolerance_m) {
    const int n = static_cast<int>(x.size());
    if (n < 4) return false;  // 닫힌 삼각형 이상은 되어야 폐루프로 취급
    const double dx = x.back() - x.front();
    const double dy = y.back() - y.front();
    return std::sqrt(dx * dx + dy * dy) <= closure_tolerance_m;
}

// (아래 3단계 스무딩 파이프라인에서 사용, 정의는 이 파일 뒤쪽에 있음)
void ComputeThetaKappa(const std::vector<double>& x, const std::vector<double>& y,
                        bool closed, std::vector<double>& theta, std::vector<double>& kappa);

// MGeo 링크를 이어붙인 원본 waypoint에는 국소적으로 물리적 회전반경(1~2m대)을
// 넘는 급격한 꺾임이 다수 섞여있어(링크 이어붙임 이음매, 원본 좌표 노이즈 등),
// FilterByCurvature가 그 지점을 지나는 모든 후보를 동시에 무효화시킨다.
// 아래 3단계(스무딩 -> 균일 재샘플링 -> 잔여 초과분 국소 이완)로 완화한다.
// 스무딩만 단독으로 하면 인접점끼리 arc-length상 다닥다닥 붙어버려 오히려
// kappa=dtheta/ds의 분모(ds)가 작아지며 곡률이 더 커지는 역효과가 실측으로
// 확인됐다 (재샘플링으로 ds를 다시 균일하게 맞춰줘야 함). 각 단계 파라미터는
// config.smoothing_window_m/resample_ds/relax_max_iter/relax_step.

// 폐루프 총 물리 길이(닫힘 구간 포함). 열린 경로는 그냥 누적 s의 마지막 값.
double TotalArcLength(const std::vector<double>& x, const std::vector<double>& y, bool closed) {
    const int n = static_cast<int>(x.size());
    std::vector<double> s = CumulativeArcLength(x, y);
    double total = s.empty() ? 0.0 : s.back();
    if (closed && n > 0) {
        const double dx = x.front() - x[n - 1];
        const double dy = y.front() - y[n - 1];
        total += std::sqrt(dx * dx + dy * dy);
    }
    return total;
}

void SmoothWaypointsOpen(const std::vector<double>& x, const std::vector<double>& y,
                         double window_m, std::vector<double>& out_x, std::vector<double>& out_y) {
    const int n = static_cast<int>(x.size());
    std::vector<double> s = CumulativeArcLength(x, y);
    out_x.resize(n);
    out_y.resize(n);

    int lo = 0, hi = 0;
    double sum_x = 0.0, sum_y = 0.0;
    for (int i = 0; i < n; i++) {
        const double s_lo = s[i] - window_m / 2.0;
        const double s_hi = s[i] + window_m / 2.0;

        while (lo < n && s[lo] < s_lo) { sum_x -= x[lo]; sum_y -= y[lo]; lo++; }
        while (hi < n && s[hi] <= s_hi) { sum_x += x[hi]; sum_y += y[hi]; hi++; }

        const int cnt = hi - lo;
        out_x[i] = (cnt > 0) ? sum_x / cnt : x[i];
        out_y[i] = (cnt > 0) ? sum_y / cnt : y[i];
    }
}

// 폐루프(unique circular sequence, 마지막 점이 첫점의 복제가 아님) 이동평균.
// 이음새 근방 점의 윈도우가 반대편(트랙 끝/시작)까지 원형으로 이어지도록
// 순환 인덱스로 확장한다 - 일반 내부 점과 동일한 방식으로 처리된다.
void SmoothWaypointsClosed(const std::vector<double>& x, const std::vector<double>& y,
                           double window_m, std::vector<double>& out_x, std::vector<double>& out_y) {
    const int n = static_cast<int>(x.size());
    out_x.resize(n);
    out_y.resize(n);
    if (n == 0) return;

    // 세그먼트 길이(닫힘 구간 포함, 순환): seg[i] = i번째 점 -> (i+1)%n 점 거리.
    std::vector<double> seg(n);
    for (int i = 0; i < n; i++) {
        const int j = (i + 1) % n;
        const double dx = x[j] - x[i];
        const double dy = y[j] - y[i];
        seg[i] = std::sqrt(dx * dx + dy * dy);
    }

    const double half = window_m / 2.0;
    for (int i = 0; i < n; i++) {
        double sum_x = x[i], sum_y = y[i];
        int cnt = 1;

        // 전방으로 half만큼 순환 누적.
        double acc = 0.0;
        for (int k = 1; k < n; k++) {
            const int idx_prev_seg = (i + k - 1) % n;
            acc += seg[idx_prev_seg];
            if (acc > half) break;
            const int j = (i + k) % n;
            sum_x += x[j]; sum_y += y[j]; cnt++;
        }
        // 후방으로 half만큼 순환 누적.
        acc = 0.0;
        for (int k = 1; k < n; k++) {
            const int idx_prev_seg = (i - k + n) % n;
            acc += seg[idx_prev_seg];
            if (acc > half) break;
            const int j = (i - k + n) % n;
            sum_x += x[j]; sum_y += y[j]; cnt++;
        }

        out_x[i] = sum_x / cnt;
        out_y[i] = sum_y / cnt;
    }
}

void SmoothWaypoints(const std::vector<double>& x, const std::vector<double>& y,
                      double window_m, bool closed,
                      std::vector<double>& out_x, std::vector<double>& out_y) {
    if (closed) SmoothWaypointsClosed(x, y, window_m, out_x, out_y);
    else SmoothWaypointsOpen(x, y, window_m, out_x, out_y);
}

// 스무딩으로 흐트러진 점 간격을 균일 arc-length 간격으로 재조정 (열린 경로).
void ResampleUniformOpen(const std::vector<double>& x, const std::vector<double>& y, double ds_target,
                         std::vector<double>& out_x, std::vector<double>& out_y) {
    std::vector<double> s = CumulativeArcLength(x, y);
    const double total = s.back();
    const int n_out = static_cast<int>(total / ds_target) + 1;
    out_x.resize(n_out);
    out_y.resize(n_out);

    int j = 0;
    for (int k = 0; k < n_out; k++) {
        const double target_s = k * ds_target;
        while (j < static_cast<int>(s.size()) - 2 && s[j + 1] < target_s) j++;
        const double ds = s[j + 1] - s[j];
        const double ratio = (ds > 1e-9) ? (target_s - s[j]) / ds : 0.0;
        out_x[k] = x[j] + ratio * (x[j + 1] - x[j]);
        out_y[k] = y[j] + ratio * (y[j + 1] - y[j]);
    }
}

// 폐루프 균일 재샘플링. 닫힘 구간(마지막 unique점 -> 첫 unique점)을 포함한
// 전체 원형 길이를 ds_target 간격으로 샘플링한다. k=0(target_s=0)만 뽑고
// target_s가 total에 도달하는 마지막 샘플은 만들지 않는다 - 그 위치는
// BuildRefLine이 나중에 "첫 점의 복제"로 s=length에 명시적으로 추가하므로,
// 여기서 만들면 같은 위치가 두 번 샘플링(seam 근처 0거리 이중 포인트)된다.
void ResampleUniformClosed(const std::vector<double>& x, const std::vector<double>& y, double ds_target,
                           std::vector<double>& out_x, std::vector<double>& out_y) {
    const int n = static_cast<int>(x.size());
    out_x.clear();
    out_y.clear();
    if (n == 0) return;

    // 순환 누적 s: s_ext[0..n]으로 확장하되 s_ext[n] = total(닫힘 구간 포함).
    std::vector<double> s_ext(n + 1);
    s_ext[0] = 0.0;
    for (int i = 0; i < n; i++) {
        const int j = (i + 1) % n;
        const double dx = x[j] - x[i];
        const double dy = y[j] - y[i];
        s_ext[i + 1] = s_ext[i] + std::sqrt(dx * dx + dy * dy);
    }
    const double total = s_ext[n];
    const int n_out = std::max(1, static_cast<int>(std::round(total / ds_target)));
    const double actual_ds = total / n_out;  // 정확히 한 바퀴를 n_out개로 균등 분할

    out_x.resize(n_out);
    out_y.resize(n_out);

    int j = 0;
    for (int k = 0; k < n_out; k++) {
        const double target_s = k * actual_ds;
        while (j < n - 1 && s_ext[j + 1] < target_s) j++;
        const double seg_len = s_ext[j + 1] - s_ext[j];
        const double ratio = (seg_len > 1e-9) ? (target_s - s_ext[j]) / seg_len : 0.0;
        const int j_next = (j + 1) % n;
        out_x[k] = x[j] + ratio * (x[j_next] - x[j]);
        out_y[k] = y[j] + ratio * (y[j_next] - y[j]);
    }
}

void ResampleUniform(const std::vector<double>& x, const std::vector<double>& y, double ds_target,
                      bool closed, std::vector<double>& out_x, std::vector<double>& out_y) {
    if (closed) ResampleUniformClosed(x, y, ds_target, out_x, out_y);
    else ResampleUniformOpen(x, y, ds_target, out_x, out_y);
}

// 스무딩+재샘플링 이후에도 남아있는 국소 곡률 초과 지점을, 양옆 점의 중점
// 쪽으로 조금씩 당겨서(라플라시안 완화) 반복적으로 줄인다.
// closed==true면 순환 시퀀스 전체(이음새 포함)를 내부 점으로 취급한다 -
// 열린 경로처럼 양 끝(i=0, i=n-1)을 예외 처리하지 않는다.
void FixInfeasibleCurvature(std::vector<double>& x, std::vector<double>& y,
                            bool closed, double max_curvature,
                            int relax_max_iter, double relax_step) {
    const int n = static_cast<int>(x.size());
    if (n < 3) return;

    std::vector<double> theta, kappa;
    for (int iter = 0; iter < relax_max_iter; iter++) {
        ComputeThetaKappa(x, y, closed, theta, kappa);

        std::vector<int> bad;
        const int begin = closed ? 0 : 1;
        const int end = closed ? n : n - 1;  // exclusive
        for (int i = begin; i < end; i++) {
            if (std::abs(kappa[i]) > max_curvature) bad.push_back(i);
        }
        if (bad.empty()) break;

        for (int i : bad) {
            const int prev = closed ? (i - 1 + n) % n : std::max(i - 1, 0);
            const int next = closed ? (i + 1) % n : std::min(i + 1, n - 1);
            const double mx = 0.5 * (x[prev] + x[next]);
            const double my = 0.5 * (y[prev] + y[next]);
            x[i] = x[i] * (1.0 - relax_step) + mx * relax_step;
            y[i] = y[i] * (1.0 - relax_step) + my * relax_step;
        }
    }
}

// theta(중앙차분) + kappa(중앙차분) 계산.
// closed==true: 모든 점이 순환 이웃(prev=(i-1+n)%n, next=(i+1)%n)을 쓴다 -
// 이음새 양쪽 점을 하나의 연속된 기하로 다룬다.
// closed==false: 기존과 동일하게 양 끝점은 한쪽 차분만 쓴다.
void ComputeThetaKappa(const std::vector<double>& x, const std::vector<double>& y,
                        bool closed, std::vector<double>& theta, std::vector<double>& kappa) {
    const int n = static_cast<int>(x.size());
    std::vector<double> s = CumulativeArcLength(x, y);

    theta.resize(n);
    for (int i = 0; i < n; i++) {
        double dx, dy;
        if (closed) {
            const int prev = (i - 1 + n) % n;
            const int next = (i + 1) % n;
            dx = x[next] - x[prev];
            dy = y[next] - y[prev];
        } else if (i == 0) {
            dx = x[1] - x[0];         dy = y[1] - y[0];
        } else if (i == n - 1) {
            dx = x[n-1] - x[n-2];     dy = y[n-1] - y[n-2];
        } else {
            dx = x[i+1] - x[i-1];     dy = y[i+1] - y[i-1];
        }
        theta[i] = std::atan2(dy, dx);
    }

    kappa.resize(n);
    for (int i = 0; i < n; i++) {
        int a, b;
        double ds;
        if (closed) {
            a = (i - 1 + n) % n;
            b = (i + 1) % n;
            // 순환 구간 길이: a->i->b를 따라간 실제 호길이(닫힘 구간 포함).
            const double seg_a = std::hypot(x[i] - x[a], y[i] - y[a]);
            const double seg_b = std::hypot(x[b] - x[i], y[b] - y[i]);
            ds = seg_a + seg_b;
        } else {
            a = (i == 0) ? 0 : i - 1;
            b = (i == n - 1) ? n - 1 : i + 1;
            ds = s[b] - s[a];
        }
        const double d_theta = NormalizeAngleDiff(theta[b] - theta[a]);
        kappa[i] = (ds > 1e-9) ? d_theta / ds : 0.0;
    }
}

}  // namespace

// =========================================================
// BuildRefLine
// =========================================================

RefLine BuildRefLine(const std::vector<double>& wx, const std::vector<double>& wy,
                     const ReferenceConfig& config) {

    if (wx.size() < 2 || wx.size() != wy.size())
        throw std::invalid_argument("Waypoints must have at least 2 points and matching sizes");

    const bool closed = DetectClosedLoop(wx, wy, config.closure_tolerance_m);

    // 폐루프 입력은 마지막 점이 첫점의 물리적 복제다(대회 global path 계약).
    // 모든 순환 전처리(smoothing/resampling/derivative)는 이 복제를 제거한
    // unique 시퀀스에서 수행한다 - 복제점이 이동평균/미분 창에 끼어들면
    // 이음새 근방 결과가 왜곡된다.
    std::vector<double> unique_x = wx;
    std::vector<double> unique_y = wy;
    if (closed && unique_x.size() > 1) {
        unique_x.pop_back();
        unique_y.pop_back();
    }

    // 0. 국소 급커브 완화: 스무딩 -> 균일 재샘플링 -> 잔여 초과분 국소 이완.
    std::vector<double> smoothed_x, smoothed_y;
    SmoothWaypoints(unique_x, unique_y, config.smoothing_window_m, closed, smoothed_x, smoothed_y);

    std::vector<double> sx, sy;
    ResampleUniform(smoothed_x, smoothed_y, config.resample_ds, closed, sx, sy);

    FixInfeasibleCurvature(sx, sy, closed, config.max_curvature,
                           config.relax_max_iter, config.relax_step);

    const int n_unique = static_cast<int>(sx.size());
    const int n = closed ? n_unique + 1 : n_unique;

    RefLine ref;
    ref.closed_loop = closed;
    ref.points.resize(n);

    // 1. arc length s 누적 계산 (닫힘 구간은 아래에서 마지막 점에 별도 반영).
    std::vector<double> s = CumulativeArcLength(sx, sy);
    const double loop_length = closed ? TotalArcLength(sx, sy, /*closed=*/true) : (s.empty() ? 0.0 : s.back());
    ref.length = loop_length;

    // 2~3. theta/kappa: 순환 또는 편측 중앙 차분.
    std::vector<double> theta, kappa;
    ComputeThetaKappa(sx, sy, closed, theta, kappa);

    for (int i = 0; i < n_unique; i++) {
        ref.points[i].x = sx[i];
        ref.points[i].y = sy[i];
        ref.points[i].s = s[i];
        ref.points[i].theta = theta[i];
        ref.points[i].kappa = kappa[i];
    }

    // 4. d_kappa: d_kappa/ds. closed는 순환 이웃, open은 기존 편측 처리.
    for (int i = 0; i < n_unique; i++) {
        int a, b;
        double ds;
        if (closed) {
            a = (i - 1 + n_unique) % n_unique;
            b = (i + 1) % n_unique;
            const double seg_a = std::hypot(sx[i] - sx[a], sy[i] - sy[a]);
            const double seg_b = std::hypot(sx[b] - sx[i], sy[b] - sy[i]);
            ds = seg_a + seg_b;
        } else if (i == 0) {
            a = 0; b = 1;
            ds = ref.points[1].s - ref.points[0].s;
        } else if (i == n_unique - 1) {
            a = n_unique - 2; b = n_unique - 1;
            ds = ref.points[n_unique - 1].s - ref.points[n_unique - 2].s;
        } else {
            a = i - 1; b = i + 1;
            ds = ref.points[i + 1].s - ref.points[i - 1].s;
        }
        const double dk = kappa[b] - kappa[a];
        ref.points[i].d_kappa = (ds > 1e-9) ? dk / ds : 0.0;
    }

    // 5. 폐루프: 첫 점을 s=length에 복제해 닫음점을 명시적으로 추가한다.
    // theta/kappa/d_kappa도 그대로 복제되므로 이음새 양쪽에서 항상 같은
    // 가운데 point 값과 만난다(Interpolate가 s=length 근방에서 보간할 때
    // 자연스럽게 연속) - F-7 요구사항.
    if (closed) {
        ref.points[n_unique] = ref.points[0];
        ref.points[n_unique].s = loop_length;
    }

    return ref;
}

// =========================================================
// Interpolate — 선형 보간
// FindClosestS가 각 Newton 반복마다 임의의 실수 s에서 RefPoint를
// 필요로 하기 때문에, Newton 함수보다 먼저 정의되어야 한다.
//
// topology(ref.closed_loop)를 이 함수가 유일하게 판단한다 - 호출자는
// s를 미리 감거나 클램핑하지 않는다("one policy, one authoritative owner",
// F-9).
// =========================================================

RefPoint Interpolate(const RefLine& ref, double s) {
    const auto& pts = ref.points;
    int n = static_cast<int>(pts.size());

    if (ref.closed_loop) {
        // 폐루프(닫힌 트랙) wrap-around: front()==back() 위치가 물리적으로
        // 같은 점(폐루프 폐합점)이므로, 범위를 벗어나면 클램핑 대신
        // [0, length)로 감아준다. 이전엔 s를 [front().s, back().s]로
        // 클램핑했는데, 이러면 이음새(피니시라인) 근처에서 s가 끝점에
        // 얼어붙어 실제 위치와 무관한 d/heading이 나오는 문제가 있었다.
        const double length = ref.length;
        double s_wrapped = std::fmod(s, length);
        if (s_wrapped < 0.0) s_wrapped += length;
        s = s_wrapped;
    } else {
        // 열린 경로: wrap하지 않는다 - 범위를 벗어나면 끝점으로 클램핑한다.
        if (s <= pts.front().s) return pts.front();
        if (s >= pts.back().s)  return pts.back();
    }

    // 부동소수 오차로 경계값을 살짝 벗어나는 경우만 클램핑.
    if (s <= pts.front().s) return pts.front();
    if (s >= pts.back().s)  return pts.back();

    // s가 속하는 구간 이분 탐색 (index 구하기)
    int lo = 0, hi = n - 1;
    while (hi - lo > 1) {
        int mid = (lo + hi) / 2;
        if (pts[mid].s <= s) lo = mid;
        else hi = mid;
    }

    double ds = pts[hi].s - pts[lo].s;
    double ratio = (ds > 1e-9) ? (s - pts[lo].s) / ds : 0.0;  // 선형 보간 비율 t값 구하기

    RefPoint rp;
    rp.s = s;
    rp.x = pts[lo].x + ratio * (pts[hi].x - pts[lo].x);
    rp.y = pts[lo].y + ratio * (pts[hi].y - pts[lo].y);
    rp.theta = pts[lo].theta + ratio * NormalizeAngleDiff(pts[hi].theta - pts[lo].theta);
    rp.kappa = pts[lo].kappa + ratio * (pts[hi].kappa - pts[lo].kappa);
    rp.d_kappa = pts[lo].d_kappa + ratio * (pts[hi].d_kappa - pts[lo].d_kappa);

    return rp;
}

// =========================================================
// FindClosestS — Newton-Raphson
//
// 논문 Appendix I은 [x - r(s)]^T * t_r(s) = 0  (원문 식, App.I 중간)
// 이 항상 성립한다고 명시한다 (트래킹 포인트에서 접선과 잔차 벡터가 직교).
// 즉 s* = argmin_s ||x - r(s)||  문제를, 아래의 스칼라 방정식의 근을
// 구하는 문제로 바꿀 수 있다.
//
//   f(s) := (r(s) - x)^T * t_r(s) = 0
//
// Newton-Raphson: s_{k+1} = s_k - f(s_k) / f'(s_k)
//
// f'(s)를 Frenet-Serret 공식 t_r'(s) = kappa_r(s) * n_r(s) 을 이용해
// 미분하면
//
//   f'(s) = t_r(s)^T t_r(s) + (r(s)-x)^T * kappa_r(s) * n_r(s)
//         = 1 + kappa_r(s) * [(r(s)-x)^T n_r(s)]
//
// 그런데 논문 식 (4)에서 d = (x - r(s))^T n_r(s) 로 정의하므로
// (r(s)-x)^T n_r(s) = -d 이고, 결국
//
//   f'(s) = 1 - kappa_r(s) * d
//
// 이는 근사식이 아니라 정확한 해석적 도함수다 (App.I 식(7) 분모에
// 등장하는 1 - kappa_r*d 항과 동일한 표현).
//
// topology(ref.closed_loop)를 이 함수가 유일하게 판단한다(F-10) - hint
// 원형 거리 비교와 Newton wrap 여부가 모두 여기 안에서 결정된다.
// =========================================================

double FindClosestS(const RefLine& ref, double x, double y,
                     const double* s_hint, double window) {

    // 초기값: 가장 가까운 웨이포인트의 s (brute-force).
    // Newton법은 국소 수렴이라 초기값이 실제 최소점 근방이어야 한다.
    //
    // s_hint가 있으면 s_hint ± window 범위의 점들만 후보로 삼는다 (헤더 주석
    // 참고 - 트랙이 자기 자신과 가까운 구간에서 전체 탐색이 엉뚱하게 먼 s로
    // 튀는 문제 방지). 범위 안에 점이 하나도 없으면(비정상 상황) 안전하게
    // 전체 탐색으로 폴백한다.
    const double length = ref.length;

    double best_s = ref.points[0].s;
    double best_d2 = std::numeric_limits<double>::max();
    bool found = false;
    if (s_hint) {
        for (const auto& pt : ref.points) {
            double diff = std::fabs(pt.s - *s_hint);
            // 폐루프만 이음새 건너편을 원형 거리로 가깝다고 본다. 열린
            // 경로는 끝점 너머가 물리적으로 이어지지 않으므로 선형 거리만
            // 쓴다(F-10: 열린 경로에서 반대쪽 끝으로 wrap하지 않는다).
            double circ_dist = ref.closed_loop
                ? std::min(diff, length - diff)
                : diff;
            if (circ_dist > window) continue;
            double dx = x - pt.x;
            double dy = y - pt.y;
            double d2 = dx * dx + dy * dy;
            if (d2 < best_d2) { best_d2 = d2; best_s = pt.s; found = true; }
        }
    }
    if (!found) {
        for (const auto& pt : ref.points) {
            double dx = x - pt.x;
            double dy = y - pt.y;
            double d2 = dx * dx + dy * dy;
            if (d2 < best_d2) { best_d2 = d2; best_s = pt.s; }
        }
    }

    double s = best_s;

    for (int iter = 0; iter < 20; iter++) {
        RefPoint rp = Interpolate(ref, s);

        double tx = std::cos(rp.theta);   // t_r(s)
        double ty = std::sin(rp.theta);

        // f(s) = (r(s) - x)^T t_r(s)
        double f = (rp.x - x) * tx + (rp.y - y) * ty;

        // d = (x - r(s))^T n_r(s), n_r = (-sin theta, cos theta)
        double d = (x - rp.x) * (-std::sin(rp.theta)) + (y - rp.y) * std::cos(rp.theta);

        // f'(s) = 1 - kappa_r(s) * d
        double fp = 1.0 - rp.kappa * d;

        if (std::abs(fp) < 1e-9) break;  // 특이점(곡률 반경과 |d|가 거의 일치) 회피

        double ds = -f / fp;
        s += ds;

        if (ref.closed_loop) {
            // 폐루프 wrap-around: 이음새 근처에서 Newton 스텝이 s=0 또는
            // s=length를 자연스럽게 넘나들 수 있어야 한다(클램핑하면 반대편
            // 으로 못 건너가고 끝점에 갇힌다). Interpolate도 동일하게
            // wrap하므로 f(s)/f'(s) 계산 자체는 항상 유효하다.
            s = std::fmod(s, length);
            if (s < 0.0) s += length;
        } else {
            // 열린 경로: 반대쪽 끝으로 넘어가지 않는다. 유효 범위로 clamp.
            s = std::max(ref.points.front().s, std::min(ref.points.back().s, s));
        }

        if (std::abs(ds) < 1e-6) break;  // 수렴
    }

    return s;
}

bool LoadReferenceLine(const std::string& path, RefLine& out_ref,
                       const ReferenceConfig& config) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::printf("[FrenetPlanner] Failed to open waypoint file: %s\n", path.c_str());
        return false;
    }

    std::vector<double> wx, wy;
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;

        // 공백 우선
        {
            std::stringstream ss(line);
            double x, y;
            if (ss >> x >> y) {
                wx.push_back(x);
                wy.push_back(y);
                continue;
            }
        }

        // CSV (콤마 구분)
        std::stringstream ss(line);
        std::string val;
        std::vector<double> row;
        while (std::getline(ss, val, ',')) {
            val.erase(0, val.find_first_not_of(" \t\r\n"));
            if (!val.empty()) val.erase(val.find_last_not_of(" \t\r\n") + 1);
            if (val.empty()) continue;
            try { row.push_back(std::stod(val)); } catch (...) { /* 헤더 라인 등 skip */ }
        }
        if (row.size() >= 2) {
            wx.push_back(row[0]);
            wy.push_back(row[1]);
        }
    }

    if (wx.size() < 2) {
        std::printf("[FrenetPlanner] Waypoint file has fewer than 2 valid points: %s\n", path.c_str());
        return false;
    }

    out_ref = BuildRefLine(wx, wy, config);
    std::printf("[FrenetPlanner] Loaded %zu waypoints from %s (closed_loop=%s, length=%.2f)\n",
               wx.size(), path.c_str(), out_ref.closed_loop ? "true" : "false", out_ref.length);
    return true;
}
