#include "Frenet/Reference/Reference.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
// The original full scan is the regression oracle for floating-point window
// boundaries and equal-distance seeds. The optimized implementation uses an
// ordered station range; both then retain the existing Newton projection.
double ScanProjection(const RefLine& ref, double x, double y,
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

int main(){unsigned checks=0,failures=0;std::mt19937_64 random(18733);
std::uniform_real_distribution<double> u(0,1);
for(bool closed:{false,true}){RefLine ref;ref.closed_loop=closed;
for(int i=0;i<=2000;++i){double s=i*.5;ref.points.push_back({150*std::sin(s/150),150*(1-std::cos(s/150)),s,s/150,1./150,0});}ref.length=ref.points.back().s;
for(int j=0;j<3000;++j){double hint=(u(random)*1.4-.2)*ref.length,window=u(random)*80;
if(j%11==0)window=0;if(j%13==0)window=-1;if(j%17==0)window=std::numeric_limits<double>::infinity();
if(j%19==0)hint=std::numeric_limits<double>::quiet_NaN();
double x=(u(random)*2-1)*170,y=u(random)*320;
const double* h=j%3?&hint:nullptr;double a=ScanProjection(ref,x,y,h,window),b=FindClosestS(ref,x,y,h,window);
++checks;if(a!=b){++failures;if(failures<5)std::cerr<<"projection mismatch "<<a<<" "<<b<<'\n';}}
// Exact and adjacent floating-point station-window boundaries.
for(int j=1;j<1999;j+=7)for(double direction:{-1.,0.,1.}){double hint=ref.points[j].s,window=.5;
if(direction)window=std::nextafter(window,direction<0?0:INFINITY);
auto p=ref.points[j+1];double a=ScanProjection(ref,p.x,p.y,&hint,window),b=FindClosestS(ref,p.x,p.y,&hint,window);++checks;if(a!=b)++failures;}}
std::cout<<checks<<" checks, "<<failures<<" failures\n";return failures?1:0;}
