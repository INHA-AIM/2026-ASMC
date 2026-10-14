#include "Parser/vehicle_curvature.hpp"
#include "Frenet/Math/FrenetConverter.hpp"
#include <cmath>
#include <iostream>
#include <limits>

int main() {
    int failures = 0;
    auto expect = [&](bool ok, const char* name) {
        std::cout << (ok ? "[PASS] " : "[FAIL] ") << name << '\n';
        failures += !ok;
    };
    using frenet_planner_ros::VehicleCurvature;
    expect(std::fabs(VehicleCurvature(10, .2)-.02)<1e-12, "moving left turn curvature");
    expect(std::fabs(VehicleCurvature(10, -.2)+.02)<1e-12, "moving right turn curvature");
    expect(std::fabs(VehicleCurvature(-10, .2)+.02)<1e-12, "signed reverse velocity");
    for (double speed : {0.0, .001, -.001, .5, -.5})
        expect(VehicleCurvature(speed, 1e-4)==0, "low-speed IMU noise is bounded");
    expect(std::fabs(VehicleCurvature(.51, .051)-.1)<1e-12, "measured curvature resumes above low-speed boundary");
    expect(std::isnan(VehicleCurvature(1, std::numeric_limits<double>::quiet_NaN())), "invalid yaw rate remains invalid");
    expect(std::isnan(VehicleCurvature(std::numeric_limits<double>::infinity(), 0)), "invalid speed remains invalid");
    expect(!std::isfinite(VehicleCurvature(.51, std::numeric_limits<double>::max())), "overflow cannot be published as finite curvature");
    // Same geometric curve at different speeds must have the same Frenet
    // lateral boundary condition, rather than an artificial outward acceleration.
    RefLine ref;
    ref.length = 40;
    for (int i=0; i<=80; ++i) {
        const double s=i*.5, angle=s/50;
        ref.points.push_back({50*std::sin(angle),50*(1-std::cos(angle)),s,angle,.02,0});
    }
    for (double speed : {2.0,5.0,10.0}) {
        const auto rp=Interpolate(ref,20);
        CartesianState ego{rp.x,rp.y,rp.theta,VehicleCurvature(speed,speed*.02),speed,0};
        double s,sd,sdd,d,dp,dpp;
        const double hint=20;
        CartesianToFrenet(ref,ego,s,sd,sdd,d,dp,dpp,&hint,10);
        expect(std::fabs(d)<1e-6 && std::fabs(dp)<1e-6 && std::fabs(dpp)<1e-6,
               "curve-following ego has zero relative lateral acceleration at each speed");
        ego.kappa=0;
        CartesianToFrenet(ref,ego,s,sd,sdd,d,dp,dpp,&hint,10);
        expect(dpp*sd*sd < -.05, "zero vehicle curvature exposes the former artificial outward boundary");
    }
    return failures ? 1 : 0;
}
