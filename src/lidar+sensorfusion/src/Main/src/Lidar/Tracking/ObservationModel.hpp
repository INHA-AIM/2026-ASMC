#pragma once
#include <Global/Global.hpp>
#include <Lidar/Tracking/TrackTrace.hpp>
#include <algorithm>
#include <cmath>

// Stage2 experimental measurement model, not a borrowed algorithm.
// No GT/class/history/innovation is read. It cannot recover unobserved extents.
namespace observation_model {
inline Eigen::Matrix2f QualityCovariance(const LiDARObjectInfo& o) {
    const double c=std::cos(o.f32_OBB_YawRad), s=std::sin(o.f32_OBB_YawRad);
    const double hu=std::max(.025, .5*double(o.f32_OBB_Width));
    const double hv=std::max(.025, .5*double(o.f32_OBB_Height));
    double su=0,sv=0,suu=0,svv=0,fit=0; int n=0;
    for(const auto& p:o.pcl_ClusterPoints) {
        if(!std::isfinite(p.x)||!std::isfinite(p.y))continue;
        const double dx=p.x-o.f32_OBB_CenterX,dy=p.y-o.f32_OBB_CenterY;
        const double u=c*dx+s*dy,v=-s*dx+c*dy;
        su+=u;sv+=v;suu+=u*u;svv+=v*v;
        const double du=std::abs(std::abs(u)-hu),dv=std::abs(std::abs(v)-hv);
        const double distance=std::min(du,dv);
        fit+=std::min(distance*distance,1.0); ++n;
    }
    if(n<2)return Eigen::Matrix2f::Identity()*.25f;
    const double neff=std::min(n,16); // correlated rings; never treat N points as N independent scans.
    const double vu=std::max(0.,suu/n-(su/n)*(su/n));
    const double vv=std::max(0.,svv/n-(sv/n)*(sv/n));
    // 0.10 m sensor/model floor plus partial-support scale. Neither can vanish with more points.
    const double ru=.01+fit/n+vu/neff+std::pow(.08*2*hu,2);
    const double rv=.01+fit/n+vv/neff+std::pow(.08*2*hv,2);
    Eigen::Matrix2f axes;axes<<c,-s,s,c;
    return axes*(Eigen::Vector2f(std::min(ru,4.),std::min(rv,4.)).asDiagonal())*axes.transpose();
}
inline void Prepare(LiDARObjectInfo& o) {
    const auto& options=tracking_options::Get();
    if(options.observation_quality_r) {
        o.observation_R=QualityCovariance(o);o.observation_R_valid=true;
    }
    if(options.observation_obb_center) {
        o.f32_CoreX=o.f32_OBB_CenterX; o.f32_CoreY=o.f32_OBB_CenterY;
        o.f32_Dist=std::hypot(o.f32_CoreX,o.f32_CoreY);
    }
}
// Integral exp(F t) L q L^T exp(F^T t) dt, interleaved x/y CA order.
inline Eigen::Matrix<float,6,6> ProcessCovariance(double dt) {
    if(!tracking_options::Get().jerk_q)return Eigen::Matrix<float,6,6>::Identity()*.003f;
    Eigen::Matrix<float,6,6> q=Eigen::Matrix<float,6,6>::Zero();
    const double t=std::max(0.,dt),d=tracking_options::Get().jerk_spectral_density;
    const double block[3][3]={{std::pow(t,5)/20,std::pow(t,4)/8,std::pow(t,3)/6},
                            {std::pow(t,4)/8,std::pow(t,3)/3,t*t/2},
                            {std::pow(t,3)/6,t*t/2,t}};
    for(int axis=0;axis<2;++axis)for(int i=0;i<3;++i)for(int j=0;j<3;++j)q(2*i+axis,2*j+axis)=d*block[i][j];
    return q;
}
}
