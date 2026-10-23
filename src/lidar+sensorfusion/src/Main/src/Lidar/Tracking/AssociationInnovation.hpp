#pragma once
#include <Geometry/FrameBoundary.hpp>
#include <Lidar/Tracking/ObservationModel.hpp>
#include <limits>

// Association-only experiment. Uses the SAME aligned CA covariance and current measurement
// covariance as the KF update. No point selection, state reset, Q/R tuning, or velocity clamp.
namespace association_innovation {
constexpr double kChiSquare99 = 9.210340371976184;
inline Eigen::Matrix2f PredictedPositionCovariance(const LiDARObjectInfo& previous,
                                                  double dt, double yaw_now_rad) {
    // Deliberately copy only the state/frame metadata, never old cluster points.
    LiDARObjectInfo aligned{};
    aligned.Eigen_KfState_CA=previous.Eigen_KfState_CA;
    aligned.Eigen_KfP_CA=previous.Eigen_KfP_CA;
    aligned.tracking_frame_valid=previous.tracking_frame_valid;
    aligned.tracking_height_input=previous.tracking_height_input;
    aligned.tracking_yaw_from_input=previous.tracking_yaw_from_input;
    aligned.tracking_origin_in_yaw=previous.tracking_origin_in_yaw;
    aligned.tracking_stamp_ns=previous.tracking_stamp_ns;
    aligned.tracking_ego_speed_mps=previous.tracking_ego_speed_mps;
    aligned.f32_PrevYaw=previous.f32_PrevYaw;
    frame_boundary::AlignTrackingFrame(aligned,yaw_now_rad,false);
    Eigen::Matrix<float,6,6> a=Eigen::Matrix<float,6,6>::Identity();
    a(0,2)=dt;a(1,3)=dt;a(0,4)=.5*dt*dt;a(1,5)=.5*dt*dt;a(2,4)=dt;a(3,5)=dt;
    const Eigen::Matrix<float,6,6> p=a*aligned.Eigen_KfP_CA*a.transpose()
        +observation_model::ProcessCovariance(dt);
    return p.topLeftCorner<2,2>();
}
inline double NormalizedInnovation(const Eigen::Vector2f& residual,
                                   const Eigen::Matrix2f& predicted,
                                   const Eigen::Matrix2f& measurement) {
    const Eigen::Matrix2d s=(predicted+measurement).cast<double>();
    if(!s.allFinite() || !residual.allFinite())return std::numeric_limits<double>::quiet_NaN();
    const Eigen::LDLT<Eigen::Matrix2d> factor(s);
    if(factor.info()!=Eigen::Success || !factor.isPositive() ||
       factor.vectorD().minCoeff()<=0.)return std::numeric_limits<double>::quiet_NaN();
    return residual.cast<double>().dot(factor.solve(residual.cast<double>()));
}
inline bool Reject(double nis) {return std::isfinite(nis) && nis>kChiSquare99;}
} // namespace association_innovation
