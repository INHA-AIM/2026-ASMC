#pragma once
#include <SensorFusion/FusionStateTrace.hpp>
#include <Lidar/ImuRansac/GroundEvidence.hpp>
#include <array>
#include <set>

// Trace-only point correspondence uses voxel indices, not raw beam identities.
// No point copy, lookup set or formatting is performed with tracing disabled.
namespace pipeline_trace {
inline void Ground(const ros::Time& stamp,
    const pcl::PointCloud<pcl::PointXYZI>& sensor,
    const pcl::PointCloud<pcl::PointXYZI>& leveled,
    const pcl::PointCloud<pcl::PointXYZI>& remaining,
    const GroundPlaneSnapshot& plane) {
    if (!fusion_trace::Sink().Enabled()) return;
    std::set<std::array<float,3>> kept;
    for (const auto& p:remaining) kept.insert({p.x,p.y,p.z});
    fusion_trace::Event("ground_points",stamp.toNSec(),[&](std::ostream& out) {
        out << ",\"plane_valid\":" << (plane.valid?"true":"false")
            << ",\"plane\":[";
        for (unsigned i=0;i<4;++i) {if(i)out<<',';out<<plane.coefficients[i];}
        out << "],\"points\":[";
        for (unsigned i=0;i<leveled.size();++i) {
            if(i)out<<','; const auto& p=leveled[i]; const auto& q=sensor[i];
            out << '[' << i << ',' << q.x << ',' << q.y << ',' << q.z
                << ',' << p.x << ',' << p.y << ',' << p.z << ','
                << (kept.count({p.x,p.y,p.z})?1:0) << ']';
        }
        out << ']';
    });
}
inline void Tracking(const ros::Time& stamp, const ros::Time& previous,
                     const std::vector<LiDARObjectInfo>& objects) {
    fusion_trace::Event("tracking_state",stamp.toNSec(),[&](std::ostream& out) {
        out << ",\"previous_stamp_ns\":" << previous.toNSec()
            << ",\"source_dt_sec\":" << (stamp-previous).toSec()
            << ",\"objects\":[";
        bool first=true;
        for(const auto& o:objects) {
            if(!first)out<<',';first=false;
            out << "{\"semantic\":";fusion_state_trace::Object(out,o);
            out << ",\"age\":" << o.s32_Age << ",\"kf_initialized\":" << (o.b_KfInitialized?"true":"false")
                << ",\"kf_init_count\":" << o.s32_KfInitCount << ",\"kf_ca\":[";
            for(unsigned j=0;j<6;++j){if(j)out<<',';out<<o.Eigen_KfState_CA(j);}
            out << "],\"legacy_absolute_velocity\":[" << o.f32_AbsVx << ',' << o.f32_AbsVy
                << "],\"legacy_speed_mps\":" << o.f32_Velocity << ",\"obb_corners\":[";
            for(unsigned j=0;j<8;++j) {
                if(j)out<<',';const auto& p=o.OBB_Corners_Body[j];
                out << '[' << p.x << ',' << p.y << ',' << p.z << ']';
            }
            out << "]}";
        }
        out << ']';
    });
}
}  // namespace pipeline_trace
