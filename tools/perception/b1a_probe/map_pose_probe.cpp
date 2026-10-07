// Diagnostic against the original map adapter; no production change.
#define main b1a_original_evaluator_main
#include "../Main/tools/tunnel_filter_offline_eval.cpp"
#undef main
#include <Eigen/Geometry>
#include <PlannerInterface/PlannerObjectAdapter.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "b1a_map_pose_probe");
    int failures=0;
    for (const float roll : {0.f, .15f}) {
        GPS_IMU pose{};
        pose.st_IMU.f32_Roll_rad=roll;
        pose.st_IMU.f32_Pitch_rad=-.2f;
        pose.st_IMU.f32_Yaw_rad=.3f;
        VehicleSpec vehicle{};
        const Eigen::Vector3d raw(8.,2.,1.);
        const auto l=tunnel_filter::LevelingRotation(roll,-.2f,true,true);
        Eigen::Matrix3d level;
        level << l[0],l[1],l[2],l[3],l[4],l[5],l[6],l[7],l[8];
        const Eigen::Vector3d leveled=level*raw;
        const auto actual=RotateLidarVectorToMap({leveled.x(),leveled.y(),leveled.z()},pose,vehicle);
        const Eigen::Matrix3d map=(Eigen::AngleAxisd(pose.st_IMU.f32_Yaw_rad,Eigen::Vector3d::UnitZ())*
            Eigen::AngleAxisd(pose.st_IMU.f32_Pitch_rad,Eigen::Vector3d::UnitY())*
            Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())).toRotationMatrix();
        const double error=(Eigen::Vector3d(actual.x,actual.y,actual.z)-map*raw).norm();
        const bool reproduced=roll==0.f ? error<1e-5 : error>.1;
        std::cout<<"B1A_MAP_PROBE {\"roll_rad\":"<<roll<<",\"vector_error_m\":"<<error
                 <<",\"diagnostic_reproduced\":"<<(reproduced?"true":"false")
                 <<",\"coordinate_contract_pass\":"<<(error<1e-5?"true":"false")<<"}"<<std::endl;
        if(!reproduced)++failures;
    }
    return failures?1:0;
}
