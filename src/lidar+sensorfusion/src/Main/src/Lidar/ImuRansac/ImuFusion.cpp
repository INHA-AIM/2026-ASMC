// #include <ros/ros.h>
// #include <iostream>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/ModelCoefficients.h>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>

// #include <pcl/filters/extract_indices.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/filters/statistical_outlier_removal.h>

// #include <pcl/kdtree/kdtree.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/segmentation/extract_clusters.h>

// #include <sensor_msgs/Imu.h>
// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include <tf/transform_datatypes.h>  //퀀터니언에서 롤피치요로 변환
// #include <boost/bind.hpp>

#include "ImuRansac.hpp"


// double roll;
// double previous_pitch = 0;
// double current_pitch;
// double delta_pitch;
// double yaw;
// double current_speed;
// float vx;
// bool imu_bool = false;


void CorrectingPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr& pcl_InputCloud3,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr& pcl_OutputCloud3,const GPS_IMU& st_GPSIMU) {
    float32 f64_Current_pitch=st_GPSIMU.st_IMU.f32_Pitch_rad;
    float32 f64_CosPitch = cos(f64_Current_pitch);  // 회전각autonav@autonav:~/Documen
    float32 f64_SinPitch = sin(f64_Current_pitch);
    // cout<<"pitch:"<<f64_Current_pitch<<endl;
    Eigen::Matrix3f R;
    R << f64_CosPitch,  0, f64_SinPitch,
         0,          1, 0,
        -f64_SinPitch, 0, f64_CosPitch;

    pcl_OutputCloud3->points.clear();  // 이전 포인트 제거

    for (const auto& point : pcl_InputCloud3->points) {
        Eigen::Vector3f pt(point.x, point.y, point.z);
        Eigen::Vector3f corrected = R * pt;

        pcl::PointXYZI pcl_NewPoint;
        pcl_NewPoint.x = corrected.x();
        pcl_NewPoint.y = corrected.y();
        pcl_NewPoint.z = corrected.z();

        pcl_OutputCloud3->points.push_back(pcl_NewPoint);
    }

}


// void imuCallBack(const sensor_msgs::ImuConstPtr& imu_msg){
//     tf::Quaternion q(
//         imu_msg->orientation.x,
//         imu_msg->orientation.y,
//         imu_msg->orientation.z,
//         imu_msg->orientation.w
//     );

//     tf::Matrix3x3(q).getRPY(roll, f64_Current_pitch, yaw);  // 여기서 변환

//     imu_bool = true;
// }


