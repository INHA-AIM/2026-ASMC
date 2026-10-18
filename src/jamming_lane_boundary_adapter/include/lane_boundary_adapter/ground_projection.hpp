#pragma once
// Consume a selected image-time plane; never read IMU or correct a plane twice.
#include <asmc_msgs/CameraGroundProjection.h>
#include <asmc_msgs/InferenceStatus.h>
#include <asmc_msgs/LaneBoundaryFamily.h>
#include <opencv2/core.hpp>
#include <deque>
#include <map>
#include <string>

namespace lane_boundary_adapter {
struct ProjectionMatch {
  bool valid = false;
  std::string reason = "waiting_for_projection_context";
  cv::Vec3d plane{};  // dz_dx, dz_dy, z_at_origin
  bool imu_corrected = false;
};

// Owned by the single ROS callback thread; callers using AsyncSpinner must
// serialize access, including their associated family/image histories.
class GroundProjectionBuffer {
 public:
  GroundProjectionBuffer(std::string config_id, std::map<std::string,std::string> calibration_ids,
                         cv::Vec3d nominal_plane);
  // True means the caller must clear its family and pending-image histories.
  bool status(const asmc_msgs::InferenceStatus&, ros::Time now, ros::SteadyTime receipt);
  bool add(const asmc_msgs::CameraGroundProjection::ConstPtr&, ros::Time now, ros::SteadyTime receipt);
  ProjectionMatch match(const asmc_msgs::LaneBoundaryFamily&, const std_msgs::Header& image,
                        const std::string& view, ros::Time now, ros::SteadyTime receipt) const;
  std::string gate(ros::Time now, ros::SteadyTime receipt) const;
  std::string last_error() const { return last_error_; }
  std::size_t size() const { return records_.size(); }
  std::uint64_t revision() const { return revision_; }
 private:
  struct Record { asmc_msgs::CameraGroundProjection::ConstPtr message; ros::SteadyTime receipt; };
  std::string config_id_, session_, projection_session_, last_error_;
  std::map<std::string,std::string> calibration_ids_;
  cv::Vec3d nominal_;
  asmc_msgs::InferenceStatus status_;
  bool has_status_ = false;
  ros::SteadyTime status_receipt_;
  std::deque<Record> records_;
  std::deque<std::string> retired_sessions_;
  std::deque<ros::Time> retired_stamps_;
  std::uint64_t revision_ = 0;
  void clear();
};

// The vertex lies on the SAME image-time plane used to project that camera's
// semantic evidence. Family reference xyz is separately checked against its
// reference plane; this is not nominal-height replacement or an ego-motion warp.
cv::Vec3d onPlane(const cv::Vec3d& plane, double x, double y);
cv::Matx33d groundHomography(const cv::Matx33d& intrinsic, const cv::Matx33d& rotation,
                           const cv::Vec3d& translation, const cv::Vec3d& plane);
bool projectGroundPoint(const cv::Matx33d& intrinsic, const cv::Matx33d& rotation,
                        const cv::Vec3d& translation, const cv::Vec3d& plane,
                        double x, double y, cv::Point2d& pixel);
}  // namespace lane_boundary_adapter
