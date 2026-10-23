#include <asmc_lidar/ros/camera_receiver.hpp>
#include <chrono>

namespace asmc::lidar::camera {
Ns steady_now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
Observation from_ros(const asmc_msgs::CameraObjectObservation& m) {
  Observation o;const auto& d=m.detections;
  o.header={d.header.seq,static_cast<Ns>(d.header.stamp.toNSec()),d.header.frame_id};
  o.width=d.image_width;o.height=d.image_height;o.valid=d.valid;
  o.session=m.producer_session_id;o.generation=m.generation;
  o.calibration_id=m.calibration_id;o.model_id=m.model_id;o.config_id=m.config_id;
  o.boxes.reserve(d.boxes.size());
  for(const auto& b:d.boxes)o.boxes.push_back({b.xmin,b.ymin,b.xmax,b.ymax,b.class_name,b.source_class_name,b.confidence,bool(b.confidence_valid)});
  return o;
}
Status from_ros(const asmc_msgs::InferenceStatus& m) {
  return {static_cast<Ns>(m.header.stamp.toNSec()),static_cast<Ns>(m.source_cutoff.toNSec()),m.producer_session_id,m.model,m.generation,m.mode};
}
Receiver::Receiver(ros::NodeHandle& nh, Calibrations c, Limits limits,
                   const std::string& observations, const std::string& status):
    buffer_(std::make_shared<Buffer>(std::move(c),limits)) {
  const auto buffer=buffer_;
  status_=nh.subscribe<asmc_msgs::InferenceStatus>(status,1,
    [buffer](const asmc_msgs::InferenceStatus::ConstPtr& m){
      buffer->status(from_ros(*m),ros::Time::now().toNSec(),steady_now_ns());
    });
  observations_=nh.subscribe<asmc_msgs::CameraObjectObservation>(observations,24,
    [buffer](const asmc_msgs::CameraObjectObservation::ConstPtr& m){
      const auto received=steady_now_ns();const auto now=ros::Time::now().toNSec();
      buffer->observe(from_ros(*m),now,received);
    });
}
Receiver::~Receiver(){close();}
void Receiver::close(){observations_.shutdown();status_.shutdown();buffer_->close();}
} // namespace asmc::lidar::camera
