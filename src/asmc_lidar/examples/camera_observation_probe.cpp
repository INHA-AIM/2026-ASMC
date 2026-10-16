// Read-only handoff example. Does not publish BBox, LiDAR objects or control.
#include <asmc_lidar/camera_receiver.hpp>
#include <asmc_lidar/camera_calibration.hpp>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <sstream>

int main(int argc,char** argv) {
  ros::init(argc,argv,"lidar_camera_probe");ros::NodeHandle nh,priv("~");
  std::string path;
  if(!priv.getParam("calibration_file",path)) {ROS_FATAL("Set ~calibration_file to the camera-owned cameras_v3.yaml");return 2;}
  try {
    const auto calibrations=asmc::lidar::camera::load_calibrations(path);
    asmc::lidar::camera::Receiver receiver(nh,calibrations);
    auto output=priv.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics",1);
    ros::AsyncSpinner spinner(2);spinner.start();ros::WallRate rate(5);
    while(ros::ok()) {
      const auto now=ros::Time::now();
      const auto q=receiver.buffer()->query(now.toNSec(),50000000,now.toNSec(),asmc::lidar::camera::steady_now_ns());
      diagnostic_msgs::DiagnosticArray message;message.header.stamp=now;
      for(const auto& entry:q.views) {
        diagnostic_msgs::DiagnosticStatus status;status.name="camera_input/"+entry.first;
        status.message=entry.second.reason;status.level=entry.second.candidates.empty()?1:0;
        diagnostic_msgs::KeyValue value;value.key="candidate_count";value.value=std::to_string(entry.second.candidates.size());status.values.push_back(value);
        message.status.push_back(status);
      }
      diagnostic_msgs::DiagnosticStatus counts;counts.name="camera_input/counters";
      for(const auto& entry:receiver.buffer()->counters()) {
        diagnostic_msgs::KeyValue value;value.key=entry.first;value.value=std::to_string(entry.second);counts.values.push_back(value);
      }
      message.status.push_back(counts);output.publish(message);rate.sleep();
    }
    receiver.close();spinner.stop();
  }catch(const std::exception& e){ROS_FATAL_STREAM(e.what());return 2;}
  return 0;
}
