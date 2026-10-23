#pragma once
// Camera input boundary only. No ROS, inference, object matching or tracking.
#include <asmc_lidar/core/frame_time.hpp>
#include <deque>
#include <map>
#include <memory>
#include <mutex>

namespace asmc::lidar::camera {
struct Header { std::uint32_t sequence=0; Ns stamp_ns=0; std::string frame_id; };
struct Box {
  float xmin=0, ymin=0, xmax=0, ymax=0;
  std::string class_name, source_class_name;
  float confidence=0; bool confidence_valid=false;
};
struct Observation {
  Header header;
  std::uint32_t width=0, height=0;
  bool valid=false;
  std::vector<Box> boxes;
  std::string session, calibration_id, model_id, config_id;
  std::uint64_t generation=0;
};
struct Status {
  Ns stamp_ns=0, cutoff_ns=0;
  std::string session, model;
  std::uint64_t generation=0;
  std::uint8_t mode=0;
};
struct Calibration {
  std::string frame_id, id;
  std::uint32_t width=0, height=0;
  Pose base_from_camera=Pose::Identity();
  Eigen::Matrix3d intrinsic=Eigen::Matrix3d::Identity();
};
using Calibrations = std::map<std::string, Calibration>;
struct Limits {
  std::size_t capacity=8;
  Ns retention_ns=350000000, status_freshness_ns=350000000;
  Ns future_tolerance_ns=50000000;
};
struct Stored {
  Observation observation;
  Ns receipt_ns=0;
  std::uint64_t epoch=0, view_revision=0;
};
struct Candidate {
  std::shared_ptr<const Stored> record;
  Ns image_minus_scan_ns=0;
};
struct ViewResult {
  std::string reason; // ok, empty, invalid, missing, stale, no_time_match, ...
  std::vector<Candidate> candidates; // nearest first; never waits or consumes
};
struct Query {
  std::map<std::string, ViewResult> views;
};
class Buffer {
 public:
  explicit Buffer(Calibrations calibrations, Limits limits={});
  std::string status(const Status&, Ns ros_now, Ns steady_now);
  std::string observe(Observation, Ns ros_now, Ns steady_now);
  Query query(Ns scan_stamp, Ns max_difference, Ns ros_now, Ns steady_now);
  // Call immediately before applying a result computed outside the buffer lock.
  bool current(const Candidate&, Ns ros_now, Ns steady_now);
  std::map<std::string, std::uint64_t> counters() const;
  std::map<std::string, std::size_t> sizes() const;
  void close();
 private:
  struct View {
    std::deque<std::shared_ptr<const Stored>> records;
    Ns watermark=0;
    std::uint64_t revision=0;
    std::string reason="missing";
  };
  void clear_locked(const std::string& reason);
  void expire_locked(Ns ros_now, Ns steady_now);
  std::string gate_locked(Ns ros_now, Ns steady_now) const;
  std::string count_locked(const std::string& reason);
  mutable std::mutex mutex_;
  Calibrations calibrations_;
  Limits limits_;
  std::map<std::string, View> views_;
  std::map<std::string, std::uint64_t> counts_;
  Status status_;
  Ns status_receipt_=0;
  bool has_status_=false, closed_=false, identity_conflict_=false;
  std::uint64_t epoch_=0;
  std::deque<std::string> retired_sessions_;
  std::string model_id_, config_id_;
};
} // namespace asmc::lidar::camera
