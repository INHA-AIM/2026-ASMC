// lidar_pipeline_node: ROS adapter of the asmc_lidar shadow pipeline.
//
// Inputs: PointCloud2, sensor_msgs/Imu, morai EgoVehicleStatus and GPSMessage,
// and the canonical camera observation/status topics (camera::Receiver).
// Outputs, by default only under /migration/lidar/*: typed local/map
// LidarObjectArray, 1 Hz diagnostics, a per-scan JSON trace (serialized only
// while subscribed), with ~publish_tf base_link -> ~tf_child_frame and, with
// ~debug_enabled (default false), rate-limited stage clouds and a scan JSON
// under ~debug_topic_prefix for the read-only web monitor (DebugPublisher).
//
// Isolation: every output name is resolved (remaps applied) and the node refuses
// to start on an operational name unless ~allow_operational_output. Its private
// reset service retires processing state; it calls no external mode/control
// service, sets no parameter and starts no driver or bridge.
//
// Threads (Main TemporalThread semantics): an AsyncSpinner serves all
// callbacks. IMU/CAN/GPS go straight into LidarRuntime. The LiDAR callback
// converts, gates (AcceptScan) and stores the scan in a latest-wins slot; one
// worker runs ProcessScan and publishes.

#include <asmc_lidar/geometry/camera_calibration.hpp>
#include <asmc_lidar/ros/camera_receiver.hpp>
#include <asmc_lidar/core/frame_time.hpp>
#include <asmc_lidar/fusion/front_camera_fusion.hpp>
#include <asmc_lidar/ros/scan_trace.hpp>
#include <asmc_lidar/ros/ros_inputs.hpp>
#include <asmc_lidar/ros/runtime.hpp>
#include <asmc_lidar/ros/debug_publisher.hpp>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/TransformStamped.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/GPSMessage.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <asmc_msgs/LidarObservation.h>
#include <asmc_msgs/LidarTrackingObservation.h>
#include <set>
#include <asmc_lidar/core/processing_epoch.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <map>
#include <atomic>
#include <condition_variable>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace asmc {
namespace lidar {
namespace runtime {
namespace {

namespace fs = std::filesystem;
namespace fusion = pipeline::fusion;

constexpr char kProducer[] = "lidar_pipeline_node";
constexpr char kVelocityStatus[] = "unproven_moving_sensor_derivative";  // Main diagnostics value
constexpr std::size_t kTimingWindow = 100;  // scans in the processing-time window

struct StartupRefused : std::runtime_error {
  using std::runtime_error::runtime_error;
};

// Topics owned by operational producers/consumers (Main, planner, camera adapter).
bool OperationalTopic(const std::string& resolved) {
  static const char* const kOperational[] = {"/last_lidar", "/lidar_nogps", "/YOLO/BBox",
                                             "/tf", "/tf_static"};
  for (const char* topic : kOperational)
    if (resolved == topic) return true;
  return resolved.rfind("/planning/", 0) == 0 || resolved.rfind("/perception/lidar/", 0) == 0;
}

std::string StripSlash(const std::string& frame) {
  return !frame.empty() && frame.front() == '/' ? frame.substr(1) : frame;
}

bool IsPackage(const fs::path& dir, const std::string& name) {
  std::ifstream in(dir / "package.xml");
  if (!in) return false;
  std::stringstream text;
  text << in.rdbuf();
  return text.str().find("<name>" + name + "</name>") != std::string::npos;
}

std::optional<fs::path> SearchPackage(const fs::path& dir, const std::string& name, int depth) {
  std::error_code ec;
  if (!fs::is_directory(dir, ec) || fs::exists(dir / "CATKIN_IGNORE", ec)) return std::nullopt;
  if (fs::exists(dir / "package.xml", ec))  // packages do not nest
    return IsPackage(dir, name) ? std::optional<fs::path>(dir) : std::nullopt;
  if (depth >= 2) return std::nullopt;
  for (fs::directory_iterator it(dir, fs::directory_options::skip_permission_denied, ec), end;
       !ec && it != end; it.increment(ec)) {
    if (auto found = SearchPackage(it->path(), name, depth + 1)) return found;
  }
  return std::nullopt;
}

// $(find name) without roslib (not a dependency of this package): a bounded
// rospack-like crawl of ROS_PACKAGE_PATH that skips CATKIN_IGNORE trees.
std::string FindPackageDir(const std::string& name) {
  const char* env = std::getenv("ROS_PACKAGE_PATH");
  if (!env) return {};
  std::stringstream paths(env);
  std::string entry;
  while (std::getline(paths, entry, ':')) {
    if (entry.empty()) continue;
    if (auto found = SearchPackage(fs::path(entry), name, 0)) return found->string();
  }
  return {};
}

double Percentile(std::vector<double> values, double q) {
  if (values.empty()) return 0.;
  std::sort(values.begin(), values.end());
  const auto rank = static_cast<std::size_t>(std::ceil(q * static_cast<double>(values.size())));
  return values[std::min(values.size() - 1, rank ? rank - 1 : 0)];
}

class LidarPipelineNode {
 public:
  // Ported from Main src/main.cpp:601-647 (main: parameters, Configure, publishers) (8f93c22e).
  LidarPipelineNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    ReadTopics();
    Guard();  // before any file is read or anything is advertised
    LoadConfig();
    SetUpCamera();
    SetUpDebug();
    FinalizeRuntimeIdentity(config_, front_ ? front_->id : "");
    runtime_ = std::make_shared<LidarRuntime>(config_, camera_, front_);
    Advertise();
    Subscribe();
    reset_service_ = pnh_.advertiseService("reset", &LidarPipelineNode::Reset, this);
  }

  // Ported from Main src/main.cpp:648-688 (threads, waitForShutdown, joins) (8f93c22e).
  int Run() {
    worker_ = std::thread([this] { Work(); });
    ros::AsyncSpinner spinner(spinner_threads_);
    spinner.start();
    ROS_INFO_STREAM("lidar_pipeline: running; outputs " << local_topic_ << ", " << map_topic_ << ", "
                    << diagnostics_topic_ << ", " << trace_topic_ << (publish_tf_ ? ", /tf" : "")
                    << "; camera fusion " << (camera_ ? std::string("on") : "off (" + camera_reason_ + ")")
                    << "; debug stages " << (debug_ ? "on under " + debug_config_.topic_prefix : std::string("off")));
    ros::waitForShutdown();
    spinner.stop();  // joins the callback threads: no callback touches the runtime after this
    {
      std::lock_guard<std::mutex> lock(slot_mutex_);
      stop_ = true;
    }
    slot_cv_.notify_all();
    if (worker_.joinable()) worker_.join();
    if (receiver_) receiver_->close();
    std::lock_guard<std::mutex> lock(stats_mutex_);
    // rosconsole may already be down after shutdown; print the totals directly.
    std::printf("[ASMC_LIDAR_PIPELINE_TOTAL] received=%llu processed=%llu invalid=%llu stamp_errors=%llu "
                "rejected=%llu overwritten=%llu\n",
                static_cast<unsigned long long>(points_received_.load()),
                static_cast<unsigned long long>(stats_.processed), static_cast<unsigned long long>(stats_.invalid),
                static_cast<unsigned long long>(stats_.stamp_errors),
                static_cast<unsigned long long>(runtime_->rejected_scans()),
                static_cast<unsigned long long>(overwritten_scans_.load()));
    return 0;
  }

 private:
  struct PendingScan {
    pipeline::ScanInput scan;
    std::shared_ptr<LidarRuntime> runtime;
    std::int64_t receipt_steady_ns = 0;
    std::int64_t receipt_ros_ns = 0;
    std::uint64_t seq = 0;
    std::string sensor_frame;
  };
  struct Stats {
    std::uint64_t processed = 0, invalid = 0, stamp_errors = 0, scans_processed = 0;
    std::string input_reason = "waiting_for_input", map_reason = "gps_missing", pose_reason;
    bool camera_active = false;
    std::string camera_reason;
    fusion::FusionCounters fusion;
    std::size_t pending_scans = 0;
    std::deque<double> total_ms, delivery_ms;
    struct AuditSample {
      double ms = 0;
      std::map<std::string, std::size_t> decisions, reasons;
    };
    std::deque<AuditSample> audit_window;
    std::string audit_reason = "off";
    bool delivery_filter_applied=false;
    std::size_t delivery_input=0,delivery_excluded=0,delivery_output=0;

    std::int64_t last_stamp_ns = 0, last_receipt_ns = 0, last_processing_ns = 0;
    std::int64_t last_output_steady_ns = 0;
    std::int64_t gps_sample_stamp_ns = 0, gps_sample_arrival_ns = 0;
    double gps_age_ms = 0.;
    std::uint64_t scan_sequence = 0;
    std::string last_reject_reason;
  };

  std::shared_ptr<LidarRuntime> RuntimeSnapshot() {
    std::lock_guard<std::mutex> lock(epoch_mutex_);
    return runtime_;
  }

  bool Reset(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& reply) {
    auto next = std::make_shared<LidarRuntime>(config_, camera_, front_);
    std::lock_guard<std::mutex> epoch(epoch_mutex_);
    epoch_.Reset();
    runtime_ = std::move(next);  // old worker owns its retired runtime until it finishes
    {
      std::lock_guard<std::mutex> slot_lock(slot_mutex_);
      slot_.reset();
    }
    {
      std::lock_guard<std::mutex> cache_lock(cache_mutex_);
      cache_ = trace::LocalInputCache{};
    }
    {
      std::lock_guard<std::mutex> stats_lock(stats_mutex_);
      stats_ = Stats{};
    }
    reply.success = true;
    reply.message = "processing generation " + std::to_string(epoch_.generation());
    return true;
  }

  asmc_msgs::LidarObservation Observation(const asmc_msgs::LidarObjectArray& objects,
                                         const PendingScan& pending) const {
    asmc_msgs::LidarObservation observation;
    observation.objects = objects;
    observation.producer_session_id = epoch_.session();
    observation.generation = pending.scan.generation;
    observation.scan_sequence = pending.scan.scan_sequence;
    observation.config_id = config_.config_id;
    observation.calibration_id = config_.calibration_id;
    return observation;
  }

  void PublishObservation(const ros::Publisher& publisher, const asmc_msgs::LidarObjectArray& objects,
                          const PendingScan& pending) {
    if (publisher.getNumSubscribers()) publisher.publish(Observation(objects, pending));
  }

  void PublishTracking(const ros::Publisher& publisher, const asmc_msgs::LidarObjectArray& objects,
                       const std::vector<asmc_msgs::LidarTrackingState>& tracking,
                       const PendingScan& pending) {
    if (!publisher.getNumSubscribers()) return;
    asmc_msgs::LidarTrackingObservation message;
    message.observation = Observation(objects, pending);
    message.tracking = tracking;
    publisher.publish(message);
  }

  void ReadTopics() {
    pnh_.param<std::string>("points_topic", points_topic_, "/velodyne_points");
    pnh_.param<std::string>("imu_topic", imu_topic_, "/imu");
    pnh_.param<std::string>("can_topic", can_topic_, "/Ego_topic");
    pnh_.param<std::string>("gps_topic", gps_topic_, "/gps");
    pnh_.param<std::string>("camera_observations_topic", camera_observations_topic_,
                            "/perception/camera/objects/observations");
    pnh_.param<std::string>("camera_status_topic", camera_status_topic_, "/perception/camera/objects/status");
    pnh_.param<std::string>("objects_local_topic", local_topic_, "objects_local");
    pnh_.param<std::string>("objects_map_topic", map_topic_, "objects_map");
    pnh_.param<std::string>("observations_local_topic", local_observation_topic_, "observations_local");
    pnh_.param<std::string>("observations_map_topic", map_observation_topic_, "observations_map");
    pnh_.param<std::string>("tracks_local_topic", local_tracking_topic_, "tracks_local");
    pnh_.param<std::string>("tracks_map_topic", map_tracking_topic_, "tracks_map");
    pnh_.param<std::string>("diagnostics_topic", diagnostics_topic_, "diagnostics");
    pnh_.param<std::string>("trace_topic", trace_topic_, "trace");
    pnh_.param("publish_tf", publish_tf_, false);
    pnh_.param<std::string>("tf_child_frame", tf_child_frame_, "lidar_pipeline/lidar_leveled");
    pnh_.param("allow_operational_output", allow_operational_, false);
    pnh_.param("spinner_threads", spinner_threads_, 4);
    spinner_threads_ = std::max(2, spinner_threads_);
    // Monitoring-only stage output, default OFF (see ros/debug_publisher.hpp).
    pnh_.param("debug_enabled", debug_config_.enabled, false);
    pnh_.param<std::string>("debug_topic_prefix", debug_config_.topic_prefix, "debug");
    pnh_.param("debug_rate_hz", debug_config_.rate_hz, 5.0);
    int debug_max_points = 20000;
    pnh_.param("debug_max_points", debug_max_points, 20000);
    debug_config_.max_points = static_cast<std::size_t>(std::max(1, debug_max_points));
    debug_config_.topic_prefix = nh_.resolveName(debug_config_.topic_prefix);
    debug_config_.leveled_frame = tf_child_frame_;
  }

  // Refuses operational outputs. Names are resolved exactly as advertise()
  // resolves them (namespace and remaps applied).
  void Guard() {
    std::vector<std::string> refused;
    std::set<std::string> output_names;
    std::vector<const std::string*> outputs = {&local_topic_, &map_topic_, &diagnostics_topic_, &trace_topic_, &local_observation_topic_, &map_observation_topic_, &local_tracking_topic_, &map_tracking_topic_};
    const auto debug_topics = DebugPublisher::TopicNames(debug_config_.topic_prefix);
    for (const auto& topic : debug_topics) outputs.push_back(&topic);
    for (const std::string* topic : outputs) {
      const std::string resolved = nh_.resolveName(*topic);
      if (!output_names.insert(resolved).second)
        throw StartupRefused("output refused: duplicate topic: " + resolved);
      if (OperationalTopic(resolved)) refused.push_back(*topic + " -> " + resolved);
      for (const std::string* input : {&points_topic_, &imu_topic_, &can_topic_, &gps_topic_,
                                       &camera_observations_topic_, &camera_status_topic_})
        if (resolved == nh_.resolveName(*input))
          throw StartupRefused("output refused: sensor/camera input: " + resolved);
    }
    const std::string child = StripSlash(tf_child_frame_);
    if (child.empty()) throw StartupRefused("~tf_child_frame must not be empty");
    if (child == "lidar_leveled") refused.push_back("tf_child_frame " + tf_child_frame_ + " (Main's frame)");
    if (child == "base_link") refused.push_back("tf_child_frame " + tf_child_frame_ + " (equals the parent)");
    if (child == "map" || child == "velodyne")
      refused.push_back("tf_child_frame " + tf_child_frame_ + " (an existing frame)");
    if (refused.empty()) return;
    std::string list;
    for (const auto& item : refused) list += (list.empty() ? "" : ", ") + item;
    if (!allow_operational_) {
      throw StartupRefused("operational output refused (set ~allow_operational_output:=true to override): " + list);
    }
    ROS_WARN_STREAM("lidar_pipeline: operational output ALLOWED by ~allow_operational_output: " << list);
  }

  void LoadConfig() {
    std::string package_dir;
    pnh_.param<std::string>("package_dir", package_dir, "");
    if (package_dir.empty()) package_dir = FindPackageDir("asmc_lidar");
    std::string path;
    pnh_.param<std::string>("pipeline_config", path, "");
    if (path.empty()) {
      if (package_dir.empty())
        throw std::runtime_error("set ~pipeline_config (asmc_lidar not found on ROS_PACKAGE_PATH)");
      path = (fs::path(package_dir) / "config" / "lidar_pipeline.yaml").string();
    }
    // Relative paths inside the file resolve against the package directory;
    // without one, against the directory that holds config/.
    if (package_dir.empty()) package_dir = fs::path(path).parent_path().parent_path().string();
    config_ = LoadRuntimeConfig(path, package_dir);
    bool flag = false;
    if (pnh_.getParam("lidar_level_roll", flag)) config_.pipeline.use_roll = flag;
    if (pnh_.getParam("enable_camera_fusion", flag)) config_.enable_camera_fusion = flag;
    if (pnh_.getParam("experimental_tunnel_filter", flag)) config_.pipeline.tunnel.enabled = flag;
    // The trace topic never carries the debug payload (detections/clusters/digests).
    config_.pipeline.capture_debug = false;
  }

  // Camera fusion needs the camera-owned calibration; without it the node runs
  // LiDAR-only and reports why (it never fails start-up for the camera). LiDAR-only
  // keeps ~enable_camera_fusion (Main enable_sensor_fusion) with no camera source,
  // i.e. Main with no camera traffic: classes stay unknown, FusionLiDAR still runs.
  void SetUpCamera() {
    if (!config_.enable_camera_fusion) {
      camera_reason_ = "disabled_by_config";
      return;
    }
    std::string path;
    pnh_.param<std::string>("camera_calibration_config", path, "");
    if (path.empty()) {
      const std::string camera_dir = FindPackageDir("asmc_camera");
      if (!camera_dir.empty()) path = (fs::path(camera_dir) / "config" / "cameras_v3.yaml").string();
    }
    try {
      if (path.empty()) throw std::runtime_error("no ~camera_calibration_config and asmc_camera not found");
      calibrations_ = camera::load_calibrations(path);
      const auto front = calibrations_.find("camera_front");
      if (front == calibrations_.end()) throw std::runtime_error("camera_front missing");
      front_ = &front->second;
      receiver_ = std::make_unique<camera::Receiver>(nh_, calibrations_, camera::Limits{},
                                                      camera_observations_topic_, camera_status_topic_);
      camera_ = std::make_shared<fusion::BufferCameraSource>(receiver_->buffer(), "camera_front");
      camera_reason_ = "enabled";
    } catch (const std::exception& e) {
      receiver_.reset();
      camera_.reset();
      front_ = nullptr;
      camera_reason_ = std::string("calibration_unreadable: ") + e.what();
      ROS_WARN_STREAM("lidar_pipeline: LiDAR-only, camera calibration '" << path << "': " << e.what());
    }
  }

  // Monitoring-only stage publisher; the pipeline observer pointer is set in the
  // config the runtime is constructed from (Pipeline keeps a copy of it).
  void SetUpDebug() {
    if (!debug_config_.enabled) return;
    debug_ = std::make_unique<DebugPublisher>(nh_, debug_config_, config_);
    config_.pipeline.observer = debug_.get();
  }

  // Ported from Main src/Geometry/FrameBoundaryPublisher.cpp:36-43 (Publisher ctor) (8f93c22e).
  void Advertise() {
    local_observation_pub_ = nh_.advertise<asmc_msgs::LidarObservation>(local_observation_topic_, 5);
    map_observation_pub_ = nh_.advertise<asmc_msgs::LidarObservation>(map_observation_topic_, 5);
    local_tracking_pub_ = nh_.advertise<asmc_msgs::LidarTrackingObservation>(local_tracking_topic_, 2);
    map_tracking_pub_ = nh_.advertise<asmc_msgs::LidarTrackingObservation>(map_tracking_topic_, 2);
    local_pub_ = nh_.advertise<asmc_msgs::LidarObjectArray>(local_topic_, 5);
    map_pub_ = nh_.advertise<asmc_msgs::LidarObjectArray>(map_topic_, 5);
    diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic_, 2);
    trace_pub_ = nh_.advertise<std_msgs::String>(trace_topic_, 10);
    if (publish_tf_) tf_ = std::make_unique<tf2_ros::TransformBroadcaster>();
    diagnostics_timer_ = nh_.createWallTimer(ros::WallDuration(1.), &LidarPipelineNode::OnDiagnostics, this);
  }

  // Ported from Main src/Parser/Parser.cpp:257-268, 357-396, 497-517, 553-564 (subscriptions) (8f93c22e).
  void Subscribe() {
    // Main subscribes every sensor with queue 1.
    const auto hints = ros::TransportHints().tcpNoDelay();
    points_sub_ = nh_.subscribe(points_topic_, 1, &LidarPipelineNode::OnPoints, this, hints);
    imu_sub_ = nh_.subscribe(imu_topic_, 1, &LidarPipelineNode::OnImu, this, hints);
    can_sub_ = nh_.subscribe(can_topic_, 1, &LidarPipelineNode::OnCan, this, hints);
    gps_sub_ = nh_.subscribe(gps_topic_, 1, &LidarPipelineNode::OnGps, this, hints);
  }

  static std::int64_t RosNow() { return static_cast<std::int64_t>(ros::Time::now().toNSec()); }

  // Ported from Main src/Parser/Parser.cpp:566-612 (IMUParser_Callback transport) (8f93c22e).
  void OnImu(const sensor_msgs::Imu::ConstPtr& msg) {
    const auto runtime = RuntimeSnapshot();
    const std::int64_t receipt = camera::steady_now_ns();
    const auto input = ImuInputFromMsg(*msg);
    const auto before = runtime->imu_rejected();
    runtime->OnImu(input, receipt, RosNow());
    if (runtime->imu_rejected() == before) {
      std::lock_guard<std::mutex> epoch(epoch_mutex_);
      if (runtime != runtime_) return;
      std::lock_guard<std::mutex> lock(cache_mutex_);
      cache_.AddImu(input.stamp_ns, pipeline::ImuFromInput(input));
    }
  }

  // Ported from Main src/Parser/Parser.cpp:519-543 (CANParser_Callback transport) (8f93c22e).
  void OnCan(const morai_msgs::EgoVehicleStatus::ConstPtr& msg) {
    const auto runtime = RuntimeSnapshot();
    const std::int64_t receipt = camera::steady_now_ns();
    const auto input = CanInputFromMsg(*msg);
    const auto before = runtime->can_rejected();
    runtime->OnCan(input, receipt, RosNow());
    if (runtime->can_rejected() == before) {
      std::lock_guard<std::mutex> epoch(epoch_mutex_);
      if (runtime != runtime_) return;
      std::lock_guard<std::mutex> lock(cache_mutex_);
      cache_.AddCan(input.stamp_ns, pipeline::CanFromInput(input));
    }
  }

  // Ported from Main src/Parser/Parser.cpp:398-468 (GPSParser_Callback transport) (8f93c22e).
  void OnGps(const morai_msgs::GPSMessage::ConstPtr& msg) {
    const auto runtime = RuntimeSnapshot();
    runtime->OnGps(GpsInputFromMsg(*msg), camera::steady_now_ns(), RosNow());
  }

  // Ported from Main src/Parser/Parser.cpp:270-350 (LiDARParser_Callback, common gates) (8f93c22e).
  void OnPoints(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    const std::int64_t receipt = camera::steady_now_ns();
    const std::int64_t receipt_ros = RosNow();
    const std::uint64_t seq = ++points_received_;
    const std::int64_t stamp = StampNs(msg->header.stamp);
    PendingScan pending;
    pending.seq = seq;
    {
      std::lock_guard<std::mutex> epoch(epoch_mutex_);
      pending.runtime = runtime_;
      pending.scan.generation = epoch_.generation();
    }
    pending.receipt_steady_ns = receipt;
    pending.receipt_ros_ns = receipt_ros;
    pending.sensor_frame = msg->header.frame_id;
    std::string reason;
    bool accepted = false;
    try {
      pending.scan.points = CloudFromMsg(*msg);
    } catch (const std::exception& e) {
      reason = std::string("point_conversion_failed: ") + e.what();
      ++conversion_errors_;
    }
    if (pending.scan.points) accepted = pending.runtime->AcceptScan(stamp, pending.scan.points->size(), &reason);
    if (!accepted) {
      std::lock_guard<std::mutex> epoch(epoch_mutex_);
      if (!epoch_.IsCurrent(pending.scan.generation)) return;
      {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.last_reject_reason = reason;
      }
      if (trace_pub_.getNumSubscribers() > 0) {
        trace::LineInput line;
        line.producer = kProducer;
        line.seq = seq;
        line.stamp_ns = stamp;
        line.arrival_ns = receipt_ros;
        line.accept_reason = reason;
        std_msgs::String text;
        text.data = trace::Line(line);
        trace_pub_.publish(text);
      }
      return;
    }
    // Main FrameBoundary scan time: ROS source stamp, steady receipt.
    pending.scan.time = asmc::lidar::Observation{stamp, receipt, receipt, "ros", config_.stamp_provenance, "steady"};
    pending.scan.scan_sequence = seq;  // former per-scan generation; includes rejected-input gaps
    {
      std::lock_guard<std::mutex> epoch(epoch_mutex_);
      if (!epoch_.IsCurrent(pending.scan.generation)) return;
      std::lock_guard<std::mutex> lock(slot_mutex_);
      if (slot_) ++overwritten_scans_;  // latest frame wins (Main TemporalThread)
      slot_ = std::move(pending);
    }
    slot_cv_.notify_one();
  }

  // Ported from Main src/main.cpp:447-477 (TemporalThread wait, latest frame wins) (8f93c22e).
  void Work() {
    while (true) {
      PendingScan pending;
      {
        std::unique_lock<std::mutex> lock(slot_mutex_);
        slot_cv_.wait(lock, [this] { return stop_ || slot_.has_value(); });
        if (stop_) return;
        pending = std::move(*slot_);
        slot_.reset();
      }
      try {
        Process(pending);
      } catch (const std::exception& e) {
        ROS_ERROR_STREAM_THROTTLE(1., "lidar_pipeline: scan " << pending.scan.time.stamp_ns
                                                                << " failed: " << e.what());
      }
    }
  }

  // Ported from Main src/main.cpp:483-540 (TemporalThread scan step; outputs via FrameBoundaryPublisher) (8f93c22e).
  void Process(PendingScan& pending) {
    const std::int64_t start = camera::steady_now_ns();
    pending.scan.time.processing_ns = start;
    ScanTimes times;
    times.receipt_ns = pending.receipt_steady_ns;
    times.processing_ns = start;
    times.decision_ns = start;
    times.ros_now_ns = RosNow();
    times.steady_now_ns = start;
    times.refresh_before_fusion = [](ScanTimes& now) {
      now.steady_now_ns = camera::steady_now_ns();
      now.decision_ns = now.steady_now_ns;
      now.ros_now_ns = RosNow();
    };
    if (debug_) debug_->SetScanContext(pending.sensor_frame,pending.seq,pending.receipt_ros_ns,epoch_.session(),pending.scan.generation,config_.config_id);
    const ScanOutput out = pending.runtime->ProcessScan(pending.scan, config_.local_wait_timeout_s, times);
    if (!ros::ok()) return;
    // Reset and publishing share this short lock. Computation never holds it.
    std::lock_guard<std::mutex> epoch(epoch_mutex_);
    if (!epoch_.IsCurrent(pending.scan.generation)) { ++retired_results_; return; }
    if (out.arrays.accepted) {
      if (local_pub_.getNumSubscribers()) local_pub_.publish(out.arrays.local);
      if (map_pub_.getNumSubscribers()) map_pub_.publish(out.arrays.map);
      PublishObservation(local_observation_pub_, out.arrays.local, pending);
      PublishObservation(map_observation_pub_, out.arrays.map, pending);
      PublishTracking(local_tracking_pub_, out.arrays.local, out.arrays.local_tracking, pending);
      PublishTracking(map_tracking_pub_, out.arrays.map, out.arrays.map_tracking, pending);
    }
    const double delivery_ms = double(camera::steady_now_ns() - pending.receipt_steady_ns) / 1e6;
    if (tf_ && out.arrays.accepted && out.arrays.observed) PublishTf(out);
    Record(pending, out, delivery_ms);
    if (trace_pub_.getNumSubscribers() > 0) PublishTrace(pending, out, delivery_ms);
    if (debug_) debug_->Finish(pending.scan.time.stamp_ns, out);
  }

  // Ported from Main src/Geometry/FrameBoundaryPublisher.cpp:80-87 (TF; child renamed, opt-in) (8f93c22e).
  void PublishTf(const ScanOutput& out) {
    const auto& base = out.arrays.base_from_level;
    geometry_msgs::TransformStamped tf;
    tf.header.stamp.fromNSec(static_cast<std::uint64_t>(out.result.context.time.stamp_ns));
    tf.header.frame_id = "base_link";
    tf.child_frame_id = tf_child_frame_;
    tf.transform.translation.x = base.translation().x();
    tf.transform.translation.y = base.translation().y();
    tf.transform.translation.z = base.translation().z();
    const Eigen::Quaterniond q(base.rotation());
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_->sendTransform(tf);
  }

  void Record(const PendingScan& pending, const ScanOutput& out, double delivery_ms) {
    const auto& output = pending.runtime->output();
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++stats_.scans_processed;
    stats_.processed = output.processed();
    stats_.invalid = output.invalid();
    stats_.stamp_errors = output.stamp_errors();
    stats_.input_reason = output.input_reason();
    stats_.map_reason = output.map_reason();
    stats_.pose_reason = out.pose_ready ? "ready" : out.reason;
    if (out.fusion_ran) {
      stats_.camera_active = out.fusion.camera_active;
      stats_.camera_reason = out.fusion.camera_reason;
      stats_.pending_scans = out.fusion.pending_scans;
    }
    if (const auto* camera_fusion = pending.runtime->fusion()) stats_.fusion = camera_fusion->counters();
    stats_.audit_reason = out.hd_map_audit.reason;
    stats_.delivery_filter_applied=out.hd_map_audit.delivery_applied;
    stats_.delivery_input=out.hd_map_audit.delivery_input_count;
    stats_.delivery_excluded=out.hd_map_audit.delivery_excluded_count;
    stats_.delivery_output=out.hd_map_audit.delivery_output_count;
    if (out.hd_map_audit.enabled) {
      Stats::AuditSample sample;
      sample.ms = out.hd_map_audit.calculation_ms;
      for (const auto& d : out.hd_map_audit.objects) {
        ++sample.decisions[d.decision]; ++sample.reasons[d.reason];
        if(d.preview_enabled) ++sample.reasons["preview:"+d.preview_reason];
      }
      stats_.audit_window.push_back(std::move(sample));
      while (stats_.audit_window.size() > kTimingWindow) stats_.audit_window.pop_front();
    }
    stats_.delivery_ms.push_back(delivery_ms);
    while (stats_.delivery_ms.size() > kTimingWindow) stats_.delivery_ms.pop_front();
    stats_.total_ms.push_back(out.total_ms);
    while (stats_.total_ms.size() > kTimingWindow) stats_.total_ms.pop_front();
    stats_.last_stamp_ns = pending.scan.time.stamp_ns;
    stats_.last_receipt_ns = pending.receipt_steady_ns;
    stats_.last_processing_ns = pending.scan.time.processing_ns;
    stats_.last_output_steady_ns = camera::steady_now_ns();
    if (out.result.processed) {
      const auto& map = out.result.context.map_pose;
      stats_.scan_sequence = out.result.context.scan_sequence;
      if (!map.sample_stamps.empty() && !map.sample_arrivals.empty()) {
        stats_.gps_sample_stamp_ns = map.sample_stamps.back();
        stats_.gps_sample_arrival_ns = map.sample_arrivals.back();
        stats_.gps_age_ms = double(out.result.context.time.stamp_ns - map.sample_stamps.back()) / 1e6;
      }
    }
  }

  void PublishTrace(const PendingScan& pending, const ScanOutput& out, double delivery_ms) {
    trace::LineInput line;
    line.producer = kProducer;
    line.session_id = epoch_.session();
    line.generation = pending.scan.generation;
    line.config_id = config_.config_id;
    line.calibration_id = config_.calibration_id;
    line.seq = pending.seq;
    line.stamp_ns = pending.scan.time.stamp_ns;
    line.arrival_ns = pending.receipt_ros_ns;
    line.accepted = true;
    line.output = &out;
    line.input_to_publish_ms = delivery_ms;
    std_msgs::String text;
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      line.imu = cache_.Imu(out.selected.imu_stamp_ns);
      line.can = cache_.Can(out.selected.can_stamp_ns);
      text.data = trace::Line(line);
    }
    trace_pub_.publish(text);
  }

  // Ported from Main src/Geometry/FrameBoundaryPublisher.cpp:129-159 (Diagnostics, shadow keys) (8f93c22e).
  void OnDiagnostics(const ros::WallTimerEvent&) {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "lidar_pipeline";
    status.hardware_id = "asmc_lidar";
    auto value = [&status](const std::string& key, const std::string& text) {
      diagnostic_msgs::KeyValue kv;
      kv.key = key;
      kv.value = text;
      status.values.push_back(kv);
    };
    auto number = [&value](const std::string& key, auto n) { value(key, std::to_string(n)); };
    std::lock_guard<std::mutex> epoch(epoch_mutex_);
    const auto& config = config_;
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      const auto now = camera::steady_now_ns();
      const bool silent = !stats_.last_output_steady_ns || now - stats_.last_output_steady_ns > 1000000000LL;
      const bool partial = runtime_->delivery_status().find("partial:") == 0;
      status.level = silent || partial || stats_.input_reason != "valid" ? diagnostic_msgs::DiagnosticStatus::WARN
                                                               : diagnostic_msgs::DiagnosticStatus::OK;
      status.message = silent ? "input_silence" : partial ? "delivery_policy_partial" : stats_.input_reason;
      number("processed", stats_.processed);
      number("invalid", stats_.invalid);
      number("stamp_errors", stats_.stamp_errors);
      number("scans_received", points_received_.load());
      number("scans_processed", stats_.scans_processed);
      number("overwritten_scans", overwritten_scans_.load());
      number("rejected_scans", runtime_->rejected_scans());
      number("conversion_errors", conversion_errors_.load());
      value("last_reject_reason", stats_.last_reject_reason);
      value("input_reason", stats_.input_reason);
      value("map_reason", stats_.map_reason);
      value("pose_reason", stats_.pose_reason);
      number("source_stamp_ns", stats_.last_stamp_ns);
      value("receipt_clock", "steady");
      number("receipt_ns", stats_.last_receipt_ns);
      number("processing_ns", stats_.last_processing_ns);
      number("scan_sequence", stats_.scan_sequence);
      if (stats_.gps_sample_stamp_ns) {
        number("gps_sample_stamp_ns", stats_.gps_sample_stamp_ns);
        number("gps_sample_arrival_ns", stats_.gps_sample_arrival_ns);
        number("gps_age_ms", stats_.gps_age_ms);
      }
      number("imu_rejected", runtime_->imu_rejected());
      number("can_rejected", runtime_->can_rejected());
      number("gps_rejected", runtime_->gps_rejected());
      value("last_gps_reason", runtime_->last_gps_reason());
      value("delivery_policy", runtime_->delivery_status());
      value("delivery_assets_id", config.delivery_assets_id);
      value("camera_fusion", camera_ ? "enabled" : config.enable_camera_fusion ? "lidar_only" : "disabled");
      value("camera_active", stats_.camera_active ? "true" : "false");
      value("camera_reason", camera_ ? stats_.camera_reason : camera_reason_);
      const auto& f = stats_.fusion;
      number("fusion_calls", f.calls);
      number("fusion_queued", f.queued);
      number("fusion_matched", f.matched);
      number("fusion_dropped", f.dropped);
      number("fusion_waits", f.waits);
      number("fusion_reused", f.reused);
      number("fusion_inactive_calls", f.inactive_calls);
      number("fusion_epoch_changes", f.epoch_changes);
      number("fusion_startup_discarded", f.startup_discarded);
      number("fusion_bound_discarded", f.bound_discarded);
      number("fusion_observations", f.observations);
      number("fusion_confirmations_applied", f.confirmations_applied);
      number("fusion_misses_applied", f.misses_applied);
      number("fusion_pending_scans", stats_.pending_scans);
      const std::vector<double> window(stats_.total_ms.begin(), stats_.total_ms.end());
      value("hd_map_audit_mode", config.hd_map_audit_mode);
      value("delivery_filter_applied",stats_.delivery_filter_applied?"true":"false");
      number("delivery_input_count",stats_.delivery_input);
      number("delivery_excluded_count",stats_.delivery_excluded);
      number("delivery_output_count",stats_.delivery_output);
      value("hd_map_audit_reason", stats_.audit_reason);
      value("hd_map_audit_setup_error", config.hd_map_audit_error);
      value("hd_map_audit_bundle_sha256", config.hd_map_manifest_sha256);
      if (config.hd_map_audit) {
        value("hd_map_audit_map_sha256", config.hd_map_audit->map_sha256());
        value("hd_map_audit_policy_sha256", config.hd_map_audit->policy_sha256());
      }
      std::vector<double> audit_times;
      std::map<std::string, std::size_t> decisions, reasons;
      for (const auto& sample : stats_.audit_window) {
        audit_times.push_back(sample.ms);
        for (const auto& d : sample.decisions) decisions[d.first] += d.second;
        for (const auto& r : sample.reasons) reasons[r.first] += r.second;
      }
      number("hd_map_audit_window_scans", audit_times.size());
      number("hd_map_audit_ms_p95", Percentile(audit_times, .95));
      for (const auto& d : decisions) number("hd_map_audit_decision/"+d.first, d.second);
      for (const auto& r : reasons) number("hd_map_audit_reason/"+r.first, r.second);
      number("input_to_publish_ms_p95", Percentile(std::vector<double>(stats_.delivery_ms.begin(), stats_.delivery_ms.end()), .95));
      number("processing_window_scans", window.size());
      number("processing_ms_p50", Percentile(window, .50));
      number("processing_ms_p95", Percentile(window, .95));
      number("processing_ms_max", window.empty() ? 0. : *std::max_element(window.begin(), window.end()));
    }
    value("producer_session_id", epoch_.session());
    number("generation", epoch_.generation());
    number("retired_results", retired_results_.load());
    value("config_id", config.config_id);
    value("observation_calibration_id", config.calibration_id);
    value("config_path", config.config_path);
    value("config_sha256", config.config_sha256);
    value("mount_config_path", config.mount_config_path);
    value("mount_config_sha256", config.mount_config_sha256);
    value("calibration_id", config.pipeline.calibration_id);
    value("roll_leveling", config.pipeline.use_roll ? "true" : "false");
    value("tunnel_filter", config.pipeline.tunnel.enabled ? "true" : "false");
    value("publish_tf", publish_tf_ ? tf_child_frame_ : "false");
    value("velocity_status", kVelocityStatus);
    value("debug_enabled", debug_ ? "true" : "false");
    if (debug_) {
      const DebugCounters d = debug_->counters();
      value("debug_topic_prefix", debug_config_.topic_prefix);
      number("debug_rate_hz", debug_config_.rate_hz);
      number("debug_max_points", debug_config_.max_points);
      number("debug_admitted", d.admitted);
      number("debug_published", d.published);
      number("debug_skipped_no_subscriber", d.skipped_no_subscriber);
      number("debug_skipped_throttle", d.skipped_throttle);
      number("debug_overwritten", d.overwritten);
      number("debug_capture_ms_last", d.capture_ms_last);
      number("debug_capture_ms_max", d.capture_ms_max);
      number("debug_encode_ms_last", d.encode_ms_last);
      number("debug_encode_ms_max", d.encode_ms_max);
    }
    number("local_subscribers", local_pub_.getNumSubscribers());
    number("map_subscribers", map_pub_.getNumSubscribers());
    array.status.push_back(status);
    diagnostics_pub_.publish(array);
  }

  ros::NodeHandle nh_, pnh_;
  std::string points_topic_, imu_topic_, can_topic_, gps_topic_;
  std::string camera_observations_topic_, camera_status_topic_;
  std::string local_topic_, map_topic_, diagnostics_topic_, trace_topic_;
  bool publish_tf_ = false, allow_operational_ = false;
  std::string tf_child_frame_;
  int spinner_threads_ = 4;

  RuntimeConfig config_;
  camera::Calibrations calibrations_;
  const camera::Calibration* front_ = nullptr;  // into calibrations_
  std::unique_ptr<camera::Receiver> receiver_;
  std::shared_ptr<fusion::CameraSource> camera_;
  std::string camera_reason_;
  DebugPublisherConfig debug_config_;
  std::unique_ptr<DebugPublisher> debug_;  // outlives per-epoch runtimes and the worker
  std::shared_ptr<LidarRuntime> runtime_;
  std::mutex epoch_mutex_;
  ProcessingEpoch epoch_;
  std::atomic<std::uint64_t> retired_results_{0};
  ros::ServiceServer reset_service_;

  ros::Publisher local_pub_, map_pub_, diagnostics_pub_, trace_pub_;
  ros::Publisher local_observation_pub_, map_observation_pub_;
  ros::Publisher local_tracking_pub_, map_tracking_pub_;
  std::string local_observation_topic_, map_observation_topic_;
  std::string local_tracking_topic_, map_tracking_topic_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  ros::Subscriber points_sub_, imu_sub_, can_sub_, gps_sub_;
  ros::WallTimer diagnostics_timer_;

  std::mutex slot_mutex_;
  std::condition_variable slot_cv_;
  std::optional<PendingScan> slot_;
  bool stop_ = false;
  std::thread worker_;

  std::atomic<std::uint64_t> points_received_{0}, overwritten_scans_{0}, conversion_errors_{0};
  // points_received_ is the scan sequence; epoch_ is the processing reset generation.

  std::mutex cache_mutex_;
  trace::LocalInputCache cache_;
  std::mutex stats_mutex_;
  Stats stats_;
};

}  // namespace
}  // namespace runtime
}  // namespace lidar
}  // namespace asmc

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_pipeline");
  ros::NodeHandle nh, pnh("~");
  try {
    asmc::lidar::runtime::LidarPipelineNode node(nh, pnh);
    return node.Run();
  } catch (const asmc::lidar::runtime::StartupRefused& e) {
    ROS_FATAL_STREAM("lidar_pipeline: " << e.what());
  } catch (const std::exception& e) {
    ROS_FATAL_STREAM("lidar_pipeline: start-up failed: " << e.what());
  }
  return 1;
}
