// lidar_replay: offline bag replay of the asmc_lidar shadow pipeline.
//
// Feeds the selected bag topics in record order into the same LidarRuntime
// calls as lidar_pipeline_node (OnImu/OnCan/OnGps, AcceptScan, then
// ProcessScan with a zero local wait) and writes one asmc_lidar_scan_trace_v1
// JSON line per message on the points topic. No ROS master is used.
//
// Clocks (Main test/lidar_continuous_replay.cpp): the logical arrival clock
// arrival - first + 1 s is the receipt/processing/decision/steady clock; ROS
// "now" is the record time. GPS fault injection uses the bag-relative arrival
// time: elapsed in [begin, end) -> "missing" drops the message, "invalid"
// forces status 0.
//
// usage: lidar_replay --bag B --output OUT.jsonl --config C
//   [--package-share DIR] [--camera-calibration Y] [--profile pitch|roll_pitch]
//   [--gps-mode normal|invalid|missing --gps-begin S --gps-end S]
//   [--points-topic T --imu-topic T --can-topic T --gps-topic T]
//   [--camera-observations-topic T --camera-status-topic T] (camera off by default)
//   [--tunnel-filter on|off] [--capture-debug]

#include <asmc_lidar/geometry/camera_calibration.hpp>
#include <asmc_lidar/fusion/camera_observations.hpp>
#include <asmc_lidar/fusion/front_camera_fusion.hpp>
#include <asmc_lidar/ros/scan_trace.hpp>
#include <asmc_lidar/ros/ros_inputs.hpp>
#include <asmc_lidar/ros/runtime.hpp>
#include <asmc_lidar/core/processing_epoch.hpp>
#include <asmc_lidar/pipeline/stage_observer.hpp>
#include <sstream>
#include <iomanip>

#include <asmc_msgs/CameraObjectObservation.h>
#include <asmc_msgs/InferenceStatus.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/GPSMessage.h>
#include <ros/time.h>
#include <rosbag/bag.h>
#include <asmc_lidar/ros/planning_output.hpp>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace asmc {
namespace lidar {
namespace runtime {
namespace {

namespace fusion = pipeline::fusion;

constexpr char kUsage[] =
    "usage: lidar_replay --bag B --output OUT.jsonl --config C [--package-share DIR]\n"
    "  [--camera-calibration Y] [--profile pitch|roll_pitch]\n"
    "  [--gps-mode normal|invalid|missing --gps-begin S --gps-end S]\n"
    "  [--points-topic /velodyne_points --imu-topic /imu --can-topic /Ego_topic --gps-topic /gps]\n"
    "  [--camera-observations-topic T --camera-status-topic S] (default: camera disabled)\n"
    "  [--fusion-diagnostics on|off] [--tunnel-filter on|off] [--capture-debug] [--planning-output NEW.bag]\n"
    "  [--dump-stage-dir NEW_DIR --dump-stage-stamps NS,NS] (offline points)\n";
struct UsageError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

struct Options {
  std::string bag, output, config, package_share, camera_calibration;
  std::string planning_output;
  std::string dump_stage_dir, dump_stage_stamps;
  std::string profile;  // empty: keep the configured leveling
  std::string gps_mode = "normal";
  double gps_begin = 0., gps_end = 0.;
  std::string points_topic = "/velodyne_points", imu_topic = "/imu", can_topic = "/Ego_topic";
  std::string gps_topic = "/gps", camera_observations_topic, camera_status_topic;
  std::string fusion_diagnostics;
  std::string tunnel_filter;  // empty: keep the configured value
  bool capture_debug = false;
};

Options Parse(int argc, char** argv) {
  Options o;
  const std::map<std::string, std::string*> text = {
      {"--bag", &o.bag},
      {"--output", &o.output},
      {"--planning-output", &o.planning_output},
      {"--dump-stage-dir", &o.dump_stage_dir},
      {"--dump-stage-stamps", &o.dump_stage_stamps},
      {"--config", &o.config},
      {"--package-share", &o.package_share},
      {"--camera-calibration", &o.camera_calibration},
      {"--profile", &o.profile},
      {"--gps-mode", &o.gps_mode},
      {"--points-topic", &o.points_topic},
      {"--imu-topic", &o.imu_topic},
      {"--can-topic", &o.can_topic},
      {"--gps-topic", &o.gps_topic},
      {"--camera-observations-topic", &o.camera_observations_topic},
      {"--camera-status-topic", &o.camera_status_topic},
      {"--fusion-diagnostics", &o.fusion_diagnostics},
      {"--tunnel-filter", &o.tunnel_filter}};
  const std::map<std::string, double*> numbers = {{"--gps-begin", &o.gps_begin}, {"--gps-end", &o.gps_end}};
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--capture-debug") {
      o.capture_debug = true;
      continue;
    }
    if (key == "--help" || key == "-h") throw UsageError("");
    if (i + 1 >= argc) throw UsageError("missing value for " + key);
    const std::string value = argv[++i];
    if (const auto t = text.find(key); t != text.end()) {
      *t->second = value;
    } else if (const auto n = numbers.find(key); n != numbers.end()) {
      std::size_t used = 0;
      try {
        *n->second = std::stod(value, &used);
      } catch (const std::exception&) {
        used = 0;
      }
      if (used != value.size()) throw UsageError("bad number for " + key + ": " + value);
    } else {
      throw UsageError("unknown option " + key);
    }
  }
  if (o.bag.empty() || o.output.empty() || o.config.empty()) throw UsageError("--bag, --output and --config are required");
  if (!o.profile.empty() && o.profile != "pitch" && o.profile != "roll_pitch")
    throw UsageError("--profile must be pitch or roll_pitch");
  if (o.gps_mode != "normal" && o.gps_mode != "invalid" && o.gps_mode != "missing")
    throw UsageError("--gps-mode must be normal, invalid or missing");
  if (!o.fusion_diagnostics.empty() && o.fusion_diagnostics != "on" && o.fusion_diagnostics != "off")
    throw UsageError("--fusion-diagnostics must be on or off");
  if (!o.tunnel_filter.empty() && o.tunnel_filter != "on" && o.tunnel_filter != "off")
    throw UsageError("--tunnel-filter must be on or off");
  if (o.camera_observations_topic.empty() != o.camera_status_topic.empty())
    throw UsageError("--camera-observations-topic and --camera-status-topic go together");
  if (!o.camera_observations_topic.empty() && o.camera_calibration.empty())
    throw UsageError("camera topics need --camera-calibration");
  std::vector<std::string> topics = {o.points_topic, o.imu_topic, o.can_topic, o.gps_topic};
  if (!o.camera_observations_topic.empty()) {
    topics.push_back(o.camera_observations_topic);
    topics.push_back(o.camera_status_topic);
  }
  if (std::set<std::string>(topics.begin(), topics.end()).size() != topics.size())
    throw UsageError("every input topic must be distinct");
  return o;
}

// Same field mapping as camera::from_ros (camera_receiver.cpp, asmc_lidar_camera_ros,
// which this tool does not link: it needs no ROS transport).
camera::Observation ObservationFrom(const asmc_msgs::CameraObjectObservation& m) {
  camera::Observation o;
  const auto& d = m.detections;
  o.header = {d.header.seq, static_cast<Ns>(d.header.stamp.toNSec()), d.header.frame_id};
  o.width = d.image_width;
  o.height = d.image_height;
  o.valid = d.valid;
  o.session = m.producer_session_id;
  o.generation = m.generation;
  o.calibration_id = m.calibration_id;
  o.model_id = m.model_id;
  o.config_id = m.config_id;
  o.boxes.reserve(d.boxes.size());
  for (const auto& b : d.boxes)
    o.boxes.push_back({b.xmin, b.ymin, b.xmax, b.ymax, b.class_name, b.source_class_name, b.confidence,
                       bool(b.confidence_valid)});
  return o;
}
camera::Status StatusFrom(const asmc_msgs::InferenceStatus& m) {
  return {static_cast<Ns>(m.header.stamp.toNSec()), static_cast<Ns>(m.source_cutoff.toNSec()),
          m.producer_session_id, m.model, m.generation, m.mode};
}

template <class Message>
boost::shared_ptr<Message> Instantiate(const rosbag::MessageInstance& m) {
  auto message = m.instantiate<Message>();
  if (!message)
    throw std::runtime_error("topic " + m.getTopic() + " carries " + m.getDataType() + ", expected " +
                             ros::message_traits::DataType<Message>::value());
  return message;
}

// Offline evidence only: selected original stage points, without changing inference.
class StageDump final : public pipeline::StageObserver {
 public:
  explicit StageDump(const Options& o):directory_(o.dump_stage_dir) {
    if(directory_.empty()!=o.dump_stage_stamps.empty())
      throw UsageError("--dump-stage-dir and --dump-stage-stamps go together");
    if(directory_.empty())return;
    std::istringstream input(o.dump_stage_stamps);std::string token;
    while(std::getline(input,token,',')) {
      std::size_t used=0;const auto stamp=std::stoll(token,&used);
      if(used!=token.size() || stamp<=0)throw UsageError("invalid stage stamp");
      stamps_.insert(stamp);
    }
    std::filesystem::create_directories(directory_);
  }
  bool Begin(std::int64_t stamp) override {return stamps_.count(stamp);}
  void OnScan(const pipeline::StageView& v) override {
    const auto path=std::filesystem::path(directory_)/(std::to_string(v.result.stamp_ns)+".csv");
    if(std::filesystem::exists(path))throw std::runtime_error("stage dump already exists: "+path.string());
    std::ofstream f(path);if(!f)throw std::runtime_error("cannot create stage dump");
    f<<"x,y,z,kept,cluster\n"<<std::setprecision(9);
    std::vector<int> kept(v.leveled.size(),0),cluster(v.leveled.size(),-1);
    for(int i:v.cluster_input_indices)kept.at(i)=1;
    for(std::size_t c=0;c<v.clusters.size();++c)for(int i:v.clusters[c].indices)
      cluster.at(v.cluster_input_indices.at(i))=static_cast<int>(c);
    for(std::size_t i=0;i<v.leveled.size();++i) {
      const auto& q=v.leveled[i];f<<q.x<<','<<q.y<<','<<q.z<<','<<kept[i]<<','<<cluster[i]<<'\n';
    }
    if(!f)throw std::runtime_error("stage dump write failed");
  }
 private:
  std::string directory_;std::set<std::int64_t> stamps_;
};

// Ported from Main test/lidar_continuous_replay.cpp:52-149 (record-order loop, logical clock,
// GPS injection) (8f93c22e); every message goes through the LidarRuntime node path.
int Replay(const Options& opt) {
  const std::string package_share = !opt.package_share.empty()
      ? opt.package_share
      : std::filesystem::path(opt.config).parent_path().parent_path().string();
  RuntimeConfig config = LoadRuntimeConfig(opt.config, package_share);
  config.fusion.capture_diagnostics = opt.fusion_diagnostics != "off";
  if (!opt.profile.empty()) config.pipeline.use_roll = opt.profile == "roll_pitch";
  if (!opt.tunnel_filter.empty()) config.pipeline.tunnel.enabled = opt.tunnel_filter == "on";
  config.pipeline.capture_debug = opt.capture_debug;
  StageDump stage_dump(opt);
  if(!opt.dump_stage_dir.empty())config.pipeline.observer=&stage_dump;

  const bool camera_enabled = !opt.camera_observations_topic.empty();
  camera::Calibrations calibrations;
  std::shared_ptr<camera::Buffer> buffer;
  std::shared_ptr<fusion::CameraSource> source;
  const camera::Calibration* front = nullptr;
  // Without camera topics fusion.enable_sensor_fusion keeps its configured value
  // and the camera source is null: Main's baseline replays run FusionProcess with
  // no camera traffic (FusionLiDAR's AABB/Core rewrite still applies).
  if (camera_enabled) {
    calibrations = camera::load_calibrations(opt.camera_calibration);
    const auto it = calibrations.find("camera_front");
    if (it == calibrations.end()) throw std::runtime_error("camera calibration has no camera_front");
    front = &it->second;
    buffer = std::make_shared<camera::Buffer>(calibrations, camera::Limits{});  // Receiver defaults
    source = std::make_shared<fusion::BufferCameraSource>(buffer, "camera_front");
  }
  FinalizeRuntimeIdentity(config, front ? front->id : "");
  LidarRuntime runtime(config, source, front);
  const ProcessingEpoch epoch;

  rosbag::Bag planning_bag;
  if (!opt.planning_output.empty()) {
    if (std::ifstream(opt.planning_output).good()) throw std::runtime_error("planning output already exists");
    planning_bag.open(opt.planning_output, rosbag::bagmode::Write);
  }

  std::ofstream output(opt.output);
  if (!output) throw std::runtime_error("cannot open " + opt.output);

  rosbag::Bag bag(opt.bag, rosbag::bagmode::Read);
  // Main uses the begin time of the whole bag (index only; no message is read).
  const std::uint64_t first = rosbag::View(bag).getBeginTime().toNSec();
  std::vector<std::string> topics = {opt.points_topic, opt.imu_topic, opt.can_topic, opt.gps_topic};
  if (camera_enabled) {
    topics.push_back(opt.camera_observations_topic);
    topics.push_back(opt.camera_status_topic);
  }
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  trace::LocalInputCache cache;
  std::uint64_t seq = 0, accepted = 0, processed = 0, ready = 0;
  std::uint64_t gps_dropped = 0, gps_forced_invalid = 0, imu = 0, can = 0, gps = 0, camera_msgs = 0;
  std::map<std::string, std::uint64_t> camera_results;
  for (const rosbag::MessageInstance& m : view) {
    const std::uint64_t arrival = m.getTime().toNSec();
    const auto logical = static_cast<std::int64_t>(arrival - first + 1000000000ULL);
    const auto ros_now = static_cast<std::int64_t>(arrival);
    ros::Time::setNow(m.getTime());  // like Main's replay, for any ros::Time::now() use
    const std::string& topic = m.getTopic();
    if (topic == opt.imu_topic) {
      const auto input = ImuInputFromMsg(*Instantiate<sensor_msgs::Imu>(m));
      const auto before = runtime.imu_rejected();
      runtime.OnImu(input, logical, ros_now);
      if (runtime.imu_rejected() == before) cache.AddImu(input.stamp_ns, pipeline::ImuFromInput(input));
      ++imu;
    } else if (topic == opt.can_topic) {
      const auto input = CanInputFromMsg(*Instantiate<morai_msgs::EgoVehicleStatus>(m));
      const auto before = runtime.can_rejected();
      runtime.OnCan(input, logical, ros_now);
      if (runtime.can_rejected() == before) cache.AddCan(input.stamp_ns, pipeline::CanFromInput(input));
      ++can;
    } else if (topic == opt.gps_topic) {
      auto input = GpsInputFromMsg(*Instantiate<morai_msgs::GPSMessage>(m));
      const double elapsed = (arrival - first) * 1e-9;
      const bool inject = elapsed >= opt.gps_begin && elapsed < opt.gps_end;
      if (inject && opt.gps_mode == "missing") {
        ++gps_dropped;
        continue;
      }
      if (inject && opt.gps_mode == "invalid") {
        input.status = 0;
        ++gps_forced_invalid;
      }
      runtime.OnGps(input, logical, ros_now);
      ++gps;
    } else if (camera_enabled && topic == opt.camera_status_topic) {
      ++camera_results[buffer->status(StatusFrom(*Instantiate<asmc_msgs::InferenceStatus>(m)), ros_now, logical)];
      ++camera_msgs;
    } else if (camera_enabled && topic == opt.camera_observations_topic) {
      ++camera_results[buffer->observe(ObservationFrom(*Instantiate<asmc_msgs::CameraObjectObservation>(m)),
                                       ros_now, logical)];
      ++camera_msgs;
    } else if (topic == opt.points_topic) {
      const auto msg = Instantiate<sensor_msgs::PointCloud2>(m);
      const std::int64_t stamp = StampNs(msg->header.stamp);
      trace::LineInput line;
      line.producer = "lidar_replay";
      line.seq = ++seq;
      line.session_id = epoch.session();
      line.generation = epoch.generation();
      line.config_id = config.config_id;
      line.calibration_id = config.calibration_id;
      line.stamp_ns = stamp;
      line.arrival_ns = static_cast<std::int64_t>(arrival);
      line.debug = opt.capture_debug;
      pipeline::ScanInput scan;
      scan.points = CloudFromMsg(*msg);
      line.accepted = runtime.AcceptScan(stamp, scan.points->size(), &line.accept_reason);
      ScanOutput out;
      if (line.accepted) {
        ++accepted;
        // Same scan time as the node: ROS source stamp, receipt on the replay clock.
        scan.time = asmc::lidar::Observation{stamp, logical, logical, "ros", config.stamp_provenance, "steady"};
        scan.scan_sequence = seq;
        scan.generation = epoch.generation();
        ScanTimes times;
        times.receipt_ns = times.processing_ns = times.decision_ns = times.steady_now_ns = logical;
        times.ros_now_ns = ros_now;
        out = runtime.ProcessScan(scan, 0., times);
        ready += out.pose_ready;
        processed += out.result.processed;
        line.output = &out;
        line.imu = cache.Imu(out.selected.imu_stamp_ns);
        line.can = cache.Can(out.selected.can_stamp_ns);
        if (!opt.planning_output.empty() && out.arrays.accepted) {
          asmc_msgs::LidarObservation observation;
          observation.objects = out.arrays.map;
          observation.producer_session_id = epoch.session();
          observation.generation = epoch.generation();
          observation.scan_sequence = seq;
          observation.config_id = config.config_id;
          observation.calibration_id = config.calibration_id;
          planning_bag.write("/migration/lidar/planning_observations_map", m.getTime(), PlanningOutput(observation, out.result));
        }
      }
      output << trace::Line(line) << '\n';
    }
  }
  output.flush();
  std::cerr << "[lidar_replay] scans=" << seq << " accepted=" << accepted << " pose_ready=" << ready
            << " processed=" << processed << " typed_processed=" << runtime.output().processed()
            << " typed_invalid=" << runtime.output().invalid() << " stamp_errors=" << runtime.output().stamp_errors()
            << " imu=" << imu << " can=" << can << " gps=" << gps << " gps_dropped=" << gps_dropped
            << " gps_forced_invalid=" << gps_forced_invalid << " imu_rejected=" << runtime.imu_rejected()
            << " can_rejected=" << runtime.can_rejected() << " gps_rejected=" << runtime.gps_rejected()
            << " camera_messages=" << camera_msgs << " roll_leveling=" << (config.pipeline.use_roll ? 1 : 0)
            << " tunnel_filter=" << (config.pipeline.tunnel.enabled ? 1 : 0)
            << " config_sha256=" << config.config_sha256 << '\n';
  for (const auto& entry : camera_results) std::cerr << "  camera " << entry.first << '=' << entry.second << '\n';
  return output ? 0 : 1;
}

}  // namespace
}  // namespace runtime
}  // namespace lidar
}  // namespace asmc

int main(int argc, char** argv) {
  using asmc::lidar::runtime::UsageError;
  asmc::lidar::runtime::Options options;
  try {
    options = asmc::lidar::runtime::Parse(argc, argv);
  } catch (const UsageError& e) {
    if (*e.what()) std::cerr << "lidar_replay: " << e.what() << '\n';
    std::cerr << asmc::lidar::runtime::kUsage;
    return 2;
  }
  ros::Time::init();
  std::srand(0);  // Main's replay seeds rand() the same way
  try {
    return asmc::lidar::runtime::Replay(options);
  } catch (const std::exception& e) {
    std::cerr << "lidar_replay: " << e.what() << '\n';
    return 1;
  }
}
