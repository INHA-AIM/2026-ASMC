// Monitoring-only stage publisher (see shadow/debug_publisher.hpp).
#include <asmc_lidar/ros/debug_publisher.hpp>

#include <asmc_lidar/pipeline/pipeline.hpp>
#include <asmc_lidar/ros/scan_trace.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/String.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <utility>
#include <unordered_map>

namespace asmc {
namespace lidar {
namespace runtime {
namespace {

using Clock = std::chrono::steady_clock;

std::int64_t SteadyNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
}

double Ms(Clock::time_point from, Clock::time_point to) {
  return std::chrono::duration<double, std::milli>(to - from).count();
}

std::string Join(const std::string& prefix, const char* leaf) {
  if (!prefix.empty() && prefix.back() == '/') return prefix + leaf;
  return prefix + "/" + leaf;
}

// Deterministic stride sample: points 0, stride, 2*stride, ... (at most max_points).
void Sample(const pipeline::Cloud& cloud, std::size_t max_points, DebugPublisher::StageSample& out,
            std::vector<std::uint32_t>* picked) {
  const std::size_t n = cloud.points.size();
  const std::size_t cap = std::max<std::size_t>(1, max_points);
  const std::size_t stride = n <= cap ? 1 : (n + cap - 1) / cap;
  out.actual = static_cast<std::uint32_t>(n);
  out.stride = static_cast<std::uint32_t>(stride);
  out.sent = static_cast<std::uint32_t>(n ? (n + stride - 1) / stride : 0);
  out.xyzi.clear();
  out.xyzi.reserve(4 * out.sent);
  if (picked) {
    picked->clear();
    picked->reserve(out.sent);
  }
  for (std::size_t i = 0; i < n; i += stride) {
    const auto& p = cloud.points[i];
    out.xyzi.push_back(p.x);
    out.xyzi.push_back(p.y);
    out.xyzi.push_back(p.z);
    out.xyzi.push_back(p.intensity);
    if (picked) picked->push_back(static_cast<std::uint32_t>(i));
  }
}

DebugPublisher::ObjectSummary Summary(const pipeline::TrackedObject& o) {
  DebugPublisher::ObjectSummary s;
  s.idx = o.detection.u16_Idx;
  s.id = o.tracking.s32_Id;
  s.age = o.tracking.s32_Age;
  s.miss = o.tracking.s32_MissCount;
  s.points = static_cast<int>(o.detection.pcl_ClusterPoints.size());
  s.cls = o.camera.str_ObjectClass;
  s.raw_cls = o.camera.str_YoloClass;
  s.dist = o.detection.f32_Dist;
  s.cx = o.detection.f32_OBB_CenterX;
  s.cy = o.detection.f32_OBB_CenterY;
  s.cz = o.detection.f32_OBB_CenterZ;
  s.width = o.detection.obb_length_m;
  s.height = o.detection.obb_width_m;
  s.zmin = o.detection.f32_OBB_ZMin;
  s.zmax = o.detection.f32_OBB_ZMax;
  s.yaw = o.detection.f32_OBB_YawRad;
  s.corners = o.detection.obb_corners_level;
  s.kf_initialized = o.tracking.b_KfInitialized;
  s.abs_vx = o.tracking.f32_AbsVx;
  s.abs_vy = o.tracking.f32_AbsVy;
  s.velocity = o.tracking.f32_Velocity;
  s.motion = o.tracking.motion;
  return s;
}

sensor_msgs::PointField Field(const char* name, std::uint32_t offset, std::uint8_t datatype) {
  sensor_msgs::PointField f;
  f.name = name;
  f.offset = offset;
  f.datatype = datatype;
  f.count = 1;
  return f;
}

sensor_msgs::PointCloud2 CloudMsg(const DebugPublisher::StageSample& s, std::int64_t stamp_ns,
                                  const std::string& frame, const std::vector<std::uint8_t>* label,
                                  const std::vector<std::int32_t>* cluster,
                                  const std::vector<std::int32_t>* track=nullptr) {
  sensor_msgs::PointCloud2 msg;
  msg.header.stamp.fromNSec(static_cast<std::uint64_t>(stamp_ns));
  msg.header.frame_id = frame;
  msg.fields = {Field("x", 0, sensor_msgs::PointField::FLOAT32), Field("y", 4, sensor_msgs::PointField::FLOAT32),
                Field("z", 8, sensor_msgs::PointField::FLOAT32),
                Field("intensity", 12, sensor_msgs::PointField::FLOAT32)};
  const bool labelled = label != nullptr;
  if (labelled) {
    msg.fields.push_back(Field("label", 16, sensor_msgs::PointField::UINT8));
  }
  if(cluster) {
    msg.fields.push_back(Field("cluster", 20, sensor_msgs::PointField::INT32));
  }
  if(track)msg.fields.push_back(Field("track",24,sensor_msgs::PointField::INT32));
  msg.point_step = track ? 28 : cluster ? 24 : labelled ? 20 : 16;
  msg.height = 1;
  msg.width = s.sent;
  msg.row_step = msg.point_step * msg.width;
  msg.is_bigendian = false;
  msg.is_dense = true;
  msg.data.assign(static_cast<std::size_t>(msg.row_step), 0);
  for (std::uint32_t i = 0; i < s.sent; ++i) {
    std::uint8_t* out = msg.data.data() + static_cast<std::size_t>(i) * msg.point_step;
    std::memcpy(out, s.xyzi.data() + 4 * static_cast<std::size_t>(i), 16);
    if (labelled) {
      out[16] = i < label->size() ? (*label)[i] : 0;
    }
    if(cluster) {
      const std::int32_t c = i < cluster->size() ? (*cluster)[i] : -1;
      std::memcpy(out + 20, &c, 4);
    }
    if(track){const std::int32_t id=i<track->size()?(*track)[i]:-1;std::memcpy(out+24,&id,4);}
  }
  return msg;
}

void Object(std::ostream& out, const DebugPublisher::ObjectSummary& o, bool track) {
  out << "{\"idx\":" << o.idx << ",\"points\":" << o.points << ",\"dist\":";
  trace::Num(out, o.dist);
  out << ",\"obb\":{\"center\":";
  trace::Vec3(out, o.cx, o.cy, o.cz);
  out << ",\"width\":";
  trace::Num(out, o.width);
  out << ",\"height\":";
  trace::Num(out, o.height);
  out << ",\"zmin\":";
  trace::Num(out, o.zmin);
  out << ",\"zmax\":";
  trace::Num(out, o.zmax);
  out << ",\"yaw\":";
  trace::Num(out, o.yaw);
  out << "},\"corners\":[";
  for (std::size_t i = 0; i < o.corners.size(); ++i) {
    if (i) out << ',';
    trace::Vec3(out, o.corners[i].x, o.corners[i].y, o.corners[i].z);
  }
  out << ']';
  if (track) {
    out << ",\"id\":" << o.id << ",\"age\":" << o.age << ",\"miss\":" << o.miss << ",\"class\":";
    trace::Str(out, o.cls);
    out << ",\"raw_class\":";
    trace::Str(out, o.raw_cls);
    out << ",\"kf_initialized\":";
    trace::Bool(out, o.kf_initialized);
    // Internal CA state: frame not validated, never a published velocity.
    out << ",\"abs_v\":[";
    trace::Num(out, o.abs_vx);
    out << ',';
    trace::Num(out, o.abs_vy);
    out << "],\"velocity\":";
    trace::Num(out, o.velocity);
    out << ",\"motion\":{\"state\":" << o.motion.s32_MotionState << ",\"valid\":";
    trace::Bool(out,o.motion.motion_valid);
    out << ",\"source\":\"main_algorithm_display_only\",\"vx_kph\":";
    trace::Num(out,o.motion.f32_DecisionVxKPH);
    out << ",\"vy_kph\":";trace::Num(out,o.motion.f32_DecisionVyKPH);
    out << ",\"displacement_kph\":";trace::Num(out,o.motion.f32_CorrectedDisplacementSpeedKPH);
    out << ",\"samples\":" << o.motion.s32_MotionAcceptedSamples << '}';
  }
  out << '}';
}

void SampleJson(std::ostream& out, const DebugPublisher::StageSample& s) {
  out << "{\"actual\":" << s.actual << ",\"sent\":" << s.sent << ",\"stride\":" << s.stride << '}';
}

}  // namespace

std::vector<std::string> DebugPublisher::TopicNames(const std::string& prefix) {
  return {Join(prefix, "raw"), Join(prefix, "roi"), Join(prefix, "voxel"), Join(prefix, "leveled"),
          Join(prefix, "scan")};
}

DebugPublisher::DebugPublisher(ros::NodeHandle& nh, const DebugPublisherConfig& config, const RuntimeConfig& runtime)
    : config_(config), use_roll_(runtime.pipeline.use_roll), tunnel_enabled_(runtime.pipeline.tunnel.enabled),
      vehicle_(runtime.pipeline.vehicle),range_(runtime.pipeline.range),roi_(runtime.pipeline.roi),height_roi_(runtime.pipeline.height_roi) {
  const auto names = TopicNames(config_.topic_prefix);
  raw_pub_ = nh.advertise<sensor_msgs::PointCloud2>(names[0], 1);
  roi_pub_ = nh.advertise<sensor_msgs::PointCloud2>(names[1], 1);
  voxel_pub_ = nh.advertise<sensor_msgs::PointCloud2>(names[2], 1);
  leveled_pub_ = nh.advertise<sensor_msgs::PointCloud2>(names[3], 1);
  scan_pub_ = nh.advertise<std_msgs::String>(names[4], 2);
  thread_ = std::thread([this] { PublishLoop(); });
}

DebugPublisher::~DebugPublisher() {
  {
    std::lock_guard<std::mutex> lock(slot_mutex_);
    stop_ = true;
  }
  slot_cv_.notify_all();
  if (thread_.joinable()) thread_.join();
}

void DebugPublisher::SetScanContext(const std::string& sensor_frame, std::uint64_t seq, std::int64_t arrival_ros_ns,
                                    const std::string& session,std::uint64_t generation,const std::string& config_id) {
  sensor_frame_ = sensor_frame;
  producer_session_id_=session;generation_=generation;config_id_=config_id;
  seq_ = seq;
  arrival_ns_ = arrival_ros_ns;
}

bool DebugPublisher::Admit(std::int64_t) {
  std::lock_guard<std::mutex> lock(counters_mutex_);
  if (!config_.enabled) {
    ++counters_.skipped_disabled;
    return false;
  }
  const bool subscribed = scan_pub_.getNumSubscribers() > 0 || raw_pub_.getNumSubscribers() > 0 ||
                          roi_pub_.getNumSubscribers() > 0 || voxel_pub_.getNumSubscribers() > 0 ||
                          leveled_pub_.getNumSubscribers() > 0;
  if (!subscribed) {
    ++counters_.skipped_no_subscriber;
    return false;
  }
  const std::int64_t now = SteadyNs();
  if (config_.rate_hz > 0.) {
    const auto period = static_cast<std::int64_t>(1e9 / config_.rate_hz);
    if (last_admit_steady_ns_ && now - last_admit_steady_ns_ < period) {
      ++counters_.skipped_throttle;
      return false;
    }
  }
  last_admit_steady_ns_ = now;
  ++counters_.admitted;
  return true;
}

bool DebugPublisher::Begin(std::int64_t stamp_ns) {
  pending_.reset();
  if (!Admit(stamp_ns)) return false;
  pending_ = std::make_unique<Snapshot>();
  pending_->stamp_ns = stamp_ns;
  pending_->seq = seq_;
  pending_->arrival_ns = arrival_ns_;
  pending_->sensor_frame = sensor_frame_;
  return true;
}

void DebugPublisher::OnScan(const pipeline::StageView& view) {
  if (!pending_ || pending_->stamp_ns != view.result.stamp_ns) return;
  const auto t0 = Clock::now();
  Snapshot& s = *pending_;
  s.stages = true;
  if (view.scan.points) Sample(*view.scan.points, config_.max_points, s.raw, nullptr);
  const pipeline::RoiClassifier classify(range_,roi_,vehicle_);
  s.raw_label.reserve(s.raw.sent);
  for(std::size_t i=0;i<s.raw.xyzi.size();i+=4) {
    pcl::PointXYZI p;p.x=s.raw.xyzi[i];p.y=s.raw.xyzi[i+1];p.z=s.raw.xyzi[i+2];
    s.raw_label.push_back(static_cast<std::uint8_t>(classify(p)));
  }
  Sample(view.roi, config_.max_points, s.roi, nullptr);
  Sample(view.voxel, config_.max_points, s.voxel, nullptr);
  std::vector<std::uint32_t> picked;
  Sample(view.leveled, config_.max_points, s.leveled, &picked);

  // Per-point provenance over the full leveled cloud, then the sampled subset.
  const std::size_t n = view.leveled.points.size();
  std::vector<std::uint8_t> label(n, 1);
  if(height_roi_.enabled) {
    std::fill(label.begin(),label.end(),3);
    for(int i:view.height_kept_indices)if(i>=0&&std::size_t(i)<n)label[i]=1;
  }
  std::vector<std::int32_t> cluster(n, -1);
  const bool tunnel = view.result.tunnel.ran;
  for (const int i : view.ground_kept_indices)
    if (i >= 0 && static_cast<std::size_t>(i) < n) label[static_cast<std::size_t>(i)] = tunnel ? 2 : 0;
  if (tunnel)
    for (const int i : view.cluster_input_indices)
      if (i >= 0 && static_cast<std::size_t>(i) < n) label[static_cast<std::size_t>(i)] = 0;
  for (std::size_t c = 0; c < view.clusters.size(); ++c) {
    for (const int j : view.clusters[c].indices) {
      if (j < 0 || static_cast<std::size_t>(j) >= view.cluster_input_indices.size()) continue;
      const int i = view.cluster_input_indices[static_cast<std::size_t>(j)];
      if (i >= 0 && static_cast<std::size_t>(i) < n) cluster[static_cast<std::size_t>(i)] = static_cast<std::int32_t>(c);
    }
  }
  s.leveled_label.resize(picked.size());
  s.leveled_cluster.resize(picked.size());
  for (std::size_t k = 0; k < picked.size(); ++k) {
    s.leveled_label[k] = label[picked[k]];
    s.leveled_cluster[k] = cluster[picked[k]];
  }

  const pipeline::ScanResult& r = view.result;
  s.counts = r.counts;
  s.timings = r.timings;
  s.plane = r.plane;
  s.leveling = r.context.applied_leveling;
  s.base_from_sensor = r.context.base_from_sensor;
  s.base_from_leveled = asmc::lidar::base_from_input(r.context);
  s.map_valid = r.context.map_pose.valid;
  s.map_reason = r.context.map_pose.reason;
  s.map_from_base = r.context.map_pose.map_from_base;
  s.roll_rad = view.local.st_IMU.f32_Roll_rad;
  s.pitch_rad = view.local.st_IMU.f32_Pitch_rad;
  s.yaw_rad = view.local.st_IMU.f32_Yaw_rad;
  s.speed_mps = view.local.st_CAN.f32_CurrentVel_MPS;
  s.tunnel_ran = r.tunnel.ran;
  s.tunnel_removed = r.tunnel.removed_points;
  s.detections.clear();
  s.detections.reserve(view.detections.size());
  for (const auto& d : view.detections) s.detections.push_back(Summary(d));
  s.capture_ms = Ms(t0, Clock::now());
  std::lock_guard<std::mutex> lock(counters_mutex_);
  counters_.capture_ms_last = s.capture_ms;
  counters_.capture_ms_max = std::max(counters_.capture_ms_max, s.capture_ms);
}

void DebugPublisher::Finish(std::int64_t stamp_ns, const ScanOutput& out) {
  std::unique_ptr<Snapshot> s;
  if (pending_ && pending_->stamp_ns == stamp_ns && pending_->stages) {
    s = std::move(pending_);
  } else {
    pending_.reset();
    // Only a scan that never reached the stages (local pose not ready, empty or
    // duplicate scan) is announced without clouds. A processed scan that the
    // rate limit skipped in Begin() stays skipped: no second admission here.
    if (out.processed || !Admit(stamp_ns)) return;
    s = std::make_unique<Snapshot>();
    s->stamp_ns = stamp_ns;
    s->seq = seq_;
    s->arrival_ns = arrival_ns_;
    s->sensor_frame = sensor_frame_;
  }
  s->producer_session_id=producer_session_id_;s->generation=generation_;s->config_id=config_id_;
  s->pose_ready = out.pose_ready;
  s->processed = out.processed;
  s->reason = out.reason;
  s->fusion_ran = out.fusion_ran;
  s->camera_active = out.fusion.camera_active;
  s->camera_reason = out.fusion.camera_reason;
  s->arrays = out.arrays;
  s->hd_map_audit = out.hd_map_audit;
  const auto ownership_start=Clock::now();
  // Exact sampled XYZ membership, never an OBB test or a second map query.
  // Read-only and bounded by the existing sampled cloud cap.
  if(s->stages&&s->hd_map_audit.enabled) {
    using Key=std::array<std::uint32_t,3>;
    struct Hasher {std::size_t operator()(const Key& k)const{return (std::size_t(k[0])*73856093)^(std::size_t(k[1])*19349663)^(std::size_t(k[2])*83492791);}};
    const auto key=[](const float* p){Key k;std::memcpy(k.data(),p,12);return k;};
    std::unordered_map<Key,std::size_t,Hasher> samples;samples.reserve(s->leveled.sent);
    s->leveled_track.assign(s->leveled.sent,-1);
    for(std::size_t i=0;i<s->leveled.sent;++i)samples.emplace(key(s->leveled.xyzi.data()+4*i),i);
    for(const auto& obj:out.result.objects)if(obj.tracking.s32_MissCount==0)
      for(const auto& p:obj.detection.pcl_ClusterPoints.points) {
        const float xyz[3]={p.x,p.y,p.z};const auto it=samples.find(key(xyz));
        if(it!=samples.end()) {
          auto& id=s->leveled_track[it->second];
          id=(id==-1||id==obj.tracking.s32_Id)?obj.tracking.s32_Id:-2;
        }
      }
    s->capture_ms+=Ms(ownership_start,Clock::now());
    std::lock_guard<std::mutex> lock(counters_mutex_);
    counters_.capture_ms_last=s->capture_ms;
    counters_.capture_ms_max=std::max(counters_.capture_ms_max,s->capture_ms);
  }
  s->tracks.clear();
  if (out.processed) {
    s->tracks.reserve(out.result.objects.size());
    for (const auto& o : out.result.objects) s->tracks.push_back(Summary(o));
  }
  Enqueue(std::move(s));
}

void DebugPublisher::Enqueue(std::unique_ptr<Snapshot> snapshot) {
  {
    std::lock_guard<std::mutex> lock(slot_mutex_);
    if (slot_) {
      std::lock_guard<std::mutex> counters(counters_mutex_);
      ++counters_.overwritten;
    }
    slot_ = std::move(snapshot);
  }
  slot_cv_.notify_one();
}

void DebugPublisher::PublishLoop() {
  while (true) {
    std::unique_ptr<Snapshot> s;
    {
      std::unique_lock<std::mutex> lock(slot_mutex_);
      slot_cv_.wait(lock, [this] { return stop_ || slot_; });
      if (stop_) return;
      s = std::move(slot_);
    }
    Publish(*s);
  }
}

void DebugPublisher::Publish(const Snapshot& s) {
  const auto t0 = Clock::now();
  if (s.stages) {
    if (raw_pub_.getNumSubscribers() > 0) raw_pub_.publish(CloudMsg(s.raw, s.stamp_ns, s.sensor_frame, &s.raw_label, nullptr));
    if (roi_pub_.getNumSubscribers() > 0) roi_pub_.publish(CloudMsg(s.roi, s.stamp_ns, s.sensor_frame, nullptr, nullptr));
    if (voxel_pub_.getNumSubscribers() > 0)
      voxel_pub_.publish(CloudMsg(s.voxel, s.stamp_ns, s.sensor_frame, nullptr, nullptr));
    if (leveled_pub_.getNumSubscribers() > 0)
      leveled_pub_.publish(CloudMsg(s.leveled, s.stamp_ns, config_.leveled_frame, &s.leveled_label, &s.leveled_cluster,s.leveled_track.empty()?nullptr:&s.leveled_track));
  }
  if (scan_pub_.getNumSubscribers() > 0) {
    std_msgs::String text;
    text.data = ScanJson(s);
    scan_pub_.publish(text);
  }
  const double encode_ms = Ms(t0, Clock::now());
  std::lock_guard<std::mutex> lock(counters_mutex_);
  ++counters_.published;
  counters_.encode_ms_last = encode_ms;
  counters_.encode_ms_max = std::max(counters_.encode_ms_max, encode_ms);
}

std::string DebugPublisher::ScanJson(const Snapshot& s) const {
  std::ostringstream out;
  out << std::setprecision(17);
  out << "{\"schema\":\"asmc_lidar_debug_scan_v1\",\"producer\":\"lidar_pipeline_node\",\"seq\":" << s.seq
      << ",\"stamp_ns\":" << s.stamp_ns << ",\"arrival_ns\":" << s.arrival_ns << ",\"pose_ready\":";
  trace::Bool(out, s.pose_ready);
  out << ",\"processed\":";
  trace::Bool(out, s.processed);
  out << ",\"reason\":";
  trace::Str(out, s.reason);
  out << ",\"producer_session_id\":";trace::Str(out,s.producer_session_id);
  out << ",\"generation\":"<<s.generation<<",\"config_id\":";trace::Str(out,s.config_id);
  out << ",\"frames\":{\"sensor\":";
  trace::Str(out, s.sensor_frame);
  out << ",\"leveled\":";
  trace::Str(out, config_.leveled_frame);
  out << ",\"base\":\"base_link\",\"map\":\"map\"},\"config\":{\"roll_leveling\":";
  trace::Bool(out, use_roll_);
  out << ",\"tunnel_filter\":";
  trace::Bool(out, tunnel_enabled_);
  out << ",\"rate_hz\":";
  trace::Num(out, config_.rate_hz);
  out << ",\"max_points\":" << config_.max_points << ",\"roi\":{\"range_m\":["<<range_.f32_Min<<','<<range_.f32_Max
      <<"],\"x_m\":["<<roi_.f32_MinX<<','<<roi_.f32_MaxX<<"],\"y_m\":["<<roi_.f32_MinY<<','<<roi_.f32_MaxY
      <<"],\"sensor_z_m\":["<<roi_.f32_MinZ<<','<<roi_.f32_MaxZ<<"],\"height_enabled\":";
  trace::Bool(out,height_roi_.enabled);out<<",\"height_above_ground_m\":"<<height_roi_.max_height_above_ground_m
      <<",\"base_height_m\":"<<height_roi_.nominal_base_height_m<<"}},\"vehicle\":{\"length\":";
  trace::Num(out, vehicle_.f32_Length);
  out << ",\"width\":";
  trace::Num(out, vehicle_.f32_Width);
  out << ",\"wheelbase\":";
  trace::Num(out, vehicle_.f32_WheelBase);
  out << ",\"front_overhang\":";
  trace::Num(out, vehicle_.f32_FrontOverhang);
  out << ",\"rear_overhang\":";
  trace::Num(out, vehicle_.f32_RearOverhang);
  out << ",\"lidar\":";
  trace::Vec3(out, vehicle_.st_LiDARPos.f32_X, vehicle_.st_LiDARPos.f32_Y, vehicle_.st_LiDARPos.f32_Z);
  out << "},\"stages\":";
  trace::Bool(out, s.stages);
  if (s.stages) {
    out << ",\"local\":{\"roll_rad\":";
    trace::Num(out, s.roll_rad);
    out << ",\"pitch_rad\":";
    trace::Num(out, s.pitch_rad);
    out << ",\"yaw_rad\":";
    trace::Num(out, s.yaw_rad);
    out << ",\"speed_mps\":";
    trace::Num(out, s.speed_mps);
    out << "},\"leveling\":";
    trace::RowMajor(out, s.leveling);
    out << ",\"base_from_sensor\":";
    trace::RowMajor(out, s.base_from_sensor.matrix());
    out << ",\"base_from_leveled\":";
    if (s.base_from_leveled) trace::RowMajor(out, s.base_from_leveled->matrix());
    else out << "null";
    out << ",\"map\":{\"valid\":";
    trace::Bool(out, s.map_valid);
    out << ",\"reason\":";
    trace::Str(out, s.map_reason);
    out << ",\"map_from_base\":";
    trace::RowMajor(out, s.map_from_base.matrix());
    const auto& n = s.counts;
    out << "},\"counts\":{\"raw\":" << n.raw_points << ",\"roi\":" << n.roi_points << ",\"voxel\":" << n.voxel_points
        << ",\"leveled\":" << n.leveled_points << ",\"ground_kept\":" << n.ground_kept_points
        << ",\"height_kept\":"<<n.height_kept_points<<",\"height_removed\":"<<n.height_removed_points
        << ",\"tunnel_removed\":" << n.tunnel_removed_points << ",\"cluster_input\":" << n.cluster_input_points
        << ",\"clusters\":" << n.clusters << ",\"centroids\":" << n.centroids << ",\"detections\":" << n.detections
        << ",\"tracks\":" << n.tracks << ",\"observed\":" << n.observed_tracks << ",\"predicted\":" << n.predicted_tracks
        << '}';
    const auto& t = s.timings;
    out << ",\"timings\":{\"roi_ms\":";
    trace::Num(out, t.roi_ms);
    out << ",\"voxel_ms\":";
    trace::Num(out, t.voxel_ms);
    out << ",\"level_ms\":";
    trace::Num(out, t.level_ms);
  out << ",\"height_ms\":";trace::Num(out,t.height_ms);
    out << ",\"ground_ms\":";
    trace::Num(out, t.ground_ms);
    out << ",\"tunnel_ms\":";
    trace::Num(out, t.tunnel_ms);
    out << ",\"cluster_ms\":";
    trace::Num(out, t.cluster_ms);
    out << ",\"detection_ms\":";
    trace::Num(out, t.detection_ms);
    out << ",\"tracking_ms\":";
    trace::Num(out, t.tracking_ms);
    out << ",\"post_ms\":";
    trace::Num(out, t.post_ms);
    out << ",\"total_ms\":";
    trace::Num(out, t.total_ms);
    out << "},\"sampled\":{\"raw\":";
    SampleJson(out, s.raw);
    out << ",\"roi\":";
    SampleJson(out, s.roi);
    out << ",\"voxel\":";
    SampleJson(out, s.voxel);
    out << ",\"leveled\":";
    SampleJson(out, s.leveled);
    out << "},\"plane\":{\"valid\":";
    trace::Bool(out, s.plane.valid);
    out << ",\"coefficients\":[";
    for (std::size_t i = 0; i < s.plane.coefficients.size(); ++i) {
      if (i) out << ',';
      trace::Num(out, s.plane.coefficients[i]);
    }
    out << "]},\"tunnel\":{\"ran\":";
    trace::Bool(out, s.tunnel_ran);
    out << ",\"removed\":" << s.tunnel_removed << "},\"detections\":[";
    for (std::size_t i = 0; i < s.detections.size(); ++i) {
      if (i) out << ',';
      Object(out, s.detections[i], false);
    }
    out << "],\"tracks\":[";
    for (std::size_t i = 0; i < s.tracks.size(); ++i) {
      if (i) out << ',';
      Object(out, s.tracks[i], true);
    }
    out << "],\"capture_ms\":";
    trace::Num(out, s.capture_ms);
  }
  out << ",\"fusion\":{\"ran\":";
  trace::Bool(out, s.fusion_ran);
  out << ",\"camera_active\":";
  trace::Bool(out, s.camera_active);
  out << ",\"camera_reason\":";
  trace::Str(out, s.camera_reason);
  out << "},\"typed\":{\"local\":";
  trace::Array(out, s.arrays.accepted, s.arrays.local);
  out << ",\"map\":";
  trace::Array(out, s.arrays.accepted, s.arrays.map);
  out << '}';
  if(s.hd_map_audit.enabled){out << ",\"hd_map_audit\":";filtering::WriteAuditReport(out,s.hd_map_audit);}
  out << '}';
  return out.str();
}

DebugCounters DebugPublisher::counters() const {
  std::lock_guard<std::mutex> lock(counters_mutex_);
  return counters_;
}

}  // namespace runtime
}  // namespace lidar
}  // namespace asmc
