// Offline experiment harness: replays Lidar_Data scenes (npy point clouds + t_lidar ego pose) or a
// MORAI bag through the unmodified production LiDAR path
//   SensorPoseHistory -> PrepareScan -> LidarProcess -> FusionProcessAt -> GateInputs/Decide -> BuildV2
// and writes one JSON line per scan (V2 /lidar/local_object payload + internal track state).
// The production entrypoint is never called: no UDP sender, no sensor/worker threads. Like
// lidar_continuous_replay it needs a ROS master; run it only inside an isolated network namespace.
// Optional --points-out writes, per processed scan, base_from_input and the cluster points of every
// observed track (tracking input frame) for offline split/over-merge and observability analysis.
#define main lidar_runtime_entrypoint
#include "../src/main.cpp"
#undef main
#include <Lidar/ImuRansac/ImuRansac.hpp>
#include <PlannerInterface/PlannerObjectAdapter.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sys/resource.h>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace {
struct FrameRow {
    long idx = 0;
    std::string stem, npy;
    std::uint64_t stamp_ns = 0;
    double ego_x = 0, ego_y = 0, ego_z = 0, ego_yaw = 0, ego_speed = 0;
};

std::vector<std::string> SplitCsv(const std::string& line) {
    std::vector<std::string> out; std::string cell; std::istringstream in(line);
    while (std::getline(in, cell, ',')) out.push_back(cell);
    if (!line.empty() && line.back() == ',') out.push_back("");
    return out;
}

std::vector<FrameRow> ReadFrames(const std::string& path) {
    std::ifstream in(path); if (!in) throw std::runtime_error("frames open failed: " + path);
    std::string line; std::getline(in, line);
    const auto header = SplitCsv(line);
    auto col = [&](const std::string& name) {
        for (std::size_t i = 0; i < header.size(); ++i) if (header[i] == name) return i;
        throw std::runtime_error("missing column " + name);
    };
    const auto c_idx = col("idx"), c_stem = col("stem"), c_ns = col("stamp_ns"), c_npy = col("npy"),
        c_x = col("ego_x"), c_y = col("ego_y"), c_z = col("ego_z"), c_yaw = col("ego_yaw"),
        c_v = col("ego_speed_mps");
    std::vector<FrameRow> rows;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        const auto c = SplitCsv(line);
        FrameRow r; r.idx = std::stol(c[c_idx]); r.stem = c[c_stem]; r.stamp_ns = std::stoull(c[c_ns]);
        r.npy = c[c_npy]; r.ego_x = std::stod(c[c_x]); r.ego_y = std::stod(c[c_y]); r.ego_z = std::stod(c[c_z]);
        r.ego_yaw = std::stod(c[c_yaw]); r.ego_speed = std::stod(c[c_v]);
        if (!rows.empty() && r.stamp_ns <= rows.back().stamp_ns) throw std::runtime_error("non-increasing stamp");
        rows.push_back(r);
    }
    return rows;
}

// Competition-restricted input stream (frames_comp.csv). Only the ALLOWED inference columns are looked up
// by name; any other column (evaluation pose, vel_y/z, accel, ...) is never read, so it cannot reach the
// tracker. vel_x = Competition Vehicle Status vel_x (m/s, signed), imu_* = IMU orientation (rad, ENU).
// imu_roll_rad / imu_pitch_rad are optional and may be NaN (= not available, NOT measured zero).
struct CompRow {
    long idx = 0;
    std::string stem, npy;
    std::uint64_t stamp_ns = 0;
    double vel_x = NAN, imu_yaw = NAN, imu_roll = NAN, imu_pitch = NAN;
};

std::vector<CompRow> ReadCompetitionFrames(const std::string& path) {
    std::ifstream in(path); if (!in) throw std::runtime_error("frames open failed: " + path);
    std::string line; std::getline(in, line);
    const auto header = SplitCsv(line);
    auto find = [&](const std::string& name) -> long {
        for (std::size_t i = 0; i < header.size(); ++i) if (header[i] == name) return static_cast<long>(i);
        return -1;
    };
    auto need = [&](const std::string& name) {
        const long i = find(name); if (i < 0) throw std::runtime_error("missing allowed column " + name); return i;
    };
    const long c_idx = need("idx"), c_stem = need("stem"), c_ns = need("stamp_ns"), c_npy = need("npy"),
        c_vx = need("vel_x_mps"), c_yaw = need("imu_yaw_rad"), c_roll = find("imu_roll_rad"), c_pitch = find("imu_pitch_rad");
    std::vector<CompRow> rows;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        const auto c = SplitCsv(line);
        CompRow r; r.idx = std::stol(c[c_idx]); r.stem = c[c_stem]; r.stamp_ns = std::stoull(c[c_ns]); r.npy = c[c_npy];
        r.vel_x = std::stod(c[c_vx]); r.imu_yaw = std::stod(c[c_yaw]);
        if (c_roll >= 0) r.imu_roll = std::stod(c[c_roll]);
        if (c_pitch >= 0) r.imu_pitch = std::stod(c[c_pitch]);
        if (!rows.empty() && r.stamp_ns <= rows.back().stamp_ns) throw std::runtime_error("non-increasing stamp");
        rows.push_back(r);
    }
    return rows;
}

// ZYX (roll, pitch, yaw) -> quaternion, same convention as Convert() below.
sensor_msgs::Imu ImuFromRpy(std::uint64_t stamp_ns, double roll, double pitch, double yaw) {
    sensor_msgs::Imu imu; imu.header.stamp.fromNSec(stamp_ns);
    const double cr = std::cos(0.5 * roll), sr = std::sin(0.5 * roll), cp = std::cos(0.5 * pitch), sp = std::sin(0.5 * pitch),
        cy = std::cos(0.5 * yaw), sy = std::sin(0.5 * yaw);
    imu.orientation.w = cr * cp * cy + sr * sp * sy;
    imu.orientation.x = sr * cp * cy - cr * sp * sy;
    imu.orientation.y = cr * sp * cy + sr * cp * sy;
    imu.orientation.z = cr * cp * sy - sr * sp * cy;
    return imu;
}

// Minimal .npy (v1/v2, little-endian float32, C order, shape (N, K>=4)) reader.
std::vector<float> LoadNpyXyzi(const std::string& path, std::size_t* count) {
    std::ifstream in(path, std::ios::binary); if (!in) throw std::runtime_error("npy open failed: " + path);
    char magic[6]; in.read(magic, 6);
    if (std::string(magic, 6) != std::string("\x93NUMPY", 6)) throw std::runtime_error("not npy: " + path);
    unsigned char ver[2]; in.read(reinterpret_cast<char*>(ver), 2);
    std::uint32_t header_len = 0;
    if (ver[0] == 1) { std::uint16_t h; in.read(reinterpret_cast<char*>(&h), 2); header_len = h; }
    else { in.read(reinterpret_cast<char*>(&header_len), 4); }
    std::string header(header_len, ' '); in.read(&header[0], header_len);
    if (header.find("'descr': '<f4'") == std::string::npos) throw std::runtime_error("npy dtype not <f4: " + path);
    if (header.find("'fortran_order': False") == std::string::npos) throw std::runtime_error("npy fortran: " + path);
    const auto s = header.find("'shape': ("); if (s == std::string::npos) throw std::runtime_error("npy shape");
    const auto e = header.find(')', s);
    const auto dims = SplitCsv(header.substr(s + 10, e - s - 10));
    const std::size_t n = std::stoull(dims.at(0)), k = std::stoull(dims.at(1));
    if (k < 4) throw std::runtime_error("npy needs >=4 columns: " + path);
    std::vector<float> all(n * k); in.read(reinterpret_cast<char*>(all.data()), all.size() * sizeof(float));
    if (!in) throw std::runtime_error("npy truncated: " + path);
    std::vector<float> xyzi(n * 4);
    for (std::size_t i = 0; i < n; ++i) for (int j = 0; j < 4; ++j) xyzi[i * 4 + j] = all[i * k + j];
    *count = n; return xyzi;
}

IMU_DATA Convert(const sensor_msgs::Imu& m) {   // identical to lidar_continuous_replay.cpp
    IMU_DATA d{};
    d.f32_QuatX=m.orientation.x;d.f32_QuatY=m.orientation.y;
    d.f32_QuatZ=m.orientation.z;d.f32_QuatW=m.orientation.w;
    const double x=d.f32_QuatX,y=d.f32_QuatY,z=d.f32_QuatZ,w=d.f32_QuatW;
    d.f32_Roll_rad=atan2(2*(w*x+y*z),1-2*(x*x+y*y));
    d.f32_Pitch_rad=asin(2*(w*y-z*x));
    d.f32_Yaw_rad=atan2(2*(w*z+x*y),1-2*(y*y+z*z));
    d.f32_Roll_deg=Rad2Deg(d.f32_Roll_rad);d.f32_Pitch_deg=Rad2Deg(d.f32_Pitch_rad);
    d.f32_Yaw_deg=Rad2Deg(d.f32_Yaw_rad);
    const double norm=x*x+y*y+z*z+w*w;
    d.b_ImuReady=!m.header.stamp.isZero()&&std::isfinite(norm)&&fabs(norm-1)<=.001;
    return d;
}

void MapInputs(GPS_IMU& local,const asmc::lidar::PoseSelection& map) {   // identical to lidar_continuous_replay.cpp
    local.st_GPS.b_isJamming=!map.valid;
    if(map.valid) {
        const auto& v=st_VehicleSpec.st_GPSPos;
        const Eigen::Vector3d gps=map.map_from_base.translation()+map.map_from_base.rotation()*Eigen::Vector3d(v.f32_X,v.f32_Y,v.f32_Z);
        local.st_GPS.f64_E=gps.x();local.st_GPS.f64_N=gps.y();local.st_GPS.f64_U=gps.z();
    } else local.st_GPS.f64_E=local.st_GPS.f64_N=local.st_GPS.f64_U=NAN;
}

void SetParam(ros::NodeHandle& pnh, const std::string& key, const YAML::Node& v) {
    const std::string s = v.as<std::string>();
    if (s == "true" || s == "True") { pnh.setParam(key, true); return; }
    if (s == "false" || s == "False") { pnh.setParam(key, false); return; }
    char* end = nullptr; const long i = std::strtol(s.c_str(), &end, 10);
    if (end && *end == '\0' && !s.empty()) { pnh.setParam(key, static_cast<int>(i)); return; }
    end = nullptr; const double d = std::strtod(s.c_str(), &end);
    if (end && *end == '\0' && !s.empty()) { pnh.setParam(key, d); return; }
    pnh.setParam(key, s);
}

// Parameter file = `roslaunch --dump-params` output; every scalar below `/Main/` becomes a private param.
std::size_t LoadMainParams(ros::NodeHandle& pnh, const std::string& path) {
    const auto root = YAML::LoadFile(path); std::size_t n = 0;
    for (const auto& kv : root) {
        const std::string key = kv.first.as<std::string>();
        if (key.rfind("/Main/", 0) != 0 || !kv.second.IsScalar()) continue;
        SetParam(pnh, key.substr(6), kv.second); ++n;
    }
    return n;
}

// JSON number, or null for NaN/Inf (JSON has no NaN).
struct J { double v; };
std::ostream& operator<<(std::ostream& out, J n) { if (std::isfinite(n.v)) out << n.v; else out << "null"; return out; }
void Vec2(std::ostream& out, double a, double b) { out << '[' << J{a} << ',' << J{b} << ']'; }

struct Stats { std::uint64_t scans = 0, ready = 0, processed = 0, local = 0, map = 0, none = 0; };

// Binary point dump, little endian, one record per scan:
//   char[4] "PTS1", int64 idx, int32 valid (base_from_input available), int32 n_tracks,
//   float64[12] base_from_input row-major 3x4, then per track: int32 id, int32 n, float32[3*n] xyz.
template <class T> void Bin(std::ostream& out, T v) { out.write(reinterpret_cast<const char*>(&v), sizeof(v)); }
void WritePoints(std::ostream& out, long idx, const LiDAR& lidar, bool processed) {
    out.write("PTS1", 4); Bin<std::int64_t>(out, idx);
    std::optional<asmc::lidar::Pose> base;
    if (processed) base = asmc::lidar::base_from_input(lidar.frame_context);
    std::vector<const LiDARObjectInfo*> tracks;
    if (processed)
        for (const auto& o : lidar.vec_Objects)
            if (o.s32_MissCount == 0 && o.s32_Id >= 0 && !o.pcl_ClusterPoints.empty()) tracks.push_back(&o);
    Bin<std::int32_t>(out, base ? 1 : 0); Bin<std::int32_t>(out, static_cast<std::int32_t>(tracks.size()));
    const Eigen::Matrix4d m = base ? base->matrix() : Eigen::Matrix4d::Identity();
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 4; ++c) Bin<double>(out, m(r, c));
    for (const auto* o : tracks) {
        Bin<std::int32_t>(out, o->s32_Id); Bin<std::int32_t>(out, static_cast<std::int32_t>(o->pcl_ClusterPoints.size()));
        for (const auto& p : o->pcl_ClusterPoints.points) { Bin<float>(out, p.x); Bin<float>(out, p.y); Bin<float>(out, p.z); }
    }
}

class Replay {
public:
    Replay(std::ostream& out, bool log_cov, std::ostream* points) : out_(out), log_cov_(log_cov), points_(points) {}
    void Scan(const std::string& scene, long idx, const std::string& stem, std::uint64_t source_ns,
              const float* xyzi, std::size_t n, SensorPoseHistory& history, MotionClassifier& classifier,
              std::uint64_t logical) {
        ++generation_; ++stats_.scans;
        ros::Time stamp; stamp.fromNSec(source_ns); ros::Time::setNow(stamp);
        track_trace::Sink().SetScan(idx, source_ns);
        const auto w0 = std::chrono::steady_clock::now();
        GPS_IMU local{}; LocalInputSelection selected; std::string reason;
        const bool ready = history.WaitForLocalLookupNs(source_ns, 0, &local, &selected, &reason);
        out_ << "{\"scene\":" << fusion_trace::Quote(scene) << ",\"idx\":" << idx << ",\"stem\":"
             << fusion_trace::Quote(stem) << ",\"stamp_ns\":" << source_ns << ",\"raw_points\":" << n
             << ",\"ready\":" << (ready ? "true" : "false") << ",\"reason\":" << fusion_trace::Quote(reason)
             << scan_extra_;
        if (!ready && track_trace::Sink().Enabled()) {
            track_trace::Event("scan_skipped", [&](std::ostream& o) { o << ",\"reason\":" << fusion_trace::Quote(reason); });
        }
        bool processed = false; double lidar_ms = 0.;
        output_gate::Decision effective;
        frame_boundary::V2Frame local_frame;
        if (ready) {
            ++stats_.ready;
            asmc::lidar::Observation observation{static_cast<asmc::lidar::Ns>(source_ns),
                static_cast<asmc::lidar::Ns>(logical), static_cast<asmc::lidar::Ns>(logical), "ros", "velodyne", "steady"};
            const auto map = history.MapAt(observation, local);
            frame_boundary::PrepareScan(observation, local, st_VehicleSpec, map, generation_);
            MapInputs(local, map);
            if (n * sizeof(pcl::PointXYZI) > sizeof(raw_->arc_Buffer)) throw std::runtime_error("raw capacity");
            raw_->u64_Timestamp = source_ns / 1000000; raw_->source_stamp_ns = source_ns;
            raw_->receipt_steady_ns = logical; raw_->s32_Num = static_cast<int32>(n);
            pcl::PointXYZI* dst = reinterpret_cast<pcl::PointXYZI*>(raw_->arc_Buffer);
            for (std::size_t i = 0; i < n; ++i) {
                pcl::PointXYZI p; p.x = xyzi[4 * i]; p.y = xyzi[4 * i + 1]; p.z = xyzi[4 * i + 2];
                p.intensity = xyzi[4 * i + 3]; std::memcpy(dst + i, &p, sizeof(p));
            }
            const auto t0 = std::chrono::steady_clock::now();
            processed = LidarProcess(*raw_, local, st_LiDAR, classifier);
            lidar_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            if (processed && b_EnableSensorFusion) {
                ros::WallTime decision; decision.fromNSec(logical);
                FusionProcessAt(raw_yolo_, st_LiDAR, st_Camera, frame_boundary::SourceStamp(*raw_), decision);
            }
            if (processed) ++stats_.processed;
        }
        // Same selection as main.cpp TemporalThread + Publisher::PublishV2 (without publishing).
        const auto gate = frame_boundary::GateInputs(ready, reason, processed, st_LiDAR, generation_);
        effective = output_gate::Decide(gate);
        if (effective.v2 != output_gate::Mode::kNone) {
            const auto context_ns = st_LiDAR.frame_context.time.stamp_ns;
            if (!last_v2_stamp_.isZero() && stamp <= last_v2_stamp_) {
                effective = output_gate::Downgrade(effective, "v2_stamp_rejected");
            } else if (context_ns <= 0 || static_cast<std::uint64_t>(context_ns) != source_ns) {
                effective = output_gate::Downgrade(effective, "context_stamp_mismatch");
            } else {
                const bool wants_local = effective.v2 == output_gate::Mode::kLocal || effective.v2 == output_gate::Mode::kBoth;
                const bool wants_map = effective.v2 == output_gate::Mode::kMap || effective.v2 == output_gate::Mode::kBoth;
                if (wants_local) {
                    local_frame = frame_boundary::BuildV2(st_LiDAR, output_gate::Mode::kLocal);
                    if (!local_frame.valid) effective = output_gate::Downgrade(effective, local_frame.reason);
                }
                if (effective.v2 != output_gate::Mode::kNone && wants_map) {
                    const auto map_frame = frame_boundary::BuildV2(st_LiDAR, output_gate::Mode::kMap);
                    if (!map_frame.valid) {
                        if (local_frame.valid) { effective.v2 = output_gate::Mode::kLocal; effective.reason = "local_only:" + map_frame.reason; }
                        else effective = output_gate::Downgrade(effective, map_frame.reason);
                    }
                }
            }
        }
        const double frame_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - w0).count();
        if (track_trace::Sink().Enabled() && processed) TracePublish(effective, local_frame, local);
        if (effective.v2 == output_gate::Mode::kBoth) { ++stats_.local; ++stats_.map; last_v2_stamp_ = stamp; }
        else if (effective.v2 == output_gate::Mode::kMap) { ++stats_.map; last_v2_stamp_ = stamp; }
        else if (effective.v2 == output_gate::Mode::kLocal) { ++stats_.local; last_v2_stamp_ = stamp; }
        else ++stats_.none;
        const bool local_out = effective.v2 == output_gate::Mode::kLocal || effective.v2 == output_gate::Mode::kBoth;
        out_ << ",\"processed\":" << (processed ? "true" : "false")
             << ",\"v2_mode\":\"" << output_gate::ModeName(effective.v2) << "\",\"v2_reason\":" << fusion_trace::Quote(effective.reason)
             << ",\"timing\":{\"lidar_process_ms\":" << lidar_ms << ",\"frame_ms\":" << frame_ms << '}';
        // Stage counts of this scan (g_LidarStageProfile is reset at the start of every LidarProcess call):
        // clusters = adaptive clusters BEFORE ConsolidateCurrentDetections; det = observed tracks after it.
        if (processed) {
            const auto& prof = g_LidarStageProfile;
            std::size_t det = 0;
            for (const auto& o : st_LiDAR.vec_Objects) if (o.s32_MissCount == 0 && o.s32_Id >= 0) ++det;
            out_ << ",\"prof\":{\"roi\":" << prof.roi_points << ",\"voxel\":" << prof.voxel_points << ",\"ransac\":" << prof.ransac_points
                 << ",\"clusters\":" << prof.clusters << ",\"det\":" << det
                 << ",\"plane_valid\":" << (CurrentGroundPlaneSnapshot().valid ? 1 : 0) << '}';
        }
        if (points_) WritePoints(*points_, idx, st_LiDAR, processed);
        // Base-from-tracking-input rotation (leveled sensor -> base_link) for the legacy KF velocity.
        Eigen::Matrix2d base_rot = Eigen::Matrix2d::Identity();
        if (processed) {
            if (const auto b = asmc::lidar::base_from_input(st_LiDAR.frame_context)) base_rot = b->rotation().topLeftCorner<2, 2>();
        }
        out_ << ",\"local\":[";
        if (local_out) {
            bool sep = false;
            for (const auto& o : local_frame.array.objects) {
                if (sep) out_ << ','; sep = true;
                out_ << "{\"id\":" << o.track_id << ",\"c\":[" << J{o.center.x} << ',' << J{o.center.y} << ',' << J{o.center.z}
                     << "],\"yaw\":" << J{o.yaw} << ",\"l\":" << J{o.length} << ",\"w\":" << J{o.width} << ",\"h\":" << J{o.height}
                     << ",\"age\":" << o.age << ",\"npts\":" << o.source_point_count << ",\"cls\":" << fusion_trace::Quote(o.classification)
                     << ",\"mv\":" << (o.motion_valid ? 1 : 0) << ",\"ms\":" << static_cast<int>(o.motion_state) << '}';
            }
        }
        out_ << "],\"tracks\":[";
        if (processed) {
            bool sep = false;
            for (const auto& o : st_LiDAR.vec_Objects) {
                if (sep) out_ << ','; sep = true;
                const Eigen::Vector2d v = base_rot * Eigen::Vector2d(o.f32_AbsVx, o.f32_AbsVy);
                double mx = NAN, my = NAN;
                if (o.s32_MissCount == 0 && !o.pcl_ClusterPoints.empty()) {
                    double sx = 0, sy = 0; std::size_t k = 0;
                    for (const auto& p : o.pcl_ClusterPoints.points)
                        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) { sx += p.x; sy += p.y; ++k; }
                    if (k) { mx = sx / k; my = sy / k; }
                }
                out_ << "{\"id\":" << o.s32_Id << ",\"age\":" << o.s32_Age << ",\"miss\":" << o.s32_MissCount
                     << ",\"cls\":" << fusion_trace::Quote(o.str_ObjectClass) << ",\"kfi\":" << (o.b_KfInitialized ? 1 : 0)
                     << ",\"kfc\":" << o.s32_KfInitCount << ",\"kf\":[";
                for (int j = 0; j < 6; ++j) { if (j) out_ << ','; out_ << J{o.Eigen_KfState_CA(j)}; }
                out_ << "],\"vb\":"; Vec2(out_, v.x(), v.y());
                out_ << ",\"meas\":"; Vec2(out_, mx, my);
                // f32_PredCore* is written only when CompareCentroids matches the track; miss==0 && age>1
                // is exactly "matched in this scan" (a new track has age 1), otherwise the field is unset.
                if (o.s32_MissCount == 0 && o.s32_Age > 1) { out_ << ",\"pred\":"; Vec2(out_, o.f32_PredCoreX, o.f32_PredCoreY); }
                else out_ << ",\"pred\":null";
                out_ << ",\"obb\":[" << J{o.f32_OBB_CenterX} << ',' << J{o.f32_OBB_CenterY} << ',' << J{o.f32_OBB_CenterZ} << ','
                     << J{o.f32_OBB_Width} << ',' << J{o.f32_OBB_Height} << ',' << J{o.f32_OBB_ZMin} << ',' << J{o.f32_OBB_ZMax} << ','
                     << J{o.f32_OBB_YawRad} << "],\"npts\":" << o.pcl_ClusterPoints.size()
                     << ",\"mv\":" << (o.motion_valid ? 1 : 0) << ",\"ms\":" << o.s32_MotionState;
                if (log_cov_) {
                    out_ << ",\"P\":[";
                    for (int r = 0; r < 6; ++r) for (int c = 0; c < 6; ++c) { if (r || c) out_ << ','; out_ << J{o.Eigen_KfP_CA(r, c)}; }
                    out_ << ']';
                }
                out_ << '}';
            }
        }
        out_ << "]}\n";
    }
    const Stats& stats() const { return stats_; }
    void SetScanExtra(const std::string& extra) { scan_extra_ = extra; }
private:
    // Final values of this scan as a consumer would read them: the V2 local objects (base_link) and,
    // when the map pose is valid, MakePlannerObjects (the values behind /last_lidar and the UDP sender,
    // before the planner-side filters). Tracks that are not published get an explicit reason.
    void TracePublish(const output_gate::Decision& effective, const frame_boundary::V2Frame& local_frame,
                      const GPS_IMU& local) {
        track_trace::Event("publish", [&](std::ostream& o) {
            o << ",\"v2_mode\":\"" << output_gate::ModeName(effective.v2) << "\",\"v2_reason\":" << fusion_trace::Quote(effective.reason)
              << ",\"local\":[";
            bool sep = false;
            if (effective.v2 == output_gate::Mode::kLocal || effective.v2 == output_gate::Mode::kBoth) {
                for (const auto& ob : local_frame.array.objects) {
                    if (sep) o << ','; sep = true;
                    o << "{\"id\":" << ob.track_id << ",\"c\":[" << ob.center.x << ',' << ob.center.y << ',' << ob.center.z
                      << "],\"yaw\":" << ob.yaw << ",\"l\":" << ob.length << ",\"w\":" << ob.width << ",\"h\":" << ob.height
                      << ",\"vel_valid\":" << (ob.absolute_velocity_valid ? 1 : 0) << ",\"mv\":" << (ob.motion_valid ? 1 : 0)
                      << ",\"ms\":" << static_cast<int>(ob.motion_state) << '}';
                }
            }
            o << "],\"unpublished\":[";
            sep = false;
            for (const auto& t : st_LiDAR.vec_Objects) {
                const char* why = t.s32_Id < 0 ? "no_id" : t.s32_MissCount > 0 ? "missed_scan" :
                    t.pcl_ClusterPoints.empty() ? "no_points" :
                    (t.str_ObjectClass == "noise" || t.str_ObjectClass == "지면노이즈") ? "noise_class" : nullptr;
                if (!why) continue;
                if (sep) o << ','; sep = true;
                o << "{\"id\":" << t.s32_Id << ",\"miss\":" << t.s32_MissCount << ",\"reason\":\"" << why << "\"}";
            }
            o << "],\"planner\":";
            if (frame_boundary::Current().map_pose.valid) {
                o << '[';
                sep = false;
                for (const auto& p : MakePlannerObjects(st_LiDAR, local, st_VehicleSpec, 200)) {
                    if (sep) o << ','; sep = true;
                    o << "{\"id\":" << p.id << ",\"obb_c_map\":[" << p.obb_center_map.x << ',' << p.obb_center_map.y
                      << "],\"yaw_map\":" << p.obb_yaw_map_rad << ",\"l\":" << p.length << ",\"w\":" << p.width
                      << ",\"v_map\":[" << p.velocity_map_mps.x << ',' << p.velocity_map_mps.y << "],\"raw_v_map\":["
                      << p.raw_velocity_map_mps.x << ',' << p.raw_velocity_map_mps.y << "],\"ms\":" << p.motion_state
                      << ",\"dyn\":" << (p.is_dynamic ? 1 : 0) << '}';
                }
                o << ']';
            } else {
                o << "null,\"planner_reason\":\"map_pose_invalid_no_planner_output\"";
            }
        });
    }
    std::string scan_extra_;
    std::ostream& out_;
    bool log_cov_;
    std::ostream* points_;
    std::uint64_t generation_ = 0;
    ros::Time last_v2_stamp_;
    Stats stats_;
    std::unique_ptr<RAW_LIDAR_DATA> raw_ = std::make_unique<RAW_LIDAR_DATA>();
    RAW_YOLO_DATA raw_yolo_{};
};

SensorPoseHistoryConfig PoseConfig(ros::NodeHandle& pnh) {   // same parameters as ConfigureSensorPoseHistory
    SensorPoseHistoryConfig config;
    pnh.param("imu_yaw_is_true_enu", config.imu_yaw_is_true_enu, config.imu_yaw_is_true_enu);
    pnh.param("lidar_pose_gps_max_age_s", config.gps_max_age_s, config.gps_max_age_s);
    pnh.param("lidar_pose_imu_max_age_s", config.imu_max_age_s, config.imu_max_age_s);
    pnh.param("lidar_pose_can_max_age_s", config.can_max_age_s, config.can_max_age_s);
    pnh.param("lidar_pose_maximum_speed_mps", config.maximum_extrapolation_speed_mps, config.maximum_extrapolation_speed_mps);
    pnh.param("lidar_pose_history_duration_s", config.history_duration_s, config.history_duration_s);
    return config;
}
}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_dataset_replay", ros::init_options::AnonymousName);
    std::string frames_path, bag_path, out_path, params_path, trace_path, points_path, scene = "scene";
    std::vector<std::pair<std::string, std::string>> overrides;
    bool log_cov = false; long max_frames = -1;
    std::string input_mode = "historical", track_trace_path, track_trace_ids, run_id = "replay";
    double track_trace_max_mb = 512.0;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() { if (i + 1 >= argc) throw std::runtime_error("missing value for " + a); return std::string(argv[++i]); };
        if (a == "--frames") frames_path = next();
        else if (a == "--input-mode") input_mode = next();
        else if (a == "--track-trace") track_trace_path = next();
        else if (a == "--track-trace-ids") track_trace_ids = next();
        else if (a == "--track-trace-max-mb") track_trace_max_mb = std::stod(next());
        else if (a == "--run-id") run_id = next();
        else if (a == "--bag") bag_path = next();
        else if (a == "--out") out_path = next();
        else if (a == "--params") params_path = next();
        else if (a == "--trace") trace_path = next();
        else if (a == "--points-out") points_path = next();
        else if (a == "--scene") scene = next();
        else if (a == "--log-cov") log_cov = true;
        else if (a == "--max-frames") max_frames = std::stol(next());
        else if (a == "--set") { const auto kv = next(); const auto eq = kv.find('=');
            if (eq == std::string::npos) throw std::runtime_error("--set needs key=value"); overrides.push_back({kv.substr(0, eq), kv.substr(eq + 1)}); }
        else { std::cerr << "unknown argument " << a << "\n"; return 2; }
    }
    if ((frames_path.empty() == bag_path.empty()) || out_path.empty() || params_path.empty() ||
        (input_mode != "historical" && input_mode != "competition") || (input_mode == "competition" && frames_path.empty())) {
        std::cerr << "usage: lidar_dataset_replay (--frames FRAMES.csv | --bag BAG) --out OUT.jsonl --params DUMP.yaml"
                     " [--scene NAME] [--set key=value]... [--trace TRACE.jsonl] [--log-cov] [--max-frames N]"
                     " [--points-out PTS.bin] [--input-mode historical|competition] [--track-trace PATH"
                     " [--track-trace-ids 1,2] [--track-trace-max-mb N] [--run-id ID]]\n";
        return 2;
    }
    ros::NodeHandle nh, pnh("~");
    const std::size_t n_params = LoadMainParams(pnh, params_path);
    for (const auto& kv : overrides) SetParam(pnh, kv.first, YAML::Node(kv.second));
    if (!track_trace_path.empty()) {
        pnh.setParam("lidar_track_trace_path", track_trace_path);
        pnh.setParam("lidar_track_trace_ids", track_trace_ids);
        pnh.setParam("lidar_track_trace_run_id", run_id);
        pnh.setParam("lidar_track_trace_max_mb", track_trace_max_mb);
    }
    ConfigureTrackingExperiment(pnh);
    std::string camera_profile;
    pnh.param("camera_class_profile", camera_profile, std::string("mission16_v1"));
    if (camera_profile != "legacy" && camera_profile != "mission16_v1") return 2;
    fusion_class::MissionProfile() = camera_profile == "mission16_v1";
    pnh.param("enable_sensor_fusion", b_EnableSensorFusion, true);
    InitMutex(); InitVehicleSpec(); InitProjectionMatrix(st_Camera, YAML::LoadFile(YamlURL));
    frame_boundary::Configure(pnh, st_VehicleSpec, st_Camera);
    if (!frame_boundary::Enabled()) { std::cerr << "harness requires enable_common_geometry=true (V2 output)\n"; return 2; }
    InitLidarSettings(); InitLidarViz(nh); InitFusionLidarViz(nh);
    MotionClassifier classifier(pnh);
    SensorPoseHistory history(PoseConfig(pnh));
    fusion_trace::Sink().Configure(trace_path);
    std::ofstream output(out_path); if (!output) throw std::runtime_error("output open failed");
    output << std::setprecision(9);
    std::unique_ptr<std::ofstream> points;
    if (!points_path.empty()) {
        points = std::make_unique<std::ofstream>(points_path, std::ios::binary);
        if (!*points) throw std::runtime_error("points output open failed");
    }
    std::srand(0);
    Replay replay(output, log_cov, points.get());
    std::uint64_t first_ns = 0, frames_done = 0;
    const auto wall0 = std::chrono::steady_clock::now();
    if (input_mode == "competition") {
        // Allowed channels only: IMU attitude (yaw; roll/pitch when provided) and vel_x as CAN speed,
        // both stamped at t_lidar. No GPS sample is fed (not available in this input) -> map pose invalid.
        const auto rows = ReadCompetitionFrames(frames_path);
        first_ns = rows.empty() ? 0 : rows.front().stamp_ns;
        for (const auto& r : rows) {
            if (max_frames >= 0 && static_cast<long>(frames_done) >= max_frames) break;
            const std::uint64_t logical = r.stamp_ns - first_ns + 1000000000ULL;
            const bool rp = std::isfinite(r.imu_roll) && std::isfinite(r.imu_pitch);
            const auto imu = ImuFromRpy(r.stamp_ns, rp ? r.imu_roll : 0.0, rp ? r.imu_pitch : 0.0, r.imu_yaw);
            if (std::isfinite(r.imu_yaw)) history.AddImu(r.stamp_ns / 1000000, Convert(imu), nullptr, r.stamp_ns, logical);
            if (std::isfinite(r.vel_x)) {
                CAN_DATA can{}; can.f32_CurrentVel_MPS = static_cast<float32>(r.vel_x);
                can.f32_CurrentVel_KPH = static_cast<float32>(3.6 * r.vel_x);
                history.AddCan(r.stamp_ns / 1000000, can, nullptr, r.stamp_ns, logical);
            }
            std::ostringstream extra; extra << std::setprecision(9);
            extra << ",\"ego_in\":{\"vel_x\":" << J{r.vel_x} << ",\"imu_yaw\":" << J{r.imu_yaw}
                  << ",\"rp_available\":" << (rp ? 1 : 0) << ",\"gps\":0}";
            replay.SetScanExtra(extra.str());
            std::size_t n = 0; const auto xyzi = LoadNpyXyzi(r.npy, &n);
            replay.Scan(scene, r.idx, r.stem, r.stamp_ns, xyzi.data(), n, history, classifier, logical);
            ++frames_done;
        }
    } else if (!frames_path.empty()) {
        const auto rows = ReadFrames(frames_path);
        first_ns = rows.empty() ? 0 : rows.front().stamp_ns;
        const auto& g = st_VehicleSpec.st_GPSPos;
        for (const auto& r : rows) {
            if (max_frames >= 0 && static_cast<long>(frames_done) >= max_frames) break;
            const std::uint64_t logical = r.stamp_ns - first_ns + 1000000000ULL;
            // Synthetic IMU/CAN/GPS samples at the scan's own t_lidar: attitude = dataset ego yaw
            // (roll = pitch = 0, the dataset has no attitude), speed = dataset ego speed (central
            // difference of t_lidar ego positions), GPS antenna = T_ego2global * GPS lever arm.
            sensor_msgs::Imu imu; imu.header.stamp.fromNSec(r.stamp_ns);
            imu.orientation.z = std::sin(0.5 * r.ego_yaw); imu.orientation.w = std::cos(0.5 * r.ego_yaw);
            history.AddImu(r.stamp_ns / 1000000, Convert(imu), nullptr, r.stamp_ns, logical);
            CAN_DATA can{}; can.f32_CurrentVel_MPS = static_cast<float32>(r.ego_speed);
            can.f32_CurrentVel_KPH = static_cast<float32>(3.6 * r.ego_speed);
            history.AddCan(r.stamp_ns / 1000000, can, nullptr, r.stamp_ns, logical);
            GPS_DATA gps{};
            gps.f64_E = r.ego_x + std::cos(r.ego_yaw) * g.f32_X - std::sin(r.ego_yaw) * g.f32_Y;
            gps.f64_N = r.ego_y + std::sin(r.ego_yaw) * g.f32_X + std::cos(r.ego_yaw) * g.f32_Y;
            gps.f64_U = r.ego_z + g.f32_Z - frame_boundary::MapAltitudeOrigin();
            history.AddGps(r.stamp_ns / 1000000, gps, r.stamp_ns, logical);
            std::size_t n = 0; const auto xyzi = LoadNpyXyzi(r.npy, &n);
            replay.Scan(scene, r.idx, r.stem, r.stamp_ns, xyzi.data(), n, history, classifier, logical);
            ++frames_done;
        }
    } else {
        // Bag mode: same sample feeding as lidar_continuous_replay.cpp (common geometry profile).
        rosbag::Bag bag(bag_path, rosbag::bagmode::Read); rosbag::View view(bag);
        const auto first = view.getBeginTime().toNSec(); long idx = 0;
        for (const auto& event : view) {
            if (max_frames >= 0 && static_cast<long>(frames_done) >= max_frames) break;
            const auto arrival = event.getTime().toNSec(); const auto logical = arrival - first + 1000000000ULL;
            ros::Time::setNow(event.getTime());
            if (auto m = event.instantiate<sensor_msgs::Imu>()) {
                history.AddImu(m->header.stamp.toNSec() / 1000000, Convert(*m), nullptr, m->header.stamp.toNSec(), logical);
            } else if (auto m = event.instantiate<morai_msgs::EgoVehicleStatus>()) {
                CAN_DATA d{}; d.f32_CurrentVel_MPS = m->velocity.x;
                d.f32_CurrentVel_KPH = 3.6 * d.f32_CurrentVel_MPS;
                history.AddCan(m->header.stamp.toNSec() / 1000000, d, nullptr, m->header.stamp.toNSec(), logical);
            } else if (auto m = event.instantiate<morai_msgs::GPSMessage>()) {
                const auto source = m->header.stamp.toNSec();
                if (!m->status || (!m->latitude && !m->longitude)) history.AddGpsInvalidAt(source, logical);
                else {
                    GPS_DATA gps{}; int zone = 52; bool north = true; double east, nn;
                    GeographicLib::UTMUPS::Forward(m->latitude, m->longitude, zone, north, east, nn);
                    gps.f64_E = east - m->eastOffset; gps.f64_N = nn - m->northOffset;
                    gps.f64_U = m->altitude - frame_boundary::MapAltitudeOrigin();
                    history.AddGps(source / 1000000, gps, source, logical);
                }
            } else if (event.getTopic() == "/velodyne_points") {
                if (auto m = event.instantiate<sensor_msgs::PointCloud2>()) {
                    pcl::PointCloud<pcl::PointXYZI> cloud; pcl::fromROSMsg(*m, cloud);
                    std::vector<float> xyzi(cloud.size() * 4);
                    for (std::size_t i = 0; i < cloud.size(); ++i) {
                        xyzi[4 * i] = cloud.points[i].x; xyzi[4 * i + 1] = cloud.points[i].y;
                        xyzi[4 * i + 2] = cloud.points[i].z; xyzi[4 * i + 3] = cloud.points[i].intensity;
                    }
                    replay.Scan(scene, idx, "bag_" + std::to_string(idx), m->header.stamp.toNSec(), xyzi.data(), cloud.size(),
                                history, classifier, logical);
                    ++idx; ++frames_done;
                }
            }
        }
    }
    struct rusage usage{}; getrusage(RUSAGE_SELF, &usage);
    const auto& s = replay.stats();
    std::cerr << "[DATASET_REPLAY_TOTAL] scene=" << scene << " scans=" << s.scans << " ready=" << s.ready
              << " processed=" << s.processed << " local=" << s.local << " map=" << s.map << " none=" << s.none
              << " params=" << n_params << " overrides=" << overrides.size() << " fusion=" << (b_EnableSensorFusion ? 1 : 0)
              << " input_mode=" << input_mode << " trace_records=" << track_trace::Sink().Records()
              << " trace_dropped=" << track_trace::Sink().Dropped()
              << " wall_s=" << std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count()
              << " peak_rss_kb=" << usage.ru_maxrss << std::endl;
    ReportFusionTotals(); fusion_trace::Sink().Flush(); track_trace::Sink().Flush(); output.flush();
    if (points) { points->flush(); if (!*points) return 1; }
    return output ? 0 : 1;
}
