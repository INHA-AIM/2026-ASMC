#include <asmc_lidar/tracking/track.hpp>
// FrontCameraFusion: Main runCameraFusion behaviour on top of the camera
// receiver buffer. Uses a fake CameraSource for exact timing cases and a real
// camera::Buffer (fed through status()/observe() with controlled clocks) for
// the receiver mapping. All clocks are injected; nothing reads wall time.

#include <asmc_lidar/geometry/camera_calibration.hpp>
#include <asmc_lidar/fusion/front_camera_fusion.hpp>
#include <asmc_lidar/fusion/multi_camera_fusion.hpp>
#include <asmc_lidar/geometry/projection.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

using namespace asmc::lidar;
using namespace asmc::lidar::pipeline;
using namespace asmc::lidar::pipeline::fusion;

namespace {

constexpr Ns kMs = 1000000;
constexpr Ns kT0 = 1000000 * kMs;  // 1000 s of ROS time

// Same construction as camera::load_calibrations for cameras_v3.yaml front.
camera::Calibration FrontCalibration() {
  camera::Calibration c;
  c.frame_id = "camera_front";
  c.id = "sha256:test_front";
  c.width = 1280;
  c.height = 720;
  c.base_from_camera.translation() = Vec(1.90, 0.0, 1.20);
  c.base_from_camera.linear() = rpy(0., 2.0 * (std::acos(-1.) / 180.), 0.);
  const double focal = 1280 / (2. * std::tan(90. * std::acos(-1.) / 360.));
  c.intrinsic << focal, 0, 640., 0, focal, 360., 0, 0, 1;
  return c;
}

// setting.yaml Vehicle.LiDAR (base_link <- velodyne).
Pose Mount() {
  Pose mount = Pose::Identity();
  mount.translation() = Vec(0.80, 0.0, 1.45);
  return mount;
}

Context LeveledContext(Ns stamp, const Rot& leveling = Rot::Identity()) {
  Context c;
  c.time.stamp_ns = stamp;
  c.time.receipt_ns = stamp;
  c.time.processing_ns = stamp;
  c.time.clock = "ros";
  c.time.stamp_provenance = "bridge_receive_ns";
  c.frame = Frame::LeveledSensor;
  c.base_from_sensor = Mount();
  c.calibration_source = "test_mount";
  c.applied_leveling = leveling;
  c.leveling_source = "test_imu";
  return c;
}

pcl::PointXYZI MakePoint(double x, double y, double z) {
  pcl::PointXYZI p;
  p.x = static_cast<float>(x);
  p.y = static_cast<float>(y);
  p.z = static_cast<float>(z);
  p.intensity = 1.f;
  return p;
}

// A car-sized observed track in the leveled frame, `x` m ahead.
TrackedObject MakeCar(int id, float x) {
  TrackedObject o;
  o.tracking.s32_Id = id;
  o.tracking.s32_Age = 3;
  for (float xi : {x, x + 1.5f})
    for (float yi : {-0.9f, -0.45f, 0.f, 0.45f, 0.9f})
      for (float zi : {-1.2f, -0.8f, -0.4f, 0.f}) o.detection.pcl_ClusterPoints.push_back(MakePoint(xi, yi, zi));
  o.detection.f32_CoreX = x + 0.75f;
  o.detection.f32_CoreY = 0.1f;
  o.detection.f32_CoreZ = -0.6f;
  o.detection.f32_MinX = x;
  o.detection.f32_MaxX = x + 1.5f;
  o.detection.f32_MinY = -0.9f;
  o.detection.f32_MaxY = 0.9f;
  o.detection.f32_OBB_CenterX = x + 0.75f;
  o.detection.obb_length_m = 1.5f;
  o.detection.f32_OBB_YawRad = 0.2f;
  return o;
}

void ExpectSameGeometry(const TrackedObject& expected, const TrackedObject& actual) {
  EXPECT_EQ(expected.tracking.s32_Id, actual.tracking.s32_Id);
  EXPECT_EQ(expected.tracking.s32_MissCount, actual.tracking.s32_MissCount);
  EXPECT_EQ(expected.tracking.s32_Age, actual.tracking.s32_Age);
  EXPECT_EQ(expected.detection.f32_CoreX, actual.detection.f32_CoreX);
  EXPECT_EQ(expected.detection.f32_CoreY, actual.detection.f32_CoreY);
  EXPECT_EQ(expected.detection.f32_CoreZ, actual.detection.f32_CoreZ);
  EXPECT_EQ(expected.detection.f32_MinX, actual.detection.f32_MinX);
  EXPECT_EQ(expected.detection.f32_MaxX, actual.detection.f32_MaxX);
  EXPECT_EQ(expected.detection.f32_OBB_CenterX, actual.detection.f32_OBB_CenterX);
  EXPECT_EQ(expected.detection.f32_OBB_YawRad, actual.detection.f32_OBB_YawRad);
  ASSERT_EQ(expected.detection.pcl_ClusterPoints.size(), actual.detection.pcl_ClusterPoints.size());
  for (std::size_t i = 0; i < expected.detection.pcl_ClusterPoints.size(); ++i) {
    EXPECT_EQ(expected.detection.pcl_ClusterPoints[i].x, actual.detection.pcl_ClusterPoints[i].x);
    EXPECT_EQ(expected.detection.pcl_ClusterPoints[i].z, actual.detection.pcl_ClusterPoints[i].z);
  }
}

LidarImageBox ProjectedBox(const TrackedObject& object, const Context& context = LeveledContext(kT0)) {
  std::vector<LidarImageBox> boxes;
  EXPECT_TRUE(LidarToImage({object}, context, FrontCalibration(), FusionConfig{}, &boxes));
  EXPECT_EQ(1u, boxes.size());
  return boxes.empty() ? LidarImageBox{} : boxes.front();
}

camera::Box CameraBox(const LidarImageBox& b, const std::string& source, const std::string& group) {
  camera::Box box;
  box.xmin = b.min_x;
  box.ymin = b.min_y;
  box.xmax = b.max_x;
  box.ymax = b.max_y;
  box.class_name = group;
  box.source_class_name = source;
  box.confidence = 0.9f;
  box.confidence_valid = true;
  return box;
}

// Sensor-frame point that the front camera sees at pixel (u, v), depth m.
Vec SensorPointForPixel(double u, double v, double depth, const Pose& base_from_sensor = Mount()) {
  const auto c = FrontCalibration();
  const Vec ray = c.intrinsic.inverse() * Vec(u, v, 1.0) * depth;  // optical frame
  Rot optical;
  optical << 0, -1, 0, 0, 0, -1, 1, 0, 0;  // camera mount -> optical
  return base_from_sensor.inverse() * (c.base_from_camera * (optical.transpose() * ray));
}

using ClassState = std::tuple<std::string, std::string, std::string, int, int, std::uint64_t, std::uint64_t>;
ClassState StateOf(const TrackedObject& o) {
  return ClassState{o.camera.str_ObjectClass, o.camera.str_YoloClass, o.camera.last_seen_class, o.camera.continuity_counter,
                    o.camera.class_miss_counter, o.camera.camera_observation_ns, o.camera.camera_observation_epoch};
}

// ---------------------------------------------------------------------------
// Fake source: returns exactly the configured snapshot.

class FakeSource : public CameraSource {
 public:
  CameraSnapshot snapshot;
  bool current = true;
  std::vector<std::pair<Ns, Ns>> reads;  // (reference, window)

  CameraSnapshot Read(Ns reference, Ns window, Ns, Ns) override {
    reads.emplace_back(reference, window);
    return snapshot;
  }
  bool Current(const CameraFrame&, Ns, Ns) override { return current; }

  void Set(std::vector<CameraFrame> frames, bool active = true, const std::string& reason = "ok",
           std::uint64_t epoch = 1) {
    snapshot.active = active;
    snapshot.reason = reason;
    snapshot.epoch = frames.empty() ? 0 : epoch;
    snapshot.frames = std::move(frames);
  }
};

CameraFrame MakeFrame(Ns stamp, std::vector<camera::Box> boxes, std::uint64_t epoch = 1) {
  CameraFrame f;
  f.stamp_ns = stamp;
  f.epoch = epoch;
  f.identity = "s1|1|camera_front|" + std::to_string(stamp) + "|sha256:test_front";
  f.width = 1280;
  f.height = 720;
  f.boxes = std::move(boxes);
  return f;
}

// Tracker stand-in: the committed history frame carries the class into the
// next scan's live track (Tracking.cpp fusion_state::Copy), and the scan is
// committed before fusion runs (LidarProcess order).
struct TrackerSim {
  TrackedObject prototype = MakeCar(7, 10.f);
  std::vector<TrackedObject> history;
  std::vector<TrackedObject> live;

  FusionReport Scan(FrontCameraFusion& fusion, CameraSource& source, Ns decision, Ns stamp, Ns ros_now,
                    Ns steady_now) {
    live = {prototype};
    if (!history.empty()) fusion_state::Copy(history[0], live[0]);
    history = live;
    return fusion.Process(decision, stamp, LeveledContext(stamp), live, &history, source, ros_now, steady_now);
  }
  FusionReport Scan(FrontCameraFusion& fusion, CameraSource& source, Ns decision, Ns stamp) {
    return Scan(fusion, source, decision, stamp, decision, decision);
  }
};

camera::Box TruckBox() { return CameraBox(ProjectedBox(MakeCar(7, 10.f)), "truck", "vehicle"); }
camera::Box PersonBox() { return CameraBox(ProjectedBox(MakeCar(7, 10.f)), "person", "dynamic_obstacle"); }

// ---------------------------------------------------------------------------
// Real receiver buffer.

struct BufferRig {
  camera::Calibration front = FrontCalibration();
  std::shared_ptr<camera::Buffer> buffer;
  std::unique_ptr<BufferCameraSource> source;
  std::unique_ptr<FrontCameraFusion> fusion;
  TrackerSim tracker;
  std::string session = "s1";
  std::uint64_t generation = 1;
  std::uint32_t sequence = 0;

  explicit BufferRig(const FusionConfig& config = {}) {
    camera::Calibrations calibrations;
    calibrations[front.frame_id] = front;
    buffer = std::make_shared<camera::Buffer>(calibrations);
    source = std::make_unique<BufferCameraSource>(buffer);
    fusion = std::make_unique<FrontCameraFusion>(config, front);
  }
  std::string Status(Ns now, std::uint8_t mode = 2) {
    camera::Status s;
    s.stamp_ns = now;
    s.cutoff_ns = 0;
    s.session = session;
    s.model = "yolo";
    s.generation = generation;
    s.mode = mode;
    return buffer->status(s, now, now);
  }
  std::string Observe(Ns stamp, Ns now, std::vector<camera::Box> boxes, bool valid = true) {
    camera::Observation o;
    o.header.sequence = ++sequence;
    o.header.stamp_ns = stamp;
    o.header.frame_id = "camera_front";
    o.width = 1280;
    o.height = 720;
    o.valid = valid;
    o.boxes = std::move(boxes);
    o.session = session;
    o.generation = generation;
    o.calibration_id = front.id;
    o.model_id = "yolo26";
    o.config_id = "config";
    return buffer->observe(o, now, now);
  }
  FusionReport Scan(Ns stamp, Ns now) { return tracker.Scan(*fusion, *source, now, stamp, now, now); }
  // One 100 ms step: heartbeat at t, frame stamped t+5 ms received at t+30 ms,
  // scan stamped t processed at t+40 ms.
  FusionReport Step(int k, std::vector<camera::Box> boxes, bool heartbeat = true, bool frame = true) {
    const Ns t = kT0 + k * 100 * kMs;
    if (heartbeat) {
      EXPECT_EQ("status_accepted", Status(t));
    }
    if (frame) {
      EXPECT_EQ("accepted", Observe(t + 5 * kMs, t + 30 * kMs, std::move(boxes)));
    }
    return Scan(t, t + 40 * kMs);
  }
  const TrackedObject& Live() const { return tracker.live.at(0); }
};

// Five matched truck frames: returns the next step index.
int ConfirmVehicle(BufferRig& rig, int k = 0) {
  for (int i = 0; i < 5; ++i, ++k) {
    const auto report = rig.Step(k, {TruckBox()});
    EXPECT_TRUE(report.camera_active);
    EXPECT_EQ(1u, report.matched_this_call);
  }
  EXPECT_EQ("vehicle", rig.Live().camera.str_ObjectClass);
  return k;
}

void ExpectInvalidated(const BufferRig& rig, const FusionReport& report, const std::string& reason) {
  EXPECT_FALSE(report.camera_active);
  EXPECT_EQ(reason, report.camera_reason);
  ASSERT_EQ(1u, rig.tracker.live.size());  // never removed
  EXPECT_EQ("unknown", rig.Live().camera.str_ObjectClass);
  EXPECT_EQ("unknown", rig.Live().camera.str_YoloClass);
  EXPECT_EQ(0, rig.Live().camera.continuity_counter);
  ASSERT_EQ(1u, rig.tracker.history.size());
  EXPECT_EQ("unknown", rig.tracker.history[0].camera.str_ObjectClass);  // persisted
  ExpectSameGeometry(MakeCar(7, 10.f), rig.Live());
  ExpectSameGeometry(MakeCar(7, 10.f), rig.tracker.history[0]);
}

}  // namespace

// ---------------------------------------------------------------------------
// Projection (LidarToImage, common-geometry branch)

TEST(FusionProjection, UsesStoredLevelingInverse) {
  const Vec raw = SensorPointForPixel(400.3, 300.4, 12.0);
  const Rot leveling = leveling_rotation(0.03, 0.05, Mount().linear(), true, true);
  const Vec leveled = leveling * raw;
  const Ns stamp = kT0;

  TrackedObject leveled_object, raw_object;
  leveled_object.detection.pcl_ClusterPoints.push_back(MakePoint(leveled.x(), leveled.y(), leveled.z()));
  raw_object.detection.pcl_ClusterPoints.push_back(MakePoint(raw.x(), raw.y(), raw.z()));
  Context raw_context = LeveledContext(stamp);
  raw_context.frame = Frame::Sensor;  // raw sensor point through base_from_sensor

  std::vector<LidarImageBox> from_leveled, from_raw;
  ASSERT_TRUE(LidarToImage({leveled_object}, LeveledContext(stamp, leveling), FrontCalibration(), {}, &from_leveled));
  ASSERT_TRUE(LidarToImage({raw_object}, raw_context, FrontCalibration(), {}, &from_raw));
  ASSERT_EQ(1u, from_leveled.size());
  ASSERT_EQ(1u, from_raw.size());
  EXPECT_EQ(400.f, from_leveled[0].min_x);
  EXPECT_EQ(300.f, from_leveled[0].min_y);
  EXPECT_EQ(401.f, from_leveled[0].max_x);  // boundingRect br() is exclusive
  EXPECT_EQ(301.f, from_leveled[0].max_y);
  EXPECT_EQ(from_raw[0].min_x, from_leveled[0].min_x);
  EXPECT_EQ(from_raw[0].min_y, from_leveled[0].min_y);
  // Same pixel as the receiver's per-point projection of the raw point.
  const auto pixel = camera::project(FrontCalibration(), raw_context, stamp, raw);
  ASSERT_TRUE(pixel.valid) << pixel.reason;
  EXPECT_NEAR(400.3, pixel.u, 1e-6);
  EXPECT_NEAR(300.4, pixel.v, 1e-6);
  // Negative control: the leveled point without the inverse leveling (Main
  // legacy approximation) lands elsewhere.
  std::vector<LidarImageBox> no_inverse;
  TrackedObject leveled_as_raw = leveled_object;
  ASSERT_TRUE(LidarToImage({leveled_as_raw}, raw_context, FrontCalibration(), {}, &no_inverse));
  ASSERT_EQ(1u, no_inverse.size());
  EXPECT_GT(std::abs(no_inverse[0].min_y - from_leveled[0].min_y), 10.f);
}

TEST(FusionProjection, PreservesTruncationBoundingRectAndSkips) {
  // u in (-1, 0) truncates to 0 and is kept (Main quirk); project() rejects it.
  const Vec edge = SensorPointForPixel(-0.5, 200.25, 8.0);
  const Vec inside = SensorPointForPixel(10.6, 220.7, 8.0);
  TrackedObject object;
  object.detection.pcl_ClusterPoints.push_back(MakePoint(edge.x(), edge.y(), edge.z()));
  object.detection.pcl_ClusterPoints.push_back(MakePoint(inside.x(), inside.y(), inside.z()));
  object.detection.pcl_ClusterPoints.push_back(MakePoint(-5.0, 0.0, 0.0));  // behind the camera
  TrackedObject ghost = object;
  ghost.tracking.s32_MissCount = 1;  // prediction-only: never projected
  TrackedObject noise = object;
  noise.camera.str_ObjectClass = "noise";
  std::vector<LidarImageBox> boxes;
  ASSERT_TRUE(LidarToImage({ghost, object, noise}, LeveledContext(kT0), FrontCalibration(), {}, &boxes));
  ASSERT_EQ(1u, boxes.size());
  EXPECT_EQ(1, boxes[0].c_idx);
  EXPECT_EQ(0.f, boxes[0].min_x);
  EXPECT_EQ(200.f, boxes[0].min_y);
  EXPECT_EQ(11.f, boxes[0].max_x);
  EXPECT_EQ(221.f, boxes[0].max_y);
  Context sensor_context = LeveledContext(kT0);
  EXPECT_FALSE(camera::project(FrontCalibration(), sensor_context, kT0, edge).valid);

  // Optional hood y-band (default off) rejects a box overlapping [637, 720).
  const Vec low = SensorPointForPixel(500.5, 650.5, 4.0);
  TrackedObject hood;
  hood.detection.pcl_ClusterPoints.push_back(MakePoint(low.x(), low.y(), low.z()));
  FusionConfig band;
  band.exclude_hood_y_band = true;
  ASSERT_TRUE(LidarToImage({hood}, LeveledContext(kT0), FrontCalibration(), {}, &boxes));
  EXPECT_EQ(1u, boxes.size());
  ASSERT_TRUE(LidarToImage({hood}, LeveledContext(kT0), FrontCalibration(), band, &boxes));
  EXPECT_TRUE(boxes.empty());

  // An unprojectable context gives no boxes (every observed object then misses).
  Context bad = LeveledContext(kT0);
  bad.leveling_source.clear();
  EXPECT_FALSE(LidarToImage({object}, bad, FrontCalibration(), {}, &boxes));
  EXPECT_TRUE(boxes.empty());
}

// ---------------------------------------------------------------------------
// IoU association

TEST(FusionIou, GreedyOneToOneStrictThresholdAndMisses) {
  FusionConfig config;
  std::vector<TrackedObject> objects(4);
  for (int i = 0; i < 4; ++i) objects[i].tracking.s32_Id = i;
  objects[3].tracking.s32_MissCount = 2;
  objects[3].camera.str_ObjectClass = "vehicle";
  objects[2].camera.str_ObjectClass = "vehicle";
  objects[2].camera.class_miss_counter = 3;
  objects[2].camera.camera_observation_epoch = 2;  // same epoch: counters are not reset
  objects[2].camera.camera_observation_ns = 500;
  std::vector<LidarImageBox> lidar(2);
  lidar[0] = {100.f, 100.f, 200.f, 200.f, 0};
  lidar[1] = {600.f, 300.f, 640.f, 380.f, 1};
  fusion_adapter::YoloBox a{100.f, 100.f, 200.f, 200.f, "vehicle", "truck"};
  fusion_adapter::YoloBox b{102.f, 101.f, 199.f, 201.f, "vehicle", "bus"};  // duplicate of a
  IOUFusion(objects, {a, b}, lidar, 1000, 2, config, nullptr);
  EXPECT_EQ(1, objects[0].camera.continuity_counter);  // one confirmation from two duplicates
  EXPECT_EQ("vehicle", objects[0].camera.last_seen_class);
  EXPECT_EQ(1, objects[1].camera.class_miss_counter);  // projected but unmatched
  EXPECT_EQ("unknown", objects[2].camera.str_ObjectClass);  // out of image: 4th miss
  EXPECT_EQ(4, objects[2].camera.class_miss_counter);
  EXPECT_EQ("vehicle", objects[3].camera.str_ObjectClass);  // prediction-only keeps class
  EXPECT_EQ(0u, objects[3].camera.camera_observation_ns);

  // Strict iou > threshold.
  const float inflated_area = 130.f * 150.f;
  const double iou = (100.f * 100.f) / (inflated_area);  // a inside the inflated box
  std::vector<TrackedObject> once(1);
  FusionConfig at_threshold;
  at_threshold.iou_threshold = static_cast<double>(static_cast<float>(100.f * 100.f) / inflated_area);
  IOUFusion(once, {a}, {lidar[0]}, 1000, 2, at_threshold, nullptr);
  EXPECT_EQ(1, once[0].camera.class_miss_counter);
  EXPECT_EQ(0, once[0].camera.continuity_counter);
  at_threshold.iou_threshold = iou - 1e-6;
  IOUFusion(once, {a}, {lidar[0]}, 2000, 2, at_threshold, nullptr);
  EXPECT_EQ(1, once[0].camera.continuity_counter);
  // Same image stamp again: neither confirmation nor miss.
  IOUFusion(once, {a}, {lidar[0]}, 2000, 2, at_threshold, nullptr);
  EXPECT_EQ(1, once[0].camera.continuity_counter);
  EXPECT_EQ(0, once[0].camera.class_miss_counter);
}

// ---------------------------------------------------------------------------
// Time matching through Process (fake source)

TEST(FrontCameraFusion, FutureFrameSettlesImmediately) {
  FakeSource source;
  FrontCameraFusion fusion(FusionConfig{}, FrontCalibration());
  TrackerSim tracker;
  const Ns scan = kT0;
  source.Set({MakeFrame(scan + 20 * kMs, {TruckBox()})});
  const auto report = tracker.Scan(fusion, source, kT0 + 30 * kMs, scan);
  EXPECT_TRUE(report.camera_active);
  EXPECT_EQ(1u, report.matched_this_call);
  EXPECT_EQ(0u, report.pending_scans);
  ASSERT_EQ(1u, report.match_offsets_ns.size());
  EXPECT_EQ(20 * kMs, report.match_offsets_ns[0]);  // recorded, not compensated
  EXPECT_EQ(1, tracker.live[0].camera.continuity_counter);
  EXPECT_EQ("vehicle", tracker.live[0].camera.last_seen_class);
  EXPECT_EQ(1, tracker.history[0].camera.continuity_counter);  // persisted
  ASSERT_EQ(1u, source.reads.size());
  EXPECT_EQ(scan, source.reads[0].first);
  EXPECT_EQ(1000 * kMs, source.reads[0].second);  // read window, not the 50 ms tolerance
}

TEST(FrontCameraFusion, TimeoutDropAfter200msOfInjectedClock) {
  FakeSource source;
  FrontCameraFusion fusion(FusionConfig{}, FrontCalibration());
  TrackerSim tracker;
  const Ns scan = kT0, decision = kT0 + 5 * kMs;
  source.Set({MakeFrame(scan - 70 * kMs, {TruckBox()})});
  auto report = tracker.Scan(fusion, source, decision, scan);
  EXPECT_EQ(1u, report.pending_scans);
  report = tracker.Scan(fusion, source, decision + 199 * kMs, scan);  // same stamp: not re-queued
  EXPECT_EQ(1u, report.pending_scans);
  EXPECT_EQ(0u, report.dropped_this_call);
  report = tracker.Scan(fusion, source, decision + 200 * kMs, scan);
  EXPECT_EQ(1u, report.dropped_this_call);
  EXPECT_EQ(0u, report.pending_scans);
  EXPECT_EQ(1u, fusion.counters().queued);
  EXPECT_EQ(2u, fusion.counters().waits);
  EXPECT_EQ(1u, fusion.counters().dropped);
  EXPECT_EQ(0, tracker.live[0].camera.continuity_counter);
  EXPECT_EQ(0, tracker.live[0].camera.class_miss_counter);  // a drop is not a miss
}

TEST(FrontCameraFusion, PlusMinus50msInclusiveAndTieGoesToEarlierStamp) {
  struct Case {
    std::vector<Ns> offsets_ms;
    Ns wait_ms;
    bool match;
  };
  for (const auto& c : std::vector<Case>{{{-50}, 200, true}, {{-51}, 200, false}, {{50}, 0, true}, {{51}, 0, false}}) {
    FakeSource source;
    FrontCameraFusion fusion(FusionConfig{}, FrontCalibration());
    TrackerSim tracker;
    // Warm-up match so the startup rules no longer apply.
    const CameraFrame warm = MakeFrame(kT0 - 500 * kMs, {});
    source.Set({warm});
    ASSERT_EQ(1u, tracker.Scan(fusion, source, kT0 - 500 * kMs, kT0 - 500 * kMs).matched_this_call);
    std::vector<CameraFrame> frames{warm};
    for (Ns offset : c.offsets_ms) frames.push_back(MakeFrame(kT0 + offset * kMs, {TruckBox()}));
    source.Set(frames);
    tracker.Scan(fusion, source, kT0, kT0);
    const auto report = tracker.Scan(fusion, source, kT0 + c.wait_ms * kMs, kT0);
    EXPECT_EQ(c.match ? 2u : 1u, fusion.counters().matched) << c.offsets_ms[0];
    EXPECT_EQ(c.match ? 0u : 1u, fusion.counters().dropped) << c.offsets_ms[0];
    EXPECT_EQ(0u, fusion.counters().startup_discarded);
    EXPECT_EQ(0u, report.pending_scans);
  }
  // Startup: a first frame 51 ms after a pending scan discards it as a startup
  // scan (Main lidar_startup_discard), not as a sync drop.
  {
    FakeSource source;
    FrontCameraFusion fusion(FusionConfig{}, FrontCalibration());
    TrackerSim tracker;
    source.Set({MakeFrame(kT0 + 51 * kMs, {TruckBox()})});
    const auto report = tracker.Scan(fusion, source, kT0, kT0);
    EXPECT_EQ(1u, fusion.counters().startup_discarded);
    EXPECT_EQ(0u, fusion.counters().dropped);
    EXPECT_EQ(0u, report.pending_scans);
  }
  // Equal offsets: the receiver lists the newer frame first; Main picks the earlier.
  FakeSource source;
  FrontCameraFusion fusion(FusionConfig{}, FrontCalibration());
  TrackerSim tracker;
  source.Set({MakeFrame(kT0 + 50 * kMs, {TruckBox()}), MakeFrame(kT0 - 50 * kMs, {PersonBox()})});
  const auto report = tracker.Scan(fusion, source, kT0, kT0);
  ASSERT_EQ(1u, report.matched_this_call);
  EXPECT_EQ("dynamic_obstacle", tracker.live[0].camera.last_seen_class);
  EXPECT_EQ(static_cast<std::uint64_t>(kT0 - 50 * kMs), tracker.live[0].camera.camera_observation_ns);
}

TEST(FrontCameraFusion, DuplicateFramesAreNotDoubleCounted) {
  // unique: a consumed frame can neither match nor settle a later scan.
  {
    FakeSource source;
    FrontCameraFusion fusion(FusionConfig{}, FrontCalibration());
    TrackerSim tracker;
    source.Set({MakeFrame(kT0 + 30 * kMs, {TruckBox()})});
    EXPECT_EQ(1u, tracker.Scan(fusion, source, kT0 + 40 * kMs, kT0).matched_this_call);
    auto report = tracker.Scan(fusion, source, kT0 + 60 * kMs, kT0 + 20 * kMs);
    EXPECT_EQ(0u, report.matched_this_call);
    EXPECT_EQ(1u, report.pending_scans);  // waits: the only future frame is used
    report = tracker.Scan(fusion, source, kT0 + 260 * kMs, kT0 + 20 * kMs);
    EXPECT_EQ(1u, report.dropped_this_call);
    EXPECT_EQ(1, tracker.live[0].camera.continuity_counter);
    EXPECT_EQ(1u, fusion.counters().observations);
    EXPECT_EQ(1u, fusion.consumed_size());
  }
  // bounded_reuse: the frame matches again but the track counts it once.
  {
    FusionConfig config;
    config.yolo_policy = "bounded_reuse";
    FakeSource source;
    FrontCameraFusion fusion(config, FrontCalibration());
    TrackerSim tracker;
    source.Set({MakeFrame(kT0 + 30 * kMs, {TruckBox()})});
    EXPECT_EQ(1u, tracker.Scan(fusion, source, kT0 + 40 * kMs, kT0).matched_this_call);
    EXPECT_EQ(1u, tracker.Scan(fusion, source, kT0 + 60 * kMs, kT0 + 20 * kMs).matched_this_call);
    EXPECT_EQ(1u, fusion.counters().reused);
    EXPECT_EQ(1u, fusion.counters().fresh);
    EXPECT_EQ(1, tracker.live[0].camera.continuity_counter);
    EXPECT_EQ(0, tracker.live[0].camera.class_miss_counter);
    EXPECT_EQ(1u, fusion.counters().observations);
  }
}

TEST(FrontCameraFusion, BoundedReuseEmptyBarrier) {
  FusionConfig config;
  config.yolo_policy = "bounded_reuse";
  FakeSource source;
  FrontCameraFusion fusion(config, FrontCalibration());
  TrackerSim tracker;
  // A newer normal empty frame hides the older detection from a waiting scan.
  source.Set({MakeFrame(kT0 - 10 * kMs, {TruckBox()}), MakeFrame(kT0 + 20 * kMs, {})});
  const auto report = tracker.Scan(fusion, source, kT0, kT0);
  ASSERT_EQ(1u, report.matched_this_call);
  EXPECT_EQ(20 * kMs, report.match_offsets_ns[0]);
  EXPECT_EQ(1, tracker.live[0].camera.class_miss_counter);
}

TEST(FrontCameraFusion, PendingBoundsStartupAndHeadOfLine) {
  // Before the first frame only the newest 2 scans are kept.
  {
    FakeSource source;
    FrontCameraFusion fusion(FusionConfig{}, FrontCalibration());
    TrackerSim tracker;
    source.Set({}, true, "no_time_match");
    FusionReport report;
    for (int k = 1; k <= 5; ++k) report = tracker.Scan(fusion, source, kT0 + k * 100 * kMs, kT0 + k * 100 * kMs);
    EXPECT_EQ(2u, report.pending_scans);
    EXPECT_EQ(3u, fusion.counters().startup_discarded);
    // First frames at 490/510 ms: scans older than 490-50 ms are startup discards,
    // the 500 ms scan takes the earlier of two equidistant frames.
    source.Set({MakeFrame(kT0 + 510 * kMs, {TruckBox()}), MakeFrame(kT0 + 490 * kMs, {PersonBox()})});
    report = tracker.Scan(fusion, source, kT0 + 600 * kMs, kT0 + 600 * kMs);
    EXPECT_EQ(4u, fusion.counters().startup_discarded);
    EXPECT_EQ(1u, report.matched_this_call);
    EXPECT_EQ(10 * kMs, report.match_offsets_ns[0]);
    EXPECT_EQ("dynamic_obstacle", tracker.live[0].camera.last_seen_class);
    EXPECT_EQ(1u, report.pending_scans);  // 600 ms scan waits for a future frame
  }
  // Waiting scans are bounded to 32 (head-of-line blocking, clock not advancing).
  {
    FakeSource source;
    FrontCameraFusion fusion(FusionConfig{}, FrontCalibration());
    TrackerSim tracker;
    source.Set({MakeFrame(kT0 - 10 * kMs, {TruckBox()})});
    FusionReport report;
    for (int k = 0; k < 40; ++k) report = tracker.Scan(fusion, source, kT0, kT0 + k * kMs);
    EXPECT_EQ(32u, report.pending_scans);
    EXPECT_EQ(8u, fusion.counters().bound_discarded);
    EXPECT_EQ(0u, fusion.counters().matched);
  }
}

TEST(FrontCameraFusion, InactiveSourceInvalidatesWithoutTouchingGeometry) {
  FakeSource source;
  FrontCameraFusion fusion(FusionConfig{}, FrontCalibration());
  TrackerSim tracker;
  for (int k = 0; k < 5; ++k) {
    source.Set({MakeFrame(kT0 + k * 100 * kMs + 5 * kMs, {TruckBox()})});
    tracker.Scan(fusion, source, kT0 + k * 100 * kMs + 10 * kMs, kT0 + k * 100 * kMs);
  }
  ASSERT_EQ("vehicle", tracker.live[0].camera.str_ObjectClass);
  ASSERT_EQ("truck", tracker.live[0].camera.str_YoloClass);
  const auto epoch = fusion.consumer_epoch();
  source.Set({}, false, "standby");
  const auto report = tracker.Scan(fusion, source, kT0 + 600 * kMs, kT0 + 500 * kMs);
  EXPECT_FALSE(report.camera_active);
  EXPECT_EQ("standby", report.camera_reason);
  EXPECT_EQ(epoch + 1, fusion.consumer_epoch());
  EXPECT_EQ(0u, report.pending_scans);  // inactive scans are not queued
  ASSERT_EQ(1u, tracker.live.size());
  EXPECT_EQ("unknown", tracker.live[0].camera.str_ObjectClass);
  EXPECT_EQ("unknown", tracker.history[0].camera.str_ObjectClass);
  EXPECT_EQ(fusion.consumer_epoch(), tracker.live[0].camera.camera_observation_epoch);
  ExpectSameGeometry(MakeCar(7, 10.f), tracker.live[0]);
  ExpectSameGeometry(MakeCar(7, 10.f), tracker.history[0]);
  EXPECT_EQ(1u, fusion.counters().inactive_calls);
}

TEST(FrontCameraFusion, NotCurrentCandidateKeepsScanPending) {
  FakeSource source;
  FrontCameraFusion fusion(FusionConfig{}, FrontCalibration());
  TrackerSim tracker;
  source.Set({MakeFrame(kT0 + 10 * kMs, {TruckBox()})});
  source.current = false;
  auto report = tracker.Scan(fusion, source, kT0 + 20 * kMs, kT0);
  EXPECT_EQ(0u, report.matched_this_call);
  EXPECT_EQ(1u, report.pending_scans);
  EXPECT_EQ(1u, fusion.counters().not_current);
  source.current = true;
  report = tracker.Scan(fusion, source, kT0 + 30 * kMs, kT0);
  EXPECT_EQ(1u, report.matched_this_call);
}

TEST(FrontCameraFusion, EgoHoodFilterOnlyUnderLegacyProfile) {
  // A wide box at the image bottom (ego hood shape) over a projected object.
  TrackedObject hood_object;
  hood_object.tracking.s32_Id = 3;
  for (const auto& px : std::vector<std::pair<double, double>>{{300.5, 620.5}, {1100.5, 718.5}}) {
    const Vec p = SensorPointForPixel(px.first, px.second, 3.0);
    hood_object.detection.pcl_ClusterPoints.push_back(MakePoint(p.x(), p.y(), p.z()));
  }
  const auto box = ProjectedBox(hood_object);
  for (bool mission : {true, false}) {
    FusionConfig config;
    config.mission_profile = mission;
    FakeSource source;
    FrontCameraFusion fusion(config, FrontCalibration());
    std::vector<TrackedObject> live{hood_object}, history{hood_object};
    source.Set({MakeFrame(kT0 + 5 * kMs, {CameraBox(box, "car", "vehicle")})});
    fusion.Process(kT0 + 10 * kMs, kT0, LeveledContext(kT0), live, &history, source, kT0, kT0);
    if (mission) {
      // Normalize("car") is "vehicle", never "car": the filter is dead (quirk).
      EXPECT_EQ(1, live[0].camera.continuity_counter);
      EXPECT_EQ("vehicle", live[0].camera.last_seen_class);
    } else {
      EXPECT_EQ(0, live[0].camera.continuity_counter);
      EXPECT_EQ(1, live[0].camera.class_miss_counter);
    }
  }
}

TEST(FrontCameraFusion, ConfigValidationAndClamps) {
  FusionConfig bad;
  bad.yolo_policy = "latest";
  EXPECT_THROW(FrontCameraFusion fusion(bad, FrontCalibration()), std::runtime_error);
  bad = FusionConfig{};
  bad.confidence_threshold = 1.5;
  EXPECT_THROW(FrontCameraFusion fusion(bad, FrontCalibration()), std::runtime_error);
  bad = FusionConfig{};
  bad.camera_read_window_s = 0.01;
  EXPECT_THROW(FrontCameraFusion fusion(bad, FrontCalibration()), std::runtime_error);
  camera::Calibration no_size = FrontCalibration();
  no_size.width = 0;
  EXPECT_THROW(FrontCameraFusion fusion(FusionConfig{}, no_size), std::runtime_error);
  FusionConfig clamp;
  clamp.max_time_offset_s = 0.;
  clamp.sync_wait_timeout_s = 0.;
  clamp.initial_confirmations = 0;
  clamp.class_miss_grace = -2;
  const FrontCameraFusion fusion(clamp, FrontCalibration());
  EXPECT_EQ(0.001, fusion.config().max_time_offset_s);
  EXPECT_EQ(0.001, fusion.config().sync_wait_timeout_s);
  EXPECT_EQ(1, fusion.config().initial_confirmations);
  EXPECT_EQ(0, fusion.config().class_miss_grace);
}

// Main test_fusion_replay DeliveryLatency fixture end to end: 50 scans every
// 80 ms, 82 frames every 50 ms delivered with 55 or 12 ms latency, decisions
// only at scan calls. The same pairs and class sequence result.
TEST(FrontCameraFusion, EndToEndDeliveryLatencyKeepsAssociation) {
  auto run = [](int delay_ms) {
    FakeSource source;
    FrontCameraFusion fusion(FusionConfig{}, FrontCalibration());
    TrackerSim tracker;
    std::vector<std::int64_t> offsets;
    Ns last_scan = 0;
    for (int wall = 1000, k = 0; wall <= 6000; wall += 80, ++k) {
      std::vector<CameraFrame> delivered;
      for (int i = 0; i < 82; ++i) {
        const int image = 1000 + i * 50;
        if (image + delay_ms > wall) continue;
        std::vector<camera::Box> boxes;
        if (image < 3500 || image >= 3950) boxes.push_back(image < 2500 ? TruckBox() : PersonBox());
        delivered.push_back(MakeFrame(kT0 + image * kMs, boxes));
      }
      source.Set(delivered);
      if (k < 50) last_scan = kT0 + wall * kMs;
      const auto report = tracker.Scan(fusion, source, kT0 + wall * kMs, last_scan);
      offsets.insert(offsets.end(), report.match_offsets_ns.begin(), report.match_offsets_ns.end());
    }
    return std::make_tuple(offsets, fusion.counters().matched, fusion.counters().dropped,
                           StateOf(tracker.live[0]));
  };
  const auto slow = run(55), fast = run(12);
  EXPECT_EQ(50u, std::get<1>(slow));
  EXPECT_EQ(0u, std::get<2>(slow));
  EXPECT_EQ(std::get<0>(slow), std::get<0>(fast));
  EXPECT_EQ(std::get<1>(slow), std::get<1>(fast));
  EXPECT_EQ(std::get<3>(slow), std::get<3>(fast));
  EXPECT_EQ("dynamic_obstacle", std::get<0>(std::get<3>(slow)));
  EXPECT_EQ("person", std::get<1>(std::get<3>(slow)));
}

// ---------------------------------------------------------------------------
// Real camera::Buffer

TEST(FrontCameraFusionBuffer, ValidFrontObservationConfirmsAfterFiveMatches) {
  BufferRig rig;
  for (int k = 0; k < 5; ++k) {
    const auto report = rig.Step(k, {TruckBox()});
    EXPECT_TRUE(report.camera_active);
    EXPECT_EQ("ok", report.camera_reason);
    ASSERT_EQ(1u, report.matched_this_call);
    EXPECT_EQ(5 * kMs, report.match_offsets_ns.at(0));
    if (k < 4) {
      EXPECT_EQ("unknown", rig.Live().camera.str_ObjectClass);
      EXPECT_EQ(k + 1, rig.Live().camera.continuity_counter);
    }
  }
  EXPECT_EQ("vehicle", rig.Live().camera.str_ObjectClass);  // mission group
  EXPECT_EQ("truck", rig.Live().camera.str_YoloClass);      // raw label kept
  EXPECT_EQ("vehicle", rig.tracker.history[0].camera.str_ObjectClass);
  EXPECT_EQ(1u, rig.fusion->counters().confirmations_applied);
  ExpectSameGeometry(MakeCar(7, 10.f), rig.Live());
  // Change needs 3 opposite observations once confirmed.
  for (int k = 5; k < 7; ++k) rig.Step(k, {PersonBox()});
  EXPECT_EQ("vehicle", rig.Live().camera.str_ObjectClass);
  rig.Step(7, {PersonBox()});
  EXPECT_EQ("dynamic_obstacle", rig.Live().camera.str_ObjectClass);
  EXPECT_EQ("person", rig.Live().camera.str_YoloClass);
}

TEST(FrontCameraFusionBuffer, NormalEmptyFramesMissAfterGrace) {
  BufferRig rig;
  int k = ConfirmVehicle(rig);
  for (int miss = 1; miss <= 4; ++miss, ++k) {
    const auto report = rig.Step(k, {});
    EXPECT_TRUE(report.camera_active);
    EXPECT_EQ("empty", report.camera_reason);
    EXPECT_EQ(1u, report.matched_this_call);
    EXPECT_EQ(miss, rig.Live().camera.class_miss_counter);
    EXPECT_EQ(miss <= 3 ? "vehicle" : "unknown", rig.Live().camera.str_ObjectClass);
    ASSERT_EQ(1u, rig.tracker.live.size());
    ExpectSameGeometry(MakeCar(7, 10.f), rig.Live());
  }
}

TEST(FrontCameraFusionBuffer, InvalidObservationResetsClasses) {
  BufferRig rig;
  const int k = ConfirmVehicle(rig);
  const Ns t = kT0 + k * 100 * kMs;
  rig.Status(t);
  ASSERT_EQ("invalid", rig.Observe(t + 5 * kMs, t + 30 * kMs, {}, false));
  ExpectInvalidated(rig, rig.Scan(t, t + 40 * kMs), "invalid");
}

TEST(FrontCameraFusionBuffer, StandbyResetsClasses) {
  BufferRig rig;
  const int k = ConfirmVehicle(rig);
  const Ns t = kT0 + k * 100 * kMs;
  rig.generation = 2;  // a mode change needs a new generation
  ASSERT_EQ("status_accepted", rig.Status(t, 0));
  ExpectInvalidated(rig, rig.Scan(t, t + 40 * kMs), "standby");
  ++rig.generation;
  ASSERT_EQ("status_accepted", rig.Status(t + 50 * kMs, 1));
  ExpectInvalidated(rig, rig.Scan(t + 60 * kMs, t + 70 * kMs), "prepare");
}

TEST(FrontCameraFusionBuffer, StatusStaleResetsClasses) {
  BufferRig rig;
  const int k = ConfirmVehicle(rig);
  const Ns last_status = kT0 + (k - 1) * 100 * kMs;
  ExpectInvalidated(rig, rig.Scan(last_status + 100 * kMs, last_status + 400 * kMs), "status_stale");
}

TEST(FrontCameraFusionBuffer, StaleFramesResetClassesAfterRetention) {
  BufferRig rig;
  int k = ConfirmVehicle(rig);
  // Heartbeats continue, frames stop: the retained (consumed) frames keep the
  // view active until the receiver expires them (350 ms after receipt).
  FusionReport report;
  for (int i = 0; i < 3; ++i, ++k) {
    report = rig.Step(k, {}, true, false);
    EXPECT_TRUE(report.camera_active);
    EXPECT_EQ("vehicle", rig.Live().camera.str_ObjectClass);  // no new image: no miss
  }
  report = rig.Step(k, {}, true, false);
  ExpectInvalidated(rig, report, "stale");
}

TEST(FrontCameraFusionBuffer, StatusMissingAndClosedAreInactive) {
  BufferRig rig;
  const int k = ConfirmVehicle(rig);
  const Ns t = kT0 + k * 100 * kMs;
  // Same fusion, a buffer that never received a status.
  camera::Calibrations calibrations;
  calibrations[rig.front.frame_id] = rig.front;
  BufferCameraSource silent(std::make_shared<camera::Buffer>(calibrations));
  ExpectInvalidated(rig, rig.tracker.Scan(*rig.fusion, silent, t, t), "status_missing");
  rig.buffer->close();
  ExpectInvalidated(rig, rig.Scan(t + 100 * kMs, t + 110 * kMs), "closed");
}

TEST(FrontCameraFusionBuffer, SessionOrGenerationChangeStartsNewEpoch) {
  for (bool new_session : {false, true}) {
    BufferRig rig;
    int k = ConfirmVehicle(rig);
    const auto epochs = rig.fusion->counters().epoch_changes;
    const auto consumer = rig.fusion->consumer_epoch();
    // Restart between two scans: the first scan already sees the new records.
    if (new_session) rig.session = "s2";
    else ++rig.generation;
    const auto report = rig.Step(k++, {TruckBox()});
    EXPECT_TRUE(report.camera_active);
    EXPECT_EQ(epochs + 1, rig.fusion->counters().epoch_changes);
    EXPECT_EQ(consumer + 1, rig.fusion->consumer_epoch());
    EXPECT_EQ("unknown", rig.Live().camera.str_ObjectClass);  // invalidated, then 1st observation
    EXPECT_EQ(1, rig.Live().camera.continuity_counter);
    EXPECT_EQ(rig.fusion->consumer_epoch(), rig.Live().camera.camera_observation_epoch);
    for (int i = 0; i < 4; ++i) rig.Step(k++, {TruckBox()});
    EXPECT_EQ("vehicle", rig.Live().camera.str_ObjectClass);
  }
}

TEST(FrontCameraFusionBuffer, DuplicateObservationsAreRejectedAndConsumedSetIsBounded) {
  BufferRig rig;
  rig.Step(0, {TruckBox()});
  const Ns t = kT0;
  EXPECT_EQ("duplicate_or_old", rig.Observe(t + 5 * kMs, t + 35 * kMs, {TruckBox()}));
  for (int k = 1; k < 40; ++k) rig.Step(k, k % 2 ? std::vector<camera::Box>{} : std::vector<camera::Box>{TruckBox()});
  EXPECT_EQ(40u, rig.fusion->counters().matched);
  EXPECT_EQ(0u, rig.fusion->counters().reused);
  // 100 ms steps, 1 s read window: at most ~11 identities kept.
  EXPECT_LE(rig.fusion->consumed_size(), 12u);
  EXPECT_GE(rig.fusion->consumed_size(), 4u);  // receiver-retained ones are kept
}

TEST(FrontCameraFusionBuffer, DeterministicForIdenticalInputsAndClock) {
  auto run = []() {
    BufferRig rig;
    std::vector<ClassState> states;
    std::vector<std::size_t> pending;
    int k = 0;
    for (; k < 6; ++k) rig.Step(k, {TruckBox()}), states.push_back(StateOf(rig.Live()));
    for (; k < 10; ++k) rig.Step(k, {}), states.push_back(StateOf(rig.Live()));
    ++rig.generation;
    for (; k < 14; ++k) rig.Step(k, {PersonBox()}), states.push_back(StateOf(rig.Live()));
    for (; k < 18; ++k) states.push_back(StateOf((rig.Step(k, {}, true, false), rig.Live())));
    const auto& c = rig.fusion->counters();
    return std::make_tuple(states, c.matched, c.dropped, c.waits, c.epoch_changes, c.misses_applied);
  };
  const auto first = run(), second = run();
  EXPECT_EQ(first, second);
  EXPECT_EQ(14u, std::get<1>(first));
}

// Front-only scope (Main fuses camera_front only): left/right records of the
// same producer, valid, bad or silent, never become candidates and never block
// or change the front result.
TEST(FrontCameraFusionBuffer, SideViewsNeverAffectFrontFusion) {
  auto run = [](int side_mode) {
    BufferRig rig;
    camera::Calibrations calibrations;
    calibrations[rig.front.frame_id] = rig.front;
    for (const char* view : {"camera_left", "camera_right"}) {
      auto side = rig.front;
      side.frame_id = view;
      side.id = std::string("sha256:test_") + view;
      calibrations[view] = side;
    }
    rig.buffer = std::make_shared<camera::Buffer>(calibrations);
    rig.source = std::make_unique<BufferCameraSource>(rig.buffer);
    std::vector<ClassState> states;
    std::vector<std::size_t> matched;
    for (int k = 0; k < 12; ++k) {
      const Ns t = kT0 + k * 100 * kMs;
      EXPECT_EQ("status_accepted", rig.Status(t));
      for (const char* view : {"camera_left", "camera_right"}) {
        if (side_mode == 0) continue;  // side views silent
        camera::Observation o;
        o.header.sequence = 1000 + k;
        o.header.stamp_ns = t + 5 * kMs;  // same stamp as the front frame
        o.header.frame_id = view;
        o.width = 1280;
        o.height = 720;
        o.valid = side_mode != 3;
        if (side_mode == 1) o.boxes = {TruckBox()};  // would confirm if it were used
        if (side_mode == 2) o.boxes = {CameraBox(ProjectedBox(MakeCar(7, 10.f)), "truck", "not_a_group")};
        o.session = rig.session;
        o.generation = rig.generation;
        o.calibration_id = std::string("sha256:test_") + view;
        o.model_id = "yolo26";
        o.config_id = "config";
        rig.buffer->observe(o, t + 20 * kMs, t + 20 * kMs);
      }
      // Front: normal empty frames (no vehicle in front).
      EXPECT_EQ("accepted", rig.Observe(t + 5 * kMs, t + 30 * kMs, {}));
      const auto report = rig.Scan(t, t + 40 * kMs);
      EXPECT_TRUE(report.camera_active);
      EXPECT_EQ("empty", report.camera_reason);
      matched.push_back(report.matched_this_call);
      states.push_back(StateOf(rig.Live()));
    }
    EXPECT_EQ("unknown", rig.Live().camera.str_ObjectClass);  // side trucks never confirm
    return std::make_tuple(states, matched, rig.fusion->counters().confirmations_applied);
  };
  const auto silent = run(0);
  EXPECT_EQ(silent, run(1));  // valid side detections
  EXPECT_EQ(silent, run(2));  // side bad_box clears only the side view
  EXPECT_EQ(silent, run(3));  // side invalid frames
  EXPECT_EQ(0u, std::get<2>(silent));
}

// Main keeps up to 2 LiDAR scans before the first YOLO frame and can match
// them with it (FusionCamera.cpp:820-833). ACTIVE status without a front record
// ("missing") is therefore active with no frames, not inactive.
TEST(FrontCameraFusionBuffer, ScansQueueBeforeTheFirstFrontFrame) {
  BufferRig rig;
  ASSERT_EQ("status_accepted", rig.Status(kT0));
  auto report = rig.Scan(kT0, kT0 + 10 * kMs);
  EXPECT_TRUE(report.camera_active);
  EXPECT_EQ("missing", report.camera_reason);
  EXPECT_EQ(1u, report.pending_scans);
  for (int k = 1; k < 4; ++k) {
    ASSERT_EQ("status_accepted", rig.Status(kT0 + k * 100 * kMs));
    report = rig.Scan(kT0 + k * 100 * kMs, kT0 + k * 100 * kMs + 10 * kMs);
    EXPECT_EQ(2u, report.pending_scans);  // newest two kept
  }
  // First front frame stamped 20 ms after the scan at k=3, received before the
  // next scan: the pending k=3 scan matches it (offset 20 ms <= 50 ms).
  const Ns t3 = kT0 + 300 * kMs;
  ASSERT_EQ("status_accepted", rig.Status(kT0 + 400 * kMs));
  ASSERT_EQ("accepted", rig.Observe(t3 + 20 * kMs, kT0 + 410 * kMs, {TruckBox()}));
  report = rig.Scan(kT0 + 400 * kMs, kT0 + 420 * kMs);
  EXPECT_TRUE(report.camera_active);
  EXPECT_EQ(1u, report.matched_this_call);
  EXPECT_EQ(20 * kMs, report.match_offsets_ns.at(0));
  EXPECT_EQ(1, rig.Live().camera.continuity_counter);
}

// A status session/generation transition clears every receiver view: Main
// bumps the camera epoch then (CameraControl), so classes reset at once even
// before the restarted producer sends a frame.
TEST(FrontCameraFusionBuffer, RestartWithoutFramesResetsClassesImmediately) {
  BufferRig rig;
  const int k = ConfirmVehicle(rig);
  const Ns t = kT0 + k * 100 * kMs;
  const auto epochs = rig.fusion->counters().epoch_changes;
  ++rig.generation;
  ASSERT_EQ("status_accepted", rig.Status(t));
  const auto report = rig.Scan(t, t + 40 * kMs);
  EXPECT_TRUE(report.camera_active);
  EXPECT_EQ("missing", report.camera_reason);
  EXPECT_EQ(epochs + 1, rig.fusion->counters().epoch_changes);
  EXPECT_EQ("unknown", rig.Live().camera.str_ObjectClass);
  EXPECT_EQ("unknown", rig.tracker.history[0].camera.str_ObjectClass);
  ExpectSameGeometry(MakeCar(7, 10.f), rig.Live());
  // Frames of the new generation then confirm again after 5 matches.
  int next = k + 1;
  for (int i = 0; i < 5; ++i) rig.Step(next++, {TruckBox()});
  EXPECT_EQ("vehicle", rig.Live().camera.str_ObjectClass);
  EXPECT_EQ(epochs + 1, rig.fusion->counters().epoch_changes);  // no second reset
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(FusionDiagnostics, HistoricalScanIdentityAndOriginalBoxIndex) {
  FusionConfig config;
  config.initial_confirmations = 1;
  FrontCameraFusion fusion(config, FrontCalibration());
  FakeSource source;
  auto filtered = TruckBox(); filtered.confidence_valid = false;
  source.Set({MakeFrame(kT0, {filtered, TruckBox()})});
  auto first = std::vector<TrackedObject>{MakeCar(7, 10.f)};
  auto c1 = LeveledContext(kT0 + 20*kMs); c1.scan_sequence = 41;
  const auto waiting = fusion.Process(kT0, c1.time.stamp_ns, c1, first, nullptr, source, kT0, kT0);
  ASSERT_EQ(waiting.decisions.size(), 1u);
  EXPECT_EQ(waiting.decisions[0].action, "wait");
  source.Set({MakeFrame(kT0 + 30*kMs, {filtered, TruckBox()})});
  auto c2 = LeveledContext(kT0 + 100*kMs); c2.scan_sequence = 42;
  auto second = std::vector<TrackedObject>{MakeCar(7, 15.f)};
  const auto report = fusion.Process(kT0+100*kMs, c2.time.stamp_ns, c2, second, nullptr,
                                     source, kT0+100*kMs, kT0+100*kMs);
  ASSERT_FALSE(report.decisions.empty());
  const auto& d = report.decisions.front();
  EXPECT_EQ(report.emitting_scan_sequence, 42u);
  EXPECT_EQ(d.scan_sequence, 41u);
  EXPECT_EQ(d.scan_stamp_ns, kT0+20*kMs);
  EXPECT_EQ(d.image_minus_scan_ns, 10*kMs);
  EXPECT_EQ(d.wait_ns, 100*kMs);
  ASSERT_EQ(d.associations.size(), 1u);
  EXPECT_EQ(d.associations[0].source_box_index, 1);
  EXPECT_TRUE(d.associations[0].applied);
  EXPECT_EQ(d.associations[0].reason, "new_observation");
  EXPECT_EQ(second[0].camera.str_ObjectClass, "vehicle");
  const auto expected = ProjectedBox(first[0], c1);
  EXPECT_EQ(d.associations[0].projected_box[0], expected.min_x);
}

TEST(FusionDiagnostics, OnOffParityAndBoundedObjects) {
  FusionConfig on, off; off.capture_diagnostics=false;
  FrontCameraFusion a(on, FrontCalibration()), b(off, FrontCalibration());
  FakeSource source;
  for (int frame=0; frame<8; ++frame) {
    const auto stamp=kT0+frame*100*kMs;
    source.Set({MakeFrame(stamp, {TruckBox()})});
    std::vector<TrackedObject> x, y;
    for (int id=0;id<150;++id) x.push_back(MakeCar(id,10.f));
    y=x;
    auto ra=a.Process(stamp,stamp,LeveledContext(stamp),x,nullptr,source,stamp,stamp);
    auto rb=b.Process(stamp,stamp,LeveledContext(stamp),y,nullptr,source,stamp,stamp);
    ASSERT_EQ(ra.decisions.size(),1u);
    EXPECT_EQ(ra.decisions[0].associations.size(),128u);
    EXPECT_EQ(ra.decisions[0].associations_omitted,22u);
    EXPECT_TRUE(rb.decisions.empty());
    EXPECT_EQ(ra.matched_this_call,rb.matched_this_call);
    for (std::size_t i=0;i<x.size();++i) { EXPECT_EQ(StateOf(x[i]),StateOf(y[i])); ExpectSameGeometry(x[i],y[i]); }
  }
}

TEST(FusionDiagnostics, TimeoutAndCurrentRejectionHaveDistinctReasons) {
  FrontCameraFusion fusion(FusionConfig{},FrontCalibration()); FakeSource source; TrackerSim tracker;
  source.Set({MakeFrame(kT0,{TruckBox()})});
  tracker.Scan(fusion,source,kT0,kT0);
  auto report=tracker.Scan(fusion,source,kT0+100*kMs,kT0+100*kMs);
  ASSERT_FALSE(report.decisions.empty()); EXPECT_EQ(report.decisions.back().action,"wait");
  report=tracker.Scan(fusion,source,kT0+300*kMs,kT0+300*kMs);
  ASSERT_FALSE(report.decisions.empty()); EXPECT_EQ(report.decisions.front().reason,"timeout_no_time_match");
  source.Set({MakeFrame(kT0+300*kMs,{TruckBox()})}); source.current=false;
  report=tracker.Scan(fusion,source,kT0+310*kMs,kT0+310*kMs);
  ASSERT_FALSE(report.decisions.empty()); EXPECT_EQ(report.decisions.front().reason,"observation_no_longer_current");
}

TEST(FusionContract, InvalidProjectionIsNotAnEmptyCameraObservation) {
  FusionConfig config; config.initial_confirmations=1;
  FrontCameraFusion fusion(config,FrontCalibration()); FakeSource source; TrackerSim tracker;
  source.Set({MakeFrame(kT0,{TruckBox()})}); tracker.Scan(fusion,source,kT0,kT0);
  const auto state=StateOf(tracker.live[0]);
  source.Set({MakeFrame(kT0+100*kMs,{TruckBox()})});
  auto invalid=LeveledContext(kT0+100*kMs); invalid.leveling_source.clear();
  const auto report=fusion.Process(kT0+100*kMs,invalid.time.stamp_ns,invalid,
                                   tracker.live,&tracker.history,source,kT0+100*kMs,kT0+100*kMs);
  EXPECT_EQ(StateOf(tracker.live[0]),state);
  EXPECT_EQ(report.matched_this_call,0u);
  ASSERT_FALSE(report.decisions.empty());
  EXPECT_EQ(report.decisions.back().reason,"projection_invalid");
}

TEST(FusionContract, RecheckCameraEpochAfterProjectionBeforeApplying) {
  class ChangesDuringProjection : public FakeSource {
   public:
    int checks=0;
    bool Current(const CameraFrame&,Ns,Ns) override { return ++checks == 1; }
  } source;
  FusionConfig config;config.initial_confirmations=1;
  FrontCameraFusion fusion(config,FrontCalibration());TrackerSim tracker;
  source.Set({MakeFrame(kT0,{TruckBox()})});
  const auto report=tracker.Scan(fusion,source,kT0,kT0);
  EXPECT_EQ(tracker.live[0].camera.str_ObjectClass,"unknown");
  EXPECT_EQ(fusion.consumed_size(),0u);
  EXPECT_EQ(report.matched_this_call,0u);
  EXPECT_EQ(report.pending_scans,1u);
  ASSERT_FALSE(report.decisions.empty());
  EXPECT_EQ(report.decisions.back().reason,"observation_changed_during_projection");
}

namespace {
camera::Calibrations OverlapCameras() {
  camera::Calibrations c;for(const auto name:{"camera_front","camera_left","camera_right"}) {
    auto k=FrontCalibration();k.frame_id=name;k.id=std::string("test_")+name;c[name]=k;
  }return c;
}
void ThreeFrames(FakeSource& source,Ns stamp,const std::vector<std::string>& labels) {
  source.snapshot={};source.snapshot.gate_active=true;source.snapshot.gate_epoch=1;
  const auto c=OverlapCameras();std::size_t i=0;
  for(const auto& v:c) {
    CameraFrame f=MakeFrame(stamp,{});f.frame_id=v.first;f.identity=v.first+std::to_string(stamp);
    const auto label=labels.at(i++);
    if(label!="empty")f.boxes={CameraBox(ProjectedBox(MakeCar(7,10)),label,fusion_class::Normalize(label,true))};
    source.snapshot.by_view[v.first]={f};
    CameraSnapshot::View d;d.frame_id=v.first;d.reason="ok";source.snapshot.views.push_back(d);
  }
}
struct MultiRig {
  MultiCameraFusion fusion{FusionConfig{},OverlapCameras()};FakeSource source;
  std::vector<TrackedObject> objects{MakeCar(7,10)},history;
  FusionReport Run(Ns stamp,Ns decision=0) {
    if(!decision)decision=stamp;
    auto c=LeveledContext(stamp);c.scan_sequence=stamp;
    return fusion.Process(decision,stamp,c,objects,&history,source,decision,decision);
  }
};
}
TEST(MultiCameraFusion, ThreeViewsGiveOnlyOneConfirmationAndPreserveGeometry) {
  MultiRig r;const auto before=r.objects.front();
  for(int i=0;i<5;++i) {
    ThreeFrames(r.source,kT0+i*100*kMs,{"car","car","car"});
    const auto report=r.Run(kT0+i*100*kMs);
    EXPECT_EQ(report.matched_this_call,3u);
    if(i<4){EXPECT_EQ(r.objects[0].camera.str_ObjectClass,"unknown");EXPECT_EQ(r.objects[0].camera.continuity_counter,i+1);}
  }
  EXPECT_EQ(r.objects[0].camera.str_ObjectClass,"vehicle");
  EXPECT_EQ(r.fusion.counters().observations,5u);ExpectSameGeometry(before,r.objects.front());
}
TEST(MultiCameraFusion, PositiveOverridesEmptyAndStaticConflictWithholdsProtection) {
  MultiRig r;ThreeFrames(r.source,kT0,{"car","empty","empty"});r.Run(kT0);
  EXPECT_EQ(r.objects[0].camera.class_miss_counter,0);EXPECT_EQ(r.objects[0].camera.continuity_counter,1);
  EXPECT_EQ(fusion_class::ObservedMissionGroup(r.objects[0].camera),"vehicle");
  ThreeFrames(r.source,kT0+100*kMs,{"car","person","empty"});r.Run(kT0+100*kMs);
  EXPECT_TRUE(r.objects[0].camera.group_conflict);EXPECT_EQ(r.objects[0].camera.continuity_counter,1);
  EXPECT_EQ(fusion_class::ObservedMissionGroup(r.objects[0].camera),"dynamic_obstacle");
  ThreeFrames(r.source,kT0+200*kMs,{"car","box","empty"});r.Run(kT0+200*kMs);
  EXPECT_EQ(fusion_class::ObservedMissionGroup(r.objects[0].camera),"unknown");
}
TEST(MultiCameraFusion, RawClassConflictDoesNotInventAnIdentity) {
  MultiRig r;r.objects[0].camera.str_ObjectClass="vehicle";r.objects[0].camera.str_YoloClass="bus";
  // Initial source epoch invalidates the pre-existing evidence, as in production.
  ThreeFrames(r.source,kT0,{"car","truck","empty"});r.Run(kT0);
  EXPECT_EQ(r.objects[0].camera.observed_raw_class,"unknown");
  EXPECT_EQ(fusion_class::ObservedMissionGroup(r.objects[0].camera),"vehicle");
  for(int i=1;i<6;++i){ThreeFrames(r.source,kT0+i*100*kMs,{"bus","bus","empty"});r.Run(kT0+i*100*kMs);}
  ThreeFrames(r.source,kT0+600*kMs,{"bus","truck","empty"});r.Run(kT0+600*kMs);
  EXPECT_EQ(r.objects[0].camera.str_YoloClass,"bus");
}
TEST(MultiCameraFusion, LateViewReaggregatesSameUnitButCannotUndoNewerScan) {
  MultiRig r;ThreeFrames(r.source,kT0,{"car","car","car"});
  const auto left=r.source.snapshot.by_view["camera_left"];
  r.source.snapshot.by_view.erase("camera_left");r.Run(kT0);
  EXPECT_EQ(r.objects[0].camera.continuity_counter,1);
  r.source.snapshot.by_view["camera_left"]=left;r.Run(kT0,kT0+20*kMs);
  EXPECT_EQ(r.objects[0].camera.continuity_counter,1);
  ThreeFrames(r.source,kT0+100*kMs,{"car","car","car"});r.source.snapshot.by_view.erase("camera_right");r.Run(kT0+100*kMs);
  ThreeFrames(r.source,kT0+200*kMs,{"car","car","car"});r.Run(kT0+200*kMs);
  const auto counter=r.objects[0].camera.continuity_counter;
  ThreeFrames(r.source,kT0+100*kMs,{"box","box","box"});r.Run(kT0+200*kMs,kT0+250*kMs);
  EXPECT_EQ(r.objects[0].camera.continuity_counter,counter);
}
TEST(MultiCameraFusion, OutOfViewIsNotAMissAndStoppedViewDoesNotBlockOtherViews) {
  MultiRig r;r.objects.push_back(MakeCar(8,-20));
  for(int i=0;i<8;++i) {
    ThreeFrames(r.source,kT0+i*100*kMs,{"car","empty","empty"});
    r.source.snapshot.by_view.erase("camera_left");
    r.Run(kT0+i*100*kMs);
    EXPECT_EQ(r.objects[1].camera.class_miss_counter,0);
    EXPECT_LE(r.fusion.pending_size(),3u);
  }
  EXPECT_EQ(r.objects[0].camera.str_ObjectClass,"vehicle");
}
TEST(MultiCameraFusion, RestartInvalidationAndUniqueFrames) {
  MultiRig r;ThreeFrames(r.source,kT0,{"car","car","car"});r.Run(kT0);
  r.Run(kT0+10*kMs);EXPECT_EQ(r.objects[0].camera.continuity_counter,1);
  r.source.snapshot.gate_epoch=2;r.source.snapshot.by_view.clear();r.Run(kT0+20*kMs);
  EXPECT_EQ(r.objects[0].camera.continuity_counter,0);
  EXPECT_EQ(fusion_class::ObservedMissionGroup(r.objects[0].camera),"unknown");
  r.source.snapshot.gate_active=false;r.Run(kT0+30*kMs);EXPECT_EQ(r.fusion.pending_size(),0u);
}

TEST(MultiCameraFusion, FrontOnlyCoordinatorMatchesExistingClassificationSequence) {
  FusionConfig cfg;cfg.capture_diagnostics=true;
  FrontCameraFusion before(cfg,FrontCalibration());
  MultiCameraFusion after(cfg,{{"camera_front",FrontCalibration()}});
  FakeSource a,b;std::vector<TrackedObject> oa{MakeCar(7,10)},ob=oa,ha,hb;
  const std::vector<std::string> labels={"car","car","car","car","car","empty","empty","car","box","box","box","empty","empty","empty","empty"};
  for(std::size_t i=0;i<labels.size();++i) {
    const auto stamp=kT0+i*100*kMs;
    auto f=MakeFrame(stamp,labels[i]=="empty"?std::vector<camera::Box>{}:std::vector<camera::Box>{CameraBox(ProjectedBox(oa[0]),labels[i],fusion_class::Normalize(labels[i],true))});
    a.Set({f});b.snapshot=a.snapshot;b.snapshot.gate_active=true;b.snapshot.gate_epoch=1;
    b.snapshot.by_view["camera_front"]={f};
    before.Process(stamp,stamp,LeveledContext(stamp),oa,&ha,a,stamp,stamp);
    after.Process(stamp,stamp,LeveledContext(stamp),ob,&hb,b,stamp,stamp);
    EXPECT_EQ(StateOf(oa[0]),StateOf(ob[0]))<<"step "<<i;
    ExpectSameGeometry(oa[0],ob[0]);
  }
}
TEST(MultiCameraFusion, SideOpticalAxesOriginalDimensionsAndCalibrationIdentity) {
  auto left=FrontCalibration();left.frame_id="camera_left";left.id="left";
  left.width=640;left.height=480;left.intrinsic<<320,0,320,0,320,240,0,0,1;
  left.base_from_camera.translation()=Vec(0,.8,1.2);
  left.base_from_camera.linear()=rpy(0,0,std::acos(-1.)*.5);
  camera::Calibrations cameras{{left.frame_id,left}};MultiCameraFusion fusion(FusionConfig{},cameras);
  auto obj=MakeCar(7,10);for(auto& p:obj.detection.pcl_ClusterPoints){const float x=p.x;p.x=-p.y;p.y=x;}
  std::vector<TrackedObject> live{obj},history;std::vector<LidarImageBox> boxes;
  ASSERT_TRUE(LidarToImage(live,LeveledContext(kT0),left,FusionConfig{},&boxes));ASSERT_EQ(1u,boxes.size());
  FakeSource source;source.snapshot.gate_active=true;source.snapshot.gate_epoch=1;
  auto f=MakeFrame(kT0,{CameraBox(boxes[0],"person","dynamic_obstacle")});f.width=640;f.height=480;f.frame_id="camera_left";
  source.snapshot.by_view["camera_left"]={f};
  auto r=fusion.Process(kT0,kT0,LeveledContext(kT0),live,&history,source,kT0,kT0);
  EXPECT_EQ(r.matched_this_call,1u);EXPECT_EQ(live[0].camera.observed_class,"dynamic_obstacle");
  auto modified=cameras;modified["camera_left"].id="other";
  EXPECT_NE(camera::fusion_calibration_identity(cameras,true),camera::fusion_calibration_identity(modified,true));
  f.stamp_ns+=100*kMs;f.identity="wrong-size";f.width=1280;source.snapshot.by_view["camera_left"]={f};
  r=fusion.Process(f.stamp_ns,f.stamp_ns,LeveledContext(f.stamp_ns),live,&history,source,f.stamp_ns,f.stamp_ns);
  EXPECT_EQ(r.matched_this_call,0u);
}

TEST(MultiCameraFusion, NewerEmptyViewDoesNotRefreshPositiveEvidenceTime) {
  MultiRig r;ThreeFrames(r.source,kT0,{"car","empty","empty"});
  for(const auto view:{"camera_left","camera_right"}) {
    auto& f=r.source.snapshot.by_view[view].front();f.stamp_ns+=40*kMs;
  }
  r.Run(kT0);EXPECT_EQ(r.objects[0].camera.camera_observation_ns,static_cast<std::uint64_t>(kT0));
}

TEST(MultiCameraFusion, UnavailableContributingViewsRevokeClassWithoutCountingAMiss) {
  struct SelectiveSource : FakeSource {
    std::set<std::string> invalid;
    bool Current(const CameraFrame& frame,Ns,Ns) override { return !invalid.count(frame.frame_id); }
  } source;
  MultiCameraFusion fusion(FusionConfig{},OverlapCameras());
  std::vector<TrackedObject> objects{MakeCar(7,10)},history;
  auto run=[&](int i) {
    const auto stamp=kT0+i*100*kMs;auto context=LeveledContext(stamp);context.scan_sequence=i+1;
    return fusion.Process(stamp,stamp,context,objects,&history,source,stamp,stamp);
  };
  for(int i=0;i<5;++i){ThreeFrames(source,kT0+i*100*kMs,{"car","car","car"});run(i);}
  ASSERT_EQ(objects[0].camera.str_ObjectClass,"vehicle");
  source.invalid={"camera_front"};
  ThreeFrames(source,kT0+500*kMs,{"car","car","car"});run(5);
  EXPECT_EQ(objects[0].camera.str_ObjectClass,"vehicle"); // surviving side evidence is independent
  const auto misses=fusion.counters().misses_applied;
  source.invalid={"camera_front","camera_left","camera_right"};
  for(auto& view:source.snapshot.views)view.reason="invalid";
  run(6);
  EXPECT_EQ(objects[0].camera.str_ObjectClass,"unknown");
  EXPECT_EQ(objects[0].camera.str_YoloClass,"unknown");
  EXPECT_EQ(fusion_class::ObservedMissionGroup(objects[0].camera),"unknown");
  EXPECT_EQ(fusion.counters().misses_applied,misses); // invalid is not a negative detection
  source.invalid.clear();
  for(int i=7;i<12;++i) {
    ThreeFrames(source,kT0+i*100*kMs,{"car","car","car"});run(i);
    EXPECT_EQ(objects[0].camera.str_ObjectClass,i<11?"unknown":"vehicle");
  }
}

TEST(MultiCameraFusion, ExpiredMatchedTokenWithHealthyViewsRetainsClassButNotProtection) {
  struct ExpiringSource : FakeSource {
    Ns valid_from=0;
    bool Current(const CameraFrame& frame,Ns,Ns) override{return frame.stamp_ns>=valid_from;}
  } source;
  MultiCameraFusion fusion(FusionConfig{},OverlapCameras());
  std::vector<TrackedObject> objects{MakeCar(7,10)};
  for(int i=0;i<5;++i) {
    const auto t=kT0+i*100*kMs;ThreeFrames(source,t,{"car","car","car"});
    fusion.Process(t,t,LeveledContext(t),objects,nullptr,source,t,t);
  }
  ASSERT_EQ(objects[0].camera.str_ObjectClass,"vehicle");
  source.valid_from=kT0+500*kMs;
  // Stream is healthy but outside the 50ms matching tolerance for this scan.
  ThreeFrames(source,kT0+500*kMs,{"car","car","car"});
  const auto t=kT0+580*kMs;
  fusion.Process(t,t,LeveledContext(t),objects,nullptr,source,t,t);
  EXPECT_EQ(objects[0].camera.str_ObjectClass,"vehicle");
  EXPECT_EQ(fusion_class::ObservedMissionGroup(objects[0].camera),"unknown");
}
