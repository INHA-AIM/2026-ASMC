// CameraSource over the asmc_lidar camera::Buffer (Receiver). Replaces Main's
// /YOLO/BBox subscription, callback queue and InferenceStatus control gate
// (FusionCamera.cpp:69-85, 192-243): the buffer validates status, time,
// identity and boxes; this adapter only maps its front view to CameraFrames.

#include <asmc_lidar/fusion/front_camera_fusion.hpp>

#include <stdexcept>
#include <string>
#include <utility>

namespace asmc {
namespace lidar {
namespace pipeline {
namespace fusion {

namespace {

// Receiver view reasons with a live front record or a normal empty result, and
// "missing": gate open (status ACTIVE, fresh) but no front record yet, which is
// Main's state before the first YOLO frame (scans are queued, at most 2 kept).
// The fusion detects a "missing" that follows frames as a buffer clear (restart).
// Every other view reason is inactive, including a frame the receiver rejected
// whole (bad_box, calibration_mismatch, identity_conflict, invalid): classes are
// reset at once. Main never saw such frames (its adapter dropped single bad boxes
// or withheld the frame and its 0.35 s silence gate fired later); documented in
// docs/pipeline.md.
bool ActiveReason(const std::string& reason) {
  return reason == "ok" || reason == "empty" || reason == "no_time_match" || reason == "missing";
}

}  // namespace

BufferCameraSource::BufferCameraSource(std::shared_ptr<camera::Buffer> buffer, std::string front_frame_id)
    : buffer_(std::move(buffer)), front_(std::move(front_frame_id)) {
  if (!buffer_ || front_.empty()) throw std::invalid_argument("BufferCameraSource needs a buffer and a frame id");
}

CameraSnapshot BufferCameraSource::Read(std::int64_t reference_stamp_ns, std::int64_t window_ns,
                                        std::int64_t ros_now_ns, std::int64_t steady_now_ns) {
  CameraSnapshot snapshot;
  // Non-blocking, non-consuming; expires old records and applies the gate.
  const camera::Query query = buffer_->query(reference_stamp_ns, window_ns, ros_now_ns, steady_now_ns);
  snapshot.gate_active=query.gate_reason.empty();
  snapshot.gate_reason=query.gate_reason;
  snapshot.gate_epoch=query.epoch;
  for (const auto& entry : query.views) {
    for(const auto& c:entry.second.candidates)if(c.record) {
      const auto& o=c.record->observation;
      CameraFrame f;f.frame_id=o.header.frame_id;f.stamp_ns=o.header.stamp_ns;
      f.epoch=c.record->epoch;f.width=o.width;f.height=o.height;f.boxes=o.boxes;f.record=c.record;
      f.identity=o.session+"|"+std::to_string(o.generation)+"|"+o.header.frame_id+"|"+
          std::to_string(o.header.stamp_ns)+"|"+o.calibration_id;
      snapshot.by_view[entry.first].push_back(std::move(f));
    }
    CameraSnapshot::View summary;
    summary.frame_id = entry.first;summary.matching_implemented=entry.first==front_;
    summary.reason = entry.second.reason;
    summary.candidates = entry.second.candidates.size();
    if (!entry.second.candidates.empty() && entry.second.candidates.front().record) {
      const auto& candidate = entry.second.candidates.front();
      const auto& o = candidate.record->observation;
      summary.stamp_ns = o.header.stamp_ns;
      summary.wire_sequence = o.header.sequence;
      summary.image_minus_scan_ns = candidate.image_minus_scan_ns;
      summary.session = o.session;
      summary.generation = o.generation;
      summary.calibration_id = o.calibration_id;
      summary.model_id = o.model_id;
      summary.config_id = o.config_id;
    }
    snapshot.views.push_back(std::move(summary));
  }
  const auto view = query.views.find(front_);
  if (view == query.views.end()) {
    snapshot.reason = "unknown_camera";
    return snapshot;
  }
  // Gate reasons (closed, status_missing, status_stale, prepare, standby,
  // identity_conflict), bad_query, or the stored reason of a view without a
  // live record (missing, stale, invalid, bad_box, calibration_mismatch).
  snapshot.reason = view->second.reason;
  snapshot.active = ActiveReason(snapshot.reason);
  if (!snapshot.active) return snapshot;

  for (const auto& candidate : view->second.candidates) {
    if (!candidate.record) continue;
    const auto& observation = candidate.record->observation;
    CameraFrame frame;
    frame.stamp_ns = observation.header.stamp_ns;
    frame.epoch = candidate.record->epoch;
    frame.identity = observation.session + "|" + std::to_string(observation.generation) + "|" +
                     observation.header.frame_id + "|" + std::to_string(observation.header.stamp_ns) +
                     "|" + observation.calibration_id;
    frame.width = static_cast<int>(observation.width);
    frame.height = static_cast<int>(observation.height);
    frame.boxes = observation.boxes;
    frame.record = candidate.record;
    snapshot.frames.push_back(std::move(frame));
  }
  // Every buffer epoch change clears all views, so retained records share it.
  if (!snapshot.frames.empty()) snapshot.epoch = snapshot.frames.front().epoch;
  return snapshot;
}

bool BufferCameraSource::Current(const CameraFrame& frame, std::int64_t ros_now_ns,
                                 std::int64_t steady_now_ns) {
  if (!frame.record) return false;
  camera::Candidate candidate;
  candidate.record = frame.record;
  return buffer_->current(candidate, ros_now_ns, steady_now_ns);
}

}  // namespace fusion
}  // namespace pipeline
}  // namespace lidar
}  // namespace asmc
