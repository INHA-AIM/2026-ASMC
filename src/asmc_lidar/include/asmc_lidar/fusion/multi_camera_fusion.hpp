#pragma once
#include <asmc_lidar/fusion/front_camera_fusion.hpp>

namespace asmc::lidar::pipeline::fusion {
// Non-blocking per-view matching. Pending scans retain projected rectangles,
// not three point clouds. Each rectangle was computed with its scan Context.
class MultiCameraFusion {
 public:
  MultiCameraFusion(const FusionConfig&, const camera::Calibrations&);
  FusionReport Process(std::int64_t decision_ns, std::int64_t scan_stamp_ns,
      const Context&, std::vector<TrackedObject>& live, std::vector<TrackedObject>* history,
      CameraSource&, std::int64_t ros_now_ns, std::int64_t steady_now_ns);
  const FusionCounters& counters() const {return counters_;}
  std::uint64_t consumer_epoch() const {return consumer_epoch_;}
  std::size_t pending_size() const {return pending_.size();}
 private:
  struct Evidence {
    bool visible=false, positive=false;
    std::string group="unknown",raw="unknown";
    CameraFrame source; // boxes cleared after matching; immutable receiver token retained
  };
  struct View {
    bool done=false, projection_valid=false;
    std::vector<LidarImageBox> projection;
    std::map<int,Evidence> evidence;
  };
  struct Scan {
    std::int64_t stamp=0,queued=0;
    std::uint64_t sequence=0;
    std::vector<int> ids;
    std::vector<std::array<float,2>> dimensions; // scan-time XY diagonal and height
    std::map<int,CameraPointSupport> point_support; // measured membership at this scan
    std::map<std::string,View> views;
    std::map<int,CameraClassEvidence> start;
  };
  void Aggregate(Scan&, std::vector<TrackedObject>&, CameraSource&, std::int64_t, std::int64_t);
  FusionConfig config_;
  camera::Calibrations calibrations_;
  std::vector<std::string> allowed_;
  std::deque<Scan> pending_;
  std::map<std::string,std::int64_t> consumed_;
  std::map<int,std::int64_t> applied_scan_;
  std::map<int,std::vector<Evidence>> latest_evidence_;
  FusionCounters counters_;
  std::uint64_t source_epoch_=0,consumer_epoch_=1;
  bool active_=false;
  std::int64_t last_scan_=0,offset_=0,wait_=0,window_=0;
};
}
