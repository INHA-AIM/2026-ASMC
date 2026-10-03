#include <asmc_lidar/ground_live.hpp>
#include <asmc_lidar/general.hpp>
#include <ground_segmentation/ground_segmentation.h>
#include <patchwork/patchworkpp.h>
#include <chrono>
#include <stdexcept>

// Generated H1 source is compiled in patchwork_live, isolated from old runtime.
namespace asmc_lidar {
namespace {
using Clock=std::chrono::steady_clock;
double Ms(Clock::time_point start) {
  return std::chrono::duration<double,std::milli>(Clock::now()-start).count();
}
GroundSegmentationParams LinefitParams(const GroundComparisonConfig& config) {
  GroundSegmentationParams p;
  p.sensor_height=1.78;p.r_min_square=1;p.r_max_square=3600;
  p.n_segments=90;p.n_bins=60;p.n_threads=4;p.visualize=false;
  p.max_dist_to_line=.05;p.min_slope=0;p.max_slope=.3;
  p.max_error_square=.05*.05;p.max_long_height=.1;
  p.max_start_height=.2;p.line_search_angle=.1;p.long_threshold=2.;
  p.cover_all_work=config.linefit_cover_all;p.n_threads=config.linefit_threads;
  p.max_dist_to_line=config.linefit_distance;p.line_search_angle=config.linefit_search;
  p.long_threshold=config.linefit_long;
  return p;
}
}
struct GroundComparison::Impl {
  std::string method,variant;
  GroundComparisonConfig config;
  std::unique_ptr<patchwork::PatchWorkpp> pp;
  std::unique_ptr<GroundSegmentation> linefit;
  Impl(std::string m,std::string v,GroundComparisonConfig c):method(std::move(m)),variant(std::move(v)),config(std::move(c)) {
    if(method!="all"&&method!="jaeho"&&method!="patchwork"&&method!="linefit")
      throw std::invalid_argument("method must be all|jaeho|patchwork|linefit");
    if(variant!="fixed_height"&&variant!="guard25")
      throw std::invalid_argument("patchwork_variant must be fixed_height|guard25");
    Reset();
  }
  void Reset() {
    if(method=="all"||method=="patchwork") {
      patchwork::Params p;p.sensor_height=1.80;p.min_range=1;p.max_range=60;
      p.enable_RNR=false;p.height_policy=2;p.height_trace=false;
      p.num_min_pts=config.patchwork_min_points;p.th_dist=config.patchwork_distance;p.th_seeds=config.patchwork_seeds;
      pp.reset(new patchwork::PatchWorkpp(p));
    }
    if(method=="all"||method=="linefit")linefit.reset(new GroundSegmentation(LinefitParams(config)));
  }
};
GroundComparison::GroundComparison(std::string m,std::string v,GroundComparisonConfig c):impl_(new Impl(std::move(m),std::move(v),std::move(c))){}
GroundComparison::~GroundComparison()=default;
void GroundComparison::Reset(){impl_->Reset();}
const GroundComparisonConfig& GroundComparison::config()const{return impl_->config;}
GroundLiveResult GroundComparison::Process(const Cloud& raw,const Calibration& cal,float pitch) {
  return ProcessPrepared(PrepareGeneralGround(raw,cal,pitch),raw.size());
}
GroundLiveResult GroundComparison::ProcessPrepared(const GroundPrepared& prepared,std::size_t raw_size) {
  if(prepared.raw_to_voxel.size()!=raw_size)throw std::invalid_argument("lineage size mismatch");
  GroundLiveResult result;result.prepared=prepared;
  const auto& cloud=*result.prepared.leveled;
  std::map<std::string,GroundMethodResult> voxel_results;
  if(impl_->method=="all"||impl_->method=="jaeho") {
    auto& r=voxel_results["jaeho"];
    const auto start=Clock::now();const auto ref=ProcessGeneralGroundLeveled(cloud);
    r.segment_ms=Ms(start);r.labels.resize(cloud.size(),2);
    r.structural_removed.resize(cloud.size(),0);
    for(std::size_t i=0;i<cloud.size();++i) {
      if(ref.labels[i]==tunnel_filter::kGround)r.labels[i]=1;
      if(ref.labels[i]==tunnel_filter::kWall)r.structural_removed[i]=1;
    }
  }
  if(impl_->pp) {
    auto& r=voxel_results["patchwork"];r.labels.assign(cloud.size(),3);
    const auto copy=Clock::now();Eigen::MatrixXf matrix(cloud.size(),4);
    for(std::size_t i=0;i<cloud.size();++i)
      matrix.row(Eigen::Index(i))<<cloud[i].x,cloud[i].y,cloud[i].z,cloud[i].intensity;
    r.copy_ms=Ms(copy);
    if(!cloud.empty()) {
      const auto start=Clock::now();impl_->pp->estimateGround(matrix);r.segment_ms=Ms(start);
      auto fill=[&](const Eigen::VectorXi& ids,std::uint8_t value){
        for(Eigen::Index k=0;k<ids.size();++k) {
          const auto i=ids[k];
          if(i<0||std::size_t(i)>=cloud.size()||r.labels[i]!=3)
            throw std::runtime_error("Patchwork index duplication/range");
          r.labels[i]=value;
        }
      };
      fill(impl_->pp->getGroundIndices(),1);fill(impl_->pp->getNongroundIndices(),2);
      r.reasons=impl_->pp->point_reasons;
    }
    const auto guard=Clock::now();
    r.pre_guard_labels=r.labels;
    if(impl_->variant=="guard25")for(std::size_t i=0;i<cloud.size();++i)
      if(r.labels[i]==1&&cloud[i].z>float(-1.80+.25)){r.labels[i]=2;r.reasons[i]|=2048;}
    r.guard_ms=Ms(guard);
  }
  if(impl_->linefit) {
    auto& r=voxel_results["linefit"];
    const auto copy=Clock::now();pcl::PointCloud<pcl::PointXYZ> xyz;xyz.reserve(cloud.size());
    for(const auto& p:cloud)xyz.emplace_back(p.x,p.y,p.z);
    r.copy_ms=Ms(copy);std::vector<int> labels;
    const auto start=Clock::now();impl_->linefit->segment(xyz,&labels);r.segment_ms=Ms(start);
    if(labels.size()!=cloud.size())throw std::runtime_error("linefit label size");
    r.labels.reserve(labels.size());for(int x:labels)r.labels.push_back(x?1:2);
    r.reasons=impl_->linefit->point_reasons;
    const char* names[]={"fit_residual_reject","fit_slope_reject","long_height_reject","start_gap_reject","start_height_reject"};
    for(int i=0;i<5;++i)r.events[names[i]]=impl_->linefit->fit_reason_counts[i];
    r.events["unvisited_sectors"]=std::count(impl_->linefit->fit_visits.begin(),impl_->linefit->fit_visits.end(),0);
    r.events["unvisited_points"]=std::count(impl_->linefit->assignment_visits.begin(),impl_->linefit->assignment_visits.end(),0);
  }
  for(auto& entry:voxel_results) {
    const auto start=Clock::now();auto& dst=result.methods[entry.first];
    dst=entry.second;dst.labels.assign(raw_size,0);dst.structural_removed.assign(raw_size,0);
    if(!entry.second.pre_guard_labels.empty())dst.pre_guard_labels.assign(raw_size,0);
    if(!entry.second.reasons.empty())dst.reasons.assign(raw_size,0);
    for(std::size_t i=0;i<raw_size;++i) {
      const auto j=result.prepared.raw_to_voxel[i];if(j<0)continue;
      dst.labels[i]=entry.second.labels.at(j);
      if(!entry.second.structural_removed.empty())dst.structural_removed[i]=entry.second.structural_removed.at(j);
      if(!entry.second.pre_guard_labels.empty())dst.pre_guard_labels[i]=entry.second.pre_guard_labels.at(j);
      if(!entry.second.reasons.empty())dst.reasons[i]=entry.second.reasons.at(j);
    }
    dst.restore_ms=Ms(start);
  }
  return result;
}
}  // namespace asmc_lidar
