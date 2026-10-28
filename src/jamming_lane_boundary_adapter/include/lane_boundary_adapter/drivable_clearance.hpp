#pragma once
// Opt-in local experiment. A semantic free-space layer supplements, never
// clears, measured LiDAR footprints or visible camera contacts. No map pose.
#include "lane_boundary_adapter/contact_clearance.hpp"
#include <cstdint>
namespace lane_boundary_adapter::clearance {
struct GridSpec { int rows=0,cols=0;double x_max=0,y_max=0,resolution=0; };
struct GridResult { bool clear=false; unsigned cells=0,blocked=0,assisted=0;double blocked_x=0.,blocked_y=0.;const char* reason="invalid_grid"; };
class DrivableClearance {
 public:
  bool prepare(GridSpec spec,const std::vector<uint8_t>& probability,const std::vector<uint8_t>& coverage,
               const Vehicle& vehicle,uint8_t threshold=166,bool physical_ego_only=false,bool near_ego_lidar_assist=false) {
    valid_=false;spec_=spec;
    if(spec.rows<1||spec.cols<1||spec.rows>1000||spec.cols>1000||!std::isfinite(spec.resolution)||spec.resolution<=0||
       !std::isfinite(spec.x_max)||!std::isfinite(spec.y_max)||probability.size()!=size_t(spec.rows*spec.cols)||coverage.size()!=probability.size())return false;
    prefix_.assign(size_t(spec.rows)*(spec.cols+1),0);
    assisted_.assign(prefix_.size(),0);observed_blocked_.assign(probability.size(),0);
    for(int r=0;r<spec.rows;++r)for(int c=0;c<spec.cols;++c){
      const double x=spec.x_max-r*spec.resolution,y=spec.y_max-c*spec.resolution;
      // The current inflated ego region is not NEW swept space. Camera blind
      // cells here cannot certify free space; positive local/image geometry
      // still checks the whole body, including this region, independently.
      const double longitudinal=physical_ego_only?0.:vehicle.longitudinal_margin;
      const double lateral=physical_ego_only?0.:vehicle.lateral_margin;
      const bool initial=x>=-vehicle.rear-longitudinal && x<=vehicle.front+longitudinal+(physical_ego_only?0.:.25) &&
          std::abs(y)<=vehicle.half_width+lateral;
      const size_t k=size_t(r)*spec.cols+c,base=size_t(r)*(spec.cols+1);
      // Explicit S3 sensitivity candidate, never "observed free": only
      // camera ego masks/FOV absence in the measured near blind region may
      // defer to a fresh, paired local LiDAR health/positive-footprint check.
      // Low-probability OBSERVED shadow pixels never enter this exception.
      const bool assisted=near_ego_lidar_assist &&
          ((coverage[k]==128 && std::abs(y)<=2.5) || (coverage[k]==0 && std::abs(y)<=1.6)) &&
          x>=-vehicle.rear-vehicle.longitudinal_margin-.25 && x<=6.5; // r06 mask extent max6.3m +2 grid cells; trial only
      observed_blocked_[k]=coverage[k]==255 && probability[k]<threshold;
      const bool bad=!initial && !assisted && (coverage[k]!=255 || probability[k]<threshold);
      assisted_[base+c+1]=assisted_[base+c]+unsigned(assisted && !initial);
      prefix_[base+c+1]=prefix_[base+c]+unsigned(bad);
    }
    valid_=true;return true;
  }
  GridResult check(const std::vector<Point>& path,const Vehicle& v,double sample_padding=.25)const {
    GridResult result;if(!valid_||path.size()<2)return result;
    result.reason="semantic_blocked_or_unobserved";
    for(size_t i=0;i<path.size();++i){
      if(!finite(path[i])){result.reason="invalid_path";return result;}
      const auto& a=path[i?i-1:i];const auto& b=path[std::min(i+1,path.size()-1)];
      double yaw=std::atan2(b.y-a.y,b.x-a.x),co=std::cos(yaw),si=std::sin(yaw);
      Polygon polygon;
      for(Point p:Polygon{{v.front+v.longitudinal_margin+sample_padding,v.half_width+v.lateral_margin},
        {-v.rear-v.longitudinal_margin-sample_padding,v.half_width+v.lateral_margin},
        {-v.rear-v.longitudinal_margin-sample_padding,-v.half_width-v.lateral_margin},
        {v.front+v.longitudinal_margin+sample_padding,-v.half_width-v.lateral_margin}})
        polygon.push_back({path[i].x+p.x*co-p.y*si,path[i].y+p.x*si+p.y*co});
      double xmin=1e100,xmax=-1e100;for(auto p:polygon){xmin=std::min(xmin,p.x);xmax=std::max(xmax,p.x);}
      const double half=spec_.resolution*.5;
      int r0=int(std::ceil((spec_.x_max-xmax-half)/spec_.resolution));
      int r1=int(std::floor((spec_.x_max-xmin+half)/spec_.resolution));
      if(r0<0||r1>=spec_.rows){result.reason="outside_grid";return result;}
      for(int r=r0;r<=r1;++r){
        double x=spec_.x_max-r*spec_.resolution,ymin=1e100,ymax=-1e100;
        // Every cell intersecting the swept rectangle is included. Row-strip
        // extrema occur at its boundaries or a rectangle vertex, not at the
        // centreline alone. Prefix sums make each row query constant time.
        auto add=[&](double y){ymin=std::min(ymin,y);ymax=std::max(ymax,y);};
        for(size_t j=0;j<polygon.size();++j){auto p=polygon[j],q=polygon[(j+1)%polygon.size()];
          if(p.x>=x-half && p.x<=x+half)add(p.y);
          if(std::abs(q.x-p.x)>1e-12)for(double edge:{x-half,x+half}){
            const double t=(edge-p.x)/(q.x-p.x);if(t>=0&&t<=1)add(p.y+t*(q.y-p.y));}
        }
        if(ymax<ymin)continue;
        int c0=int(std::ceil((spec_.y_max-ymax-half)/spec_.resolution));
        int c1=int(std::floor((spec_.y_max-ymin+half)/spec_.resolution));
        if(c0<0||c1>=spec_.cols){result.reason="outside_grid";return result;}
        const size_t base=size_t(r)*(spec_.cols+1);result.cells+=c1-c0+1;
        result.blocked+=prefix_[base+c1+1]-prefix_[base+c0];
        result.assisted+=assisted_[base+c1+1]-assisted_[base+c0];
        if(result.blocked){
          // First blocked cell, for bounded diagnostics/replay; no extra grid scan.
          auto begin=prefix_.begin()+base;
          auto first=std::upper_bound(begin+c0,begin+c1+2,prefix_[base+c0]);
          const int column=int(first-begin)-1;
          result.blocked_x=x;result.blocked_y=spec_.y_max-column*spec_.resolution;
          result.reason=observed_blocked_[size_t(r)*spec_.cols+column]?"semantic_nonroad":"semantic_unobserved";
          return result;
        }
      }
    }
    result.clear=true;result.reason="semantic_swept_space_clear";return result;
  }
 private:GridSpec spec_;bool valid_=false;std::vector<unsigned> prefix_,assisted_;std::vector<uint8_t> observed_blocked_;
};
} // namespace lane_boundary_adapter::clearance
