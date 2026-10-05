#include "AvoidRoadMap.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <fstream>

namespace {
std::uint64_t BinKey(int x,int y) { return (std::uint64_t(std::uint32_t(x))<<32)|std::uint32_t(y); }
}

bool AvoidRoadMap::Load(const std::string& areas, const std::string& lines,
                        std::string& error) {
  enabled_ = false; areas_.clear(); protected_lines_.clear(); line_bins_.clear();
  try {
    const auto product = YAML::LoadFile(areas), markings = YAML::LoadFile(lines);
    if (product["frame_id"].as<std::string>() != "map" ||
        product["map_id"].as<std::string>() != "waize_katri_2025-10-23" ||
        markings["map_id"].as<std::string>() != "kcity2025_competition_route_curation_candidate_v14" ||
        markings["frame"]["local_origin_in_global"][0].as<double>() != 302595.0 ||
        markings["frame"]["local_origin_in_global"][1].as<double>() != 4124145.0)
      throw std::runtime_error("HD map frame/origin mismatch");
    for (const auto& area : product["drivable_areas"]) {
      if (area["validity"].as<std::string>() != "validated") continue;
      Polygon polygon{{}, INFINITY, INFINITY, -INFINITY, -INFINITY};
      for (const auto& coordinate : area["polygon"]) {
        Point point{coordinate[0].as<double>(), coordinate[1].as<double>()};
        if (!std::isfinite(point.x) || !std::isfinite(point.y))
          throw std::runtime_error("nonfinite HD map polygon");
        polygon.points.push_back(point);
        polygon.xmin=std::min(polygon.xmin, point.x); polygon.xmax=std::max(polygon.xmax, point.x);
        polygon.ymin=std::min(polygon.ymin, point.y); polygon.ymax=std::max(polygon.ymax, point.y);
      }
      if (polygon.points.size() < 3) throw std::runtime_error("degenerate HD map polygon");
      areas_.push_back(std::move(polygon));
    }
    for (const auto& feature : markings["features"]) {
      const auto attributes = feature["attributes"];
      if (feature["feature_type"].as<std::string>() != "lane_marking" ||
          attributes["pattern"].as<std::string>() != "solid" ||
          attributes["functional_role"].as<std::string>() != "lane_boundary_paint") continue;
      const auto coordinates=feature["geometry"]["coordinates"];
      for (std::size_t i=1; i<coordinates.size(); ++i) {
        const Point a{coordinates[i-1][0].as<double>(),coordinates[i-1][1].as<double>()};
        const Point b{coordinates[i][0].as<double>(),coordinates[i][1].as<double>()};
        protected_lines_.push_back({a,b,std::min(a.x,b.x),std::min(a.y,b.y),std::max(a.x,b.x),std::max(a.y,b.y)});
      }
    }
    if (areas_.empty() || protected_lines_.empty()) throw std::runtime_error("empty HD map");
    for(std::size_t index=0;index<protected_lines_.size();++index){
      const auto& line=protected_lines_[index];
      for(int x=static_cast<int>(std::floor(line.xmin/10));x<=static_cast<int>(std::floor(line.xmax/10));++x)
        for(int y=static_cast<int>(std::floor(line.ymin/10));y<=static_cast<int>(std::floor(line.ymax/10));++y)
          line_bins_[BinKey(x,y)].push_back(index);
    }
    const auto slash=areas.find_last_of("/\\");
    std::ifstream grid(areas.substr(0,slash+1)+"drivable_grid.rle");
    if(!(grid>>grid_width_>>grid_height_>>grid_resolution_>>grid_x_>>grid_y_) ||
       grid_width_!=3085 || grid_height_!=7232 || grid_resolution_!=.2 ||
       std::fabs(grid_x_-product["grid"]["origin_x"].as<double>())>.001 ||
       std::fabs(grid_y_-product["grid"]["origin_y"].as<double>())>.001)
      throw std::runtime_error("HD map raster metadata mismatch");
    grid_rows_.assign(grid_height_,{});
    int row,begin,end;
    while(grid>>row>>begin>>end){
      if(row<0||row>=grid_height_||begin<0||end>grid_width_||begin>=end)
        throw std::runtime_error("invalid HD map raster run");
      grid_rows_[row].push_back({begin,end});
    }
    if(!grid.eof())throw std::runtime_error("malformed HD map raster");
    enabled_=true; return true;
  } catch (const std::exception& exception) { error=exception.what(); return false; }
}

bool AvoidRoadMap::Inside(double x, double y) const {
  const int ix=static_cast<int>(std::floor((x-grid_x_)/grid_resolution_));
  const int iy=static_cast<int>(std::floor((y-grid_y_)/grid_resolution_));
  if(ix<0||ix>=grid_width_||iy<0||iy>=grid_height_)return false;
  for(const auto& run:grid_rows_[iy])if(ix>=run.first&&ix<run.second)return true;
  return false;
}

bool AvoidRoadMap::PoseClear(double x,double y,double yaw,const VehicleShape& shape) const {
  if (!enabled_) return true;
  if (!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw)||
      !std::isfinite(shape.width)||!std::isfinite(shape.length)||!std::isfinite(shape.rear_overhang)||
      shape.width<=0||shape.length<=0||shape.rear_overhang<0||shape.rear_overhang>shape.length) return false;
  const double c=std::cos(yaw),s=std::sin(yaw);
  const double rear=shape.rear_overhang, front=shape.length-rear, half=shape.width*.5;
  // Entire body, including its interior, at <=0.20m spacing. This is deliberately
  // a sampled guard, not an exact polygon-union containment claim.
  const int nx=static_cast<int>(std::ceil(shape.length/.20)), ny=static_cast<int>(std::ceil(shape.width/.20));
  for(int i=0;i<=nx;++i) for(int j=0;j<=ny;++j) {
    const double a=-rear+shape.length*i/nx,b=-half+shape.width*j/ny;
    if(!Inside(x+c*a-s*b,y+s*a+c*b))return false;
  }
  // Clip actual paint segments against the body rectangle. Do not bridge paint
  // gaps or infer pattern from them, and do not reject junction guide paint.
  const double center_x=x+c*(front-rear)*.5,center_y=y+s*(front-rear)*.5;
  const double radius_x=std::fabs(c)*shape.length*.5+std::fabs(s)*half;
  const double radius_y=std::fabs(s)*shape.length*.5+std::fabs(c)*half;
  for(int ix=static_cast<int>(std::floor((center_x-radius_x)/10));ix<=static_cast<int>(std::floor((center_x+radius_x)/10));++ix)
  for(int iy=static_cast<int>(std::floor((center_y-radius_y)/10));iy<=static_cast<int>(std::floor((center_y+radius_y)/10));++iy) {
    const auto found=line_bins_.find(BinKey(ix,iy));
    if(found==line_bins_.end())continue;
    for(const auto index:found->second) {
    const auto& line=protected_lines_[index];
    if(line.xmax<center_x-radius_x||line.xmin>center_x+radius_x||
       line.ymax<center_y-radius_y||line.ymin>center_y+radius_y)continue;
    const double ax=c*(line.a.x-x)+s*(line.a.y-y),ay=-s*(line.a.x-x)+c*(line.a.y-y);
    const double bx=c*(line.b.x-x)+s*(line.b.y-y),by=-s*(line.b.x-x)+c*(line.b.y-y);
    double lo=0,hi=1;
    auto clip=[&](double a,double b,double min,double max){
      const double delta=b-a;
      if(std::fabs(delta)<1e-12) return a>=min&&a<=max;
      double u=(min-a)/delta,v=(max-a)/delta;
      if(u>v)std::swap(u,v);
      lo=std::max(lo,u);hi=std::min(hi,v);return lo<=hi;
    };
    if(clip(ax,bx,-rear,front)&&clip(ay,by,-half,half))return false;
  }
  }
  return true;
}

bool AvoidRoadMap::PathClear(const CartesianPath& path,const VehicleShape& shape) const {
  if(!enabled_)return true;
  if(path.x.empty()||path.y.size()!=path.x.size()||path.yaw.size()!=path.x.size())return false;
  for(std::size_t i=0;i<path.x.size();++i){
    // Reject invalid endpoints before computing an interpolation count.
    // Converting an infinite distance to int would otherwise be undefined.
    if(!std::isfinite(path.x[i])||!std::isfinite(path.y[i])||
       !std::isfinite(path.yaw[i]))return false;
    if(!PoseClear(path.x[i],path.y[i],path.yaw[i],shape))return false;
    if(!i)continue;
    const double angle=std::remainder(path.yaw[i]-path.yaw[i-1],2*M_PI);
    const int count=std::max(1,static_cast<int>(std::ceil(std::max(
      std::hypot(path.x[i]-path.x[i-1],path.y[i]-path.y[i-1])/.20,std::fabs(angle)/.02))));
    for(int j=1;j<count;++j){const double f=double(j)/count;
      if(!PoseClear(path.x[i-1]+f*(path.x[i]-path.x[i-1]),path.y[i-1]+f*(path.y[i]-path.y[i-1]),
                    path.yaw[i-1]+f*angle,shape))return false;
    }
  }
  return true;
}
