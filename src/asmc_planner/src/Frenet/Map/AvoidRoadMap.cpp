#include "AvoidRoadMap.hpp"
#include "Frenet/Geometry/PolygonUtils.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <fstream>
#include <sstream>

namespace {
std::uint64_t BinKey(int x,int y) { return (std::uint64_t(std::uint32_t(x))<<32)|std::uint32_t(y); }
}

bool AvoidRoadMap::IntersectsDrivable(const geometry::Polygon2D& footprint) const {
  if(!enabled_ || footprint.size()<3 || footprint.size()>64) return false;
  double xmin=INFINITY,ymin=INFINITY,xmax=-INFINITY,ymax=-INFINITY;
  for(const auto& p:footprint) {
    if(!std::isfinite(p.x)||!std::isfinite(p.y)) return false;
    xmin=std::min(xmin,p.x);xmax=std::max(xmax,p.x);
    ymin=std::min(ymin,p.y);ymax=std::max(ymax,p.y);
  }
  const auto hull=geometry::ComputeConvexHull(footprint);
  if(geometry::PolygonArea(hull)<1e-6 || xmin<grid_x_ || ymin<grid_y_ ||
      xmax>=grid_x_+grid_width_*grid_resolution_ || ymax>=grid_y_+grid_height_*grid_resolution_) return false;
  if(paint_exempt_)for(const auto& area:shadow_wall_areas_){
    if(xmax<area.xmin||xmin>area.xmax||ymax<area.ymin||ymin>area.ymax)continue;
    geometry::Polygon2D polygon;for(const auto& p:area.points)polygon.push_back({p.x,p.y});
    if(geometry::CheckConvexPolygonOverlapSAT(hull,polygon))return true;
  }
  const int x0=std::max(0,int(std::floor((xmin-grid_x_)/grid_resolution_))-1);
  const int x1=std::min(grid_width_-1,int(std::floor((xmax-grid_x_)/grid_resolution_)));
  const int y0=std::max(0,int(std::floor((ymin-grid_y_)/grid_resolution_))-1);
  const int y1=std::min(grid_height_-1,int(std::floor((ymax-grid_y_)/grid_resolution_)));
  // A raster run is exactly the union of its adjacent free cells. Intersect
  // each clipped run rectangle once, rather than imposing a cell-count cutoff
  // that could mislabel a long/rotated bus as outside the road.
  for(int y=y0;y<=y1;++y) for(const auto& run:grid_rows_[y]) {
    const int begin=std::max(x0,run.first),end=std::min(x1+1,run.second);
    if(begin>=end)continue;
    const double a=grid_x_+begin*grid_resolution_,z=grid_x_+end*grid_resolution_;
    const double b=grid_y_+y*grid_resolution_;
    if(geometry::CheckConvexPolygonOverlapSAT(hull,{{a,b},{z,b},
        {z,b+grid_resolution_},{a,b+grid_resolution_}})) return true;
  }
  return false;
}

bool AvoidRoadMap::Load(const std::string& areas, const std::string& lines,
                        std::string& error) {
  enabled_ = false; paint_exempt_=false; areas_.clear(); shadow_wall_areas_.clear(); product_sha256_.clear(); protected_lines_.clear(); line_bins_.clear();
  try {
    const auto product = YAML::LoadFile(areas), markings = YAML::LoadFile(lines);
    product_sha256_=product["product_sha256"].as<std::string>("");
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
          attributes["pattern"].as<std::string>() != "solid") continue;
      const std::string color = attributes["color"]
          ? attributes["color"].as<std::string>() : "";
      if (!color.empty() && color != "yellow" && color != "white") continue;
      const double half_width = attributes["line_width_m"]
          ? attributes["line_width_m"].as<double>() * 0.5 : 0.0;
      if (!std::isfinite(half_width) || half_width < 0.0)
        throw std::runtime_error("invalid HD map solid-paint width");
      const auto coordinates=feature["geometry"]["coordinates"];
      for (std::size_t i=1; i<coordinates.size(); ++i) {
        const Point a{coordinates[i-1][0].as<double>(),coordinates[i-1][1].as<double>()};
        const Point b{coordinates[i][0].as<double>(),coordinates[i][1].as<double>()};
        protected_lines_.push_back({a,b,std::min(a.x,b.x)-half_width,
            std::min(a.y,b.y)-half_width,std::max(a.x,b.x)+half_width,
            std::max(a.y,b.y)+half_width,half_width,
            attributes["functional_role"] &&
                attributes["functional_role"].as<std::string>() == "lane_boundary_paint"});
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
    // All modes share this reader. Refuse a v6 vector paired with the older
    // same-sized raster; dimensions alone cannot identify the road product.
    if(grid.peek()=='#') {
      std::string header;std::getline(grid,header);
      std::istringstream metadata(header);std::string marker,format,product_sha,grid_sha,extra;
      if(!(metadata>>marker>>format>>product_sha>>grid_sha)||metadata>>extra||
         format!="asmc.drivable_grid/v1"||
         product_sha!=product["product_sha256"].as<std::string>("")||
         grid_sha!=product["grid"]["sha256"].as<std::string>(""))
        throw std::runtime_error("HD map vector/raster identity mismatch");
    } else if(product["schema_version"].as<std::string>("")=="asmc.hd_map.compact_drivable/v6") {
      throw std::runtime_error("v6 HD map requires its matching raster identity header");
    }
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
  if(!std::isfinite(x+y))return false;
  if(paint_exempt_)for(const auto& area:shadow_wall_areas_){
    if(x<area.xmin||x>area.xmax||y<area.ymin||y>area.ymax)continue;
    bool inside=true;
    for(size_t i=0;i<area.points.size();++i){const auto& a=area.points[i];const auto& b=area.points[(i+1)%area.points.size()];
      if((b.x-a.x)*(y-a.y)-(b.y-a.y)*(x-a.x)<-1e-8){inside=false;break;}}
    if(inside)return true;
  }
  const int ix=static_cast<int>(std::floor((x-grid_x_)/grid_resolution_));
  const int iy=static_cast<int>(std::floor((y-grid_y_)/grid_resolution_));
  if(ix<0||ix>=grid_width_||iy<0||iy>=grid_height_)return false;
  for(const auto& run:grid_rows_[iy])if(ix>=run.first&&ix<run.second)return true;
  return false;
}

bool AvoidRoadMap::SetShadowWallOverlay(const std::vector<geometry::Polygon2D>& polygons){
  if(!enabled_||polygons.size()>100)return false;
  std::vector<Polygon> checked;
  for(const auto& points:polygons){
    if(points.size()<3||points.size()>8)return false;
    auto hull=geometry::ComputeConvexHull(points);
    if(hull.size()!=points.size()||geometry::PolygonArea(hull)>50.||geometry::PolygonArea(hull)<.01)return false;
    Polygon area;area.xmin=area.ymin=INFINITY;area.xmax=area.ymax=-INFINITY;
    for(const auto& p:hull){
      if(!std::isfinite(p.x+p.y)||p.x<grid_x_||p.y<grid_y_||
         p.x>=grid_x_+grid_width_*grid_resolution_||p.y>=grid_y_+grid_height_*grid_resolution_)return false;
      // The authoring layer may add only shoulders near existing road cells.
      const int ix=int(std::floor((p.x-grid_x_)/grid_resolution_)),iy=int(std::floor((p.y-grid_y_)/grid_resolution_));
      bool near=false;
      for(int row=std::max(0,iy-13);row<=std::min(grid_height_-1,iy+13)&&!near;++row)
        for(const auto& run:grid_rows_[row])if(run.second>ix-13&&run.first<=ix+13){near=true;break;}
      if(!near)return false;
      area.points.push_back({p.x,p.y});area.xmin=std::min(area.xmin,p.x);area.xmax=std::max(area.xmax,p.x);
      area.ymin=std::min(area.ymin,p.y);area.ymax=std::max(area.ymax,p.y);
    }
    checked.push_back(std::move(area));
  }
  shadow_wall_areas_=std::move(checked);return true;
}

bool AvoidRoadMap::DrivablePoseClear(double x,double y,double yaw,const VehicleShape& shape) const {
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
  return true;
}

AvoidRoadMap::Overlap AvoidRoadMap::FootprintOverlap(
    double x,double y,double yaw,const VehicleShape& shape) const {
  Overlap result;
  if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw)||
     !std::isfinite(shape.width)||!std::isfinite(shape.length)||!std::isfinite(shape.rear_overhang)||
     shape.width<=0||shape.length<=0||shape.rear_overhang<0||shape.rear_overhang>shape.length)
    return {INFINITY,1.0};
  const double c=std::cos(yaw),s=std::sin(yaw);
  const int nx=static_cast<int>(std::ceil(shape.length/.20)),ny=static_cast<int>(std::ceil(shape.width/.20));
  int outside=0;
  for(int i=0;i<=nx;++i)for(int j=0;j<=ny;++j){
    const double a=-shape.rear_overhang+shape.length*i/nx,b=-shape.width*.5+shape.width*j/ny;
    const double px=x+c*a-s*b,py=y+s*a+c*b;
    if(Inside(px,py))continue;
    ++outside; double nearest=INFINITY;
    const int ix=static_cast<int>(std::floor((px-grid_x_)/grid_resolution_));
    const int iy=static_cast<int>(std::floor((py-grid_y_)/grid_resolution_));
    // Distance to the original free-cell rectangles, not a dilated map.
    for(int row=std::max(0,iy-3);row<=std::min(grid_height_-1,iy+3);++row)
      for(const auto& run:grid_rows_[row]){
        if(run.second<=ix-3||run.first>ix+3)continue;
        const double left=grid_x_+run.first*grid_resolution_,right=grid_x_+run.second*grid_resolution_;
        const double bottom=grid_y_+row*grid_resolution_,top=bottom+grid_resolution_;
        const double dx=std::max({left-px,0.0,px-right}),dy=std::max({bottom-py,0.0,py-top});
        nearest=std::min(nearest,std::hypot(dx,dy));
      }
    result.maximum=std::max(result.maximum,nearest);
  }
  result.fraction=double(outside)/((nx+1)*(ny+1));return result;
}

bool AvoidRoadMap::TouchesPaint(double x,double y,double yaw,const VehicleShape& shape,
                                bool all_solids) const {
  const double c=std::cos(yaw),s=std::sin(yaw);
  const double rear=shape.rear_overhang,front=shape.length-rear;
  // AVOID counts the exterior body as a conservative proxy for the referee's
  // wheel contacts. The 0.5 s duration reserve handles tracking/sampling error.
  const double half=(all_solids?std::max(shape.width,shape.road_outer_width):shape.width)*.5;
  // Clip actual paint segments against the body rectangle. Do not bridge gaps
  // or infer paint from them; broken guides are absent from protected_lines_.
  const double center_x=x+c*(front-rear)*.5,center_y=y+s*(front-rear)*.5;
  const double radius_x=std::fabs(c)*shape.length*.5+std::fabs(s)*half;
  const double radius_y=std::fabs(s)*shape.length*.5+std::fabs(c)*half;
  for(int ix=static_cast<int>(std::floor((center_x-radius_x)/10));ix<=static_cast<int>(std::floor((center_x+radius_x)/10));++ix)
  for(int iy=static_cast<int>(std::floor((center_y-radius_y)/10));iy<=static_cast<int>(std::floor((center_y+radius_y)/10));++iy) {
    const auto found=line_bins_.find(BinKey(ix,iy));
    if(found==line_bins_.end())continue;
    for(const auto index:found->second) {
    const auto& line=protected_lines_[index];
    if(!all_solids&&!line.legacy_boundary)continue;
    const double paint_pad=all_solids?line.half_width:0.0;
    const double xmin=all_solids?line.xmin:std::min(line.a.x,line.b.x);
    const double xmax=all_solids?line.xmax:std::max(line.a.x,line.b.x);
    const double ymin=all_solids?line.ymin:std::min(line.a.y,line.b.y);
    const double ymax=all_solids?line.ymax:std::max(line.a.y,line.b.y);
    if(xmax<center_x-radius_x||xmin>center_x+radius_x||
       ymax<center_y-radius_y||ymin>center_y+radius_y)continue;
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
    if(clip(ax,bx,-rear-paint_pad,front+paint_pad)&&
       clip(ay,by,-half-paint_pad,half+paint_pad))return true;
  }
  }
  return false;
}

bool AvoidRoadMap::PoseClear(double x,double y,double yaw,const VehicleShape& shape) const {
  return !enabled_ || (DrivablePoseClear(x,y,yaw,shape) && (paint_exempt_||!TouchesPaint(x,y,yaw,shape)));
}

AvoidRoadMap::PathEvaluation AvoidRoadMap::EvaluatePath(
    const CartesianPath& path,const VehicleShape& shape,double dt,bool budget_paint,
    double prior_contact_s) const {
  return Evaluate(path,shape,dt,0.0,0.0,false,0.0,false,5.0,budget_paint,prior_contact_s);
}

AvoidRoadMap::PathEvaluation AvoidRoadMap::EvaluateRecoveryPath(
    const CartesianPath& path,const VehicleShape& shape,double dt,double budget,double overlap_limit,bool continuing_recovery,double settled_travel_m,bool budget_paint,double prior_contact_s) const {
  if(!std::isfinite(budget)||budget<0||!std::isfinite(overlap_limit)||overlap_limit<0||
     !std::isfinite(settled_travel_m)||settled_travel_m<0||
     !std::isfinite(prior_contact_s)||prior_contact_s<0)return {};
  return Evaluate(path,shape,dt,budget,std::min(.35,overlap_limit),continuing_recovery,settled_travel_m,false,5.0,budget_paint,prior_contact_s);
}

AvoidRoadMap::PathEvaluation AvoidRoadMap::EvaluateMergeBoundaryBridge(
    const CartesianPath& path,const VehicleShape& shape,double dt,double distance,
    double time,double settled_travel) const {
  if(!std::isfinite(distance)||!std::isfinite(time)||distance<=0||time<=0)return {};
  return Evaluate(path,shape,dt,std::min(6.0,distance),.15,true,
                  settled_travel,true,std::min(5.0,time));
}

AvoidRoadMap::PathEvaluation AvoidRoadMap::Evaluate(
    const CartesianPath& path,const VehicleShape& shape,double dt,double recovery_distance_m,double overlap_limit_m,bool continuing_recovery,double settled_travel_m,bool allow_initial_excursion,double time_budget_s,bool budget_paint,double prior_contact_s) const {
  PathEvaluation result;
  if(path.x.empty()||path.y.size()!=path.x.size()||path.yaw.size()!=path.x.size()||
     !std::isfinite(dt)||dt<=0||!std::isfinite(prior_contact_s)||prior_contact_s<0)return result;
  if(!enabled_){result.drivable=true;return result;}
  const auto initial=FootprintOverlap(path.x[0],path.y[0],path.yaw[0],shape);
  VehicleShape settled_shape=shape;
  settled_shape.width+=.10;settled_shape.length+=.10;settled_shape.rear_overhang+=.05;
  const bool recovery=initial.fraction>0 || continuing_recovery || allow_initial_excursion;
  if(recovery&&(recovery_distance_m<=0||initial.maximum>overlap_limit_m||initial.fraction>.10))return result;
  result.initial_overlap_m=initial.maximum;
  double continuous=0,travel=0,elapsed_total=0,settled_travel=0;
  // The referee penalizes a wheel on solid paint after 3 s. The planner uses
  // the larger body footprint and reserves 0.5 s for tracking/sampling error.
  constexpr double kAvoidContinuousPaintLimitS=2.5;
  // Credit actual settled travel from this episode only while the enlarged
  // body is still inside. Replanning must not restart the 0.5 m confirmation,
  // nor renew the original distance/overlap limits.
  if(continuing_recovery&&DrivablePoseClear(path.x[0],path.y[0],path.yaw[0],settled_shape))
    settled_travel=std::min(.5,settled_travel_m);
  double previous_x=path.x[0],previous_y=path.y[0];
  bool recovered=!recovery||settled_travel>=.5;
  bool previous_contact=false;
  bool first_sample=true,initial_contact=false,left_initial_contact=false;
  bool reentered_paint=false;
  auto sample=[&](double x,double y,double yaw,double elapsed){
    if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw))return false;
    const double movement=std::hypot(x-previous_x,y-previous_y);
    travel+=movement;elapsed_total+=elapsed;
    previous_x=x;previous_y=y;
    if(!DrivablePoseClear(x,y,yaw,shape)){
      if(recovered)return false; // no fresh departure after settled recovery
      settled_travel=0;
      const auto overlap=FootprintOverlap(x,y,yaw,shape);
      if(overlap.maximum>((continuing_recovery||allow_initial_excursion)?overlap_limit_m:std::min(overlap_limit_m,initial.maximum+.10))||overlap.fraction>.10||
         travel>recovery_distance_m||elapsed_total>time_budget_s)return false;
      result.maximum_overlap_m=std::max(result.maximum_overlap_m,overlap.maximum);
      result.recovery_seconds=elapsed_total;
    }else {
      if(!recovered&&(travel>recovery_distance_m||elapsed_total>time_budget_s))return false;
      if(DrivablePoseClear(x,y,yaw,settled_shape))settled_travel+=movement;
      else settled_travel=0;
      if(settled_travel>=.5)recovered=true;
      if(!recovered)result.recovery_seconds=elapsed_total;
    }
    const bool contact=!paint_exempt_&&TouchesPaint(x,y,yaw,shape,budget_paint);
    if(first_sample){
      initial_contact=contact;
      first_sample=false;
      if(budget_paint&&contact)continuous=prior_contact_s;
    }else if(initial_contact){
      if(!contact)left_initial_contact=true;
      else if(left_initial_contact)reentered_paint=true;
    }
    // Conservatively count the interval touching either endpoint. Dense samples
    // prevent skipping a line between trajectory points; duplicates cost no time.
    if(contact||previous_contact){
      result.paint_contact_seconds+=elapsed;
      continuous+=elapsed;
      result.longest_paint_contact_seconds=std::max(result.longest_paint_contact_seconds,continuous);
    }
    if(!contact)continuous=0;
    previous_contact=contact;
    return true;
  };
  if(!sample(path.x[0],path.y[0],path.yaw[0],0))return result;
  for(std::size_t i=1;i<path.x.size();++i){
    if(!std::isfinite(path.x[i])||!std::isfinite(path.y[i])||
       !std::isfinite(path.yaw[i]))return result;
    const double angle=std::remainder(path.yaw[i]-path.yaw[i-1],2*M_PI);
    const double subdivisions=std::ceil(std::max(
      std::hypot(path.x[i]-path.x[i-1],path.y[i]-path.y[i-1])/.20,std::fabs(angle)/.02));
    if(!std::isfinite(subdivisions)||subdivisions>100000)return result;
    const int count=std::max(1,static_cast<int>(subdivisions));
    for(int j=1;j<=count;++j){const double f=double(j)/count;
      if(!sample(path.x[i-1]+f*(path.x[i]-path.x[i-1]),path.y[i-1]+f*(path.y[i]-path.y[i-1]),
                 path.yaw[i-1]+f*angle,dt/count))return result;
    }
  }
  const bool over_budget=budget_paint&&
      result.longest_paint_contact_seconds>=kAvoidContinuousPaintLimitS;
  // Once actual contact has already consumed the planning reserve, refusing
  // every exit would hold the body on paint indefinitely. Keep only a path
  // which clears this same episode and stays clear; PaintCost makes it lose to
  // every compliant path. Road and collision checks still apply at the caller.
  result.best_effort_paint_exit=over_budget&&prior_contact_s>0&&
      initial_contact&&left_initial_contact&&!reentered_paint&&!previous_contact;
  result.drivable=recovered&&(!over_budget||result.best_effort_paint_exit);
  return result;
}

bool AvoidRoadMap::PoseTouchesSolidPaint(double x,double y,double yaw,
                                         const VehicleShape& shape) const {
  return enabled_&&!paint_exempt_&&TouchesPaint(x,y,yaw,shape,true);
}

bool AvoidRoadMap::PathClear(const CartesianPath& path,const VehicleShape& shape) const {
  return EvaluatePath(path,shape,.1).drivable;
}

double AvoidRoadMap::PaintCost(const PathEvaluation& evaluation) {
  // Prefer no contact; duration-limited AVOID paths are still possible when a
  // strict no-contact route would leave the vehicle stopped in the corridor.
  const double prolonged=std::max(0.0,evaluation.longest_paint_contact_seconds-2.5);
  return (evaluation.best_effort_paint_exit?1e6:0.0)+
      20.0*evaluation.paint_contact_seconds+100.0*prolonged*prolonged
      +100.0*evaluation.maximum_overlap_m+20.0*evaluation.recovery_seconds;
}
