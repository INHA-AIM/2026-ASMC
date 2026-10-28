#include "Map/HdLaneMap.hpp"

// Match newer Boost's unscaled overlay semantics for local footprint clips.
// The legacy integer rescale grid can leave a false sliver at a shared edge.
// This switch does not disable any source/result is_valid checks below.
#define BOOST_GEOMETRY_NO_ROBUSTNESS
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace asmc_planner::lane_reference {
namespace bg = boost::geometry;
namespace {
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;
using MultiPolygon = bg::model::multi_polygon<Polygon>;
using Line = bg::model::linestring<Point>;
using Box = bg::model::box<Point>;
struct RegionPart { Polygon polygon; Box bounds; };
RegionPart regionPart(Polygon polygon) {
  RegionPart part; part.polygon=std::move(polygon);
  bg::envelope(part.polygon,part.bounds);
  return part;
}
bool finite(const Point2D& p) { return std::isfinite(p.x) && std::isfinite(p.y); }
bool makePolygon(const std::vector<Point2D>& points, Polygon& out, std::string& error) {
  if (points.size() < 3) { error = "polygon needs three vertices"; return false; }
  for (const auto& p : points) {
    if (!finite(p)) { error = "non-finite polygon"; return false; }
    bg::append(out.outer(), Point(p.x, p.y));
  }
  bg::correct(out);
  // Reviewed v7 rings can contain an exact A-B-A excursion with zero area.
  // Remove only those exact retraced edges in this derived representation;
  // do not buffer, snap or repair a crossing/overlapping polygon.
  auto& ring=out.outer();
  bool changed=true;
  while(changed && ring.size()>4) {
    changed=false;
    const std::size_t n=ring.size()-1;
    for(std::size_t i=0;i<n;++i) {
      const auto& a=ring[(i+n-1)%n];const auto& b=ring[(i+1)%n];
      if(bg::get<0>(a)!=bg::get<0>(b) || bg::get<1>(a)!=bg::get<1>(b))continue;
      std::vector<Point> next;
      for(std::size_t j=0;j<n;++j)if(j!=i && j!=(i+1)%n)next.push_back(ring[j]);
      if(next.size()<3)break;
      next.push_back(next.front());ring.assign(next.begin(),next.end());
      changed=true;break;
    }
  }
  bg::correct(out);
  // A pinched inner loop is an interior ring, not additional drivable area.
  // Split only at an exact repeated source vertex, preserve its edges as a
  // hole and require the resulting topology to pass all validity checks.
  if(!bg::is_valid(out)) {
    const auto original=out.outer();const std::size_t n=original.size()-1;
    bool extracted=false;
    for(std::size_t i=0;i<n && !extracted;++i)for(std::size_t j=i+2;j<n && !extracted;++j) {
      if(bg::get<0>(original[i])!=bg::get<0>(original[j]) ||
         bg::get<1>(original[i])!=bg::get<1>(original[j]))continue;
      Polygon rest,loop;
      for(std::size_t k=0;k<=i;++k)rest.outer().push_back(original[k]);
      for(std::size_t k=j+1;k<original.size();++k)rest.outer().push_back(original[k]);
      for(std::size_t k=i;k<=j;++k)loop.outer().push_back(original[k]);
      if(bg::area(rest)*bg::area(loop)>=0)continue;
      bg::correct(rest);bg::correct(loop);
      if(!bg::is_valid(rest)||!bg::is_valid(loop)||!bg::covered_by(loop,rest))continue;
      rest.inners().push_back(loop.outer());bg::correct(rest);
      if(!bg::is_valid(rest))continue;
      out=std::move(rest);extracted=true;
    }
  }
  if (!bg::is_valid(out, error) || std::abs(bg::area(out)) <= 1e-9) return false;
  return true;
}
Polygon footprint(double x, double y, double yaw, const FootprintShape& s, double margin) {
  Polygon p;
  const double c = std::cos(yaw), sn = std::sin(yaw);
  for (const auto& q : std::vector<Point2D>{{-s.rear_m-margin,-s.half_width_m-margin},
       {s.front_m+margin,-s.half_width_m-margin}, {s.front_m+margin,s.half_width_m+margin},
       {-s.rear_m-margin,s.half_width_m+margin}}) {
    bg::append(p.outer(), Point(x+c*q.x-sn*q.y, y+sn*q.x+c*q.y));
  }
  bg::correct(p);
  return p;
}
bool validPose(double x, double y, double yaw, const FootprintShape& shape, double margin) {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(yaw) &&
         std::isfinite(margin) && margin >= 0.0 && ValidFootprint(shape);
}
}  // namespace

// Keep the validated source polygons. Boost 1.71 can create self-intersecting
// holes when repeatedly unioning the complete map; a materialized global union
// is unnecessary for a set-coverage query over a vehicle footprint.
struct LaneRegion::Impl { std::vector<RegionPart> parts; };
struct HdLaneMap::Impl {
  LaneMapData data;
  std::unordered_map<std::string, std::size_t> by_id;
  std::unordered_map<std::string, Polygon> polygons;
  std::unordered_map<std::string, Box> lane_bounds;
  struct Paint { std::string id; double half_width; Line line; Box bounds; };
  std::vector<Paint> solid;
  std::shared_ptr<const LaneRegion> road;
};

bool ValidFootprint(const FootprintShape& s) {
  return std::isfinite(s.front_m) && std::isfinite(s.rear_m) && std::isfinite(s.half_width_m) &&
         s.front_m > 0.0 && s.rear_m >= 0.0 && s.half_width_m > 0.0;
}
bool MapIdentity::valid() const {
  return frame_id == "map" && !build_id.empty() && !map_id.empty() &&
    geometry_sha256.size() == 64 && std::all_of(geometry_sha256.begin(), geometry_sha256.end(),
      [](char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'); });
}
bool MapIdentity::operator==(const MapIdentity& o) const {
  return frame_id==o.frame_id && build_id==o.build_id && map_id==o.map_id && geometry_sha256==o.geometry_sha256;
}
bool LaneRegion::coversPoint(double x, double y) const {
  if (!impl_ || !std::isfinite(x) || !std::isfinite(y)) return false;
  const Point p(x,y);
  for (const auto& part:impl_->parts)
    if (bg::covered_by(p,part.bounds) && bg::covered_by(p,part.polygon)) return true;
  return false;
}
bool LaneRegion::intersectsPolygon(const std::vector<Point2D>& points) const {
  Polygon polygon;std::string error;
  if (!impl_ || !makePolygon(points,polygon,error)) return true;
  Box bounds;bg::envelope(polygon,bounds);
  for (const auto& part:impl_->parts)
    if (bg::intersects(bounds,part.bounds) && bg::intersects(polygon,part.polygon)) return true;
  return false;
}
bool LaneRegion::coversFootprint(double x, double y, double yaw,
                                const FootprintShape& shape, double margin) const {
  if (!impl_ || !validPose(x,y,yaw,shape,margin)) return false;
  const Polygon car=footprint(x,y,yaw,shape,margin);
  if (!bg::is_valid(car)) return false;
  Box bounds; bg::envelope(car,bounds);
  std::vector<const RegionPart*> nearby;
  for (const auto& part:impl_->parts) {
    if (!bg::intersects(bounds,part.bounds)) continue;
    if (bg::covered_by(car,part.polygon)) return true;
    nearby.push_back(&part);
  }
  // Only polygons intersecting this footprint's bounds can cover it. Combine
  // their original edges before containment testing, rather than repeatedly
  // subtracting from the car (which can leave rounded intersection slivers).
  // Holes/gaps remain present; no buffer or area tolerance is applied.
  struct Cache {
    std::weak_ptr<const Impl> owner;
    std::vector<const RegionPart*> parts;
    MultiPolygon polygons;
    bool valid=false;
  };
  // One derived region per thread. Reuse depends on the immutable owner and
  // exact part set, never on a previous vehicle pose or containment verdict.
  static thread_local Cache cache;
  if (cache.owner.lock()==impl_ && cache.parts==nearby)
    return cache.valid && bg::covered_by(car,cache.polygons);
  MultiPolygon local;
  bool valid=!nearby.empty();
  for (const auto* part:nearby) {
    if (local.empty()) { local.push_back(part->polygon); continue; }
    MultiPolygon next;
    bg::union_(local,part->polygon,next);
    if (next.empty() || !bg::is_valid(next)) { valid=false; break; }
    local=std::move(next);
  }
  cache.owner=impl_;cache.parts=std::move(nearby);
  cache.polygons=std::move(local);cache.valid=valid;
  return cache.valid && bg::covered_by(car,cache.polygons);
}

bool LaneRegion::coversPolygon(const std::vector<Point2D>& points) const {
  Polygon polygon;std::string error;
  if(!impl_||!makePolygon(points,polygon,error))return false;
  Box bounds;bg::envelope(polygon,bounds);
  MultiPolygon remaining;remaining.push_back(polygon);
  for(const auto& part:impl_->parts) {
    if(!bg::intersects(bounds,part.bounds))continue;
    if(bg::covered_by(polygon,part.polygon))return true;
    MultiPolygon next;
    for(const auto& piece:remaining) {
      MultiPolygon difference;bg::difference(piece,part.polygon,difference);
      if(!bg::is_valid(difference))return false;
      next.insert(next.end(),difference.begin(),difference.end());
    }
    remaining=std::move(next);
    if(remaining.empty())return true;
  }
  return false;
}

std::shared_ptr<const HdLaneMap> HdLaneMap::create(LaneMapData data, std::string& error) {
  error.clear();
  if (!data.complete || !data.identity.valid() || data.lanes.empty() || data.areas.empty()) {
    error="incomplete or invalid global map identity/layers"; return {};
  }
  auto impl=std::make_shared<Impl>();
  impl->data=std::move(data);
  for (std::size_t i=0;i<impl->data.lanes.size();++i) {
    const auto& l=impl->data.lanes[i];
    if (l.id.empty() || l.link_id.empty() || !impl->by_id.emplace(l.id,i).second || l.centerline.size()<2) {
      error="duplicate/empty lane identity or missing centerline"; return {};
    }
    double length=0.0;
    for (std::size_t j=0;j<l.centerline.size();++j) {
      if (!finite(l.centerline[j])) { error="non-finite centerline: "+l.id; return {}; }
      if (j) length+=std::hypot(l.centerline[j].x-l.centerline[j-1].x,l.centerline[j].y-l.centerline[j-1].y);
    }
    Polygon polygon;
    if (!std::isfinite(length) || length<=1e-6 || !makePolygon(l.polygon,polygon,error)) { error="invalid lane "+l.id+": "+error; return {}; }
    Box bounds;bg::envelope(polygon,bounds);impl->lane_bounds.emplace(l.id,bounds);
    impl->polygons.emplace(l.id,std::move(polygon));
  }
  for (const auto& l:impl->data.lanes) {
    auto check=[&](const std::string& id) { return id.empty() || impl->by_id.count(id); };
    if (!check(l.left_adjacent) || !check(l.right_adjacent)) { error="dangling adjacency: "+l.id; return {}; }
    for (const auto& id:l.predecessors) if (id.empty() || !check(id)) { error="dangling predecessor: "+l.id; return {}; }
    for (const auto& id:l.successors) if (id.empty() || !check(id)) { error="dangling successor: "+l.id; return {}; }
  }
  for (const auto& l:impl->data.lanes) {
    if ((!l.left_adjacent.empty() && impl->data.lanes[impl->by_id.at(l.left_adjacent)].right_adjacent!=l.id) ||
        (!l.right_adjacent.empty() && impl->data.lanes[impl->by_id.at(l.right_adjacent)].left_adjacent!=l.id)) {
      error="non-reciprocal adjacency: "+l.id; return {};
    }
    for (const auto& id:l.successors) {
      const auto& next=impl->data.lanes[impl->by_id.at(id)].predecessors;
      if (std::find(next.begin(),next.end(),l.id)==next.end()) { error="non-reciprocal successor: "+l.id; return {}; }
    }
    for (const auto& id:l.predecessors) {
      const auto& previous=impl->data.lanes[impl->by_id.at(id)].successors;
      if (std::find(previous.begin(),previous.end(),l.id)==previous.end()) { error="non-reciprocal predecessor: "+l.id; return {}; }
    }
  }
  auto region=std::make_shared<LaneRegion::Impl>();
  std::unordered_set<std::string> area_ids;
  for (const auto& a:impl->data.areas) {
    Polygon polygon;
    if (a.id.empty() || !area_ids.insert(a.id).second || !a.validated || !makePolygon(a.polygon,polygon,error)) {
      error="invalid or provisional drivable area "+a.id+": "+error; return {};
    }
    region->parts.push_back(regionPart(std::move(polygon)));
  }
  impl->road=std::shared_ptr<const LaneRegion>(new LaneRegion(std::move(region)));
  std::unordered_set<std::string> marking_ids;
  for (const auto& m:impl->data.markings) {
    if (m.id.empty() || !marking_ids.insert(m.id).second || m.points.size()<2 ||
        !std::isfinite(m.width_m) || m.width_m<0.0 ||
        !std::all_of(m.points.begin(),m.points.end(),finite)) { error="invalid marking: "+m.id; return {}; }
    if (m.pattern!=PaintPattern::SOLID || m.stop_line || m.virtual_line) continue;
    Impl::Paint paint; paint.id=m.id; paint.half_width=m.width_m*.5;
    for (const auto& p:m.points) paint.line.emplace_back(p.x,p.y);
    bg::envelope(paint.line,paint.bounds);
    impl->solid.push_back(std::move(paint));
  }
  error.clear();
  return std::shared_ptr<const HdLaneMap>(new HdLaneMap(std::move(impl)));
}
const MapIdentity& HdLaneMap::identity() const { return impl_->data.identity; }
const LaneData* HdLaneMap::lane(const std::string& id) const {
  const auto i=impl_->by_id.find(id);
  return i==impl_->by_id.end()?nullptr:&impl_->data.lanes[i->second];
}
std::optional<double> HdLaneMap::laneHeading(double x,double y,double vx,double vy) const {
  const double speed=std::hypot(vx,vy);
  if(!impl_||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(speed)||speed<.5)return {};
  const Point point(x,y);double best=INFINITY;std::optional<double> heading;
  for(const auto& lane:impl_->data.lanes) {
    if(!lane.traversable||!bg::covered_by(point,impl_->lane_bounds.at(lane.id))||
       !bg::covered_by(point,impl_->polygons.at(lane.id)))continue;
    for(std::size_t i=1;i<lane.centerline.size();++i) {
      const auto a=lane.centerline[i-1],b=lane.centerline[i];
      const double dx=b.x-a.x,dy=b.y-a.y,length=std::hypot(dx,dy);
      if(length<1e-6||(vx*dx+vy*dy)/(speed*length)<.5)continue;
      const double u=std::clamp(((x-a.x)*dx+(y-a.y)*dy)/(length*length),0.0,1.0);
      const double distance=std::hypot(x-a.x-u*dx,y-a.y-u*dy);
      if(distance<best){best=distance;heading=std::atan2(dy,dx);}
    }
  }
  return heading;
}
std::size_t HdLaneMap::laneCount() const { return impl_->data.lanes.size(); }
std::shared_ptr<const LaneRegion> HdLaneMap::road() const { return impl_->road; }
std::shared_ptr<const LaneRegion> HdLaneMap::corridor(const std::vector<std::string>& ids,std::string& error) const {
  error.clear();
  if (ids.empty()) { error="empty corridor lane sequence"; return {}; }
  auto region=std::make_shared<LaneRegion::Impl>();
  for (const auto& id:ids) {
    const auto it=impl_->polygons.find(id);
    if (it==impl_->polygons.end()) { error="unknown corridor lane: "+id; return {}; }
    region->parts.push_back(regionPart(it->second));
  }
  return std::shared_ptr<const LaneRegion>(new LaneRegion(std::move(region)));
}
bool HdLaneMap::touchesSolid(double x,double y,double yaw,const FootprintShape& shape,
                            double margin,std::string* id) const {
  if (!validPose(x,y,yaw,shape,margin)) return true;
  const auto car=footprint(x,y,yaw,shape,margin);
  Box bounds; bg::envelope(car,bounds);
  for (const auto& paint:impl_->solid) {
    Box expanded=paint.bounds;
    bg::set<bg::min_corner,0>(expanded,bg::get<bg::min_corner,0>(expanded)-paint.half_width);
    bg::set<bg::min_corner,1>(expanded,bg::get<bg::min_corner,1>(expanded)-paint.half_width);
    bg::set<bg::max_corner,0>(expanded,bg::get<bg::max_corner,0>(expanded)+paint.half_width);
    bg::set<bg::max_corner,1>(expanded,bg::get<bg::max_corner,1>(expanded)+paint.half_width);
    if (bg::intersects(bounds,expanded) && bg::distance(car,paint.line)<=paint.half_width+1e-9) {
      if (id) *id=paint.id;
      return true;
    }
  }
  return false;
}
}  // namespace asmc_planner::lane_reference
