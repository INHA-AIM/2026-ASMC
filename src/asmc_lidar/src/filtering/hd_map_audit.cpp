#include <asmc_lidar/filtering/hd_map_audit.hpp>
#include <opencv2/imgproc.hpp>
#include <openssl/sha.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
namespace asmc::lidar::filtering {
namespace {
const std::array<std::string, 3> kKinds{{"static_obstacle", "dynamic_obstacle", "roundabout"}};
std::string Hash(const unsigned char* bytes, std::size_t size) {
  unsigned char d[SHA256_DIGEST_LENGTH];
  SHA256(bytes, size, d);
  std::ostringstream out;
  for (auto b : d) out << std::hex << std::setw(2) << std::setfill('0') << unsigned(b);
  return out.str();
}
std::string Hash(const std::string& s) { return Hash(reinterpret_cast<const unsigned char*>(s.data()),s.size()); }
void Require(bool ok, const char* reason) { if (!ok) throw std::runtime_error(reason); }
void Str(std::ostream& out, const std::string& s) {
  out << '"';
  for (unsigned char c : s) {
    if (c == '"' || c == '\\') out << '\\' << c;
    else if (c < 32) out << "\\u00" << "0123456789abcdef"[c >> 4] << "0123456789abcdef"[c & 15];
    else out << c;
  }
  out << '"';
}
void Strings(std::ostream& out, const std::vector<std::string>& v) {
  out << '[';
  for (std::size_t i=0; i<v.size(); ++i) { if (i) out << ','; Str(out, v[i]); }
  out << ']';
}
void Bool(std::ostream& out, bool b) { out << (b ? "true" : "false"); }
}
std::string FileSha256(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  Require(bool(in), "audit_file_unreadable");
  SHA256_CTX state; SHA256_Init(&state);
  char buffer[65536];
  while (in) { in.read(buffer, sizeof(buffer)); SHA256_Update(&state, buffer, in.gcount()); }
  Require(in.eof(), "audit_file_read_error");
  unsigned char d[SHA256_DIGEST_LENGTH]; SHA256_Final(d, &state);
  std::ostringstream out;
  for (auto b : d) out << std::hex << std::setw(2) << std::setfill('0') << unsigned(b);
  return out.str();
}
struct HdMapAudit::Impl {
  std::string map_id, map_sha, bundle_sha, policy_sha;
  std::array<double,3> map_origin{};
  double res = 0, ox = 0, oy = 0, age_limit = 0, guard = 0, exception_guard = 0;
  cv::Mat occupancy, exceptions;
  SpatialRelation Relation(const std::vector<cv::Point2d>& hull, double expansion) const {
    SpatialRelation r;
    if (!std::isfinite(expansion) || expansion < 0 || expansion > 20) {
      r.reason="uncertainty_window_out_of_range"; return r;
    }
    std::vector<cv::Point2d> px;
    double minx=INFINITY,miny=INFINITY,maxx=-INFINITY,maxy=-INFINITY;
    for (const auto& p:hull) {
      px.emplace_back((p.x-ox)/res, (p.y-oy)/res);
      minx=std::min(minx,px.back().x); miny=std::min(miny,px.back().y);
      maxx=std::max(maxx,px.back().x); maxy=std::max(maxy,px.back().y);
    }
    // Check in double before integer conversion, including absurd coordinates.
    const double lx=std::floor(minx-1), ly=std::floor(miny-1);
    const double hx=std::ceil(maxx+1)+1, hy=std::ceil(maxy+1)+1;
    const int pad=int(std::ceil(expansion/res))+1;
    if (lx-pad<0 || ly-pad<0 || hx+pad>occupancy.cols || hy+pad>occupancy.rows) {
      r.reason="map_extent_or_uncertainty_outside"; return r;
    }
    if ((hx-lx+2*pad)*(hy-ly+2*pad)>1000000) {r.reason="footprint_window_too_large"; return r;}
    const int x=int(lx), y=int(ly), w=int(hx-lx), h=int(hy-ly);
    cv::Mat mask=cv::Mat::zeros(h,w,CV_8U);
    std::vector<cv::Point> polygon;
    // NumPy rint uses ties-to-even; nearbyint uses the default FE_TONEAREST too.
    for (const auto& p:px) polygon.emplace_back(int(std::nearbyint(p.x-x)),int(std::nearbyint(p.y-y)));
    cv::fillConvexPoly(mask,polygon,cv::Scalar(1));
    int total=0, road=0;
    for(int row=0;row<h;++row) {
      const auto* m=mask.ptr<std::uint8_t>(row);
      const auto* o=occupancy.ptr<std::int8_t>(y+row)+x;
      for(int col=0;col<w;++col) if(m[col]) {++total; road += o[col]==0;}
    }
    if(!total) {r.reason="empty_raster_footprint"; return r;}
    std::array<bool,3> kinds{};
    for(int row=y-pad;row<y+h+pad;++row) {
      const auto* o=occupancy.ptr<std::int8_t>(row);
      const auto* e=exceptions.ptr<std::uint8_t>(row);
      for(int col=x-pad;col<x+w+pad;++col) {
        r.boundary_protected |= o[col]==0;
        r.unknown_cells |= o[col]==-1;
        if(e[col]) kinds[e[col]-1]=true;
      }
    }
    r.available=true; r.road_ratio_grid=double(road)/total; r.raster_resolution_m=res;
    for(std::size_t i=0;i<kinds.size();++i) if(kinds[i]) r.exception_types.push_back(kKinds[i]);
    return r;
  }
};
HdMapAudit::HdMapAudit(const std::string& path,const std::string& expected) {
  auto p=std::make_shared<Impl>();
  Require(std::filesystem::file_size(path) <= 8*1024*1024,"audit_manifest_too_large");
  std::ifstream file(path,std::ios::binary);
  Require(bool(file),"audit_manifest_unreadable");
  std::ostringstream content;content << file.rdbuf();const auto bytes=content.str();
  p->bundle_sha=Hash(bytes);
  Require(expected.size()==64 && p->bundle_sha==expected,"audit_manifest_sha_mismatch");
  const auto m=YAML::Load(bytes);
  Require(m["schema_version"].as<std::string>()=="asmc.lidar.hd_map_audit_bundle/v1","audit_schema_mismatch");
  Require(m["frame_id"].as<std::string>()=="map","audit_frame_mismatch");
  p->map_id=m["map_id"].as<std::string>();
  p->map_sha=m["source_sha256"]["product"].as<std::string>();
  p->policy_sha=m["policy_sha256"].as<std::string>();
  const auto canonical=m["policy_canonical"].as<std::string>();
  Require(Hash(canonical)==p->policy_sha,"audit_policy_sha_mismatch");
  // Parse the hashed policy bytes, not an independently editable YAML copy.
  const auto policy=YAML::Load(canonical);
  Require(policy["expected_map_id"].as<std::string>()==p->map_id,"audit_policy_map_mismatch");
  Require(p->map_sha.size()==64,"audit_source_sha_invalid");
  const auto origin=m["map_origin"].as<std::vector<double>>();
  const auto po=policy["map_origin"].as<std::vector<double>>();
  Require(origin.size()==3 && origin==po,"audit_map_origin_mismatch");
  std::copy(origin.begin(),origin.end(),p->map_origin.begin());
  const auto xy=m["origin_xy"].as<std::vector<double>>();
  Require(xy.size()==2,"audit_grid_origin_invalid"); p->ox=xy[0];p->oy=xy[1];
  p->res=m["resolution_m"].as<double>();
  p->age_limit=policy["pose_age_limit_ms"].as<double>();
  p->guard=policy["candidate_guard_m"].as<double>();
  p->exception_guard=policy["exception_protection_m"].as<double>();
  Require(std::isfinite(p->res)&&p->res>=0.01&&p->res<=10&&std::isfinite(p->ox)&&std::isfinite(p->oy),"audit_grid_geometry_invalid");
  for(double v:p->map_origin) Require(std::isfinite(v),"audit_map_origin_invalid");
  Require(std::isfinite(p->age_limit)&&p->age_limit>0&&p->age_limit<=300&&
          std::isfinite(p->guard)&&p->guard>=0&&p->guard<=20&&
          std::isfinite(p->exception_guard)&&p->exception_guard>=0&&p->exception_guard<=20,"audit_policy_invalid");
  const int h=m["height"].as<int>(),w=m["width"].as<int>();
  Require(h>0&&w>0&&std::int64_t(h)*w<=100000000,"audit_grid_size_invalid");
  auto load=[&](const char* name,int type) {
    const auto spec=m["arrays"][name];
    const auto file=spec["file"].as<std::string>();
    Require(!file.empty()&&std::filesystem::path(file).filename()==file,"audit_array_path_invalid");
    const auto full=(std::filesystem::path(path).parent_path()/file).string();
    Require(std::filesystem::file_size(full)==std::uint64_t(h)*w&&spec["bytes"].as<std::uint64_t>()==std::uint64_t(h)*w,"audit_array_size_mismatch");
    cv::Mat out(h,w,type);
    std::ifstream in(full,std::ios::binary);in.read(reinterpret_cast<char*>(out.data),std::int64_t(h)*w);
    Require(bool(in),"audit_array_read_error");
    Require(Hash(out.data,out.total())==spec["sha256"].as<std::string>(),"audit_array_sha_mismatch");
    return out;
  };
  p->occupancy=load("occupancy",CV_8S);p->exceptions=load("exceptions",CV_8U);
  for(int y=0;y<h;++y) for(int x=0;x<w;++x) {
    const auto o=p->occupancy.at<std::int8_t>(y,x);const auto e=p->exceptions.at<std::uint8_t>(y,x);
    Require(o==-1||o==0||o==100,"audit_occupancy_invalid"); Require(e<=3,"audit_exception_invalid");
  }
  impl_=std::move(p);
}
const std::string& HdMapAudit::map_id()const{return impl_->map_id;}
const std::string& HdMapAudit::map_sha256()const{return impl_->map_sha;}
const std::string& HdMapAudit::bundle_sha256()const{return impl_->bundle_sha;}
const std::string& HdMapAudit::policy_sha256()const{return impl_->policy_sha;}
AuditDecision HdMapAudit::Evaluate(const AuditFrame& f,const AuditObject& o) const {
  const auto start=std::chrono::steady_clock::now();AuditDecision r;r.track_id=o.track_id;r.input_state=o.observation_state;
  auto finish=[&](const char* d,const std::string& why) {
    r.decision=d;r.reason=why;
    r.comparison_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();return r;
  };
  if(f.frame_id!="map"||(f.status!=1&&f.status!=2))return finish("undetermined","map_frame_or_array_status_unavailable");
  if(!f.pose_valid)return finish("undetermined","pose_invalid_or_missing");
  if(f.map_id!=impl_->map_id)return finish("undetermined","map_id_unverified");
  for(int i=0;i<3;++i) if(!std::isfinite(f.map_origin[i])||std::abs(f.map_origin[i]-impl_->map_origin[i])>1e-6)return finish("undetermined","map_origin_unverified");
  if(f.stamp_ns<=0||f.pose_stamp_ns!=f.stamp_ns)return finish("undetermined","pose_scan_stamp_mismatch");
  if(o.corners.size()!=8)return finish("undetermined","eight_finite_corners_required");
  std::vector<cv::Point2f> points;
  for(const auto& p:o.corners) {
    for(double v:p)if(!std::isfinite(v))return finish("undetermined","eight_finite_corners_required");
    if(std::abs(p[0])>1e7||std::abs(p[1])>1e7)return finish("undetermined","coordinate_range_invalid");
    points.emplace_back(float(p[0]),float(p[1]));
  }
  std::vector<cv::Point2f> ch;cv::convexHull(points,ch);
  if(ch.size()<3||cv::contourArea(ch)<1e-6)return finish("undetermined","degenerate_footprint");
  std::vector<cv::Point2d> hull;for(const auto& p:ch)hull.emplace_back(p.x,p.y);
  const bool verified=f.position_error_verified&&std::isfinite(f.position_error_bound_m)&&f.position_error_bound_m>=0;
  const double expansion=impl_->guard+(verified?f.position_error_bound_m:0);
  r.has_spatial=true;r.spatial=impl_->Relation(hull,expansion);
  if(!r.spatial.available)return finish("undetermined",r.spatial.reason);
  const auto exceptions=impl_->Relation(hull,std::max(expansion,impl_->exception_guard));
  if(!exceptions.available)return finish("undetermined",exceptions.reason);
  if(!exceptions.exception_types.empty()){r.exception_types=exceptions.exception_types;return finish("keep","exception_space_protected");}
  if(r.spatial.unknown_cells)return finish("undetermined","unknown_map_cells");
  if(r.spatial.road_ratio_grid>0)return finish("keep","drivable_overlap");
  if(r.spatial.boundary_protected)return finish("keep","boundary_or_uncertainty_protected");
  if(!verified)return finish("undetermined","pose_error_bound_unverified");
  if(!std::isfinite(f.source_age_ms)||f.source_age_ms<0||f.source_age_ms>impl_->age_limit)return finish("undetermined","pose_source_age_unverified_or_stale");
  if(!f.mission_scope_verified)return finish("undetermined","mission_scope_unverified");
  return finish("exclude_candidate","ordinary_space_clearly_outside");
}
void WriteAuditDecision(std::ostream& out,const AuditDecision& d) {
  out<<"{\"track_id\":"<<d.track_id<<",\"input_state\":";Str(out,d.input_state);
  out<<",\"decision\":";Str(out,d.decision);out<<",\"reason\":";Str(out,d.reason);
  out<<",\"comparison_ms\":"<<d.comparison_ms<<",\"exception_types\":";Strings(out,d.exception_types);
  out<<",\"spatial\":";
  if(!d.has_spatial)out<<"null";
  else {const auto& s=d.spatial;out<<"{\"available\":";Bool(out,s.available);
    if(!s.available){out<<",\"reason\":";Str(out,s.reason);}
    else {out<<",\"road_ratio_grid\":"<<s.road_ratio_grid<<",\"boundary_protected\":";Bool(out,s.boundary_protected);
      out<<",\"unknown_cells\":";Bool(out,s.unknown_cells);out<<",\"exception_types\":";Strings(out,s.exception_types);
      out<<",\"raster_resolution_m\":"<<s.raster_resolution_m;}
    out<<'}';}
  out<<'}';
}
void WriteAuditReport(std::ostream& out,const AuditReport& r) {
  out<<"{\"enabled\":";Bool(out,r.enabled);out<<",\"available\":";Bool(out,r.available);
  out<<",\"reason\":";Str(out,r.reason);out<<",\"map_id\":";Str(out,r.map_id);
  out<<",\"map_sha256\":";Str(out,r.map_sha256);out<<",\"bundle_sha256\":";Str(out,r.bundle_sha256);
  out<<",\"policy_sha256\":";Str(out,r.policy_sha256);
  out<<",\"map_identity_provenance\":\"pinned_bundle_and_runtime_origin_config\",\"geometry_authority\":\"vector; grid ratios are diagnostic approximation\",\"position_error_verified\":false,\"mission_scope_verified\":false,\"calculation_ms\":"<<r.calculation_ms<<",\"objects\":[";
  for(std::size_t i=0;i<r.objects.size();++i){if(i)out<<',';WriteAuditDecision(out,r.objects[i]);}out<<"]}";
}
}  // namespace asmc::lidar::filtering
