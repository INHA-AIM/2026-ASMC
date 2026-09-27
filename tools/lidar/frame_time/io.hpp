#pragma once
#include "core.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <variant>

namespace asmc::lidar_eval {
// Small output-only JSON value. YAML-cpp reads both JSONL and YAML fixtures;
// writing YAML flow syntax would not produce valid JSON, hence explicit JSON.
struct Json {
  using Array=std::vector<Json>; using Map=std::map<std::string,Json>;
  std::variant<std::nullptr_t,bool,Ns,double,std::string,Array,Map> value=nullptr;
  Json()=default; Json(bool v):value(v){} Json(Ns v):value(v){} Json(int v):value(Ns(v)){}
  Json(double v):value(v){} Json(const char* v):value(std::string(v)){}
  Json(std::string v):value(std::move(v)){} Json(Array v):value(std::move(v)){}
  Json(Map v):value(std::move(v)){}
  Json& operator[](const std::string& key) {
    if (!std::holds_alternative<Map>(value)) value=Map{};
    return std::get<Map>(value)[key];
  }
  static std::string quote(const std::string& s) {
    std::ostringstream out; out << '"';
    for (unsigned char c:s) {
      if (c=='"' || c=='\\') out << '\\' << char(c);
      else if(c<32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
      else out << char(c);
    }
    return out.str()+'"';
  }
  std::string dump() const {
    std::ostringstream out; out << std::setprecision(17);
    std::visit([&](auto&& v){
      using T=std::decay_t<decltype(v)>;
      if constexpr(std::is_same_v<T,std::nullptr_t>) out << "null";
      else if constexpr(std::is_same_v<T,bool>) out << (v?"true":"false");
      else if constexpr(std::is_same_v<T,std::string>) out << quote(v);
      else if constexpr(std::is_same_v<T,Array>) {
        out << '['; bool first=true; for(const auto& x:v){if(!first)out<<',';first=false;out<<x.dump();} out<<']';
      } else if constexpr(std::is_same_v<T,Map>) {
        out << '{'; bool first=true; for(const auto& [k,x]:v){if(!first)out<<',';first=false;out<<quote(k)<<':'<<x.dump();} out<<'}';
      } else if constexpr(std::is_same_v<T,double>) { if(std::isfinite(v))out<<v;else out<<"null"; }
      else out<<v;
    },value);
    return out.str();
  }
};
inline Json vector_json(const Vec& p) {return Json::Array{p.x(),p.y(),p.z()};}
inline Json numbers_json(const std::vector<Ns>& v) {Json::Array a;for(auto x:v)a.emplace_back(x);return a;}
inline Vec vec(const YAML::Node& n) {
  if(!n.IsSequence() || n.size()!=3) throw std::runtime_error("expected xyz[3]");
  return Vec(n[0].as<double>(),n[1].as<double>(),n[2].as<double>());
}
inline Rot matrix(const YAML::Node& n) {
  if(!n.IsSequence() || n.size()!=3) throw std::runtime_error("expected rotation[3][3]");
  Rot r; for(int i=0;i<3;++i)r.row(i)=vec(n[i]);return r;
}
inline Pose pose(const YAML::Node& n) {
  Pose p=Pose::Identity(); p.translation()=vec(n["translation"]);
  p.linear()=matrix(n["rotation"]);return p;
}
inline Observation observation(const YAML::Node& n) {
  return {n["stamp_ns"].as<Ns>(),n["receipt_ns"].as<Ns>(),n["processing_ns"].as<Ns>(),
          n["clock"].as<std::string>(),n["stamp_provenance"].as<std::string>()};
}
inline Context context(const YAML::Node& n,PoseHistory& h) {
  Context c; c.time=observation(n["time"]);
  const auto frame=n["frame"].as<std::string>();
  if(frame=="sensor")c.frame=Frame::Sensor;
  else if(frame=="leveled_sensor")c.frame=Frame::LeveledSensor;
  else if(frame=="base_link")c.frame=Frame::Base;
  else throw std::runtime_error("unsupported input frame");
  c.calibration_source=n["calibration_source"].as<std::string>("");
  if(c.frame!=Frame::Base && !n["base_from_sensor"])throw std::runtime_error("mount transform required");
  if(c.frame==Frame::LeveledSensor && !n["applied_leveling"])throw std::runtime_error("exact leveling required");
  if(n["base_from_sensor"])c.base_from_sensor=pose(n["base_from_sensor"]);
  if(n["applied_leveling"])c.applied_leveling=matrix(n["applied_leveling"]);
  c.leveling_source=n["leveling_source"].as<std::string>("");
  std::string pose_input_error;
  for(const auto& v:n["pose_samples"]) {
    PoseSample s;
    s.stamp_ns=v["stamp_ns"].as<Ns>();s.arrival_ns=v["arrival_ns"].as<Ns>();
    s.clock=v["clock"].as<std::string>();s.valid=v["valid"].as<bool>();
    s.map_from_reference=pose(v["pose"]);
    const auto ref=v["reference"].as<std::string>();
    if(ref!="base_link" && ref!="gps_antenna")throw std::runtime_error("unknown pose origin");
    s.reference_is_gps=ref=="gps_antenna";
    if(s.reference_is_gps)s.gps_in_base=vec(v["gps_in_base"]);
    if(const auto e=h.append(s);!e.empty())pose_input_error=e;
  }
  c.map_pose=h.at(c.time);
  // Bad pose timing invalidates projection, not otherwise valid local geometry.
  // Reject the entire pose history rather than interpolate across dropped state.
  if(!pose_input_error.empty()) {
    c.map_pose.valid=false;c.map_pose.reason=pose_input_error;
    c.map_pose.sample_stamps.clear();c.map_pose.sample_arrivals.clear();
  }
  if(n["twist"]) {
    const auto v=n["twist"];c.twist_valid=v["valid"].as<bool>();
    c.twist_stamp_ns=v["stamp_ns"].as<Ns>();c.twist_clock=v["clock"].as<std::string>();
    c.twist_arrival_ns=v["arrival_ns"].as<Ns>();
    c.ego_linear_base=vec(v["linear_base"]);c.ego_angular_base=vec(v["angular_base"]);
  }
  return c;
}
inline Object object(const YAML::Node& n) {
  Object o;o.id=n["id"].as<Ns>();o.classification=n["classification"].as<std::string>();
  o.center=vec(n["center"]);o.long_axis=vec(n["long_axis"]);
  if(!n["corners"].IsSequence() || n["corners"].size()!=8)throw std::runtime_error("expected 8 OBB corners");
  for(int i=0;i<8;++i)o.corners[i]=vec(n["corners"][i]);
  const auto meaning=n["velocity_meaning"].as<std::string>();
  if(meaning=="absolute_input_vector")o.velocity_meaning=VelocityMeaning::AbsoluteInputVector;
  else if(meaning=="base_position_derivative")o.velocity_meaning=VelocityMeaning::BasePositionDerivative;
  else if(meaning!="unknown")throw std::runtime_error("unknown velocity declaration");
  if(n["velocity"])o.velocity=vec(n["velocity"]);
  else if(o.velocity_meaning!=VelocityMeaning::Unknown)throw std::runtime_error("velocity missing");
  return o;
}
inline Json geometry_json(const Geometry& g) {
  Json j;j["center"]=vector_json(g.center);j["long_axis"]=vector_json(g.long_axis);
  j["yaw_rad"]=g.yaw?Json(*g.yaw):Json();
  Json::Array a;for(const auto& p:g.corners)a.push_back(vector_json(p));j["corners"]=a;return j;
}
inline Json result_json(const Result& r) {
  Json j;j["id"]=r.id;j["classification"]=r.classification;j["valid"]=r.valid;
  j["reason"]=r.reason;j["map_reason"]=r.map_reason;j["velocity_reason"]=r.velocity_reason;
  j["stamp_ns"]=r.time.stamp_ns;
  j["base"]=r.valid?geometry_json(r.base):Json();j["map"]=r.map?geometry_json(*r.map):Json();
  j["absolute_velocity_base"]=r.absolute_velocity_base?vector_json(*r.absolute_velocity_base):Json();
  j["absolute_velocity_map"]=r.absolute_velocity_map?vector_json(*r.absolute_velocity_map):Json();
  return j;
}
} // namespace
