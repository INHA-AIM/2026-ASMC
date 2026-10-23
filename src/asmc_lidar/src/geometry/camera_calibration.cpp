#include <asmc_lidar/geometry/camera_calibration.hpp>
#include <yaml-cpp/yaml.h>
#include <openssl/sha.h>
#include <algorithm>
#include <codecvt>
#include <cstdio>
#include <iomanip>
#include <locale>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace asmc::lidar::camera {
namespace {
std::string quoted(const std::string& s) {
  std::wstring_convert<std::codecvt_utf8<char32_t>,char32_t> converter;
  std::ostringstream out;out << '"';
  for(char32_t c:converter.from_bytes(s)) {
    if(c=='"'||c=='\\') out << '\\' << char(c);
    else if(c=='\b')out<<"\\b";else if(c=='\f')out<<"\\f";
    else if(c=='\n')out<<"\\n";else if(c=='\r')out<<"\\r";else if(c=='\t')out<<"\\t";
    else if(c>=32 && c<128)out<<char(c);
    else {
      auto hex=[&](unsigned v){out<<"\\u"<<std::hex<<std::setfill('0')<<std::setw(4)<<v<<std::dec;};
      if(c<=0xffff)hex(c);else {c-=0x10000;hex(0xd800+(c>>10));hex(0xdc00+(c&1023));}
    }
  }
  return out.str()+'"';
}
// Find the shortest round-trip decimal, then use Python JSON's fixed/scientific
// boundary. Calibration values are finite YAML numbers; ambiguous YAML forms
// (octal, sexagesimal, tags) are rejected rather than hashed differently.
std::string number(double v) {
  if(!std::isfinite(v))throw std::runtime_error("nonfinite calibration number");
  if(v==0)return std::signbit(v)?"-0.0":"0.0";
  char buf[128];int digits=17;
  for(int p=1;p<=17;++p) {
    std::snprintf(buf,sizeof(buf),"%.*g",p,v);
    if(std::stod(buf)==v){digits=p;break;}
  }
  const int exp=static_cast<int>(std::floor(std::log10(std::abs(v))));
  if(std::abs(v)>=1e-4 && std::abs(v)<1e16) {
    const int decimals=std::max(0,digits-1-exp);
    std::snprintf(buf,sizeof(buf),"%.*f",decimals,v);
    std::string s(buf);if(s.find('.')==std::string::npos)s+=".0";return s;
  }
  std::snprintf(buf,sizeof(buf),"%.*e",digits-1,v);return buf;
}
std::string canonical(const YAML::Node& n) {
  if(n.IsNull())return "null";
  if(n.IsSequence()) {
    std::string s="[";for(std::size_t i=0;i<n.size();++i){if(i)s+=',';s+=canonical(n[i]);}return s+"]";
  }
  if(n.IsMap()) {
    std::map<std::string,std::string> sorted;
    for(const auto& e:n) {
      const auto key=e.first.as<std::string>();
      if(!sorted.emplace(key,canonical(e.second)).second)throw std::runtime_error("duplicate calibration key");
    }
    std::string s="{";for(const auto& e:sorted){if(s.size()>1)s+=',';s+=quoted(e.first)+":"+e.second;}return s+"}";
  }
  const auto s=n.Scalar();
  if(n.Tag()=="!" || n.Tag()=="tag:yaml.org,2002:str")return quoted(s);
  std::string low=s;std::transform(low.begin(),low.end(),low.begin(),[](unsigned char c){return std::tolower(c);});
  if(low=="true"||low=="yes"||low=="on")return "true";
  if(low=="false"||low=="no"||low=="off")return "false";
  static const std::regex integer("[-+]?(0|[1-9][0-9]*)");
  static const std::regex floating("[-+]?([0-9]+\\.[0-9]*|\\.[0-9]+)([eE][-+][0-9]+)?");
  if(std::regex_match(s,integer))return std::to_string(std::stoll(s));
  if(std::regex_match(s,floating))return number(std::stod(s));
  // Python SafeLoader treats plain 1e3 as a string. Reject other numeric-looking
  // unsupported encodings so a bad calibration cannot silently become trusted.
  if(!s.empty() && (std::isdigit(static_cast<unsigned char>(s[0]))||s[0]=='+'||s[0]=='-'||s[0]=='.') &&
      s.find(':')==std::string::npos && s.find('/')==std::string::npos)
    throw std::runtime_error("unsupported calibration scalar: "+s);
  return quoted(s);
}
std::string digest(const std::string& text) {
  unsigned char bytes[SHA256_DIGEST_LENGTH];SHA256(reinterpret_cast<const unsigned char*>(text.data()),text.size(),bytes);
  std::ostringstream out;out<<"sha256:"<<std::hex<<std::setfill('0');for(auto b:bytes)out<<std::setw(2)<<unsigned(b);return out.str();
}
Vec vector3(const YAML::Node& n) {
  if(!n.IsSequence()||n.size()!=3)throw std::runtime_error("expected calibration vector3");
  Vec v(n[0].as<double>(),n[1].as<double>(),n[2].as<double>());
  if(!v.allFinite())throw std::runtime_error("nonfinite calibration vector");
  return v;
}
}
Calibrations load_calibrations(const std::string& path) {
  const auto root=YAML::LoadFile(path);
  const auto source=root["source_sha256"].as<std::string>();
  if(!std::regex_match(source,std::regex("[0-9a-f]{64}")))throw std::runtime_error("missing/invalid sensor source SHA");
  Calibrations out;
  if(!root["cameras"].IsMap()||root["cameras"].size()!=3)throw std::runtime_error("expected front/left/right calibration");
  for(const std::string view:{"front","left","right"}) {
    const auto n=root["cameras"][view];Calibration c;c.frame_id="camera_"+view;
    const int width=n["width"].as<int>(),height=n["height"].as<int>();
    const double fov=n["horizontal_fov_deg"].as<double>();
    if(width<=0||height<=0||!std::isfinite(fov)||fov<=0||fov>=180)throw std::runtime_error("invalid camera dimensions/FOV");
    c.width=width;c.height=height;
    c.id=digest("{\"camera\":"+canonical(n)+",\"source_sha256\":"+quoted(source)+"}");
    c.base_from_camera.translation()=vector3(n["translation_m"]);
    const auto angles=vector3(n["rotation_deg"])*(std::acos(-1.)/180.);
    c.base_from_camera.linear()=rpy(angles.x(),angles.y(),angles.z());
    const double focal=width/(2.*std::tan(fov*std::acos(-1.)/360.));
    c.intrinsic<<focal,0,width/2.,0,focal,height/2.,0,0,1;
    out.emplace(c.frame_id,c);
  }
  return out;
}
PixelResult project(const Calibration& camera, const Context& scan, Ns image_stamp,
                    const Vec& point, const std::optional<Pose>& compensation) {
  PixelResult result;
  if(scan.time.stamp_ns<=0||image_stamp<=0||scan.time.clock!="ros"||!validate_time(scan.time).empty()) {
    result.reason="invalid_time";return result;
  }
  result.image_minus_scan_ns=image_stamp-scan.time.stamp_ns;
  auto base=base_from_input(scan);
  if(!base||!point.allFinite()||!pose_valid(camera.base_from_camera)||!camera.intrinsic.allFinite()||
     camera.intrinsic(0,0)<=0||camera.intrinsic(1,1)<=0||!camera.width||!camera.height) {
    result.reason="invalid_geometry";return result;
  }
  Context projected=scan;
  Vec input=point;
  if(compensation) {
    if(!pose_valid(*compensation)){result.reason="invalid_compensation";return result;}
    input=(*compensation)*((*base)*point);projected.frame=Frame::Base;
    result.time_compensated=true;
  }
  const auto matrix=camera_projection(projected,camera.base_from_camera,camera.intrinsic);
  if(!matrix){result.reason="invalid_projection";return result;}
  const Vec pixel=(*matrix)*input.homogeneous();result.depth=pixel.z();
  if(!pixel.allFinite()||pixel.z()<=0){result.reason="behind_camera";return result;}
  result.u=pixel.x()/pixel.z();result.v=pixel.y()/pixel.z();
  if(result.u<0||result.v<0||result.u>=camera.width||result.v>=camera.height){result.reason="outside_image";return result;}
  result.valid=true;
  result.reason=result.image_minus_scan_ns!=0&&!compensation?"ok_uncompensated_time_offset":"ok";
  return result;
}
} // namespace asmc::lidar::camera
