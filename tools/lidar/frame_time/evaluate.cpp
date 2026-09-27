// JSONL stdin -> JSONL stdout; no ROS master or runtime publication.
#include "io.hpp"
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <numeric>
using namespace asmc::lidar_eval;
using Clock=std::chrono::steady_clock;
static double ms(Clock::time_point begin){return std::chrono::duration<double,std::milli>(Clock::now()-begin).count();}
static Json percentiles(std::vector<double> v) {
  std::sort(v.begin(),v.end());Json j;
  for(auto [key,p]:{std::pair{"p50_ms",.50}, {"p95_ms",.95}, {"p99_ms",.99}}) {
    const auto index=std::min(v.size()-1,std::size_t(std::ceil(p*v.size())-1));j[key]=v[index];
  }
  j["repetitions"]=Ns(v.size());return j;
}
static Json cloud_eval(const std::string& path,const Pose& t) {
  std::ifstream in(path,std::ios::binary|std::ios::ate);
  if(!in)throw std::runtime_error("cloud file unavailable");
  const auto bytes=in.tellg();
  if(bytes<=0 || bytes%16 || bytes>256*1024*1024)throw std::runtime_error("invalid/oversize XYZI float32 cloud");
  in.seekg(0);std::vector<float> raw(std::size_t(bytes)/4);in.read(reinterpret_cast<char*>(raw.data()),bytes);
  if(!in)throw std::runtime_error("truncated cloud");
  std::vector<Point> points;points.reserve(raw.size()/4);
  for(std::size_t i=0;i<raw.size();i+=4) {
    Point p{Vec(raw[i],raw[i+1],raw[i+2]),raw[i+3]};
    if(!p.xyz.allFinite())throw std::runtime_error("nonfinite XYZ; no implicit point deletion");
    points.push_back(p);
  }
  auto result=transform_points(points,t);auto back=transform_points(result,t.inverse());
  double maximum=0.;Ns attr_mismatch=0;
  for(std::size_t i=0;i<points.size();++i) {
    maximum=std::max(maximum,(back[i].xyz-points[i].xyz).norm());
    if(std::memcmp(&points[i].intensity,&result[i].intensity,sizeof(float)))++attr_mismatch;
  }
  std::vector<double> copy_times,transform_times;volatile double sink=0.;
  for(int i=0;i<34;++i) {
    auto start=Clock::now();auto copy=points;double a=ms(start);
    start=Clock::now();auto transformed=transform_points(points,t);double b=ms(start);
    sink+=copy.front().xyz.x()+transformed.back().xyz.x();
    if(i>=3){copy_times.push_back(a);transform_times.push_back(b);}
  }
  (void)sink;Json j;j["input_points"]=Ns(points.size());j["output_points"]=Ns(result.size());
  j["intensity_bit_mismatches"]=attr_mismatch;j["roundtrip_max_m"]=maximum;
  j["copy_timing"]=percentiles(copy_times);j["copy_and_transform_timing"]=percentiles(transform_times);
  j["timing_scope"]="double XYZ + float intensity; steady_clock; allocation/copy included; decode/file IO excluded; no deskew";
  return j;
}
int main() {
  std::string line;Ns line_number=0;bool failed=false;
  std::map<std::string,StampGuard> guards;
  while(std::getline(std::cin,line)) {
    if(line.empty())continue;++line_number;Json out;
    try {
      auto n=YAML::Load(line);out["input_id"]=n["input_id"].as<std::string>();
      PoseHistory h(n["max_pose_gap_ns"].as<Ns>(100000000));Context c=context(n,h);
      const auto error=guards[n["stream_id"].as<std::string>("default")].accept(c.time);
      out["stamp_ns"]=c.time.stamp_ns;out["stamp_provenance"]=c.time.stamp_provenance;
      out["clock"]=c.time.clock;out["receipt_ns"]=c.time.receipt_ns;out["processing_ns"]=c.time.processing_ns;
      out["valid"]=error.empty();out["reason"]=error.empty()?"valid":error;
      out["pose_reason"]=c.map_pose.reason;out["pose_sample_stamps"]=numbers_json(c.map_pose.sample_stamps);
      out["pose_sample_arrivals"]=numbers_json(c.map_pose.sample_arrivals);
      Json::Array objects;
      if(error.empty()) {
        std::vector<Object> parsed;for(const auto& item:n["objects"])parsed.push_back(object(item));
        for(const auto& item:parsed)objects.push_back(result_json(transform(item,c)));
        if(n["cloud_path"]) {
          auto t=base_from_input(c);if(!t)throw std::runtime_error("invalid cloud transform");
          out["cloud"]=cloud_eval(n["cloud_path"].as<std::string>(),*t);
        }
        if(!parsed.empty()) {
          std::vector<double> timing,copy_timing;volatile double sink=0.;
          for(int i=0;i<103;++i) {
            auto start=Clock::now();auto copy=parsed;const double a=ms(start);
            start=Clock::now();for(const auto& o:parsed){auto r=transform(o,c);if(r.valid)sink+=r.base.center.x();}
            const double b=ms(start);sink+=copy.front().center.x();
            if(i>=3){copy_timing.push_back(a);timing.push_back(b);}
          }
          (void)sink;out["object_batch_copy_timing"]=percentiles(copy_timing);
          out["object_batch_transform_timing"]=percentiles(timing);
        }
      }
      out["objects"]=objects;
    } catch(const std::exception& e) {
      failed=true;out["valid"]=false;out["reason"]="input_error";out["detail"]=e.what();out["line"]=line_number;
    }
    std::cout<<out.dump()<<'\n';
  }
  return failed?2:0;
}
