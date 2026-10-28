// Offline diagnostic of the exact native camera calibration and local support.
#define main shadow_classifier_runtime_main
#include "../src/camera_bbox_lane_node.cpp"
#undef main
#include <ros/serialization.h>
#include <iostream>
template<class T> boost::shared_ptr<T> readFixture(const char* file){
 std::ifstream f(file,std::ios::binary);std::vector<uint8_t>b((std::istreambuf_iterator<char>(f)),{});
 if(b.empty())throw std::runtime_error("empty fixture");auto m=boost::make_shared<T>();
 ros::serialization::IStream s(b.data(),b.size());ros::serialization::deserialize(s,*m);return m;
}
int main(int argc,char**argv){
 if(argc!=4&&argc!=5)return 2;ros::Time::init();params.calibration_file=argv[3];params.projection_context_required=false;loadCalibration();
 auto bbox=readFixture<Result>(argv[1]);auto local=readFixture<asmc_msgs::LidarTrackingObservation>(argv[2]);
 const auto matches=lane_boundary_adapter::LocalObstacleSupport::associate(bbox->boxes,*local,camera.intrinsic,camera.rotation,camera.translation);
 for(unsigned i=0;i<bbox->boxes.size();++i){std::cout<<"box="<<i<<" class="<<bbox->boxes[i].label<<" match="<<matches[i]<<'\n';
  for(const auto& o:local->observation.objects.objects){lane_boundary_adapter::CameraBBox p;
   if(lane_boundary_adapter::LocalObstacleSupport::projectedBox(o,camera.intrinsic,camera.rotation,camera.translation,p))
    std::cout<<" track="<<o.track_id<<" class="<<o.camera_class<<" iou="<<lane_boundary_adapter::LocalObstacleSupport::iou(bbox->boxes[i],p)<<" projected="<<p.xmin<<','<<p.ymin<<','<<p.xmax<<','<<p.ymax<<'\n';
  }
 }
 if(argc==5){
  if(!bbox->local_obstacles_valid || bbox->local_obstacles_stamp!=local->observation.objects.header.stamp ||
     bbox->local_obstacles_session!=local->observation.producer_session_id ||
     bbox->local_obstacles_generation!=local->observation.generation || bbox->local_obstacles_sequence!=local->observation.scan_sequence)return 7;
  bbox->local_obstacle_footprints.clear();
  for(const auto& o:local->observation.objects.objects){geometry_msgs::Polygon p;
   if(!lane_boundary_adapter::LocalObstacleSupport::footprint(o,p))return 8;
   bbox->local_obstacle_footprints.push_back(p);}
  for(size_t i=0;i<bbox->boxes.size();++i)if(bbox->ground_contact_sources[i]=="lidar_observed"){
   if(matches[i]<0)return 9;
   bbox->ground_contacts[i]=bbox->local_obstacle_footprints[matches[i]];
  }
  std::vector<uint8_t> bytes(ros::serialization::serializationLength(*bbox));
  ros::serialization::OStream stream(bytes.data(),bytes.size());ros::serialization::serialize(stream,*bbox);
  std::ofstream f(argv[4],std::ios::binary);f.write(reinterpret_cast<const char*>(bytes.data()),bytes.size());
 }

}
