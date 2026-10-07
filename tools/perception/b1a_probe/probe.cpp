// Reuse the evaluator's global definitions and initialization unchanged.
#define main b1a_original_evaluator_main
#include "../Main/tools/tunnel_filter_offline_eval.cpp"
#undef main

int main(int argc, char** argv) {
    ros::init(argc, argv, "b1a_protection_probe");
    ros::NodeHandle nh, pnh("~");
    InitMutex(); InitVehicleSpec(); InitLidarSettings(pnh); InitCameraSettings();
    pnh.setParam("enable_tunnel_experimental_filter", true);
    pnh.setParam("tunnel_protect_enable", true);
    pnh.setParam("tunnel_wall_enable", false);
    auto cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    for (int i=0;i<30;++i) { pcl::PointXYZI p; p.x=8.f+.01f*i; p.y=.01f*(i%3); p.z=0.f; cloud->push_back(p); }
    GPS_IMU gps{}; gps.st_GPS.b_isJamming=true; gps.st_IMU.b_ImuReady=true;
    BBox box{}; box.st_Min_pt.f32_X=0; box.st_Min_pt.f32_Y=0;
    box.st_Max_pt.f32_X=1280; box.st_Max_pt.f32_Y=720; box.str_class="car";
    struct Case { const char* name; long long offset_ns; int kind; bool available; bool protected_expected; };
    const Case cases[] = {
      {"exact",0,1,true,true}, {"boundary_p50",50000000,1,true,true},
      {"boundary_m50",-50000000,1,true,true}, {"outside",50000001,1,false,false},
      {"missing",0,0,false,false}, {"empty",0,2,true,false}, {"late",0,3,false,false}
    };
    int failures=0;
    for (const auto& c:cases) {
      tunnel_filter::InitTunnelExperiment(nh,pnh,st_Camera,1280,720,false);
      const ros::Time stamp(100,0);
      ros::Time yolo; yolo.fromNSec(100000000000ULL+c.offset_ns);
      if(c.kind==1) tunnel_filter::TunnelExperimentIngestYolo(yolo,{box});
      if(c.kind==2) tunnel_filter::TunnelExperimentIngestYolo(yolo,{});
      tunnel_filter::TunnelExperimentCapture(cloud,stamp,gps);
      tunnel_filter::TunnelExperimentRun();
      const auto& f=tunnel_filter::TunnelExperimentLastFrame();
      const bool available=f.protection_available;
      const int protected_points=f.result?f.result->protected_points:0;
      if(c.kind==3) {
        tunnel_filter::TunnelExperimentIngestYolo(yolo,{box});
        tunnel_filter::TunnelExperimentRun();
      }
      const bool unchanged=f.protection_available==available;
      const bool ok=f.active && available==c.available && (protected_points>0)==c.protected_expected && unchanged;
      std::cout<<"B1A_PROBE {\"case\":\""<<c.name<<"\",\"available\":"<<(available?"true":"false")
               <<",\"protected_points\":"<<protected_points<<",\"pass\":"<<(ok?"true":"false")<<"}"<<std::endl;
      if(!ok) ++failures;
    }
    // A tall object intersects the ceiling cut even when marked protected.
    tunnel_filter::Params params; params.wall_enable=false; params.cluster_min_points=3;
    tunnel_filter::TunnelPipeline pipeline(params);
    auto tall=*cloud;
    for(auto& p:tall.points) p.z=3.f;
    std::vector<std::uint8_t> protect(tall.size(),tunnel_filter::kProtectTruck),ambiguous(tall.size(),0);
    const auto& r=pipeline.Process(tall,0,0,&protect,&ambiguous,true);
    const bool cutoff_observed=r.ceiling_removed==static_cast<int>(tall.size());
    std::cout<<"B1A_PROBE {\"case\":\"tall_protected_ceiling_cut_diagnostic\",\"removed\":"<<r.ceiling_removed
             <<",\"pass\":"<<(cutoff_observed?"true":"false")<<",\"object_preservation_pass\":false}"<<std::endl;
    if(!cutoff_observed) ++failures;
    return failures?1:0;
}
