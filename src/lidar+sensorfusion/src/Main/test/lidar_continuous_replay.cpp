// The production entrypoint is never called: no UDP, sensor worker or control
// publisher is started. Reuse its globals and exact settings initialization.
#define main lidar_runtime_entrypoint
#include "../src/main.cpp"
#undef main
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CompressedImage.h>
#include <SensorFusion/FusionStateTrace.hpp>
#include <Lidar/LidarProcess/PipelineTrace.hpp>
#include <iomanip>

namespace {
IMU_DATA Convert(const sensor_msgs::Imu& m) {
    IMU_DATA d{};
    d.f32_QuatX=m.orientation.x;d.f32_QuatY=m.orientation.y;
    d.f32_QuatZ=m.orientation.z;d.f32_QuatW=m.orientation.w;
    const double x=d.f32_QuatX,y=d.f32_QuatY,z=d.f32_QuatZ,w=d.f32_QuatW;
    d.f32_Roll_rad=atan2(2*(w*x+y*z),1-2*(x*x+y*y));
    d.f32_Pitch_rad=asin(2*(w*y-z*x));
    d.f32_Yaw_rad=atan2(2*(w*z+x*y),1-2*(y*y+z*z));
    d.f32_Roll_deg=Rad2Deg(d.f32_Roll_rad);d.f32_Pitch_deg=Rad2Deg(d.f32_Pitch_rad);
    d.f32_Yaw_deg=Rad2Deg(d.f32_Yaw_rad);
    const double norm=x*x+y*y+z*z+w*w;
    d.b_ImuReady=!m.header.stamp.isZero()&&std::isfinite(norm)&&fabs(norm-1)<=.001;
    return d;
}
void WriteObject(std::ostream& out,const LiDARObjectInfo& object) {
    std::ostringstream semantic;semantic<<std::setprecision(17);
    fusion_state_trace::Object(semantic,object);
    auto prefix=semantic.str();prefix.pop_back();out<<prefix;
    out<<",\"kf_initialized\":"<<(object.b_KfInitialized?"true":"false")
       <<",\"kf_init_count\":"<<object.s32_KfInitCount<<",\"kf_ca\":[";
    for(unsigned j=0;j<6;++j){if(j)out<<',';out<<object.Eigen_KfState_CA(j);}
    out<<"],\"corners\":[";
    for(unsigned j=0;j<8;++j){if(j)out<<',';const auto& q=object.OBB_Corners_Body[j];out<<'['<<q.x<<','<<q.y<<','<<q.z<<']';}
    out<<"],\"obb\":["<<object.f32_OBB_CenterX<<','<<object.f32_OBB_CenterY<<','<<object.f32_OBB_CenterZ
        <<','<<object.f32_OBB_Width<<','<<object.f32_OBB_Height<<','<<object.f32_OBB_YawRad<<"]}";
}

void MapInputs(GPS_IMU& local,const asmc::lidar::PoseSelection& map) {
    local.st_GPS.b_isJamming=!map.valid;
    if(map.valid) {
        const auto& v=st_VehicleSpec.st_GPSPos;
        const Eigen::Vector3d gps=map.map_from_base.translation()+map.map_from_base.rotation()*Eigen::Vector3d(v.f32_X,v.f32_Y,v.f32_Z);
        local.st_GPS.f64_E=gps.x();local.st_GPS.f64_N=gps.y();local.st_GPS.f64_U=gps.z();
    } else local.st_GPS.f64_E=local.st_GPS.f64_N=local.st_GPS.f64_U=NAN;
}
}

int main(int argc,char** argv) {
    ros::init(argc,argv,"lidar_continuous_replay");
    if(argc<5) {std::cerr<<"usage: lidar_continuous_replay BAG PROFILE OUTPUT TRACE [GPS_MODE BEGIN END]\n";return 2;}
    const std::string profile=argv[2],gps_mode=argc>5?argv[5]:"normal";
    if(profile!="legacy"&&profile!="common_pitch"&&profile!="common_roll_pitch")return 2;
    if(gps_mode!="normal"&&gps_mode!="invalid"&&gps_mode!="missing")return 2;
    const double begin=argc>6?std::stod(argv[6]):0,end=argc>7?std::stod(argv[7]):0;
    ros::NodeHandle nh,pnh("~");
    // The replay bypasses production main(); select its same class policy explicitly.
    std::string camera_profile;
    pnh.param("camera_class_profile",camera_profile,std::string("mission16_v1"));
    if(camera_profile!="legacy"&&camera_profile!="mission16_v1")return 2;
    fusion_class::MissionProfile()=camera_profile=="mission16_v1";
    pnh.setParam("enable_common_geometry",profile!="legacy");
    pnh.setParam("lidar_level_roll",profile=="common_roll_pitch");
    InitMutex();InitVehicleSpec();InitProjectionMatrix(st_Camera,YAML::LoadFile(YamlURL));
    frame_boundary::Configure(pnh,st_VehicleSpec,st_Camera);InitLidarSettings();
    InitLidarViz(nh);InitFusionLidarViz(nh);
    MotionClassifier classifier(pnh);SensorPoseHistory history;
    fusion_trace::Sink().Configure(std::string(argv[4])=="-"?"":argv[4]);
    std::ofstream output(argv[3]);if(!output)throw std::runtime_error("output open failed");
    output<<std::setprecision(17);std::srand(0);
    rosbag::Bag bag(argv[1],rosbag::bagmode::Read);rosbag::View view(bag);
    const auto first=view.getBeginTime().toNSec();std::uint64_t generation=0,yolo_seq=0;
    RAW_YOLO_DATA raw_yolo{};
    // Record order fixes availability. This clock is a logical offline arrival
    // clock (not a CPU runtime benchmark, nor a claim of actual callback order).
    for(const auto& event:view) {
        const auto arrival=event.getTime().toNSec();const auto logical=arrival-first+1000000000ULL;
        ros::Time::setNow(event.getTime());
        if(auto m=event.instantiate<sensor_msgs::Imu>()) {
            auto d=Convert(*m);history.AddImu(m->header.stamp.toNSec()/1000000,d,nullptr,
                profile=="legacy"?0:m->header.stamp.toNSec(),profile=="legacy"?0:logical);
        } else if(auto m=event.instantiate<morai_msgs::EgoVehicleStatus>()) {
            CAN_DATA d{};d.f32_CurrentVel_MPS=std::sqrt(m->velocity.x*m->velocity.x+m->velocity.y*m->velocity.y+m->velocity.z*m->velocity.z);
            d.f32_CurrentVel_KPH=3.6*d.f32_CurrentVel_MPS;
            history.AddCan(m->header.stamp.toNSec()/1000000,d,nullptr,
                profile=="legacy"?0:m->header.stamp.toNSec(),profile=="legacy"?0:logical);
        } else if(auto m=event.instantiate<morai_msgs::GPSMessage>()) {
            const double elapsed=(arrival-first)*1e-9;
            const bool inject=elapsed>=begin&&elapsed<end;
            if(inject&&gps_mode=="missing")continue;
            const auto source=m->header.stamp.toNSec();
            if((inject&&gps_mode=="invalid")||!m->status||(!m->latitude&&!m->longitude)) {
                if(profile=="legacy")history.AddGpsInvalid(source/1000000,arrival/1000000);
                else history.AddGpsInvalidAt(source,logical);
            } else {
                GPS_DATA gps{};int zone=52;bool north=true;double east,n;
                GeographicLib::UTMUPS::Forward(m->latitude,m->longitude,zone,north,east,n);
                gps.f64_E=east-m->eastOffset;gps.f64_N=n-m->northOffset;
                gps.f64_U=profile=="legacy"?0:m->altitude-frame_boundary::MapAltitudeOrigin();
                history.AddGps(source/1000000,gps,profile=="legacy"?0:source,profile=="legacy"?0:logical);
            }
        } else if(auto m=event.instantiate<sensor_msgs::CompressedImage>()) {
            if((event.getTopic()=="/image_jpeg/compressed" || event.getTopic()=="/camera/image/compressed"))
                BufferFusionCameraFrame(m->header.stamp,m->data);
        } else if(auto m=event.instantiate<Msg::Yolo_bbox_arr>()) {
            std::vector<BBox> boxes;
            for(const auto& b:m->yolo_bbox_arr) {
                BBox box{};box.st_Min_pt.f32_X=b.xMin;box.st_Min_pt.f32_Y=b.yMin;
                box.st_Max_pt.f32_X=b.xMax;box.st_Max_pt.f32_Y=b.yMax;
                box.str_class=box.str_raw_class=b.classes;boxes.push_back(box);
            }
            BufferFusionYoloFrame(++yolo_seq,m->header.stamp,boxes);
        } else if(auto m=event.instantiate<sensor_msgs::PointCloud2>()) {
            ++generation;const auto source=m->header.stamp.toNSec();GPS_IMU local{};
            std::string reason;LocalInputSelection selected;
            const bool ready=profile=="legacy"?history.Lookup(source/1000000,&local,&reason):
                history.WaitForLocalLookupNs(source,0,&local,&selected,&reason);
            output<<"{\"stamp_ns\":"<<source<<",\"ready\":"<<(ready?"true":"false")
                <<",\"reason\":"<<fusion_trace::Quote(reason)<<",\"imu_ns\":"<<selected.imu_stamp_ns
                <<",\"can_ns\":"<<selected.can_stamp_ns;
            if(!ready){output<<",\"objects\":[]}\n";continue;}
            asmc::lidar::Observation observation{static_cast<asmc::lidar::Ns>(source),
                static_cast<asmc::lidar::Ns>(logical),static_cast<asmc::lidar::Ns>(logical),"ros","velodyne","steady"};
            if(profile!="legacy") {
                const auto map=history.MapAt(observation,local);
                frame_boundary::PrepareScan(observation,local,st_VehicleSpec,map,generation);MapInputs(local,map);
            }
            pcl::PointCloud<pcl::PointXYZI> cloud;pcl::fromROSMsg(*m,cloud);
            auto raw=std::make_unique<RAW_LIDAR_DATA>();raw->u64_Timestamp=source/1000000;
            raw->source_stamp_ns=source;raw->receipt_steady_ns=logical;
            raw->s32_Num=cloud.size();if(cloud.size()*sizeof(pcl::PointXYZI)>sizeof(raw->arc_Buffer))throw std::runtime_error("raw capacity");
            std::memcpy(raw->arc_Buffer,cloud.points.data(),cloud.size()*sizeof(pcl::PointXYZI));
            if(!LidarProcess(*raw,local,st_LiDAR,classifier))throw std::runtime_error("scan processing rejected");
            ros::WallTime decision;decision.fromNSec(logical);
            FusionProcessAt(raw_yolo,st_LiDAR,st_Camera,frame_boundary::SourceStamp(*raw),decision);
            const auto plane=CurrentGroundPlaneSnapshot();
            output<<",\"plane_valid\":"<<(plane.valid?"true":"false")<<",\"plane\":[";
            for(unsigned i=0;i<4;++i){if(i)output<<',';output<<plane.coefficients[i];}
            output<<"],\"remaining_points\":"<<g_LidarStageProfile.ransac_points<<",\"objects\":[";
            bool sep=false;for(const auto& object:st_LiDAR.vec_Objects){if(sep)output<<',';sep=true;WriteObject(output,object);}
            output<<"]}\n";
        }
    }
    ReportFusionTotals();fusion_trace::Sink().Flush();output.flush();
    return output?0:1;
}
