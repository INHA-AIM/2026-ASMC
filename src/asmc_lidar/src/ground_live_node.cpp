#include <asmc_lidar/ground_live.hpp>
#include <asmc_lidar/imu_selection.hpp>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <iomanip>
#include <sstream>

namespace {
using Clock=std::chrono::steady_clock;
double Ms(Clock::time_point t){return std::chrono::duration<double,std::milli>(Clock::now()-t).count();}
class Node {
 public:
  Node():p_("~") {
    std::string method,calibration,cloud,imu,comparison;
    p_.param<std::string>("comparison_config",comparison,"");
    p_.param<std::string>("method",method,"all");
    p_.param<std::string>("patchwork_variant",variant_,"guard25");
    p_.param<std::string>("calibration_path",calibration,"");
    p_.param<std::string>("cloud_topic",cloud,"/velodyne_points");
    p_.param<std::string>("imu_topic",imu,"/imu");
    p_.param<std::string>("stamp_provenance",stamp_provenance_,"bridge_receive");
    // An existing bridge may have been started with a different timestamp mode.
    std::string active_mode;
    if(cloud=="/velodyne_points"&&nh_.getParam("/morai_sensor_bridge/timestamp_mode",active_mode))
      stamp_provenance_=active_mode=="source"?"simulator_source":
                        active_mode=="receive"?"bridge_receive":"unknown_bridge_mode";
    cal_=asmc_lidar::LoadCalibration(calibration);
    model_.reset(new asmc_lidar::GroundComparison(method,variant_,asmc_lidar::LoadGroundComparisonConfig(comparison)));
    for(const auto& name:{"jaeho","patchwork","linefit"}) {
      if(method!="all"&&method!=name)continue;
      for(const auto& kind:{"ground","nonground","uncertain","structural_removed","native_retained"})
        pubs_[std::string(name)+"/"+kind]=nh_.advertise<sensor_msgs::PointCloud2>(
            "/lidar_candidate/ground_live/"+std::string(name)+"/"+kind,1);
    }
    status_=nh_.advertise<diagnostic_msgs::DiagnosticArray>("/lidar_candidate/ground_live/status",10);
    cloud_=nh_.subscribe(cloud,1,&Node::Enqueue,this);
    imu_=nh_.subscribe(imu,100,&Node::Imu,this);
    timer_=nh_.createWallTimer(ros::WallDuration(.25),&Node::Timer,this);
    worker_=std::thread(&Node::Work,this);
    ROS_WARN("Ground live REVIEW: general-road ground only; rear-axle base_link; Patchwork=%s",variant_.c_str());
  }
  ~Node(){
    {std::lock_guard<std::mutex> lock(mutex_);stop_=true;}
    cv_.notify_all();worker_.join();
  }
 private:
  struct Request {
    sensor_msgs::PointCloud2::ConstPtr cloud;
    asmc_lidar::ImuSelection imu;
    Clock::time_point arrival;
    std::uint64_t epoch=0,receipt_ns=0;
    std::string error;
  };
  void Imu(const sensor_msgs::Imu::ConstPtr& msg){
    std::lock_guard<std::mutex> lock(mutex_);
    Eigen::Quaterniond q(msg->orientation.w,msg->orientation.x,msg->orientation.y,msg->orientation.z);
    asmc_lidar::ImuSample s;s.stamp_ns=msg->header.stamp.toNSec();
    if(!s.stamp_ns)return;
    const auto norm=q.squaredNorm();s.valid=std::isfinite(norm)&&norm>=.9&&norm<=1.1;
    s.valid=s.valid&&msg->orientation_covariance[0]!=-1;
    if(s.valid){
      const auto r=q.normalized().toRotationMatrix();
      s.roll_rad=std::atan2(r(2,1),r(2,2));
      s.pitch_rad=std::asin(std::max(-1.,std::min(1.,-r(2,0))));
    }
    history_.push_back(s);while(history_.size()>100)history_.pop_front();
  }
  void Enqueue(const sensor_msgs::PointCloud2::ConstPtr& msg){
    std::lock_guard<std::mutex> lock(mutex_);
    auto r=std::make_shared<Request>();r->cloud=msg;r->arrival=Clock::now();
    r->receipt_ns=ros::Time::now().toNSec();++received_;
    const auto ns=msg->header.stamp.toNSec();
    if(ns&&last_stamp_&&ns<last_stamp_){
      ++epoch_;++resets_;history_.clear();reset_needed_=true;
      if(pending_){++dropped_;pending_.reset();}
    }
    r->epoch=epoch_;
    if(!ns)r->error="zero_stamp";
    else if(ns==last_stamp_)r->error="duplicate_stamp";
    else {
      last_stamp_=ns;r->imu=asmc_lidar::SelectImu(history_,ns);
      if(r->imu.status!=asmc_lidar::ImuSelectionStatus::Valid)
        r->error=std::string("imu_")+asmc_lidar::ImuSelectionName(r->imu.status);
    }
    last_arrival_=r->arrival;last_header_=msg->header;stale_=false;
    if(pending_)++dropped_;
    pending_=r;cv_.notify_one();
  }
  sensor_msgs::PointCloud2 Cloud(const asmc_lidar::Cloud& input,
      const std::vector<std::size_t>& ids,const std_msgs::Header& header){
    sensor_msgs::PointCloud2 out;out.header=header;out.header.frame_id="base_link";
    sensor_msgs::PointCloud2Modifier modifier(out);
    modifier.setPointCloud2Fields(5,"x",1,sensor_msgs::PointField::FLOAT32,
      "y",1,sensor_msgs::PointField::FLOAT32,"z",1,sensor_msgs::PointField::FLOAT32,
      "intensity",1,sensor_msgs::PointField::FLOAT32,"source_index",1,sensor_msgs::PointField::UINT32);
    modifier.resize(ids.size());
    sensor_msgs::PointCloud2Iterator<float> x(out,"x"),y(out,"y"),z(out,"z"),intensity(out,"intensity");
    sensor_msgs::PointCloud2Iterator<std::uint32_t> index(out,"source_index");
    for(auto i:ids){
      const auto& p=input[i];
      // Same as inverse(leveling) then mount, without the float round trip.
      const auto b=cal_.base_from_sensor*Eigen::Vector3d(p.x,p.y,p.z);
      *x=float(b.x());*y=float(b.y());*z=float(b.z());*intensity=p.intensity;*index=i;
      ++x;++y;++z;++intensity;++index;
    }
    out.is_dense=false;return out;
  }
  void Clear(const std_msgs::Header& header){
    const auto empty=Cloud(asmc_lidar::Cloud{}, {},header);
    for(auto& pub:pubs_)pub.second.publish(empty);
  }
  void Status(const std::string& state,const Request* r,
      const asmc_lidar::GroundLiveResult* result,double wait,double compute,double output_ms){
    diagnostic_msgs::DiagnosticArray array;array.header.stamp=ros::Time::now();
    diagnostic_msgs::DiagnosticStatus d;d.name="asmc_lidar/ground_live";d.hardware_id="Autonav v3";
    d.level=(state=="valid"||state=="observed_empty"||state=="roi_empty")?d.OK:d.WARN;d.message=state;
    auto add=[&](const std::string& k,const auto& value){
      diagnostic_msgs::KeyValue v;v.key=k;std::ostringstream s;s<<std::setprecision(17)<<value;
      v.value=s.str();d.values.push_back(v);
    };
    add("received",received_);add("processed",processed_);add("invalid",invalid_);
    add("queue_dropped",dropped_);add("time_resets",resets_);
    add("source_stamp_ns",r?r->cloud->header.stamp.toNSec():last_header_.stamp.toNSec());
    add("receipt_ros_ns",r?r->receipt_ns:0);
    add("stamp_provenance",stamp_provenance_);add("frame","base_link");
    add("patchwork_variant",variant_);add("config_id",model_->config().id);add("config_sha256",model_->config().sha256);
    add("queue_wait_ms",wait);add("compute_ms",compute);add("publish_ms",output_ms);
    add("receipt_to_publish_ms",r?Ms(r->arrival):0.);
    if(r){add("imu_stamp_ns",r->imu.sample.stamp_ns);add("imu_pitch",r->imu.sample.pitch_rad);
      add("imu_roll",r->imu.sample.roll_rad);add("imu_newer_invalid_seen",r->imu.newer_invalid_seen);}
    if(result){
      add("prepare_ms",result->prepared.prepare_ms);add("voxel_points",result->prepared.voxel->size());
      for(const auto& item:result->methods){
        add(item.first+"_segment_ms",item.second.segment_ms);
        add(item.first+"_copy_ms",item.second.copy_ms);
        add(item.first+"_guard_ms",item.second.guard_ms);
        add(item.first+"_restore_ms",item.second.restore_ms);
        for(const auto& event:item.second.events)add(item.first+"_"+event.first,event.second);
        std::size_t unknown=0,guard_restored=0;
        for(std::size_t i=0;i<item.second.labels.size();++i){
          unknown+=item.second.labels[i]==3;
          if(item.first=="patchwork"&&!item.second.reasons.empty())guard_restored+=(item.second.reasons[i]&2048)!=0;
        }
        add(item.first+"_unclassified_raw_points",unknown);
        if(item.first=="patchwork")add("patchwork_guard_restored_raw_points",guard_restored);
      }
    }
    array.status.push_back(d);status_.publish(array);
  }
  void Work(){
    for(;;){
      std::shared_ptr<Request> r;bool reset;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock,[&]{return stop_||bool(pending_);});if(stop_)return;
        r=std::move(pending_);reset=reset_needed_;reset_needed_=false;
      }
      const double wait=Ms(r->arrival);const auto start=Clock::now();
      asmc_lidar::Cloud cloud;asmc_lidar::GroundLiveResult result;std::string state=r->error;
      try{
        if(reset)model_->Reset();
        if(state.empty()){
          for(const auto& field:{"x","y","z","intensity"})
            if(std::none_of(r->cloud->fields.begin(),r->cloud->fields.end(),[&](const auto& f){
              return f.name==field&&f.datatype==sensor_msgs::PointField::FLOAT32&&f.count==1;}))
              throw std::runtime_error(std::string("missing_float32_field:")+field);
          pcl::fromROSMsg(*r->cloud,cloud);
          result=model_->Process(cloud,cal_,float(r->imu.sample.pitch_rad));
          state=cloud.empty()?"observed_empty":result.prepared.voxel->empty()?"roi_empty":"valid";
        }
      }catch(const std::exception& e){state=std::string("processing_error:")+e.what();}
      const double compute=Ms(start);const auto output=Clock::now();
      std::lock_guard<std::mutex> lock(mutex_);
      if(r->epoch!=epoch_){++dropped_;continue;}
      if(Ms(r->arrival)>500){++invalid_;Clear(r->cloud->header);Status("processing_stale",r.get(),nullptr,wait,compute,0);continue;}
      if(state!="valid"&&state!="observed_empty"&&state!="roi_empty"){
        ++invalid_;Clear(r->cloud->header);Status(state,r.get(),nullptr,wait,compute,0);continue;
      }
      ++processed_;
      for(const auto& method:result.methods){
        std::map<std::string,std::vector<std::size_t>> ids;
        for(const auto& name:{"ground","nonground","uncertain","structural_removed","native_retained"})ids[name]={};
        for(std::size_t i=0;i<cloud.size();++i){
          const auto label=method.second.labels[i];if(!label)continue;
          ids[label==1?"ground":label==2?"nonground":"uncertain"].push_back(i);
          if(method.second.structural_removed[i])ids["structural_removed"].push_back(i);
          else if(label==2)ids["native_retained"].push_back(i);
        }
        for(const auto& item:ids)pubs_.at(method.first+"/"+item.first).publish(Cloud(cloud,item.second,r->cloud->header));
      }
      Status(state,r.get(),&result,wait,compute,Ms(output));
      ROS_INFO_THROTTLE(5.,"Ground comparison rx=%lu processed=%lu invalid=%lu queue_drop=%lu compute=%.2fms",
          received_,processed_,invalid_,dropped_,compute);
    }
  }
  void Timer(const ros::WallTimerEvent&){
    std::lock_guard<std::mutex> lock(mutex_);
    if(last_arrival_!=Clock::time_point{}&&Ms(last_arrival_)<500)return;
    if(!stale_){Clear(last_header_);stale_=true;}
    Status("sensor_missing",nullptr,nullptr,0,0,0);
  }
  ros::NodeHandle nh_,p_;ros::Subscriber cloud_,imu_;ros::Publisher status_;
  ros::WallTimer timer_;std::map<std::string,ros::Publisher> pubs_;
  asmc_lidar::Calibration cal_;std::unique_ptr<asmc_lidar::GroundComparison> model_;
  std::deque<asmc_lidar::ImuSample> history_;
  std::mutex mutex_;std::condition_variable cv_;std::thread worker_;
  std::shared_ptr<Request> pending_;bool stop_=false,reset_needed_=false,stale_=false;
  Clock::time_point last_arrival_;std_msgs::Header last_header_;
  std::uint64_t epoch_=0,last_stamp_=0,received_=0,processed_=0,invalid_=0,dropped_=0,resets_=0;
  std::string variant_,stamp_provenance_;
};
}
int main(int argc,char** argv){
  ros::init(argc,argv,"ground_compare_live");
  try{Node node;ros::spin();return 0;}catch(const std::exception& e){ROS_FATAL("%s",e.what());return 1;}
}
