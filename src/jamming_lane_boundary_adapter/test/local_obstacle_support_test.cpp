#include <gtest/gtest.h>
#include <boost/make_shared.hpp>
#include "lane_boundary_adapter/local_obstacle_support.hpp"
using Support=lane_boundary_adapter::LocalObstacleSupport;
using Scan=asmc_msgs::LidarTrackingObservation;
Scan::Ptr scan(double stamp=10.,std::string session="one",uint64_t generation=1){
 auto m=boost::make_shared<Scan>();auto& o=m->observation;o.producer_session_id=session;o.generation=generation;
 o.scan_sequence=std::llround(stamp*1000);o.config_id="config";o.calibration_id="calibration";o.objects.header.frame_id="base_link";
 o.objects.header.stamp=ros::Time(stamp);o.objects.status=o.objects.OBSERVED_EMPTY;return m;
}
asmc_msgs::LidarObject object(){
 asmc_msgs::LidarObject o;o.observation_state=o.OBSERVED;o.source_point_count=20;o.length=o.width=o.height=1.;o.camera_class="traffic_drum";
 unsigned i=0;for(double z:{5.,6.})for(const auto& xy:std::vector<cv::Point2d>{{-1,-1},{1,-1},{1,1},{-1,1}}){auto& p=o.obb_corners[i++];p.x=xy.x;p.y=xy.y;p.z=z;}return o;
}
TEST(LocalSupport,FreshnessSkewAndReset){
 Support c;auto m=scan();c.accept(m,ros::Time(10.),ros::SteadyTime(20.));
 EXPECT_TRUE(c.select(ros::Time(10.05),ros::Time(10.1),ros::SteadyTime(20.1)));
 EXPECT_FALSE(c.select(ros::Time(10.051),ros::Time(10.1),ros::SteadyTime(20.1)));
 EXPECT_FALSE(c.select(ros::Time(10.),ros::Time(10.36),ros::SteadyTime(20.1)));
 EXPECT_FALSE(c.select(ros::Time(10.),ros::Time(10.1),ros::SteadyTime(20.36)));
 auto newer=scan(10.1,"two");c.accept(newer,ros::Time(10.1),ros::SteadyTime(20.1));
 EXPECT_FALSE(c.select(ros::Time(10.),ros::Time(10.1),ros::SteadyTime(20.1)));
 auto invalid=scan(10.2,"two");invalid->observation.objects.status=3;c.accept(invalid,ros::Time(10.2),ros::SteadyTime(20.2));
 EXPECT_FALSE(c.select(ros::Time(10.1),ros::Time(10.2),ros::SteadyTime(20.2)));
}
TEST(LocalSupport,OldProducerAndSameGenerationIdentityMutationAreNotAccepted){
 Support c;auto first=scan();c.accept(first,ros::Time(10.),ros::SteadyTime(20.));
 auto second=scan(10.1,"two");c.accept(second,ros::Time(10.1),ros::SteadyTime(20.1));
 auto delayed=scan(10.2,"one");c.accept(delayed,ros::Time(10.2),ros::SteadyTime(20.2));
 EXPECT_FALSE(c.select(ros::Time(10.2),ros::Time(10.2),ros::SteadyTime(20.2)));
 auto mutation=scan(10.21,"two");mutation->observation.config_id="changed-without-reset";
 c.accept(mutation,ros::Time(10.21),ros::SteadyTime(20.21));
 EXPECT_FALSE(c.select(ros::Time(10.21),ros::Time(10.21),ros::SteadyTime(20.21)));
 mutation->observation.generation=2;c.accept(mutation,ros::Time(10.21),ros::SteadyTime(20.21));
 EXPECT_TRUE(c.select(ros::Time(10.21),ros::Time(10.21),ros::SteadyTime(20.21)));
}
TEST(LocalSupport,RetiredOrDuplicatePacketsDoNotInvalidateCurrentEvidence){
 Support c;c.accept(scan(),ros::Time(10.),ros::SteadyTime(20.));
 auto current=scan(10.1,"two");c.accept(current,ros::Time(10.1),ros::SteadyTime(20.1));
 c.accept(scan(9.7,"one"),ros::Time(10.2),ros::SteadyTime(20.2));
 EXPECT_EQ(current,c.select(ros::Time(10.1),ros::Time(10.2),ros::SteadyTime(20.2)));
 auto duplicate=scan(10.2,"two");duplicate->observation.scan_sequence=current->observation.scan_sequence;
 c.accept(duplicate,ros::Time(10.2),ros::SteadyTime(20.2));
 EXPECT_FALSE(c.select(ros::Time(10.2),ros::Time(10.2),ros::SteadyTime(20.2)));
 EXPECT_EQ(current,c.select(ros::Time(10.1),ros::Time(10.2),ros::SteadyTime(20.2)));
 EXPECT_FALSE(c.select(ros::Time(10.1),ros::Time(10.2),ros::SteadyTime(20.46)));
 auto reset=scan(10.2,"two",2);reset->observation.scan_sequence=0;c.accept(reset,ros::Time(10.2),ros::SteadyTime(20.3));
 EXPECT_EQ(reset,c.select(ros::Time(10.2),ros::Time(10.2),ros::SteadyTime(20.3)));
}
TEST(LocalSupport,UniqueBothWaysAndEmptyIsNotMatched){
 const cv::Matx33d K(100,0,100,0,100,100,0,0,1),R=cv::Matx33d::eye();const cv::Vec3d t(0,0,0);
 auto m=scan();m->observation.objects.objects.push_back(object());lane_boundary_adapter::CameraBBox b;
 b.xmin=b.ymin=80;b.xmax=b.ymax=120;b.label="traffic_drum";
 EXPECT_EQ(0,Support::associate({b},*m,K,R,t)[0]);
 auto many=Support::associate({b,b},*m,K,R,t);EXPECT_EQ(-1,many[0]);EXPECT_EQ(-1,many[1]);
 m->observation.objects.objects.push_back(object());EXPECT_EQ(-1,Support::associate({b},*m,K,R,t)[0]);
 m->observation.objects.objects.clear();EXPECT_EQ(-1,Support::associate({b},*m,K,R,t)[0]);
 m->observation.objects.objects.push_back(object());b.label="car";EXPECT_EQ(-1,Support::associate({b},*m,K,R,t)[0]);
}
TEST(LocalSupport,PredictionAndMalformedGeometryCannotSupport){
 auto o=object();geometry_msgs::Polygon p;EXPECT_TRUE(Support::footprint(o,p));EXPECT_EQ(4,p.points.size());
 EXPECT_NEAR(-1.25,p.points[0].x,1e-6);o.miss_count=1;EXPECT_FALSE(Support::footprint(o,p));
 o=object();o.obb_corners[0].z=NAN;EXPECT_FALSE(Support::footprint(o,p));
 o=object();o.source_point_count=2;EXPECT_FALSE(Support::footprint(o,p));
}

int main(int argc,char** argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
