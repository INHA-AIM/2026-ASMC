#include <asmc_lidar/detection/euclidean_components.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <gtest/gtest.h>
#include <algorithm>
#include <limits>
#include <numeric>
#include <random>
using namespace asmc::lidar::pipeline;
using Cloud=pcl::PointCloud<pcl::PointXYZI>;
namespace {
pcl::PointXYZI Point(float x,float y,float z) {pcl::PointXYZI p{};p.x=x;p.y=y;p.z=z;return p;}
void Compare(const Cloud& cloud,const std::vector<int>& indices,float radius,int min,int max) {
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> pcl;
  pcl.setInputCloud(cloud.makeShared());pcl.setIndices(pcl::IndicesPtr(new std::vector<int>(indices)));
  pcl.setSearchMethod(pcl::search::KdTree<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>()));
  pcl.setClusterTolerance(radius);pcl.setMinClusterSize(min);pcl.setMaxClusterSize(max);
  std::vector<pcl::PointIndices> expected;pcl.extract(expected);
  const auto actual=EuclideanComponents(cloud,indices,radius,min,max);
  ASSERT_EQ(expected.size(),actual.size());
  for(std::size_t i=0;i<expected.size();++i)EXPECT_EQ(expected[i].indices,actual[i].indices);
}
}
TEST(EuclideanComponents, SameComponentsAndTieOrderAsPclAcrossSparseAndDenseClouds) {
  std::mt19937 rng(20261027);std::uniform_real_distribution<float> jitter(-1.f,1.f);
  for(int trial=0;trial<60;++trial) {
    Cloud cloud;
    for(int i=0;i<600;++i)cloud.push_back(Point((i%4)*5+jitter(rng),jitter(rng),jitter(rng)));
    std::vector<int> indices;
    for(int i=0;i<600;++i)if(i%7)indices.push_back(i);
    std::shuffle(indices.begin(),indices.end(),rng);
    Compare(cloud,indices,trial%2?.7f:.15f,trial%3+1,trial%4?500:80);
  }
}
TEST(EuclideanComponents, RadiusBoundaryNegativeCellsAndDisconnectedEqualSizeComponents) {
  for(float distance:{std::nextafter(.5f,0.f),.5f,std::nextafter(.5f,1.f)}) {
    Cloud c;for(float offset:{-20.f,-4.f,0.f,4.f,20.f}) {
      c.push_back(Point(offset,0,0));c.push_back(Point(offset+distance,0,0));
    }
    std::vector<int> indices(c.size());std::iota(indices.begin(),indices.end(),0);
    Compare(c,indices,.5f,1,100);std::reverse(indices.begin(),indices.end());Compare(c,indices,.5f,1,100);
  }
}
TEST(EuclideanComponents, WallGridPointOrderAndClusterLimitsPreserved) {
  Cloud c;for(float x=-10;x<30;x+=.11f)for(float z=-1;z<2;z+=.13f)c.push_back(Point(x,4,z));
  c.push_back(Point(-30,-30,0));
  std::vector<int> indices(c.size());std::iota(indices.begin(),indices.end(),0);
  Compare(c,indices,.55f,10,20000);Compare(c,indices,.55f,1,100);
}
TEST(EuclideanComponents, InvalidInputsAndEmptyScan) {
  Cloud c;c.push_back(Point(0,0,0));
  EXPECT_TRUE(EuclideanComponents(c,{},.5f,1,100).empty());
  EXPECT_THROW(EuclideanComponents(c,{0,0},.5f,1,100),std::invalid_argument);
  EXPECT_THROW(EuclideanComponents(c,{1},.5f,1,100),std::invalid_argument);
  EXPECT_THROW(EuclideanComponents(c,{0},0,1,100),std::invalid_argument);
  c[0].x=std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(EuclideanComponents(c,{0},.5f,1,100),std::invalid_argument);
}
TEST(EuclideanComponents, ExtremeNumericScaleFallsBackWithoutCellOverflow) {
  Cloud c;c.push_back(Point(1e20f,0,0));c.push_back(Point(-1e20f,0,0));
  Compare(c,{0,1},1.f,1,100);
}
int main(int argc,char**argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
