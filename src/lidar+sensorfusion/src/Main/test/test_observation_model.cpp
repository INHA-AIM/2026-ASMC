#include <gtest/gtest.h>
#include <Lidar/Tracking/ObservationModel.hpp>
#include <Eigen/Eigenvalues>

namespace {
LiDARObjectInfo Box() {
    LiDARObjectInfo o{};o.f32_CoreX=8;o.f32_CoreY=2;
    o.f32_OBB_CenterX=10;o.f32_OBB_CenterY=2;o.f32_OBB_Width=4;o.f32_OBB_Height=2;o.f32_OBB_YawRad=0;
    for(int k=0;k<40;++k) {pcl::PointXYZI p;p.x=8+.1*k;p.y=1;p.z=0;o.pcl_ClusterPoints.push_back(p);}
    return o;
}
TEST(ObservationModel, DisabledIsNeutral) {
    tracking_options::Get()=tracking_options::Options{};auto o=Box();observation_model::Prepare(o);
    EXPECT_FLOAT_EQ(o.f32_CoreX,8);EXPECT_FALSE(o.observation_R_valid);
    EXPECT_EQ((observation_model::ProcessCovariance(.1)-Eigen::Matrix<float,6,6>::Identity()*.003f).norm(),0);
}
TEST(ObservationModel, ObbReferenceUsedAtPreparation) {
    tracking_options::Get()=tracking_options::Options{};tracking_options::Get().observation_obb_center=true;
    auto o=Box();observation_model::Prepare(o);EXPECT_FLOAT_EQ(o.f32_CoreX,10);EXPECT_FLOAT_EQ(o.f32_CoreY,2);
}
TEST(ObservationModel, CovarianceHasNonzeroModelFloorAndIsPositive) {
    auto o=Box();auto r=observation_model::QualityCovariance(o);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigen(r);EXPECT_GT(eigen.eigenvalues().minCoeff(),.01);
    for(int k=0;k<5;++k) {auto points=o.pcl_ClusterPoints;o.pcl_ClusterPoints+=points;}
    EXPECT_LT((observation_model::QualityCovariance(o)-r).norm(),1e-5);
}
TEST(ObservationModel, WhiteJerkHasCorrectOrderUnitsAndSemigroup) {
    tracking_options::Get()=tracking_options::Options{};tracking_options::Get().jerk_q=true;
    auto q=observation_model::ProcessCovariance(.2);EXPECT_NEAR(q(0,4),std::pow(.2,3)/6,1e-8);
    EXPECT_FLOAT_EQ(q(0,1),0);EXPECT_NEAR(q(4,4),.2,1e-7);
    Eigen::Matrix<float,6,6> f=Eigen::Matrix<float,6,6>::Identity();
    f(0,2)=f(1,3)=f(2,4)=f(3,5)=.1;f(0,4)=f(1,5)=.005;
    auto qh=observation_model::ProcessCovariance(.1);EXPECT_LT((f*qh*f.transpose()+qh-q).norm(),1e-7);
}
}
int main(int argc,char**argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
