#include <gtest/gtest.h>
#include <Lidar/Tracking/ObservationModel.hpp>
#include <Eigen/Eigenvalues>
#include <Lidar/Kalman/kalman_utils.hpp>
#include <random>

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
TEST(ObservationModel, TwoObservationCovarianceMatchesIndependentNoiseSampling) {
    tracking_options::Get()=tracking_options::Options{};tracking_options::Get().two_point_init_cov=true;
    LiDARObjectInfo o{};kalman_filter_update(o,3.f,2.f,0,0,.1,0);
    kalman_filter_update(o,3.1f,2.f,3.f,2.f,.1,0);
    EXPECT_EQ(o.s32_KfInitCount,2);EXPECT_FLOAT_EQ(o.Eigen_KfState_CA(0),3.1f);
    std::mt19937 rng(731);std::normal_distribution<double> normal(0,1);
    Eigen::Vector3d mean=Eigen::Vector3d::Zero();Eigen::Matrix3d second=Eigen::Matrix3d::Zero();
    for(int k=0;k<50000;++k) {
        const double e0=.1*normal(rng),e1=.1*normal(rng),a=normal(rng);
        Eigen::Vector3d sample(e1,(e1-e0)/.1+.05*a,a);mean+=sample;second+=sample*sample.transpose();
    }
    mean/=50000.;const Eigen::Matrix3d sampled=second/50000.-mean*mean.transpose();
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)EXPECT_NEAR(sampled(i,j),o.Eigen_KfP_CA(2*i,2*j),.04);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float,6,6>> eigen(o.Eigen_KfP_CA);
    EXPECT_GT(eigen.eigenvalues().minCoeff(),0.f);
}
TEST(ObservationModel, FirstMeasurementCovarianceFollowsEgoAxes) {
    tracking_options::Get()=tracking_options::Options{};tracking_options::Get().two_point_init_cov=true;
    LiDARObjectInfo o{};o.observation_R_valid=true;o.observation_R<<.01f,0,0,.04f;
    kalman_filter_update(o,3.f,2.f,0,0,.1,0);
    o.observation_R<<.04f,0,0,.01f;
    kalman_filter_update(o,2.1f,-3.f,2.f,-3.f,.1,90);
    EXPECT_NEAR(o.Eigen_KfP_CA(2,2),8.0025,.001);
    EXPECT_NEAR(o.Eigen_KfP_CA(3,3),2.0025,.001);
    EXPECT_NEAR(o.Eigen_KfP_CA(0,2),.4,.0001);
}
}
int main(int argc,char**argv){testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
