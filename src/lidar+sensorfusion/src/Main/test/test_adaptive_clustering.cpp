#include <gtest/gtest.h>

#include <Lidar/Clustering/Clustering.hpp>

namespace {

LiDARClustering AdaptiveConfig() {
    LiDARClustering config{};
    config.b_Adaptive = true;
    config.f32_Eps = 0.5F;
    config.s32_MinPoints = 5;
    config.f32_AdaptiveNearRange = 8.0F;
    config.f32_AdaptiveFarRange = 50.0F;
    config.f32_AdaptiveNearEps = 0.55F;
    config.f32_AdaptiveFarEps = 0.85F;
    return config;
}

TEST(AdaptiveClustering, ClampsToleranceAtNearAndFarLimits) {
    const auto config = AdaptiveConfig();
    EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(0.0F, config), 0.55F);
    EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(8.0F, config), 0.55F);
    EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(50.0F, config), 0.85F);
    EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(80.0F, config), 0.85F);
}

TEST(AdaptiveClustering, InterpolatesWithoutGlobalLargeRadius) {
    const auto config = AdaptiveConfig();
    EXPECT_NEAR(AdaptiveClusterTolerance(29.0F, config), 0.70F, 1e-5F);
    EXPECT_LT(AdaptiveClusterTolerance(50.0F, config), 1.0F);
}

TEST(AdaptiveClustering, RetainsFixedRadiusWhenDisabled) {
    auto config = AdaptiveConfig();
    config.b_Adaptive = false;
    EXPECT_FLOAT_EQ(AdaptiveClusterTolerance(50.0F, config), 0.5F);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
