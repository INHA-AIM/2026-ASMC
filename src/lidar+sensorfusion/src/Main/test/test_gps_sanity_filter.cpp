#include <Parser/GpsSanityFilter.hpp>

#include <gtest/gtest.h>

TEST(GpsSanityFilter, RejectsZeroCoordinateProjectionOutsideMap) {
    GpsSanityFilter filter;
    EXPECT_FALSE(filter.Evaluate(-136000.0, -4124000.0, 10.0).accepted);
    EXPECT_TRUE(filter.Evaluate(-120.0, -410.0, 10.1).accepted);
}

TEST(GpsSanityFilter, RejectsAlternatingHundredMeterJump) {
    GpsSanityFilter filter;
    EXPECT_TRUE(filter.Evaluate(-60.0, 100.0, 10.0).accepted);
    const auto bad = filter.Evaluate(120.0, 100.0, 10.1);
    EXPECT_FALSE(bad.accepted);
    EXPECT_EQ("implausible position jump", bad.reason);
    EXPECT_TRUE(filter.Evaluate(-58.0, 100.0, 10.2).accepted);
}

TEST(GpsSanityFilter, AllowsFastButPlausibleMotion) {
    GpsSanityFilter filter;
    EXPECT_TRUE(filter.Evaluate(-100.0, -400.0, 10.0).accepted);
    EXPECT_TRUE(filter.Evaluate(-88.0, -400.0, 10.2).accepted);
}

TEST(GpsSanityFilter, ReanchorsAfterSustainedGapForRespawn) {
    GpsSanityFilter filter;
    EXPECT_TRUE(filter.Evaluate(-100.0, -400.0, 10.0).accepted);
    EXPECT_FALSE(filter.Evaluate(100.0, 300.0, 10.2).accepted);
    EXPECT_TRUE(filter.Evaluate(100.0, 300.0, 12.1).accepted);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
