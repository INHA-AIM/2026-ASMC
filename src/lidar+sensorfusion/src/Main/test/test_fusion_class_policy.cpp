#include <SensorFusion/FusionClassPolicy.hpp>

#include <gtest/gtest.h>

TEST(FusionClassPolicy, NormalizesToThreeOutputClasses) {
    EXPECT_EQ("person", fusion_class::Normalize("Person"));
    EXPECT_EQ("car", fusion_class::Normalize("car"));
    EXPECT_EQ("car", fusion_class::Normalize("TRUCK"));
    EXPECT_EQ("car", fusion_class::Normalize("bus"));
    EXPECT_EQ("unknown", fusion_class::Normalize("bicycle"));
}

TEST(FusionClassPolicy, InitialClassNeedsFiveConsistentObservations) {
    LiDARObjectInfo object;
    for (int i = 0; i < 4; ++i) {
        fusion_class::Observe(object, "truck", 5, 3);
        EXPECT_EQ("unknown", object.str_ObjectClass);
    }
    fusion_class::Observe(object, "truck", 5, 3);
    EXPECT_EQ("car", object.str_ObjectClass);
}

TEST(FusionClassPolicy, ConfirmedClassChangesAfterThreeOppositeObservations) {
    LiDARObjectInfo object;
    object.str_ObjectClass = "car";
    fusion_class::Observe(object, "person", 5, 3);
    fusion_class::Observe(object, "person", 5, 3);
    EXPECT_EQ("car", object.str_ObjectClass);
    fusion_class::Observe(object, "person", 5, 3);
    EXPECT_EQ("person", object.str_ObjectClass);
}

TEST(FusionClassPolicy, ThreeMissesKeepClassAndFourthResetsIt) {
    LiDARObjectInfo object;
    object.str_ObjectClass = "person";
    for (int i = 0; i < 3; ++i) {
        fusion_class::Miss(object, 3);
        EXPECT_EQ("person", object.str_ObjectClass);
    }
    fusion_class::Miss(object, 3);
    EXPECT_EQ("unknown", object.str_ObjectClass);
}

TEST(FusionClassPolicy, UnsupportedObservationDoesNotEraseConfirmedClass) {
    LiDARObjectInfo object;
    object.str_ObjectClass = "car";
    fusion_class::Observe(object, "bicycle", 5, 3);
    EXPECT_EQ("car", object.str_ObjectClass);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
