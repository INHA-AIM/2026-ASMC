#include <SensorFusion/FusionClassPolicy.hpp>

#include <gtest/gtest.h>

TEST(FusionClassPolicy, NormalizesSevenLabelsWithLegacyVehicleCategory) {
    EXPECT_EQ("person", fusion_class::Normalize("Person"));
    EXPECT_EQ("car", fusion_class::Normalize("car"));
    EXPECT_EQ("car", fusion_class::Normalize("TRUCK"));
    EXPECT_EQ("car", fusion_class::Normalize("bus"));
    EXPECT_EQ("bicycle", fusion_class::Normalize("bicycle"));
}

TEST(FusionClassPolicy, InitialClassNeedsFiveConsistentObservations) {
    LiDARObjectInfo object;
    for (int i = 0; i < 4; ++i) {
        fusion_class::Observe(object, "truck", "truck", 5, 3);
        EXPECT_EQ("unknown", object.str_ObjectClass);
    }
    fusion_class::Observe(object, "truck", "truck", 5, 3);
    EXPECT_EQ("car", object.str_ObjectClass);
    EXPECT_EQ("truck", object.str_YoloClass);
}

TEST(FusionClassPolicy, ConfirmedClassChangesAfterThreeOppositeObservations) {
    LiDARObjectInfo object;
    object.str_ObjectClass = "car";
    fusion_class::Observe(object, "person", "person", 5, 3);
    fusion_class::Observe(object, "person", "person", 5, 3);
    EXPECT_EQ("car", object.str_ObjectClass);
    fusion_class::Observe(object, "person", "person", 5, 3);
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
    fusion_class::Observe(object, "train", "train", 5, 3);
    EXPECT_EQ("car", object.str_ObjectClass);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(FusionClassPolicy, SevenLabelsAndRepeatedSourceCannotConfirm) {
    for (const auto& label : {"person", "dog", "bicycle", "motorcycle", "car", "truck", "bus"}) {
        LiDARObjectInfo object;
        EXPECT_NE("unknown", fusion_class::Normalize(label));
        ASSERT_TRUE(fusion_class::BeginObservation(object, 100, 1));
        fusion_class::Observe(object, label, label, 2, 3);
        EXPECT_FALSE(fusion_class::BeginObservation(object, 100, 1));
        EXPECT_FALSE(fusion_class::BeginObservation(object, 99, 1));
        EXPECT_EQ(1, object.continuity_counter);
        ASSERT_TRUE(fusion_class::BeginObservation(object, 200, 1));
        fusion_class::Observe(object, label, label, 2, 3);
        EXPECT_EQ(label, object.str_YoloClass);
        EXPECT_TRUE(fusion_class::IsConfirmed(object.str_ObjectClass));
        ASSERT_TRUE(fusion_class::BeginObservation(object, 201, 2));
        EXPECT_EQ(0, object.class_miss_counter);
    }
}

TEST(FusionClassPolicy, EpochInvalidationRemovesOldClassWithoutResettingGeometryOrTrack) {
    LiDARObjectInfo object;
    object.s32_Id = 12;
    object.str_ObjectClass = "car";
    object.str_YoloClass = "truck";
    object.continuity_counter = 4;
    object.class_miss_counter = 2;
    object.camera_observation_ns = 300;
    fusion_class::InvalidateObservation(object, 2);
    EXPECT_EQ(12, object.s32_Id);
    EXPECT_EQ("unknown", object.str_ObjectClass);
    EXPECT_EQ("unknown", object.str_YoloClass);
    EXPECT_EQ(0, object.continuity_counter);
    EXPECT_EQ(0, object.class_miss_counter);
    EXPECT_EQ(0u, object.camera_observation_ns);
    EXPECT_EQ(2u, object.camera_observation_epoch);
    ASSERT_TRUE(fusion_class::BeginObservation(object, 400, 2));
    fusion_class::Observe(object, "truck", "truck", 5, 3);
    EXPECT_EQ("unknown", object.str_ObjectClass);
    object.str_ObjectClass = "noise";
    fusion_class::InvalidateObservation(object, 3);
    EXPECT_EQ("noise", object.str_ObjectClass);
}
