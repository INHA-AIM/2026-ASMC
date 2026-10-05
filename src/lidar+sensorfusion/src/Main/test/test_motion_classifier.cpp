#include <Lidar/Tracking/MotionClassifier.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace {

constexpr double kKphToMps = 1.0 / 3.6;

void UpdateObject(MotionClassifier& classifier, LiDARObjectInfo& object,
                  int frame, double vx_kph, double vy_kph,
                  double map_e, double map_n, int miss_count = 0) {
    object.s32_Id = 7;
    object.s32_Age = frame;
    object.s32_MissCount = miss_count;
    object.f32_AbsVx = static_cast<float>(vx_kph * kKphToMps);
    object.f32_AbsVy = static_cast<float>(vy_kph * kKphToMps);
    object.f32_Velocity = static_cast<float>(
        std::hypot(vx_kph, vy_kph) * kKphToMps);
    object.f32_CoreE = static_cast<float>(map_e);
    object.f32_CoreN = static_cast<float>(map_n);
    std::vector<LiDARObjectInfo> objects{object};
    classifier.Update(objects, ros::Time(0.1 * frame));
    object = objects.front();
}

void SetAxisAlignedObb(LiDARObjectInfo& object,
                       double min_x, double max_x,
                       double min_y, double max_y) {
    const double z_values[2] = {0.0, 1.0};
    int index = 0;
    for (double z : z_values) {
        object.OBB_Corners_Body[index++] =
            {static_cast<float>(min_x), static_cast<float>(min_y),
             static_cast<float>(z)};
        object.OBB_Corners_Body[index++] =
            {static_cast<float>(max_x), static_cast<float>(min_y),
             static_cast<float>(z)};
        object.OBB_Corners_Body[index++] =
            {static_cast<float>(max_x), static_cast<float>(max_y),
             static_cast<float>(z)};
        object.OBB_Corners_Body[index++] =
            {static_cast<float>(min_x), static_cast<float>(max_y),
             static_cast<float>(z)};
    }
}

void UpdateObjectWithObb(MotionClassifier& classifier,
                         LiDARObjectInfo& object, int frame,
                         double vx_kph, double vy_kph,
                         double min_x, double max_x,
                         double min_y, double max_y) {
    object.s32_Id = 7;
    object.s32_Age = frame;
    object.s32_MissCount = 0;
    object.f32_AbsVx = static_cast<float>(vx_kph * kKphToMps);
    object.f32_AbsVy = static_cast<float>(vy_kph * kKphToMps);
    object.f32_Velocity = static_cast<float>(
        std::hypot(vx_kph, vy_kph) * kKphToMps);
    object.f32_CoreE = static_cast<float>(0.5 * (min_x + max_x));
    object.f32_CoreN = static_cast<float>(0.5 * (min_y + max_y));
    SetAxisAlignedObb(object, min_x, max_x, min_y, max_y);

    GPS_IMU gps_imu{};
    VehicleSpec vehicle{};
    std::vector<LiDARObjectInfo> objects{object};
    classifier.Update(objects, ros::Time(0.1 * frame), gps_imu, vehicle);
    object = objects.front();
}

MotionState StateOf(const LiDARObjectInfo& object) {
    return static_cast<MotionState>(object.s32_MotionState);
}

TEST(MotionClassifier, WarmupDoesNotOverwriteRawVelocity) {
    MotionClassifier classifier;
    LiDARObjectInfo object{};
    for (int frame = 1; frame <= 3; ++frame) {
        UpdateObject(classifier, object, frame, 45.0, 12.0, 1.0, 2.0);
        EXPECT_EQ(StateOf(object), MotionState::Unknown);
        EXPECT_FALSE(object.b_IsDynamic);
    }
    EXPECT_NEAR(object.f32_AbsVx * 3.6, 45.0, 1e-4);
    EXPECT_NEAR(object.f32_AbsVy * 3.6, 12.0, 1e-4);
    EXPECT_NEAR(object.f32_Velocity * 3.6,
                std::hypot(45.0, 12.0), 1e-4);
}

TEST(MotionClassifier, LateralFastPathIsPendingThenDynamic) {
    MotionClassifier classifier;
    LiDARObjectInfo object{};
    for (int frame = 1; frame <= 3; ++frame) {
        UpdateObject(classifier, object, frame, 0.0, 0.0, 0.0, 0.0);
    }
    UpdateObject(classifier, object, 4, 1.0, 13.0, 0.0, 0.30);
    EXPECT_EQ(StateOf(object), MotionState::DynamicPending);
    EXPECT_TRUE(object.b_IsDynamic);

    for (int frame = 5; frame <= 10; ++frame) {
        UpdateObject(classifier, object, frame, 1.0, 13.0,
                     0.0, 0.30 * (frame - 3));
    }
    EXPECT_EQ(StateOf(object), MotionState::Dynamic);
    EXPECT_TRUE(object.b_IsDynamic);
}

TEST(MotionClassifier, StaticLongitudinalBiasNeedsMapDisplacement) {
    MotionClassifier classifier;
    LiDARObjectInfo object{};
    for (int frame = 1; frame <= 3; ++frame) {
        UpdateObject(classifier, object, frame, 50.0, 0.0, 10.0, 20.0);
    }
    for (int frame = 4; frame <= 10; ++frame) {
        UpdateObject(classifier, object, frame, 11.0, 0.2, 10.0, 20.0);
    }
    EXPECT_EQ(StateOf(object), MotionState::LowThreat);
    EXPECT_FALSE(object.b_IsDynamic);
    EXPECT_NEAR(object.f32_DecisionVxKPH, 11.0, 1e-4);
    EXPECT_NEAR(object.f32_MapDisplacementSpeedKPH, 0.0, 1e-4);
}

TEST(MotionClassifier, ConflictingSubThresholdMotionRemainsUnknown) {
    MotionClassifier classifier;
    LiDARObjectInfo object{};
    double map_e = 0.0;
    for (int frame = 1; frame <= 9; ++frame) {
        map_e += 8.0 * kKphToMps * 0.1;
        UpdateObject(classifier, object, frame, 8.0, 2.0, map_e, 0.0);
    }
    EXPECT_GT(object.f32_DecisionSpeedKPH, 7.0);
    EXPECT_EQ(StateOf(object), MotionState::Unknown);
    EXPECT_FALSE(object.b_IsDynamic);
}

TEST(MotionClassifier, AppliesAbsoluteValueAfterSignedMedian) {
    MotionClassifierConfig config;
    config.velocity_jump_kph = 30.0;
    MotionClassifier classifier(config);
    LiDARObjectInfo object{};
    for (int frame = 1; frame <= 3; ++frame) {
        UpdateObject(classifier, object, frame, 0.0, 0.0, 0.0, 0.0);
    }
    const double signed_vx[] = {-11.0, -11.0, 0.0, 11.0, 11.0};
    double map_e = 0.0;
    for (int i = 0; i < 5; ++i) {
        map_e += std::abs(signed_vx[i]) * kKphToMps * 0.1;
        UpdateObject(classifier, object, 4 + i, signed_vx[i], 0.0,
                     map_e, 0.0);
    }
    EXPECT_NEAR(object.f32_DecisionVxKPH, 0.0, 1e-4);
    EXPECT_EQ(StateOf(object), MotionState::DynamicPending);
    EXPECT_TRUE(object.b_IsDynamic);
}

TEST(MotionClassifier, SustainedLongitudinalMotionUsesFiveSamples) {
    MotionClassifier classifier;
    LiDARObjectInfo object{};
    double map_e = 0.0;
    for (int frame = 1; frame <= 3; ++frame) {
        UpdateObject(classifier, object, frame, 0.0, 0.0, map_e, 0.0);
    }
    for (int frame = 4; frame <= 10; ++frame) {
        map_e += 11.0 * kKphToMps * 0.1;
        UpdateObject(classifier, object, frame, 11.0, 0.0, map_e, 0.0);
    }
    EXPECT_EQ(object.s32_MotionAcceptedSamples, 5);
    EXPECT_EQ(StateOf(object), MotionState::Dynamic);
    EXPECT_TRUE(object.b_IsDynamic);
    EXPECT_NEAR(object.f32_MapDisplacementSpeedKPH, 11.0, 1e-3);
}

TEST(MotionClassifier, OneFrameLateralSpikeRestoresLowThreat) {
    MotionClassifier classifier;
    LiDARObjectInfo object{};
    for (int frame = 1; frame <= 10; ++frame) {
        UpdateObject(classifier, object, frame, 6.0, 0.0, 4.0, 8.0);
    }
    ASSERT_EQ(StateOf(object), MotionState::LowThreat);

    UpdateObject(classifier, object, 11, 6.0, 15.0, 4.0, 8.0);
    EXPECT_EQ(StateOf(object), MotionState::DynamicPending);
    EXPECT_EQ(object.s32_MotionCandidateSamples, 1);

    for (int frame = 12; frame <= 14; ++frame) {
        UpdateObject(classifier, object, frame, 6.0, 0.0, 4.0, 8.0);
    }
    EXPECT_EQ(StateOf(object), MotionState::LowThreat);
    EXPECT_FALSE(object.b_IsDynamic);
    EXPECT_EQ(object.s32_MotionCandidateSamples, 0);
}

TEST(MotionClassifier, ThreeConsistentJumpsReplaceHistory) {
    MotionClassifier classifier;
    LiDARObjectInfo object{};
    for (int frame = 1; frame <= 10; ++frame) {
        UpdateObject(classifier, object, frame, 2.0, 0.0, 0.0, 0.0);
    }
    ASSERT_EQ(StateOf(object), MotionState::LowThreat);

    UpdateObject(classifier, object, 11, 2.0, 15.0, 0.0, 0.4);
    EXPECT_EQ(object.s32_MotionCandidateSamples, 1);
    UpdateObject(classifier, object, 12, 2.0, 17.0, 0.0, 0.8);
    EXPECT_EQ(StateOf(object), MotionState::DynamicPending);
    EXPECT_EQ(object.s32_MotionCandidateSamples, 2);
    UpdateObject(classifier, object, 13, 3.0, 20.0, 0.0, 1.3);
    EXPECT_EQ(object.s32_MotionCandidateSamples, 0);
    EXPECT_EQ(object.s32_MotionAcceptedSamples, 3);
    EXPECT_NEAR(object.f32_DecisionVyKPH, 17.0, 1e-4);
    UpdateObject(classifier, object, 14, 3.0, 20.0, 0.0, 1.8);
    UpdateObject(classifier, object, 15, 3.0, 20.0, 0.0, 2.3);
    EXPECT_EQ(StateOf(object), MotionState::Dynamic);
}

TEST(MotionClassifier, SameDirectionButDivergingJumpsDoNotReplaceHistory) {
    MotionClassifier classifier;
    LiDARObjectInfo object{};
    for (int frame = 1; frame <= 10; ++frame) {
        UpdateObject(classifier, object, frame, 2.0, 0.0, 0.0, 0.0);
    }
    ASSERT_EQ(StateOf(object), MotionState::LowThreat);

    // 모두 +y 방향이지만 새 후보 간 크기 차가 10 km/h를
    // 넘으므로 3개의 수렴한 후보로 보면 안 된다.
    UpdateObject(classifier, object, 11, 2.0, 15.0, 0.0, 0.4);
    EXPECT_EQ(object.s32_MotionCandidateSamples, 1);
    UpdateObject(classifier, object, 12, 2.0, 35.0, 0.0, 1.2);
    EXPECT_EQ(object.s32_MotionCandidateSamples, 1);
    UpdateObject(classifier, object, 13, 2.0, 60.0, 0.0, 2.8);
    EXPECT_EQ(object.s32_MotionCandidateSamples, 1);
    EXPECT_EQ(object.s32_MotionAcceptedSamples, 5);
    EXPECT_NEAR(object.f32_DecisionVyKPH, 0.0, 1e-4);
}

TEST(MotionClassifier, RawVelocityStaysCurrentWhileDecisionUsesRecentFive) {
    MotionClassifier classifier;
    LiDARObjectInfo object{};
    for (int frame = 1; frame <= 3; ++frame) {
        UpdateObject(classifier, object, frame, 0.0, 0.0, 0.0, 0.0);
    }
    for (int frame = 4; frame <= 23; ++frame) {
        UpdateObject(classifier, object, frame, 4.0, 0.0,
                     0.1 * frame, 0.0);
    }
    ASSERT_EQ(object.s32_MotionAcceptedSamples, 5);

    UpdateObject(classifier, object, 24, 30.0, 0.0, 2.4, 0.0);
    EXPECT_NEAR(object.f32_AbsVx * 3.6, 30.0, 1e-4);
    EXPECT_NEAR(object.f32_Velocity * 3.6, 30.0, 1e-4);
    EXPECT_NEAR(object.f32_DecisionVxKPH, 4.0, 1e-4);
    EXPECT_EQ(object.s32_MotionAcceptedSamples, 5);
    EXPECT_EQ(object.s32_MotionCandidateSamples, 1);
}

TEST(MotionClassifier, ShortMissKeepsHistoryAndReobservationUsesJumpGate) {
    MotionClassifier classifier;
    LiDARObjectInfo object{};
    for (int frame = 1; frame <= 10; ++frame) {
        UpdateObject(classifier, object, frame, 2.0, 0.0, 0.0, 0.0);
    }
    ASSERT_EQ(object.s32_MotionAcceptedSamples, 5);
    ASSERT_EQ(StateOf(object), MotionState::LowThreat);

    UpdateObject(classifier, object, 11, 2.0, 0.0, 0.0, 0.0, 1);
    EXPECT_EQ(object.s32_MotionAcceptedSamples, 5);
    EXPECT_EQ(StateOf(object), MotionState::LowThreat);

    UpdateObject(classifier, object, 12, 20.0, 0.0, 0.5, 0.0);
    EXPECT_NEAR(object.f32_AbsVx * 3.6, 20.0, 1e-4);
    EXPECT_NEAR(object.f32_DecisionVxKPH, 2.0, 1e-4);
    EXPECT_EQ(object.s32_MotionAcceptedSamples, 5);
    EXPECT_EQ(object.s32_MotionCandidateSamples, 1);
}

TEST(MotionClassifier, LongMissResetsOnlyDecisionHistory) {
    MotionClassifier classifier;
    LiDARObjectInfo object{};
    for (int frame = 1; frame <= 10; ++frame) {
        UpdateObject(classifier, object, frame, 4.0, 0.0, 0.0, 0.0);
    }
    ASSERT_EQ(StateOf(object), MotionState::LowThreat);

    for (int miss = 1; miss <= 3; ++miss) {
        UpdateObject(classifier, object, 10 + miss, 4.0, 0.0,
                     0.0, 0.0, miss);
        EXPECT_EQ(StateOf(object), MotionState::LowThreat);
    }
    UpdateObject(classifier, object, 14, 4.0, 0.0, 0.0, 0.0, 4);
    EXPECT_EQ(StateOf(object), MotionState::Unknown);
    EXPECT_EQ(object.s32_MotionAcceptedSamples, 0);

    UpdateObject(classifier, object, 15, 4.0, 0.0, 0.0, 0.0);
    EXPECT_EQ(StateOf(object), MotionState::Unknown);
    EXPECT_EQ(object.s32_MotionValidObservations, 1);
}

TEST(MotionClassifier, ObbGrowthDoesNotBecomeRigidMotion) {
    MotionClassifier classifier;
    LiDARObjectInfo object{};
    for (int frame = 1; frame <= 3; ++frame) {
        UpdateObjectWithObb(classifier, object, frame, 12.0, 0.0,
                            10.0, 11.0, -0.5, 0.5);
    }
    for (int frame = 4; frame <= 10; ++frame) {
        const double growing_max_x = 11.0 + 0.40 * (frame - 4);
        UpdateObjectWithObb(classifier, object, frame, 12.0, 0.0,
                            10.0, growing_max_x, -0.5, 0.5);
    }

    EXPECT_GT(object.f32_MapDisplacementSpeedKPH, 5.0);
    EXPECT_NEAR(object.f32_CorrectedDisplacementSpeedKPH, 0.0, 1e-4);
    EXPECT_EQ(StateOf(object), MotionState::LowThreat);
    EXPECT_FALSE(object.b_IsDynamic);
    EXPECT_NEAR(object.f32_PlannerVxKPH, 0.0, 1e-4);
}

TEST(MotionClassifier, RigidObbTranslationBecomesDynamic) {
    MotionClassifier classifier;
    LiDARObjectInfo object{};
    constexpr double kStep = 12.0 * kKphToMps * 0.1;
    for (int frame = 1; frame <= 3; ++frame) {
        UpdateObjectWithObb(classifier, object, frame, 12.0, 0.0,
                            10.0, 12.0, -0.5, 0.5);
    }
    for (int frame = 4; frame <= 10; ++frame) {
        const double shift = kStep * (frame - 4);
        UpdateObjectWithObb(classifier, object, frame, 12.0, 0.0,
                            10.0 + shift, 12.0 + shift, -0.5, 0.5);
    }

    EXPECT_NEAR(object.f32_CorrectedDisplacementSpeedKPH, 12.0, 1e-3);
    EXPECT_EQ(StateOf(object), MotionState::Dynamic);
    EXPECT_TRUE(object.b_IsDynamic);
    EXPECT_NEAR(object.f32_PlannerVxKPH, 12.0, 1e-3);
}

TEST(MotionClassifier, AmbiguousPendingFallsBackToUnknownNotStatic) {
    MotionClassifier classifier;
    LiDARObjectInfo object{};
    double map_e = 0.0;
    for (int frame = 1; frame <= 3; ++frame) {
        UpdateObject(classifier, object, frame, 0.0, 0.0, map_e, 0.0);
    }
    for (int frame = 4; frame <= 12; ++frame) {
        map_e += 4.0 * kKphToMps * 0.1;
        UpdateObject(classifier, object, frame, 12.0, 0.0, map_e, 0.0);
    }

    EXPECT_EQ(StateOf(object), MotionState::Unknown);
    EXPECT_FALSE(object.b_IsDynamic);
    EXPECT_GT(object.f32_PlannerVxKPH, 0.0);
}

TEST(MotionClassifier, ConfirmedDynamicNeedsEightStationaryFramesToRelease) {
    MotionClassifier classifier;
    LiDARObjectInfo object{};
    double map_e = 0.0;
    for (int frame = 1; frame <= 3; ++frame) {
        UpdateObject(classifier, object, frame, 0.0, 0.0, map_e, 0.0);
    }
    for (int frame = 4; frame <= 10; ++frame) {
        map_e += 12.0 * kKphToMps * 0.1;
        UpdateObject(classifier, object, frame, 12.0, 0.0, map_e, 0.0);
    }
    ASSERT_EQ(StateOf(object), MotionState::Dynamic);

    for (int frame = 11; frame <= 20; ++frame) {
        UpdateObject(classifier, object, frame, 0.0, 0.0, map_e, 0.0);
    }
    EXPECT_EQ(StateOf(object), MotionState::Dynamic);
    EXPECT_TRUE(object.b_IsDynamic);

    UpdateObject(classifier, object, 21, 0.0, 0.0, map_e, 0.0);
    EXPECT_EQ(StateOf(object), MotionState::LowThreat);
    EXPECT_FALSE(object.b_IsDynamic);
}

TEST(MotionClassifier, StationaryBoundsRemainStaticDespiteFastTrackerVelocity) {
    MotionClassifier classifier;
    LiDARObjectInfo object{};
    for (int frame = 1; frame <= 40; ++frame) {
        UpdateObjectWithObb(classifier, object, frame, 30.0, 0.0,
                            10.0, 12.0, -0.5, 0.5);
        if (frame >= 10) {
            EXPECT_EQ(StateOf(object), MotionState::LowThreat) << frame;
            EXPECT_FALSE(object.b_IsDynamic) << frame;
            EXPECT_NEAR(object.f32_PlannerVxKPH, 0.0, 1e-4) << frame;
        }
    }
    // Real rigid translation must still upgrade the same track to DYNAMIC.
    for (int frame = 41; frame <= 55; ++frame) {
        const double shift = (frame - 40) * 30.0 * kKphToMps * 0.1;
        UpdateObjectWithObb(classifier, object, frame, 30.0, 0.0,
                            10.0 + shift, 12.0 + shift, -0.5, 0.5);
    }
    EXPECT_EQ(StateOf(object), MotionState::Dynamic);
    EXPECT_TRUE(object.b_IsDynamic);
}

}  // namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
