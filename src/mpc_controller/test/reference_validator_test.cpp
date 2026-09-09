#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "reference/reference_validator.hpp"

namespace {

MPCParams MakeParams() {
    MPCParams params;
    params.dt = 0.1;
    params.wheelbase = 3.0;
    params.steering_max = 0.63268;  // ~36.25 deg, production value
    params.vel_min = 0.0;
    params.vel_max = 30.0;
    return params;
}

ReferencePath MakeStraightReference(double speed, int steps, double dt) {
    ReferencePath ref;
    for (int i = 0; i < steps; ++i) {
        ref.x_ref.push_back(speed * dt * i);
        ref.y_ref.push_back(0.0);
        ref.yaw_ref.push_back(0.0);
        ref.v_ref.push_back(speed);
        ref.k_ref.push_back(0.0);
    }
    return ref;
}

ReferencePath MakeCurvedReference(double speed, double kappa, int steps, double dt) {
    ReferencePath ref;
    double x = 0.0, y = 0.0, yaw = 0.0;
    for (int i = 0; i < steps; ++i) {
        ref.x_ref.push_back(x);
        ref.y_ref.push_back(y);
        ref.yaw_ref.push_back(yaw);
        ref.v_ref.push_back(speed);
        ref.k_ref.push_back(kappa);
        const double ds = speed * dt;
        x += ds * std::cos(yaw);
        y += ds * std::sin(yaw);
        yaw += ds * kappa;
    }
    return ref;
}

}  // namespace

// Test 1: valid straight reference -> PASS.
TEST(ReferenceValidatorTest, ValidStraightReferencePasses) {
    const MPCParams params = MakeParams();
    const ReferencePath ref = MakeStraightReference(9.0, 20, params.dt);
    const auto result = ValidateReference(ref, params);
    EXPECT_TRUE(result.ok());
}

// Test 2: valid curved reference (well inside the steering-derived curvature
// bound) -> PASS.
TEST(ReferenceValidatorTest, ValidCurvedReferencePasses) {
    const MPCParams params = MakeParams();
    const double kappa_max = std::tan(params.steering_max) / params.wheelbase;
    const ReferencePath ref =
        MakeCurvedReference(9.0, kappa_max * 0.5, 20, params.dt);
    const auto result = ValidateReference(ref, params);
    EXPECT_TRUE(result.ok());
}

// Test 3: a Commit-F-like seam fixture. MPC only ever sees Cartesian
// ReferencePath points - a Frenet s wrap is a planner-internal concept that
// never appears in this struct. The important invariant is that adjacent
// Cartesian points remain physically continuous across what would have been
// the seam; the validator must not know or care about lap count / s.
// We build the fixture as a closed circular loop sampled past its "restart"
// point, i.e. the trajectory keeps advancing physically even though a
// Frenet-s tracker would have wrapped back to a small value at this point.
TEST(ReferenceValidatorTest, CorrectedSeamLikeReferencePasses) {
    const MPCParams params = MakeParams();
    // Large-radius circle: curvature well inside the steering-derived bound,
    // physically continuous point-to-point motion "through" a seam.
    const double radius = 50.0;
    const double kappa = 1.0 / radius;
    ASSERT_LT(kappa, std::tan(params.steering_max) / params.wheelbase);
    const ReferencePath ref = MakeCurvedReference(8.5, kappa, 30, params.dt);
    const auto result = ValidateReference(ref, params);
    EXPECT_TRUE(result.ok());
}

// Test 4: empty -> REJECT.
TEST(ReferenceValidatorTest, EmptyReferenceIsRejected) {
    const MPCParams params = MakeParams();
    const ReferencePath ref;
    const auto result = ValidateReference(ref, params);
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(ReferenceValidationStatus::EMPTY, result.status);
}

// Test 5: x/y/yaw/v/k vector size mismatch -> REJECT.
TEST(ReferenceValidatorTest, SizeMismatchIsRejected) {
    const MPCParams params = MakeParams();
    ReferencePath ref = MakeStraightReference(9.0, 10, params.dt);
    ref.y_ref.pop_back();  // now shorter than x_ref
    const auto result = ValidateReference(ref, params);
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(ReferenceValidationStatus::SIZE_MISMATCH, result.status);
}

// Test 6: NaN x -> REJECT.
TEST(ReferenceValidatorTest, NanXIsRejected) {
    const MPCParams params = MakeParams();
    ReferencePath ref = MakeStraightReference(9.0, 10, params.dt);
    ref.x_ref[3] = std::numeric_limits<double>::quiet_NaN();
    const auto result = ValidateReference(ref, params);
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(ReferenceValidationStatus::NON_FINITE, result.status);
    EXPECT_EQ(3u, result.failed_index);
}

// Test 7: NaN y -> REJECT.
TEST(ReferenceValidatorTest, NanYIsRejected) {
    const MPCParams params = MakeParams();
    ReferencePath ref = MakeStraightReference(9.0, 10, params.dt);
    ref.y_ref[2] = std::numeric_limits<double>::quiet_NaN();
    const auto result = ValidateReference(ref, params);
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(ReferenceValidationStatus::NON_FINITE, result.status);
}

// Test 8: NaN yaw -> REJECT.
TEST(ReferenceValidatorTest, NanYawIsRejected) {
    const MPCParams params = MakeParams();
    ReferencePath ref = MakeStraightReference(9.0, 10, params.dt);
    ref.yaw_ref[4] = std::numeric_limits<double>::quiet_NaN();
    const auto result = ValidateReference(ref, params);
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(ReferenceValidationStatus::NON_FINITE, result.status);
}

// Test 9: NaN velocity -> REJECT.
TEST(ReferenceValidatorTest, NanVelocityIsRejected) {
    const MPCParams params = MakeParams();
    ReferencePath ref = MakeStraightReference(9.0, 10, params.dt);
    ref.v_ref[1] = std::numeric_limits<double>::quiet_NaN();
    const auto result = ValidateReference(ref, params);
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(ReferenceValidationStatus::NON_FINITE, result.status);
}

// Test 10: NaN curvature -> REJECT.
TEST(ReferenceValidatorTest, NanCurvatureIsRejected) {
    const MPCParams params = MakeParams();
    ReferencePath ref = MakeStraightReference(9.0, 10, params.dt);
    ref.k_ref[5] = std::numeric_limits<double>::quiet_NaN();
    const auto result = ValidateReference(ref, params);
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(ReferenceValidationStatus::NON_FINITE, result.status);
}

// Test 11: +Inf / -Inf representative cases -> REJECT.
TEST(ReferenceValidatorTest, InfiniteValuesAreRejected) {
    const MPCParams params = MakeParams();
    {
        ReferencePath ref = MakeStraightReference(9.0, 10, params.dt);
        ref.x_ref[0] = std::numeric_limits<double>::infinity();
        EXPECT_FALSE(ValidateReference(ref, params).ok());
    }
    {
        ReferencePath ref = MakeStraightReference(9.0, 10, params.dt);
        ref.v_ref[0] = -std::numeric_limits<double>::infinity();
        EXPECT_FALSE(ValidateReference(ref, params).ok());
    }
}

// Test 12: velocity > vel_max -> REJECT.
TEST(ReferenceValidatorTest, VelocityAboveMaxIsRejected) {
    const MPCParams params = MakeParams();
    ReferencePath ref = MakeStraightReference(9.0, 10, params.dt);
    ref.v_ref[6] = params.vel_max + 5.0;
    const auto result = ValidateReference(ref, params);
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(ReferenceValidationStatus::VELOCITY_OUT_OF_RANGE, result.status);
}

// Test 13: velocity below supported minimum -> REJECT.
TEST(ReferenceValidatorTest, VelocityBelowMinIsRejected) {
    const MPCParams params = MakeParams();
    ReferencePath ref = MakeStraightReference(9.0, 10, params.dt);
    ref.v_ref[6] = params.vel_min - 1.0;
    const auto result = ValidateReference(ref, params);
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(ReferenceValidationStatus::VELOCITY_OUT_OF_RANGE, result.status);
}

// Test 14: curvature just inside the steering-derived bound -> PASS.
TEST(ReferenceValidatorTest, CurvatureJustInsideBoundPasses) {
    const MPCParams params = MakeParams();
    const double kappa_max = std::tan(params.steering_max) / params.wheelbase;
    const ReferencePath ref =
        MakeCurvedReference(5.0, kappa_max * 0.98, 10, params.dt);
    const auto result = ValidateReference(ref, params);
    EXPECT_TRUE(result.ok());
}

// Test 15: curvature clearly above the steering-derived bound -> REJECT.
TEST(ReferenceValidatorTest, CurvatureAboveBoundIsRejected) {
    const MPCParams params = MakeParams();
    const double kappa_max = std::tan(params.steering_max) / params.wheelbase;
    ReferencePath ref = MakeStraightReference(5.0, 10, params.dt);
    ref.k_ref[3] = kappa_max * 2.0;
    const auto result = ValidateReference(ref, params);
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(ReferenceValidationStatus::CURVATURE_OUT_OF_RANGE, result.status);
}

// Test 16: normal adjacent displacement at max supported speed -> PASS.
TEST(ReferenceValidatorTest, NormalDisplacementAtMaxSpeedPasses) {
    const MPCParams params = MakeParams();
    const ReferencePath ref =
        MakeStraightReference(params.vel_max, 10, params.dt);
    const auto result = ValidateReference(ref, params);
    EXPECT_TRUE(result.ok());
}

// Test 17: multi-meter single-step teleport -> REJECT. This is the direct
// regression for the seam-teleport failure mode Commit F fixed at the
// producer; Commit G is the defense-in-depth guard in case it recurs.
TEST(ReferenceValidatorTest, SingleStepTeleportIsRejected) {
    const MPCParams params = MakeParams();
    ReferencePath ref = MakeStraightReference(8.0, 10, params.dt);
    // Physically plausible step at 8 m/s, dt=0.1s is ~0.8m. Jump the point
    // several meters instead.
    ref.x_ref[5] += 10.0;
    const auto result = ValidateReference(ref, params);
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(ReferenceValidationStatus::POSITION_DISCONTINUITY, result.status);
    EXPECT_EQ(5u, result.failed_index);
}

// Test 18: yaw +pi -> -pi with otherwise valid geometry -> PASS. The
// validator only checks finiteness of yaw (not angle-wrapped continuity), so
// a raw +pi/-pi jump must not false-trigger any other rule.
TEST(ReferenceValidatorTest, YawWrapDoesNotFalseTrigger) {
    const MPCParams params = MakeParams();
    ReferencePath ref = MakeStraightReference(6.0, 6, params.dt);
    ref.yaw_ref[2] = M_PI;
    ref.yaw_ref[3] = -M_PI;
    const auto result = ValidateReference(ref, params);
    EXPECT_TRUE(result.ok());
}

// Test 19: low-speed, tightly spaced reference -> PASS. Small displacement
// at low speed must not be mistaken for a discontinuity.
TEST(ReferenceValidatorTest, LowSpeedTightlySpacedReferencePasses) {
    const MPCParams params = MakeParams();
    const ReferencePath ref = MakeStraightReference(0.3, 10, params.dt);
    const auto result = ValidateReference(ref, params);
    EXPECT_TRUE(result.ok());
}

// Test 20: current horizon-sized normal reference -> PASS. Uses the
// production horizon length (36, post-Commit-D) at production dt.
TEST(ReferenceValidatorTest, HorizonSizedNormalReferencePasses) {
    MPCParams params = MakeParams();
    params.horizon = 36;
    const ReferencePath ref =
        MakeStraightReference(9.0, params.horizon, params.dt);
    const auto result = ValidateReference(ref, params);
    EXPECT_TRUE(result.ok());
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
