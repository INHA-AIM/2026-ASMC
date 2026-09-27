#include <Lidar/TunnelFilter/TunnelFilterCore.hpp>

#include <gtest/gtest.h>

#include <cmath>

using namespace tunnel_filter;

namespace {

constexpr float kGroundZ = -1.70f;

pcl::PointXYZI MakePoint(float x, float y, float z, float intensity = 0.0f) {
    pcl::PointXYZI point;
    point.x = x;
    point.y = y;
    point.z = z;
    point.intensity = intensity;
    return point;
}

void AddGround(pcl::PointCloud<pcl::PointXYZI>* cloud, float z = kGroundZ) {
    for (float x = -10.0f; x <= 40.0f; x += 0.5f) {
        for (float y = -5.0f; y <= 5.5f; y += 0.5f) {
            cloud->points.push_back(MakePoint(x, y, z));
        }
    }
}

// y 위치의 수직 벽 (지면 위 0.3~3.0 m)
void AddWall(pcl::PointCloud<pcl::PointXYZI>* cloud, float y, float x_min,
             float x_max) {
    for (float x = x_min; x <= x_max; x += 0.4f) {
        for (float h = 0.3f; h <= 3.0f; h += 0.2f) {
            cloud->points.push_back(MakePoint(x, y, kGroundZ + h));
        }
    }
}

void AddBox(pcl::PointCloud<pcl::PointXYZI>* cloud, float x0, float x1,
            float y0, float y1, float z0, float z1) {
    for (float x = x0; x <= x1; x += 0.2f) {
        for (float y = y0; y <= y1; y += 0.2f) {
            for (float z = z0; z <= z1; z += 0.2f) {
                cloud->points.push_back(MakePoint(x, y, z));
            }
        }
    }
}

Params TestParams() {
    Params params;
    params.ground_min_inliers = 50;
    params.wall_min_inliers = 40;
    return params;
}

}  // namespace

TEST(TunnelFilterGate, GpsNormalDoesNotRunExperiment) {
    EXPECT_FALSE(IsGpsDenied(false, false));
    EXPECT_TRUE(IsGpsDenied(true, false));   // 무효 샘플 즉시 진입
    EXPECT_TRUE(IsGpsDenied(false, true));   // 정상 3회 전까지 유지
    EXPECT_FALSE(ShouldRunExperiment(true, false));
    EXPECT_FALSE(ShouldRunExperiment(false, true));
    EXPECT_TRUE(ShouldRunExperiment(true, true));
}

TEST(TunnelFilterCeiling, CutoffBoundaryKeepsExactThreshold) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.points.push_back(MakePoint(5.0f, 0.0f, 2.55f));
    cloud.points.push_back(MakePoint(5.0f, 0.0f, 2.5501f));
    cloud.points.push_back(MakePoint(5.0f, 0.0f, 0.0f));
    std::vector<std::uint8_t> labels(cloud.points.size(), kKept);
    EXPECT_EQ(1, MarkCeiling(cloud, 2.55f, &labels));
    EXPECT_EQ(kKept, labels[0]);
    EXPECT_EQ(kCeiling, labels[1]);
    EXPECT_EQ(kKept, labels[2]);
}

TEST(TunnelFilterLeveling, MatchesPitchOnlyCorrectionAndKeepsIntensity) {
    const float pitch = 0.03f;
    const auto rotation = LevelingRotation(0.02f, pitch, false, true);
    pcl::PointCloud<pcl::PointXYZI> input;
    input.points.push_back(MakePoint(10.0f, 2.0f, -1.0f, 42.0f));
    pcl::PointCloud<pcl::PointXYZI> output;
    ApplyRotation(input, rotation, &output);
    // 기존 CorrectingPoints: R = [cos 0 sin; 0 1 0; -sin 0 cos]
    const float c = std::cos(pitch);
    const float s = std::sin(pitch);
    EXPECT_NEAR(c * 10.0f + s * -1.0f, output.points[0].x, 1e-5f);
    EXPECT_NEAR(2.0f, output.points[0].y, 1e-5f);
    EXPECT_NEAR(-s * 10.0f + c * -1.0f, output.points[0].z, 1e-5f);
    EXPECT_FLOAT_EQ(42.0f, output.points[0].intensity);
}

TEST(TunnelFilterLeveling, RollAndPitchLevelTiltedGround) {
    // 센서가 roll/pitch만큼 기울어 보는 지면을 만들고 수평화하면 z가 일정해야 한다.
    const float roll = 0.03f;
    const float pitch = -0.02f;
    const auto level = LevelingRotation(roll, pitch, true, true);
    // 수평 지면 점을 R^T로 센서 좌표로 되돌린 뒤 다시 R을 적용한다.
    pcl::PointCloud<pcl::PointXYZI> sensor;
    for (float x = 2.0f; x < 30.0f; x += 3.0f) {
        for (float y = -6.0f; y < 6.0f; y += 3.0f) {
            const float z = kGroundZ;
            pcl::PointXYZI p;
            p.x = level[0] * x + level[3] * y + level[6] * z;
            p.y = level[1] * x + level[4] * y + level[7] * z;
            p.z = level[2] * x + level[5] * y + level[8] * z;
            p.intensity = 0.0f;
            sensor.points.push_back(p);
        }
    }
    pcl::PointCloud<pcl::PointXYZI> leveled;
    ApplyRotation(sensor, level, &leveled);
    for (const auto& p : leveled.points) EXPECT_NEAR(kGroundZ, p.z, 1e-4f);
}

TEST(TunnelFilterGround, RejectsPlaneOutsideGroundHeight) {
    Params params;
    GroundFit ceiling;
    ceiling.found = true;
    ceiling.plane.c = 1.0f;
    ceiling.plane.d = -2.4f;  // z = +2.4 (천장)
    ceiling.candidates = 1000;
    ceiling.inliers = 900;
    EXPECT_FALSE(ValidateGroundFit(&ceiling, params, nullptr));
    EXPECT_EQ("height_out_of_range", ceiling.reject_reason);

    GroundFit ground = ceiling;
    ground.plane.d = 1.7f;  // z = -1.7
    EXPECT_TRUE(ValidateGroundFit(&ground, params, nullptr));
    EXPECT_NEAR(-1.7f, ground.height_at_origin, 1e-5f);
}

TEST(TunnelFilterGround, RejectsTiltedNormalAndJumps) {
    Params params;
    GroundFit tilted;
    tilted.found = true;
    const float angle = 25.0f * 3.14159265f / 180.0f;
    tilted.plane.a = std::sin(angle);
    tilted.plane.c = std::cos(angle);
    tilted.plane.d = 1.7f * std::cos(angle);
    tilted.candidates = 1000;
    tilted.inliers = 900;
    EXPECT_FALSE(ValidateGroundFit(&tilted, params, nullptr));
    EXPECT_EQ("normal_tilt", tilted.reject_reason);

    Plane previous;
    previous.d = 1.70f;
    GroundFit jump;
    jump.found = true;
    jump.plane.d = 1.40f;  // 0.30 m 점프
    jump.candidates = 1000;
    jump.inliers = 900;
    EXPECT_FALSE(ValidateGroundFit(&jump, params, &previous));
    EXPECT_EQ("height_jump", jump.reject_reason);

    GroundFit few;
    few.found = true;
    few.plane.d = 1.70f;
    few.candidates = 1000;
    few.inliers = 100;
    EXPECT_FALSE(ValidateGroundFit(&few, params, nullptr));
    EXPECT_EQ("few_inliers", few.reject_reason);
}

TEST(TunnelFilterGround, ReusesPreviousPlaneForThreeFramesThenUnavailable) {
    GroundTemporalFilter filter(3);
    Plane plane;
    plane.d = 1.7f;
    EXPECT_EQ(PlaneSource::kCurrent, filter.Update(true, plane).source);
    for (int i = 1; i <= 3; ++i) {
        const GroundDecision decision = filter.Update(false, Plane());
        EXPECT_EQ(PlaneSource::kPrevious, decision.source) << "failure " << i;
        EXPECT_NEAR(1.7f, decision.plane.d, 1e-6f);
        EXPECT_NE(nullptr, filter.ConsistencyReference());
    }
    const GroundDecision fourth = filter.Update(false, Plane());
    EXPECT_EQ(PlaneSource::kUnavailable, fourth.source);
    EXPECT_EQ(4, fourth.consecutive_failures);
    EXPECT_EQ(nullptr, filter.ConsistencyReference());
    EXPECT_EQ(PlaneSource::kCurrent, filter.Update(true, plane).source);
}

TEST(TunnelFilterGround, FourthFailureUsesFailSafeBandNotPassThrough) {
    Params params = TestParams();
    TunnelPipeline pipeline(params);
    pcl::PointCloud<pcl::PointXYZI> good;
    AddGround(&good);
    AddBox(&good, 12.0f, 16.0f, -1.0f, 0.8f, -1.4f, 0.0f);
    EXPECT_EQ(PlaneSource::kCurrent,
              pipeline.Process(good, 0.0f, 0.0f, nullptr, nullptr, false)
                  .ground.source);

    // 지면점이 사라진 프레임(물체 + 노면 잡음 몇 점) 4개
    pcl::PointCloud<pcl::PointXYZI> bad;
    AddBox(&bad, 12.0f, 16.0f, -1.0f, 0.8f, -1.4f, 0.0f);
    for (int i = 0; i < 20; ++i) {
        bad.points.push_back(MakePoint(3.0f + i, 0.0f, kGroundZ + 0.05f));
    }
    for (int frame = 1; frame <= 3; ++frame) {
        const FrameResult& r =
            pipeline.Process(bad, 0.0f, 0.0f, nullptr, nullptr, false);
        EXPECT_EQ(PlaneSource::kPrevious, r.ground.source);
        EXPECT_FALSE(r.ground_fallback_used);
    }
    const FrameResult& fourth =
        pipeline.Process(bad, 0.0f, 0.0f, nullptr, nullptr, false);
    EXPECT_EQ(PlaneSource::kUnavailable, fourth.ground.source);
    EXPECT_TRUE(fourth.ground_fallback_used);
    // fail-safe: 마지막 정상 높이 + 띠 이하는 제거된다 (원본 전체 통과 금지).
    for (const int index : fourth.kept_indices) {
        EXPECT_GT(fourth.leveled->points[static_cast<std::size_t>(index)].z,
                  kGroundZ + params.ground_fallback_band_m);
    }
    EXPECT_GE(fourth.ground_removed, 20);
}

TEST(TunnelFilterGround, HorizontalCeilingIsNeverChosenAsGround) {
    Params params = TestParams();
    params.ceiling_max_z_lidar_m = 2.55f;
    TunnelPipeline pipeline(params);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    AddGround(&cloud);
    // 크롭 아래(z=2.3)의 넓은 수평 천장: 지면보다 점이 많아도 지면이 되면 안 된다.
    for (float x = -10.0f; x <= 40.0f; x += 0.25f) {
        for (float y = -5.0f; y <= 5.5f; y += 0.25f) {
            cloud.points.push_back(MakePoint(x, y, 2.3f));
        }
    }
    const FrameResult& r =
        pipeline.Process(cloud, 0.0f, 0.0f, nullptr, nullptr, false);
    ASSERT_EQ(PlaneSource::kCurrent, r.ground.source);
    EXPECT_NEAR(kGroundZ, r.ground.plane.HeightAt(0.0f, 0.0f), 0.05f);
}

TEST(TunnelFilterWall, SupportRequiresContinuousBins) {
    std::vector<float> s;
    for (float v = 0.0f; v < 10.0f; v += 0.5f) s.push_back(v);   // 0~10 m
    for (float v = 14.0f; v < 34.0f; v += 0.5f) s.push_back(v);  // 14~34 m
    const WallSupport support = ComputeWallSupport(s, 2.0f, 3, 1);
    ASSERT_TRUE(support.ok);
    // 10~14 m 사이 빈 bin 2개는 max_gap=1을 넘으므로 긴 뒤쪽 구간만 선택된다.
    EXPECT_NEAR(14.0f, support.s_start, 1e-4f);
    EXPECT_NEAR(34.0f, support.s_end, 1e-4f);
    EXPECT_EQ(10, support.occupied_bins);

    const WallSupport tolerant = ComputeWallSupport(s, 2.0f, 3, 2);
    ASSERT_TRUE(tolerant.ok);
    EXPECT_NEAR(0.0f, tolerant.s_start, 1e-4f);
}

TEST(TunnelFilterWall, ShortSpanIsNotAWall) {
    Params params;
    WallSegment segment;
    segment.plane.a = 0.0f;
    segment.plane.b = -1.0f;
    segment.plane.c = 0.0f;
    segment.plane.d = 6.0f;
    segment.lateral_distance = 6.0f;
    segment.inliers = 500;
    segment.inlier_ratio = 0.8f;
    segment.occupied_bins = 6;
    segment.vertical_span = 2.5f;
    segment.span = 12.0f;  // 버스 옆면 길이
    EXPECT_FALSE(ValidateWallSegment(&segment, params));
    EXPECT_EQ("short_span", segment.reject_reason);
    segment.span = 20.0f;
    segment.occupied_bins = 10;
    EXPECT_TRUE(ValidateWallSegment(&segment, params));
    segment.beyond_fraction = 0.6f;  // 뒤에 진짜 벽이 있으면 벽 앞의 물체다
    EXPECT_FALSE(ValidateWallSegment(&segment, params));
    EXPECT_EQ("not_outermost", segment.reject_reason);
}

TEST(TunnelFilterWall, LeftAndRightAreIndependentAndNeedPersistence) {
    Params params = TestParams();
    TunnelPipeline pipeline(params);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    AddGround(&cloud);
    AddWall(&cloud, 6.0f, -8.0f, 40.0f);  // 왼쪽 벽만 있다
    for (int frame = 1; frame <= 3; ++frame) {
        const FrameResult& r =
            pipeline.Process(cloud, 0.0f, 0.0f, nullptr, nullptr, true);
        EXPECT_TRUE(r.walls[0].fit_valid) << "frame " << frame;
        EXPECT_FALSE(r.walls[1].fit_valid) << "frame " << frame;
        EXPECT_FALSE(r.walls[1].removal_allowed);
        // 3프레임 지속 전에는 벽을 지우지 않는다.
        EXPECT_EQ(frame >= 3, r.walls[0].removal_allowed) << "frame " << frame;
        if (frame < 3) EXPECT_EQ(0, r.wall_removed);
    }
    const FrameResult& r = pipeline.LastResult();
    EXPECT_GT(r.wall_removed, 500);
    ASSERT_FALSE(r.walls[0].segments.empty());
    EXPECT_NEAR(6.0f, r.walls[0].segments[0].lateral_distance, 0.1f);
}

// 허용 범위 안의 수직 기울기(±tilt)가 프레임마다 부호를 바꿔도 같은 벽은 추적이
// 유지되어야 한다. 방위각이 실제로 크게 바뀌면 다시 확인 단계로 돌아간다.
TEST(TunnelFilterWall, TrackIgnoresTiltSignFlipButNotAzimuthChange) {
    Params params = TestParams();
    auto make_segment = [](float yaw_deg, float tilt_deg, float d) {
        WallSegment segment;
        const float yaw = yaw_deg * static_cast<float>(M_PI) / 180.0f;
        const float tilt = tilt_deg * static_cast<float>(M_PI) / 180.0f;
        segment.plane.a = -std::sin(yaw) * std::cos(tilt);
        segment.plane.b = -std::cos(yaw) * std::cos(tilt);
        segment.plane.c = std::sin(tilt);
        segment.plane.d = d;
        segment.valid = true;
        return std::vector<WallSegment>{segment};
    };
    WallTrack track;
    const float tilts[] = {4.9f, -1.5f, 4.9f, -1.6f};
    for (int frame = 0; frame < 4; ++frame) {
        const bool confirmed =
            track.Update(make_segment(0.5f * frame, tilts[frame], 4.6f), params);
        EXPECT_EQ(frame + 1, track.State().persist) << "frame " << frame;
        EXPECT_EQ(frame + 1 >= 3, confirmed) << "frame " << frame;
    }
    // 방위각이 10° 바뀌면 다른 벽으로 보고 persist를 다시 센다.
    EXPECT_FALSE(track.Update(make_segment(11.5f, 0.0f, 4.6f), params));
    EXPECT_EQ(1, track.State().persist);
}

// 실시간 노드는 스캔을 건너뛰어 처리 간격이 0.2 s까지 늘어난다. 요레이트 35°/s,
// 8 m/s에서 정지한 벽도 프레임마다 방위가 7° 돈다: 자차 이동으로 이전 평면을
// 옮기면 같은 벽으로 이어지고, 보정이 없으면 매번 새 벽으로 다시 센다.
TEST(TunnelFilterWall, TrackCompensatesEgoMotionBetweenSparseFrames) {
    Params params = TestParams();
    const float kDeg = static_cast<float>(M_PI) / 180.0f;
    const float yaw_steps[] = {0.0f, 7.0f, 7.0f, -7.0f, -7.0f};
    // 세계 좌표의 왼쪽 벽: y = 5 (차량 쪽 법선 (0,-1), d = 5)
    float heading = 0.0f, x = 0.0f, y = 0.0f;
    WallTrack compensated;
    WallTrack uncompensated;
    for (int k = 0; k < 5; ++k) {
        EgoMotion motion;
        if (k > 0) {
            const float dyaw = yaw_steps[k] * kDeg;
            const float distance = 8.0f * 0.2f;
            motion.valid = true;
            motion.dt_s = 0.2f;
            motion.dyaw_rad = dyaw;
            motion.dx_m = distance * std::cos(0.5f * dyaw);
            motion.dy_m = distance * std::sin(0.5f * dyaw);
            x += distance * std::cos(heading + 0.5f * dyaw);
            y += distance * std::sin(heading + 0.5f * dyaw);
            heading += dyaw;
        }
        WallSegment segment;
        segment.valid = true;
        // n_v = Rz(-heading) n_w, d_v = d_w + n_w · t
        segment.plane.a = -std::sin(heading);
        segment.plane.b = -std::cos(heading);
        segment.plane.c = 0.0f;
        segment.plane.d = 5.0f + (0.0f * x - 1.0f * y);
        const std::vector<WallSegment> segments{segment};
        const bool with = compensated.Update(segments, params, k > 0 ? &motion : nullptr);
        const bool without = uncompensated.Update(segments, params, nullptr);
        EXPECT_EQ(k + 1, compensated.State().persist) << "frame " << k;
        EXPECT_EQ(k + 1 >= 3, with) << "frame " << k;
        if (k > 0) {
            EXPECT_EQ(1, uncompensated.State().persist) << "frame " << k;
            EXPECT_FALSE(without) << "frame " << k;
        }
    }
    // 보정된 이전 평면은 현재 관측과 거의 같다.
    Plane previous;
    previous.a = 0.0f;
    previous.b = -1.0f;
    previous.c = 0.0f;
    previous.d = 5.0f;
    EgoMotion forward;
    forward.valid = true;
    forward.dt_s = 0.1f;
    forward.dy_m = 1.0f;  // 왼쪽으로 1 m 이동하면 왼쪽 벽은 1 m 가까워진다
    EXPECT_NEAR(4.0f, TransformPlaneToCurrent(previous, forward).d, 1e-5f);
}

TEST(TunnelFilterWall, ProtectedPointsSurviveWallRemoval) {
    Params params = TestParams();
    TunnelPipeline pipeline(params);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    AddGround(&cloud);
    AddWall(&cloud, 6.0f, -8.0f, 40.0f);
    AddWall(&cloud, -5.5f, -8.0f, 40.0f);
    std::vector<std::uint8_t> protect(cloud.points.size(), kProtectNone);
    int protected_wall_points = 0;
    for (std::size_t i = 0; i < cloud.points.size(); ++i) {
        const auto& p = cloud.points[i];
        if (p.y > 5.5f && p.x > 20.0f && p.x < 22.0f) {
            protect[i] = kProtectPerson;
            ++protected_wall_points;
        }
    }
    ASSERT_GT(protected_wall_points, 0);
    for (int frame = 0; frame < 3; ++frame) {
        pipeline.Process(cloud, 0.0f, 0.0f, &protect, nullptr, true);
    }
    const FrameResult& r = pipeline.LastResult();
    ASSERT_TRUE(r.walls[0].removal_allowed);
    ASSERT_TRUE(r.walls[1].removal_allowed);
    int kept_protected = 0;
    for (std::size_t i = 0; i < cloud.points.size(); ++i) {
        if (protect[i] == kProtectNone) continue;
        EXPECT_NE(kWall, r.labels[i]);
        if (r.labels[i] == kKept) ++kept_protected;
    }
    EXPECT_EQ(protected_wall_points, kept_protected);
    EXPECT_GT(r.walls[0].removed, 300);
    EXPECT_GT(r.walls[1].removed, 300);
}

TEST(TunnelFilterWall, AmbiguousPointsKeptWhenCameraUnavailable) {
    Params params = TestParams();
    TunnelPipeline pipeline(params);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    AddGround(&cloud);
    AddWall(&cloud, 6.0f, -8.0f, 40.0f);
    std::vector<std::uint8_t> ambiguous(cloud.points.size(), 0);
    for (std::size_t i = 0; i < cloud.points.size(); ++i) {
        if (cloud.points[i].x > 10.0f) ambiguous[i] = 1;  // 카메라 FOV 가정
    }
    for (int frame = 0; frame < 3; ++frame) {
        pipeline.Process(cloud, 0.0f, 0.0f, nullptr, &ambiguous, false);
    }
    const FrameResult& r = pipeline.LastResult();
    ASSERT_TRUE(r.walls[0].removal_allowed);
    for (std::size_t i = 0; i < cloud.points.size(); ++i) {
        if (ambiguous[i]) EXPECT_NE(kWall, r.labels[i]);
    }
    EXPECT_GT(r.walls[0].ambiguous_skipped, 0);
    EXPECT_GT(r.walls[0].removed, 0);  // FOV 밖 벽점은 제거
}

TEST(TunnelFilterWall, CarBesideWallIsNotAbsorbed) {
    Params params = TestParams();
    TunnelPipeline pipeline(params);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    AddGround(&cloud);
    AddWall(&cloud, 6.0f, -8.0f, 40.0f);
    // 벽에서 1 m 떨어진 차량 (y 3.2~5.0)
    AddBox(&cloud, 15.0f, 19.6f, 3.2f, 5.0f, kGroundZ + 0.3f, kGroundZ + 1.5f);
    for (int frame = 0; frame < 3; ++frame) {
        pipeline.Process(cloud, 0.0f, 0.0f, nullptr, nullptr, true);
    }
    const FrameResult& r = pipeline.LastResult();
    ASSERT_TRUE(r.walls[0].removal_allowed);
    int car_points = 0;
    int car_kept = 0;
    for (std::size_t i = 0; i < cloud.points.size(); ++i) {
        const auto& p = cloud.points[i];
        if (p.x >= 15.0f && p.x <= 19.6f && p.y >= 3.2f && p.y <= 5.0f &&
            p.z > kGroundZ + 0.25f) {
            ++car_points;
            if (r.labels[i] == kKept) ++car_kept;
        }
    }
    ASSERT_GT(car_points, 0);
    EXPECT_EQ(car_points, car_kept);
}

TEST(TunnelFilterObb, CeilingAndWallRulesRespectProtection) {
    Params params;
    Obb ceiling;
    ceiling.cx = 10.0f;
    ceiling.length = 3.0f;
    ceiling.width = 1.0f;
    ceiling.z_min = 1.0f;
    ceiling.z_max = 1.5f;
    EXPECT_TRUE(ClassifyObb(ceiling, 2.7f, 3.2f, false, false, params).removed);
    EXPECT_FALSE(ClassifyObb(ceiling, 2.4f, 3.2f, false, false, params).ceiling);
    EXPECT_FALSE(ClassifyObb(ceiling, 2.7f, 3.2f, true, false, params).removed);

    Obb wall;
    wall.cy = -5.0f;
    wall.length = 30.0f;
    wall.width = 0.5f;
    wall.z_min = -1.5f;
    wall.z_max = 1.0f;
    EXPECT_TRUE(ClassifyObb(wall, 0.2f, 2.7f, false, false, params).wall);
    EXPECT_TRUE(ClassifyObb(wall, 0.2f, 2.7f, false, false, params).removed);
    EXPECT_FALSE(ClassifyObb(wall, 0.2f, 2.7f, true, false, params).removed);
    wall.cy = -3.0f;  // 차로 안쪽의 긴 물체는 |y| 규칙 대상이 아님
    EXPECT_FALSE(ClassifyObb(wall, 0.2f, 2.7f, false, false, params).wall);
    // 확정 벽 평면에 붙은 평행 OBB면 |y|<4여도 벽 구조물
    EXPECT_TRUE(ClassifyObb(wall, 0.2f, 2.7f, false, true, params).removed);
    Obb bus = wall;  // 12 m 버스: 길이 조건 미달
    bus.length = 12.0f;
    bus.width = 2.5f;
    EXPECT_FALSE(ClassifyObb(bus, 0.3f, 3.4f, false, true, params).removed);

    Obb flat;
    flat.length = 3.0f;
    flat.width = 2.0f;
    flat.z_min = -1.60f;
    flat.z_max = -1.45f;
    const auto flags = ClassifyObb(flat, 0.1f, 0.25f, false, false, params);
    EXPECT_TRUE(flags.ground_residual);
    EXPECT_FALSE(flags.removed);  // 기본은 진단만
}

TEST(TunnelFilterWall, TwoLevelWallRemovesBarrierAndUpperWall) {
    // 이 bag의 터널: 하부 방호벽(노면 0.3~1.3 m)이 상부 벽보다 0.4 m 안쪽.
    Params params = TestParams();
    TunnelPipeline pipeline(params);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    AddGround(&cloud);
    for (float x = -8.0f; x <= 40.0f; x += 0.4f) {
        for (float h = 0.3f; h <= 1.3f; h += 0.2f) {
            cloud.points.push_back(MakePoint(x, 5.6f, kGroundZ + h));
        }
        for (float h = 1.5f; h <= 4.0f; h += 0.2f) {
            cloud.points.push_back(MakePoint(x, 6.0f, kGroundZ + h));
        }
    }
    for (int frame = 0; frame < 3; ++frame) {
        pipeline.Process(cloud, 0.0f, 0.0f, nullptr, nullptr, true);
    }
    const FrameResult& r = pipeline.LastResult();
    ASSERT_TRUE(r.walls[0].removal_allowed);
    ASSERT_GE(r.walls[0].segments.size(), 2u);
    int wall_points = 0;
    int removed_as_wall = 0;
    int kept = 0;
    for (std::size_t i = 0; i < cloud.points.size(); ++i) {
        // 지면 격자(y 5.5)는 제외하고 벽 점만 센다.
        if (cloud.points[i].y > 5.0f && cloud.points[i].z > kGroundZ + 0.25f) {
            ++wall_points;
            if (r.labels[i] == kWall) ++removed_as_wall;
            if (r.labels[i] == kKept) ++kept;
        }
    }
    EXPECT_EQ(0, kept);                     // 두 단 모두 남지 않는다
    EXPECT_EQ(wall_points, removed_as_wall);  // 지면으로 오인되지도 않는다
    // 지면은 방호벽 하단에 끌려 올라가지 않는다.
    EXPECT_NEAR(kGroundZ, r.ground.plane.HeightAt(0.0f, 0.0f), 0.03f);
}

// 상부 벽과 방호벽이 서로 반대로 기울면(각각 tilt 한도 안) 3D 법선 각도는 단차
// 허용각(5°)을 넘지만 방위는 같다. 방호벽을 단차로 인정해 지워야 한다.
TEST(TunnelFilterWall, OppositelyTiltedBarrierIsStillAStep) {
    Params params = TestParams();
    TunnelPipeline pipeline(params);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    AddGround(&cloud);
    const float upper_tilt = std::tan(2.0f * static_cast<float>(M_PI) / 180.0f);
    const float barrier_tilt = std::tan(3.5f * static_cast<float>(M_PI) / 180.0f);
    for (float x = -8.0f; x <= 40.0f; x += 0.4f) {
        for (float h = 0.3f; h <= 1.3f; h += 0.1f) {
            cloud.points.push_back(
                MakePoint(x, 5.6f + barrier_tilt * (h - 0.8f), kGroundZ + h));
        }
        for (float h = 1.5f; h <= 4.0f; h += 0.2f) {
            cloud.points.push_back(
                MakePoint(x, 6.0f - upper_tilt * (h - 1.3f), kGroundZ + h));
        }
    }
    for (int frame = 0; frame < 3; ++frame) {
        pipeline.Process(cloud, 0.0f, 0.0f, nullptr, nullptr, true);
    }
    const FrameResult& r = pipeline.LastResult();
    ASSERT_TRUE(r.walls[0].removal_allowed);
    ASSERT_GE(r.walls[0].segments.size(), 2u);
    // 두 평면의 3D 각도는 5°를 넘는다: 방위 비교가 아니면 이 단차는 거부된다.
    EXPECT_GT(AngleBetweenNormalsDeg(r.walls[0].segments[0].plane,
                                     r.walls[0].segments[1].plane),
              params.wall_step_max_angle_deg);
    EXPECT_TRUE(r.walls[0].segments[1].step);
    int barrier_points = 0;
    int barrier_kept = 0;
    for (std::size_t i = 0; i < cloud.points.size(); ++i) {
        const auto& p = cloud.points[i];
        if (p.y > 5.0f && p.z > kGroundZ + 0.25f && p.z < kGroundZ + 1.35f) {
            ++barrier_points;
            if (r.labels[i] == kKept) ++barrier_kept;
        }
    }
    ASSERT_GT(barrier_points, 0);
    EXPECT_EQ(0, barrier_kept);
}

TEST(TunnelFilterWall, CoplanarRunAcrossOcclusionGapIsRemoved) {
    Params params = TestParams();
    TunnelPipeline pipeline(params);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    AddGround(&cloud);
    AddWall(&cloud, 6.0f, -8.0f, 14.0f);   // 22 m
    AddWall(&cloud, 6.0f, 20.0f, 40.0f);   // 가림으로 6 m 끊긴 뒤 20 m
    for (int frame = 0; frame < 3; ++frame) {
        pipeline.Process(cloud, 0.0f, 0.0f, nullptr, nullptr, true);
    }
    const FrameResult& r = pipeline.LastResult();
    ASSERT_TRUE(r.walls[0].removal_allowed);
    ASSERT_FALSE(r.walls[0].segments.empty());
    EXPECT_EQ(2u, r.walls[0].segments.front().intervals.size());
    for (std::size_t i = 0; i < cloud.points.size(); ++i) {
        if (cloud.points[i].y > 5.8f && cloud.points[i].z > kGroundZ + 0.25f) {
            EXPECT_EQ(kWall, r.labels[i]) << "x=" << cloud.points[i].x;
        }
    }
}

TEST(TunnelFilterWall, YawedVehicleStillRemovesWallCrossingAhead) {
    // 차량이 터널에 대해 17도 틀어져 왼쪽 벽이 전방에서 y<2.75로 넘어온다.
    Params params = TestParams();
    TunnelPipeline pipeline(params);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    AddGround(&cloud);
    const float slope = std::tan(-17.0f * 3.14159265f / 180.0f);
    for (float x = -25.0f; x <= 45.0f; x += 0.4f) {
        for (float h = 0.3f; h <= 3.0f; h += 0.2f) {
            cloud.points.push_back(MakePoint(x, 5.0f + slope * x, kGroundZ + h));
            cloud.points.push_back(MakePoint(x, -7.0f + slope * x, kGroundZ + h));
        }
    }
    for (int frame = 0; frame < 3; ++frame) {
        pipeline.Process(cloud, 0.0f, 0.0f, nullptr, nullptr, true);
    }
    const FrameResult& r = pipeline.LastResult();
    ASSERT_TRUE(r.walls[0].removal_allowed);
    ASSERT_TRUE(r.walls[1].removal_allowed);
    // 좌/우 주 평면은 서로 다른 벽이어야 한다 (반대쪽 벽 평면을 가져오지 않음).
    ASSERT_FALSE(r.walls[0].segments.empty());
    ASSERT_FALSE(r.walls[1].segments.empty());
    EXPECT_GT(std::fabs(r.walls[0].segments.front().plane.d -
                        r.walls[1].segments.front().plane.d), 1.0f);
    int crossing = 0;
    for (std::size_t i = 0; i < cloud.points.size(); ++i) {
        const auto& p = cloud.points[i];
        if (p.z < kGroundZ + 0.25f) continue;
        EXPECT_NE(kKept, r.labels[i]) << "x=" << p.x << " y=" << p.y;
        if (std::fabs(p.y) < 2.75f) ++crossing;
    }
    EXPECT_GT(crossing, 100);  // 후보 영역 밖으로 넘어온 벽 점이 실제로 있었다
}

TEST(TunnelFilterWall, BendingWallCrossingAheadIsRemovedByContinuation) {
    // 왼쪽 벽이 x=15 m부터 8도 꺾여 전방에서 차로 쪽(|y|<2.75)으로 들어온다.
    Params params = TestParams();
    TunnelPipeline pipeline(params);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    AddGround(&cloud);
    const float bend = std::tan(8.0f * 3.14159265f / 180.0f);
    for (float x = -25.0f; x <= 45.0f; x += 0.4f) {
        const float y = x <= 15.0f ? 6.0f : 6.0f - bend * (x - 15.0f);
        for (float h = 0.3f; h <= 3.0f; h += 0.2f) {
            cloud.points.push_back(MakePoint(x, y, kGroundZ + h));
        }
    }
    for (int frame = 0; frame < 3; ++frame) {
        pipeline.Process(cloud, 0.0f, 0.0f, nullptr, nullptr, true);
    }
    const FrameResult& r = pipeline.LastResult();
    ASSERT_TRUE(r.walls[0].removal_allowed);
    int far_points = 0;
    int far_kept = 0;
    for (std::size_t i = 0; i < cloud.points.size(); ++i) {
        const auto& p = cloud.points[i];
        if (p.z < kGroundZ + 0.25f || p.x < 20.0f) continue;
        ++far_points;
        if (r.labels[i] == kKept) ++far_kept;
    }
    ASSERT_GT(far_points, 0);
    EXPECT_LE(far_kept, far_points / 20);  // 꺾인 전방 벽도 95% 이상 제거
    EXPECT_GE(r.walls[0].segments.size(), 2u);  // 직선부 + 꺾인 구간 평면
}

TEST(TunnelFilterWall, PlaneInFrontOfWallIsRejectedAndPersonSurvives) {
    // 2단 벽 + 벽에서 1 m 앞쪽(차로 쪽) 기둥 열 + 그 선 위의 비보호 사람.
    // 기둥 열은 길고 높은 평면이지만 벽 두께 층(0.6 m) 밖이므로 벽이 아니다.
    Params params = TestParams();
    TunnelPipeline pipeline(params);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    AddGround(&cloud);
    for (float x = -25.0f; x <= 45.0f; x += 0.4f) {
        for (float h = 0.3f; h <= 1.3f; h += 0.2f) {
            cloud.points.push_back(MakePoint(x, 5.6f, kGroundZ + h));
        }
        for (float h = 1.5f; h <= 4.0f; h += 0.2f) {
            cloud.points.push_back(MakePoint(x, 6.0f, kGroundZ + h));
        }
    }
    for (float x = -24.0f; x <= 44.0f; x += 2.0f) {  // 기둥 (0.2 m 폭)
        for (float dx = 0.0f; dx <= 0.2f; dx += 0.1f) {
            for (float h = 0.3f; h <= 3.0f; h += 0.2f) {
                cloud.points.push_back(MakePoint(x + dx, 5.0f, kGroundZ + h));
            }
        }
    }
    const std::size_t person_begin = cloud.points.size();
    AddBox(&cloud, 9.0f, 9.4f, 4.8f, 5.2f, kGroundZ + 0.3f, kGroundZ + 1.8f);
    const std::size_t person_end = cloud.points.size();
    for (int frame = 0; frame < 3; ++frame) {
        pipeline.Process(cloud, 0.0f, 0.0f, nullptr, nullptr, true);
    }
    const FrameResult& r = pipeline.LastResult();
    ASSERT_TRUE(r.walls[0].removal_allowed);
    for (const auto& segment : r.walls[0].segments) {
        EXPECT_GT(segment.plane.d, 5.3f) << "plane in front of the wall accepted";
    }
    for (std::size_t i = person_begin; i < person_end; ++i) {
        EXPECT_EQ(kKept, r.labels[i]);
    }
}

TEST(TunnelFilterGround, RaisedSideSlabDoesNotLiftGround) {
    Params params = TestParams();
    TunnelPipeline pipeline(params);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    AddGround(&cloud);
    // 벽 옆 0.2 m 높이의 촘촘한 턱(보도/방호벽 기초)
    for (float x = -10.0f; x <= 40.0f; x += 0.2f) {
        for (float y = 5.0f; y <= 6.0f; y += 0.2f) {
            cloud.points.push_back(MakePoint(x, y, kGroundZ + 0.2f));
        }
    }
    const FrameResult& r =
        pipeline.Process(cloud, 0.0f, 0.0f, nullptr, nullptr, false);
    ASSERT_EQ(PlaneSource::kCurrent, r.ground.source);
    EXPECT_NEAR(kGroundZ, r.ground.plane.HeightAt(0.0f, 0.0f), 0.03f);
    EXPECT_EQ(1, r.ground_fit.prior_source);  // 첫 프레임은 z 최빈값 기준
    const FrameResult& next =
        pipeline.Process(cloud, 0.0f, 0.0f, nullptr, nullptr, false);
    EXPECT_EQ(0, next.ground_fit.prior_source);  // 이후는 직전 정상 평면 기준
    EXPECT_NEAR(kGroundZ, next.ground.plane.HeightAt(0.0f, 0.0f), 0.03f);
    // 2단계 재적합이 켜져 있어도 턱 쪽으로 들리지 않는다.
    EXPECT_FALSE(next.ground_fit.refit_used &&
                 next.ground_fit.refit_height_delta_m > params.ground_refit_max_height_delta_m);
}

// 직전 평면(평평)이 오래되어 이번 프레임 노면(1.2° 오르막)과 어긋나면 1단계 후보
// 띠(±0.15 m)가 원거리 노면을 잘라낸다. 근거리 뒤쪽 턱(+0.12 m)이 1단계 최소제곱을
// 평평하게 끌면 전방 원거리 노면이 제거 거리 밖에 남는다. 2단계 재적합은 이번 프레임
// 평면 주변에서 다시 맞춰 원거리 노면까지 지운다.
TEST(TunnelFilterGround, RefitRemovesFarGroundWhenPriorIsStale) {
    const float slope = std::tan(1.2f * static_cast<float>(M_PI) / 180.0f);
    auto make_scene = [&](bool tilted) {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        for (float x = -10.0f; x <= 40.0f; x += 0.5f) {
            for (float y = -5.0f; y <= 5.0f; y += 0.5f) {
                cloud.points.push_back(
                    MakePoint(x, y, kGroundZ + (tilted ? slope * x : 0.0f)));
            }
        }
        if (tilted) {
            for (float x = -7.0f; x <= -2.0f; x += 0.1f) {
                for (float y = -4.0f; y <= 4.0f; y += 0.25f) {
                    cloud.points.push_back(MakePoint(x, y, kGroundZ + slope * x + 0.12f));
                }
            }
        }
        return cloud;
    };
    const auto flat = make_scene(false);
    const auto tilted = make_scene(true);
    auto far_ground_kept = [&](bool refit) {
        Params params = TestParams();
        params.ground_refit_enable = refit;
        TunnelPipeline pipeline(params);
        pipeline.Process(flat, 0.0f, 0.0f, nullptr, nullptr, false);
        const FrameResult& r = pipeline.Process(tilted, 0.0f, 0.0f, nullptr, nullptr, false);
        EXPECT_EQ(PlaneSource::kCurrent, r.ground.source);
        EXPECT_EQ(0, r.ground_fit.prior_source);
        int kept = 0;
        for (std::size_t i = 0; i < tilted.points.size(); ++i) {
            if (tilted.points[i].x >= 20.0f && r.labels[i] == kKept) ++kept;
        }
        return std::make_pair(kept, r.ground_fit.refit_used);
    };
    const auto without = far_ground_kept(false);
    const auto with = far_ground_kept(true);
    EXPECT_GT(without.first, 50);  // 1단계만으로는 전방 원거리 노면이 남는다
    EXPECT_TRUE(with.second);
    EXPECT_EQ(0, with.first);
}

TEST(TunnelFilterPipeline, EndToEndSyntheticTunnelKeepsCar) {
    Params params = TestParams();
    TunnelPipeline pipeline(params);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    AddGround(&cloud);
    AddWall(&cloud, 6.0f, -8.0f, 40.0f);
    AddWall(&cloud, -5.5f, -8.0f, 40.0f);
    for (float x = -8.0f; x <= 40.0f; x += 0.5f) {       // 천장 (크롭 대상)
        for (float y = -5.0f; y <= 5.5f; y += 0.5f) {
            cloud.points.push_back(MakePoint(x, y, 2.8f));
        }
    }
    AddBox(&cloud, 12.0f, 16.6f, -1.0f, 0.8f, kGroundZ + 0.3f, kGroundZ + 1.5f);
    for (int frame = 0; frame < 3; ++frame) {
        pipeline.Process(cloud, 0.0f, 0.0f, nullptr, nullptr, true);
    }
    const FrameResult& r = pipeline.LastResult();
    EXPECT_GT(r.ceiling_removed, 0);
    for (const int index : r.kept_indices) {
        EXPECT_LE(r.leveled->points[static_cast<std::size_t>(index)].z,
                  params.ceiling_max_z_lidar_m);
    }
    int car_objects = 0;
    for (const auto& object : r.objects) {
        EXPECT_LT(object.obb.length, 15.0f);  // 거대 OBB 없음
        if (!object.flags.removed && object.obb.cx > 11.0f && object.obb.cx < 17.0f &&
            std::fabs(object.obb.cy) < 1.5f) {
            ++car_objects;
        }
    }
    EXPECT_EQ(1, car_objects);
}

TEST(TunnelFilterPreGpsGuard, RemovesOnlyTunnelSpanningObb) {
    Params params = TestParams();
    EXPECT_TRUE(IsPreGpsStructuralObb(44.15f, 12.36f, 4.36f, params));

    // 긴 버스/트럭이어도 폭과 높이가 구조물 기준보다 작으면 보존한다.
    EXPECT_FALSE(IsPreGpsStructuralObb(18.0f, 2.8f, 3.8f, params));
    // 넓은 차량 군집도 터널처럼 길지 않으면 보존한다.
    EXPECT_FALSE(IsPreGpsStructuralObb(10.0f, 7.0f, 3.5f, params));
    // 필터를 끄면 같은 구조물 OBB도 보존한다.
    params.pre_gps_structural_obb_filter_enable = false;
    EXPECT_FALSE(IsPreGpsStructuralObb(44.15f, 12.36f, 4.36f, params));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
