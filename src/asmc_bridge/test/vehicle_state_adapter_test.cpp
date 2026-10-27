#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "asmc_bridge/vehicle_state_adapter.hpp"

namespace {
constexpr double kLatitudeMetersPerDegree = 111000.0;
constexpr double kLongitudeMetersPerDegree = 88900.0;

void Expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(1);
    }
}

ros::SteadyTime Steady(double seconds) {
    ros::SteadyTime time;
    time.fromSec(seconds);
    return time;
}

double HeaderStamp(double steady_time) {
    const int tick = static_cast<int>(std::llround(steady_time * 100.0));
    return 1000.0 + steady_time + (tick % 2 == 0 ? 8.0 : -9.0);
}

morai_msgs::GPSMessage MakeGps(double steady_time, double east_m,
                               double north_m = 0.0) {
    morai_msgs::GPSMessage gps;
    gps.header.stamp = ros::Time(HeaderStamp(steady_time));
    gps.latitude = 37.0 + north_m / kLatitudeMetersPerDegree;
    gps.longitude = 127.0 + east_m / kLongitudeMetersPerDegree;
    gps.status = 1;
    return gps;
}

sensor_msgs::Imu MakeImu(double steady_time, double acceleration) {
    sensor_msgs::Imu imu;
    imu.header.stamp = ros::Time(HeaderStamp(steady_time));
    imu.orientation.w = 1.0;
    imu.linear_acceleration.x = acceleration;
    return imu;
}

frenet_planner_ros::StateAdapterConfig TestConfig() {
    frenet_planner_ros::StateAdapterConfig config;
    config.gps_timeout_s = 0.5;
    config.imu_timeout_s = 0.3;
    return config;
}

void Imu(frenet_planner_ros::GpsImuStateEstimator& estimator,
         double time, double acceleration) {
    estimator.updateImu(MakeImu(time, acceleration), Steady(time));
}

void Gps(frenet_planner_ros::GpsImuStateEstimator& estimator,
         double time, double east_m, double north_m = 0.0) {
    estimator.updateGps(MakeGps(time, east_m, north_m), Steady(time));
}

double Speed(const frenet_planner_ros::GpsImuStateEstimator& estimator,
             double time) {
    const auto state = estimator.makeState(Steady(time));
    Expect(state.valid, "expected valid GPS/IMU state");
    Expect(std::isfinite(state.speed) && state.speed >= 0.0,
           "speed must be finite and non-negative");
    return state.speed;
}

void InitializeAtSpeed(frenet_planner_ros::GpsImuStateEstimator& estimator,
                       double start_time, double speed, double& x) {
    for (int i = 0; i <= 20; ++i) {
        const double time = start_time + 0.05 * i;
        x = speed * 0.05 * i;
        Imu(estimator, time, 0.0);
        Gps(estimator, time, x);
    }
}

void CheckInvalidGpsRecovery() {
    // Each invalid form must retire both the position window and the old speed.
    // Recovery at a distant location must not create a velocity across the gap.
    for (int kind = 0; kind < 6; ++kind) {
        frenet_planner_ros::GpsImuStateEstimator estimator(TestConfig());
        double x = 0.0;
        InitializeAtSpeed(estimator, 100.0, 8.0, x);
        Expect(Speed(estimator, 101.0) > 7.0, "moving baseline before invalid GPS");
        auto invalid = MakeGps(101.05, x);
        switch (kind) {
            case 0: invalid.latitude = invalid.longitude = 0.0; break;
            case 1: invalid.status = 0; break;
            case 2: invalid.latitude = std::numeric_limits<double>::quiet_NaN(); break;
            case 3: invalid.header.stamp = ros::Time(); break;
            case 4: invalid.longitude = 181.0; break;
            case 5: invalid.eastOffset = std::numeric_limits<double>::infinity(); break;
        }
        Imu(estimator, 101.05, 5.0);
        estimator.updateGps(invalid, Steady(101.05));
        const auto unavailable = estimator.makeState(Steady(101.05));
        Expect(!unavailable.valid && unavailable.x == 0.0 && unavailable.y == 0.0,
               "invalid GPS must not project or publish a previous valid position");
        Imu(estimator, 101.10, 0.0);
        Gps(estimator, 101.10, x + 1000.0);
        Expect(!estimator.makeState(Steady(101.10)).valid,
               "first recovered fix must not revive pre-dropout speed");
        Imu(estimator, 101.30, 0.0);
        Gps(estimator, 101.30, x + 1000.0);
        Expect(Speed(estimator, 101.30) < 1e-9,
               "recovered stationary baseline must not inherit speed or acceleration");
    }
}

void CheckMissingGpsRecovery() {
    frenet_planner_ros::GpsImuStateEstimator estimator(TestConfig());
    double x = 0.0;
    InitializeAtSpeed(estimator, 110.0, 8.0, x);
    Imu(estimator, 111.6, 0.0);
    Expect(!estimator.makeState(Steady(111.6)).valid, "GPS timeout must be invalid");
    Gps(estimator, 111.6, x + 2.0);
    Expect(!estimator.makeState(Steady(111.6)).valid,
           "first fix after silence requires a new baseline even without an invalid packet");
    Imu(estimator, 111.8, 0.0);
    Gps(estimator, 111.8, x + 2.0);
    Expect(Speed(estimator, 111.8) < 1e-9, "no velocity from pre-timeout history");
}

void CheckReceiptRegressionAndRepeatedZero() {
    frenet_planner_ros::GpsImuStateEstimator estimator(TestConfig());
    double x = 0.0;
    InitializeAtSpeed(estimator, 120.0, 8.0, x);
    Gps(estimator, 121.0, x);  // Duplicate receipt cannot seed a velocity interval.
    Expect(!estimator.makeState(Steady(121.0)).valid, "duplicate receipt clears baseline");
    Gps(estimator, 120.9, x);
    Expect(!estimator.makeState(Steady(120.9)).valid, "regressed receipt clears baseline");
    Imu(estimator, 121.1, 0.0);
    Gps(estimator, 121.1, x);
    Expect(Speed(estimator, 121.1) < 1e-9, "fresh baseline after receipt regression");
    for (int i = 0; i < 5; ++i) {
        const double time = 122.0 + .2 * i;
        auto gps = MakeGps(time, x);
        gps.latitude = gps.longitude = 0.0;  // status remains 1, as in the live tunnel.
        Imu(estimator, time, 0.0);
        estimator.updateGps(gps, Steady(time));
        Expect(!estimator.makeState(Steady(time)).valid, "repeated GPS zero remains invalid");
    }
}
}  // namespace

int main() {
    // 1. Constant 10 m/s with quantized longitudinal and lateral positions.
    frenet_planner_ros::GpsImuStateEstimator constant(TestConfig());
    for (int i = 0; i <= 40; ++i) {
        const double time = 10.0 + 0.05 * i;
        const double east =
            std::round((10.0 * 0.05 * i) / 0.1) * 0.1;
        const double lateral = i % 2 == 0 ? 0.12 : -0.12;
        Imu(constant, time, 0.0);
        Gps(constant, time, east, lateral);
    }
    const double constant_speed = Speed(constant, 12.0);
    Expect(std::abs(constant_speed - 10.0) < 0.8,
           "constant 10 m/s must converge despite GPS quantization");

    // 2. Strong braking: IMU prediction must react before the GPS window does.
    frenet_planner_ros::GpsImuStateEstimator braking(TestConfig());
    double x = 0.0;
    InitializeAtSpeed(braking, 20.0, 10.0, x);
    const double before_braking = Speed(braking, 21.0);
    double truth_speed = 10.0;
    const double brake_start = 21.0;
    double first_prediction = before_braking;
    for (int i = 1; i <= 20; ++i) {
        const double time = brake_start + 0.01 * i;
        truth_speed = std::max(0.0, truth_speed - 8.0 * 0.01);
        x += truth_speed * 0.01;
        Imu(braking, time, -8.0);
        if (i == 1) first_prediction = Speed(braking, time);
        if (i % 5 == 0) Gps(braking, time, x);
    }
    const double speed_after_200ms = Speed(braking, brake_start + 0.20);
    Expect(first_prediction < before_braking,
           "IMU prediction must decrease speed on the first braking callback");
    Expect(speed_after_200ms < before_braking - 1.0 &&
               std::abs(speed_after_200ms - truth_speed) < 1.5,
           "GPS window must not keep braking speed high for 0.2 s");

    // 3. Accelerate from rest to 10 m/s using IMU prediction plus GPS updates.
    frenet_planner_ros::GpsImuStateEstimator acceleration(TestConfig());
    x = 0.0;
    for (int i = 0; i <= 10; ++i) {
        const double time = 30.0 + 0.05 * i;
        Imu(acceleration, time, 0.0);
        Gps(acceleration, time, x);
    }
    truth_speed = 0.0;
    for (int i = 1; i <= 200; ++i) {
        const double time = 30.5 + 0.01 * i;
        truth_speed = std::min(10.0, truth_speed + 5.0 * 0.01);
        x += truth_speed * 0.01;
        Imu(acceleration, time, 5.0);
        if (i % 5 == 0) Gps(acceleration, time, x);
    }
    Expect(std::abs(Speed(acceleration, 32.5) - 10.0) < 1.0,
           "0 to 10 m/s acceleration must be tracked");

    // 4. At 1 m/s, 10 cm GPS quantization repeats fixes but must not trigger ZUPT.
    frenet_planner_ros::GpsImuStateEstimator low_speed(TestConfig());
    for (int i = 0; i <= 40; ++i) {
        const double time = 40.0 + 0.05 * i;
        const double east = std::floor((1.0 * 0.05 * i) / 0.1) * 0.1;
        Imu(low_speed, time, 0.0);
        Gps(low_speed, time, east);
    }
    Expect(Speed(low_speed, 42.0) > 0.65,
           "repeated low-speed GPS fixes must not cause false stationary");

    // 5. A true stationary period with low acceleration converges to zero.
    frenet_planner_ros::GpsImuStateEstimator stopped(TestConfig());
    x = 0.0;
    InitializeAtSpeed(stopped, 50.0, 1.0, x);
    double time = 51.0;
    for (int i = 1; i <= 40; ++i) {
        time += 0.05;
        Imu(stopped, time, i <= 5 ? -2.0 : 0.0);
        Gps(stopped, time, x);
    }
    Expect(Speed(stopped, time) < 0.05,
           "confirmed stationary state must converge to zero");

    // 6. One GPS outlier must not cause a large velocity jump.
    frenet_planner_ros::GpsImuStateEstimator outlier(TestConfig());
    x = 0.0;
    InitializeAtSpeed(outlier, 60.0, 10.0, x);
    const double before_outlier = Speed(outlier, 61.0);
    Imu(outlier, 61.05, 0.0);
    Gps(outlier, 61.05, x + 100.0);
    Expect(std::abs(Speed(outlier, 61.05) - before_outlier) < 0.2,
           "single GPS outlier must not cause a large speed jump");

    // 7. Non-positive and oversized steady IMU dt values are not integrated.
    Imu(outlier, 61.04, -12.0);
    Imu(outlier, 62.0, 8.0);
    Gps(outlier, 62.0, x + 10.0);
    Expect(!outlier.makeState(Steady(62.0)).valid,
           "oversized GPS gap requires reacquisition");
    Imu(outlier, 62.2, 0.0);
    Gps(outlier, 62.2, x + 10.0);
    const double abnormal_dt_speed = Speed(outlier, 62.2);
    Expect(std::isfinite(abnormal_dt_speed) && abnormal_dt_speed >= 0.0 &&
               abnormal_dt_speed <= TestConfig().maximum_speed_mps,
           "abnormal steady dt must not produce NaN or runaway speed");

    // 8. Freshness behavior remains unchanged.
    Expect(!constant.makeState(Steady(12.6)).valid,
           "stale GPS/IMU must keep state invalid");

    CheckInvalidGpsRecovery();
    CheckMissingGpsRecovery();
    CheckReceiptRegressionAndRepeatedZero();

    std::cout << "vehicle_state_adapter_test: PASS"
              << " constant=" << constant_speed
              << " braking_200ms=" << speed_after_200ms << '\n';
    return 0;
}
