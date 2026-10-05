#include <PlannerInterface/DrivableFootprint.hpp>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

double DrivableFootprintRatio(
    const cv::Mat& drivable_mask,
    double pixels_per_meter,
    double world_offset_x,
    double world_offset_y,
    const std::array<cv::Point2d, 4>& corners_map,
    int supersample) {
    if (drivable_mask.empty() || drivable_mask.type() != CV_8UC1 ||
        !(pixels_per_meter > 0.0)) {
        return 0.0;
    }
    supersample = std::max(1, supersample);

    std::array<cv::Point2d, 4> corners_px;
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < corners_map.size(); ++i) {
        const double x = pixels_per_meter *
            (corners_map[i].x - world_offset_x);
        const double y = pixels_per_meter *
            (corners_map[i].y - world_offset_y);
        if (!std::isfinite(x) || !std::isfinite(y)) return 0.0;
        corners_px[i] = {x, y};
        min_x = std::min(min_x, x);
        min_y = std::min(min_y, y);
        max_x = std::max(max_x, x);
        max_y = std::max(max_y, y);
    }

    // 각 래스터 값은 정수 좌표를 중심으로 하는 한 픽셀 셀로 해석한다.
    // OBB와 맵 셀이 전혀 겹치지 않으면 큰 임시 영상을 만들지 않고 바로 끝낸다.
    if (max_x < -0.5 || max_y < -0.5 ||
        min_x > static_cast<double>(drivable_mask.cols) - 0.5 ||
        min_y > static_cast<double>(drivable_mask.rows) - 0.5) {
        return 0.0;
    }

    const int roi_x0 = static_cast<int>(std::floor(min_x - 0.5));
    const int roi_y0 = static_cast<int>(std::floor(min_y - 0.5));
    const int roi_x1 = static_cast<int>(std::ceil(max_x + 0.5));
    const int roi_y1 = static_cast<int>(std::ceil(max_y + 0.5));
    const int roi_width = std::max(1, roi_x1 - roi_x0);
    const int roi_height = std::max(1, roi_y1 - roi_y0);

    // 비정상적으로 거대한 OBB 하나가 메모리를 고갈시키지 않게 방어한다.
    constexpr std::int64_t kMaxSupersampledPixels = 16'000'000;
    const std::int64_t high_width =
        static_cast<std::int64_t>(roi_width) * supersample;
    const std::int64_t high_height =
        static_cast<std::int64_t>(roi_height) * supersample;
    if (high_width <= 0 || high_height <= 0 ||
        high_width * high_height > kMaxSupersampledPixels) {
        return 0.0;
    }

    cv::Mat road_native = cv::Mat::zeros(roi_height, roi_width, CV_8UC1);
    const cv::Rect object_roi(roi_x0, roi_y0, roi_width, roi_height);
    const cv::Rect map_roi(0, 0, drivable_mask.cols, drivable_mask.rows);
    const cv::Rect overlap = object_roi & map_roi;
    if (overlap.area() > 0) {
        const cv::Rect destination(
            overlap.x - roi_x0, overlap.y - roi_y0,
            overlap.width, overlap.height);
        drivable_mask(overlap).copyTo(road_native(destination));
    }

    cv::Mat road_high;
    cv::resize(
        road_native, road_high,
        cv::Size(static_cast<int>(high_width), static_cast<int>(high_height)),
        0.0, 0.0, cv::INTER_NEAREST);

    std::vector<cv::Point> polygon;
    polygon.reserve(corners_px.size());
    for (const auto& point : corners_px) {
        // roi_x0 픽셀 셀의 왼쪽 경계는 roi_x0 - 0.5 이다.
        polygon.emplace_back(
            static_cast<int>(std::lround(
                (point.x - (static_cast<double>(roi_x0) - 0.5)) *
                supersample)),
            static_cast<int>(std::lround(
                (point.y - (static_cast<double>(roi_y0) - 0.5)) *
                supersample)));
    }

    cv::Mat object_high = cv::Mat::zeros(road_high.size(), CV_8UC1);
    cv::fillConvexPoly(object_high, polygon, cv::Scalar(255), cv::LINE_8);
    const int object_pixels = cv::countNonZero(object_high);
    if (object_pixels == 0) return 0.0;

    cv::Mat intersection;
    cv::bitwise_and(object_high, road_high, intersection);
    return static_cast<double>(cv::countNonZero(intersection)) /
        static_cast<double>(object_pixels);
}

bool DrivableFootprintIntersectionRect(
    const cv::Mat& drivable_mask,
    double pixels_per_meter,
    double world_offset_x,
    double world_offset_y,
    const std::array<cv::Point2d, 4>& corners_map,
    std::array<cv::Point2d, 4>* intersection_corners_map,
    int supersample) {
    if (intersection_corners_map == nullptr || drivable_mask.empty() ||
        drivable_mask.type() != CV_8UC1 || !(pixels_per_meter > 0.0)) {
        return false;
    }
    supersample = std::max(1, supersample);

    std::array<cv::Point2d, 4> corners_px;
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < corners_map.size(); ++i) {
        const double x = pixels_per_meter *
            (corners_map[i].x - world_offset_x);
        const double y = pixels_per_meter *
            (corners_map[i].y - world_offset_y);
        if (!std::isfinite(x) || !std::isfinite(y)) return false;
        corners_px[i] = {x, y};
        min_x = std::min(min_x, x);
        min_y = std::min(min_y, y);
        max_x = std::max(max_x, x);
        max_y = std::max(max_y, y);
    }
    if (max_x < -0.5 || max_y < -0.5 ||
        min_x > static_cast<double>(drivable_mask.cols) - 0.5 ||
        min_y > static_cast<double>(drivable_mask.rows) - 0.5) {
        return false;
    }

    const int roi_x0 = static_cast<int>(std::floor(min_x - 0.5));
    const int roi_y0 = static_cast<int>(std::floor(min_y - 0.5));
    const int roi_x1 = static_cast<int>(std::ceil(max_x + 0.5));
    const int roi_y1 = static_cast<int>(std::ceil(max_y + 0.5));
    const int roi_width = std::max(1, roi_x1 - roi_x0);
    const int roi_height = std::max(1, roi_y1 - roi_y0);
    constexpr std::int64_t kMaxSupersampledPixels = 16'000'000;
    const std::int64_t high_width =
        static_cast<std::int64_t>(roi_width) * supersample;
    const std::int64_t high_height =
        static_cast<std::int64_t>(roi_height) * supersample;
    if (high_width <= 0 || high_height <= 0 ||
        high_width * high_height > kMaxSupersampledPixels) {
        return false;
    }

    cv::Mat road_native = cv::Mat::zeros(roi_height, roi_width, CV_8UC1);
    const cv::Rect object_roi(roi_x0, roi_y0, roi_width, roi_height);
    const cv::Rect map_roi(0, 0, drivable_mask.cols, drivable_mask.rows);
    const cv::Rect overlap = object_roi & map_roi;
    if (overlap.area() > 0) {
        const cv::Rect destination(
            overlap.x - roi_x0, overlap.y - roi_y0,
            overlap.width, overlap.height);
        drivable_mask(overlap).copyTo(road_native(destination));
    }

    cv::Mat road_high;
    cv::resize(
        road_native, road_high,
        cv::Size(static_cast<int>(high_width), static_cast<int>(high_height)),
        0.0, 0.0, cv::INTER_NEAREST);
    std::vector<cv::Point> polygon;
    polygon.reserve(corners_px.size());
    for (const auto& point : corners_px) {
        polygon.emplace_back(
            static_cast<int>(std::lround(
                (point.x - (static_cast<double>(roi_x0) - 0.5)) *
                supersample)),
            static_cast<int>(std::lround(
                (point.y - (static_cast<double>(roi_y0) - 0.5)) *
                supersample)));
    }
    cv::Mat object_high = cv::Mat::zeros(road_high.size(), CV_8UC1);
    cv::fillConvexPoly(object_high, polygon, cv::Scalar(255), cv::LINE_8);
    cv::Mat intersection;
    cv::bitwise_and(object_high, road_high, intersection);
    std::vector<cv::Point> intersection_points;
    cv::findNonZero(intersection, intersection_points);
    if (intersection_points.empty()) return false;

    cv::RotatedRect rectangle = cv::minAreaRect(intersection_points);
    // findNonZero represents sub-pixel centers. Add one sub-pixel to each
    // dimension so the returned rectangle conservatively covers the occupied
    // cells instead of shrinking by one sample at both boundaries.
    rectangle.size.width = std::max(
        1.0F, rectangle.size.width + 1.0F);
    rectangle.size.height = std::max(
        1.0F, rectangle.size.height + 1.0F);
    std::array<cv::Point2f, 4> high_corners;
    rectangle.points(high_corners.data());
    for (std::size_t i = 0; i < high_corners.size(); ++i) {
        const double pixel_x =
            (static_cast<double>(high_corners[i].x) + 0.5) / supersample +
            (static_cast<double>(roi_x0) - 0.5);
        const double pixel_y =
            (static_cast<double>(high_corners[i].y) + 0.5) / supersample +
            (static_cast<double>(roi_y0) - 0.5);
        (*intersection_corners_map)[i] = {
            pixel_x / pixels_per_meter + world_offset_x,
            pixel_y / pixels_per_meter + world_offset_y};
    }
    return true;
}

cv::Mat DrivableClearanceMeters(
    const cv::Mat& drivable_mask, double pixels_per_meter) {
    cv::Mat result;
    if (drivable_mask.empty() || drivable_mask.type() != CV_8UC1 ||
        !(pixels_per_meter > 0.0)) {
        return result;
    }
    cv::distanceTransform(
        drivable_mask, result, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    result /= static_cast<float>(pixels_per_meter);
    return result;
}

double DrivablePointClearanceMeters(
    const cv::Mat& clearance_m,
    double pixels_per_meter,
    double world_offset_x,
    double world_offset_y,
    const cv::Point2d& point_map) {
    if (clearance_m.empty() || clearance_m.type() != CV_32FC1 ||
        !(pixels_per_meter > 0.0) || !std::isfinite(point_map.x) ||
        !std::isfinite(point_map.y)) {
        return 0.0;
    }
    const double x = pixels_per_meter * (point_map.x - world_offset_x);
    const double y = pixels_per_meter * (point_map.y - world_offset_y);
    if (x < 0.0 || y < 0.0 ||
        x > static_cast<double>(clearance_m.cols - 1) ||
        y > static_cast<double>(clearance_m.rows - 1)) {
        return 0.0;
    }
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = std::min(x0 + 1, clearance_m.cols - 1);
    const int y1 = std::min(y0 + 1, clearance_m.rows - 1);
    const double tx = x - x0;
    const double ty = y - y0;
    const double a = clearance_m.at<float>(y0, x0);
    const double b = clearance_m.at<float>(y0, x1);
    const double c = clearance_m.at<float>(y1, x0);
    const double d = clearance_m.at<float>(y1, x1);
    return (1.0 - ty) * ((1.0 - tx) * a + tx * b) +
        ty * ((1.0 - tx) * c + tx * d);
}
