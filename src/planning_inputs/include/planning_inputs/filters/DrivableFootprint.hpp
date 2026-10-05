#pragma once

#include <array>

#include <opencv2/core.hpp>

// OBB 바닥 사각형과 주행가능영역 래스터가 겹치는 면적 비율을 계산한다.
// 반환값은 [0, 1]이며, 맵 밖 영역은 주행불가로 취급한다.
double DrivableFootprintRatio(
    const cv::Mat& drivable_mask,
    double pixels_per_meter,
    double world_offset_x,
    double world_offset_y,
    const std::array<cv::Point2d, 4>& corners_map,
    int supersample = 4);

// OBB와 주행가능영역이 실제로 겹치는 부분을 planner용 최소 회전 사각형으로
// 근사한다. 반환되는 사각형은 map 좌표계이며, 원본 OBB 전체가 아니라 road
// mask와 교차한 면적만 감싼다. 교차 면적이 없거나 입력이 잘못되면 false다.
bool DrivableFootprintIntersectionRect(
    const cv::Mat& drivable_mask,
    double pixels_per_meter,
    double world_offset_x,
    double world_offset_y,
    const std::array<cv::Point2d, 4>& corners_map,
    std::array<cv::Point2d, 4>* intersection_corners_map,
    int supersample = 4);

// road 내부에서 가장 가까운 경계까지의 거리 맵을 만든다. 단위는 meter다.
cv::Mat DrivableClearanceMeters(
    const cv::Mat& drivable_mask, double pixels_per_meter);

// 연속 픽셀 좌표를 bilinear 보간해 점의 road 내부 깊이를 반환한다.
// 맵 밖 또는 road 밖은 0이다.
double DrivablePointClearanceMeters(
    const cv::Mat& clearance_m,
    double pixels_per_meter,
    double world_offset_x,
    double world_offset_y,
    const cv::Point2d& point_map);
