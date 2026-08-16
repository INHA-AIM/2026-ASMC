#include "Camera.hpp"
#include <vector>
#include <numeric>
#include <map>
#include <algorithm>
#include <iostream>

// 이미지 필터링 (HSV)
cv::Mat img_filter_for_lane_points(CAMERA& st_Camera, const cv::Mat& image) {
    cv::Mat hsv, bright_yellow_mask, dark_yellow_mask, yellow_mask;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    const auto& th = st_Camera.hsv_thresholds;
    cv::Scalar lower_bright(th.lower_bright_yellow[0], th.lower_bright_yellow[1], th.lower_bright_yellow[2]);
    cv::Scalar upper_bright(th.upper_bright_yellow[0], th.upper_bright_yellow[1], th.upper_bright_yellow[2]);
    cv::Scalar lower_dark(th.lower_dark_yellow[0], th.lower_dark_yellow[1], th.lower_dark_yellow[2]);
    cv::Scalar upper_dark(th.upper_dark_yellow[0], th.upper_dark_yellow[1], th.upper_dark_yellow[2]);

    cv::inRange(hsv, lower_bright, upper_bright, bright_yellow_mask);
    cv::inRange(hsv, lower_dark, upper_dark, dark_yellow_mask);
    cv::bitwise_or(bright_yellow_mask, dark_yellow_mask, yellow_mask);
    
    return yellow_mask;
}

// 각 수평 구간에서 차선 대표점을 찾는 함수
std::map<int, std::vector<cv::Point>> findLanePointsBySection(const cv::Mat& masked_image, int n_sections) {
    std::map<int, std::vector<cv::Point>> section_points;
    int section_height = masked_image.rows / n_sections;
    const double MIN_PIXEL_AREA = 30.0;
    const double MIN_LANE_DISTANCE = 100.0;

    for (int i = 0; i < n_sections; ++i) {
        int y_start = masked_image.rows - (i + 1) * section_height;
        cv::Rect roi_rect(0, y_start, masked_image.cols, section_height);
        cv::Mat section = masked_image(roi_rect);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(section, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        std::vector<cv::Moments> valid_moments;
        for (const auto& contour : contours) {
            cv::Moments M = cv::moments(contour);
            if (M.m00 > MIN_PIXEL_AREA) {
                valid_moments.push_back(M);
            }
        }

        std::sort(valid_moments.begin(), valid_moments.end(), [](const cv::Moments& a, const cv::Moments& b) {
            return a.m00 > b.m00;
        });

        std::vector<cv::Point> points_in_section;
        int count = 0;
        for (const auto& M : valid_moments) {
            if (count >= 2) break;
            cv::Point center(M.m10 / M.m00, M.m01 / M.m00);
            points_in_section.push_back(cv::Point(center.x, center.y + y_start));
            count++;
        }

        if (points_in_section.size() == 2) {
            double distance = cv::norm(points_in_section[0] - points_in_section[1]);
            if (distance < MIN_LANE_DISTANCE) {
                points_in_section.clear();
            }
        }

        if (!points_in_section.empty()) {
            section_points[i] = points_in_section;
        }
    }
    return section_points;
}

// 차선 분류 함수
void classifyLanePoints(std::map<int, std::vector<cv::Point>>& section_points, int n_sections,
                        std::vector<cv::Point>& left_lane_pts, std::vector<cv::Point>& right_lane_pts) {
    
    std::map<int, cv::Point> left_classified, right_classified;

    for (int i = 0; i < n_sections; ++i) {
        if (section_points.count(i) && section_points[i].size() == 2) {
            if (section_points[i][0].x < section_points[i][1].x) {
                left_classified[i] = section_points[i][0];
                right_classified[i] = section_points[i][1];
            } else {
                left_classified[i] = section_points[i][1];
                right_classified[i] = section_points[i][0];
            }
        }
    }

    for (int i = 0; i < n_sections; ++i) {
        if (section_points.count(i) && section_points[i].size() == 1) {
            cv::Point& single_pt = section_points[i][0];
            
            int closest_ref_idx = -1;
            for (int d = 1; d < n_sections; ++d) {
                int upper_idx = i + d;
                int lower_idx = i - d;
                bool found = false;
                if (upper_idx < n_sections && left_classified.count(upper_idx) && right_classified.count(upper_idx)) {
                    closest_ref_idx = upper_idx; found = true;
                } else if (lower_idx >= 0 && left_classified.count(lower_idx) && right_classified.count(lower_idx)) {
                    closest_ref_idx = lower_idx; found = true;
                }
                if (found) break;
            }

            if (closest_ref_idx != -1) {
                double dist_to_left = cv::norm(single_pt - left_classified[closest_ref_idx]);
                double dist_to_right = cv::norm(single_pt - right_classified[closest_ref_idx]);
                if (dist_to_left < dist_to_right) {
                    left_classified[i] = single_pt;
                } else {
                    right_classified[i] = single_pt;
                }
            }
        }
    }

    for (auto const& [key, val] : left_classified) left_lane_pts.push_back(val);
    for (auto const& [key, val] : right_classified) right_lane_pts.push_back(val);
}

// 1차 함수(직선)를 피팅하고 파라미터를 반환
cv::Vec4f fitLine(const std::vector<cv::Point>& points) {
    if (points.size() < 2) return cv::Vec4f(0,0,0,0);
    cv::Vec4f line_params;
    cv::fitLine(points, line_params, cv::DIST_L2, 0, 0.01, 0.01);
    return line_params;
}


// ★★★ 요청하신 로직으로 최종 수정한 함수 ★★★
void calculateSteeringAngle(cv::Mat& image, CAMERA& st_Camera, const cv::Vec4f& left_line, const cv::Vec4f& right_line) {
    if (left_line == cv::Vec4f(0,0,0,0) || right_line == cv::Vec4f(0,0,0,0)) {
        st_Camera.lane_angle = 0.0;
        return;
    }

    // 1. 직선 파라미터 및 주요 포인트 정의
    double vx_l = left_line[0], vy_l = left_line[1], x_l = left_line[2], y_l = left_line[3];
    double vx_r = right_line[0], vy_r = right_line[1], x_r = right_line[2], y_r = right_line[3];
    cv::Point car_center_bottom(image.cols / 2, image.rows);

    // 2. 소실점 계산
    double det = vx_l * vy_r - vx_r * vy_l;
    if (std::abs(det) < 1e-6) return;
    double c_l = vy_l * x_l - vx_l * y_l;
    double c_r = vy_r * x_r - vx_r * y_r;
    double vanish_x = (vx_l * c_r - vx_r * c_l) / det;
    double vanish_y = (vy_l * c_r - vy_r * c_l) / det;
    cv::Point vp(vanish_x, vanish_y);

    // 3. 실제 차선 하단 중앙점 계산
    int left_bottom_x = static_cast<int>(x_l + (image.rows - y_l) / vy_l * vx_l);
    int right_bottom_x = static_cast<int>(x_r + (image.rows - y_r) / vy_r * vx_r);
    double lane_center_x_bottom = (left_bottom_x + right_bottom_x) / 2.0;
    cv::Point lane_center_bottom(lane_center_x_bottom, image.rows);

    // 4. 각도 계산
    // 벡터 a (화면 중앙 -> 소실점)
    cv::Point vec_a = vp - car_center_bottom;
    // 벡터 c (차선 중앙 -> 소실점)
    cv::Point vec_c = vp - lane_center_bottom;

    // 각 벡터의 각도 (수직 위쪽 기준)
    double angle_a_rad = std::atan2(vec_a.x, -vec_a.y);
    double angle_c_rad = std::atan2(vec_c.x, -vec_c.y);
    
    // 두 벡터 사이의 사잇각 x 계산
    double angle_x_rad = angle_c_rad - angle_a_rad;
    
    // 최종 조향각 계산 (기본 방향각에 보정각 x를 더하거나 빼줌)
    double base_angle_deg = angle_a_rad * 180.0 / CV_PI;
    
    // x의 부호가 잘못되었다는 지적을 반영하여, x의 부호를 뒤집어(빼기) 사용
    st_Camera.lane_angle = base_angle_deg - (angle_x_rad * 180.0 / CV_PI);

    // 5. 시각화 및 디버깅 출력
    if (st_Camera.s32_Display) {
        // 직선 a (화면 중앙 기준, 연두색)
        cv::line(image, car_center_bottom, vp, cv::Scalar(150, 255, 150), 2);
        // 직선 c (차선 중앙 기준, 진한 녹색)
        cv::line(image, lane_center_bottom, vp, cv::Scalar(0, 255, 0), 2);


        // std::cout << "Angle(a): " << base_angle_deg << ", Angle(c): " << angle_c_rad * 180.0 / CV_PI
        //           << ", Angle(x): " << angle_x_rad * 180.0 / CV_PI 
        //           << ", Final Angle: " << st_Camera.lane_angle << std::endl;
    }
}


// 차선 인식 메인 함수
void LaneDetectionProcess(CAMERA& st_Camera) {
    if (st_Camera.bgr_image.empty()) return;

    cv::Mat image = st_Camera.bgr_image.clone();
    cv::Mat roi_image; 
    cv::Point vertices[1][4];
    vertices[0][0] = cv::Point(image.cols * 0.01, image.rows);
    vertices[0][1] = cv::Point(image.cols * 0.4, image.rows * 0.5);
    vertices[0][2] = cv::Point(image.cols * 0.6, image.rows * 0.5);
    vertices[0][3] = cv::Point(image.cols * 0.99, image.rows);
    cv::Mat mask = cv::Mat::zeros(image.size(), image.type());
    const cv::Point* ppt[1] = { vertices[0] };
    int npt[] = { 4 };
    cv::fillPoly(mask, ppt, npt, 1, cv::Scalar(255, 255, 255));
    cv::bitwise_and(image, mask, roi_image);
    cv::Mat masked_img = img_filter_for_lane_points(st_Camera, roi_image);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat opened_mask;
    cv::morphologyEx(masked_img, opened_mask, cv::MORPH_OPEN, kernel);
    int num_sections = 40;
    auto section_points = findLanePointsBySection(opened_mask, num_sections);
    std::vector<cv::Point> left_lane_points, right_lane_points;
    classifyLanePoints(section_points, num_sections, left_lane_points, right_lane_points);

    if(st_Camera.s32_Display){
        // 대표선 피팅
        cv::Vec4f left_line_params = fitLine(left_lane_points);
        cv::Vec4f right_line_params = fitLine(right_lane_points);
        
        // 대표선 그리기
        if (left_line_params != cv::Vec4f(0,0,0,0))
            cv::line(image, cv::Point(left_line_params[2] + (image.rows - left_line_params[3]) / left_line_params[1] * left_line_params[0], image.rows), cv::Point(left_line_params[2] + (image.rows*0.5 - left_line_params[3]) / left_line_params[1] * left_line_params[0], image.rows*0.5), cv::Scalar(255,255,0),3);
        if (right_line_params != cv::Vec4f(0,0,0,0))
            cv::line(image, cv::Point(right_line_params[2] + (image.rows - right_line_params[3]) / right_line_params[1] * right_line_params[0], image.rows), cv::Point(right_line_params[2] + (image.rows*0.5 - right_line_params[3]) / right_line_params[1] * right_line_params[0], image.rows*0.5), cv::Scalar(255,0,255),3);

        // 조향각 계산 및 시각화
        calculateSteeringAngle(image, st_Camera, left_line_params, right_line_params);
        
        // cv::imshow("Lane Control", image);
        cv::waitKey(1);
    }
}