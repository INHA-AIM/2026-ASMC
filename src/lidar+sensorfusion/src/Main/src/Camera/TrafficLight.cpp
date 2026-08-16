#include "Camera.hpp"

void IdentifyTrafficLight(CAMERA& st_Camera)
{
    if (!st_Camera.is_traffic_light_camera_available) return;
    const cv::Mat& src = st_Camera.traffic_light_image;
    if (src.empty() || src.cols < 2 || src.rows < 2) {
        st_Camera.traffic_light_state.str_Color = "unknown";
        st_Camera.traffic_light_state.f64_ElapsedTime = 0.0;
        return;
    }

    // --- ROI (기존 고정값을 이미지 경계에 맞게 클램프) ---
    const int ROI_X = 1, ROI_Y = 1, ROI_W = 450, ROI_H = 299;
    const int x = std::max(0, ROI_X);
    const int y = std::max(0, ROI_Y);
    const int w = std::max(0, std::min(ROI_W, src.cols - x));
    const int h = std::max(0, std::min(ROI_H, src.rows - y));
    if (w == 0 || h == 0) {
        st_Camera.traffic_light_state.str_Color = "unknown";
        st_Camera.traffic_light_state.f64_ElapsedTime = 0.0;
        return;
    }

    cv::Mat roi = src(cv::Rect(x, y, w, h));
    cv::Mat scaled, blurred, hsv;
    cv::resize(roi, scaled, cv::Size(), 2.0, 2.0, cv::INTER_LINEAR);
    cv::GaussianBlur(scaled, blurred, cv::Size(5, 5), 0);
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

    // --- HSV inRange 카운트 헬퍼 ---
    auto inRangeCount = [&](const cv::Scalar& lo, const cv::Scalar& hi) -> int {
        cv::Mat mask; cv::inRange(hsv, lo, hi, mask);
        return cv::countNonZero(mask);
    };

    // --- HSV 범위 (기존 st_Camera.hsv_thresholds 사용) ---
    const auto& th = st_Camera.hsv_thresholds;
    const cv::Scalar lowR1(th.lower_red1[0], th.lower_red1[1], th.lower_red1[2]);
    const cv::Scalar upR1 (th.upper_red1[0], th.upper_red1[1], th.upper_red1[2]);
    const cv::Scalar lowR2(th.lower_red2[0], th.lower_red2[1], th.lower_red2[2]);
    const cv::Scalar upR2 (th.upper_red2[0], th.upper_red2[1], th.upper_red2[2]);
    const cv::Scalar lowG (th.lower_green[0],  th.lower_green[1],  th.lower_green[2]);
    const cv::Scalar upG  (th.upper_green[0],  th.upper_green[1],  th.upper_green[2]);
    const cv::Scalar lowY (th.lower_yellow[0], th.lower_yellow[1], th.lower_yellow[2]);
    const cv::Scalar upY  (th.upper_yellow[0], th.upper_yellow[1], th.upper_yellow[2]);

    // --- 색상별 픽셀 카운트 ---
    const int redCount    = inRangeCount(lowR1, upR1) + inRangeCount(lowR2, upR2);
    const int greenCount  = inRangeCount(lowG , upG );
    const int yellowCount = inRangeCount(lowY , upY );

    //std::cout << "[PIXEL COUNT] Red: " << redCount << ", Green: " << greenCount << ", Yellow: " << yellowCount << std::endl;


    // --- 색상별 임계치 (공통 threshold 기반) ---
    const int base = st_Camera.traffic_light_state.s32_Threshold;
    const int TH_RED    = static_cast<int>(base * 0.8);
    const int TH_GREEN  = static_cast<int>(base * 1.0);
    const int TH_YELLOW = static_cast<int>(base * 0.08);

    // --- 판정 로직: 절대 임계치 기반 ---
    bool isRed    = (redCount   >= TH_RED);
    bool isGreen  = (greenCount >= TH_GREEN);
    bool isYellow = (yellowCount>= TH_YELLOW);

    std::string decided = "unknown";

    if (yellowCount >= TH_YELLOW) {
        decided = "yellow";   // yellow가 최우선
    }
    else if (redCount >= TH_RED) {
        decided = "red";
    }
    else if (greenCount >= TH_GREEN) {
        decided = "green";
    }
    else {
        decided = "unknown";
    }

    //std::cout << "[TrafficLight Result] " << decided << std::endl;

    // --- 상태 변화/경과시간 관리 ---
    static std::string prev = "unknown";
    static ros::Time lastChange = ros::Time::now();
    if (decided != prev) {
        prev = decided;
        lastChange = ros::Time::now();
    }
    st_Camera.traffic_light_state.str_Color = decided;
    st_Camera.traffic_light_state.f64_ElapsedTime =
        (ros::Time::now() - lastChange).toSec();

    // --- 디스플레이 ---
    if (st_Camera.s32_Display) {
        cv::Mat vis = src.clone();
        cv::rectangle(vis, cv::Rect(x, y, w, h), cv::Scalar(0, 255, 255), 2);
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "TL:%s (%.0fs) [R:%d/%d G:%d/%d Y:%d/%d]",
                      decided.c_str(),
                      st_Camera.traffic_light_state.f64_ElapsedTime,
                      redCount, TH_RED, greenCount, TH_GREEN, yellowCount, TH_YELLOW);

        cv::putText(vis, decided, cv::Point(30, 50),
                    cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0,255,0), 3, cv::LINE_AA);

        // cv::imshow("Traffic Light Cam", vis);
        // cv::waitKey(1);
    }
}
