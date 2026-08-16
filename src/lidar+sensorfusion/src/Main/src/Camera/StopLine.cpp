#include "Camera.hpp"

void FindStopLine(CAMERA& st_Camera, GPS_IMU &st_GPSIMU)
{
    if (st_Camera.bgr_image.empty())
    {
        return;
    }

    cv::Mat image = st_Camera.bgr_image.clone(); // 디스플레이를 위해 원본 복사
    int white_pixel_area = 0;

    // ============== 감지 로직 ==============
 
    if (!st_Camera.endSig) {
        cv::Mat hsvImage;

        // ROI 설정
        int roi_x = 300;
        int roi_y = 450;
        int roi_width = 500;
        int roi_height = 30;
        cv::Rect roi_rect(roi_x, roi_y, roi_width, roi_height);
        cv::Mat roi_image = image(roi_rect);

        cv::cvtColor(roi_image, hsvImage, cv::COLOR_BGR2HSV);

        // 흰색 마스크 생성
        vector<int32> lowerWhite = st_Camera.hsv_thresholds.lower_white;
        vector<int32> upperWhite = st_Camera.hsv_thresholds.upper_white;
        cv::Scalar lowerWhiteScalar(lowerWhite[0], lowerWhite[1], lowerWhite[2]);
        cv::Scalar upperWhiteScalar(upperWhite[0], upperWhite[1], upperWhite[2]);
        cv::Mat whiteMask;
        cv::inRange(hsvImage, lowerWhiteScalar, upperWhiteScalar, whiteMask);

        // 흰색 픽셀 면적 계산
        white_pixel_area = cv::countNonZero(whiteMask);
        //std::cout << "[DEBUG] White Pixel Area in ROI: " << white_pixel_area << std::endl;

        // 임계값 비교
        const int AREA_THRESHOLD = 1900;
        if(st_GPSIMU.st_GPS.b_isJamming)
        {
            if (white_pixel_area > AREA_THRESHOLD)
            {
                st_Camera.endSig = true;
            }
        }
    }

    // ============== 화면 표시 로직 ==============
    if (st_Camera.s32_Display)
    {

        int roi_x = 300;
        int roi_y = 450;
        int roi_width = 500;
        int roi_height = 30;
        cv::Rect roi_rect(roi_x, roi_y, roi_width, roi_height);
        cv::rectangle(image, roi_rect, cv::Scalar(0, 255, 0), 2);
        

        std::string text;
        if (st_Camera.endSig) {
            text = "White Area: (Detection Complete)";
        } else {
            text = "White Area: " + std::to_string(white_pixel_area);
        }
        cv::putText(image, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);


        if (st_Camera.endSig) {
            cv::putText(image, "ENDSIG ACTIVATED", cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        }
        
        // cv::imshow("Stop Line Detection", image);
    }
}