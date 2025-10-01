#include "frame_processing.hpp"

namespace TASK03
{
    bool processFrame(const cv::Mat& frame, Measurement& m)
    {
        cv::Mat hsv,mask,kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {5,5});
        std::vector<std::vector<cv::Point>> contours;
        double bestArea = 0;
        cv::Point2f bestCenter;
        cv::Scalar lower_bound(0, 0, 100);
        cv::Scalar upper_bound(180, 255, 255);

        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, lower_bound, upper_bound, mask);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (auto &c : contours) {
            cv::Point2f center;
            double area = cv::contourArea(c);
            float radius;
            cv::minEnclosingCircle(c, center, radius);
            if (area > bestArea) {
                bestArea = area;
                bestCenter = center;
            }
        }
        if(bestArea < 100)
        {
            return false;
        }
        m.x=bestCenter.x;
        m.y=h-bestCenter.y;
        return true;
    }

    void test(const cv::Mat& frame)
    {
        cv::Mat hsv,mask,kernel;
        std::vector<std::vector<cv::Point>> contours;
        double bestArea = 0;
        float bestRadius = 0;
        cv::Point2f bestCenter;
        cv::Scalar lower_bound(0, 0, 100);
        cv::Scalar upper_bound(180, 255, 255);

        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, lower_bound, upper_bound, mask);
        // cv::imwrite("../outputs/test/test2.png", mask);
        kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {5,5});
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (auto &c : contours) {
            cv::Point2f center;
            double area = cv::contourArea(c);
            float radius;
            cv::minEnclosingCircle(c, center, radius);
            if (area > bestArea) {
                bestArea = area;
                bestCenter = center;
                bestRadius = radius;
            }
        }
        std::cout<<"Best circle: center="<<bestCenter<<", radius="<<bestRadius<<", area="<<bestArea<<std::endl;
        cv::circle(frame, bestCenter, bestRadius, {0,255,0}, 2);
        cv::imshow("frame", frame);
        cv::waitKey(0);
    }
    int detect_main(std::vector<Measurement>& raw, const std::string& video_path, double fps)
    {
        int frame_count;
        cv::Mat frame;
        cv::VideoCapture cap(video_path);
        if (!cap.isOpened()) {
            std::cerr << "Failed to open video: " << video_path << "\n";
            return -1;
        }
        frame_count = (int)cap.get(cv::CAP_PROP_FRAME_COUNT);
        h = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        w = cap.get(cv::CAP_PROP_FRAME_WIDTH);
        // std::cout << "Video resolution: " << w << "x" << h << "\n";
        // std::cout << "Total frames: " << frame_count << "\n";
        raw.clear();
        raw.reserve(frame_count);
        for(int i=1;i<=frame_count;i++)
        {
            TASK03::Measurement meas;
            cap>>frame;
            if(!TASK03::processFrame(frame, meas))
                continue;
            meas.t = i/fps;
            raw.push_back(meas);
        }
        cap.release();
        return frame_count;
    }
}
