#include <iostream>
#include <opencv2/opencv.hpp>
#include "data.hpp"

namespace TASK03
{
    bool processFrame(const cv::Mat& frame, Measurement& m);
    void pftest(const cv::Mat& frame);
    int detect_main(std::vector<Measurement>& measurements, const std::string& video_path, double fps);
}