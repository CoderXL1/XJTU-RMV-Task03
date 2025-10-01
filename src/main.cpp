//
//  main.cpp
//  Created by Leo Xia on 2025/10/1
//  Created with VSCode on Ubuntu 22.04
//

#include <iostream>
#include <string>
#include <vector>
#include "data.hpp"
#include "frame_processing.hpp"
#include "solver.hpp"

double TASK03::x0 = 0;
double TASK03::y0 = 0;
double TASK03::h = 0;
double TASK03::w = 0;

const std::string video_path = "/home/leoxia/Code/XJTU-RMV-Task03/resources/video.mp4";
const double fps = 60.0;
std::vector<TASK03::Measurement> raw;
void print_raw()
{
    for(auto &m:raw)
        std::cout<<m.t<<","<<m.x<<","<<m.y<<"\n";
}
void print_summary(const ceres::Solver::Summary& summary)
{
    std::cout << summary.BriefReport() << "\n";
    std::cout << summary.FullReport() << "\n";
}
void print_results(const TASK03::Params& params)
{
    std::cout << "Estimated parameters:\n";
    std::cout << "vx0: " << params.vx0 << "\n";
    std::cout << "vy0: " << params.vy0 << "\n";
    std::cout << "k: " << exp(params.k_log) << "\n";
    std::cout << "g: " << params.g << "\n";
}
int main()
{
    // freopen("../outputs/results.txt","w",stdout);
    int frame_count;
    TASK03::Params params;
    ceres::Solver::Summary summary;
    frame_count = TASK03::detect_main(raw, video_path, fps);
    if(frame_count < 0)
    {
        std::cerr<<"Error in video processing.\n";
        return -1;
    }
    std::cout << "Total detections: " << raw.size() << "\n";
    // print_raw();
    TASK03::x0 = raw.front().x;
    TASK03::y0 = raw.front().y; 
    // std::cout<<TASK03::x0<<","<<TASK03::y0<<","<<TASK03::w<<","<<TASK03::h<<std::endl;
    // return 0;
    summary = TASK03::solve(raw, params, fps);
    // print_summary(summary);
    print_results(params);
    return 0;
}