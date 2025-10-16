#pragma once
#include <opencv2/opencv.hpp>

struct SiftModule {
    static cv::Ptr<cv::Feature2D> create();  // 工厂：返回一个特征子
};
