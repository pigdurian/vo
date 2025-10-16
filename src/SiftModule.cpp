#include "SiftModule.h"
#include <opencv2/features2d.hpp>

cv::Ptr<cv::Feature2D> SiftModule::create() {
#if CV_VERSION_MAJOR >= 4
    return cv::SIFT::create(2000);
#else
    return cv::xfeatures2d::SIFT::create(2000);
#endif
}
