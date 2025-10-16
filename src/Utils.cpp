// Utils.cpp
#include "Utils.h"
using namespace cv;

QImage cvMatToQImageGray(const Mat& m) {
    CV_Assert(m.type()==CV_8UC1);
    return QImage(m.data, m.cols, m.rows, m.step, QImage::Format_Grayscale8).copy();
}
QImage cvMatToQImageColor(const Mat& m) {
    Mat bgr;
    if (m.channels()==1) cvtColor(m, bgr, COLOR_GRAY2BGR);
    else bgr = m;
    Mat rgb; cvtColor(bgr, rgb, COLOR_BGR2RGB);
    return QImage(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888).copy();
}
