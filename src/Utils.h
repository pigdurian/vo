// Utils.h
#pragma once
#include <QImage>
#include <opencv2/opencv.hpp>

QImage cvMatToQImageGray(const cv::Mat& m);
QImage cvMatToQImageColor(const cv::Mat& m);
