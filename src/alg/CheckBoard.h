#pragma once

#include <iostream>
#include <cassert>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include "src/alg/Utils.h"

class CheckBoard
{
public:
    int points_per_row;    // 某行的内角点数量
    int points_per_col;    // 某列的内角点数量
    cv::Size pattern_size; // 棋盘格的尺寸
    float square_size;     // 方块格的实际大小

    std::vector<cv::String> calib_images; // 标定的图片
    cv::Size calib_images_size;           // 标定图片的尺寸大小

    std::vector<std::vector<cv::Point3f>> corners_world_all; // 角点（世界坐标）
    std::vector<std::vector<cv::Point2f>> corners_pixel_all; // 角点（像素坐标）

public:
    CheckBoard(int points_per_col, int points_per_row, float square_size)
    {
        assert(points_per_col > 0);
        this->points_per_col = points_per_col;
        assert(points_per_row > 0);
        this->points_per_row = points_per_row;

        this->pattern_size = cv::Size(points_per_row, points_per_col);
        if (square_size != -1)
        {
            assert(square_size > 0);
            this->square_size = square_size;
        }
    };

public:
    void detect_coners(const std::string &calib_folder)
    {
        calib_images.resize(0);
        corners_pixel_all.resize(0);
        corners_world_all.resize(0);

        Utils::list_files(calib_folder, calib_images);
        calib_images_size = cv::imread(calib_images[0]).size();

        std::cout << "开始检测角点！" << std::endl;
        std::cout << "----------------------" << std::endl;

        for (int idx = 0; idx < calib_images.size(); ++idx)
        {
            _add_corners_world();
        }

        cv::TermCriteria ctiteria = cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1);

        std::vector<int> idxs_delete;

        for (int idx = 0; idx < calib_images.size(); ++idx)
        {
            std::string filename = calib_images[idx];
            cv::Mat img = cv::imread(filename);
            bool found = _add_corners_pixel(img, ctiteria);
            if (!found)
            {
                idxs_delete.emplace_back(idx);
                std::cout << "fail:\t" << filename << std::endl;
            }
            else
            {
                std::cout << "sucess:\t" << filename << std::endl;
            }
        }

        for (auto idx : idxs_delete)
        {
            calib_images.erase(calib_images.begin() + idx);
            corners_world_all.erase(corners_world_all.begin() + idx);
        }
    }

    void show_params()
    {
        std::cout << "标定板尺寸(points_per_row, points_per_col):\t" << pattern_size << std::endl;
        std::cout << "棋盘格大小:\t" << square_size << std::endl;
    }

public:
    void _add_corners_world()
    {
        if (corners_world_all.empty())
        {
            std::vector<cv::Point3f> cornersWorld;
            for (int row = 0; row < points_per_col; ++row)
            {
                for (int col = 0; col < points_per_row; ++col)
                {
                    cornersWorld.emplace_back(float(col) * square_size, float(row) * square_size, 0.f);
                }
            }
            corners_world_all.emplace_back(cornersWorld);
        }
        else
        {
            corners_world_all.emplace_back(corners_world_all[0]);
        }
    }

    bool _add_corners_pixel(const cv::Mat &img, cv::TermCriteria &criteria)
    {
        assert(calib_images_size == img.size());
        std::vector<cv::Point2f> corners_pixel;
        bool found = cv::findChessboardCorners(img, pattern_size, corners_pixel);
        if (found)
        {
            cv::Mat img_gray;
            cv::cvtColor(img, img_gray, CV_BGR2GRAY);

            cv::cornerSubPix(img_gray, corners_pixel, cv::Size(5, 5), cv::Size(-1, -1), criteria);
            corners_pixel_all.emplace_back(corners_pixel);
        }
        return found;
    }
};
