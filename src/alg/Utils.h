#pragma once

#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include <boost/algorithm/string.hpp>

class Utils{
public:
    static void list_files(const std::string &folder,std::vector<cv::String>&files_lists){
        cv::glob(folder,files_lists);
        assert(!files_lists.empty());
    }
};