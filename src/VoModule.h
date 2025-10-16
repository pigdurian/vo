#ifndef VO_MODULE_H
#define VO_MODULE_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

struct VoResult {
    bool ok = false;
    cv::Mat R;             // 累积旋转（3x3, CV_64F）
    cv::Mat t;             // 累积平移（3x1, CV_64F）
    cv::Point3d t_world;   // 当前相机位移 (x,y,z)
    cv::Mat overlay;       // 叠字的当前帧
    cv::Mat matchesVis;    // 轨迹画布
};

class VoEngine {
public:
    VoEngine() = default;

    // base: 序列根目录（含 image_0/000000.png ...）
    // K: 相机内参（可选；不传则用默认 focal_/pp_）
    // poses_root: 可选：KITTI poses/00.txt 目录，用于绝对尺度
    bool init(const std::string& base,
              const cv::Mat& K = cv::Mat(),
              const std::string& poses_root = "");

    // 推进到第 numFrame 帧（从 0 开始）
    VoResult step(int numFrame);

    // 导出
    bool exportMapPLY(const std::string& path) const;      // 累计三角化点（世界系）
    bool exportFeaturesCSV(const std::string& path) const; // 当前帧 inlier 2D 像素

    // 取图
    cv::Mat getTrajectory() const { return trajCanvas_; }
    cv::Mat getCurrentPose() const { return Tcw_; }

private:
    double getAbsoluteScale(int frame_id) const;
    static bool writeTextFile(const std::string& path, const std::string& content);

private:
    // 路径/相机
    std::string base_;
    std::string poses_root_;
    cv::Mat K_;          // 3x3, CV_64F

    // 累计位姿（相机->世界）
    cv::Mat R_f_;        // 3x3, CV_64F
    cv::Mat t_f_;        // 3x1, CV_64F
    cv::Mat Tcw_;        // 4x4, CV_64F

    // 可视化
    cv::Mat trajCanvas_ = cv::Mat::zeros(600, 800, CV_8UC3);

    // 跟踪缓存
    cv::Mat prevImage_;                       // 上一帧灰度
    std::vector<cv::Point2f> prevFeatures_;   // 上一帧特征（像素）

    // 导出缓存
    std::vector<cv::Point2f> last_inliers_;   // 当前帧 inlier 2D
    std::vector<cv::Point3d> map_points_;     // 累计 3D（世界系）

    // 默认内参（KITTI 00）
    double    focal_ = 718.8560;
    cv::Point2d pp_  = {607.1928, 185.2157};
};

#endif // VO_MODULE_H
