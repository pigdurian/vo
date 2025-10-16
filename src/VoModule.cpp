#include "VoModule.h"
#include "alg/vo_features.h" // 提供 featureDetection(img, pts)
#include <fstream>
#include <sstream>
#include <iostream>
#include <cctype>

using namespace cv;
using std::vector;

// ----------------- 工具与调试 -----------------
static const char* dstr(int d) {
    switch (d) { case CV_32F: return "32F"; case CV_64F: return "64F"; default: return "?"; }
}

static void dumpTypes(const cv::Mat& F, const cv::Mat& K,
                      const cv::Mat& R, const cv::Mat& t,
                      const cv::Mat& Rf, const cv::Mat& tf) {
    fprintf(stderr, "F:%s K:%s | R:%s t:%s | Rf:%s tf:%s\n",
            dstr(F.depth()), dstr(K.depth()),
            dstr(R.depth()), dstr(t.depth()),
            dstr(Rf.depth()), dstr(tf.depth()));
}

static inline std::string imgName(int idx) {
    char b[16];
    std::snprintf(b, sizeof(b), "%06d.png", idx);
    return b;
}

static inline std::string normalizeSlashes(std::string s) {
    for (auto& c : s) if (c == '\\') c = '/';
    return s;
}

// ----------------- poses 读入与尺度 -----------------
bool VoEngine::loadPoses(const std::string& poses_root_in, const std::string& seq_id) {
    poses_gt_.clear();

    std::string root = normalizeSlashes(poses_root_in);
    std::vector<std::string> candidates = {
        root + "/dataset/poses/" + seq_id + ".txt",
        root + "/poses/" + seq_id + ".txt",
        root + "/" + seq_id + ".txt"
    };

    std::ifstream fin;
    for (auto& p : candidates) {
        fin.open(p);
        if (fin.is_open()) {
            std::cout << "[loadPoses] using: " << p << "\n";
            break;
        }
    }
    if (!fin.is_open()) {
        std::cerr << "[loadPoses] cannot open poses file under root: " << root << "\n";
        return false;
    }

    std::string line;
    while (std::getline(fin, line)) {
        std::stringstream ss(line);
        double m[12];
        for (int i=0;i<12;i++) {
            if (!(ss >> m[i])) { fin.close(); poses_gt_.clear(); return false; }
        }
        cv::Mat T = (cv::Mat_<double>(3,4) <<
            m[0], m[1], m[2],  m[3],
            m[4], m[5], m[6],  m[7],
            m[8], m[9], m[10], m[11]
        );
        poses_gt_.push_back(T);
    }
    fin.close();
    return !poses_gt_.empty();
}

double VoEngine::getAbsoluteScale(int frame_id) const {
    // 返回 frame_{id-1} -> frame_{id} 的真值位移长度（米）
    if (poses_gt_.empty()) return 1.0;
    if (frame_id <= 0)     return 1.0;
    if (frame_id >= (int)poses_gt_.size()) return 1.0;

    cv::Mat t_prev = poses_gt_[frame_id-1](cv::Rect(3,0,1,3)); // 3x1
    cv::Mat t_curr = poses_gt_[frame_id  ](cv::Rect(3,0,1,3)); // 3x1
    return cv::norm(t_curr - t_prev);
}

// ----------------- 初始化 -----------------
bool VoEngine::init(const std::string& base, const Mat& K, const std::string& poses_root)
{
    base_ = normalizeSlashes(base);
    poses_root_ = normalizeSlashes(poses_root);

    // 从 base 推断序列号（…/sequences/00）
    {
        std::string tail = base_;
        auto pos = base_.find_last_of('/');
        if (pos != std::string::npos) tail = base_.substr(pos+1);
        if (tail.size()==2 && std::isdigit((unsigned char)tail[0]) && std::isdigit((unsigned char)tail[1]))
            seq_id_ = tail;
        else
            seq_id_ = "00";
    }

    // 内参
    K_ = K.empty() ? (Mat_<double>(3, 3) << focal_, 0, pp_.x, 0, focal_, pp_.y, 0, 0, 1)
                   : K.clone();
    K_.convertTo(K_, CV_64F);

    // 清零累计位姿
    R_f_ = Mat::eye(3, 3, CV_64F);
    t_f_ = (Mat_<double>(3, 1) << 0, 0, 0);
    Tcw_ = Mat::eye(4, 4, CV_64F);

    // 画布
    if (trajCanvas_.empty())
        trajCanvas_ = cv::Mat::zeros(600, 800, CV_8UC3);
    else
        trajCanvas_.setTo(cv::Scalar::all(0));

    map_points_.clear();
    last_inliers_.clear();

    // 读前两帧（同一相机：image_0）
    Mat img1c = imread(base_ + "/image_0/" + imgName(0), IMREAD_COLOR);
    Mat img2c = imread(base_ + "/image_0/" + imgName(1), IMREAD_COLOR);
    if (img1c.empty() || img2c.empty())
        return false;

    Mat img1, img2;
    cvtColor(img1c, img1, COLOR_BGR2GRAY);
    cvtColor(img2c, img2, COLOR_BGR2GRAY);

    // 检测+LK 跟踪
    vector<Point2f> pts1, pts2;
    featureDetection(img1, pts1);
    vector<uchar> status;
    vector<float> err;
    calcOpticalFlowPyrLK(img1, img2, pts1, pts2, status, err);

    vector<Point2f> p1, p2;
    p1.reserve(status.size());
    p2.reserve(status.size());
    for (size_t i = 0; i < status.size(); ++i)
        if (status[i]) {
            p1.push_back(pts1[i]);
            p2.push_back(pts2[i]);
        }
    if (p1.size() < 8)
        return false;

    Mat maskE;
    Mat E = findEssentialMat(p1, p2, K_, RANSAC, 0.999, 1.0, maskE);
    if (E.empty())
        return false;

    vector<Point2f> p1_in, p2_in;
    p1_in.reserve(maskE.total());
    p2_in.reserve(maskE.total());
    for (int i = 0; i < (int)maskE.total(); ++i)
        if (maskE.at<uchar>(i)) {
            p1_in.push_back(p1[i]);
            p2_in.push_back(p2[i]);
        }
    if (p1_in.size() < 8)
        return false;

    Mat R, t, poseInliers;
    recoverPose(E, p1_in, p2_in, K_, R, t, poseInliers);
    R.convertTo(R, CV_64F);
    t.convertTo(t, CV_64F);

    R_f_ = R.clone();
    t_f_ = t.clone();

    prevImage_ = img2.clone();
    prevFeatures_ = p2_in.empty() ? p2 : p2_in;
    last_inliers_ = p2_in;

    // 画一个起点（x=tx, y=tz）
    auto clamp = [](int v, int lo, int hi){ return std::max(lo, std::min(hi, v)); };
    int x0 = clamp(int(1.0 * t_f_.at<double>(0)) + 300, 0, trajCanvas_.cols - 1);
    int y0 = clamp(int(1.0 * t_f_.at<double>(2)) + 100, 0, trajCanvas_.rows - 1);
    circle(trajCanvas_, Point(x0, y0), 2, CV_RGB(0, 255, 0), -1, cv::LINE_AA);

    // 更新 Tcw_
    Mat Tk = Mat::eye(4, 4, CV_64F);
    R_f_.copyTo(Tk(Rect(0, 0, 3, 3)));
    t_f_.copyTo(Tk(Rect(3, 0, 1, 3)));
    Tcw_ = Tk.clone();

    // 载入 GT poses（可选）
    if (!poses_root_.empty()) {
        if (!loadPoses(poses_root_, seq_id_)) {
            std::cerr << "[init] warning: failed to load poses for seq " << seq_id_ << "\n";
        } else {
            std::cout << "[init] poses loaded: " << poses_gt_.size() << " frames for seq " << seq_id_ << "\n";
        }
    }
    return true;
}

// ----------------- 前进一帧 -----------------
VoResult VoEngine::step(int numFrame)
{
    VoResult out;

    // 当前帧（同一相机：image_0）
    Mat currC = imread(base_ + "/image_0/" + imgName(numFrame), IMREAD_COLOR);
    if (currC.empty())
        return out;

    Mat curr;
    cvtColor(currC, curr, COLOR_BGR2GRAY);

    // 跟踪
    vector<Point2f> currFeatures;
    vector<uchar> status;
    vector<float> err;
    calcOpticalFlowPyrLK(prevImage_, curr, prevFeatures_, currFeatures, status, err);

    vector<Point2f> prevGood, currGood;
    prevGood.reserve(status.size());
    currGood.reserve(status.size());
    for (size_t i = 0; i < status.size(); ++i)
        if (status[i]) {
            prevGood.push_back(prevFeatures_[i]);
            currGood.push_back(currFeatures[i]);
        }

    auto refreshAndReturn = [&](bool redetect)
    {
        prevImage_ = curr.clone();
        if (redetect) {
            prevFeatures_.clear();
            featureDetection(curr, prevFeatures_);
        } else {
            prevFeatures_ = currGood;
        }
        out.ok = false;
        return out;
    };

    if (prevGood.size() < 8)
        return refreshAndReturn(true);

    // Essential + RANSAC
    Mat maskE;
    Mat E = findEssentialMat(prevGood, currGood, K_, RANSAC, 0.999, 1.0, maskE);
    if (E.empty())
        return refreshAndReturn(true);

    vector<Point2f> prevIn, currIn;
    prevIn.reserve(maskE.total());
    currIn.reserve(maskE.total());
    for (int i = 0; i < (int)maskE.total(); ++i)
        if (maskE.at<uchar>(i)) {
            prevIn.push_back(prevGood[i]);
            currIn.push_back(currGood[i]);
        }
    if (prevIn.size() < 8)
        return refreshAndReturn(true);

    // 相对位姿
    Mat R, t;
    Mat poseInliers;
    recoverPose(E, prevIn, currIn, K_, R, t, poseInliers);

    // ---------- 三角化到世界系 ----------
    Mat R_prev = R_f_.clone(); // 上一帧 相机->世界
    Mat t_prev = t_f_.clone();

    // 投影矩阵（上一帧相机系为参考）
    Mat P0 = K_ * (Mat_<double>(3, 4) << 1, 0, 0, 0,
                   0, 1, 0, 0,
                   0, 0, 1, 0);
    Mat Rt1 = Mat::zeros(3, 4, CV_64F);
    R.copyTo(Rt1(Rect(0, 0, 3, 3)));
    t.copyTo(Rt1(Rect(3, 0, 1, 3)));
    Mat P1 = K_ * Rt1;

    Mat X4; // 4×N
    triangulatePoints(P0, P1, prevIn, currIn, X4);

    for (int i = 0; i < X4.cols; ++i) {
        double W = X4.at<double>(3, i);
        if (std::abs(W) < 1e-12) continue;
        double Xc = X4.at<double>(0, i) / W;
        double Yc = X4.at<double>(1, i) / W;
        double Zc = X4.at<double>(2, i) / W;

        // 正深度：在 prev / curr 都应 Z>0
        if (Zc <= 0) continue;
        Mat Xc1 = (Mat_<double>(3, 1) << Xc, Yc, Zc);
        Mat Xc1p = R * Xc1 + t;
        if (Xc1p.at<double>(2) <= 0) continue;

        // 变到世界：Xw = R_prev * Xc + t_prev
        Mat Xw = R_prev * Xc1 + t_prev;
        map_points_.emplace_back(Xw.at<double>(0), Xw.at<double>(1), Xw.at<double>(2));
    }
    last_inliers_ = currIn;
    // -----------------------------------

    // 累计位姿（右乘）
    R.convertTo(R, CV_64F);
    t.convertTo(t, CV_64F);
    R_f_.convertTo(R_f_, CV_64F);
    t_f_.convertTo(t_f_, CV_64F);

    double scale = getAbsoluteScale(numFrame);

    // 可选：若发现多数点负深度，翻转 (R,t)（简单版）
    int front = 0, back = 0;
    for (int i = 0; i < X4.cols; ++i) {
        double W = X4.at<double>(3, i);
        if (std::abs(W) < 1e-12) continue;
        double Z0 = X4.at<double>(2, i) / W;
        double Xc = X4.at<double>(0, i) / W;
        double Yc = X4.at<double>(1, i) / W;
        double Z1 = (R.at<double>(2, 0) * Xc +
                     R.at<double>(2, 1) * Yc +
                     R.at<double>(2, 2) * Z0 +
                     t.at<double>(2, 0));
        if (Z0 > 0 && Z1 > 0) front++; else back++;
    }
    if (back > front) {
        R = R.t();
        t = -R * t;
    }

    // 缩放并累计
    if (scale > 1e-3) {
        t_f_ = t_f_ + scale * (R_f_ * t);
        R_f_ = R * R_f_;
    }

    // 更新 Tcw_
    Mat Tk = Mat::eye(4, 4, CV_64F);
    R_f_.copyTo(Tk(Rect(0, 0, 3, 3)));
    t_f_.copyTo(Tk(Rect(3, 0, 1, 3)));
    Tcw_ = Tk.clone();

    // 轨迹画布（越界保护）
    auto clamp = [](int v, int lo, int hi){ return std::max(lo, std::min(hi, v)); };
    double s = 1.0;
    int x = clamp(int(s * t_f_.at<double>(0)) + 300, 0, trajCanvas_.cols - 1);
    int y = clamp(int(s * t_f_.at<double>(2)) + 100, 0, trajCanvas_.rows - 1);
    circle(trajCanvas_, Point(x, y), 1, CV_RGB(255, 0, 0), 2);

    // overlay（叠字）
    cvtColor(curr, out.overlay, COLOR_GRAY2BGR);
    char text[128];
    std::snprintf(text, sizeof(text), "x=%.4fm y=%.4fm z=%.4fm",
                  t_f_.at<double>(0), t_f_.at<double>(1), t_f_.at<double>(2));
    putText(out.overlay, text, Point(10, 30), FONT_HERSHEY_PLAIN, 1.2, Scalar(0, 255, 0), 2);

    // 输出
    out.ok = true;
    out.R = R_f_.clone();
    out.t = t_f_.clone();
    out.t_world = cv::Point3d(
        t_f_.at<double>(0),
        t_f_.at<double>(1),
        t_f_.at<double>(2));
    out.matchesVis = trajCanvas_.clone();

    // 滚动缓存（不足则重打点）
    prevImage_ = curr.clone();
    prevFeatures_ = currIn;
    if (prevFeatures_.size() < 100) {
        featureDetection(curr, prevFeatures_);
    }

    return out;
}

// ----------------- 导出 -----------------
bool VoEngine::writeTextFile(const std::string& path, const std::string& content)
{
    std::ofstream f(path, std::ios::out | std::ios::binary);
    if (!f.is_open()) return false;
    f.write(content.data(), (std::streamsize)content.size());
    return (bool)f;
}

bool VoEngine::exportMapPLY(const std::string& path) const
{
    // ASCII PLY：仅顶点
    std::ostringstream oss;
    oss << "ply\nformat ascii 1.0\n"
        << "element vertex " << map_points_.size() << "\n"
        << "property float x\nproperty float y\nproperty float z\n"
        << "end_header\n";
    for (const auto& p : map_points_) {
        oss << (float)p.x << " " << (float)p.y << " " << (float)p.z << "\n";
    }
    return writeTextFile(path, oss.str());
}

bool VoEngine::exportFeaturesCSV(const std::string& path) const
{
    std::ostringstream oss;
    oss << "u,v\n";
    for (const auto& pt : last_inliers_) {
        oss << pt.x << "," << pt.y << "\n";
    }
    return writeTextFile(path, oss.str());
}
