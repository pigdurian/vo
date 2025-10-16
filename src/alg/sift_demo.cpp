#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <filesystem>
#include <iostream>
#include <fstream>
#include "sift.h"

namespace fs = std::filesystem;
using std::cout;
using std::endl;

static void ensure_dir(const fs::path& p) {
    std::error_code ec;
    fs::create_directories(p, ec);
    if (ec) std::cerr << "[WARN] create_directories failed: " << p << " (" << ec.message() << ")\n";
}

static std::vector<cv::Point3f> dehomogenize(const cv::Mat& pts4d) {
    std::vector<cv::Point3f> xyz;
    xyz.reserve(pts4d.cols);
    for (int i = 0; i < pts4d.cols; ++i) {
        cv::Vec4f h = pts4d.col(i);
        float w = h[3];
        if (std::abs(w) < 1e-8f) continue;
        xyz.emplace_back(h[0]/w, h[1]/w, h[2]/w);
    }
    return xyz;
}

static void saveMatTxt(const fs::path& path, const cv::Mat& M) {
    std::ofstream ofs(path.string());
    ofs << std::setprecision(10);
    for (int r = 0; r < M.rows; ++r) {
        for (int c = 0; c < M.cols; ++c) {
            ofs << M.at<double>(r,c) << (c+1==M.cols?'\n':' ');
        }
    }
}

int main() {
    const fs::path img1_path = "/home/jinniewu/projects/work1/data/left0000.jpg";
    const fs::path img2_path = "/home/jinniewu/projects/work1/data/left0002.jpg";
    const fs::path out_dir   = "/home/jinniewu/projects/work1/output/sift";
    ensure_dir(out_dir);

    // 1) 读图
    cv::Mat img1 = cv::imread(img1_path.string(), cv::IMREAD_COLOR);
    cv::Mat img2 = cv::imread(img2_path.string(), cv::IMREAD_COLOR);
    if (img1.empty() || img2.empty()) {
        std::cerr << "[ERROR] read images failed.\n";
        return 1;
    }

    // 2) 相机内参与畸变（使用你提供的值）
    cv::Mat K = (cv::Mat_<double>(3,3) <<
        420.7175528733715, 0.0, 160.7831054526866,
        0.0, 422.9158796608776, 109.065248953414,
        0.0, 0.0, 1.0);

    cv::Mat D = (cv::Mat_<double>(1,5) <<
        -0.09576244783873929,
        -0.3267059096457082,
        0.0007692186442917947,
        0.0007849264957612677,
         2.420200551570148);

    // 3) 去畸变：用 Knew 作为新的（可能稍微变化的）内参
    cv::Mat Knew1 = cv::getOptimalNewCameraMatrix(K, D, img1.size(), 0.0); // alpha=0裁剪
    cv::Mat Knew2 = cv::getOptimalNewCameraMatrix(K, D, img2.size(), 0.0);
    // 若两图像来自同一相机、同分辨率，Knew1==Knew2，取其一
    cv::Mat Knew = Knew1;

    cv::Mat img1u, img2u;
    cv::undistort(img1, img1u, K, D, Knew);
    cv::undistort(img2, img2u, K, D, Knew);

    // 4) SIFT 特征（用你的 sift 实现）——在去畸变图上提取
    sift siftlab(3, 1.6);
    std::vector<keypoint> kpts1, kpts2;
    cv::Mat fvec1, fvec2;

    if (!siftlab.detect(img1u, kpts1, fvec1) || !siftlab.detect(img2u, kpts2, fvec2)) {
        std::cerr << "[ERROR] sift.detect failed.\n";
        return 1;
    }
    cout << "img1 keypoint: " << kpts1.size() << "\nimg2 keypoint: " << kpts2.size() << endl;

    std::vector<cv::KeyPoint> KPts1, KPts2;
    keypoint::convertToKeyPoint(kpts1, KPts1);
    keypoint::convertToKeyPoint(kpts2, KPts2);

    if (fvec1.type() != CV_32F) fvec1.convertTo(fvec1, CV_32F);
    if (fvec2.type() != CV_32F) fvec2.convertTo(fvec2, CV_32F);

    // 5) 匹配（对称一致性）
    cv::BFMatcher matcher(cv::NORM_L2, /*crossCheck=*/true);
    std::vector<cv::DMatch> matches;
    matcher.match(fvec1, fvec2, matches);
    std::sort(matches.begin(), matches.end(),
              [](const cv::DMatch& a, const cv::DMatch& b){ return a.distance < b.distance; });
    matches.resize(std::max<size_t>(1, matches.size()/2)); // 先留前50%，RANSAC再剔除

    // 6) 点坐标
    std::vector<cv::Point2f> pts1, pts2;
    pts1.reserve(matches.size());
    pts2.reserve(matches.size());
    for (auto &m : matches) {
        pts1.push_back(KPts1[m.queryIdx].pt);
        pts2.push_back(KPts2[m.trainIdx].pt);
    }

    // 7) 基础矩阵 F（八点法 + RANSAC）
    cv::Mat maskF;
    cv::Mat F = cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC,
                                       1.0 /* reprojThresh(px) */, 0.999 /* conf */, maskF);
    if (F.empty()) {
        std::cerr << "[ERROR] findFundamentalMat failed.\n";
        return 1;
    }
    F.convertTo(F, CV_64F);
    saveMatTxt(out_dir / "F.txt", F);

    // 保留 RANSAC(F) 内点，可视化匹配
    std::vector<cv::DMatch> inlier_matches;
    inlier_matches.reserve(matches.size());
    for (size_t i = 0; i < matches.size(); ++i) {
        if (maskF.at<uchar>(int(i))) inlier_matches.push_back(matches[i]);
    }
    cv::Mat img_matches;
    cv::drawMatches(img1u, KPts1, img2u, KPts2, inlier_matches, img_matches,
                    cv::Scalar::all(-1), cv::Scalar::all(-1),
                    std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    cv::imwrite((out_dir / "lenademo.jpg").string(), img_matches);

    // 8) 本质矩阵 E（五点法 + RANSAC）+ 恢复位姿
    // 使用去畸变后的点 + Knew
    std::vector<cv::Point2f> in1, in2;
    in1.reserve(inlier_matches.size());
    in2.reserve(inlier_matches.size());
    for (size_t i = 0; i < matches.size(); ++i) {
        if (!maskF.at<uchar>(int(i))) continue;
        in1.push_back(pts1[i]);
        in2.push_back(pts2[i]);
    }

    cv::Mat maskE;
    cv::Mat E = cv::findEssentialMat(in1, in2, Knew, cv::RANSAC, 0.999, 1.0, maskE);
    if (E.empty()) {
        std::cerr << "[WARN] findEssentialMat failed. You can still use F for projective reconstruction only.\n";
        return 0;
    }
    E.convertTo(E, CV_64F);
    saveMatTxt(out_dir / "E.txt", E);

    cv::Mat R, t;
    int inliers_pose = cv::recoverPose(E, in1, in2, Knew, R, t, maskE);
    cout << "recoverPose inliers: " << inliers_pose << endl;

    // 9) 三角化（仅用 E 内点）
    std::vector<cv::Point2f> tri1, tri2;
    tri1.reserve(in1.size()); tri2.reserve(in2.size());
    for (size_t i = 0; i < in1.size(); ++i) {
        if (maskE.at<uchar>(int(i))) {
            tri1.push_back(in1[i]);
            tri2.push_back(in2[i]);
        }
    }

    // P0 = Knew[I|0], P1 = Knew[R|t]
    cv::Mat P0 = Knew * (cv::Mat_<double>(3,4) <<
        1,0,0,0,
        0,1,0,0,
        0,0,1,0);
    cv::Mat P1 = Knew * (cv::Mat_<double>(3,4) <<
        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0),
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1),
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2));

    cv::Mat pts4d;
    cv::triangulatePoints(P0, P1, tri1, tri2, pts4d);
    auto xyz = dehomogenize(pts4d);

    std::ofstream ofs((out_dir / "points_xyz.txt").string());
    ofs << std::setprecision(8);
    for (auto& p : xyz) ofs << p.x << ' ' << p.y << ' ' << p.z << '\n';

    cout << "3D points saved: " << xyz.size() << " -> " << (out_dir / "points_xyz.txt") << endl;
    cout << "F saved to: " << (out_dir / "F.txt") << "\nE saved to: " << (out_dir / "E.txt") << endl;

    // 也把去畸变图另存（方便确认）
    cv::imwrite((out_dir / "undistort_left.jpg").string(),  img1u);
    cv::imwrite((out_dir / "undistort_right.jpg").string(), img2u);

    return 0;
}