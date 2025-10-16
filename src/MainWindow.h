#pragma once
#include <QMainWindow>
#include <QTimer>
#include <opencv2/opencv.hpp>
#include "TrajectoryView.h"
#include "VoModule.h"

class QLabel;
class QPushButton;
class QLineEdit;
class QCheckBox;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent=nullptr);

private slots:
    void chooseSequence();
    void toggleRun();
    void stepFrame();
    void exportCSV();

private:
    void setupUI();
    void log(const QString& s);
    bool loadFrame(int idx, cv::Mat& left, cv::Mat& right);
    void runSIFT(const cv::Mat& L, const cv::Mat& R);
    void runVO2D2D(const cv::Mat& prev, const cv::Mat& curr);

private:
    // --- UI 控件（与 MainWindow.cpp 完全一致的命名） ---
    QLabel* imgLeft_ = nullptr;
    QLabel* imgRight_ = nullptr;
    QLabel* status_ = nullptr;

    QLineEdit* editSeq_ = nullptr;
    QLineEdit* editFx_ = nullptr;
    QLineEdit* editFy_ = nullptr;
    QLineEdit* editCx_ = nullptr;
    QLineEdit* editCy_ = nullptr;
    QLineEdit* editBaseline_ = nullptr;
    QLineEdit* editMaxFrames_ = nullptr;
    QCheckBox* cbSavePng_ = nullptr;
    QPushButton* btnRun_ = nullptr;
    QPushButton* btnStep_ = nullptr;

    TrajectoryView* trajView_ = nullptr;
    QTimer timer_;

    // --- 运行状态 ---
    bool running_ = false;
    int  frame_ = 0;
    int  maxFrames_ = 500;
    cv::Mat prevLeft_;
    cv::Mat K_;
    double baseline_ = 0.537;
    std::vector<cv::Point3d> traj_;
    cv::Mat Tcw_ = cv::Mat::eye(4,4,CV_64F);

    // --- VO 引擎 ---
    VoEngine engine_;
};
