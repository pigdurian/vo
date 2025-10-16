#include "MainWindow.h"
#include "TrajectoryView.h"   // 需要完整类型
#include <QStatusBar>         // 使用 statusBar() 也需要这个头
#include "Utils.h"
#include "SiftModule.h"
#include "VoModule.h"
#include <QFileDialog>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QCheckBox>
#include <QPlainTextEdit>
#include <QGroupBox>

using namespace cv;

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setupUI();
    connect(&timer_, &QTimer::timeout, this, &MainWindow::stepFrame);
}

void MainWindow::setupUI() {
    resize(1200, 720);
    auto central = new QWidget(this);
    setCentralWidget(central);

    imgLeft_ = new QLabel;  imgLeft_->setMinimumSize(600, 300);  imgLeft_->setScaledContents(true);
    imgRight_= new QLabel;  imgRight_->setMinimumSize(600, 300);  imgRight_->setScaledContents(true);
    trajView_= new TrajectoryView; trajView_->setMinimumSize(400, 300);

    auto btnChoose = new QPushButton("Open Sequence");
    btnRun_  = new QPushButton("Start");
    btnStep_ = new QPushButton("Step");
    auto btnExport = new QPushButton("Export CSV");
    cbSavePng_ = new QCheckBox("Save PNG");

    editSeq_ = new QLineEdit("/mnt/hgfs/vo_data/data_odometry_gray/dataset/sequences/00");
    editFx_  = new QLineEdit("718.856");
    editFy_  = new QLineEdit("718.856");
    editCx_  = new QLineEdit("607.1928");
    editCy_  = new QLineEdit("185.2157");
    editBaseline_   = new QLineEdit("0.537");
    editMaxFrames_  = new QLineEdit("500");

    auto grid = new QGridLayout;
    int r=0;
    grid->addWidget(new QLabel("Sequence"), r,0); grid->addWidget(editSeq_, r,1,1,3); grid->addWidget(btnChoose, r++,4);
    grid->addWidget(new QLabel("fx"), r,0); grid->addWidget(editFx_, r,1);
    grid->addWidget(new QLabel("fy"), r,2); grid->addWidget(editFy_, r,3);
    grid->addWidget(new QLabel("cx"), ++r,0); grid->addWidget(editCx_, r,1);
    grid->addWidget(new QLabel("cy"), r,2); grid->addWidget(editCy_, r,3);
    grid->addWidget(new QLabel("baseline"), ++r,0); grid->addWidget(editBaseline_, r,1);
    grid->addWidget(new QLabel("max frames"), r,2); grid->addWidget(editMaxFrames_, r,3);
    grid->addWidget(cbSavePng_, ++r,0);
    grid->addWidget(btnRun_, r,2); grid->addWidget(btnStep_, r,3);
    grid->addWidget(btnExport, ++r,2,1,2);

    auto leftBox = new QVBoxLayout;
    leftBox->addWidget(new QLabel("Left / Overlay"));
    leftBox->addWidget(imgLeft_);
    leftBox->addWidget(new QLabel("Right / Matches"));
    leftBox->addWidget(imgRight_);

    auto rightBox = new QVBoxLayout;
    rightBox->addLayout(grid);
    rightBox->addWidget(new QLabel("Trajectory"));
    rightBox->addWidget(trajView_);

    auto mainLay = new QHBoxLayout;
    mainLay->addLayout(leftBox, 2);
    mainLay->addLayout(rightBox, 2);
    central->setLayout(mainLay);

    connect(btnChoose, &QPushButton::clicked, this, &MainWindow::chooseSequence);
    connect(btnRun_,   &QPushButton::clicked, this, &MainWindow::toggleRun);
    connect(btnStep_,  &QPushButton::clicked, this, &MainWindow::stepFrame);
    connect(btnExport, &QPushButton::clicked, this, &MainWindow::exportCSV);

    status_ = new QLabel; statusBar()->addPermanentWidget(status_, 1);
}

void MainWindow::chooseSequence() {
    auto d = QFileDialog::getExistingDirectory(this, "Select KITTI sequence (…/sequences/XX)");
    if (!d.isEmpty()) editSeq_->setText(d);
}

void MainWindow::toggleRun() {
    running_ = !running_;
    btnRun_->setText(running_ ? "Pause" : "Start");
    if (running_) {
        frame_ = 2;  // 你的算法从第2帧开始循环
        traj_.clear(); Tcw_ = cv::Mat::eye(4,4,CV_64F);

        // K 从界面读
        K_ = (cv::Mat_<double>(3,3) << editFx_->text().toDouble(), 0, editCx_->text().toDouble(),
                                       0, editFy_->text().toDouble(), editCy_->text().toDouble(),
                                       0, 0, 1);
        std::string base = editSeq_->text().toStdString();

        // 改成你数据集 poses 的真实目录：
        std::string poses_root = "/mnt/hgfs/vo_data/data_odometry_poses/dataset/poses";

        if (!engine_.init(base, K_, poses_root)) {
            status_->setText("Init failed (check sequence path)");
            running_ = false; btnRun_->setText("Start"); return;
        }
        timer_.start(1);
    } else {
        timer_.stop();
    }
}


bool MainWindow::loadFrame(int idx, Mat& left, Mat& right) {
    char buf[16]; std::snprintf(buf, sizeof(buf), "%06d.png", idx);
    std::string base = editSeq_->text().toStdString();
    left  = imread((cv::String)(base + "/image_0/" + buf), IMREAD_GRAYSCALE);
    right = imread((cv::String)(base + "/image_1/" + buf), IMREAD_GRAYSCALE);
    return !left.empty() && !right.empty();
}

void MainWindow::stepFrame() {
    maxFrames_ = editMaxFrames_->text().toInt();

    VoResult res = engine_.step(frame_);
    if (!res.ok) { timer_.stop(); running_=false; btnRun_->setText("Start"); return; }

    if (!res.overlay.empty())
        imgLeft_->setPixmap(QPixmap::fromImage(cvMatToQImageColor(res.overlay)));
    if (!res.matchesVis.empty())
        imgRight_->setPixmap(QPixmap::fromImage(cvMatToQImageColor(res.matchesVis)));

    traj_.push_back(res.t_world);
    trajView_->appendPoint(QPointF(res.t_world.x, res.t_world.z));

    status_->setText(QString("Frame %1 / %2   | Traj size: %3")
                     .arg(frame_).arg(maxFrames_).arg(traj_.size()));

    frame_++;
    if (frame_ >= maxFrames_) { timer_.stop(); running_=false; btnRun_->setText("Start"); }
}


void MainWindow::runSIFT(const Mat& L, const Mat& R) {
    static Ptr<Feature2D> sift = SiftModule::create();  // 你可改为 ORB
    static BFMatcher matcher(NORM_L2, true);

    std::vector<KeyPoint> k1, k2; Mat d1, d2;
    sift->detectAndCompute(L, noArray(), k1, d1);
    sift->detectAndCompute(R, noArray(), k2, d2);
    std::vector<DMatch> ms; if(!d1.empty() && !d2.empty()) matcher.match(d1,d2,ms);
    std::sort(ms.begin(), ms.end(), [](auto& a, auto& b){return a.distance<b.distance;});
    if (ms.size()>300) ms.resize(300);

    Mat vis; drawMatches(L, k1, R, k2, ms, vis);
    if (cbSavePng_->isChecked()) imwrite("output/matches.png", vis);
    imgRight_->setPixmap(QPixmap::fromImage(cvMatToQImageColor(vis)));
}

void MainWindow::runVO2D2D(const Mat& prev, const Mat& curr) {
    // 轻量演示：K + E + recoverPose，累加位姿（尺度未定，仅用于可视化）
    Ptr<ORB> orb = ORB::create(2000);
    std::vector<KeyPoint> kp1,kp2; Mat d1,d2;
    orb->detectAndCompute(prev, noArray(), kp1, d1);
    orb->detectAndCompute(curr, noArray(), kp2, d2);
    BFMatcher m(NORM_HAMMING, true);
    std::vector<DMatch> ms; if(!d1.empty() && !d2.empty()) m.match(d1,d2,ms);
    if (ms.size()<8) return;

    std::vector<Point2f> p1, p2;
    for (auto& mm : ms) { p1.push_back(kp1[mm.queryIdx].pt); p2.push_back(kp2[mm.trainIdx].pt); }

    Mat E = findEssentialMat(p2, p1, K_, RANSAC, 0.999, 1.0);
    Mat R, t; if (E.empty()) return;
    recoverPose(E, p2, p1, K_, R, t);

    Mat Tk = Mat::eye(4,4,CV_64F);
    R.copyTo(Tk(Rect(0,0,3,3))); t.copyTo(Tk(Rect(3,0,1,3)));
    Tcw_ = Tcw_ * Tk;  // 简单连乘（2D-2D 无真实尺度）

    Point3d tw(Tcw_.at<double>(0,3), Tcw_.at<double>(1,3), Tcw_.at<double>(2,3));
    traj_.push_back(tw);
    trajView_->appendPoint(QPointF(tw.x, tw.z)); // x-z 平面画轨迹

    if (cbSavePng_->isChecked()) {
        cv::Mat disp; cv::cvtColor(curr, disp, cv::COLOR_GRAY2BGR);
        cv::putText(disp, cv::format("x=%.3f y=%.3f z=%.3f", tw.x, tw.y, tw.z),
                    {10,30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0,255,0}, 2);
        imwrite(cv::format("output/frame_%06d.png", frame_), disp);
    }
}

void MainWindow::exportCSV() {
    QFileDialog fd(this);
    auto path = fd.getSaveFileName(this, "Save trajectory.csv", "trajectory.csv", "CSV (*.csv)");
    if (path.isEmpty()) return;
    FILE* f = fopen(path.toStdString().c_str(), "w");
    fprintf(f, "frame,x,y,z\n");
    for (size_t i=0;i<traj_.size();++i)
        fprintf(f, "%zu,%.9f,%.9f,%.9f\n", i, traj_[i].x, traj_[i].y, traj_[i].z);
    fclose(f);
}
