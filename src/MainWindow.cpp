#include "MainWindow.h"
#include "TrajectoryView.h"
#include <QStatusBar>
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
#include <QFormLayout>
#include <QTableWidget>
#include <QHeaderView>
#include <QSplitter>

using namespace cv;

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    setupUI();
    connect(&timer_, &QTimer::timeout, this, &MainWindow::stepFrame);
}

void MainWindow::setupUI()
{
    resize(1280, 760);
    auto central = new QWidget(this);
    setCentralWidget(central);

    // 左侧：两张图上下排列（用 splitter）
    imgLeft_ = new QLabel;
    imgLeft_->setMinimumHeight(280);
    imgLeft_->setScaledContents(true);
    imgRight_ = new QLabel;
    imgRight_->setMinimumHeight(280);
    imgRight_->setScaledContents(true);

    splitterLeft_ = new QSplitter(Qt::Vertical);
    auto leftTopBox = new QWidget;
    {
        auto vb = new QVBoxLayout(leftTopBox);
        vb->setContentsMargins(0, 0, 0, 0);
        vb->addWidget(new QLabel("左图 / 叠字"));
        vb->addWidget(imgLeft_);
    }
    auto leftBottomBox = new QWidget;
    {
        auto vb = new QVBoxLayout(leftBottomBox);
        vb->setContentsMargins(0, 0, 0, 0);
        vb->addWidget(new QLabel("右图 / 匹配"));
        vb->addWidget(imgRight_);
    }
    splitterLeft_->addWidget(leftTopBox);
    splitterLeft_->addWidget(leftBottomBox);
    splitterLeft_->setStretchFactor(0, 1);
    splitterLeft_->setStretchFactor(1, 1);

    // 右侧：参数组、控制组、轨迹视图、表格
    trajView_ = new TrajectoryView;
    trajView_->setMinimumHeight(240);

    // —— 参数组（中文标签）
    auto gbParams = new QGroupBox("参数设置");
    auto form = new QFormLayout;
    editSeq_ = new QLineEdit("/mnt/hgfs/vo_data/data_odometry_gray/dataset/sequences/00");
    auto btnChoose = new QPushButton("选择序列…");
    auto seqRow = new QWidget;
    {
        auto hb = new QHBoxLayout(seqRow);
        hb->setContentsMargins(0, 0, 0, 0);
        hb->addWidget(editSeq_, 1);
        hb->addWidget(btnChoose, 0);
    }
    form->addRow("序列目录", seqRow);

    editFx_ = new QLineEdit("718.856");
    editFy_ = new QLineEdit("718.856");
    editCx_ = new QLineEdit("607.1928");
    editCy_ = new QLineEdit("185.2157");
    editBaseline_ = new QLineEdit("0.537");
    editMaxFrames_ = new QLineEdit("500");

    form->addRow("fx", editFx_);
    form->addRow("fy", editFy_);
    form->addRow("cx", editCx_);
    form->addRow("cy", editCy_);
    form->addRow("基线 (m)", editBaseline_);
    form->addRow("最大帧数", editMaxFrames_);
    gbParams->setLayout(form);

    // —— 控制组
    auto gbCtrl = new QGroupBox("运行控制");
    cbSavePng_ = new QCheckBox("保存每帧 PNG");
    btnRun_ = new QPushButton("开始");
    btnStep_ = new QPushButton("单步");
    auto btnExport = new QPushButton("导出 CSV");

    auto ctrlLay = new QHBoxLayout;
    ctrlLay->addWidget(cbSavePng_);
    ctrlLay->addStretch();
    ctrlLay->addWidget(btnRun_);
    ctrlLay->addWidget(btnStep_);
    ctrlLay->addWidget(btnExport);
    gbCtrl->setLayout(ctrlLay);

    // —— 中文表格：帧 / X / Y / Z
    tableTraj_ = new QTableWidget(0, 4);
    QStringList headers;
    headers << "帧" << "X (m)" << "Y (m)" << "Z (m)";
    tableTraj_->setHorizontalHeaderLabels(headers);
    tableTraj_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    tableTraj_->verticalHeader()->setVisible(false);
    tableTraj_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    tableTraj_->setSelectionBehavior(QAbstractItemView::SelectRows);
    tableTraj_->setMinimumHeight(200);

    // 右侧总体布局（滚动不强求，这里用竖向堆叠）
    auto rightBox = new QVBoxLayout;
    rightBox->addWidget(gbParams);
    rightBox->addWidget(gbCtrl);
    auto gbTraj = new QGroupBox("轨迹");
    auto vbT = new QVBoxLayout(gbTraj);
    vbT->setContentsMargins(8, 8, 8, 8);
    vbT->addWidget(trajView_);
    rightBox->addWidget(gbTraj, 1);
    auto gbTable = new QGroupBox("轨迹数据（表）");
    auto vbTab = new QVBoxLayout(gbTable);
    vbTab->setContentsMargins(8, 8, 8, 8);
    vbTab->addWidget(tableTraj_);
    rightBox->addWidget(gbTable, 1);

    auto rightPane = new QWidget;
    rightPane->setLayout(rightBox);

    // 主分割器：左图 / 右控制
    splitterMain_ = new QSplitter(Qt::Horizontal);
    splitterMain_->addWidget(splitterLeft_);
    splitterMain_->addWidget(rightPane);
    splitterMain_->setStretchFactor(0, 3);
    splitterMain_->setStretchFactor(1, 2);

    auto mainLay = new QHBoxLayout;
    mainLay->addWidget(splitterMain_);
    central->setLayout(mainLay);

    // 信号连接
    connect(btnChoose, &QPushButton::clicked, this, &MainWindow::chooseSequence);
    connect(btnRun_, &QPushButton::clicked, this, &MainWindow::toggleRun);
    connect(btnStep_, &QPushButton::clicked, this, &MainWindow::stepFrame);
    connect(btnExport, &QPushButton::clicked, this, &MainWindow::exportCSV);

    // 状态栏
    status_ = new QLabel;
    statusBar()->addPermanentWidget(status_, 1);
}

void MainWindow::chooseSequence()
{
    auto d = QFileDialog::getExistingDirectory(this, "选择 KITTI 序列目录（…/sequences/XX）");
    if (!d.isEmpty())
        editSeq_->setText(d);
}

void MainWindow::toggleRun()
{
    haveOrigin_ = false;
    running_ = !running_;
    btnRun_->setText(running_ ? "暂停" : "开始");
    if (running_)
    {
        frame_ = 2; // 算法从第2帧开始
        traj_.clear();
        Tcw_ = cv::Mat::eye(4, 4, CV_64F);
        trajView_->clear();         // ← 清空右侧绿色轨迹
        tableTraj_->setRowCount(0); // 清空表格

        // K 从界面读
        K_ = (cv::Mat_<double>(3, 3) << editFx_->text().toDouble(), 0, editCx_->text().toDouble(),
              0, editFy_->text().toDouble(), editCy_->text().toDouble(),
              0, 0, 1);
        std::string base = editSeq_->text().toStdString();

        // ✅ Linux: 传根目录，VoEngine 会自己去 /dataset/poses/00.txt 等候选路径找
        std::string poses_root = "/mnt/hgfs/vo_data/data_odometry_poses";

        if (!engine_.init(base, K_, poses_root))
        {
            status_->setText("初始化失败：请检查序列路径或 poses 路径");
            running_ = false;
            btnRun_->setText("开始");
            return;
        }

        timer_.start(1);
    }
    else
    {
        timer_.stop();
    }
}

bool MainWindow::loadFrame(int idx, Mat &left, Mat &right)
{
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%06d.png", idx);
    std::string base = editSeq_->text().toStdString();
    left = imread((cv::String)(base + "/image_0/" + buf), IMREAD_GRAYSCALE);
    right = imread((cv::String)(base + "/image_1/" + buf), IMREAD_GRAYSCALE);
    return !left.empty() && !right.empty();
}

void MainWindow::stepFrame()
{
    maxFrames_ = editMaxFrames_->text().toInt();

    VoResult res = engine_.step(frame_);
    if (!res.ok)
    {
        timer_.stop();
        running_ = false;
        btnRun_->setText("开始");
        return;
    }

    // 更新两张图
    if (!res.overlay.empty())
        imgLeft_->setPixmap(QPixmap::fromImage(cvMatToQImageColor(res.overlay)));
    if (!res.matchesVis.empty())
        imgRight_->setPixmap(QPixmap::fromImage(cvMatToQImageColor(res.matchesVis)));

    // —— 以“本次 run 的起点”为 (0,0) —— //
    // 用 static 保存本次 run 的起点；当 frame_==2（你算法的第一帧）时重置
    static QPointF runOrigin(0.0, 0.0);
    if (frame_ == 2)
    {
        runOrigin = QPointF(res.t_world.x, res.t_world.z);
    }
    // 相对坐标（仅用于 UI 显示，不改动引擎内部）
    QPointF relXZ(res.t_world.x - runOrigin.x(),
                  res.t_world.z - runOrigin.y());

    // 轨迹/视图（绿色小窗画相对坐标）
    traj_.push_back(cv::Point3d(res.t_world.x, res.t_world.y, res.t_world.z));
    trajView_->appendPoint(relXZ);

    // 中文表格：如需显示“相对坐标”，就写 rel；想保留“绝对坐标”，就保持 res.t_world
    int row = tableTraj_->rowCount();
    tableTraj_->insertRow(row);
    tableTraj_->setItem(row, 0, new QTableWidgetItem(QString::number(frame_)));
    // 这里我给出两列版本：左边显示相对，右边仍显示绝对（你也可二选一）
    tableTraj_->setItem(row, 1, new QTableWidgetItem(QString::number(relXZ.x(), 'f', 6)));     // X(相对)
    tableTraj_->setItem(row, 2, new QTableWidgetItem(QString::number(res.t_world.y, 'f', 6))); // Y(绝对)
    tableTraj_->setItem(row, 3, new QTableWidgetItem(QString::number(relXZ.y(), 'f', 6)));     // Z(相对)

    // 状态栏
    status_->setText(QString("帧 %1 / %2   | 轨迹点数: %3")
                         .arg(frame_)
                         .arg(maxFrames_)
                         .arg(traj_.size()));

    // 帧推进与停止
    frame_++;
    if (frame_ >= maxFrames_)
    {
        timer_.stop();
        running_ = false;
        btnRun_->setText("开始");
    }
}

void MainWindow::runSIFT(const Mat &L, const Mat &R)
{
    static Ptr<Feature2D> sift = SiftModule::create();
    static BFMatcher matcher(NORM_L2, true);

    std::vector<KeyPoint> k1, k2;
    Mat d1, d2;
    sift->detectAndCompute(L, noArray(), k1, d1);
    sift->detectAndCompute(R, noArray(), k2, d2);
    std::vector<DMatch> ms;
    if (!d1.empty() && !d2.empty())
        matcher.match(d1, d2, ms);
    std::sort(ms.begin(), ms.end(), [](auto &a, auto &b)
              { return a.distance < b.distance; });
    if (ms.size() > 300)
        ms.resize(300);

    Mat vis;
    drawMatches(L, k1, R, k2, ms, vis);
    if (cbSavePng_->isChecked())
        imwrite("output/matches.png", vis);
    imgRight_->setPixmap(QPixmap::fromImage(cvMatToQImageColor(vis)));
}

void MainWindow::runVO2D2D(const Mat &prev, const Mat &curr)
{
    Ptr<ORB> orb = ORB::create(2000);
    std::vector<KeyPoint> kp1, kp2;
    Mat d1, d2;
    orb->detectAndCompute(prev, noArray(), kp1, d1);
    orb->detectAndCompute(curr, noArray(), kp2, d2);
    BFMatcher m(NORM_HAMMING, true);
    std::vector<DMatch> ms;
    if (!d1.empty() && !d2.empty())
        m.match(d1, d2, ms);
    if (ms.size() < 8)
        return;

    std::vector<Point2f> p1, p2;
    for (auto &mm : ms)
    {
        p1.push_back(kp1[mm.queryIdx].pt);
        p2.push_back(kp2[mm.trainIdx].pt);
    }

    Mat E = findEssentialMat(p2, p1, K_, RANSAC, 0.999, 1.0);
    Mat R, t;
    if (E.empty())
        return;
    recoverPose(E, p2, p1, K_, R, t);

    Mat Tk = Mat::eye(4, 4, CV_64F);
    R.copyTo(Tk(Rect(0, 0, 3, 3)));
    t.copyTo(Tk(Rect(3, 0, 1, 3)));
    Tcw_ = Tcw_ * Tk;

    Point3d tw(Tcw_.at<double>(0, 3), Tcw_.at<double>(1, 3), Tcw_.at<double>(2, 3));
    traj_.push_back(tw);
    trajView_->appendPoint(QPointF(tw.x, tw.z));

    if (cbSavePng_->isChecked())
    {
        cv::Mat disp;
        cv::cvtColor(curr, disp, cv::COLOR_GRAY2BGR);
        cv::putText(disp, cv::format("x=%.3f y=%.3f z=%.3f", tw.x, tw.y, tw.z),
                    {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 0}, 2);
        imwrite(cv::format("output/frame_%06d.png", frame_), disp);
    }
}

void MainWindow::exportCSV()
{
    QFileDialog fd(this);
    auto path = fd.getSaveFileName(this, "保存轨迹 CSV", "trajectory.csv", "CSV (*.csv)");
    if (path.isEmpty())
        return;
    FILE *f = fopen(path.toStdString().c_str(), "w");
    fprintf(f, "frame,x,y,z\n");
    for (size_t i = 0; i < traj_.size(); ++i)
        fprintf(f, "%zu,%.9f,%.9f,%.9f\n", i, traj_[i].x, traj_[i].y, traj_[i].z);
    fclose(f);
}
