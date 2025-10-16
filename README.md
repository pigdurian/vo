📦 VO_GUI：基于 Qt + OpenCV 的视觉里程计可视化程序
📘 项目简介

本项目是一个使用 C++ / Qt / OpenCV 开发的图形化视觉里程计（Visual Odometry, VO）演示程序，能够在 KITTI 数据集上进行相机运动估计、轨迹绘制、特征点可视化和数据导出等操作。

程序的主要功能包括：

✅ 光流 + 本质矩阵法的两帧视觉里程计

✅ 实时轨迹绘制（2D 平面）与表格输出

✅ 特征点三角化与点云导出

✅ 位姿 CSV 导出

✅ 自动使用 KITTI ground truth 对齐绝对尺度

🛠️ 环境依赖

在运行本项目之前，请确保系统已安装以下依赖：

Linux 系统（推荐 Ubuntu 20.04+）

C++17 编译器（g++ ≥ 9.0）

Qt 5/6

OpenCV ≥ 4.5

CMake ≥ 3.15

可使用以下命令快速安装常用依赖（Ubuntu 示例）：

sudo apt update
sudo apt install -y build-essential cmake qtbase5-dev libopencv-dev

📁 数据集结构要求

程序依赖 KITTI Odometry 数据集
，请确保数据目录结构如下（以序列 00 为例）：

vo_data/
├─ data_odometry_gray/
│   └─ dataset/sequences/00/
│       ├─ image_0/000000.png
│       └─ image_1/000000.png
└─ data_odometry_poses/
    └─ dataset/poses/00.txt


📌 说明：

image_0/ 和 image_1/ 中分别是左、右相机灰度图像。

00.txt 是 KITTI 官方提供的真值位姿文件（3×4 矩阵，每行 12 个数）。

⚙️ 编译步骤

在项目根目录下执行以下命令完成构建：

mkdir -p build && cd build
cmake ..
make -j$(nproc)


编译完成后，会生成可执行文件：

build/vo_gui

▶️ 运行程序
✅ 终端启动（推荐）

由于程序依赖动态库和运行路径，请务必通过 Linux 终端执行：

cd build
./vo_gui


⚠️ 注意事项：

程序必须从终端运行，否则可能无法正确找到动态库或数据路径。

首次运行时，请在 GUI 界面中修改「序列目录」为你的数据集路径，例如：

/mnt/hgfs/vo_data/data_odometry_gray/dataset/sequences/00


poses_root 默认为：

/mnt/hgfs/vo_data/data_odometry_poses


程序会自动从该路径查找 00.txt 以进行绝对尺度对齐。

📊 功能说明
功能	描述
开始	从第 2 帧开始连续运行 VO 并绘制轨迹
单步	手动处理下一帧
保存 PNG	保存每帧的叠字图像
导出 CSV	导出估计轨迹数据（frame, x, y, z）
轨迹表格	显示每一帧的位姿坐标
绿色轨迹	UI 中绘制的轨迹（每次运行从 0,0 开始）
红色轨迹	引擎内部的绝对轨迹（世界坐标系）
📤 导出功能

CSV 轨迹：保存估计位姿坐标 (frame, x, y, z)

PLY 点云：保存三角化点，可在 CloudCompare / MeshLab 等软件中打开

2D 特征 CSV：保存当前帧的 inlier 特征点坐标 (u, v)

⚠️ 常见问题
问题	解决方案
程序提示“初始化失败”	检查序列路径和 poses_root 路径是否正确
找不到 00.txt	确认路径为 .../data_odometry_poses/dataset/poses/00.txt
程序启动但无图像	检查图像路径是否正确，以及文件名是否为 6 位数（如 000000.png）
绿色轨迹和红色轨迹不重合	属于正常现象，绿色为本次 run 相对坐标，红色为全局绝对坐标
✅ 推荐运行方式（总结）
cd build
./vo_gui


然后在程序界面中：

设置序列目录为 .../sequences/00

设置位姿文件目录为 .../data_odometry_poses

点击「开始」运行即可
