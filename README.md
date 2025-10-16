# 📦 VO_GUI：基于 Qt + OpenCV 的视觉里程计可视化程序

## 📘 项目简介

本项目是一个使用 **C++ / Qt / OpenCV** 开发的图形化视觉里程计（Visual Odometry, VO）演示程序，能够在 **KITTI 数据集** 上实现相机运动估计、轨迹绘制、特征点可视化和数据导出等功能。

✨ 程序的主要功能包括：

* ✅ 光流 + 本质矩阵法的两帧视觉里程计
* ✅ 实时轨迹绘制（2D 平面）与表格输出
* ✅ 特征点三角化与点云导出
* ✅ 位姿 CSV 导出
* ✅ 自动使用 KITTI ground truth 对齐绝对尺度
* ✅ 通过 `config.ini` 自动保存 / 加载参数（无需每次手动输入）

---

## 🛠️ 环境依赖

在运行本项目之前，请确保系统已安装以下依赖：

* Linux 系统（推荐 Ubuntu 20.04+）
* C++17 编译器（g++ ≥ 9.0）
* Qt 5 或 Qt 6
* OpenCV ≥ 4.5
* CMake ≥ 3.15

📦 Ubuntu 可快速安装：

```bash
sudo apt update
sudo apt install -y build-essential cmake qtbase5-dev libopencv-dev
```

---

## 📁 数据集结构要求

程序依赖 **KITTI Odometry** 数据集，请确保目录结构如下（以序列 `00` 为例）：

```
vo_data/
├─ data_odometry_gray/
│   └─ dataset/sequences/00/
│       ├─ image_0/000000.png
│       └─ image_1/000000.png
└─ data_odometry_poses/
    └─ dataset/poses/00.txt
```

📌 说明：

* `image_0/` 和 `image_1/`：左、右相机灰度图像序列
* `00.txt`：KITTI 官方提供的真值位姿文件（3×4 矩阵，每行 12 个数）

---

## ⚙️ 编译步骤

在项目根目录下运行：

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

编译完成后，生成的可执行文件位于：

```
build/vo_gui
```

---

## ▶️ 运行程序

✅ **推荐：在 Linux 终端中运行**

由于程序依赖动态库和相对路径，**必须从终端运行**：

```bash
cd build
./vo_gui
```

⚠️ 注意事项：

* **不要直接双击运行**，否则可能找不到依赖或数据路径。
* 首次运行时，在 GUI 中修改「序列目录」为你的数据集路径，例如：

```
/mnt/hgfs/vo_data/data_odometry_gray/dataset/sequences/00
```

* 同时确保「位姿目录」为：

```
/mnt/hgfs/vo_data/data_odometry_poses
```

程序会自动从该路径查找 `00.txt` 以进行绝对尺度对齐。

---

## ⚙️ 配置文件（推荐）

程序支持通过 `config.ini` 自动加载参数（首次运行后会自动生成）。
手动创建一个示例如下（放在可执行文件同级目录）：

```ini
[paths]
sequence=/mnt/hgfs/vo_data/data_odometry_gray/dataset/sequences/00
poses_root=/mnt/hgfs/vo_data/data_odometry_poses

[camera]
fx=718.856
fy=718.856
cx=607.1928
cy=185.2157
baseline=0.537

[run]
max_frames=500
save_png=false
```

> 下次启动时，程序会自动加载这些参数。

---

## 📊 功能说明

| 功能         | 描述                       |
| ---------- | ------------------------ |
| **开始**     | 从第 2 帧开始连续运行 VO 并绘制轨迹    |
| **单步**     | 手动处理下一帧                  |
| **保存 PNG** | 保存每帧的叠字图像                |
| **导出 CSV** | 导出估计轨迹数据（frame, x, y, z） |
| **轨迹表格**   | 显示每一帧的位姿坐标               |
| **绿色轨迹**   | UI 中绘制的轨迹（每次运行以起点为 0,0）  |
| **红色轨迹**   | 引擎内部的绝对轨迹（世界坐标系）         |

---

## 📤 导出功能

* **CSV 轨迹**：保存估计位姿坐标 `(frame, x, y, z)`
* **PLY 点云**：保存三角化点，可用 CloudCompare / MeshLab 打开
* **2D 特征 CSV**：保存当前帧的 inlier 特征点坐标 `(u, v)`

---

## ⚠️ 常见问题

| 问题           | 解决方案                                                 |
| ------------ | ---------------------------------------------------- |
| 程序提示“初始化失败”  | 检查「序列目录」和「位姿目录」是否正确                                  |
| 找不到 `00.txt` | 确认路径为 `.../data_odometry_poses/dataset/poses/00.txt` |
| 程序启动但无图像     | 检查图像路径及命名格式（应为 6 位数字，例如 000000.png）                  |
| 绿色轨迹和红色轨迹不重合 | 属于正常现象：绿色为本次 run 的相对坐标，红色为全局绝对坐标                     |
| 找不到/mnt/hgfs | 手动搭载共享文件夹，命令：
  sudo mkdir -p /mnt/hgfs 
  sudo vmhgfs-fuse .host:/ /mnt/hgfs -o allow_other -o uid=$(id -u),gid=$(id -g)   |

---

## ✅ 推荐运行流程（总结）

```bash
cd build
./vo_gui
```

然后在程序界面中：

1. 设置「序列目录」为：`.../sequences/00`
2. 设置「位姿目录」为：`.../data_odometry_poses`
3. 点击「开始」运行视觉里程计！

---
需在终端运行 `./vo_gui` 即可开始测试。

---