cmake_minimum_required(VERSION 3.10)
project(VO_GUI)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)

find_package(Qt5 5.15 COMPONENTS Widgets REQUIRED)
find_package(OpenCV REQUIRED)

set(GUI_SRCS
    src/main.cpp
    src/MainWindow.cpp
    src/TrajectoryView.cpp
    src/Utils.cpp          # GUI 的 Utils（Mat->QImage）
    src/SiftModule.cpp
    src/VoModule.cpp
)

# 你的算法实现（只列 .cpp；不要把 sift_demo.cpp 放这里）
set(ALG_SRCS
    src/alg/sift.cpp
)

add_executable(vo_gui
    src/main.cpp
    src/MainWindow.cpp
    src/TrajectoryView.cpp
    src/Utils.cpp
    src/SiftModule.cpp
    src/VoModule.cpp
    ${ALG_SRCS}
)
target_include_directories(vo_gui PRIVATE src src/alg)
target_link_libraries(vo_gui Qt5::Widgets ${OpenCV_LIBS})
