编译命令
kdir -p build && cd build
cmake ..
make -j$(nproc)
./vo_gui

