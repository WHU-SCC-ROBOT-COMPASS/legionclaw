source ../../../message/ros2/install/setup.bash
cp make/CMakeLists.txt.x64 CMakeLists.txt
rm build/* -rf
rm bin/lidar_cluster_detect
mkdir build
cd build
cmake -DROS_ENABLE=OFF -DROS2_ENABLE=ON -DLCM_ENABLE=OFF -DDDS_ENABLE=OFF -DGLOG_ENABLE=ON  -DCMAKE_BUILD_TYPE=Release ../
make -j$(nproc)
