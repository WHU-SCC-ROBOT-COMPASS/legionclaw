cp make/CMakeLists.txt.x64 CMakeLists.txt
mkdir build
cd build
cmake -DROS_ENABLE=OFF -DROS2_ENABLE=ON -DLCM_ENABLE=OFF -DGLOG_ENABLE=ON -DDDS_ENABLE=OFF -DCMAKE_BUILD_TYPE=Debug ../
make -j$(nproc)
