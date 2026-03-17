cp make/CMakeLists.txt.arm64 CMakeLists.txt
mkdir build
cd build
cmake -DROS_ENABLE=OFF -DROS2_ENABLE=OFF -DLCM_ENABLE=ON -DDDS_ENABLE=OFF -DGLOG_ENABLE=ON  -DCMAKE_BUILD_TYPE=Release ../
make -j$(nproc)