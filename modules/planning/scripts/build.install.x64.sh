cp make/CMakeLists.txt.lib.install.x64 CMakeLists.txt
mkdir build
cd build
cmake -DROS_ENABLE=OFF -DROS2_ENABLE=OFF -DLCM_ENABLE=OFF -DDDS_ENABLE=OFF -DGLOG_ENABLE=ON  -DCMAKE_BUILD_TYPE=Release ../
make install -j$(nproc)