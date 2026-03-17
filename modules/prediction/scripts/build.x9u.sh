cp make/CMakeLists.txt.x9u CMakeLists.txt
mkdir build
cd build
cmake -DROS_ENABLE=OFF -DROS2_ENABLE=OFF -DLCM_ENABLE=ON -DGLOG_ENABLE=ON -DDDS_ENABLE=OFF -DCMAKE_BUILD_TYPE=Release ../
make -j$(nproc)
