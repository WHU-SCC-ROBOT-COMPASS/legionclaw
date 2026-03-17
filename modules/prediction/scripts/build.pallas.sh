cp make/CMakeLists.txt.pallas CMakeLists.txt
mkdir build
cd build
cmake -DROS_ENABLE=OFF -DLCM_ENABLE=ON -DGLOG_ENABLE=ON -DDDS_ENABLE=OFF -DCMAKE_BUILD_TYPE=Release ../
make -j8
