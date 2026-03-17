cp make/CMakeLists.txt.pallas CMakeLists.txt
mkdir build
cd build
cmake -DROS_ENABLE=OFF -DROS2_ENABLE=OFF -DLCM_ENABLE=OFF -DDDS_ENABLE=ON -DGLOG_ENABLE=ON  -DCMAKE_BUILD_TYPE=Release ../
rm -rf ../bin/control
make -j$(nproc)