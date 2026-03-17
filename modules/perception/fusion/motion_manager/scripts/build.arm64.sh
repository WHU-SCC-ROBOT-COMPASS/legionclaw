
cp make/CMakeLists.txt.arm64 CMakeLists.txt
export CC=/usr/bin/aarch64-linux-gnu-gcc
export CXX=/usr/bin/aarch64-linux-gnu-gcc
rm build/* -rf
mkdir build
cd build
cmake -DROS_ENABLE=OFF -DROS2_ENABLE=ON -DLCM_ENABLE=OFF -DDDS_ENABLE=OFF -DGLOG_ENABLE=ON  -DCMAKE_BUILD_TYPE=Release ../
make
