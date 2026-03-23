#!/bin/bash
set -e
cp make/CMakeLists.txt.x64 CMakeLists.txt
mkdir -p build
cd build
cmake -DROS2_ENABLE=ON -DROS_ENABLE=OFF -DLCM_ENABLE=OFF -DDDS_ENABLE=OFF -DGLOG_ENABLE=ON -DOPENMP_ENABLE=ON -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
