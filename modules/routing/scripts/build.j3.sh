#!/bin/bash
SCRIPT_PATH=`realpath $0`
SCRIPT_DIR=`dirname $SCRIPT_PATH`
#PLATFORM=ubuntu
PLATFORM=aarch64-journey
PROJECT_DIR=$SCRIPT_DIR/..
CMAKE_DIR=$PROJECT_DIR/cmake
BUILD_DIR=$PROJECT_DIR/build/${PLATFORM}
rm -rf $BUILD_DIR/*
mkdir -p $BUILD_DIR
cd $BUILD_DIR
cp $CMAKE_DIR/build.properties.local $BUILD_DIR/build.properties.local
cp $PROJECT_DIR/make/CMakeLists.txt.j3 $PROJECT_DIR/CMakeLists.txt;
cmake -DROS_ENABLE=OFF -DGLOG_ENABLE=ON -DDDS_ENABLE=ON -DLCM_ENABLE=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=$CMAKE_DIR/aarch64-journey.toolchain.cmake -DCMAKE_INSTALL_PREFIX:PATH=$PROJECT_DIR/output/ $PROJECT_DIR
#cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX:PATH=$BUILD_DIR/output/ $PROJECT_DIR 
cmake --build . -j$(nproc)
#make install
