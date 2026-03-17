cp make/CMakeLists.txt.mdc CMakeLists.txt
cmake .
rm -rf ../bin/control
make -j$(nproc)