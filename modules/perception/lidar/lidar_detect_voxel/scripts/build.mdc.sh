cp make/CMakeLists.txt.mdc CMakeLists.txt
cmake .
make -j$(nproc)