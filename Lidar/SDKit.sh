#!/bin/bash

cd $ROOT
git clone https://github.com/scanse/sweep-sdk.git
cd sweep-sdk/libsweep
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
sudo cmake --build . --target install
sudo ldconfig
cd ..
cd examples
sudo apt-get install libsfml-dev -y
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
