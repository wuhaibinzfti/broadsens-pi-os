#!/bin/bash
cd libiio-0.15
cmake -DCMAKE_TOOLCHAIN_FILE=../toolchainfile.cmake
cd -
