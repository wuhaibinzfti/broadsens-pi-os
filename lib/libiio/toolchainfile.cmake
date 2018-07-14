# this one is important
SET(CMAKE_SYSTEM_NAME Linux)
 
#this one not so much
SET(CMAKE_SYSTEM_VERSION 1)
 
# specify the cross compiler
SET(CMAKE_C_COMPILER $ENV{PWD}/../../../tools/toolchain/arm-bcm2708/arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc)
SET(CMAKE_CXX_COMPILER $ENV{PWD}/../../../tools/toolchain/arm-bcm2708/arm-linux-gnueabihf/bin/arm-linux-gnueabihf-g++)
 
# where is the target environment
SET(CMAKE_FIND_ROOT_PATH $ENV{PWD}/../../output/libxml2 $ENV{PWD}/../../../tools/toolchain/arm-bcm2708/arm-linux-gnueabihf)
 
# search for programs in the build host directories
#SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ONLY)
 
# for libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)

SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

