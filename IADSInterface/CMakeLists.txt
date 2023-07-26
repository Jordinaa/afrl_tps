cmake_minimum_required(VERSION 3.5)
project(IADSInterface)

set(CMAKE_CXX_STANDARD 14)

# Adding definitions
add_definitions(-DLINUX)

include_directories(.)
add_library(IADSInterface SHARED IADSInterface.cpp SocketBuffer.cpp Socket.cpp)
