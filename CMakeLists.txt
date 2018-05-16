cmake_minimum_required(VERSION 3.0)
project(ZSerial)
set(CMAKE_CXX_STANDARD 11)
add_executable(${PROJECT_NAME} test.cpp ZSerialUnixImpl.cpp)