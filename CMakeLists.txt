cmake_minimum_required(VERSION 3.0)
project(ZSerial)
set(CMAKE_CXX_STANDARD 11)
if(CMAKE_HOST_WIN32)
  set(IMP_FILE ZSerialWindowsImpl.cpp)
else(CMAKE_HOST_UNIX)
  set(IMP_FILE ZSerialUnixImpl.cpp)
endif()
add_library(${PROJECT_NAME} ${IMP_FILE})
# add_executable(${PROJECT_NAME}_exe ${IMP_FILE} test.cpp)
if(CMAKE_HOST_APPLE)
  find_library(IOKIT_LIBRARY IOKit)
  find_library(COREFOUNDATION_LIBRARY CoreFoundation)
  target_link_libraries(${PROJECT_NAME} ${IOKIT_LIBRARY}
                        ${COREFOUNDATION_LIBRARY})
endif(CMAKE_HOST_APPLE)
