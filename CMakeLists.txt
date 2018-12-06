cmake_minimum_required(VERSION 3.0)
project(ZSerial)
set(CMAKE_CXX_STANDARD 11)
if(CMAKE_HOST_WIN32)
  set(IMP_FILE ZSerialWindowsImpl.cpp)
else(CMAKE_HOST_UNIX)
  set(IMP_FILE ZSerialUnixImpl.cpp)
endif()
add_library(${PROJECT_NAME}_shared SHARED ${IMP_FILE})
add_library(${PROJECT_NAME}_static STATIC ${IMP_FILE})
add_library(${PROJECT_NAME}_module MODULE ${IMP_FILE})
set_target_properties(${PROJECT_NAME}_shared
                      PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}_static
                      PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}_module
                      PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}_static PROPERTIES CLEAN_DIRECT_OUTPUT 1)
set_target_properties(${PROJECT_NAME}_shared PROPERTIES CLEAN_DIRECT_OUTPUT 1)
set_target_properties(${PROJECT_NAME}_module PROPERTIES CLEAN_DIRECT_OUTPUT 1)
if(CMAKE_HOST_APPLE)
  find_library(IOKIT_LIBRARY IOKit)
  find_library(COREFOUNDATION_LIBRARY CoreFoundation)
  target_link_libraries(${PROJECT_NAME}_shared ${IOKIT_LIBRARY}
                        ${COREFOUNDATION_LIBRARY})
  target_link_libraries(${PROJECT_NAME}_static ${IOKIT_LIBRARY}
                        ${COREFOUNDATION_LIBRARY})
  target_link_libraries(${PROJECT_NAME}_module ${IOKIT_LIBRARY}
                        ${COREFOUNDATION_LIBRARY})
endif(CMAKE_HOST_APPLE)
install(TARGETS ${PROJECT_NAME}_shared DESTINATION lib)
install(TARGETS ${PROJECT_NAME}_static DESTINATION lib)
install(TARGETS ${PROJECT_NAME}_module DESTINATION lib)
install(FILES ZSerial.h DESTINATION include)
