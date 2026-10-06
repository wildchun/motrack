cmake_minimum_required(VERSION 3.14)
project(motrack)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

option(WITH_TEST "Build test"  OFF)
option(WITH_PYTHON "Build python bindings" OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -MMD -Wall -Wextra -Winit-self")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install)


add_library(${PROJECT_NAME} SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/src/Tracker.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/core/BaseTracker.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/core/Association.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/core/FeatureMetric.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/core/KalmanFilter.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/core/lapjv.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/core/STrack.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/algos/motion/bytetrack/ByteTrackTracker.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/algos/motion/sort/SortTracker.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/algos/motion/ocsort/OCSortTracker.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/algos/appearance/deepsort/DeepSortTracker.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/algos/appearance/jde/JDETracker.cpp
  )

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_LIST_DIR}/3rd/eigen-3.3.9
  )


if(WITH_TEST)
  add_executable(test_motrack ${CMAKE_CURRENT_SOURCE_DIR}/test/test_motrack.cpp)
  target_link_libraries(test_motrack ${PROJECT_NAME})

  add_executable(test_algos ${CMAKE_CURRENT_SOURCE_DIR}/test/test_algos.cpp)
  target_link_libraries(test_algos ${PROJECT_NAME})

  add_executable(stress_test ${CMAKE_CURRENT_SOURCE_DIR}/test/stress_test.cpp)
  target_link_libraries(stress_test ${PROJECT_NAME})
endif()

install(TARGETS ${PROJECT_NAME})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/Motrack.h
        DESTINATION include/)
