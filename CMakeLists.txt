cmake_minimum_required(VERSION 3.14)
project(motrack)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

option(WITH_TEST "Build test"  ON)
option(WITH_PYTHON "Build python bindings" OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -MMD -Wall -Wextra -Winit-self")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install)

set(EIGEN3_ZIP "${CMAKE_CURRENT_SOURCE_DIR}/3rd/eigen-3.3.9.zip")
execute_process(
    COMMAND ${CMAKE_COMMAND} -E tar xzf ${EIGEN3_ZIP}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    RESULT_VARIABLE EIGEN3_UNPACK_RESULT
    OUTPUT_VARIABLE EIGEN3_UNPACK_OUTPUT
    ERROR_VARIABLE EIGEN3_UNPACK_ERROR
)
if(NOT EIGEN3_UNPACK_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to unpack Eigen3:
        Output: ${EIGEN3_UNPACK_OUTPUT}
        Error: ${EIGEN3_UNPACK_ERROR}")
endif()

if(WITH_PYTHON)
  set(PYBIND11_ZIP "${CMAKE_CURRENT_SOURCE_DIR}/3rd/pybind11-2.10.4.zip")
  execute_process(
      COMMAND ${CMAKE_COMMAND} -E tar xzf ${PYBIND11_ZIP}
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      RESULT_VARIABLE EPYBIND11_UNPACK_RESULT
      OUTPUT_VARIABLE PYBIND11_UNPACK_OUTPUT
      ERROR_VARIABLE PYBIND11_UNPACK_ERROR
  )
endif()

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
  ${CMAKE_CURRENT_BINARY_DIR}/eigen-3.3.9
  )

if(WITH_PYTHON)
  set(PYTHON_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/python/pymotrack.cpp)
  add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/pybind11-2.10.4)
  pybind11_add_module(py${PROJECT_NAME} ${PYTHON_SRC})
  target_link_libraries(py${PROJECT_NAME} PRIVATE ${PROJECT_NAME})

  # Wheel packaging: stage the built .so into build/wheel/pymotrack/, drop a
  # setup.py alongside it, and let `python setup.py bdist_wheel` build a
  # platform-tagged wheel under build/dist/. The wheel is a thin re-export
  # so `import pymotrack` still resolves to Tracker/Rect/Object/Track.
  set(MOTRACK_WHEEL_VERSION "1.0.0" CACHE STRING "pymotrack wheel version")
  set(WHEEL_DIR   ${CMAKE_BINARY_DIR}/wheel)
  set(WHEEL_PKG   ${WHEEL_DIR}/pymotrack)
  set(DIST_DIR    ${CMAKE_BINARY_DIR}/dist)
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/python/setup.py.in
    ${WHEEL_DIR}/setup.py
    @ONLY)

  add_custom_command(TARGET py${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${WHEEL_PKG}
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_SOURCE_DIR}/src/python/pymotrack_init.py
            ${WHEEL_PKG}/__init__.py
    COMMAND ${CMAKE_COMMAND} -E copy
            $<TARGET_FILE:py${PROJECT_NAME}>
            ${WHEEL_PKG}/$<TARGET_FILE_NAME:py${PROJECT_NAME}>
    COMMENT "Staging pymotrack wheel layout in ${WHEEL_DIR}")

  find_package(PythonInterp)
  if(PYTHONINTERP_FOUND)
    add_custom_target(py${PROJECT_NAME}_wheel ALL
      COMMAND ${CMAKE_COMMAND} -E remove_directory ${WHEEL_DIR}/build
      COMMAND ${PYTHON_EXECUTABLE} setup.py bdist_wheel --dist-dir ${DIST_DIR}
      WORKING_DIRECTORY ${WHEEL_DIR}
      DEPENDS py${PROJECT_NAME}
      COMMENT "Building pybytetrack wheel into ${DIST_DIR}")
  else()
    message(WARNING "WITH_PYTHON=ON but no python interpreter found; "
                    "skipping wheel build (the .so still builds in ${CMAKE_BINARY_DIR}).")
  endif()
endif()

if(WITH_TEST)
  add_executable(test_motrack ${CMAKE_CURRENT_SOURCE_DIR}/test/test_motrack.cpp)
  target_link_libraries(test_motrack ${PROJECT_NAME})

  add_executable(stress_test ${CMAKE_CURRENT_SOURCE_DIR}/test/stress_test.cpp)
  target_link_libraries(stress_test ${PROJECT_NAME})
endif()

install(TARGETS ${PROJECT_NAME})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/Motrack.h
        DESTINATION include/)
