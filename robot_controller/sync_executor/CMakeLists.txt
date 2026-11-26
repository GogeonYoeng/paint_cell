cmake_minimum_required(VERSION 3.16)
project(sync_executor LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(API_DRFL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../doosan_a0912_controller/API-DRFL")
set(DOOSAN_DRCF_VERSION 2 CACHE STRING "Doosan DRCF major version (2: M2.x, 3: M3.x)")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(DRFL_ARCH "arm64")
else()
  set(DRFL_ARCH "amd64")
endif()
find_program(LSB_RELEASE_EXECUTABLE lsb_release)
if(LSB_RELEASE_EXECUTABLE)
  execute_process(COMMAND "${LSB_RELEASE_EXECUTABLE}" -rs OUTPUT_VARIABLE DRFL_UBUNTU_VERSION
                  OUTPUT_STRIP_TRAILING_WHITESPACE)
else()
  set(DRFL_UBUNTU_VERSION "22.04")
endif()
set(DRFL_LIB_DIR "${API_DRFL_DIR}/library/Linux/64bits/${DRFL_ARCH}/${DRFL_UBUNTU_VERSION}")

if(EXISTS "${DRFL_LIB_DIR}/libDRFL.a" AND EXISTS "${API_DRFL_DIR}/include/DRFLEx.h")
  set(_drfl_default ON)
else()
  set(_drfl_default OFF)
endif()
option(SYNC_WITH_DRFL "Build the real Doosan DRFL robot backend" ${_drfl_default})

add_executable(sync_executor src/main.cpp src/xgt_client.cpp src/plc_gantry.cpp)
target_include_directories(sync_executor PRIVATE include)
target_compile_options(sync_executor PRIVATE -Wall -Wextra)
target_link_libraries(sync_executor PRIVATE pthread)

if(SYNC_WITH_DRFL)
  target_sources(sync_executor PRIVATE src/drfl_robot_backend.cpp)
  target_compile_definitions(sync_executor PRIVATE SYNC_WITH_DRFL=1 DRCF_VERSION=${DOOSAN_DRCF_VERSION})
  target_include_directories(sync_executor PRIVATE "${API_DRFL_DIR}/include")
  target_link_libraries(sync_executor PRIVATE
    "${DRFL_LIB_DIR}/libDRFL.a"
    "${DRFL_LIB_DIR}/libPocoFoundation.so"
    "${DRFL_LIB_DIR}/libPocoNet.so")
  set_target_properties(sync_executor PROPERTIES BUILD_RPATH "${DRFL_LIB_DIR}" INSTALL_RPATH "${DRFL_LIB_DIR}")
  message(STATUS "sync_executor: DRFL backend ON (${DRFL_LIB_DIR})")
else()
  message(STATUS "sync_executor: DRFL backend OFF (mock only)")
endif()
