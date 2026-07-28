# CMakeLists.txt
cmake_minimum_required(VERSION 3.20)
project(VN100Sample)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

set(PYBIND11_FINDPYTHON ON)
find_package(Python COMPONENTS Interpreter Development REQUIRED)

# Use the pybind11 that ships with the active interpreter. The system package
# (/usr/lib/cmake/pybind11, version 2.9) predates Python 3.11 making
# PyFrameObject opaque, so it cannot compile against these headers.
execute_process(
    COMMAND ${Python_EXECUTABLE} -m pybind11 --cmakedir
    OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE PYBIND11_CMAKE_DIR_RESULT
    ERROR_VARIABLE PYBIND11_CMAKE_DIR_ERROR)
if(NOT PYBIND11_CMAKE_DIR_RESULT EQUAL 0 OR NOT EXISTS "${PYBIND11_CMAKE_DIR}")
    message(FATAL_ERROR
        "Could not locate pybind11 for ${Python_EXECUTABLE}.\n"
        "Install it with: ${Python_EXECUTABLE} -m pip install pybind11\n"
        "(the system pybind11 2.9 is incompatible with Python ${Python_VERSION})\n"
        "${PYBIND11_CMAKE_DIR_ERROR}")
endif()
set(pybind11_DIR "${PYBIND11_CMAKE_DIR}" CACHE PATH "pybind11 cmake dir" FORCE)

find_package(pybind11 REQUIRED)

list(APPEND CMAKE_PREFIX_PATH "/usr/local")
find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)

# Include directories
include_directories(include)
# VectorNav library path
set(VECTORNAV_PATH "${PROJECT_SOURCE_DIR}/vectornav/cpp")

include_directories(${VECTORNAV_PATH}/include)

# List VectorNav sources explicitly
set(VN_SOURCES
    ${VECTORNAV_PATH}/src/attitude.cpp
    ${VECTORNAV_PATH}/src/compositedata.cpp
    ${VECTORNAV_PATH}/src/conversions.cpp
    ${VECTORNAV_PATH}/src/criticalsection.cpp
    ${VECTORNAV_PATH}/src/dllvalidator.cpp
    ${VECTORNAV_PATH}/src/error_detection.cpp
    ${VECTORNAV_PATH}/src/event.cpp
    ${VECTORNAV_PATH}/src/ezasyncdata.cpp
    ${VECTORNAV_PATH}/src/memoryport.cpp
    ${VECTORNAV_PATH}/src/packet.cpp
    ${VECTORNAV_PATH}/src/packetfinder.cpp
    ${VECTORNAV_PATH}/src/port.cpp
    ${VECTORNAV_PATH}/src/position.cpp
    ${VECTORNAV_PATH}/src/searcher.cpp
    ${VECTORNAV_PATH}/src/sensors.cpp
    ${VECTORNAV_PATH}/src/serialport.cpp
    ${VECTORNAV_PATH}/src/thread.cpp
    ${VECTORNAV_PATH}/src/types.cpp
    ${VECTORNAV_PATH}/src/util.cpp
    ${VECTORNAV_PATH}/src/utilities.cpp
    ${VECTORNAV_PATH}/src/vntime.cpp
)

# Add executable
add_executable(vn100_sample src/main.cpp src/vn100.cpp src/butterworth.cpp ${VN_SOURCES})
target_link_libraries(vn100_sample PRIVATE Eigen3::Eigen zenohcxx::zenohc)
if(UNIX)
    target_link_libraries(vn100_sample PRIVATE pthread)
endif()
target_compile_options(vn100_sample PRIVATE -Wall -O0 -Wno-class-memaccess -Wno-catch-value)

# Add Python bindings module
pybind11_add_module(vn100 MODULE src/bindings.cpp src/vn100.cpp src/butterworth.cpp ${VN_SOURCES})
target_link_libraries(vn100 PRIVATE Eigen3::Eigen zenohcxx::zenohc)
if(UNIX)
    target_link_libraries(vn100 PRIVATE pthread)
endif()
target_compile_options(vn100 PRIVATE -Wall -O0 -Wno-class-memaccess -Wno-catch-value)

install(TARGETS vn100 LIBRARY DESTINATION .)

add_custom_target(copy_config ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/config.cfg ${CMAKE_BINARY_DIR}/config.cfg
    COMMENT "Copying config.cfg to build directory"
)