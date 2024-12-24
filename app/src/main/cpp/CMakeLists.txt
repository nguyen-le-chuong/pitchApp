cmake_minimum_required(VERSION 3.5)
project(native-lib)

set(CMAKE_CXX_STANDARD 14)

# include_directories("/home/eigen-doc") 
set(Eigen3_DIR "/home/Eigen3/share/eigen3/cmake")
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(SRC_DIR ${CMAKE_SOURCE_DIR})
set(SOURCES
    ${SRC_DIR}/native-lib.cpp
    ${SRC_DIR}/kalmanfilter.cpp
    ${SRC_DIR}/utils.cpp
    ${SRC_DIR}/sensors.cpp
    ${SRC_DIR}/simulation.cpp
    ${SRC_DIR}/readData.cpp
    ${SRC_DIR}/filterroad.cpp
    # ${SRC_DIR}/kalmanfilter.h
    # ${SRC_DIR}/utils.h
    # ${SRC_DIR}/sensors.h
    # ${SRC_DIR}/simulation.h
    # ${SRC_DIR}/readData.h
    # ${SRC_DIR}/filterroad.h
    # ${SRC_DIR}/roadslope.h
)

add_library(
    native-lib
    SHARED
    ${SOURCES}
)

find_library(
    log-lib
    log
)

find_library(
    android-lib
    android
)
target_link_libraries(${PROJECT_NAME}
        Eigen3::Eigen
        )
target_link_libraries(
    native-lib
    ${log-lib}
    ${android-lib}
)
