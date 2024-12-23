
cmake_minimum_required(VERSION 3.10.2)

project(native-lib)

set(SRC_DIR src/main/cpp)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

set(SOURCES
    ${SRC_DIR}/native-lib.cpp
    ${SRC_DIR}/kalmanfilter.cpp
    ${SRC_DIR}/utils.cpp
    ${SRC_DIR}/sensors.cpp
    ${SRC_DIR}/simulation.cpp
    ${SRC_DIR}/readData.cpp
    ${SRC_DIR}/filterroad.cpp
    ${SRC_DIR}/kalmanfilter.h
    ${SRC_DIR}/utils.h
    ${SRC_DIR}/sensors.h
    ${SRC_DIR}/simulation.h
    ${SRC_DIR}/readData.h
    ${SRC_DIR}/filterroad.h
    ${SRC_DIR}/roadslope.h
    ${SRC_DIR}/roadslope.cpp
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
