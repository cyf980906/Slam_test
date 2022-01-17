cmake_minimum_required(VERSION 3.0.2)
project(mylib)

include_directories(${PROJECT_SOURCE_DIR}/../include/slam)
list(APPEND CMAKE_MODULE_PATH  ${PROJECT_SOURCE_DIR}/../cmake_modules)
# Eigen
include_directories("/usr/include/eigen3")

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# gtest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})


find_package(catkin)
include_directories(${GTEST_INCLUDE_DIRS})

# gflags
find_package(GFlags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})


set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        Sophus::Sophus
        ${Pangolin_LIBRARIES} GL GLU GLEW glut
        g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
        ${GLOG_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        pthread
        ${CSPARSE_LIBRARY}
        ${GTEST_BOTH_LIBRARIES}
        )


#link_libraries(THIRD_PARTY_LIBS)


add_library(myslamlib STATIC
        Frame.cpp
        MapPoint.cpp
        Map.cpp
        Camera.cpp
        Config.cpp
        Feature.cpp
        Front.cpp
        back.cpp
        viewer.cpp
        visual_odometry.cpp
        )

target_link_libraries(myslamlib  ${THIRD_PARTY_LIBS})
target_include_directories(myslamlib PUBLIC  ${PROJECT_SOURCE_DIR}/../include/slam)