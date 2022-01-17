cmake_minimum_required(VERSION 3.0.2)
project(slam)


set(CMAKE_BUILD_TYPE Release)
#SET(CMAKE_BUILD_TYPE "Debug")
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
#SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

set(CMAKE_CXX_STANDARD 14)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


set(catkin_DIR /opt/ros/melodic/share/catkin/cmake)

include_directories(
        ${PROJECT_SOURCE_DIR}/include/slam
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        /opt/ros/melodic/include
        /usr/include/suitesparse
)

find_package(
        catkin REQUIRED COMPONENTS
        cv_bridge
        geometry_msgs
        roscpp
        image_transport
        message_generation
        sensor_msgs
        std_msgs
        tf
        )


catkin_package(
        CATKIN_DEPENDS cv_bridge geometry_msgs image_transport  sensor_msgs std_msgs tf roscpp
)

set(SOURCE_FILES
        mylib/Frame.cpp
        mylib/MapPoint.cpp
        mylib/Map.cpp
        mylib/Camera.cpp
        mylib/Config.cpp
        mylib/Feature.cpp
        mylib/Front.cpp
        mylib/back.cpp
        mylib/viewer.cpp
        mylib/visual_odometry.cpp
        src/image_sub.cpp
        )


add_subdirectory(${PROJECT_SOURCE_DIR}/mylib)

link_directories(${PROJECT_SOURCE_DIR}/lib)

add_executable(image_listener ${SOURCE_FILES} )
target_link_libraries(image_listener myslamlib ${catkin_LIBRARIES}   ${THIRD_PARTY_LIBS} )




