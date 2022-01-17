//
// Created by denghanjun on 2021/11/25.
//
#pragma once
#ifndef MYSLAM_COMMON_INCLUDE_H
#define MYSLAM_COMMON_INCLUDE_H

#include <iostream>
#include <vector>
#include <thread>
#include <cmath>
#include <mutex>
#include <string>
#include <list>
#include <unordered_map>
#include <fstream>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <condition_variable>

#include <typeinfo>
#include <atomic>
// glog log(info)
#include <glog/logging.h>


#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/features2d.hpp>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

typedef Sophus::SO3d SO3d;
typedef Sophus::SE3d SE3d;
#endif //MYSLAM_COMMON_INCLUDE_H
