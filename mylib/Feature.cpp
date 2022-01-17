//
// Created by denghanjun on 2021/11/25.
//

#include "Feature.h"


namespace myslam {
    myslam::Feature::Feature(std::shared_ptr<myslam::Frame> Frame, cv::KeyPoint &Kp) : Frame_(Frame), position_(Kp) {};

}
