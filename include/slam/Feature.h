//
// Created by denghanjun on 2021/11/25.
//
#pragma once
#ifndef MYSLAM_FEATURE_H
#define MYSLAM_FEATURE_H

#include "common_include.h"



namespace myslam {

    class Frame;
    class MapPoint;

    class Feature {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Feature> Ptr;
        Feature(){};
        Feature(std::shared_ptr<Frame> Frame, cv::KeyPoint &Kp);
        std::weak_ptr<Frame> Frame_;
        std::weak_ptr<MapPoint> MapPoint_;
        bool is_on_left_image = true;
        bool is_outlier = false;
        cv::KeyPoint position_;

    };

}
#endif //MYSLAM_FEATURE_H
