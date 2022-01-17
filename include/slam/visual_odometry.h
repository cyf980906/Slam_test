//
// Created by denghanjun on 2021/11/30.
//
#pragma once
#ifndef MYSLAM_VIUSAL_ODOMETRY_H
#define MYSLAM_VIUSAL_ODOMETRY_H

#include "common_include.h"
#include "Front.h"
#include "viewer.h"
#include "Map.h"
#include "back.h"
namespace myslam {

    class visual_odometry {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr <visual_odometry> Ptr;

        visual_odometry(std::string &config_path);//set config file

        bool Init();

        bool Run(const cv::Mat &image_left, const cv::Mat &image_right);

        bool Step(const cv::Mat &image_left, const cv::Mat &image_right);

        Frame::Ptr NextFrame(const cv::Mat &image_left, const cv::Mat &image_right);

        FrontStatus GetFrontedStatus() { return fronted_->FrontStatus_; };

        void Stop() {
                 backend_->Stop();
                viewer_->Close();
                LOG(INFO) << "VO exit";
            };

            private:
            std::string config_file_path_;
            Front::Ptr fronted_ = nullptr;
            Back::Ptr backend_ = nullptr;
            Map::Ptr map_ = nullptr;
            Viewer::Ptr viewer_ = nullptr;


        };
    }

#endif //MYSLAM_VIUSAL_ODOMETRY_H
