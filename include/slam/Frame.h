//
// Created by denghanjun on 2021/11/25.
//
#pragma once
#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include "common_include.h"



namespace myslam{
    class Feature;
    class MapPoint;

    class Frame{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frame> Ptr;

        Frame(){};

        Frame(unsigned long id,SE3d pose_,cv::Mat leftimgae_,cv::Mat rightimage_);


        static std::shared_ptr<Frame> CreatNewFrame();



        inline SE3d Get_Pose()
        {
            std::unique_lock<std::mutex> lck(pose_mutex);
            return pose_;
        }

        inline void Set_Pose(const SE3d &pose)
        {
            std::unique_lock<std::mutex> lck(pose_mutex);
            pose_ = pose;

        }
        void Set_Id(unsigned long &id);

        void Set_KeyFrame();



        unsigned long frame_Id_;
        unsigned long keyframe_id_ ;  // id of key frame
        bool isKeyFrame_ = false;
        cv::Mat left_imgae_,right_image_;
        std::vector<std::shared_ptr<Feature>> letf_features_;
        std::vector<std::shared_ptr<Feature>> right_features_;
    private:
        std::mutex pose_mutex;
        SE3d pose_;



    };




}

#endif












