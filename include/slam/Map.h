//
// Created by denghanjun on 2021/11/30.
//
#pragma once
#ifndef MYSLAM_MAP_H
#define MYSLAM_MAP_H

#include "common_include.h"
#include "Frame.h"
#include "MapPoint.h"
namespace myslam {
    class Map {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::unordered_map<unsigned long, Frame::Ptr> KeyFrameType;
        typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
        typedef std::shared_ptr<Map> Ptr;

        Map(){};

        void InsertNewKeyFrame(Frame::Ptr frame);

        void InsertNewMapPoint(MapPoint::Ptr mapppoint);

        void RemoveKeyFrame();

        void CleanMap();

        KeyFrameType Get_AllKeyFrames()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return KeyFrames_;
        }

        KeyFrameType Get_ActiveKeyFrames()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return ActiveKeyFrames_;
        }

        LandmarksType Get_MapPoints()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return MapPoints_;
        }
        LandmarksType Get_ActiveMapPoints()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return ActiveMapPoints_;
        }


    private:
        std::mutex data_mutex_;
        KeyFrameType KeyFrames_;
        KeyFrameType ActiveKeyFrames_;
        LandmarksType MapPoints_;
        LandmarksType ActiveMapPoints_;
        Frame::Ptr CurrentFrame_ = nullptr;
        unsigned int num_active_keyframes_ = 7;  // 激活的关键帧数量


    };
}

#endif //MYSLAM_MAP_H
