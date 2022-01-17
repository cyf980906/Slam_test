//
// Created by denghanjun on 2021/12/1.
//
#pragma once
#ifndef MYSLAM_BACK_H
#define MYSLAM_BACK_H
#include "common_include.h"
#include "Camera.h"
#include "Map.h"

namespace myslam {

    class Back {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Back> Ptr;

        void BackendLoop();

        Back();

        void Stop();

        void UpdateMap();

        void Optimize(Map::KeyFrameType &active_KFs,Map::LandmarksType &active_Mappoints);

        void  Set_Map(Map::Ptr map ){map_=map;}

        void  Set_Cameras(Camera::Ptr left , Camera::Ptr right){
            camera_left_ = left;
            camera_right_ = right;}

    private:
        std::condition_variable map_update_;

        std::thread backend_thread_;
        std::atomic<bool> backend_running_;

        std::mutex data_mutex_;
        Map::Ptr map_;
        Camera::Ptr camera_left_ = nullptr;   // 左侧相机
        Camera::Ptr camera_right_ = nullptr;  // 右侧相机
    };
}

#endif //MYSLAM_BACK_H
