//
// Created by denghanjun on 2021/11/26.
//

#pragma once
#ifndef MYSLAM_FRONT_H
#define MYSLAM_FRONT_H


#include "common_include.h"
#include "Frame.h"
#include "Feature.h"
#include "Camera.h"
#include "MapPoint.h"
#include "MapPoint.h"
#include "Map.h"
#include "viewer.h"
#include "back.h"
namespace myslam {

    class Camera;
    class Frame;

    enum class FrontStatus  {Initialize,Track_Good,Track_bad,Lost};
    class Front {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Front> Ptr;

        Front();


        bool GrabNewimage(Frame::Ptr frame);

        void SeteroInitialize();

        void TrackWithMotionModel();

        void TrackReferenceKeyFrame();

        int Extract();

        inline double CalDepth();

        inline  bool triangulation(const std::vector<SE3d> &pose,const std::vector<Eigen::Vector3d> &pos_cam,Eigen::Vector3d &pos_world);

        void TrackLastFrame(Frame::Ptr TrackFrame);

        int EstimatePos();

        bool BuildMap(bool isIniti = false);

        int FindRightFeature();//LightFlow

        void CreateNewKeyFrame();

        FrontStatus FrontStatus_ = FrontStatus::Initialize;

        void SetObservationsForKeyFrame();

        void Reset();

        void SetViewer(Viewer::Ptr viewer){viewer_ = viewer;};

        void SetMap(Map::Ptr map){map_ = map;};

        void SetCamera(Camera::Ptr left ,Camera::Ptr right){camera_left_ = left;camera_right_=right;};

        void SetBackend(Back::Ptr backend){backend_ = backend;};

       Frame::Ptr CurrentFrame_ = Frame::Ptr(new Frame);

       Frame::Ptr LastFrame_= nullptr;

        SE3d Tcl;//realtive pose

        Camera::Ptr camera_ = nullptr;

        Camera::Ptr camera_left_ = nullptr;   // 左侧相机
        Camera::Ptr camera_right_ = nullptr;  // 右侧相机

        int Track_inlier_num_ = 0;

        int test = 0;

    private:

        Frame::Ptr RefKeyFrame_ = nullptr;

        Viewer::Ptr viewer_ = nullptr;
        Map::Ptr map_ = nullptr;
        Back::Ptr backend_ = nullptr;
        cv::Ptr<cv::GFTTDetector> gftt_;
        int num_features_init_;//initialize extra num success ctiteria
        int num_features_;//track extra num success ctiteria
        int num_features_tracking_ ; //track good criteria
        int num_features_tracking_bad_ ; //track bad criteria
        int num_features_needed_for_keyframe_;
    };
}

#endif //MYSLAM_FRONT_H
