//
// Created by denghanjun on 2021/11/29.
//
#pragma once
#ifndef MYSLAM_CAMERA_H
#define MYSLAM_CAMERA_H
#include "common_include.h"
namespace myslam {
    class Camera {

    public:
        typedef std::shared_ptr<Camera> Ptr;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        Camera();

        Camera(double fx, double fy, double cx, double cy,double baseline, SE3d pose) :fx_(fx), fy_(fy), cx_(cx), cy_(cy),
        baseline_(baseline), pose_(pose) {
            K_ << fx_, 0, cx_,
                    0, fy_, cy_,
                    0, 0, 1;
            pose_inverse_ = pose_.inverse();


        };

        double CalDepth(cv::KeyPoint &kp_left,cv::KeyPoint &kp_right);

        Eigen::Vector3d world2camera(const SE3d &Tcw,const Eigen::Vector3d &position);

         Eigen::Vector3d camera2world(const SE3d &Tcw,const Eigen::Vector3d &position);

         Eigen::Vector2d camera2pixel(const Eigen::Vector3d &position);

        Eigen::Matrix3d Get_K()const{return K_;};

         Eigen::Vector3d pixel2camera(const Eigen::Vector2d &pixel, double depth = 1);

         Eigen::Vector2d world2pixel(const SE3d &Tcw,const Eigen::Vector3d &position);

         Eigen::Vector3d pixel2world(const SE3d &Tcw,const Eigen::Vector2d &pixel,double depth = 1);

        SE3d Get_Pos()const{return pose_;};

    private:

        Eigen::Matrix3d K_;
        double fx_ = 0;
        double fy_ = 0;
        double cx_ = 0;
        double cy_ = 0;
        double baseline_ = 0;
        SE3d pose_;
        SE3d pose_inverse_;
    };

}
#endif //MYSLAM_CAMERA_H
