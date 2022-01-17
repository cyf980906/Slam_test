//
// Created by denghanjun on 2021/11/29.
//

#include "Camera.h"


namespace myslam
{
    Camera::Camera()
    {
        fx_ = 517.3;
        fy_ = 516.5;
        cx_= 325.1;
        cy_ = 249.7;
        baseline_ = 120.0;//
        K_ << fx_, 0, cx_,
                0, fy_, fy_,
                0, 0, 1;
        pose_ = SE3d(SO3d(), Eigen::Vector3d(-baseline_, 0, 0));
        pose_inverse_ = pose_.inverse();

    }


    double myslam::Camera::CalDepth(cv::KeyPoint &kp_left,cv::KeyPoint &kp_right)
    {
        double disparty = kp_left.pt.x - kp_right.pt.x;
        return fx_ * baseline_ /disparty;
    }



    Eigen::Vector3d Camera::world2camera(const SE3d &Tcw,const Eigen::Vector3d &position)
    {
               return pose_*Tcw*position;

    }
    Eigen::Vector3d Camera::camera2world(const SE3d &Tcw,const Eigen::Vector3d &position)
    {
        return        Tcw.inverse()*pose_inverse_*position;

    }
    Eigen::Vector2d Camera::camera2pixel(const Eigen::Vector3d &position)
    {

        return  Eigen::Vector2d(K_.row(0)*(position/position[2]),K_.row(1)*(position/position[2]));

    }
    Eigen::Vector3d Camera::pixel2camera(const Eigen::Vector2d &pixel, double depth)
    {

        return       Eigen::Vector3d((pixel[0]-cx_)*depth/fx_,
                                     (pixel[1]-cy_)*depth/fy_,
                                     depth);

    }
    Eigen::Vector2d Camera::world2pixel(const SE3d &Tcw,const Eigen::Vector3d &position)
    {
        return  camera2pixel(world2camera(Tcw,position));

    }

    Eigen::Vector3d Camera::pixel2world(const SE3d &Tcw,const Eigen::Vector2d &pixel, double depth)
    {
        return camera2world(Tcw,pixel2camera(pixel,depth));
    }
}
