//
// Created by denghanjun on 2021/11/30.
//
#include "visual_odometry.h"
#include "Config.h"
#include <chrono>
#include <Eigen/Core>

namespace myslam
{

    visual_odometry::visual_odometry(std::string &config_path): config_file_path_(config_path){};




    bool myslam::visual_odometry::Init()
    {
        // read config file
        if (!Config::SetParameterFile(config_file_path_))
        {
            return false; // fail
        }

      //  dataset_ = Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
       // CHECK_EQ(dataset_->Init(), true);

        fronted_ = Front::Ptr(new Front);
        map_ = Map::Ptr(new Map);
        viewer_ = Viewer::Ptr(new Viewer);
        backend_ = Back::Ptr(new Back);

        fronted_->SetViewer(viewer_);
        fronted_->SetMap(map_);

        double fx = Config::Get<double>("camera_fx");
        double fy = Config::Get<double>("camera_fy");
        double cx = Config::Get<double>("camera_cx");
        double cy = Config::Get<double>("camera_cy");
        double baseline = Config::Get<double>("camera_baseline");


        Camera::Ptr new_camera_left(new Camera(fx, fy, cx, cy,
                                               baseline, SE3d(SO3d (), Eigen::Vector3d(baseline/2,0,0))));


        Camera::Ptr new_camera_right(new Camera(fx, fy, cx, cy,
                                                baseline, SE3d(SO3d (), Eigen::Vector3d(-baseline/2,0,0))));

        fronted_->SetCamera(new_camera_left,new_camera_right);
        fronted_->SetBackend(backend_);


        backend_->Set_Map(map_);
        backend_->Set_Cameras(new_camera_left,new_camera_right);

        viewer_->SetMap(map_);

        return true;
    }


    bool myslam::visual_odometry::Run(const cv::Mat &image_left,const cv::Mat &image_right)
    {
       // if (Init())

         LOG(INFO) << "VO is running";
        if (Step(image_left,image_right))
            return true;
        else
            return false;



    }

    Frame::Ptr myslam::visual_odometry::NextFrame(const cv::Mat &image_left,const cv::Mat &image_right)
    {

        LOG(INFO) << "NextFrame is running";
        if (image_left.data == nullptr || image_right.data == nullptr) {
            LOG(WARNING) << "cannot find images ";
            return nullptr;
        }
        LOG(INFO) << "NextFrame 1";
        cv::Mat image_left_resized, image_right_resized;
        cv::resize(image_left, image_left_resized, cv::Size(), 0.5, 0.5,
                   cv::INTER_NEAREST);
        cv::resize(image_right, image_right_resized, cv::Size(), 0.5, 0.5,
                   cv::INTER_NEAREST);

        auto new_frame = Frame::CreatNewFrame();
        new_frame->left_imgae_ = image_left_resized;
        new_frame->right_image_ = image_right_resized;
        LOG(INFO) << "NextFrame 2";
        return new_frame;

    }

    bool myslam::visual_odometry::Step(const cv::Mat &image_left,const cv::Mat &image_right)
    {


        //Frame::Ptr  new_frame = dataset_->NextFrame();
        LOG(INFO) << "Step is running 1";
        Frame::Ptr  new_frame = NextFrame(image_left,image_right);
        if (new_frame == nullptr) return false;
        LOG(INFO) << "Step is running 2";
        auto t1 = std::chrono::steady_clock::now();
        bool flag = fronted_->GrabNewimage(new_frame);
        LOG(INFO) << "Step is running 3";
        auto t2 = std::chrono::steady_clock::now();
        auto time_used =
                std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
        return flag;


    }



}
