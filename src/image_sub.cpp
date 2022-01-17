//
// Created by denghanjun on 2021/12/20.
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <gflags/gflags.h>
#include "visual_odometry.h"
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

DEFINE_string(config_file, "./config/default.yaml", "config file path");


std::mutex m_buf_l;
std::mutex m_buf_r;
std::mutex estimator;
std::condition_variable con;
std::queue<cv::Mat> imLeft;
std::queue<cv::Mat> imRight;
using std::cout;

void imagelCallback(const sensor_msgs::ImageConstPtr &msg,const sensor_msgs::CameraInfoConstPtr &cameraInfoPtr)
{
    static int iter = 0;
    cv::Mat Image;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
    cv_ptr->image.copyTo(Image);
    if(Image.empty()) return ;

    m_buf_l.lock();
    imLeft.push(Image);
    m_buf_l.unlock();
    con.notify_one();
    //ROS_INFO("Receive Left_Image");
   // cout<<"fx:"<< cameraInfoPtr->P[0] << "\tcx:"<< cameraInfoPtr->P[2] <<"\tTx:"<< cameraInfoPtr->P[3]<<std::endl;
  //  cout<<"fy:"<< cameraInfoPtr->P[5] << "\tcy:"<< cameraInfoPtr->P[6] <<"\tTy:"<< cameraInfoPtr->P[7]<<std::endl;
   // cv::namedWindow("Left_Image");
   // cv::imshow("Left_Image",Image);
   // cv::waitKey(10);
    return;

}
void imagerCallback(const sensor_msgs::ImageConstPtr &msg,const sensor_msgs::CameraInfoConstPtr &cameraInfoPtr)
{

    cv::Mat Image;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
    cv_ptr->image.copyTo(Image);
    if(Image.empty()) return;
   // std::cout << "push back image right\n";
    m_buf_r.lock();
    imRight.push(Image);
    m_buf_r.unlock();
   // con.notify_one();
   // ROS_INFO("Receive Right_Image");
   // cout<<"fx:"<< cameraInfoPtr->P[0] << "\tcx:"<< cameraInfoPtr->P[2] <<"\tTx:"<< cameraInfoPtr->P[3]<<std::endl;
   // cout<<"fy:"<< cameraInfoPtr->P[5] << "\tcy:"<< cameraInfoPtr->P[6] <<"\tTy:"<< cameraInfoPtr->P[7]<<std::endl;
   // cv::namedWindow("Right_Image");
   //cv::imshow("Right_Image",Image);
    //cv::waitKey(10);
    return;

}

void process()
{

    myslam::visual_odometry::Ptr vo(new myslam::visual_odometry(FLAGS_config_file));
    vo->Init();


    while (true) {
        cout<<"queue size:"<<imLeft.size()<<std::endl;
        std::unique_lock<std::mutex> lk(estimator);
        con.wait(lk, [&]
        {
            return ( imRight.size()!= 0 && imLeft.size() !=0);
        });
        lk.unlock();
        cout<<"queue size:"<<imLeft.size()<<std::endl;
        m_buf_r.lock();m_buf_l.lock();
            cv::Mat imagel = imLeft.front();imLeft.pop();
            cv::Mat imageR = imRight.front();imRight.pop();
        m_buf_r.unlock();  m_buf_l.unlock();
        if ( !imagel.empty() && !imageR.empty()) {
            std::cout << "enter vo" << std::endl;
            if (!vo->Run(imagel, imageR)) {
                vo->Stop();
                ros::shutdown();
                return;
            };
        }
    }
}



int main(int argc,char** argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);

    ros::init(argc,argv,"image_sub");
    ros::NodeHandle nh;
    image_transport::ImageTransport it_imsee(nh);
    image_transport::CameraSubscriber image_l_sub = it_imsee.subscribeCamera("/imsee/image/left",1,imagelCallback);
    image_transport::CameraSubscriber image_r_sub = it_imsee.subscribeCamera("/imsee/image/right",1,imagerCallback);

    std::thread measurement_process{process};
    //cout<<"queue size:"<<imLeft.size()<<std::endl;
    ros::spin();


    return 0;
}


