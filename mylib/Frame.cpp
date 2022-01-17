//
// Created by denghanjun on 2021/11/25.
//

#include "Frame.h"



namespace myslam {
    myslam::Frame::Frame(unsigned long id, SE3d pose, cv::Mat left_imgae, cv::Mat right_image) : frame_Id_(id),
                                                                                                 left_imgae_(
                                                                                                         left_imgae),
                                                                                                 right_image_(
                                                                                                         right_image),pose_(pose){}


    void myslam::Frame::Set_Id(unsigned long &id) {
        frame_Id_ = id;

    }


    std::shared_ptr<myslam::Frame> myslam::Frame::CreatNewFrame() {
        static unsigned long id = 0;
        Frame::Ptr newframe(new Frame());
        newframe->Set_Id(id);
        id++;
        return newframe;
    }

    void myslam::Frame::Set_KeyFrame()
    {
        static unsigned long KeyFrameid_ = 0;
        isKeyFrame_ = true;
        keyframe_id_ =  KeyFrameid_++;

    }
}
