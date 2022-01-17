//
// Created by denghanjun on 2021/11/30.
//

#include "Map.h"


namespace myslam
{

     void myslam::Map::RemoveKeyFrame()
     {

        // STEP1 Find min and max Frame

        if (CurrentFrame_ == nullptr)return;
        double max_dis = 0,min_dis = MAXFLOAT;
        double min_idx ,max_idx;
         SE3d Twc =  CurrentFrame_->Get_Pose().inverse();
         double dis;
        for (auto &iter : ActiveKeyFrames_)
        {
            if (iter.first == CurrentFrame_->keyframe_id_) continue;
            dis = (Twc * iter.second->Get_Pose()).log().norm();
            if (dis > max_dis)
            {
                max_dis = dis;
              max_idx = iter.first;

            }
            if (dis < min_dis)
            {
                min_dis = dis;
                min_idx = iter.first;
            }
        }
        // choose remove which one
         double dis_ch = 0.2;
         double remove_idx;
         // too silimiar (close)
        if (min_dis < dis_ch)
            remove_idx = min_idx;
        else
            remove_idx = max_idx;


        Frame::Ptr removekeyframe =  ActiveKeyFrames_[remove_idx];

        ActiveKeyFrames_.erase(remove_idx);

        //remove map-observation

    for (auto &feat:removekeyframe->letf_features_)
    {
        auto mp = feat->MapPoint_.lock();
        if (mp)
        {
            mp->RemoveObservation(feat);
        }
    }

         for (auto &feat:removekeyframe->right_features_)
         {
             if (feat == nullptr) continue;
             auto mp = feat->MapPoint_.lock();
             if (mp)
             {
                 mp->RemoveObservation(feat);
             }
         }

        //remove mappoint that observation_num is 0 from Map

        CleanMap();

     }

    void myslam::Map::InsertNewKeyFrame(Frame::Ptr frame)
    {
        CurrentFrame_ = frame;
                    if (KeyFrames_.find(frame->keyframe_id_) == KeyFrames_.end())
                    {
                        // new
                        KeyFrames_.insert(std::make_pair(frame->keyframe_id_,frame));
                        ActiveKeyFrames_.insert(std::make_pair(frame->keyframe_id_,frame));
                    }
                    else
                    {
                        //old ??? back will insert a keyframe with the same id?
                        KeyFrames_[frame->keyframe_id_] = frame;
                        ActiveKeyFrames_[frame->keyframe_id_] = frame;
                    }


       if ( ActiveKeyFrames_.size() > num_active_keyframes_)
        RemoveKeyFrame();
    }

    void myslam::Map::InsertNewMapPoint(MapPoint::Ptr mappoint)
    {
        if (MapPoints_.find(mappoint->id_) == MapPoints_.end())
        {
            // new
            MapPoints_.insert(std::make_pair(mappoint->id_,mappoint));
            ActiveMapPoints_.insert(std::make_pair(mappoint->id_,mappoint));
        }
        else
        {
            //old ??? back will insert a mappoint with the same id?
            MapPoints_[mappoint->id_] = mappoint;
            ActiveMapPoints_[mappoint->id_] = mappoint;
        }

    }


    void  myslam::Map:: CleanMap()
    {
         for(auto iter = ActiveMapPoints_.begin(),end = ActiveMapPoints_.end();iter != end;)
         {
            if (iter->second->obs_num == 0)
                iter =  ActiveMapPoints_.erase(iter);//return erase position next iterator
            else
                iter++;
         }
    }

}
