//
// Created by denghanjun on 2021/11/25.
//

#include "MapPoint.h"


namespace myslam {

    void myslam::MapPoint::AddnewObservation(std::shared_ptr<Feature> feature) {

        std::unique_lock<std::mutex> lck(data_mutex_);
        observation_.push_back(feature);
        obs_num++;
    }

    void myslam::MapPoint::RemoveObservation(std::shared_ptr<Feature> feature) {
        std::unique_lock<std::mutex> lck(data_mutex_);
        for (auto iter = observation_.begin(), end = observation_.end(); iter != end; iter++) {
            if (iter->lock() == feature) {
                observation_.erase(iter);
                feature->MapPoint_.reset();
                obs_num--;
                break;
            }

        }
    }

    myslam::MapPoint::Ptr myslam::MapPoint::CreatnewMappoint() {
        static unsigned long id = 0;
        Ptr newpoint(new MapPoint());
        newpoint->id_ = id;
        id++;
        return newpoint;


    }
}
