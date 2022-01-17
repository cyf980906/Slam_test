//
// Created by denghanjun on 2021/11/25.
//
#pragma once
#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H
#include "common_include.h"
#include "Feature.h"

namespace myslam {

    class Feature;
    class Frame;


    class MapPoint {

    public:
        typedef std::shared_ptr<MapPoint> Ptr;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        MapPoint(){};
        MapPoint(unsigned long id,Eigen::Vector3d position);
        inline Eigen::Vector3d Get_Pos()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return position_;
        }
        inline void Set_Pos(const Eigen::Vector3d &position)
        {

            std::unique_lock<std::mutex> lck(data_mutex_);
            position_ = position;

        }

        std::list<std::weak_ptr<Feature>> Get_Obs()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return observation_;

        }

        static Ptr CreatnewMappoint();


        void AddnewObservation(std::shared_ptr<Feature> feature);
        void RemoveObservation(std::shared_ptr<Feature> feature);
        unsigned long id_;
        std::mutex data_mutex_;
        bool is_outlier_ = false;
        unsigned long obs_num = 0;

       // std::map<std::shared_ptr<Frame>,int>  observation_;
    private:
        Eigen::Vector3d position_;
        std::list<std::weak_ptr<Feature>> observation_;


    };
}


#endif //MYSLAM_MAPPOINT_H
