//
// Created by denghanjun on 2021/12/1.
//

#include "back.h"
#include "g2o_types.h"
#include "Feature.h"

namespace myslam
{




  myslam::Back::Back()
  {
     backend_running_.store(true);
     backend_thread_ = std::thread(std::bind(&Back::BackendLoop, this));


  }

    void myslam::Back::UpdateMap()
    {
      std::unique_lock<std::mutex> lock(data_mutex_);
      map_update_.notify_one();

    }

    void myslam::Back::Stop() {
        backend_running_.store(false);
        map_update_.notify_one();
        backend_thread_.join();
    }

    void myslam::Back::BackendLoop()
    {
      while (backend_running_.load())
      {
          std::unique_lock<std::mutex> lock(data_mutex_);
          map_update_.wait(lock); // wait for notify ,execute

          //optimizier

          Map::KeyFrameType Active_KeyFrames = map_->Get_ActiveKeyFrames();
          Map::LandmarksType Active_MapPoints = map_->Get_ActiveMapPoints();

          Optimize(Active_KeyFrames,Active_MapPoints);

      }

    }

    void  myslam::Back::Optimize(Map::KeyFrameType &active_KFs,Map::LandmarksType &active_Mappoints){

      //set
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>
                LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<BlockSolverType>(
                        g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);


    //add pose
    std::map<unsigned long ,VertexPose* > vertices;
    unsigned long max_KFid = 0;
    for (auto &kf : active_KFs )
    {
        auto keyframe = kf.second;
        VertexPose *vertex_pose = new VertexPose();
        vertex_pose->setId(keyframe->keyframe_id_);
        vertex_pose->setEstimate(keyframe->Get_Pose());
        optimizer.addVertex(vertex_pose);
        if(kf.first > max_KFid)
            max_KFid = kf.first;

        vertices.insert({kf.first,vertex_pose});
    }

    std::map<unsigned long ,VertexXYZ *> vertices_landmarks;
    Eigen::Matrix3d K = camera_left_->Get_K();

    int idx = 1; // edge idx
    double chi2_th = 5.991;
    std::map<EdgeProjection *,Feature::Ptr> edges_and_features;
    SE3d left_cam_pose = camera_left_->Get_Pos() ;
    SE3d right_cam_pose = camera_right_->Get_Pos();

    for (auto &mp : active_Mappoints)
    {
        if(mp.second->is_outlier_) continue;
        unsigned long landmark_id = mp.second->id_;
        auto observation = mp.second->Get_Obs();

        for (auto &obs : observation)
        {

            if ( obs.lock() == nullptr) continue;
            auto feat = obs.lock();
            if (feat->is_outlier || feat->Frame_.lock() == nullptr) continue;

            auto frame = feat->Frame_.lock();
            EdgeProjection *edge= nullptr;
            if (feat->is_on_left_image)
                {
                    edge = new EdgeProjection(K,left_cam_pose);
                }
            else
                {
                    edge = new EdgeProjection(K,right_cam_pose);
                }

            if(vertices_landmarks.find(landmark_id) == vertices_landmarks.end())
            {
                VertexXYZ * vertex_mp = new VertexXYZ();
                vertex_mp->setId(max_KFid + 1 + landmark_id);
                vertex_mp->setEstimate(mp.second->Get_Pos());
                vertex_mp->setMarginalized(true);
                optimizer.addVertex(vertex_mp);
                vertices_landmarks.insert({landmark_id,vertex_mp});
            }


            edge->setId(idx);
            edge->setVertex(0,vertices.at(frame->keyframe_id_));
            edge->setVertex(1,vertices_landmarks.at(landmark_id));
            edge->setMeasurement(Eigen::Vector2d(feat->position_.pt.x,feat->position_.pt.y));
            edge->setInformation(Eigen::Matrix2d::Identity());
            auto rk = new g2o::RobustKernelHuber();
            rk->setDelta(chi2_th);
            edge->setRobustKernel(rk);
            edges_and_features.insert({edge,feat});

            optimizer.addEdge(edge);
            idx++;
        }



    }

    optimizer.initializeOptimization();
    optimizer.optimize(10);

    //adjust chi2_th
    int cnt_inlier,cnt_outlier;
    int iteration = 0;
    while (iteration < 5)
    {
        cnt_outlier = 0;
        cnt_inlier = 0;
        for (auto &iter : edges_and_features)
        {
            if (iter.first->chi2() > chi2_th )
            {
                cnt_outlier ++;
            }
            else
            {
                cnt_inlier ++;
            }
        }
        double inlier_ratio = cnt_inlier / double (cnt_outlier + cnt_inlier);
        if (inlier_ratio > 0.5)
            break;
        else
            chi2_th *= 2;
        iteration++;
    }

    for (auto &iter : edges_and_features)
    {
        if(iter.first->chi2() > chi2_th)
        {
            iter.second->is_outlier = true;
            iter.second->MapPoint_.lock()->RemoveObservation(iter.second); //removeObs have reset feature->mappoint_ （shuang xiang)

        } else
            iter.second->is_outlier = false;
    }

        LOG(INFO) << "Outlier/Inlier in optimization: " << cnt_outlier << "/"
                  << cnt_inlier;

    for (auto &vertex : vertices)
        active_KFs.at(vertex.first)->Set_Pose(vertex.second->estimate());
    for (auto &vertex : vertices_landmarks)
        active_Mappoints.at(vertex.first)->Set_Pos(vertex.second->estimate());

  }
}
