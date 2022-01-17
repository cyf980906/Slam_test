//
// Created by denghanjun on 2021/11/26.

#include "Front.h"
#include "g2o_types.h"
#include "Config.h"
#include "viewer.h"


#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

namespace myslam {

Front::Front() {
    gftt_ = cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
    num_features_init_ = Config::Get<int>("num_features_init");
    num_features_ = Config::Get<int>("num_features");
    num_features_tracking_ = Config::Get<int>("num_features_tracking"); //track good criteria
    num_features_tracking_bad_ = Config::Get<int>("num_features_tracking_bad"); //track bad criteria
    num_features_needed_for_keyframe_ = Config::Get<int>("num_features_needed_for_keyframe");
    std::cout << "1" <<std::endl;
}
    bool myslam::Front::GrabNewimage(Frame::Ptr frame) {
        if(frame == nullptr)LOG(INFO) <<"error frame is nullptr";else LOG(INFO) <<"frame is ok";




      //  LOG(INFO) << "GrabNewimage is running 1";
     //   LOG(INFO) << "GrabNewimage is running 2";


  // std::cout<< test <<std::endl;
       // CurrentFrame_ = frame;


//        LOG(INFO) << "GrabNewimage is running 3";
//        switch (FrontStatus_) {
//            case FrontStatus::Initialize: {
//                SeteroInitialize();
//                LOG(INFO) << "GrabNewimage is running 3";
//            }
//                break;
//
//            case FrontStatus::Track_Good :
//                TrackWithMotionModel();
//                break;
//            case FrontStatus::Track_bad :
//                TrackReferenceKeyFrame();
//                break;
//            case FrontStatus::Lost :
//                //TrackReferenceKeyFrame();
//                Reset();
//                break;
//
//        }
//       LastFrame_ = CurrentFrame_;
        return true;
    }


    int myslam::Front::Extract()
    {
        LOG(INFO) << "Extract is running";
        cv::Mat mask(CurrentFrame_->left_imgae_.size(), CV_8UC1, 255);
        for (auto &feat : CurrentFrame_->letf_features_) {
            cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                          feat->position_.pt + cv::Point2f(10, 10), 0, CV_FILLED);
        }
        std::vector <cv::KeyPoint> KeyPoints;

        gftt_->detect(CurrentFrame_->left_imgae_, KeyPoints, mask);
        if (KeyPoints.empty()) return 0 ;
        int cnt_detected = 0;
        for (auto &keypoint : KeyPoints) {

            CurrentFrame_->letf_features_.push_back(Feature::Ptr(new Feature(CurrentFrame_,keypoint)));
            cnt_detected++;
        }

        LOG(INFO) << "Detect " << cnt_detected << " new features";
        return cnt_detected;

    }

    int myslam::Front::FindRightFeature() {
         std::vector<cv::Point2f>  kps_left,kps_right;
        for (auto &kp:CurrentFrame_->letf_features_) {
            kps_left.push_back(kp->position_.pt);
            auto mp = kp->MapPoint_.lock();// some will have mappoint(set keyframe addobservation),some won't have(new extract)
            if (mp) {
                //pixel
               auto pp = camera_right_->world2pixel(CurrentFrame_->Get_Pose(), mp->Get_Pos());
                kps_right.push_back(cv::Point2f(pp[0],pp[1]));
            }
            else
                kps_right.push_back(kp->position_.pt);
        }
        int cnd_cor = 0;
        std::vector<uchar> status;
        cv::Mat error;
        cv::calcOpticalFlowPyrLK(
                CurrentFrame_->left_imgae_, CurrentFrame_->right_image_, kps_left,
                kps_right, status, error, cv::Size(11, 11), 3,
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                                 0.01),
                cv::OPTFLOW_USE_INITIAL_FLOW);

        for (size_t i = 0; i < status.size(); ++i) {
            if(status[i])
            {
                auto kp = cv::KeyPoint(kps_right.at(i),7);
                Feature::Ptr feat(new Feature(CurrentFrame_,kp));
                feat->is_on_left_image = false;
                CurrentFrame_->right_features_.push_back(feat);
                cnd_cor++;
            }
            else
                CurrentFrame_->right_features_.push_back(nullptr);
        }

        LOG(INFO) << "Find " << cnd_cor << " in the right image.";
        return cnd_cor;
    }

   bool myslam::Front::triangulation(const std::vector<SE3d> &pose,const std::vector<Eigen::Vector3d> &pos_cam,Eigen::Vector3d &pos_world)
   {
        Eigen::MatrixXd A(pose.size()*2,4);
        Eigen::VectorXd b(2 * pose.size());
         b.setZero();
       for (size_t i = 0; i < pose.size(); ++i) {
           Eigen::Matrix<double,3,4> T = pose[i].matrix3x4();
           Eigen::Vector3d px = pos_cam[i];
          A.block<1,4>(2 * i,0) = -T.row(0) + px[0] * T.row(2);
           A.block<1,4>(2 * i + 1,0) = -T.row(1) + px[1] * T.row(2);

       }
       auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
       pos_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();
       if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
           // 解质量不好，放弃
           return true;
       }
       return false;
   }

    bool myslam::Front::BuildMap(bool isIniti)
    {
        int cnt_triangulated_pts = 0;
        std::vector<SE3d> poses{camera_left_->Get_Pos(),camera_right_->Get_Pos()};

        for (size_t i = 0; i < CurrentFrame_->letf_features_.size(); ++i) {
            if(CurrentFrame_->right_features_.at(i) == nullptr)continue;
            if(!CurrentFrame_->letf_features_.at(i)->MapPoint_.expired())continue; // had have mappoint by matching
            auto kp1 = CurrentFrame_->letf_features_.at(i)->position_;
            auto kp2 = CurrentFrame_->right_features_.at(i)->position_;



            std::vector<Eigen::Vector3d> cam_position
            {camera_left_->pixel2camera(Eigen::Vector2d (kp1.pt.x,kp1.pt.y)),
            camera_right_->pixel2camera(Eigen::Vector2d (kp2.pt.x,kp2.pt.y))};
            Eigen::Vector3d position = Eigen::Vector3d::Zero();
            if(triangulation(poses,cam_position,position) && position[2]>0.0) {

                if(!isIniti)
                {
                    SE3d current_pose_Twc = CurrentFrame_->Get_Pose().inverse();
                    position = current_pose_Twc * position;
                }
                MapPoint::Ptr mappoint = MapPoint::CreatnewMappoint();
                mappoint->Set_Pos(position);
                mappoint->AddnewObservation(CurrentFrame_->letf_features_.at(i));
                mappoint->AddnewObservation(CurrentFrame_->right_features_.at(i));
                CurrentFrame_->letf_features_.at(i)->MapPoint_ = mappoint;
                CurrentFrame_->right_features_.at(i)->MapPoint_ = mappoint;
                cnt_triangulated_pts++;
                map_->InsertNewMapPoint(mappoint);
            }


        }
        if(isIniti) {
            //set keyframe
            CurrentFrame_->Set_KeyFrame();
            //insert map
            map_->InsertNewKeyFrame(CurrentFrame_);

            backend_->UpdateMap();
            LOG(INFO) << "Initial map created with " << cnt_triangulated_pts
                      << " map points";
        } else
        {

            LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
        }
        return true;
    }


    void myslam::Front::SeteroInitialize() {
        //extract
        LOG(INFO) << "SeteroInitialize is running";
        Extract();
        int num_coor_right_feature = FindRightFeature();
        if(num_coor_right_feature < num_features_init_)
            return ;
        // match success,build map

      bool build_success = BuildMap(true);
    if(build_success)
        {
            FrontStatus_ = FrontStatus::Track_Good;
            RefKeyFrame_ = CurrentFrame_;
            if (viewer_)
            {
                viewer_->AddCurrentFrame(CurrentFrame_,0);
                 viewer_->UpdateMap();
            }
            LOG(INFO) << "finish initialization";
        }
    }

    int myslam::Front::EstimatePos()
    {
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
                LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<BlockSolverType>(
                        g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        VertexPose *vertex  = new VertexPose();
        vertex->setId(0);
        vertex->setEstimate(CurrentFrame_->Get_Pose());
        optimizer.addVertex(vertex);

        int id=1;
        std::vector<EdgeProjectionPoseOnly *> Edges;
        std::vector<Feature::Ptr> Features;
        Eigen::Matrix3d K = camera_left_->Get_K();
        for (auto &kp : CurrentFrame_->letf_features_) {
            auto mp = kp->MapPoint_.lock();
            if (mp) {
                Features.push_back(kp);
                EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(mp->Get_Pos(), K);
                edge->setId(id);
                edge->setVertex(0,vertex);
                edge->setMeasurement(Eigen::Vector2d(kp->position_.pt.x,kp->position_.pt.y));
                edge->setInformation(Eigen::Matrix2d::Identity());
                edge->setRobustKernel(new g2o::RobustKernelHuber);
                optimizer.addEdge(edge);
                Edges.push_back(edge);
                id++;
            }
        }

        // optimilize
        int cnt_outlier = 0;
        const double  chi2_th = 5.991;
        for (int i = 0; i < 4; ++i) {
            vertex->setEstimate(CurrentFrame_->Get_Pose());
            optimizer.initializeOptimization();
            optimizer.optimize(10);
            cnt_outlier = 0;


            for (size_t j = 0; j < Edges.size() ; ++j)
            {
                auto edge = Edges.at(j);
                if( Features.at(j)->is_outlier)
                {
                    edge->computeError();

                }
                if(edge->chi2() > chi2_th)
                {
                    Features.at(j)->is_outlier = true;
                    edge->setLevel(1);
                    cnt_outlier++;
                }
                else
                {
                    Features.at(j)->is_outlier = false;
                    edge->setLevel(0);
                }
                if(i == 2)edge->setRobustKernel(nullptr);
            }


        }
        int inlier_num = Features.size() - cnt_outlier;
        LOG(INFO) << "Inlier in pose estimating: " << cnt_outlier << "/"
             << inlier_num ;
            CurrentFrame_->Set_Pose(vertex->estimate());
        LOG(INFO)  << "Current Pose = \n" << CurrentFrame_->Get_Pose().matrix();


        for (auto &feat : Features) {
            if(feat->is_outlier)
            {
                feat->MapPoint_.reset();
                feat->is_outlier = false;
            }
        }

        return inlier_num;

    }

    void myslam::Front::TrackLastFrame(Frame::Ptr TrackFrame)
    {


        std::vector<cv::Point2f> kps_last;
        std::vector<cv::Point2f> kps_current;
        for (auto &kp: TrackFrame->letf_features_) {
            if (kp->MapPoint_.lock()) {
                // use project point
                auto mp = kp->MapPoint_.lock();
                Eigen::Vector2d pp = camera_left_->world2pixel(CurrentFrame_->Get_Pose(), mp->Get_Pos());
                kps_last.push_back(kp->position_.pt);
                kps_current.push_back(cv::Point2f(pp[0],pp[1]));
            }
            else {
                kps_last.push_back(kp->position_.pt);
                kps_current.push_back(kp->position_.pt);
            }
        }

        std::vector<uchar> status;
        cv::Mat error;
        cv::calcOpticalFlowPyrLK(TrackFrame->left_imgae_,CurrentFrame_->left_imgae_,kps_last,kps_current,status,error,
                                 cv::Size(11,11),3,
                                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,0.01),
                                 cv::OPTFLOW_USE_INITIAL_FLOW);

        int num_good_pts = 0;
        for (size_t i = 0; i < status.size(); ++i) {
            if(status[i])
            {
                auto kp = cv::KeyPoint(kps_current.at(i),7);
                Feature::Ptr feat(new Feature(CurrentFrame_,kp));
                feat->MapPoint_ = TrackFrame->letf_features_.at(i)->MapPoint_;

                CurrentFrame_->letf_features_.push_back(feat);
                num_good_pts++;

            }
        }

        LOG(INFO) << "Track " << num_good_pts << " in the Current image.";


    }
    void myslam::Front::SetObservationsForKeyFrame(){

        for (auto &feat:CurrentFrame_->letf_features_) {
            auto mp = feat->MapPoint_.lock();
            if (mp)
            {
                mp->AddnewObservation(feat);

            }
        }


    }
    void myslam::Front::CreateNewKeyFrame()
    {
        if (Track_inlier_num_ >= num_features_needed_for_keyframe_)
            return; // still have enough features, don't insert keyframe

        CurrentFrame_->Set_KeyFrame();
        //insert map_
        map_->InsertNewKeyFrame(CurrentFrame_);
        RefKeyFrame_ = CurrentFrame_;

        LOG(INFO) << "Set frame " << CurrentFrame_->frame_Id_ << " as keyframe "
                  << CurrentFrame_->keyframe_id_;
       SetObservationsForKeyFrame(); //mappoint add observation(mapppoint -> keyframe)

       Extract(); //add new features ,currentFrame only obtained features by LKflow matching,not extract yet;

       FindRightFeature(); // add letf-right match

       BuildMap();

        backend_->UpdateMap();

        if (viewer_) viewer_->UpdateMap();
    }


    void myslam::Front::TrackWithMotionModel()
    {
        if(LastFrame_) {
            CurrentFrame_->Set_Pose(Tcl * LastFrame_->Get_Pose());

        }
        TrackLastFrame(LastFrame_);

        Track_inlier_num_ =  EstimatePos();
        if (Track_inlier_num_ > num_features_tracking_)
            FrontStatus_ = FrontStatus::Track_Good;
        else if (Track_inlier_num_ > num_features_tracking_bad_)
            FrontStatus_ = FrontStatus::Track_bad;
        else
            FrontStatus_ = FrontStatus::Lost;

        // SET NEW KEYFRAME
        CreateNewKeyFrame();

        // UPDATE Realtive transform
        Tcl = CurrentFrame_->Get_Pose() * LastFrame_->Get_Pose().inverse();
        if (viewer_) viewer_->AddCurrentFrame(CurrentFrame_,1);
    }

    void myslam::Front::TrackReferenceKeyFrame()
    {

        if(RefKeyFrame_ == nullptr)
        {
            FrontStatus_ = FrontStatus::Lost;
            return;
        }

        CurrentFrame_ ->Set_Pose(LastFrame_->Get_Pose()) ;
        TrackLastFrame(RefKeyFrame_);

        Track_inlier_num_ =  EstimatePos();
        if (Track_inlier_num_ > num_features_tracking_)
            FrontStatus_ = FrontStatus::Track_Good;
        else if (Track_inlier_num_ > num_features_tracking_bad_)
            FrontStatus_ = FrontStatus::Track_bad;
        else
            FrontStatus_ = FrontStatus::Lost;

        // SET NEW KEYFRAME
        CreateNewKeyFrame();

        // UPDATE Realtive transform
        Tcl = CurrentFrame_->Get_Pose() * LastFrame_->Get_Pose().inverse();
        if (viewer_) viewer_->AddCurrentFrame(CurrentFrame_,2);

    }


void myslam::Front::Reset()
{

    LOG(INFO) << "Reset is not implemented. ";

}
}
