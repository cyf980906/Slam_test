//
// Created by denghanjun on 2021/11/30.
//
#include "Config.h"


namespace myslam
{

    bool Config::SetParameterFile(const std::string &filename)
    {
        if (config_ == nullptr)
            config_ = Config::Ptr(new Config());
        // OPEN FILE
        config_->file_ = cv::FileStorage(filename.c_str(),cv::FileStorage::READ);
        if(!config_->file_.isOpened())
        {
            //LOG(ERROR) << "parameter file " << filename << " does not exist.";
            config_->file_.release();//desctruct

            return false;
        }
        return true;
    }


    Config::~Config()
    {
       if (file_.isOpened())
           file_.release();
    }

    std::shared_ptr<Config> Config::config_ = nullptr;//static member need initialize
}
