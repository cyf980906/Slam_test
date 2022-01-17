//
// Created by denghanjun on 2021/11/30.
#pragma once
#ifndef MYSLAM_CONFIG_H
#define MYSLAM_CONFIG_H
#include "common_include.h"

namespace myslam {
    class Config {
    public:
        typedef std::shared_ptr<Config> Ptr;

        Config(){}

        ~Config();

        static bool SetParameterFile(const std::string &filename);

        //read value according to key from config file
        template<typename T>
        static T Get(const std::string &key)
        {
            return Config::config_->file_[key];
        };

    private:
        static std::shared_ptr<Config> config_; //config_ is static
        cv::FileStorage file_;
    };
}


#endif //MYSLAM_CONFIG_H
