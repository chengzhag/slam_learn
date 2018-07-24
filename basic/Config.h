//
// Created by pidan1231239 on 18-7-23.
//

#ifndef SLAM_LEARN_CONFIG_H
#define SLAM_LEARN_CONFIG_H

#include "common_include.h"

namespace sky {

    class Config {
    private:
        static std::shared_ptr<Config> config_;
        cv::FileStorage file_;

        Config() {} // private constructor makes a singleton
    public:
        ~Config();  // close the file when deconstructing

        // set a new config file
        static void setParameterFile(const std::string &filename);

        // access the parameter values
        template<typename T>
        static T get(const std::string &key) {
            if (!Config::config_->file_[key].empty())
                return T(Config::config_->file_[key]);
            else
                cerr << "Config: Cannot find '" + key + "' in configure file" << endl;
        }
    };

}


#endif //SLAM_LEARN_CONFIG_H
