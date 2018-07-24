//
// Created by pidan1231239 on 18-7-15.
//

#ifndef SLAM_LEARN_MATCHER_H
#define SLAM_LEARN_MATCHER_H

#include "common_include.h"
#include "Map.h"
#include <opencv2/opencv.hpp>
#include "Config.h"

namespace sky {

    class Matcher {

    private:
        cv::Ptr<cv::DescriptorMatcher> matcher;
        double disThresRatio, disThresMin;

    public:
        vector<cv::DMatch> matches;

        Matcher(cv::Ptr<cv::DescriptorMatcher> matcher,
                double disThresRatio = Config::get<double>("Matcher.disThresRatio"),
                double disThresMin = Config::get<double>("Matcher.disThresMin")) :
                matcher(matcher),
                disThresRatio(disThresRatio), disThresMin(disThresMin) {}

        void match(Mat descriptors1,Mat descriptors2);

        size_t getMatchesNum();

    };

}


#endif //SLAM_LEARN_MATCHER_H
