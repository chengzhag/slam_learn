//
// Created by pidan1231239 on 18-7-15.
//

#ifndef SLAM_LEARN_MATCHER_H
#define SLAM_LEARN_MATCHER_H

#include "common_include.h"
#include "Map.h"
#include <opencv2/opencv.hpp>

namespace sky {

    class Matcher {

    protected:
        cv::Ptr<cv::DescriptorMatcher> matcher;
        double disThresRatio, disThresMin;
        vector<cv::DMatch> matches;

    public:

        Matcher(cv::Ptr<cv::DescriptorMatcher> matcher,
                double disThresRatio, double disThresMin) :
                matcher(matcher),
                disThresRatio(disThresRatio), disThresMin(disThresMin) {}

        size_t getMatchesNum();

        void match(Mat descriptors1,Mat descriptors2);

    };

}


#endif //SLAM_LEARN_MATCHER_H
