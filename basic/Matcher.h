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
        float disThresRatio, disThresMin, testRatio;

    public:
        vector<cv::DMatch> matches;

        Matcher(float disThresRatio = Config::get<float>("Matcher.disThresRatio"),
                float disThresMin = Config::get<float>("Matcher.disThresMin"),
                float testRatio = Config::get<float>("Matcher.testRatio")
        ) :
                disThresRatio(disThresRatio),
                disThresMin(disThresMin),
                testRatio(testRatio) {}

        void match(Mat descriptors1, Mat descriptors2);

        size_t getMatchesNum();

    };

}


#endif //SLAM_LEARN_MATCHER_H
