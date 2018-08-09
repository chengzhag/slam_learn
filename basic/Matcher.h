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
    public:
        float rankRatio, disThresRatio, disThresMin, testRatio;
        vector<cv::DMatch> matches;

        Matcher(float rankRatio = Config::get<float>("Matcher.rankRatio"),
                float disThresRatio = Config::get<float>("Matcher.disThresRatio"),
                float disThresMin = Config::get<float>("Matcher.disThresMin"),
                float testRatio = Config::get<float>("Matcher.testRatio"),
                string matcherType = Config::get<string>("Matcher.matcherType")
        ) :
                rankRatio(rankRatio),
                disThresRatio(disThresRatio),
                disThresMin(disThresMin),
                testRatio(testRatio),
                matcher(cv::DescriptorMatcher::create(matcherType)) {
#ifdef DEBUG
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "Matcher: Initializing..." << endl;
            printVariable(rankRatio);
            printVariable(disThresRatio);
            printVariable(disThresMin);
            printVariable(testRatio);
            printVariable(matcherType);
#endif
        }

        void match(const Mat &descriptors1, const Mat &descriptors2);

        inline size_t getMatchesNum() const {
            return matches.size();
        }

    };

}


#endif //SLAM_LEARN_MATCHER_H
