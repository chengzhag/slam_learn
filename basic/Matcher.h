//
// Created by pidan1231239 on 18-7-15.
//

#ifndef SLAM_LEARN_MATCHER_H
#define SLAM_LEARN_MATCHER_H

#include "common_include.h"
#include "Map.h"
#include "Frame.h"
#include <opencv2/opencv.hpp>
#include <opencv2/cvv.hpp>
#include "KeyFrame.h"
#include <algorithm>

namespace sky {

    using namespace cv;

    class Matcher {

    protected:
        cv::Ptr<DescriptorMatcher> matcher;
        double disThresRatio, disThresMin;
        vector<DMatch> matches;

    public:

        Matcher(cv::Ptr<DescriptorMatcher> matcher,
                double disThresRatio, double disThresMin) :
                matcher(matcher),
                disThresRatio(disThresRatio), disThresMin(disThresMin) {}

        size_t getMatchesNum() {
            return matches.size();
        }

        void match(Mat descriptors1,Mat descriptors2){
#ifdef DEBUG
            cout << "Matcher: matching keypoints... " << endl;
#endif

            matcher->match(descriptors1, descriptors2, matches, noArray());
#ifdef DEBUG
            cout << "\tfound " << matches.size() << " keypoints matched with last frame" << endl;
#endif

/*#ifdef CVVISUAL_DEBUGMODE
            cvv::debugDMatch(keyFrame1->image, keyFrame1->keyPoints, keyFrame2->image, keyFrame2->keyPoints, matches,
                             CVVISUAL_LOCATION,
                             "matching");
#endif*/

            //过滤匹配点
            auto minMaxDis = minmax_element(
                    matches.begin(), matches.end(),
                    [](const DMatch &m1, const DMatch &m2) {
                        return m1.distance < m2.distance;
                    });
            auto minDis = minMaxDis.first->distance;
            auto maxDis = minMaxDis.second->distance;
            vector<DMatch> goodMatches;
            for (auto match:matches) {
                if (match.distance <= max(disThresRatio * minDis, disThresMin))
                    goodMatches.push_back(match);
            }
            matches = goodMatches;
#ifdef DEBUG
            cout << "\tfound " << matches.size() << " good matches" << endl;
#endif
        }

    };

}


#endif //SLAM_LEARN_MATCHER_H
