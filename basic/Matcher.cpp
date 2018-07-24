//
// Created by pidan1231239 on 18-7-15.
//

#include "Matcher.h"
#include <opencv2/cvv.hpp>

namespace sky {

    size_t Matcher::getMatchesNum() {
        return matches.size();
    }

    void Matcher::match(Mat descriptors1,Mat descriptors2){
#ifdef DEBUG
        cout << "Matcher: matching... ";
#endif

        matcher->match(descriptors1, descriptors2, matches, cv::noArray());
#ifdef DEBUG
        cout << matches.size() <<" matches found ";
#endif

/*#ifdef CVVISUAL_DEBUGMODE
            cvv::debugDMatch(keyFrame1->image, keyFrame1->keyPoints, keyFrame2->image, keyFrame2->keyPoints, matches,
                             CVVISUAL_LOCATION,
                             "matching");
#endif*/

        //过滤匹配点
        auto minMaxDis = minmax_element(
                matches.begin(), matches.end(),
                [](const cv::DMatch &m1, const cv::DMatch &m2) {
                    return m1.distance < m2.distance;
                });
        auto minDis = minMaxDis.first->distance;
        auto maxDis = minMaxDis.second->distance;
        vector<cv::DMatch> goodMatches;
        for (auto match:matches) {
            if (match.distance <= max(disThresRatio * minDis, disThresMin))
                goodMatches.push_back(match);
        }
        matches = goodMatches;
#ifdef DEBUG
        cout << "(" << matches.size() << " good matches)" << endl;
#endif
    }

}