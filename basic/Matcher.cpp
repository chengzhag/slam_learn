//
// Created by pidan1231239 on 18-7-15.
//

#include "Matcher.h"

namespace sky {

    size_t Matcher::getMatchesNum() {
        return matches.size();
    }

    void Matcher::match(Mat descriptors1,Mat descriptors2){
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

}