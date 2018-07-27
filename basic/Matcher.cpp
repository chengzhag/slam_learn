//
// Created by pidan1231239 on 18-7-15.
//

#include "Matcher.h"
#include <opencv2/cvv.hpp>

namespace sky {

    void Matcher::match(const Mat &descriptors1, const Mat &descriptors2) {
        if (testRatio == 0) {
#ifdef DEBUG
            cout << "Matcher: matching... ";
#endif
            matcher->match(descriptors1, descriptors2, matches, cv::noArray());

#ifdef DEBUG
            cout << matches.size() << " matches found ";
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
            auto thres = max(disThresRatio * minDis, disThresMin);
            for (auto match:matches) {
                if (match.distance <= thres)
                    goodMatches.push_back(match);
            }
            matches = goodMatches;
#ifdef DEBUG
            cout << "(" << matches.size() << " good matches, thres " << thres << ")" << endl;
#endif

        } else {

            vector<vector<cv::DMatch>> knnMatches;
#ifdef DEBUG
            cout << "Matcher: matching... ";
#endif
            matcher->knnMatch(descriptors1, descriptors2, knnMatches, 2);

#ifdef DEBUG
            cout << knnMatches.size() << " matches found ";
#endif

/*#ifdef CVVISUAL_DEBUGMODE
            cvv::debugDMatch(keyFrame1->image, keyFrame1->keyPoints, keyFrame2->image, keyFrame2->keyPoints, matches,
                             CVVISUAL_LOCATION,
                             "matching");
#endif*/
            //计算满足Ratio Test的最小距离
            float minDis = numeric_limits<float>::max();
            for (auto &m:knnMatches) {
                //Ratio Test
                if (m[0].distance > testRatio * m[1].distance)
                    continue;

                float dis = m[0].distance;
                if (dis < minDis) minDis = dis;
            }

            matches.clear();
            auto thres = max(disThresRatio * minDis, disThresMin);
            for (auto &m:knnMatches) {
                //排除不满足Ratio Test的点和匹配距离过大的点
                if (m[0].distance > testRatio * m[1].distance
                    || m[0].distance > thres)
                    continue;

                //保存匹配点
                matches.push_back(m[0]);
            }

#ifdef DEBUG
            cout << "(" << matches.size() << " good matches, thres " << thres << ")" << endl;
#endif
        }

    }

}