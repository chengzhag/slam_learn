//
// Created by pidan1231239 on 18-7-13.
//

#ifndef SLAM_LEARN_SOLVER2D2D_H
#define SLAM_LEARN_SOLVER2D2D_H

#include "common_include.h"
#include "Map.h"
#include "Frame.h"
#include <opencv2/opencv.hpp>
#include <opencv2/cvv.hpp>
#include "BA.h"
#include "KeyFrame.h"
#include <algorithm>

namespace sky {

    using namespace cv;

    class Solver2D2D {
    public:
        typedef shared_ptr<Solver2D2D> Ptr;
        vector<DMatch> matches;
        Mat inlierMask;

        Solver2D2D(KeyFrame::Ptr &keyFrame1, KeyFrame::Ptr &keyFrame2, cv::Ptr<DescriptorMatcher> matcher,
                   double disThresRatio = 5, double disThresMin = 200) :
                keyFrame1(keyFrame1), keyFrame2(keyFrame2), matcher(matcher),
                disThresRatio(disThresRatio),disThresMin(disThresMin){
            match();
            filtMatches();
            solve();
        }

        double getInlierRatio() {
            return (double) countNonZero(inlierMask) / matches.size();
        }

        size_t getMatcheNum() {
            return matches.size();
        }

    private:
        KeyFrame::Ptr keyFrame1, keyFrame2;
        cv::Ptr<DescriptorMatcher> matcher;
        double disThresRatio, disThresMin;

        void match() {
#ifdef DEBUG
            cout << "matching keypoints: " << endl;
#endif

            matcher->match(keyFrame1->descriptors, keyFrame2->descriptors, matches, noArray());
#ifdef DEBUG
            cout << "\tfound " << matches.size() << " keypoints matched with last frame" << endl;
#endif

/*#ifdef CVVISUAL_DEBUGMODE
            cvv::debugDMatch(keyFrame1->image, keyFrame1->keyPoints, keyFrame2->image, keyFrame2->keyPoints, matches,
                             CVVISUAL_LOCATION,
                             "2D-2D points matching");
#endif*/
        }

        void filtMatches() {
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

        void solve() {
            vector<Point2f> matchPoints1, matchPoints2;
            for (auto match:matches) {
                matchPoints1.push_back(keyFrame1->keyPoints[match.queryIdx].pt);
                matchPoints2.push_back(keyFrame2->keyPoints[match.trainIdx].pt);
            }

            Mat essentialMatrix;
            essentialMatrix = findEssentialMat(matchPoints1, matchPoints2,
                                               keyFrame2->frame->camera->getFocalLength(),
                                               keyFrame2->frame->camera->getPrincipalPoint(),
                                               RANSAC, 0.999, 1.0, inlierMask);
#ifdef DEBUG
            int nPointsFindEssentialMat = countNonZero(inlierMask);
            cout << "findEssentialMat: \n\t" << nPointsFindEssentialMat << " valid points, " <<
                 (float) nPointsFindEssentialMat * 100 / matchPoints1.size()
                 << "% of " << matchPoints1.size() << " points are used" << endl;
#endif
            //可视化用于解对极约束的点
#ifdef CVVISUAL_DEBUGMODE
            vector<DMatch> inlierMatches;
            vector<cv::KeyPoint> inlierKeyPoints1, inlierKeyPoints2;
            for (int i = 0; i < matches.size(); ++i) {
                if (!inlierMask.at<uint8_t>(i, 0))
                    continue;
                inlierMatches.push_back(matches[i]);
                inlierMatches.back().trainIdx = inlierKeyPoints1.size();
                inlierMatches.back().queryIdx = inlierKeyPoints2.size();
                inlierKeyPoints1.push_back(keyFrame1->keyPoints[matches[i].queryIdx]);
                inlierKeyPoints2.push_back(keyFrame2->keyPoints[matches[i].trainIdx]);
            }
            cvv::debugDMatch(keyFrame1->image, inlierKeyPoints1, keyFrame2->image, inlierKeyPoints2, inlierMatches,
                             CVVISUAL_LOCATION,
                             "match used in triangulation");

#endif

            //解frame2的R、t并计算se3
            Mat R, t;
            recoverPose(essentialMatrix, matchPoints1, matchPoints2,
                        keyFrame2->frame->camera->getKMatxCV(), R, t, inlierMask);
            Eigen::Matrix3d eigenR2;
            cv2eigen(R, eigenR2);
            keyFrame2->frame->Tcw = SE3(
                    eigenR2,
                    Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0))
            );
#ifdef DEBUG
            int nPointsRecoverPose = countNonZero(inlierMask);
            cout << "recoverPose: \n\t" << nPointsRecoverPose << " valid points, " <<
                 (float) nPointsRecoverPose * 100 / matchPoints1.size()
                 << "% of " << matchPoints1.size() << " points are used" << endl;
/*        cout << "2D-2D frame2 R: " << R.size << endl << R << endl;
        cout << "2D-2D frame2 t: " << t.size << endl << t << endl;
        cout << "2D-2D frame2 SE3: " << endl << keyFrame2->frame->Tcw << endl;
        cout << "2D-2D frame2 Tcw: " << endl << keyFrame2->frame->getTcwMatCV() << endl << endl;
        cout << "2D-2D frame2 ProjMat: " << endl << keyFrame2->frame->getTcw34MatCV() << endl << endl;*/
#endif
        }

    };

}


#endif //SLAM_LEARN_SOLVER2D2D_H
