//
// Created by pidan1231239 on 18-7-13.
//

#include "Solver2D2D.h"
#include <opencv2/cvv.hpp>
#include <opencv2/core/eigen.hpp>
#include "BA.h"

namespace sky {

    void Solver2D2D::solve(KeyFrame::Ptr &keyFrame1, KeyFrame::Ptr &keyFrame2, bool saveResult) {
        //重置中间变量
        inlierMask = Mat();

        this->keyFrame1 = keyFrame1;
        this->keyFrame2 = keyFrame2;

        match(keyFrame1->descriptors, keyFrame2->descriptors);
        solvePose(saveResult);
    }

    double Solver2D2D::getInlierRatio() {
        return (double) countNonZero(inlierMask) / getMatchesNum();
    }

    Map::Ptr Solver2D2D::triangulate() {
        return triangulater.triangulate(keyFrame1, keyFrame2, matches, inlierMask);
    }

    void Solver2D2D::solvePose(bool saveResult) {
        vector<cv::Point2f> matchPoints1, matchPoints2;
        for (auto match:matches) {
            matchPoints1.push_back(keyFrame1->getKeyPointCoor(match.queryIdx));
            matchPoints2.push_back(keyFrame2->getKeyPointCoor(match.trainIdx));
        }

        cout << "Solver2D2D: findEssentialMat... \n\t";
        Mat essentialMatrix;
        essentialMatrix = findEssentialMat(matchPoints1, matchPoints2,
                                           keyFrame2->camera->getFocalLength(),
                                           keyFrame2->camera->getPrincipalPoint(),
                                           cv::RANSAC, 0.999, 1.0, inlierMask);
#ifdef DEBUG
        int nPointsFindEssentialMat = countNonZero(inlierMask);
        cout << nPointsFindEssentialMat << " valid points, " <<
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
        cout << "Solver2D2D: recoverPose... \n\t";
        Mat R, t;
        recoverPose(essentialMatrix, matchPoints1, matchPoints2,
                    keyFrame2->camera->getKMatxCV(), R, t, inlierMask);
        if (saveResult) {
            Eigen::Matrix3d eigenR2;
            cv2eigen(R, eigenR2);
            keyFrame2->Tcw = SE3(
                    eigenR2,
                    Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0))
            );
        }

#ifdef DEBUG
        int nPointsRecoverPose = countNonZero(inlierMask);
        cout << nPointsRecoverPose << " valid points, " <<
             (float) nPointsRecoverPose * 100 / matchPoints1.size()
             << "% of " << matchPoints1.size() << " points are used" << endl;
/*        cout << "2D-2D frame2 R: " << R.size << endl << R << endl;
        cout << "2D-2D frame2 t: " << t.size << endl << t << endl;
        cout << "2D-2D frame2 SE3: " << endl << keyFrame2->Tcw << endl;
        cout << "2D-2D frame2 Tcw: " << endl << keyFrame2->getTcwMatCV() << endl << endl;
        cout << "2D-2D frame2 ProjMat: " << endl << keyFrame2->getTcw34MatCV() << endl << endl;*/
#endif
    }

}
