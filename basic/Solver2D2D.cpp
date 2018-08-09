//
// Created by pidan1231239 on 18-7-13.
//

#include "Solver2D2D.h"
#include <opencv2/cvv.hpp>
#include <opencv2/core/eigen.hpp>
#include "BA.h"

namespace sky {

    bool Solver2D2D::solve(const KeyFrame::Ptr &keyFrame1, const KeyFrame::Ptr &keyFrame2, bool saveResult) {
        //重置中间变量
        inlierMask = Mat();

        this->keyFrame1 = keyFrame1;
        this->keyFrame2 = keyFrame2;

        matcher.match(keyFrame1->descriptors, keyFrame2->descriptors);
        if (matcher.getMatchesNum() < minInlierNum) {
#ifdef DEBUG
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "Solver2D2D: Failed! matchesNum "
                 << matcher.getMatchesNum() << " is less than minInlierNum " << minInlierNum << endl;
#endif
            return false;
        }
        solvePose(saveResult);

        if (inlierNum < minInlierNum) {
#ifdef DEBUG
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "Solver2D2D: Failed! inlierNum "
                 << inlierNum << " is less than minInlierNum " << minInlierNum << endl;
#endif
            return false;
        }
        if (inlierRatio < minInlierRatio) {
#ifdef DEBUG
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "Solver2D2D: Failed! inlierRatio "
                 << inlierRatio << " is less than minInlierRatio " << minInlierRatio << endl;
#endif
            return false;
        }
        return true;
    }

    void Solver2D2D::viewInliersInCVV() const {
        //可视化用于解对极约束的点
#ifdef CVVISUAL_DEBUGMODE
        vector<cv::DMatch> inlierMatches;
        vector<cv::KeyPoint> inlierKeyPoints1, inlierKeyPoints2;
        for (int i = 0; i < matcher.getMatchesNum(); ++i) {
            if (!inlierMask.at<uint8_t>(i, 0))
                continue;
            inlierMatches.push_back(matcher.matches[i]);
            inlierMatches.back().trainIdx = inlierKeyPoints1.size();
            inlierMatches.back().queryIdx = inlierKeyPoints2.size();
            inlierKeyPoints1.push_back(keyFrame1->keyPoints[matcher.matches[i].queryIdx]);
            inlierKeyPoints2.push_back(keyFrame2->keyPoints[matcher.matches[i].trainIdx]);
        }
        cvv::debugDMatch(keyFrame1->image, inlierKeyPoints1, keyFrame2->image, inlierKeyPoints2, inlierMatches,
                         CVVISUAL_LOCATION,
                         "match used in triangulation");

#endif
    }

    Map::Ptr Solver2D2D::triangulate() {
        return triangulater.triangulate(keyFrame1, keyFrame2, matcher.matches, inlierMask);
    }

    void Solver2D2D::solvePose(bool saveResult) {
        vector<cv::Point2f> matchPoints1, matchPoints2;
        for (auto match:matcher.matches) {
            matchPoints1.push_back(keyFrame1->getKeyPointCoor(match.queryIdx));
            matchPoints2.push_back(keyFrame2->getKeyPointCoor(match.trainIdx));
        }

        Mat essentialMatrix;
        essentialMatrix = findEssentialMat(matchPoints1, matchPoints2,
                                           keyFrame2->camera->getFocalLength(),
                                           keyFrame2->camera->getPrincipalPoint(),
                                           cv::RANSAC, 0.999, 1.0, inlierMask);

        //解frame2的R、t并计算se3
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

        inlierNum = countNonZero(inlierMask);
        inlierRatio = (double) inlierNum / matcher.getMatchesNum();
#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "Solver2D2D: Pose solved. " << inlierNum
             << " valid points of "
             << matchPoints1.size()
             << " , " << inlierRatio * 100 << "% "
             << " are used" << endl;
#endif
    }

}
