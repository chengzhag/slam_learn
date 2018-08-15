//
// Created by pidan1231239 on 18-7-15.
//

#include "Solver3D2D.h"
#include <opencv2/cvv.hpp>
#include "BA.h"
#include "basic.h"
#include "utility.h"

namespace sky {

    bool Solver3D2D::solve(const Map::Ptr &map, const KeyFrame::Ptr &keyFrame2) {
        //重置中间变量
        points3D.clear();
        pointsCandi.clear();
        descriptorsMap = Mat();
        indexInliers = Mat();
        this->map = map;

        this->keyFrame2 = keyFrame2;
        auto keyFrame1 = map->getLastFrame();

        for (MapPoint::Ptr &point:map->mapPoints) {

            if (isInFrame(point->pos, keyFrame1)
                && disBetween(keyFrame1, point) <= max3Ddis) {

                Vector3d n = keyFrame1->getCamCenterEigen() - point->pos;
                n.normalize();
                auto angle = acos(n.transpose() * (point->norm / point->norm.norm()));
                //printVariable(angle);
                if (angle > M_PI / 6)
                    continue;

                points3D.push_back(point->getPosPoint3_CV<float>());
                pointsCandi.push_back(point);
                descriptorsMap.push_back(point->descriptor);
            }
        }

#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "Solver3D2D: Found " << points3D.size()
             << " points in the last keyFrame. " << endl;
#endif
        if (points3D.size() < min3Dnum) {
#ifdef DEBUG
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "Solver3D2D: Failed! " << points3D.size()
                 << " points in the last frame is less than min3Dnum: " << min3Dnum << endl;
#endif
            return false;
        }

        matcher.match(descriptorsMap, keyFrame2->descriptors);
        if (matcher.matches.size() < minInlierNum) {
#ifdef DEBUG
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "Solver3D2D: Failed! matchesNum "
                 << matcher.getMatchesNum() << " is less than minInlierNum " << minInlierNum << endl;
#endif
            return false;
        }
        solvePose();

        if (inlierNum < minInlierNum) {
#ifdef DEBUG
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "Solver3D2D: Failed! inlierNum "
                 << inlierNum << " is less than minInlierNum " << minInlierNum << endl;
#endif
            return false;
        }
        if (inlierRatio < minInlierRatio) {
#ifdef DEBUG
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "Solver3D2D: Failed! inlierRatio "
                 << inlierRatio << " is less than minInlierRatio " << minInlierRatio << endl;
#endif
            return false;
        }

        //建立BA用局部地图
        Map::Ptr mapLastFrame(new Map);//用于最后帧BA的地图
        mapLastFrame->addFrame(keyFrame2);

        //添加观测帧
        for (int i = 0; i < indexInliers.rows; ++i) {
            auto iInlier = indexInliers.at<int>(i);
            MapPoint::Ptr mapPoint(new MapPoint(*pointsCandi[matcher.matches[iInlier].queryIdx]));
            mapPoint->addFrame(keyFrame2, matcher.matches[iInlier].trainIdx, false);
            mapLastFrame->addMapPoint(mapPoint);
        }
        BA ba({BA::Mode_Fix_Points, BA::Mode_Fix_Intrinsic});
        ba.loadMap(mapLastFrame);
        ba.ba();
        ba.writeMap();

        return true;
    }

    void Solver3D2D::solvePose() {
        //解PnP得相机位姿
        vector<cv::Point2f> points2DPnP;
        vector<cv::Point3f> points3DPnP;
        for (auto match:matcher.matches) {
            points2DPnP.push_back(keyFrame2->getKeyPointCoor(match.trainIdx));
            points3DPnP.push_back(points3D[match.queryIdx]);
        }
        Mat r, t;
        solvePnPRansac(points3DPnP, points2DPnP, keyFrame2->camera->getKMatxCV(),
                       cv::noArray(), r, t, false, 100, 8.0, 0.99,
                       indexInliers);
        Mat R;
        cv::Rodrigues(r, R);

        keyFrame2->Tcw = SE3(
                SO3(r.at<double>(0, 0), r.at<double>(1, 0), r.at<double>(2, 0)),
                Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0))
        );

        inlierNum = indexInliers.rows;
        inlierRatio = (double) inlierNum / matcher.getMatchesNum();
#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "Solver3D2D: Pose solved. "
             << inlierNum << " valid points of " << points2DPnP.size()
             << " , " << (float) inlierRatio * 100 << "% are used. " << endl;
#endif
    }

    void Solver3D2D::addFrame2inliers(bool add2mapPoints) {
        for (int i = 0; i < indexInliers.rows; ++i) {
            auto iInlier = indexInliers.at<int>(i);
/*            cv::Point2d reprojCoor;
            proj2frame(pointsCandi[matches[iInlier].queryIdx], keyFrame2, reprojCoor);
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: "   << disBetween(reprojCoor, rawCoor) << endl;*/
            auto mapPoint = pointsCandi[matcher.matches[iInlier].queryIdx];
            if (add2mapPoints)
                map->addObservation(keyFrame2, mapPoint, matcher.matches[iInlier].trainIdx);
            else {
                keyFrame2->addMapPoint(matcher.matches[iInlier].trainIdx, mapPoint);
            }
        }
#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "Solver3D2D: addFrame2inliers done. "
             << "Added currFrame as observedFrame to " << indexInliers.rows << " of " << pointsCandi.size()
             << " old index2mapPoints. " << endl;
#endif
    }

}
