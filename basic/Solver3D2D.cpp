//
// Created by pidan1231239 on 18-7-15.
//

#include "Solver3D2D.h"
#include <opencv2/cvv.hpp>
#include "BA.h"
#include "basic.h"

namespace sky {

    bool Solver3D2D::solve(const Map::Ptr &map, const KeyFrame::Ptr &keyFrame2) {
        //重置中间变量
        points3D.clear();
        descriptorsMap = Mat();
        indexInliers = Mat();
        this->map = map;


        this->keyFrame2 = keyFrame2;
#ifdef DEBUG
        cout << "Solver3D2D: finding mapPoints in the last frame... ";
#endif
        auto keyFrame1 = map->getLastFrame();

        Mat descriptorsMap;

        for (MapPoint::Ptr &point:map->mapPoints) {
            if (isInFrame(point->pos, keyFrame1)
                && disBetween(keyFrame1, point) <= max3Ddis) {
                points3D.push_back(point->getPosPoint3_CV<float>());
                descriptorsMap.push_back(point->descriptor);
            }
        }

/*        for (MapPoint::Ptr &point:map->mapPoints) {
            if (keyFrame1->isInFrame(point->pos)
                && keyFrame1->getDis2(point) <= max3Ddis) {
                pointsInView.push_back(point);
            }
        }
        //随机选取一定数量的地图点,提取地图的特征点matches
        random_shuffle(pointsInView.begin(), pointsInView.end());
        int i = 0;
        for (MapPoint::Ptr &point:pointsInView) {
            if (i >= max3Dnum)
                break;
            points3D.push_back(point->getPosPoint3_CV<float>());
            descriptorsMap.push_back(point->descriptor);
            ++i;
        }*/

#ifdef DEBUG
        cout << points3D.size() << " points found" << endl;
#endif
        if (points3D.size() < min3Dnum) {
#ifdef DEBUG
            cout << "Failed! " << points3D.size() << " is less than min3Dnum: " << min3Dnum << endl;
#endif
            return false;
        }

        match(descriptorsMap, keyFrame2->descriptors);
        if (matches.size() < minInlierNum) {
#ifdef DEBUG
            cout << "Solver3D2D: Failed! matchesNum " << matches.size()
                 << " is less than minInlierNum " << minInlierNum << endl;
#endif
            return false;
        }
        solvePose();

        if (inlierNum < minInlierNum) {
#ifdef DEBUG
            cout << "Solver3D2D: Failed! inlierNum " << inlierNum
                 << " is less than minInlierNum " << minInlierNum << endl;
#endif
            return false;
        }
        if (inlierRatio < minInlierRatio) {
#ifdef DEBUG
            cout << "Solver3D2D: Failed! inlierRatio " << inlierRatio
                 << " is less than minInlierRatio " << minInlierRatio << endl;
#endif
            return false;
        }

        //建立BA用局部地图
        //TODO:以下建图没有考虑到当前帧可能不是KeyFrame，导致MapPoint的observedFrames存有非KeyFrame
/*        Map::Ptr mapLastFrame(new Map);//用于最后帧BA的地图
        mapLastFrame->addFrame(keyFrame2);
        //cout << indexInliers << endl;
        //cout << indexInliers.type() << endl;
        //建立matches从3D地图点到2D特征点的对应
        unordered_map<int, int> i3Dto2D;
        for (auto &match:matches) {
            i3Dto2D[match.queryIdx] = match.trainIdx;
        }
        //添加地图点
        for (int i = 0; i < indexInliers.rows; ++i) {
            //将当前帧添加到地图点的观测帧中
            auto indexInlier = indexInliers.at<int>(i);
            auto mapPointInlier = pointsInView[indexInlier];
            //cout << "\tadding mapPoint " << indexInlier << " to BA map" << endl;
            auto indexKeyPoint = i3Dto2D[indexInlier];
            //cout << "\tadding observedFrame to mapPoint, corresponding to keyPoint "
            //        << indexKeyPoint << " of " << keyFrame2->keyPoints.size()
            //     << endl;
            mapPointInlier->addObservedFrame(keyFrame2, keyFrame2->getKeyPointCoor(indexKeyPoint));
            mapLastFrame->addMapPoint(mapPointInlier);
        }

        //BA当前相机位姿，固定点云和其他帧不动
        BA ba;
        ba(mapLastFrame, {BA::Mode_Fix_Points, BA::Mode_Fix_Intrinsic});*/
        return true;
    }

    void Solver3D2D::solvePose() {
#ifdef DEBUG
        cout << "Solver3D2D: solvePose... ";
#endif
        //解PnP得相机位姿
        vector<cv::Point2f> points2DPnP;
        vector<cv::Point3f> points3DPnP;
        for (auto match:matches) {
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
        keyFrame2->inlierPnPnum = inlierNum;
        inlierRatio = (double) inlierNum / getMatchesNum();
#ifdef DEBUG
        cout << inlierNum << " valid points of " << points2DPnP.size()
             << " , " << (float) inlierRatio * 100 << "% "
             << "are used" << endl;
/*        cout << "2D-2D frame2 R: " << R.size << endl << R << endl;
        cout << "2D-2D frame2 t: " << t.size << endl << t << endl;
        cout << "2D-2D frame2 SE3: " << endl << keyFrame2->frame->Tcw << endl;
        cout << "2D-2D frame2 Tcw: " << endl << keyFrame2->frame->getTcwMatCV() << endl << endl;
        cout << "2D-2D frame2 ProjMat: " << endl << keyFrame2->frame->getTcw34MatCV() << endl << endl;*/
#endif
    }

}
