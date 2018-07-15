//
// Created by pidan1231239 on 18-7-15.
//

#ifndef SLAM_LEARN_SOLVER3D2D_H
#define SLAM_LEARN_SOLVER3D2D_H

#include "common_include.h"
#include "Map.h"
#include "Frame.h"
#include <opencv2/opencv.hpp>
#include <opencv2/cvv.hpp>
#include "BA.h"
#include "KeyFrame.h"
#include <algorithm>
#include "Matcher.h"

namespace sky {

    using namespace cv;

    class Solver3D2D : protected Matcher {
    private:

    public:
        typedef shared_ptr<Solver3D2D> Ptr;
        Mat descriptorsMap;
        vector<Point3f> points3D;
        KeyFrame::Ptr keyFrame2;

        Solver3D2D(cv::Ptr<DescriptorMatcher> matcher,
                   double disThresRatio = 5, double disThresMin = 200) :
                Matcher(matcher, disThresRatio, disThresMin) {}

        void solve(Map::Ptr map, KeyFrame::Ptr keyFrame2) {
            this->keyFrame2 = keyFrame2;
#ifdef DEBUG
            cout << "Solver3D2D: finding mapPoints in the last frame... " << endl;
#endif
            auto keyFrame1 = map->keyFrames.back();
            Map::Ptr mapLastFrame(new Map);//用于最后帧BA的地图
            mapLastFrame->addFrame(keyFrame2);
            //提取地图的特征点matches
            Mat descriptorsMap;
            for (MapPoint::Ptr &point:map->mapPoints) {
                if (keyFrame1->isInFrame(point->pos)) {
                    points3D.push_back(point->getPosPoint3_CV<float>());
                    descriptorsMap.push_back(point->descriptor);
                    mapLastFrame->addMapPoint(point);
                }
            }
            //TODO:随机选取一定数量的地图点
#ifdef DEBUG
            cout << "\t" << points3D.size() << " 3D points found" << endl;
#endif
            match(descriptorsMap, keyFrame2->descriptors);
            solvePose();

            //BA当前相机位姿，固定点云和其他帧不动
            BA ba;
            ba(mapLastFrame, {BA::Mode_Fix_Points, BA::Mode_Fix_Intrinsic});
        }

    private:
        void solvePose() {
#ifdef DEBUG
            cout << "Solver3D2D: solvePose..." << endl;
#endif
            //解PnP得相机位姿
            vector<Point2f> points2DPnP;
            vector<Point3f> points3DPnP;
            for (auto match:matches) {
                points2DPnP.push_back(keyFrame2->keyPoints[match.trainIdx].pt);
                points3DPnP.push_back(points3D[match.queryIdx]);
            }
            Mat r, t, indexInliers;
            solvePnPRansac(points3DPnP, points2DPnP, keyFrame2->camera->getKMatxCV(),
                           cv::noArray(), r, t, false, 100, 8.0, 0.99,
                           indexInliers);
            Mat R;
            cv::Rodrigues(r, R);

            keyFrame2->Tcw = SE3(
                    SO3(r.at<double>(0, 0), r.at<double>(1, 0), r.at<double>(2, 0)),
                    Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0))
            );

#ifdef DEBUG
            cout << "\tsolvePnPRansac: " << indexInliers.rows << " valid points, " <<
                 (float) indexInliers.rows * 100 / points2DPnP.size()
                 << "% of " << points2DPnP.size() << " points are used" << endl;
/*        cout << "2D-2D frame2 R: " << R.size << endl << R << endl;
        cout << "2D-2D frame2 t: " << t.size << endl << t << endl;
        cout << "2D-2D frame2 SE3: " << endl << keyFrame2->frame->Tcw << endl;
        cout << "2D-2D frame2 Tcw: " << endl << keyFrame2->frame->getTcwMatCV() << endl << endl;
        cout << "2D-2D frame2 ProjMat: " << endl << keyFrame2->frame->getTcw34MatCV() << endl << endl;*/
#endif
        }
    };

}


#endif //SLAM_LEARN_SOLVER3D2D_H
