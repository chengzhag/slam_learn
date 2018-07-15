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

        Solver2D2D(cv::Ptr<DescriptorMatcher> matcher,
                   double disThresRatio = 5, double disThresMin = 200) :
                matcher(matcher),
                disThresRatio(disThresRatio), disThresMin(disThresMin) {}

        void solve(KeyFrame::Ptr &keyFrame1, KeyFrame::Ptr &keyFrame2) {
            this->keyFrame1 = keyFrame1;
            this->keyFrame2 = keyFrame2;

            match();
            filtMatches();
            solvePose();
        }

        double getInlierRatio() {
            return (double) countNonZero(inlierMask) / matches.size();
        }

        size_t getMatchesNum() {
            return matches.size();
        }

        Map::Ptr triangulate() {
            Map::Ptr map(new Map);
            map->addFrame(keyFrame1->frame);
            map->addFrame(keyFrame2->frame);

            vector<Point2f> matchPointsNorm1, matchPointsNorm2;
            matchPointsNorm1.reserve(matches.size());
            matchPointsNorm2.reserve(matches.size());
            for (auto &match:matches) {
                matchPointsNorm1.push_back(
                        keyFrame1->frame->camera->pixel2normal(keyFrame1->keyPoints[match.queryIdx].pt));
                matchPointsNorm2.push_back(
                        keyFrame2->frame->camera->pixel2normal(keyFrame2->keyPoints[match.trainIdx].pt));
            }
            Mat points4D;
            triangulatePoints(keyFrame1->frame->getTcw34MatCV(CV_32F), keyFrame2->frame->getTcw34MatCV(CV_32F),
                              matchPointsNorm1, matchPointsNorm2, points4D);

            //转换齐次坐标点，保存到Map，并做局部BA
            convAndAddMappoints(map, inlierMask, points4D, matches);

            BA ba;
            ba(map, {BA::Mode_Fix_Points, BA::Mode_Fix_Intrinsic, BA::Mode_Fix_First_Frame});
            return map;
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

        void solvePose() {
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

        Map::Ptr convAndAddMappoints(Map::Ptr map, const Mat &inlierMask,
                                     const Mat &points4D, const vector<DMatch> &matches) {
#ifdef DEBUG
            cout << "convAndAddMappoints: " << endl;
#endif

#ifdef DEBUG
            int numOldMappoints = map->mapPoints.size();
#endif
            for (int i = 0; i < points4D.cols; ++i) {
                MapPoint::Ptr mapPoint;

                //如果是outlier，跳过
                if (!inlierMask.empty() && !inlierMask.at<uint8_t>(i, 0))
                    continue;

                int iMapPoint1 = matches[i].queryIdx;
                int iMapPoint2 = matches[i].trainIdx;

                //获取描述子
                Mat descriptor = keyFrame2->descriptors.row(iMapPoint2);

                //如果是上一帧加到地图中的点，更新描述子、加入观测帧后跳过

                if (keyFrame1->hasMapPoint(iMapPoint1)) {
                    mapPoint = keyFrame1->getMapPoint(iMapPoint1);
                    //更新描述子
                    mapPoint->descriptor = descriptor;
                    //加入观测帧
                    mapPoint->addObervedFrame(
                            keyFrame2->frame, keyFrame2->getKeyPointCoor(iMapPoint2));
                    //记录当前帧加入地图的mapPoint和特征点下标
                    keyFrame2->addMapPoint(iMapPoint2, mapPoint);

                } else {

                    //转换齐次坐标
                    Mat x = points4D.col(i);

                    //向地图增加点
                    //获取颜色
                    Vec3b rgb;
                    if (keyFrame2->image.type() == CV_8UC3) {
                        rgb = keyFrame2->image.at<Vec3b>(keyFrame2->getKeyPointCoor(iMapPoint2));
                        swap(rgb[0], rgb[2]);
                    } else if (keyFrame2->image.type() == CV_8UC1) {
                        cvtColor(keyFrame2->image.at<uint8_t>(keyFrame2->getKeyPointCoor(iMapPoint2)),
                                 rgb,
                                 COLOR_GRAY2RGB);
                    }


                    if (x.type() == CV_32FC1) {
                        x /= x.at<float>(3, 0); // 归一化
                        mapPoint = MapPoint::Ptr(new MapPoint(Vector3d(x.at<float>(0, 0),
                                                                       x.at<float>(1, 0),
                                                                       x.at<float>(2, 0)),
                                                              descriptor, rgb
                        ));
                    } else if (x.type() == CV_64FC1) {
                        x /= x.at<double>(3, 0);
                        mapPoint = MapPoint::Ptr(new MapPoint(Vector3d(x.at<double>(0, 0),
                                                                       x.at<double>(1, 0),
                                                                       x.at<double>(2, 0)),
                                                              descriptor, rgb
                        ));
                    }

                    //记录当前帧加入地图的mapPoint和特征点下标
                    keyFrame2->addMapPoint(iMapPoint2, mapPoint);

/*#ifdef DEBUG
            if (i < 5)
                cout << mapPoint->pos << endl << endl;
#endif*/
                    mapPoint->addObervedFrame(keyFrame1->frame, keyFrame1->getKeyPointCoor(iMapPoint1));
                    mapPoint->addObervedFrame(keyFrame2->frame, keyFrame2->getKeyPointCoor(iMapPoint2));
                    map->addMapPoint(mapPoint);
                }


            }
#ifdef DEBUG
            cout << "\t" << map->mapPoints.size() - numOldMappoints << " new 3D points added to the map" << endl
                 << "\t" << map->mapPoints.size() << " in total" << endl;
#endif

        }
    };

}


#endif //SLAM_LEARN_SOLVER2D2D_H
