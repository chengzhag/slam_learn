//
// Created by pidan1231239 on 18-7-15.
//

#ifndef SLAM_LEARN_TRIANGULATER_H
#define SLAM_LEARN_TRIANGULATER_H

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

    class Triangulater {
    public:
        KeyFrame::Ptr keyFrame1, keyFrame2;

        Map::Ptr triangulate(KeyFrame::Ptr keyFrame1,KeyFrame::Ptr keyFrame2,vector<DMatch> &matches,Mat &inlierMask) {
#ifdef DEBUG
            cout << "Triangulater: triangulatePoints... " << endl;
#endif
            this->keyFrame1=keyFrame1;
            this->keyFrame2=keyFrame2;

            Map::Ptr map(new Map);
            map->addFrame(keyFrame1);
            map->addFrame(keyFrame2);

            vector<Point2f> matchPointsNorm1, matchPointsNorm2;
            matchPointsNorm1.reserve(matches.size());
            matchPointsNorm2.reserve(matches.size());
            for (auto &match:matches) {
                matchPointsNorm1.push_back(
                        keyFrame1->camera->pixel2normal(keyFrame1->keyPoints[match.queryIdx].pt));
                matchPointsNorm2.push_back(
                        keyFrame2->camera->pixel2normal(keyFrame2->keyPoints[match.trainIdx].pt));
            }
            Mat points4D;
            triangulatePoints(keyFrame1->getTcw34MatCV(CV_32F), keyFrame2->getTcw34MatCV(CV_32F),
                              matchPointsNorm1, matchPointsNorm2, points4D);

            //转换齐次坐标点，保存到Map，并做局部BA
            convAndAddMappoints(map, inlierMask, points4D, matches);

            BA ba;
            ba(map, {BA::Mode_Fix_Points, BA::Mode_Fix_Intrinsic, BA::Mode_Fix_First_Frame});
            return map;
        }

    private:
        void convAndAddMappoints(Map::Ptr map, const Mat &inlierMask,
                                     const Mat &points4D, const vector<DMatch> &matches) {
#ifdef DEBUG
            cout << "Triangulater: convAndAddMappoints... " << endl;
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
                            keyFrame2, keyFrame2->getKeyPointCoor(iMapPoint2));
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
                    mapPoint->addObervedFrame(keyFrame1, keyFrame1->getKeyPointCoor(iMapPoint1));
                    mapPoint->addObervedFrame(keyFrame2, keyFrame2->getKeyPointCoor(iMapPoint2));
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

#endif //SLAM_LEARN_TRIANGULATER_H
