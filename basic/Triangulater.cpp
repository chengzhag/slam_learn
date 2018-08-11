//
// Created by pidan1231239 on 18-7-15.
//

#include "Triangulater.h"
#include <opencv2/cvv.hpp>
#include "BA.h"
#include <algorithm>
#include "utility.h"
#include "basic.h"

namespace sky {

    Map::Ptr
    Triangulater::triangulate(const KeyFrame::Ptr &keyFrame1,
                              const KeyFrame::Ptr &keyFrame2,
                              const vector<cv::DMatch> &matches,
                              Mat inlierMask) {
        this->keyFrame1 = keyFrame1;
        this->keyFrame2 = keyFrame2;

        map = Map::Ptr(new Map);
        map->addFrame(keyFrame1);
        map->addFrame(keyFrame2);

        vector<cv::Point2f> matchPointsNorm1, matchPointsNorm2;
        matchPointsNorm1.reserve(matches.size());
        matchPointsNorm2.reserve(matches.size());
        for (auto &match:matches) {
            matchPointsNorm1.push_back(
                    keyFrame1->camera->pixel2normal(keyFrame1->getKeyPointCoor(match.queryIdx)));
            matchPointsNorm2.push_back(
                    keyFrame2->camera->pixel2normal(keyFrame2->getKeyPointCoor(match.trainIdx)));
        }
        Mat points4D;
        triangulatePoints(keyFrame1->getTcw34MatCV(CV_32F), keyFrame2->getTcw34MatCV(CV_32F),
                          matchPointsNorm1, matchPointsNorm2, points4D);

        //转换齐次坐标点，保存到Map，并做局部BA
        convAndAddMappoints(map, inlierMask, points4D, matches);

        return map;
    }

    void Triangulater::viewReprojInCVV() const {
#ifdef CVVISUAL_DEBUGMODE
        if (!map)
            return;
        map->viewFrameProjInCVV(keyFrame2, "Triangulater: Proj triangulated points to crrFrame");
#endif
    }


    void Triangulater::convAndAddMappoints(const Map::Ptr &map, const Mat &inlierMask,
                                           const Mat &points4D, const vector<cv::DMatch> &matches) {
        int numOldMapPoint = 0;
        for (int i = 0; i < points4D.cols; ++i) {
            MapPoint::Ptr mapPoint;

            //如果是outlier，跳过
            if (!inlierMask.empty() && !inlierMask.at<uint8_t>(i, 0))
                continue;

            int iMapPoint1 = matches[i].queryIdx;
            int iMapPoint2 = matches[i].trainIdx;

            //获取描述子
            Mat descriptor = keyFrame2->getKeyPointDesciptor(iMapPoint2);

            //如果是上一帧加到地图中的点，更新描述子、加入观测帧后跳过
            if (keyFrame1->hasMapPoint(iMapPoint1)) {
                mapPoint = keyFrame1->getMapPoint(iMapPoint1);

                //创建临时mapPoint拷贝并加入关键帧，再作isGoodPoint判断
                MapPoint::Ptr testMapPoint(new MapPoint(*mapPoint));
                testMapPoint->addFrame(keyFrame2, iMapPoint2);

                if (!isGoodPoint(testMapPoint))
                    continue;
                ++numOldMapPoint;

                //保存添加的观测帧和更新的norm
                *mapPoint = *testMapPoint;

                //更新描述子
                //mapPoint->descriptor = descriptor;

                //记录当前帧加入地图的mapPoint和特征点下标
                keyFrame2->addMapPoint(iMapPoint2, mapPoint);

            } else {

                //转换齐次坐标
                Mat x = points4D.col(i);

                //向地图增加点
                //获取颜色
                cv::Vec3b rgb;
                if (keyFrame2->image.type() == CV_8UC3) {
                    rgb = keyFrame2->image.at<cv::Vec3b>(keyFrame2->getKeyPointCoor(iMapPoint2));
                    swap(rgb[0], rgb[2]);
                } else if (keyFrame2->image.type() == CV_8UC1) {
                    cvtColor(keyFrame2->image.at<uint8_t>(keyFrame2->getKeyPointCoor(iMapPoint2)),
                             rgb,
                             cv::COLOR_GRAY2RGB);
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

                mapPoint->addFrame(keyFrame1, iMapPoint1);
                mapPoint->addFrame(keyFrame2, iMapPoint2);

                if (!isGoodPoint(mapPoint))
                    continue;

                map->addMapPoint(mapPoint);

                //记录当前帧加入地图的mapPoint和特征点下标
                keyFrame2->addMapPoint(iMapPoint2, mapPoint);
            }

        }
#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: "
             << "Triangulater: convAndAddMappoints done. "
             << numOldMapPoint << " points are old. "
             << map->mapPoints.size() << " of " << points4D.cols - numOldMapPoint << " new points are good. " << endl;
#endif

/*#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: "   << "Triangulater: Showing rawPos and projPos of index2mapPoints... " << endl;
        for (auto &mapPoint:map->index2mapPoints) {
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: "  .width(6);

            auto &rawPos1 = mapPoint->frame2indexs[keyFrame1];
            cv::Point2d projPos1;
            keyFrame1->proj2frame(mapPoint, projPos1);
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: "   << setiosflags(ios::fixed) << setprecision(2)
                 << "rawPos1: " << rawPos1 << "\tprojPos1: " << projPos1 << "\tdis: " << point2dis(rawPos1, projPos1);

            auto &rawPos2 = mapPoint->frame2indexs[keyFrame1];
            cv::Point2d projPos2;
            keyFrame2->proj2frame(mapPoint, projPos2);
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: "   << setiosflags(ios::fixed) << setprecision(2)
                 << "\trawPos2: " << rawPos2 << "\tprojPos2: " << projPos2 << "\tdis: " << disBetween(rawPos2, projPos2)
                 << endl;
        }
#endif*/
    }

    bool Triangulater::isGoodPoint(const MapPoint::Ptr &mapPoint) const {
        //检查是否在距离范围内
        auto dis2keyFrame2 = disBetween(keyFrame2, mapPoint);
        auto disB12 = disBetween(keyFrame2, keyFrame1);
        //printVariable(dis2keyFrame2);
        if (dis2keyFrame2 > maxDisRatio * disB12
            || dis2keyFrame2 < minDisRatio * disB12)
            return false;


        //检查是否在帧的视野内
        if (!isInFrame(mapPoint, keyFrame1))
            return false;
        if (!isInFrame(mapPoint, keyFrame2))
            return false;


        //测试重投影误差
/*        cv::Point2f rawPos1;
        mapPoint->getPixelCoor(keyFrame1, rawPos1);
        cv::Point2d projPos1;
        if (!proj2frame(mapPoint, keyFrame1, projPos1))
            return false;
        //printVariable(disBetween<float>(rawPos1, projPos1));
        if (disBetween<float>(rawPos1, projPos1) > maxReprojErr)
            return false;

        cv::Point2f rawPos2;
        mapPoint->getPixelCoor(keyFrame2, rawPos2);
        cv::Point2d projPos2;
        if (!proj2frame(mapPoint, keyFrame2, projPos2))
            return false;
        //printVariable(disBetween<float>(rawPos2, projPos2));
        if (disBetween<float>(rawPos2, projPos2) > maxReprojErr)
            return false;*/


        return true;
    }

}
