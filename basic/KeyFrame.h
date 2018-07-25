//
// Created by pidan1231239 on 18-7-13.
//

#ifndef SLAM_LEARN_KEYFRAME_H
#define SLAM_LEARN_KEYFRAME_H

#include "common_include.h"
#include "MapPoint.h"
#include "Camera.h"
#include <opencv2/opencv.hpp>
#include <unordered_map>

namespace sky {

    class KeyFrame {
    public:
        typedef shared_ptr<KeyFrame> Ptr;
        SE3 Tcw;      // transform from world to camera
        Camera::Ptr camera;     // Pinhole RGBD Camera model
        Mat image;
        vector<cv::KeyPoint> keyPoints;
        Mat descriptors;
        unordered_map<int, MapPoint::Ptr> mapPoints;//在descriptors或keyPoints中的序号和对应的地图点

        KeyFrame(const Camera::Ptr camera, const Mat &image, cv::Ptr<cv::Feature2D> feature2D);

        Vector3d getCamCenterEigen() const;

        cv::Mat getTcwMatCV(int rtype);

        cv::Mat getTcw34MatCV(int rtype);

        cv::Mat getTwcMatCV(int rtype);


        template<typename T>
        cv::Matx<T, 1, 3> getAngleAxisWcMatxCV() {
            Sophus::AngleAxisd angleAxis(Tcw.so3().matrix());
            auto axis = angleAxis.angle() * angleAxis.axis();
            cv::Matx<T, 1, 3> angleAxisCV(axis[0], axis[1], axis[2]);
            return angleAxisCV;
        }

        template<typename T>
        void setTcw(cv::Matx<T, 2, 3> angleAxisAndTranslation) {
            Tcw.so3() = SO3(angleAxisAndTranslation(0, 0),
                            angleAxisAndTranslation(0, 1),
                            angleAxisAndTranslation(0, 2));
            Tcw.translation() = Vector3d(angleAxisAndTranslation(1, 0),
                                         angleAxisAndTranslation(1, 1),
                                         angleAxisAndTranslation(1, 2));
        }

        // check if a point is in this frame
        bool isInFrame(const Vector3d &pt_world);

        bool isInFrame(MapPoint::Ptr mapPoint);

        //计算帧到某坐标的距离
        float getDis2(Sophus::Vector3d &coor);

        //计算到某帧的距离
        float getDis2(KeyFrame::Ptr keyFrame);

        //计算到某地图点的距离
        float getDis2(MapPoint::Ptr mapPoint);


        //在descriptors或keyPoints中的序号
        bool hasMapPoint(int i) {
            return mapPoints.find(i) != mapPoints.end();
        }

        MapPoint::Ptr getMapPoint(int i) {
            if (hasMapPoint(i))
                return mapPoints[i];
            else {
                cerr << "MapPoint: getMapPoint failed! KeyPoint "
                     << i << " has no corresponding mapPoint" << endl;
                return nullptr;
            }
        }

        cv::Point2f getKeyPointCoor(int i) {
            return keyPoints[i].pt;
        }

        Mat getKeyPointDesciptor(int i){
            return descriptors.row(i);
        }

        void addMapPoint(int i, MapPoint::Ptr &mapPoint) {
            mapPoints[i] = mapPoint;
        }
    };

}


#endif //SLAM_LEARN_KEYFRAME_H
