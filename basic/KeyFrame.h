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
#include "utility.h"

namespace sky {

    class KeyFrame {
    private:
        unordered_map<int, MapPoint::Ptr> mapPoints;//在descriptors或keyPoints中的序号和对应的地图点

    public:
        typedef shared_ptr<KeyFrame> Ptr;
        SE3 Tcw;      // transform from world to camera
        Camera::Ptr camera;     // Pinhole RGBD Camera model
        Mat image;
        vector<cv::KeyPoint> keyPoints;
        Mat descriptors;


        KeyFrame(const Camera::Ptr &camera, const Mat &image, cv::Ptr<cv::Feature2D> feature2D);

        Vector3d getCamCenterEigen() const;

        cv::Mat getTcwMatCV(int rtype) const;

        cv::Mat getTcw34MatCV(int rtype) const;

        cv::Mat getTwcMatCV(int rtype) const;


        template<typename T>
        cv::Matx<T, 1, 3> getAngleAxisWcMatxCV() const {
            Sophus::AngleAxisd angleAxis(Tcw.so3().matrix());
            auto axis = angleAxis.angle() * angleAxis.axis();
            cv::Matx<T, 1, 3> angleAxisCV(axis[0], axis[1], axis[2]);
            return angleAxisCV;
        }

        template<typename T>
        void setTcw(const cv::Matx<T, 2, 3> &angleAxisAndTranslation) {
            Tcw.so3() = SO3(angleAxisAndTranslation(0, 0),
                            angleAxisAndTranslation(0, 1),
                            angleAxisAndTranslation(0, 2));
            Tcw.translation() = Vector3d(angleAxisAndTranslation(1, 0),
                                         angleAxisAndTranslation(1, 1),
                                         angleAxisAndTranslation(1, 2));
        }

        //将3D点投影到帧，如果在帧里，返回真
        bool proj2frame(const Vector3d &pt_world, Vector2d &pixelColRow) const;

        inline bool proj2frame(const MapPoint::Ptr &mapPoint, Vector2d &pixelColRow) const {
            return proj2frame(mapPoint->pos, pixelColRow);
        }

        inline bool proj2frame(const MapPoint::Ptr &mapPoint, cv::Point2d &pixelColRow) const {
            Vector2d pixelVec;
            bool r = proj2frame(mapPoint->pos, pixelVec);
            pixelColRow.x = pixelVec[0];
            pixelColRow.y = pixelVec[1];
            return r;
        }

        // check if a point is in this frame
        inline bool isInFrame(const Vector3d &pt_world) const {
            Vector2d pixelColRow;
            return proj2frame(pt_world, pixelColRow);
        }

        inline bool isInFrame(const MapPoint::Ptr &mapPoint) const {
            return isInFrame(mapPoint->pos);
        }


        //计算帧到某坐标的距离
        float getDis2(const Sophus::Vector3d &coor) const;

        inline float getDis2(const KeyFrame::Ptr &keyFrame) const {
            return getDis2(keyFrame->Tcw.translation());
        }

        inline float getDis2(const MapPoint::Ptr &mapPoint) const {
            return getDis2(mapPoint->pos);
        }


        //在descriptors或keyPoints中的序号
        inline bool hasMapPoint(int i) const {
            return mapHas(mapPoints, i);
        }

        MapPoint::Ptr getMapPoint(int i) const;

        bool setMapPoint(int i, MapPoint::Ptr &mapPoint);

        inline cv::Point2f getKeyPointCoor(int i) const {
            return keyPoints[i].pt;
        }

        inline Mat getKeyPointDesciptor(int i) const {
            return descriptors.row(i);
        }

        inline void addMapPoint(int i, MapPoint::Ptr &mapPoint) {
            mapPoints[i] = mapPoint;
        }
    };

}


#endif //SLAM_LEARN_KEYFRAME_H
