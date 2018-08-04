//
// Created by pidan1231239 on 18-7-13.
//

#ifndef SLAM_LEARN_KEYFRAME_H
#define SLAM_LEARN_KEYFRAME_H

#include "common_include.h"
#include "Camera.h"
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include "utility.h"

namespace sky {

    class Map;

    typedef shared_ptr<Map> MapPtr;

    class MapPoint;

    typedef shared_ptr<MapPoint> MapPointPtr;

    class KeyFrame {
    private:
        unordered_map<int, MapPointPtr> mapPoints;//在descriptors或keyPoints中的序号和对应的地图点

    public:
        typedef shared_ptr<KeyFrame> Ptr;
        SE3 Tcw;      // transform from world to camera
        Camera::Ptr camera;     // Pinhole RGBD Camera model
        Mat image;
        vector<cv::KeyPoint> keyPoints;
        Mat descriptors;
        int inlierPnPnum;


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

        //在descriptors或keyPoints中的序号
        inline bool hasMapPoint(int i) const {
            return mapHas(mapPoints, i);
        }

        MapPointPtr getMapPoint(int i) const;

        bool setMapPoint(int i, MapPointPtr &mapPoint);

        inline cv::Point2f getKeyPointCoor(int i) const {
            return keyPoints[i].pt;
        }

        inline Mat getKeyPointDesciptor(int i) const {
            return descriptors.row(i);
        }

        inline void addMapPoint(int i, MapPointPtr &mapPoint) {
            mapPoints[i] = mapPoint;
        }
    };

}


#endif //SLAM_LEARN_KEYFRAME_H
