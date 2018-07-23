//
// Created by pidan1231239 on 18-7-13.
//

#ifndef SLAM_LEARN_KEYFRAME_H
#define SLAM_LEARN_KEYFRAME_H

#include "common_include.h"
#include "MapPoint.h"
#include "Camera.h"
#include <opencv2/opencv.hpp>
#include <opencv2/cvv.hpp>
#include <opencv2/core/eigen.hpp>
#include <unordered_map>

namespace sky {

    using namespace cv;

    class KeyFrame {
    public:
        typedef shared_ptr<KeyFrame> Ptr;
        SE3 Tcw;      // transform from world to camera
        Camera::Ptr camera;     // Pinhole RGBD Camera model
        Mat image;
        vector<cv::KeyPoint> keyPoints;
        Mat descriptors;
        unordered_map<int, MapPoint::Ptr> mapPoints;//在descriptors或keyPoints中的序号和对应的地图点

        KeyFrame(const Camera::Ptr camera, const Mat &image, cv::Ptr<cv::Feature2D> feature2D) :
                camera(camera), image(image) {
#ifdef DEBUG
            cout << "KeyFrame: detectAndCompute features... " << endl;
#endif
            feature2D->detectAndCompute(image, noArray(), keyPoints, descriptors);
#ifdef DEBUG
            cout << "\tfound " << keyPoints.size() << " keypoints" << endl;
#endif
        }

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
        void setTcw(Matx<T, 2, 3> angleAxisAndTranslation) {
            Tcw.so3() = SO3(angleAxisAndTranslation(0, 0),
                            angleAxisAndTranslation(0, 1),
                            angleAxisAndTranslation(0, 2));
            Tcw.translation() = Vector3d(angleAxisAndTranslation(1, 0),
                                         angleAxisAndTranslation(1, 1),
                                         angleAxisAndTranslation(1, 2));
        }

        // check if a point is in this frame
        bool isInFrame(const Vector3d &pt_world);

        //计算帧到某坐标的距离
        double dis2Coor(Sophus::Vector3d coor);


        //在descriptors或keyPoints中的序号
        bool hasMapPoint(int i) {
            return mapPoints.find(i) != mapPoints.end();
        }

        MapPoint::Ptr getMapPoint(int i) {
            return mapPoints[i];
        }

        Point2f getKeyPointCoor(int i) {
            return keyPoints[i].pt;
        }

        void addMapPoint(int i, MapPoint::Ptr &mapPoint) {
            mapPoints[i] = mapPoint;
        }
    };

}


#endif //SLAM_LEARN_KEYFRAME_H
