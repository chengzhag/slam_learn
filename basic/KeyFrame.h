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
#include "boost/bimap.hpp"

namespace sky {

    class Map;

    typedef shared_ptr<Map> MapPtr;

    class MapPoint;

    typedef shared_ptr<MapPoint> MapPointPtr;

    class KeyFrame {
    private:
        boost::bimap<int, MapPointPtr> index2mapPoints;//在descriptors或keyPoints中的序号和对应的地图点
        typedef boost::bimap<int, MapPointPtr>::value_type Index2mapPoint;

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

        //观测到的地图点
        //在descriptors或keyPoints中的序号
        inline void addMapPoint(int i, const MapPointPtr &mapPoint) {
            index2mapPoints.insert(Index2mapPoint(i, mapPoint));
        }

        inline void deleteMapPoint(int i) {
            index2mapPoints.left.erase(i);
        }

        inline void deleteMapPoint(const MapPointPtr &mapPoint) {
            index2mapPoints.right.erase(mapPoint);
        }

        inline void deleteAllMapPoint(){
            index2mapPoints.clear();
        }

        inline auto getMapPointsNum() {
            return index2mapPoints.size();
        }

        inline bool hasMapPoint(int i) const {
            return index2mapPoints.left.find(i) != index2mapPoints.left.end();
        }

        inline bool hasMapPoint(const MapPointPtr &mapPoint) {
            return index2mapPoints.right.find(mapPoint) != index2mapPoints.right.end();
        }

        MapPointPtr getMapPoint(int i) const;

        bool setMapPoint(int i, MapPointPtr &mapPoint);

        inline cv::Point2f getKeyPointCoor(int i) const {
            return keyPoints[i].pt;
        }

        inline Mat getKeyPointDesciptor(int i) const {
            return descriptors.row(i);
        }

        //循环遍历frame2indexs，不能用于删除
        template<typename L>
        void forEachMapPoint(L func) {
            for (auto &index2mapPoint:index2mapPoints.left) {
                func(index2mapPoint.second);
            }
        }


    };

}


#endif //SLAM_LEARN_KEYFRAME_H
