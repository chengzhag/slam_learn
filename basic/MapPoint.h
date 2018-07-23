//
// Created by pidan1231239 on 18-7-12.
//

#ifndef SLAM_LEARN_MAPPOINT_H
#define SLAM_LEARN_MAPPOINT_H


#include "common_include.h"
#include <unordered_map>
#include <utility>

namespace sky {

    class KeyFrame;
    typedef shared_ptr<KeyFrame> KeyFramePtr;

    using namespace cv;

    class MapPoint {

    public:

        typedef shared_ptr<MapPoint> Ptr;
        Mat descriptor; // Descriptor for matching
        unordered_map<KeyFramePtr, cv::Point2d> observedFrames;//观测帧和像素坐标
        Vector3d pos;       // Position in world
        Vec3b rgb;


        MapPoint();

        MapPoint(const Vector3d &pos, const Mat &descriptor, const Vec3b &rgb) :
                pos(pos),
                descriptor(descriptor),
                rgb(rgb) {}

        template<typename T>
        cv::Point3_<T> getPosPoint3_CV() const {
            return cv::Point3_<T>(pos(0, 0), pos(1, 0), pos(2, 0));
        }

        template<typename T>
        cv::Matx<T, 1, 3> getPosMatx13() const {
            return cv::Matx<T, 1, 3>(pos(0, 0), pos(1, 0), pos(2, 0));
        };

        template<typename T>
        void setPos(cv::Matx<T, 1, 3> posMatx13) {
            pos(0) = posMatx13(0);
            pos(1) = posMatx13(1);
            pos(2) = posMatx13(2);
        }

        void addObervedFrame(const KeyFramePtr &observedFrame, const cv::Point2d &pixelCoor);
    };


}

#endif //SLAM_LEARN_MAPPOINT_H
