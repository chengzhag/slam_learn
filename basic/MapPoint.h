//
// Created by pidan1231239 on 18-7-12.
//

#ifndef SLAM_LEARN_MAPPOINT_H
#define SLAM_LEARN_MAPPOINT_H


#include "common_include.h"
#include <unordered_map>
#include "utility.h"
#include "KeyFrame.h"

namespace sky {

    class MapPoint {
    private:

    public:
        typedef shared_ptr<MapPoint> Ptr;
        unordered_map<KeyFrame::Ptr, cv::Point2d> observedFrames;//观测帧和像素坐标
        Mat descriptor; // Descriptor for matching
        Vector3d pos;       // Position in world
        Vector3d norm;      // Normal of viewing direction
        cv::Vec3b rgb;


        MapPoint();

        MapPoint(const Vector3d &pos, const Mat &descriptor, const cv::Vec3b &rgb) :
                pos(pos),
                descriptor(descriptor),
                rgb(rgb) {}

/*        MapPoint(const MapPoint &mapPoint) :
                observedFrames(mapPoint.observedFrames),
                pos(mapPoint.pos),
                rgb(mapPoint.rgb) {
            mapPoint.descriptor.copyTo(descriptor);
        }

        MapPoint &operator=(const MapPoint &mapPoint) {
            observedFrames = mapPoint.observedFrames;
            pos = mapPoint.pos;
            rgb = mapPoint.rgb;
            mapPoint.descriptor.copyTo(descriptor);
        }

        MapPoint(const MapPoint::Ptr mapPoint) : MapPoint(*mapPoint) {}*/


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

        void addObservedFrame(const KeyFrame::Ptr &observedFrame, const cv::Point2d &pixelCoor);

        inline void deleteObservedFrame(const KeyFrame::Ptr &observedFrame) {
            observedFrames.erase(observedFrame);
        }

        inline bool hasObservedFrame(const KeyFrame::Ptr &observedFrame) const {
            return mapHas(observedFrames, observedFrame);
        }

        bool getPixelCoor(const KeyFrame::Ptr &observedFrame, cv::Point2d &pixelCoor) const;

        template<typename L>
        void forObservedFrames(L func) {
            for (auto &observedFrame:observedFrames) {
                func(observedFrame);
            }
        }
    };


}

#endif //SLAM_LEARN_MAPPOINT_H
