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
        unordered_map<KeyFrame::Ptr, int> frame2indexs;//观测帧和像素坐标
    public:
        typedef shared_ptr<MapPoint> Ptr;
        Mat descriptor; // Descriptor for matching
        Vector3d pos;       // Position in world
        Vector3d norm;      // Normal of viewing direction
        cv::Vec3b rgb;


        MapPoint();

        MapPoint(const Vector3d &pos, const Mat &descriptor, const cv::Vec3b &rgb) :
                pos(pos),
                descriptor(descriptor),
                rgb(rgb) {}

        MapPoint(const MapPoint &mapPoint) :
                frame2indexs(mapPoint.frame2indexs),
                pos(mapPoint.pos),
                norm(mapPoint.norm),
                rgb(mapPoint.rgb) {
            mapPoint.descriptor.copyTo(descriptor);
        }

        MapPoint &operator=(const MapPoint &mapPoint) {
            frame2indexs = mapPoint.frame2indexs;
            pos = mapPoint.pos;
            norm = mapPoint.norm;
            rgb = mapPoint.rgb;
            mapPoint.descriptor.copyTo(descriptor);
        }


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

        //观测帧
        void
        addFrame(const KeyFrame::Ptr &frame, const int index, bool doRefreshNorm = true);

        void refreshNorm(const KeyFrame::Ptr &observedFrame);

        inline void deleteFrame(const KeyFrame::Ptr &observedFrame) {
            frame2indexs.erase(observedFrame);
        }

        inline void deleteAllFrame() {
            frame2indexs.clear();
        }

        inline bool hasFrame(const KeyFrame::Ptr &observedFrame) const {
            return mapHas(frame2indexs, observedFrame);
        }

        int getIndex(const KeyFrame::Ptr &observedFrame) const;

        bool getPixelCoor(const KeyFrame::Ptr &frame, cv::Point2f &pixelCoor) const;

        inline auto getFrameNum() {
            return frame2indexs.size();
        }

        //循环遍历frame2indexs，不能用于删除
        template<typename L>
        void forEachFrame2index(L func) {
            for (auto &frame2index:frame2indexs) {
                func(frame2index);
            }
        }

        //循环遍历frame2indexs，不能用于删除
        template<typename L>
        void forEachFrames(L func) {
            for (auto &frame2index:frame2indexs) {
                func(frame2index.first);
            }
        }
    };


}

#endif //SLAM_LEARN_MAPPOINT_H
