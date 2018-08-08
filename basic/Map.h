//
// Created by pidan1231239 on 18-7-13.
//

#ifndef SLAM_LEARN_MAP_H
#define SLAM_LEARN_MAP_H


#include "MapPoint.h"
#include "KeyFrame.h"

namespace sky {

    class Map {
    public:
        typedef shared_ptr<Map> Ptr;
        list<MapPoint::Ptr> mapPoints;        // all landmarks
        list<KeyFrame::Ptr> keyFrames;         // all key-keyFrames

        Map() {}

        void addFrame(const KeyFrame::Ptr &frame);

        void addMapPoint(const MapPoint::Ptr &mapPoint);

        inline void addObservation(const KeyFrame::Ptr &frame, const MapPoint::Ptr &mapPoint, const int index) {
            mapPoint->addFrame(frame, index);
            frame->addMapPoint(index, mapPoint);
        }

        //删除KeyFrame和MapPoint的观测关系
        //不能用于循环删除某KeyFrame或MapPoint的所有观测关系！
/*        inline void deleteObservation(const KeyFrame::Ptr &frame, const MapPoint::Ptr &mapPoint) {
            frame->deleteMapPoint(mapPoint);
            mapPoint->deleteFrame(frame);
        }*/

        //删除某地图点的所有观测关系
        inline void deleteObservation(const MapPoint::Ptr &mapPoint) {
            mapPoint->forEachFrames([&](auto &frame) {
                frame->deleteMapPoint(mapPoint);
            });
            mapPoint->deleteAllFrame();
        }

        //删除某关键帧的所有观测关系
        inline void deleteObservation(const KeyFrame::Ptr &frame) {
            frame->forEachMapPoint([&](auto &mapPoint) {
                mapPoint->deleteFrame(frame);
            });
            frame->deleteAllMapPoint();
        }

        KeyFrame::Ptr getLastFrame() const;

        bool viewFrameProjInCVV(const KeyFrame::Ptr &frame, string message = "Reprojection") const;
    };

}


#endif //SLAM_LEARN_MAP_H
