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

        KeyFrame::Ptr getLastFrame() const;

        bool viewFrameProjInCVV(const KeyFrame::Ptr &frame, string message = "Reprojection") const;
    };

}


#endif //SLAM_LEARN_MAP_H
