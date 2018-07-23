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

        void addFrame(KeyFrame::Ptr frame);

        void addMapPoint(MapPoint::Ptr mapPoint);

    };

}


#endif //SLAM_LEARN_MAP_H
