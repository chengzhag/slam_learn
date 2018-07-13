//
// Created by pidan1231239 on 18-7-13.
//

#ifndef SLAM_LEARN_MAP_H
#define SLAM_LEARN_MAP_H


#include "MapPoint.h"
#include "Frame.h"

namespace sky {

    class Map {
    public:
        typedef shared_ptr<Map> Ptr;
        list<MapPoint::Ptr> mapPoints;        // all landmarks
        list<Frame::Ptr> frames;         // all key-frames

        Map() {}

        void addFrame(Frame::Ptr frame);

        void addMapPoint(MapPoint::Ptr mapPoint);

        void visInCloudViewer();
    };

}


#endif //SLAM_LEARN_MAP_H
