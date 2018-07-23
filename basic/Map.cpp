//
// Created by pidan1231239 on 18-7-13.
//

#include "Map.h"

namespace sky {

    void Map::addFrame(KeyFrame::Ptr frame) {
        if (frame)
            keyFrames.push_back(frame);
    }

    void Map::addMapPoint(MapPoint::Ptr mapPoint) {
        if (mapPoint)
            mapPoints.push_back(mapPoint);
    }


}