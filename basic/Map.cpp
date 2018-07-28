//
// Created by pidan1231239 on 18-7-13.
//

#include "Map.h"

namespace sky {

    void Map::addFrame(const KeyFrame::Ptr &frame) {
        if (frame)
            keyFrames.push_back(frame);
    }

    void Map::addMapPoint(const MapPoint::Ptr &mapPoint) {
        if (mapPoint)
            mapPoints.push_back(mapPoint);
    }

    KeyFrame::Ptr Map::getLastFrame() const {
        if (!keyFrames.empty())
            return keyFrames.back();
        else {
            cerr << "Map: Cannot get lastFrame! No Frame in the map" << endl;
            return nullptr;
        }

    }

}