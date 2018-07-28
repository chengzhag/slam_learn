//
// Created by pidan1231239 on 18-7-12.
//

#include "MapPoint.h"

namespace sky {

    void MapPoint::addObservedFrame(const KeyFramePtr &observedFrame, const cv::Point2d &pixelCoor) {
        if (observedFrame)
            observedFrames[observedFrame] = pixelCoor;
    }

    bool MapPoint::getPixelCoor(const KeyFramePtr &observedFrame, cv::Point2d &pixelCoor) const {
        auto it = observedFrames.find(observedFrame);
        if (it != observedFrames.end()) {
            pixelCoor = it->second;
            return true;
        } else {
            cerr << "MapPoint: getPixelPos failed! No such observedFrame " << endl;
            return false;
        }
    }
}
