//
// Created by pidan1231239 on 18-7-12.
//

#include "MapPoint.h"

namespace sky {

    void MapPoint::addObservedFrame(const KeyFramePtr &observedFrame, const cv::Point2d &pixelCoor) {
        if (observedFrame)
            observedFrames[observedFrame] = pixelCoor;
    }

    bool MapPoint::getPixelCoor(const KeyFramePtr &observedFrame, cv::Point2d &pixelCoor) {
        if (hasObservedFrame(observedFrame)) {
            pixelCoor = observedFrames[observedFrame];
            return true;
        } else {
            cerr << "MapPoint: getPixelPos failed! No such observedFrame " << endl;
            return false;
        }
    }
}
