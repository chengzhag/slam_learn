//
// Created by pidan1231239 on 18-7-12.
//

#include "MapPoint.h"

namespace sky {

    void MapPoint::addObervedFrame(const KeyFramePtr &observedFrame, const cv::Point2d &pixelCoor) {
        if (observedFrame)
            observedFrames[observedFrame] = pixelCoor;
    }

}
