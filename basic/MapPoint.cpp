//
// Created by pidan1231239 on 18-7-12.
//

#include "MapPoint.h"

namespace sky {

    void MapPoint::addObservedFrame(const KeyFrame::Ptr &observedFrame, const cv::Point2d &pixelCoor,
                                    bool doRefreshNorm) {
        if (!mapHas(observedFrames, observedFrame)) {
            observedFrames[observedFrame] = pixelCoor;

            if (doRefreshNorm) {
                refreshNorm(observedFrame);
            }
        }
    }

    void MapPoint::refreshNorm(const KeyFrame::Ptr &observedFrame) {
        Vector3d n = observedFrame->getCamCenterEigen() - pos;
        n.normalize();
        if (observedFrames.size() == 1) {
            norm = n;
        } else {
            norm += n;
        }
    }

    bool MapPoint::getPixelCoor(const KeyFrame::Ptr &observedFrame, cv::Point2d &pixelCoor) const {
        auto it = observedFrames.find(observedFrame);
        if (it != observedFrames.end()) {
            pixelCoor = it->second;
            return true;
        } else {
            cerr << "[" << boost::this_thread::get_id() << "]ERROR: "
                 << "MapPoint: getPixelPos failed! No such observedFrame " << endl;
            return false;
        }
    }
}
