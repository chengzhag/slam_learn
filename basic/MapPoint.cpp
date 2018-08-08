//
// Created by pidan1231239 on 18-7-12.
//

#include "MapPoint.h"

namespace sky {

    void MapPoint::addFrame(const KeyFrame::Ptr &frame, const int index,
                            bool doRefreshNorm) {
        if (!mapHas(frame2indexs, frame)) {
            frame2indexs[frame] = index;

            if (doRefreshNorm) {
                refreshNorm(frame);
            }
        }
    }

    void MapPoint::refreshNorm(const KeyFrame::Ptr &observedFrame) {
        Vector3d n = observedFrame->getCamCenterEigen() - pos;
        n.normalize();
        if (frame2indexs.size() == 1) {
            norm = n;
        } else {
            norm += n;
        }
    }

    bool MapPoint::getPixelCoor(const KeyFrame::Ptr &frame, cv::Point2f &pixelCoor) const {
        auto it = frame2indexs.find(frame);
        if (it != frame2indexs.end()) {
            pixelCoor = frame->getKeyPointCoor(it->second);
            return true;
        } else {
            cerr << "[" << boost::this_thread::get_id() << "]ERROR: "
                 << "MapPoint: getPixelPos failed! No such observedFrame " << endl;
            return false;
        }
    }
}
