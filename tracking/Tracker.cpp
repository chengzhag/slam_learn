//
// Created by pidan1231239 on 18-7-14.
//

#include "Tracker.h"

namespace sky {

    void Tracker::step(KeyFrame::Ptr frame) {
        if (solver3D2D.solve(localMap->map, frame)) {
            //判断是否插入关键帧
            if (isKeyFrame(frame)) {
#ifdef DEBUG
                cout << "Tracker: Adding keyframe..." << endl;
#endif
                localMap->addFrame(frame);
            }
        }
    }

    bool Tracker::isKeyFrame(KeyFrame::Ptr frame) {
        if (frame->dis2Coor(localMap->map->keyFrames.back()->Tcw.translation()) < minKeyFrameDis) {
#ifdef DEBUG
            cout << "Tracker: Not a keyFrame cause distance to the last keyFrame "
                 << frame->dis2Coor(localMap->map->keyFrames.back()->Tcw.translation())
                 << " is less than minKeyFrameDis " << minKeyFrameDis << endl;
#endif
            return false;
        }
        return true;
    }

}