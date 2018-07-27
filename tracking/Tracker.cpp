//
// Created by pidan1231239 on 18-7-14.
//

#include "Tracker.h"

namespace sky {

    void Tracker::step(const KeyFrame::Ptr &frame) {
        this->frame = frame;
        boost::mutex::scoped_lock lock(localMap->mapMutex);
        bool solverPass=solver3D2D.solve(localMap->map, frame);
        lock.unlock();
        if (solverPass) {
            //判断是否插入关键帧
            if (isKeyFrame()) {
#ifdef DEBUG
                cout << "Tracker: Adding keyframe..." << endl;
#endif
                localMap->addFrame(frame);
            }
        }
    }

    bool Tracker::isKeyFrame() {
        auto dis2LastFrame = frame->getDis2(localMap->getLastFrame());
        if (dis2LastFrame < minKeyFrameDis) {
#ifdef DEBUG
            cout << "Tracker: Not a keyFrame. Distance to the last keyFrame " << dis2LastFrame
                 << " is less than minKeyFrameDis " << minKeyFrameDis << endl;
#endif
            return false;
        }
        if (dis2LastFrame > maxKeyFrameDis) {
#ifdef DEBUG
            cout << "Tracker: Not a keyFrame. Distance to the last keyFrame " << dis2LastFrame
                 << " is more than maxKeyFrameDis " << maxKeyFrameDis << endl;
#endif
            return false;
        }

        if (solver3D2D.getInlierNum() < minKeyFrameInlierNum) {
#ifdef DEBUG
            cout << "Tracker: Not a keyFrame. InlierNum " << solver3D2D.getInlierNum()
                 << " is less than minKeyFrameInlierNum " << minKeyFrameInlierNum << endl;
#endif
            return false;
        }
        return true;
    }

}